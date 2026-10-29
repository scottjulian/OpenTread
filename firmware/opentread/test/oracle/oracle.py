#!/usr/bin/env python3
"""OpenTread independent reference model (oracle) -> committed goldens.

Written from the behaviour definitions only (integer units and rounding,
replay parser rules, PulseFilter / Metrics / the test 1.11 bias definition,
the Workout state machine, the plain Format functions) and from the original
pulse logger's `acceptClosure`, which is the authoritative pulse rule. It is
deliberately NOT derived from the C++ in src/core/. Python standard library
only (run with the pinned interpreter and `-I`).

Usage:
    oracle.py --out DIR [--root REPO_ROOT]

Writes every golden file into DIR. Output is deterministic and
byte-identical across runs. The format contract is documented in
test/golden/README.md.

Number representation:
  * time: integer microseconds (Python int, unbounded; values fit int64);
  * belt travel: integer pulse counts, distance = pulses x 597400 um;
  * speed: integer mm/s, mmps = (N*597400*1000 + dt/2) // dt  (dt in us).
  * the 1.11 integrals use exact Fractions; the one transcendental term of
    the continuous integral (the M/(t-t_k) tail integrates to M*ln(...)) is
    evaluated with `decimal` at 60 significant digits, so it is exact far
    beyond the 6 printed decimals and independent of the platform libm.
"""

from __future__ import annotations

import argparse
import bisect
import decimal
import os
import re
import sys
from fractions import Fraction

# --------------------------------------------------------------------------
# Constants. Integer units: us, um, mm/s.
# --------------------------------------------------------------------------
M_PER_PULSE_UM = 597400          # 0.5974 m per accepted pulse
RELEASE_US = 5000                # re-arm: line seen open this long
LOCKOUT_US = 20000               # lockout after an accepted pulse
TS_RING = 64                     # pulse timestamp ring (includes the newest)
SPAN_US = 3_000_000              # speed estimator span
STOP_US = 2_500_000              # speed -> 0 / bout boundary

# Speed numerator for one interval: M (um) * 1000 -> mm/s when divided by us.
M_MMPS_NUM = M_PER_PULSE_UM * 1000   # 597_400_000

# Golden generation parameters (contract, see golden/README.md).
STEP_US = 50_000                 # 20 Hz metrics samples (test 1.10)
PRE_US = 1_000_000               # metrics window starts 1 s before 1st pulse
POST_US = 5_000_000              # ... and ends 5 s after the last pulse
BIAS_PRE_US = 2_000_000          # 1.11: t_first - 2 s
BIAS_POST_US = 5_000_000         # 1.11: until t_last + 5 s
BIAS_STEP_US = 1_000_000         # 1.11: 1 Hz
BIAS_PHASE_US = 100_000          # 1.11: phase p * 0.1 s, p = 0..9

LOGS = {
    "S2_walk": "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv",
    "S1b_10rev": "firmware/opentread/test/data/2026-10-18_1416_S1b_10rev.csv",
    "S1_handturn": "firmware/opentread/test/data/2026-10-18_1323_S1_handturn.csv",
    "bench_jumper": "firmware/opentread/test/data/2026-10-18_1315_bench_jumper.csv",
}
METRIC_LOGS = ("S2_walk", "S1b_10rev", "S1_handturn")
CONSOLE_LOG = "S2_walk"
CONSOLE_MARKER_N = 1             # test 1.15: "S2 after marker M1"

# --------------------------------------------------------------------------
# Log parsing (replay parser rules)
# --------------------------------------------------------------------------
# Lines of a known type must match exactly; anything else of a known type is
# MALFORMED and makes the oracle fail (never silently accepted). Lines that
# do not start with a known type letter + comma (the '#' capture header,
# boot-ROM garbage, blank lines) are skipped. re.ASCII: \d is [0-9] only
# (a str pattern's \d would also match non-ASCII digits, which the C++ replay
# parser rejects). Lines end only at "\n" (see open() below), as in the C++.
_RE = {
    "E": re.compile(r"^E,(\d+),([01])$", re.ASCII),
    "P": re.compile(r"^P,(\d+),(\d+),(\d+)$", re.ASCII),
    "M": re.compile(r"^M,(\d+),(\d+)$", re.ASCII),
    "F": re.compile(r"^F,(\d+),((?:[0-9a-f]{2})+)$", re.ASCII),
    "H": re.compile(r"^H,(\d+)(?:,\d+){7}$", re.ASCII),
    "S": re.compile(r"^S,(\d+),(.*)$", re.ASCII),      # split at most 2 times
}
_BANNER_LEVEL = re.compile(r"(?:^| )line_level=([01])(?: |$)")


class LogError(Exception):
    pass


class Log:
    """Parsed pulse-logger or OpenTread log. All event lists are sorted by t_us (stable)."""

    def __init__(self, path: str):
        self.path = path
        self.edges: list[tuple[int, int]] = []    # (t_us, level)
        self.pulses: list[tuple[int, int]] = []   # logged P: (t_us, n)
        self.markers: list[tuple[int, int]] = []  # (t_us, n)
        self.ftms: list[tuple[int, bytes]] = []   # (t_us, payload)
        self.banners: list[tuple[int, int]] = []  # (t_us, line_level)
        malformed: list[str] = []
        with open(path, "r", encoding="utf-8", errors="replace", newline="\n") as fh:
            for lineno, raw in enumerate(fh, 1):
                line = raw.rstrip("\r\n")
                if len(line) < 2 or line[1] != "," or line[0] not in _RE:
                    continue  # header, boot-ROM garbage, blank
                m = _RE[line[0]].match(line)
                if not m:
                    malformed.append(f"{path}:{lineno}: {line!r}")
                    continue
                t = int(m.group(1))
                kind = line[0]
                if kind == "E":
                    self.edges.append((t, int(m.group(2))))
                elif kind == "P":
                    self.pulses.append((t, int(m.group(2))))
                elif kind == "M":
                    self.markers.append((t, int(m.group(2))))
                elif kind == "F":
                    self.ftms.append((t, bytes.fromhex(m.group(2))))
                elif kind == "S":
                    text = m.group(2)
                    if "pulse logger" in text or "OpenTread fw=" in text:
                        lv = _BANNER_LEVEL.search(text)
                        if not lv:
                            malformed.append(f"{path}:{lineno}: banner without line_level: {line!r}")
                            continue
                        self.banners.append((t, int(lv.group(1))))
        if malformed:
            raise LogError("malformed lines:\n  " + "\n  ".join(malformed))
        if len(self.banners) != 1:
            raise LogError(f"{path}: expected exactly 1 boot banner, found {len(self.banners)}")
        # Sort by t_us before replaying (stable: keeps file order on ties).
        for lst in (self.edges, self.pulses, self.markers, self.ftms):
            lst.sort(key=lambda x: x[0])
        self.boot_t, self.boot_level = self.banners[0]


# --------------------------------------------------------------------------
# PulseFilter: literal port of the pulse logger's acceptClosure (authoritative)
# --------------------------------------------------------------------------
def filter_edges(boot_t: int, boot_level: int, edges) -> list[int]:
    """Return accepted pulse timestamps.

    Logger state at boot: {prevLevel=initLevel, prevT=t0, lastAcceptT=0,
    armed=false, accepted=false}. For every edge (t, level):
        if prevLevel == 1 and t - prevT >= RELEASE: armed = true
        prevLevel, prevT = level, t
        if level != 0: reject
        was = armed; armed = false            # every closure disarms
        if not was or (accepted and t - lastAcceptT < LOCKOUT): reject
        accepted = true; lastAcceptT = t; accept
    """
    prev_level, prev_t = boot_level, boot_t
    armed = False
    accepted = False
    last_accept = 0
    out = []
    for t, level in edges:
        if prev_level == 1 and t - prev_t >= RELEASE_US:
            armed = True
        prev_level, prev_t = level, t
        if level != 0:
            continue
        was_armed = armed
        armed = False
        if not was_armed or (accepted and t - last_accept < LOCKOUT_US):
            continue
        accepted = True
        last_accept = t
        out.append(t)
    return out


# --------------------------------------------------------------------------
# Metrics (span estimator + decay cap, integer units)
# --------------------------------------------------------------------------
def mmps_over(n_intervals: int, dt_us: int) -> int:
    """mmps = (N*597400*1000 + dt/2) / dt, integer (dt > 0)."""
    if dt_us <= 0:
        raise ValueError("non-positive interval")
    return (n_intervals * M_MMPS_NUM + dt_us // 2) // dt_us


def decay_cap(d_us: int) -> int:
    """Decay cap M*10^9/(now - t_last) in mm/s, floored (d > 0)."""
    if d_us <= 0:
        raise ValueError("non-positive interval")
    return M_MMPS_NUM // d_us


def raw_speed(ts: list[int], k: int, stop_us: int = STOP_US, span_us: int = SPAN_US) -> int:
    """Span estimator rawSpeed(k) for pulse index k (0-based) of `ts`.

    * first pulse of a bout (no predecessor, or t_k - t_{k-1} > STOP) -> 0;
    * else the smallest N >= 1 with t_k - t_{k-N} >= SPAN, stopping early
      (keeping the current N) if pulse k-N starts a bout
      (t_{k-N} - t_{k-N-1} > STOP), if k-N has no predecessor (N == k), or
      if the ring runs out (the ring holds TS_RING timestamps including t_k,
      so N <= TS_RING - 1);
    * v = N*M / (t_k - t_{k-N}) in integer mm/s.
    """
    if k == 0 or ts[k] - ts[k - 1] > stop_us:
        return 0
    n = 1
    while True:
        if ts[k] - ts[k - n] >= span_us:
            break
        if n == k:                                   # no older pulse at all
            break
        if ts[k - n] - ts[k - n - 1] > stop_us:      # never span a stop gap
            break
        if n == TS_RING - 1:                         # ring exhausted
            break
        n += 1
    return mmps_over(n, ts[k] - ts[k - n])


class Metrics:
    def __init__(self, ts: list[int], stop_us: int = STOP_US, span_us: int = SPAN_US):
        self.ts = ts
        self.stop_us = stop_us
        self.v_at = [raw_speed(ts, k, stop_us, span_us) for k in range(len(ts))]

    def last_index(self, now: int) -> int:
        """Index of the last pulse with t <= now (a pulse AT now counts), or -1."""
        return bisect.bisect_right(self.ts, now) - 1

    def odometer(self, now: int) -> int:
        return self.last_index(now) + 1

    def speed(self, now: int) -> int:
        """speedMmps(now): 0 if no pulse or now - t_last > STOP; else
        min(speedAtLastPulse, decay cap M/(now - t_last)) for now > t_last.
        The cap is FLOORED: it is defined as the plain quotient
        M*10^9/(now - t_last) with no rounding term, the speed is bounded by
        v <= M/(now - t_last) (rounding half up can exceed it), and test 1.9
        says the speed "equals M/(now - t_last)". The expected 1.11 phase
        figures are reproduced only with the floored cap (INTEGRATION_NOTES.md)."""
        k = self.last_index(now)
        if k < 0:
            return 0
        d = now - self.ts[k]
        if d > self.stop_us:
            return 0
        v = self.v_at[k]
        if d > 0:
            v = min(v, decay_cap(d))
        return v


# --------------------------------------------------------------------------
# Test 1.11: watch-integration bias
# --------------------------------------------------------------------------
_DCTX = decimal.Context(prec=60, rounding=decimal.ROUND_HALF_EVEN)


def continuous_integral_um(met: Metrics) -> decimal.Decimal:
    """Exact piecewise integral of the speed, in um (1.11 definition).

    After pulse k: constant v_k until tau = M/v_k after t_k, then M/(t-t_k),
    ending at the next pulse or at t_k + STOP. With v in mm/s and time in
    us, distance in um = v*T/1000 and tau_us = M_um*1000/v_k. For T > tau the
    integral is M + M*ln(T/tau) um.
    """
    ts = met.ts
    rational = Fraction(0)
    logs = decimal.Decimal(0)
    for k, v in enumerate(met.v_at):
        if v == 0:
            continue
        end = ts[k] + STOP_US
        if k + 1 < len(ts) and ts[k + 1] < end:
            end = ts[k + 1]
        T = end - ts[k]
        tau = Fraction(M_MMPS_NUM, v)                # us
        if T <= tau:
            rational += Fraction(v * T, 1000)
        else:
            rational += M_PER_PULSE_UM
            ratio = _DCTX.divide(decimal.Decimal(T * v), decimal.Decimal(M_MMPS_NUM))
            logs = _DCTX.add(logs, _DCTX.multiply(decimal.Decimal(M_PER_PULSE_UM), _DCTX.ln(ratio)))
    r = _DCTX.divide(decimal.Decimal(rational.numerator), decimal.Decimal(rational.denominator))
    return _DCTX.add(r, logs)


def sampled_integral_mm(met: Metrics, phase: int) -> int:
    """1 Hz sampling, phase p: sum of v(t) * 1 s over
    t = t_first - 2 s + p*0.1 s + i*1 s, i = 0, 1, ... while t <= t_last + 5 s.
    v in mm/s times 1 s = mm (exact integer)."""
    t = met.ts[0] - BIAS_PRE_US + phase * BIAS_PHASE_US
    t_end = met.ts[-1] + BIAS_POST_US
    total = 0
    while t <= t_end:
        total += met.speed(t)
        t += BIAS_STEP_US
    return total


# --------------------------------------------------------------------------
# Test 1.15: console / ours speed ratio
# --------------------------------------------------------------------------
def ftms_speed_centikmh(payload: bytes):
    """Instantaneous Speed of an FTMS Treadmill Data packet (0.01 km/h), or
    None when the packet doesn't carry it (flags bit 0 'More Data' set) or is
    too short."""
    if len(payload) < 4:
        return None
    flags = payload[0] | (payload[1] << 8)
    if flags & 0x0001:
        return None
    return payload[2] | (payload[3] << 8)


def console_ratios(log: Log, met: Metrics) -> list[Fraction]:
    """Samples: every F line with t_us > t(marker M1) whose console speed c
    (0.01 km/h) is > 0 and whose ours = speedMmps(t_F) is > 0.
    ratio = (c * 1000 / 360) / ours = 25*c / (9*ours), exact."""
    mk = [t for t, n in log.markers if n == CONSOLE_MARKER_N]
    if len(mk) != 1:
        raise LogError(f"{log.path}: marker M{CONSOLE_MARKER_N} not found exactly once")
    t_m = mk[0]
    out = []
    for t, payload in log.ftms:
        if t <= t_m:
            continue
        c = ftms_speed_centikmh(payload)
        if c is None or c == 0:
            continue
        ours = met.speed(t)
        if ours == 0:
            continue
        out.append(Fraction(25 * c, 9 * ours))
    return out


def median(values: list[Fraction]) -> Fraction:
    s = sorted(values)
    n = len(s)
    if n == 0:
        raise ValueError("empty median")
    if n % 2:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2


# --------------------------------------------------------------------------
# Workout state machine (transition table, displayed values, hold-to-stop).
# Written from the behaviour definition only, not from the C++.
# --------------------------------------------------------------------------
WARMUP_US = 5_000_000            # bout confirms on the first pulse >= 5 s after its start
AUTO_END_US = 900_000_000        # 15 min since the last pulse
SUMMARY_US = 120_000_000         # summary shown 2 min
MOVING_FREEZE_K = 2              # moving-clock tail cap = min(STOP, K*M/v)

IDLE, WARMUP, RUNNING, PAUSED, SUMMARY = "IDLE", "WARMUP", "RUNNING", "PAUSED", "SUMMARY"
STATES = (IDLE, WARMUP, RUNNING, PAUSED, SUMMARY)
W_REASONS = ("bout_start", "warmup_ok", "warmup_fail", "pause", "resume", "hold_stop",
             "hold_cancel_warmup", "autoend", "summary_timeout", "rearm")
I_EVENTS = ("hold_begin", "hold_abort", "hold_complete", "units_toggle", "touch_down", "touch_up")
HOLD_OK, HOLD_VOID, HOLD_IGNORED = "ok", "void", "ignored"


class WorkoutParams:
    def __init__(self, stop_us=STOP_US, warmup_us=WARMUP_US, auto_end_us=AUTO_END_US,
                 summary_us=SUMMARY_US, span_us=SPAN_US):
        self.stop_us = stop_us
        self.warmup_us = warmup_us
        self.auto_end_us = auto_end_us
        self.summary_us = summary_us
        self.span_us = span_us

    def header(self) -> str:
        return (f"STOP_US={self.stop_us} WARMUP_US={self.warmup_us} AUTO_END_US={self.auto_end_us} "
                f"SUMMARY_US={self.summary_us} SPAN_US={self.span_us}")


def tail_us(v_mmps: int, stop_us: int) -> int:
    """Moving-clock tail after the last pulse: v > 0 ? min(STOP, 2*M/v) : 0.
    2*M/v in us = 2 * 597400 um * 1000 / v (mm/s), FLOORED (integer division)."""
    if v_mmps <= 0:
        return 0
    return min(stop_us, (MOVING_FREEZE_K * M_MMPS_NUM) // v_mmps)


def avg_mmps(wk_pulses: int, moving_us: int) -> int:
    """Summary average speed = distance / moving time, rounded half up."""
    if moving_us <= 0:
        return 0
    return (wk_pulses * M_MMPS_NUM + moving_us // 2) // moving_us


class Workout:
    """Workout transition table. Time-based transitions are stamped at their
    threshold instant and applied in the order a 1 us tick would apply them
    (see advance_to), so the result never depends on tick spacing.

    Outputs: self.out = list of ("w", t, from, to, reason) and
    ("z", t, reason, wk_pulses, moving_us, total_us, top_mmps, avg_mmps, id)
    in emission order (which is non-decreasing in t; asserted)."""

    def __init__(self, p: WorkoutParams):
        self.p = p
        self.state = IDLE
        self.last = None          # lastPulseUs: every pulse, every state
        self.bout = False         # a bout is being tracked in SUMMARY (IDLE/WARMUP: implied by state)
        self.bout_start = None
        self.rearm = False        # rearmBlocked
        self.hold_t = None        # instant of the hold-complete that set rearmBlocked
        # workout fields
        self.workout_id = 0       # last id issued; ids start at 1 per replay
        self.t_conf = None
        self.wk = 0
        self.committed = 0
        self.v_last = 0
        self.top = 0
        # summary
        self.summary = None       # dict, or None
        self.summary_start = None
        # hold
        self.hold = None          # hold-begin snapshot, or None
        # input time clamp: max timestamp of every
        # input and tick processed; inputs are processed at max(t, latest)
        self.latest = None
        # bookkeeping
        self.out: list[tuple] = []
        self.workouts = 0
        self.warmup_fails = 0
        self.pauses = 0
        self.confirm_info: list[tuple[int, int]] = []   # (t, pulse index) per confirm
        self.hold_results: list[tuple[int, str]] = []
        self._pulse_idx = -1

    # ---- outputs -----------------------------------------------------------
    def _w(self, t, frm, to, reason):
        assert reason in W_REASONS
        if self.out:
            assert t >= self.out[-1][1], "W/Z emitted out of time order"
        self.out.append(("w", t, frm, to, reason))

    def _z(self, t):
        s = self.summary
        if self.out:
            assert t >= self.out[-1][1]
        self.out.append(("z", t, s["reason"], s["wk"], s["moving"], s["total"], s["top"],
                         s["avg"], s["id"]))

    # ---- derived values ----------------------------------------------------
    def active_id(self) -> int:
        """Workout id of the open workout (RUNNING/PAUSED), else 0."""
        return self.workout_id if self.state in (RUNNING, PAUSED) else 0

    def moving_live(self, now) -> int:
        if self.state == RUNNING:
            return self.committed + min(now - self.last, tail_us(self.v_last, self.p.stop_us))
        if self.state == PAUSED:
            return self.committed
        return self.summary["moving"] if self.state == SUMMARY and self.summary else 0

    def total_live(self, now) -> int:
        if self.state in (RUNNING, PAUSED):
            return now - self.t_conf
        return self.summary["total"] if self.state == SUMMARY and self.summary else 0

    def snapshot(self, now) -> dict:
        """snapshot(now) (call tick(now) first), evaluated at
        max(now, latest) like every input (input time clamp). Workout fields
        are live in RUNNING/PAUSED, except while a hold begun in this same
        workout is active: then wk/moving/total/top are the hold-begin
        snapshot (so they never step backwards at the hold-stop). SUMMARY:
        the summary record. IDLE/WARMUP: 0 (timers show 0)."""
        if self.latest is not None and now < self.latest:
            now = self.latest
        s = self.state
        if s in (RUNNING, PAUSED):
            h = self.hold
            if h is not None and h["id"] == self.workout_id:
                wk, mv, tot, top = h["wk"], h["moving"], h["total"], h["top"]
            else:
                wk, mv, tot, top = self.wk, self.moving_live(now), self.total_live(now), self.top
        elif s == SUMMARY and self.summary:
            sm = self.summary
            wk, mv, tot, top = sm["wk"], sm["moving"], sm["total"], sm["top"]
        else:
            wk = mv = tot = top = 0
        return {"state": s, "workout_id": self.workout_id, "wk": wk, "moving": mv, "total": tot,
                "top": top, "hold_active": self.hold is not None, "rearm": self.rearm}

    # ---- time ----------------------------------------------------------------
    def _next_timed(self):
        """Pending time transition as (due, stamp, name), or None.
        due = first instant at which the guard is true (strict '>' guards are
        due 1 us after their stamp; the summary timeout's '>=' guard is due at
        its stamp). Ordered by (due, stamp)."""
        p = self.p
        c = []
        if self.rearm:
            if self.last is None:
                c.append((self.hold_t, self.hold_t, "rearm"))
            else:
                c.append((max(self.last + p.stop_us + 1, self.hold_t),
                          max(self.last + p.stop_us, self.hold_t), "rearm"))
        s = self.state
        if s == WARMUP or (s == SUMMARY and self.bout):
            c.append((self.last + p.stop_us + 1, self.last + p.stop_us, "warmup_fail"))
        if s == RUNNING:
            c.append((self.last + p.stop_us + 1, self.last + p.stop_us, "pause"))
        if s == PAUSED:
            c.append((self.last + p.auto_end_us + 1, self.last + p.auto_end_us, "autoend"))
        if s == SUMMARY:
            t = self.summary_start + p.summary_us
            c.append((t, t, "summary_timeout"))
        return min(c) if c else None

    def _eff(self, t: int) -> int:
        """Input time clamp: t_eff = max(t, latestUs),
        latestUs = the max timestamp of every input and tick processed."""
        if self.latest is not None and t < self.latest:
            return self.latest
        self.latest = t
        return t

    def tick(self, now: int):
        """A device tick at `now` (clamped like every input)."""
        self.advance_to(self._eff(now))

    def advance_to(self, now):
        """Apply every time transition due at or before `now` (None = run
        until nothing is pending: 'tick past the end of the input')."""
        while True:
            nx = self._next_timed()
            if nx is None or (now is not None and nx[0] > now):
                break
            _, stamp, name = nx
            getattr(self, "_t_" + name)(stamp)

    def _t_rearm(self, t):
        self.rearm = False
        self._w(t, self.state, self.state, "rearm")

    def _t_warmup_fail(self, t):
        frm = self.state
        if frm == WARMUP:
            self.state = IDLE
        self.bout = False
        self.bout_start = None
        self.warmup_fails += 1
        self._w(t, frm, self.state, "warmup_fail")

    def _t_pause(self, t):
        self.committed += tail_us(self.v_last, self.p.stop_us)
        self.state = PAUSED
        self.pauses += 1
        self._w(t, RUNNING, PAUSED, "pause")

    def _t_autoend(self, t):
        self._finish(t, "autoend", self.wk, self.committed, t - self.t_conf, self.top)
        self._w(t, PAUSED, SUMMARY, "autoend")
        self._z(t)

    def _t_summary_timeout(self, t):
        to = WARMUP if self.bout else IDLE
        self.bout = False
        self.summary = None
        self.summary_start = None
        self.state = to
        self._w(t, SUMMARY, to, "summary_timeout")

    def _finish(self, t, reason, wk, moving, total, top):
        self.summary = {"reason": reason, "wk": wk, "moving": moving, "total": total, "top": top,
                        "avg": avg_mmps(wk, moving), "id": self.workout_id}
        self.summary_start = t
        self.state = SUMMARY
        self.bout = False
        self.bout_start = None

    # ---- inputs --------------------------------------------------------------
    def on_pulse(self, t: int, v: int):
        self._pulse_idx += 1
        t = self._eff(t)
        self.advance_to(t)
        s = self.state
        if s == IDLE:
            if not self.rearm:
                self.bout_start = t
                self.state = WARMUP
                self._w(t, IDLE, WARMUP, "bout_start")
        elif s == WARMUP or (s == SUMMARY and self.bout):
            # advance_to(t) guarantees t - last <= STOP here
            if t - self.bout_start >= self.p.warmup_us:
                self._confirm(t, v, s)
        elif s == SUMMARY:
            if not self.rearm:
                self.bout = True
                self.bout_start = t
                self._w(t, SUMMARY, SUMMARY, "bout_start")
        elif s == RUNNING:
            self.committed += t - self.last
            self.wk += 1
            self.top = max(self.top, v)
            self.v_last = v
        elif s == PAUSED:
            self.wk += 1
            self.v_last = v
            self.state = RUNNING
            self._w(t, PAUSED, RUNNING, "resume")
        self.last = t

    def _confirm(self, t, v, frm):
        self.t_conf = t
        self.wk = 0
        self.committed = 0
        self.v_last = v
        self.top = 0
        self.workout_id += 1
        self.workouts += 1
        self.bout = False
        self.bout_start = None
        self.summary = None          # a confirmed new bout replaces the summary
        self.summary_start = None
        self.state = RUNNING
        self.confirm_info.append((t, self._pulse_idx))
        self._w(t, frm, RUNNING, "warmup_ok")

    def hold_began(self, t: int) -> int:
        """Returns t_eff, the instant the snapshot was taken (the I line's t)."""
        t = self._eff(t)
        self.advance_to(t)
        op = self.state in (RUNNING, PAUSED)
        self.hold = {"id": self.active_id(), "state": self.state,
                     "wk": self.wk if op else 0,
                     "moving": self.moving_live(t) if op else 0,
                     "total": self.total_live(t) if op else 0,
                     "top": self.top if op else 0}
        return t

    def hold_aborted(self, t: int) -> int:
        t = self._eff(t)
        self.advance_to(t)
        self.hold = None
        return t

    def hold_completed(self, now: int) -> str:
        """Order of checks: no hold ->
        ignored; open workout id changed -> void; the hold BEGAN or completes in
        IDLE/SUMMARY (no stop button was shown) -> ignored; WARMUP -> cancel;
        RUNNING/PAUSED -> stop. Processed at t_eff = max(now, latest), which is
        also the t recorded in hold_results (the device logs it in the I line)."""
        now = self._eff(now)
        self.advance_to(now)
        snap, self.hold = self.hold, None
        if snap is None:
            res = HOLD_IGNORED                 # no hold in progress
        elif self.active_id() != snap["id"]:
            res = HOLD_VOID                    # confirm or auto-end during the hold
        elif snap["state"] in (IDLE, SUMMARY) or self.state in (IDLE, SUMMARY):
            res = HOLD_IGNORED                 # began (or completes) with no stop button shown
        elif self.state == WARMUP:             # cancel, nothing recorded
            self.state = IDLE
            self.bout_start = None
            self.rearm = True
            self.hold_t = now
            self._w(now, WARMUP, IDLE, "hold_cancel_warmup")
            res = HOLD_OK
        else:                                  # RUNNING / PAUSED: stop with the hold-begin snapshot
            frm = self.state
            self._finish(now, "hold_stop", snap["wk"], snap["moving"], snap["total"], snap["top"])
            self.rearm = True
            self.hold_t = now
            self._w(now, frm, SUMMARY, "hold_stop")
            self._z(now)
            res = HOLD_OK
        self.hold_results.append((now, res))
        # the hold-complete instant may itself be past the re-arm threshold
        self.advance_to(now)
        return res


def replay_workout(inputs, p: WorkoutParams, end=None, probes=None) -> Workout:
    """inputs: list of (t, kind, arg) ALREADY in replay order, kind 'P' (arg
    ignored) or 'I' (arg = event name). Speeds at the pulse instants come from
    the span estimator over all the P inputs (speedAtLastPulse at the
    pulse instant = rawSpeed). end=None: tick past the end until quiescent.
    probes: optional sorted times; wk.probes gets (t, snapshot(t)) for each,
    taken after every input with t_input <= t."""
    ts = [t for t, k, _ in inputs if k == "P"]
    if ts != sorted(ts):
        raise ValueError("P inputs not in time order")
    met = Metrics(ts, p.stop_us, p.span_us)
    wk = Workout(p)
    wk.probes = []
    pr = list(probes or [])
    j = 0
    k = 0
    for t, kind, arg in inputs:
        while j < len(pr) and pr[j] < t:
            wk.tick(pr[j])
            wk.probes.append((pr[j], wk.snapshot(pr[j])))
            j += 1
        if kind == "P":
            wk.on_pulse(t, met.v_at[k])
            k += 1
        elif arg == "hold_begin":
            wk.hold_began(t)
        elif arg == "hold_abort":
            wk.hold_aborted(t)
        elif arg == "hold_complete":
            wk.hold_completed(t)
        else:
            wk.tick(t)            # units/touch events don't touch the workout (a tick)
    for tp in pr[j:]:
        if end is not None and tp > end:
            break
        wk.tick(tp)
        wk.probes.append((tp, wk.snapshot(tp)))
    wk.advance_to(end)
    return wk


def merge_inputs(pulses: list[int], ievents: list[tuple[int, str]]):
    """Golden replay order: by t_us; at equal t a P input precedes an I input
    (stable otherwise)."""
    ev = [(t, 0, "P", None) for t in pulses] + [(t, 1, "I", e) for t, e in ievents]
    ev.sort(key=lambda x: (x[0], x[1]))
    return [(t, k, a) for t, _, k, a in ev]


# Workout golden cases (tests 2.1-2.5, 2.7, 2.8, 2.12, 2.13).
def _synth_train(t0_us, t1_us, step_us=500_000):
    return list(range(t0_us, t1_us + 1, step_us))


def workout_cases(logs: dict) -> list[tuple]:
    """-> [(case, source, WorkoutParams, pulses, i_events)]"""
    s2 = [t for t, _ in logs["S2_walk"].pulses]
    s1b = [t for t, _ in logs["S1b_10rev"].pulses]
    s1 = [t for t, _ in logs["S1_handturn"].pulses]
    p28 = (_synth_train(10_000_000, 80_000_000)
           + [90_000_000, 90_500_000, 91_000_000, 91_500_000])
    h28 = [(50_250_000, "hold_begin"), (52_250_000, "hold_complete")]
    return [
        ("S2_walk", LOGS["S2_walk"], WorkoutParams(), s2, []),
        ("S2_walk_auto600", LOGS["S2_walk"], WorkoutParams(auto_end_us=600_000_000), s2, []),
        ("S1b_10rev", LOGS["S1b_10rev"], WorkoutParams(), s1b, []),
        ("S1_handturn_stop2000", LOGS["S1_handturn"], WorkoutParams(stop_us=2_000_000), s1, []),
        ("S1_handturn_stop2500", LOGS["S1_handturn"], WorkoutParams(stop_us=2_500_000), s1, []),
        ("S1_handturn_stop3000", LOGS["S1_handturn"], WorkoutParams(stop_us=3_000_000), s1, []),
        ("S2_nudges_264_272", LOGS["S2_walk"], WorkoutParams(),
         [t for t in s2 if 264_000_000 < t < 272_000_000], []),
        ("synth_2_8a", "synthetic", WorkoutParams(), p28 + _synth_train(110_000_000, 130_000_000), h28),
        ("synth_2_8b", "synthetic", WorkoutParams(), p28, h28),
        ("S2_hold_1105", LOGS["S2_walk"], WorkoutParams(), s2,
         [(1_105_000_000, "hold_begin"), (1_107_000_000, "hold_complete")]),
        ("synth_2_13", "synthetic", WorkoutParams(),
         _synth_train(10_000_000, 40_000_000) + [50_000_000],
         [(11_000_000, "hold_begin"), (13_000_000, "hold_complete")]),
        # A hold that BEGAN with no stop button shown (IDLE / SUMMARY) is
        # ignored even though the state is WARMUP when it completes; the
        # warm-up continues and confirms.
        ("synth_hold_idle_to_warmup", "synthetic", WorkoutParams(),
         _synth_train(1_500_000, 20_000_000),
         [(1_000_000, "hold_begin"), (3_000_000, "hold_complete")]),
        ("synth_hold_summary_to_warmup", "synthetic", WorkoutParams(),
         _synth_train(10_000_000, 20_000_000) + _synth_train(141_000_000, 160_000_000),
         [(21_000_000, "hold_begin"), (23_000_000, "hold_complete"),
          (142_000_000, "hold_begin"), (144_000_000, "hold_complete")]),
    ]


def fmt_out(o) -> str:
    if o[0] == "w":
        return f"w={o[1]} {o[2]} {o[3]} {o[4]}"
    return "z=" + " ".join(str(x) for x in o[1:])


def gen_workout(case, source, p: WorkoutParams, pulses, ievents) -> tuple[list[str], Workout]:
    wk = replay_workout(merge_inputs(pulses, ievents), p)
    lines = [f"case={case}", f"source={source}", f"params={p.header()}",
             f"inputs={len(pulses)} {len(ievents)}"]
    lines += [fmt_out(o) for o in wk.out]
    lines += [f"workouts={wk.workouts}", f"warmup_fails={wk.warmup_fails}", f"pauses={wk.pauses}"]
    return lines, wk


# --------------------------------------------------------------------------
# Display Format (core formatters, all integer)
# --------------------------------------------------------------------------
# Plain (stateless) formatters. The PaceDisplayFilter / SpeedDisplayFilter
# hysteresis and dead-band state are NOT modelled here. Units: "km" (metric)
# or "mi" (imperial). Exact unit facts: 1 km = 10**6 mm, 1 mi = 1609.344 m =
# 1_609_344 mm = 1_609_344_000 um, 1 h = 3600 s. Every formula below is an
# exact rational rounded once (no float, no double rounding).
PACE_MAX_S = 5999                # `--:--` above 99:59
PACE_NONE = "--:--"
MM_PER_KM = 1_000_000
MM_PER_MI = 1_609_344
UM_PER_KM_HUNDREDTH = 10_000_000      # 10 m
UM_PER_MI_HUNDREDTH = 16_093_440      # 16.09344 m
US_PER_S = 1_000_000
UNITS = ("km", "mi")


def _mm_per_unit(units: str) -> int:
    if units == "km":
        return MM_PER_KM
    if units == "mi":
        return MM_PER_MI
    raise ValueError(f"bad units {units!r}")


def pace_s(mmps: int, units: str) -> int:
    """Whole seconds per km/mi, round half up: (D + mmps//2) // mmps, D in mm.

    D/mmps is the exact pace in s. For odd mmps a remainder of exactly
    mmps/2 is impossible, so floor(mmps/2) gives true round-half-up."""
    assert mmps > 0
    d = _mm_per_unit(units)
    return (d + mmps // 2) // mmps


def fmt_pace(mmps: int, units: str) -> str:
    """Plain pace string (no visibility hysteresis): m:ss, minutes unpadded.

    `--:--` if mmps <= 0 (no finite pace; outside the defined domain,
    chosen as the only safe output) or the ROUNDED pace > PACE_MAX_S. The
    rounding to whole seconds happens before the m:ss split, so `x:60` is
    impossible by construction."""
    if mmps <= 0:
        return PACE_NONE
    s = pace_s(mmps, units)
    if s > PACE_MAX_S:
        return PACE_NONE
    return f"{s // 60}:{s % 60:02d}"


def speed_tenths(mmps: int, units: str) -> int:
    """Speed in tenths of km/h or mph, round half up, single rounding.

    km/h: v = mmps * 3600 / 10**6 = mmps*36/10**4 ; tenths = mmps*36/1000
          -> (mmps*36 + 500) // 1000.
    mph:  v = mmps * 3600 / 1609344 ; tenths = mmps*36000/1609344
          -> (mmps*36000 + 804672) // 1609344   (804672 = 1609344/2).
    Both are round-half-up of the exact rational, straight from mm/s (NOT via
    the hundredths `c` and then (c+5)/10, which double-rounds)."""
    assert mmps >= 0
    if units == "km":
        return (mmps * 36 + 500) // 1000
    if units == "mi":
        return (mmps * 36000 + MM_PER_MI // 2) // MM_PER_MI
    raise ValueError(f"bad units {units!r}")


def fmt_speed(mmps: int, units: str) -> str:
    t = speed_tenths(mmps, units)
    return f"{t // 10}.{t % 10}"


def distance_hundredths(pulses: int, units: str) -> int:
    """Floor to hundredths of km/mi from um = pulses * 597400."""
    assert pulses >= 0
    um = pulses * M_PER_PULSE_UM
    if units == "km":
        return um // UM_PER_KM_HUNDREDTH
    if units == "mi":
        return um // UM_PER_MI_HUNDREDTH
    raise ValueError(f"bad units {units!r}")


def fmt_distance(pulses: int, units: str) -> str:
    h = distance_hundredths(pulses, units)
    return f"{h // 100}.{h % 100:02d}"


def fmt_duration(us: int) -> str:
    """Floor to whole seconds; m:ss below 1 h, h:mm:ss from 1 h (hours unpadded)."""
    assert us >= 0
    s = us // US_PER_S
    if s < 3600:
        return f"{s // 60}:{s % 60:02d}"
    return f"{s // 3600}:{(s // 60) % 60:02d}:{s % 60:02d}"


PACE_SWEEP_LO = 361              # PACE_HIDE_MMPS
PACE_SWEEP_HI = 12000
MISC_SPEED_HI = 12000
MISC_DIST_HI = 20000


def gen_format_pace_sweep() -> list[str]:
    lines = ["case=format_pace_sweep", f"range={PACE_SWEEP_LO}..{PACE_SWEEP_HI}"]
    for v in range(PACE_SWEEP_LO, PACE_SWEEP_HI + 1):
        lines.append(f"p={v} {fmt_pace(v, 'km')} {fmt_pace(v, 'mi')}")
    return lines


def gen_format_misc() -> list[str]:
    lines = ["case=format_misc"]
    for v in range(0, MISC_SPEED_HI + 1):
        lines.append(f"speed={v} {fmt_speed(v, 'km')} {fmt_speed(v, 'mi')}")
    for n in range(0, MISC_DIST_HI + 1):
        lines.append(f"dist={n} {fmt_distance(n, 'km')} {fmt_distance(n, 'mi')}")
    return lines


# --------------------------------------------------------------------------
# Formatting
# --------------------------------------------------------------------------
_Q6 = decimal.Decimal("0.000001")


def fmt6(x) -> str:
    """Fixed 6 decimals, round-half-even, from an exact Fraction/Decimal/int."""
    if isinstance(x, Fraction):
        d = _DCTX.divide(decimal.Decimal(x.numerator), decimal.Decimal(x.denominator))
    else:
        d = decimal.Decimal(x)
    q = d.quantize(_Q6, rounding=decimal.ROUND_HALF_EVEN, context=_DCTX)
    if q == 0:
        q = abs(q)  # never print -0.000000
    return f"{q:f}"


README_TEXT = r'''# Goldens: format contract

Every file in this directory is GENERATED by `../oracle/oracle.py` (this
README included, so `diff -r` stays exact). Never edit by hand. Regenerate
(from `firmware/opentread/`):

    ../../.venv/bin/python -I test/oracle/oracle.py --out test/golden

`run_host_tests.sh` regenerates into a temp dir and fails if
`diff -r $TMP test/golden` is non-empty. The C++ tests only READ these files;
the C++ never writes goldens. A golden change needs a reason in the commit.

The oracle is an independent Python (stdlib only) model written from
the behaviour definitions below and the original pulse logger's
`acceptClosure`, not from the C++.

## General rules

- One `key=value` per line, `\n` line ends, ASCII, no trailing spaces.
- Integer quantities (µs, mm/s, counts) are plain base-10 integers.
- Metres are printed with exactly 6 decimals, rounded half-to-even from an
  exact value (Fractions; the one logarithm in the continuous 1.11 integral is
  evaluated with `decimal` at 60 digits). `-0.000000` is never printed.
- Log paths are repo-relative. Logs used:
  `S2_walk` = firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv,
  `S1b_10rev` = firmware/opentread/test/data/2026-10-18_1416_S1b_10rev.csv,
  `S1_handturn` = firmware/opentread/test/data/2026-10-18_1323_S1_handturn.csv,
  `bench_jumper` = firmware/opentread/test/data/2026-10-18_1315_bench_jumper.csv.

## Log parsing (shared by every file)

- A line is considered only if it starts with one of `E P M F H S` followed by
  a comma; everything else (`#` header, boot-ROM garbage, blanks) is skipped.
- A considered line must match its exact shape (`E,<t>,<0|1>`,
  `P,<t>,<n>,<dt>`, `M,<t>,<n>`, `F,<t>,<lowercase hex, even length>`,
  `H,<t>` + 7 integer fields, `S,<t>,<text>` split at most twice). Any
  mismatch is malformed and the oracle fails (never silently accepted).
- Exactly one boot banner (`S` line whose text contains `pulse logger` or
  `OpenTread fw=`) with a `line_level=<0|1>` token; its `t_us` and `line_level`
  initialise the pulse filter.
- Events are stably sorted by `t_us` before replay.

## filter_<name>.txt  (tests 1.1 / 1.16)

    log=<repo-relative path>
    edges=<number of E lines used>
    pulses=<n>
    p=<t_us>            (one per accepted pulse, in order)

Pulses = every E line (sorted) fed through the original pulse logger's
`acceptClosure` rule: state `{prevLevel=banner line_level, prevT=banner t_us,
armed=false, accepted=false}`; per edge: if `prevLevel==1 && t-prevT >= 5000` then
`armed=true`; `prevLevel,prevT = level,t`; a level-1 edge stops here; a
level-0 edge is accepted iff `armed` and (`!accepted` or
`t-lastAcceptT >= 20000`); every level-0 edge sets `armed=false`.

## metrics_<name>.txt  (test 1.10; S2_walk, S1b_10rev, S1_handturn)

    log=<repo-relative path>
    step_us=50000
    start_us=<first accepted pulse t - 1000000>
    end_us=<last accepted pulse t + 5000000>
    s=<t_us> <speed_mmps> <odometer_pulses>     (t = start_us + k*step_us <= end_us)

Pulses are the filter output above. A pulse AT time t counts as already
received at t (odometer = #pulses with t_p <= t). `speed_mmps` = speedMmps(t):

- rawSpeed(k) (cached per pulse): 0 if k is the first pulse or
  `t_k - t_{k-1} > 2500000`; otherwise the smallest N>=1 with
  `t_k - t_{k-N} >= 3000000`, stopping early with the current N if
  `t_{k-N} - t_{k-N-1} > 2500000`, if N == k, or if N == 63 (ring of 64
  timestamps including t_k); `v = (N*597400000 + dt/2) / dt` (integer
  division, dt = t_k - t_{k-N} in µs).
- speedMmps(now): 0 if no pulse yet or `now - t_last > 2500000`; else
  `v = rawSpeed(last)`, and if `now > t_last`,
  `v = min(v, 597400000 / d)` with `d = now - t_last`, integer division
  (FLOORED, no rounding term: `M·10⁹/(now−t_last)`, which keeps
  the bound `v <= M/(now−t_last)`; see test/INTEGRATION_NOTES.md D1).

## bias_<name>.txt  (test 1.11; S2_walk, S1b_10rev, S1_handturn)

    truth_m=<n * 0.5974>
    continuous_err_m=<...>
    phase0_err_m=<...> ... phase9_err_m=<...>
    phase_min_err_m=<...>
    phase_max_err_m=<...>
    phase_mean_err_m=<...>

- truth = n x 0.5974 m, n = number of accepted pulses.
- continuous: exact integral of the speed, per pulse k from t_k to
  `min(t_{k+1}, t_k + 2500000)`: v_k (integer mm/s from rawSpeed) until
  `tau = M/v_k` after t_k, then the real-valued tail `M/(t - t_k)`, i.e.
  `v_k*T` if `T <= tau`, else `M + M*ln(T/tau)`; 0 when v_k = 0.
  (The tail is NOT rounded to integer mm/s here, per the written
  definition; integrating the integer (floored-cap) speedMmps microsecond by
  microsecond instead moves it by -0.0107 / -0.0018 / -0.0046 m on
  S2_walk / S1b_10rev / S1_handturn, inside the 1.11 tolerance of 0.02 m.)
- phase p (0..9): `sum over i>=0 of speedMmps(t) * 1 s` at
  `t = t_first - 2000000 + p*100000 + i*1000000` while `t <= t_last + 5000000`.
- error = integral - truth. min/max/mean are over the 10 phase errors (mean
  exact, then rounded).

## console_ratio_S2_walk.txt  (test 1.15)

    median_ratio=<6 dp>
    samples=<n>

Sampling: every F line of S2_walk with `t_us` strictly greater than the
`t_us` of marker `M,<t>,1`, whose FTMS Treadmill Data payload has flags bit 0
clear (Instantaneous Speed present at bytes 2..3, uint16 LE, 0.01 km/h) and a
console speed c > 0, and where ours = speedMmps(t_F) (definition above,
evaluated at the F line's own t_us) is > 0. Each sample's ratio is
`(c * 1000 / 360) / ours` = `25c / (9 * ours)` (console mm/s over ours),
computed exactly. median = middle element of the sorted ratios, or the mean
of the two middle elements when the count is even.

## Workout goldens  workout_<case>.txt  (tests 2.1-2.5, 2.7, 2.8, 2.12, 2.13)

Model: the workout transition table, displayed values and hold-to-stop
semantics, written from the behaviour definition, not from the C++.

    case=<name>
    source=<repo-relative log path, or synthetic>
    params=STOP_US=<n> WARMUP_US=<n> AUTO_END_US=<n> SUMMARY_US=<n> SPAN_US=<n>
    inputs=<number of P inputs> <number of I inputs>
    w=<t_us> <FROM> <TO> <reason>        (every W transition)
    z=<t_us> <reason> <wk_pulses> <moving_us> <total_us> <top_mmps> <avg_mmps> <workout_id>
    workouts=<n>
    warmup_fails=<n>
    pauses=<n>

- w/z lines are in emission order, which is non-decreasing in t_us; a z line
  directly follows the W `hold_stop` / `autoend` line it belongs to (same
  t_us). States: `IDLE WARMUP RUNNING PAUSED SUMMARY`. W reasons: `bout_start
  warmup_ok warmup_fail pause resume hold_stop hold_cancel_warmup autoend
  summary_timeout rearm`. Z reason = `hold_stop` or `autoend`. workout_id
  starts at 1 per replay and increments on every `warmup_ok`.
- Footer: workouts = number of `warmup_ok`; warmup_fails = number of
  `warmup_fail`; pauses = number of W `pause` lines (this INCLUDES the final
  pause that precedes an auto-end; the tests' "N pauses" figures count only
  pauses that ended in a `resume`, i.e. this value - 1 when the workout ends by
  auto-end).
- Inputs. Log cases: the log's own P lines (not re-filtered E), sorted by
  t_us. I inputs (hold_begin / hold_complete) as listed below. Replay order is
  by t_us; at equal t_us a P input precedes an I input (for these cases the
  opposite order gives identical output). After the last input the replay
  "ticks past the end" until nothing is pending (auto-end, summary timeout).
- STOP_US and SPAN_US parameterise BOTH the speed estimator (as in metrics_*)
  and the workout. speedAtPulse v(t) = rawSpeed of that pulse (the span
  estimator at the pulse instant, no decay cap), over the case's P inputs.
- Time transitions are stamped at their threshold instant: pause and
  warmup_fail at `last+STOP` (due when `now-last > STOP`), autoend at
  `last+AUTO_END` (due when `now-last > AUTO_END`), summary_timeout at
  `summaryStart+SUMMARY` (due when `now-summaryStart >= SUMMARY`), rearm at
  `max(last+STOP, holdCompleteT)` (due when `now-last > STOP`, never before
  the hold completed). Pending transitions are applied in the order a 1 us
  tick would apply them: by due instant, then by stamp. Every input first
  applies all transitions due at or before its own t_us (a pulse at exactly
  `last+STOP` does not pause; a pulse at exactly `summaryStart+SUMMARY` sees
  the timeout first).
- Confirm (WARMUP, or a bout inside SUMMARY): on the first pulse with
  `t - boutStart >= WARMUP_US` (the guard `t - last <= STOP` is implied); that
  pulse is the baseline: wk_pulses=0, moving=0, top=0, vLast=v(t), tConf=t.
  RUNNING pulse: moving += t-last, ++wk_pulses, top=max(top,v(t)), vLast=v(t).
  PAUSED pulse (resume): ++wk_pulses, vLast=v(t) (=0), moving += 0.
  Pause: moving += tail(vLast).
  `tail(v) = v > 0 ? min(STOP_US, floor(2*597400*1000 / v)) : 0` (v in mm/s).
- Live values: moving(now) = committed + min(now-last, tail(vLast)) in
  RUNNING, committed in PAUSED; total(now) = now - tConf.
- Hold: hold_begin snapshots {active id (workout id in RUNNING/PAUSED, else
  0), the state, wk_pulses, moving(t), total(t), top}. hold_complete at `now`:
  no hold in progress -> ignored; active id changed -> void; the hold BEGAN in
  IDLE/SUMMARY, or completes in IDLE/SUMMARY -> ignored (no stop button was
  shown, e.g. a hold begun in IDLE that completes
  after a bout started is ignored and the warm-up continues); WARMUP -> IDLE
  `hold_cancel_warmup` (nothing recorded); RUNNING/PAUSED -> SUMMARY
  `hold_stop` with the snapshot as the final values (pulses during the hold
  are not counted). Both successful cases set the re-arm block. While it is
  set, pulses only update `last` (no bout_start) in IDLE and SUMMARY.
- Input time clamp: the model keeps latestUs, the
  maximum t of every input and tick processed; every input (P, hold_begin,
  hold_abort, hold_complete, units/touch ticks) is processed at
  max(t, latestUs), and snapshots are evaluated at max(now, latestUs). The
  device logs each I line with that t_eff, so in a device log the clamp is a
  no-op; it matters only for out-of-order direct calls. The golden inputs are
  in time order, so no golden case exercises it.
- Auto-end: total = autoEndT - tConf (as defined), moving = committed.
- Summary: a bout may start in SUMMARY once re-armed (W SUMMARY SUMMARY
  bout_start / warmup_fail); its confirm replaces the summary (W SUMMARY
  RUNNING warmup_ok). summary_timeout goes to WARMUP if such a bout is active,
  else IDLE.
- avg_mmps = moving_us > 0 ? (wk_pulses*597400*1000 + moving_us//2) // moving_us : 0.
  top_mmps = max v(t) over RUNNING pulses after the confirm pulse.
- Integer arithmetic throughout. The tests' quoted figures were made
  with real-valued speeds; integer mm/s moves moving time by < 1 ms and top
  speed by < 0.001 km/h (inside the stated +-1 ms / +-0.01 km/h).

Cases:

| case | test | inputs |
|---|---|---|
| S2_walk | 2.1 | S2_walk P lines |
| S2_walk_auto600 | 2.2 | S2_walk P lines, AUTO_END_US=600000000 |
| S1b_10rev | 2.3 | S1b_10rev P lines |
| S1_handturn_stop2000/2500/3000 | 2.4 | S1_handturn P lines, STOP_US=2000000/2500000/3000000 |
| S2_nudges_264_272 | 2.5 | S2_walk P lines with 264000000 < t < 272000000, from IDLE |
| synth_2_8a | 2.8 | P every 500000 from 10000000 to 80000000; P 90000000 90500000 91000000 91500000; P every 500000 from 110000000 to 130000000; I hold_begin 50250000, hold_complete 52250000 |
| synth_2_8b | 2.8 (b) | as synth_2_8a without the 110-130 s walk |
| S2_hold_1105 | 2.12 | S2_walk P lines; I hold_begin 1105000000, hold_complete 1107000000 |
| synth_2_13 | 2.13 | P every 500000 from 10000000 to 40000000, P 50000000; I hold_begin 11000000, hold_complete 13000000 |
| synth_hold_idle_to_warmup | 2.7 | I hold_begin 1000000 (IDLE); P every 500000 from 1500000 to 20000000 (bout_start 1.5 s); I hold_complete 3000000 (state WARMUP) -> ignored, the warm-up continues and confirms at 6.5 s |
| synth_hold_summary_to_warmup | 2.7 | P every 500000 from 10000000 to 20000000; I hold_begin 21000000, hold_complete 23000000 (hold-stop from PAUSED, re-arm at once); P every 500000 from 141000000 to 160000000 (bout_start behind the summary); I hold_begin 142000000 (SUMMARY); summary_timeout SUMMARY->WARMUP at 143 s during the hold; I hold_complete 144000000 (WARMUP) -> ignored, confirm at 146 s |

## Format goldens  format_pace_sweep.txt, format_misc.txt  (tests 3.1, 3.4, 3.5)

Model: the core display formatters (units, rounding, `PACE_MAX_S`), all
integer, written from their definitions, not from the C++. These are the
PLAIN formatters: no PaceDisplayFilter / SpeedDisplayFilter state (visibility hysteresis, dead-band, speed
hysteresis are tested separately and are not in these files). Unit columns
are always metric first, imperial second. Exact unit facts: 1 km = 10^6 mm,
1 mi = 1609.344 m = 1 609 344 mm (exactly; never 1.609), 1 h = 3600 s.
Every value is ONE integer rounding of an exact rational (no float, no
double rounding).

    case=format_pace_sweep
    range=361..12000
    p=<mmps> <km_pace> <mi_pace>          (every integer mm/s 361..12000, ascending)

- pace seconds `s = (D + mmps/2) / mmps` (integer division), D = 1000000
  (s/km) or 1609344 (s/mi): round half up of the exact pace D/mmps.
- string: `--:--` if `s > 5999` (PACE_MAX_S); else `<s/60>:<s%60 as 2
  digits>`, minutes unpadded (`8:00`, `46:10`, `99:48`). Seconds are rounded
  BEFORE the m:ss split, so `x:60` never appears. (`--:--` never appears in
  this range: the slowest row, 361 mm/s, is 46:10 /km and 74:18 /mi.)
- Not in the file: `mmps <= 0` formats as `--:--` (no finite pace).

    case=format_misc
    speed=<mmps> <kmh_tenths> <mph_tenths>   (every mmps 0..12000, ascending)
    dist=<pulses> <km> <mi>                   (every pulse count 0..20000, ascending)

- speed: tenths of km/h `t = (mmps*36 + 500) / 1000`; tenths of mph
  `t = (mmps*36000 + 804672) / 1609344` (804672 = 1609344/2). Both are round
  half up of the exact value, computed directly from mm/s. String
  `<t/10>.<t%10>` (`0.0`, `5.0`, `43.2`). NOTE: this is not the same as
  first rounding to hundredths `c` and then `(c+5)/10` (double
  rounding); that path belongs to the SpeedDisplayFilter, not to fmtSpeed.
- dist: `um = pulses * 597400`; hundredths of km `h = um / 10000000`,
  hundredths of mi `h = um / 16093440` (FLOOR, integer division). String
  `<h/100>.<h%100 as 2 digits>` (`0.00`, `0.99`, `1.00`, `11.94`).

fmtDuration (test 3.6) has no golden file (its rows are literal in
test_format.cpp):
`s = us / 1000000` floored; `m:ss` (minutes unpadded) when `s < 3600`, else
`h:mm:ss` (hours unpadded), e.g. 3599999999 -> `59:59`, 3600000000 ->
`1:00:00`, 36000000000 -> `10:00:00`.
'''


# --------------------------------------------------------------------------
# Golden writers
# --------------------------------------------------------------------------
def write(path: str, lines: list[str]) -> None:
    with open(path, "w", encoding="ascii", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")


def gen_filter(rel: str, log: Log, pulses: list[int]) -> list[str]:
    lines = [f"log={rel}", f"edges={len(log.edges)}", f"pulses={len(pulses)}"]
    lines += [f"p={t}" for t in pulses]
    return lines


def gen_metrics(rel: str, met: Metrics) -> list[str]:
    start = met.ts[0] - PRE_US
    end = met.ts[-1] + POST_US
    lines = [f"log={rel}", f"step_us={STEP_US}", f"start_us={start}", f"end_us={end}"]
    t = start
    while t <= end:
        lines.append(f"s={t} {met.speed(t)} {met.odometer(t)}")
        t += STEP_US
    return lines


def bias_numbers(met: Metrics) -> dict:
    n = len(met.ts)
    truth_um = n * M_PER_PULSE_UM
    cont_um = continuous_integral_um(met)
    cont_err_m = _DCTX.divide(_DCTX.subtract(cont_um, decimal.Decimal(truth_um)), decimal.Decimal(1_000_000))
    phase_err_m = []
    for p in range(10):
        mm = sampled_integral_mm(met, p)
        phase_err_m.append(Fraction(mm * 1000 - truth_um, 1_000_000))
    return {
        "truth_m": Fraction(truth_um, 1_000_000),
        "continuous_err_m": cont_err_m,
        "phases": phase_err_m,
    }


def gen_bias(met: Metrics) -> list[str]:
    b = bias_numbers(met)
    ph = b["phases"]
    lines = [f"truth_m={fmt6(b['truth_m'])}", f"continuous_err_m={fmt6(b['continuous_err_m'])}"]
    lines += [f"phase{p}_err_m={fmt6(e)}" for p, e in enumerate(ph)]
    lines += [
        f"phase_min_err_m={fmt6(min(ph))}",
        f"phase_max_err_m={fmt6(max(ph))}",
        f"phase_mean_err_m={fmt6(sum(ph, Fraction(0)) / len(ph))}",
    ]
    return lines


def gen_console(log: Log, met: Metrics) -> list[str]:
    r = console_ratios(log, met)
    return [f"median_ratio={fmt6(median(r))}", f"samples={len(r)}"]


def default_root() -> str:
    # <root>/firmware/opentread/test/oracle/oracle.py
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "..", ".."))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="OpenTread oracle: generate goldens")
    ap.add_argument("--out", required=True, help="directory to write golden files into")
    ap.add_argument("--root", default=default_root(), help="repo root (default: derived from script path)")
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    try:
        logs = {name: Log(os.path.join(args.root, rel)) for name, rel in LOGS.items()}
    except (OSError, LogError) as e:
        print(f"oracle: {e}", file=sys.stderr)
        return 2

    with open(os.path.join(args.out, "README.md"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(README_TEXT)

    for name, rel in LOGS.items():
        log = logs[name]
        pulses = filter_edges(log.boot_t, log.boot_level, log.edges)
        write(os.path.join(args.out, f"filter_{name}.txt"), gen_filter(rel, log, pulses))
        if name in METRIC_LOGS:
            if not pulses:
                print(f"oracle: {name}: no pulses", file=sys.stderr)
                return 2
            met = Metrics(pulses)
            write(os.path.join(args.out, f"metrics_{name}.txt"), gen_metrics(rel, met))
            write(os.path.join(args.out, f"bias_{name}.txt"), gen_bias(met))
            if name == CONSOLE_LOG:
                try:
                    lines = gen_console(log, met)
                except (LogError, ValueError) as e:
                    print(f"oracle: {e}", file=sys.stderr)
                    return 2
                write(os.path.join(args.out, f"console_ratio_{name}.txt"), lines)

    for case, source, p, pulses, ievents in workout_cases(logs):
        lines, _ = gen_workout(case, source, p, pulses, ievents)
        write(os.path.join(args.out, f"workout_{case}.txt"), lines)

    write(os.path.join(args.out, "format_pace_sweep.txt"), gen_format_pace_sweep())
    write(os.path.join(args.out, "format_misc.txt"), gen_format_misc())
    return 0


if __name__ == "__main__":
    sys.exit(main())
