#!/usr/bin/env python3
"""Offline checks on a captured OpenTread (or pulse-logger) serial log.

Run with the pinned interpreter, standard library only:

    .venv/bin/python -I tools/replay.py logs/<file>.csv
    .venv/bin/python -I tools/replay.py logs/<file>.csv --expect-build 1a2b3c4d --require-release
    .venv/bin/python -I tools/replay.py logs/<file>.csv --expect-build 1a2b3c4d --require-release --allow-dev
                                                        (a dev=1 build: every other release flag)
    .venv/bin/python -I tools/replay.py --self-test

Checks (the treadmill replay test, plus log-integrity checks):

  precondition  banner verb=2, no runtime verbosity change away from 2, every
                H debug_drop (field 6) = 0, no malformed line after the banner,
                at least one H line. Exact-line checks are INVALID without it.
                (pulse-logger logs: the logger always printed every E/P line and
                had no debug drop counter, so only the malformed-line rule
                applies.)
  (a) refilter  the logged E lines, re-filtered with the pulse-logger rule
                (oracle.filter_edges, initialised from the banner's t_us and
                line_level), give exactly the device's P timestamps.
  p_sequence    P lines are numbered 1..N in file order, dt = t - previous t
                (0 for the first).
  h_pulses      every H accepted_pulses (field 2) equals the number of P lines
                printed before it ("P-count truth").
  h_edges       every H isr_edges (field 3) equals the number of E lines
                printed before it (reed edges only), so a lost E line fails
                even when (a) can't see it.
  (b) oracle    the device's P lines replayed through the oracle
                (oracle.Metrics): every K speed_mmps is within +-1 mm/s of
                the oracle's speedMmps at that K's t_us, and every K
                odo_pulses equals the oracle odometer at that t_us.
  w_oracle      the device's own P and I lines (stably sorted by t_us,
                file order on ties) replayed through the oracle workout model
                (oracle.replay_workout, short=1 -> AUTO_END 30 s / SUMMARY
                12 s) up to the log's last t_us + 1 us (a log may END on a W
                line whose strict threshold `now - last > X` was due 1 us after
                its stamp): the W transitions must be identical (t_us to the
                us, from, to, reason). Oracle transitions stamped in the last
                1 s of the log may be missing from the device (printed after
                the capture ended).
  z_oracle      the same for the Z summary lines (every field).
  hold_result   every I hold_complete arg (ok/void/ignored) equals the
                oracle's result for that input.
  k_workout     every K line has moving <= total; total never decreases
                between consecutive K lines of one workout
                (RUNNING/PAUSED/SUMMARY, split at IDLE/WARMUP K lines and at
                device W warmup_ok). Workout checks are SKIPped for firmware
                without the workout (no W/I/Z line and every K state '-').
  l_backlight   the L lines after the S ui;reset line (which gives
                the idle-clock start and dim_us/off_us) equal a backlight
                model run on the device's P lines + I touch_down (activity)
                and W lines (SUMMARY = FULL; leaving SUMMARY restarts the idle
                timers at its stamp): same levels in the same order, each L
                0..50 ms after the model's timer/summary instant, or 0..100 ms
                after the triggering P / touch_down (wake latency).
                Levels must be 255/30/0. SKIP without a ui;reset line.
  n_nvs         N lines: exactly 2 boot reads; every units commit
                >= 3 s after the last I units_toggle (none before the first
                toggle: never at boot), >= 10 s after the previous commit, and
                > STOP after the last P (stationary); <= 1 defer per toggle.
                Commits inside OT_DEV nX / nw<k> windows (S cmd;nX, cmd;nw ..
                nvs;forced_done) are exempt. SKIP without N lines.
  ble_payload   every R (RSC Measurement) and X (FTMS Treadmill Data)
                line decodes to exactly what the K line of the same t_us
                encodes (independent Python encoder: flags, speed rounding and
                clamps, RSC distance floor(odo x 5.974) dm, FTMS distance
                floor(wk x 0.5974) m, elapsed floor(total_ms / 1000) s); one R
                (and X when ftms=1) per K; no X when ftms=0; the RSC distance
                never decreases; the FTMS distance and elapsed never decrease
                within a workout (same split as k_workout).
  ble_cp        the B lines keep the CP rules: never a second cp_ind
                while one is in flight (released by cp_ack, cp_timeout, a
                status / send_fail cp_drop of that link, or a disconnect with
                token=1), and every cp_ind is >= 5 ms after that link's last
                cp_write. B lines must name a known event.
  (c) console   K speed x 1.143 (the console reads ~14% long) as a table to
                compare with the console display; for logs with console FTMS
                (F lines, pulse-logger captures) the measured median
                console/ours ratio.
  edge_drops    H drop_edges (field 4) = 0 for the whole session.
  reset_reason  banner reset_reason is 1 or 3; anything else fails (4 panic,
                5/6/7 watchdog, 9 brownout, 0 unknown, ...).
  isr_iram      banner isr_iram=1 (OpenTread firmware).
  reboots       exactly one boot banner and no protocol lines before it.
  build/release optional: --expect-build ID8 checks build=; --require-release
                checks sim=0 dev=0 short=0 loopback=0 bringup=0 verb=2 dirty=0
                and ftms=<--ftms, default 1> (1 = FTMS build, 0 = RSC only).
                --allow-dev (with --require-release; for a dev build driven by
                the serial hold commands): every other release flag is required,
                dev=1 is permitted (and reported).

Also reported: banner flags, drop counters, G loop stats, D heap minima,
counts of every line letter (unknown well-formed letters are "other").

Exit code: 0 = every check PASS (or SKIP/WARN), 1 = any FAIL/INVALID,
2 = usage or file error. --self-test runs the checker on the pulse-logger
logs in firmware/opentread/test/data/ (where (a) must pass) and on synthetic
OpenTread logs with injected faults (each fault must be caught).
"""
from __future__ import annotations

import argparse
import os
import re
import statistics
import sys
from fractions import Fraction

sys.dont_write_bytecode = True   # don't drop __pycache__ into test/oracle

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ORACLE_DIR = os.path.join(REPO, "firmware", "opentread", "test", "oracle")
sys.path.insert(0, ORACLE_DIR)
import oracle  # noqa: E402  (the reference model; never duplicated here)

CONSOLE_RATIO = 1.143            # console/ours ~ 1.143 (sanity, not a target)
SPEED_TOL_MMPS = 1               # (b): |K - oracle| <= 1 mm/s
BANNER_TAGS = ("M4 pulse logger", "OpenTread fw=")
BAD_RESET = {4: "panic", 5: "interrupt watchdog", 6: "task watchdog",
             7: "other watchdog", 9: "brownout"}
RELEASE_FLAGS = {"sim": "0", "dev": "0", "short": "0", "loopback": "0",
                 "bringup": "0", "verb": "2", "dirty": "0"}   # + ftms=<--ftms> (build dependent)


def release_flags(args) -> dict:
    """The release flag set this run requires: RELEASE_FLAGS plus ftms
    (--ftms); --allow-dev drops the dev=0 requirement."""
    want = dict(RELEASE_FLAGS, ftms=getattr(args, "ftms", "0") or "0")
    if getattr(args, "allow_dev", False):
        del want["dev"]
    return want

# --------------------------------------------------------------------------
# Line grammar. ASCII digits only; a known letter that doesn't match
# its exact shape is MALFORMED; a well-formed line with an unknown letter is
# "other" (counted, otherwise ignored).
# --------------------------------------------------------------------------
_D = r"(\d+)"
SHAPES = {
    "E": re.compile(rf"^E,{_D},([01])$", re.ASCII),
    "P": re.compile(rf"^P,{_D},{_D},{_D}$", re.ASCII),
    "M": re.compile(rf"^M,{_D},{_D}$", re.ASCII),
    "S": re.compile(rf"^S,{_D},(.*)$", re.ASCII | re.DOTALL),
    "H": re.compile(rf"^H,{_D}" + rf",{_D}" * 7 + "$", re.ASCII),
    "K": re.compile(rf"^K,{_D},([^,]+)" + rf",{_D}" * 6 + "$", re.ASCII),
    "D": re.compile(rf"^D,{_D}" + rf",{_D}" * 4 + "$", re.ASCII),
    # G: 5 fields after t (older firmware), or 7 (+ nvs_pass_max_us, nvs_passes;
    # loop_max/p99 then exclude NVS-write passes)
    "G": re.compile(rf"^G,{_D}" + rf",{_D}" * 5 + rf"(?:,{_D},{_D})?$", re.ASCII),
    "F": re.compile(rf"^F,{_D},((?:[0-9a-f]{{2}})+)$", re.ASCII),
    "W": re.compile(rf"^W,{_D},([A-Za-z_-]+),([A-Za-z_-]+),([a-z_]+)$", re.ASCII),
    "I": re.compile(rf"^I,{_D},([a-z_]+)(?:,([^,]*))?$", re.ASCII),
    "Z": re.compile(rf"^Z,{_D},([a-z_]+)" + rf",{_D}" * 6 + "$", re.ASCII),
    # backlight, NVS, BLE
    "L": re.compile(rf"^L,{_D},{_D}$", re.ASCII),
    "N": re.compile(rf"^N,{_D},([a-z_]+),([a-z_]+),([a-z0-9_]+)$", re.ASCII),
    "B": re.compile(rf"^B,{_D},([a-z_]+),(\d+|-),([^,]*)$", re.ASCII),
    "R": re.compile(rf"^R,{_D},([0-9a-f]{{16}})$", re.ASCII),
    "X": re.compile(rf"^X,{_D},([0-9a-f]{{18}})$", re.ASCII),
}
B_EVENTS = ("connect", "disconnect", "sub", "unsub", "adv_restart", "cp_write", "cp_ind", "cp_ack", "cp_drop",
            "cp_timeout", "ts", "ms")
CP_IND_DEFER_US = 5000           # a CP response is indicated >= 5 ms after the write
WALK_RUN_MMPS = 1944             # RSC running flag
WK_GRACE_US = 1_000_000          # w/z_oracle: trailing oracle items the capture may have missed
SHORT_AUTO_END_US = 30_000_000   # short-timer set (OT_SHORT_TIMERS=1 builds, banner short=1)
SHORT_SUMMARY_US = 12_000_000


def norm_state(x: str) -> str:
    """W/K state token -> IDLE/WARMUP/RUNNING/PAUSED/SUMMARY ('-' kept); None if unknown."""
    if x == "-":
        return x
    y = x.upper().replace("_", "").replace("-", "")
    return y if y in oracle.STATES else None
GENERIC = re.compile(r"^([A-Z]),(\d+)(?:,.*)?$", re.ASCII | re.DOTALL)
KV = re.compile(r"(?:^| )([A-Za-z_]+)=(\S+)")
VERB_CHANGE = re.compile(r"^verbosity;(\d+)$")


class Segment:
    """Everything between one boot banner and the next (or end of file)."""

    def __init__(self, banner_t=None, banner_text=None):
        self.banner_t = banner_t
        self.banner_text = banner_text
        self.kind = None
        self.flags: dict[str, str] = {}
        if banner_text is not None:
            self.kind = "device" if banner_text.startswith("OpenTread fw=") else "logger"
            self.flags = dict(KV.findall(banner_text))
        self.edges: list[tuple[int, int]] = []
        self.pulses: list[tuple[int, int, int]] = []      # (t, n, dt) in file order
        self.markers: list[tuple[int, int]] = []
        self.ftms: list[tuple[int, bytes]] = []
        self.h: list[tuple[int, tuple, int, int]] = []    # (t, fields, #P before, #E before)
        self.k: list[tuple] = []                          # (t, state, odo, wk, mmps, mov, tot, top)
        self.w: list[tuple] = []                          # (t, from, to, reason), states normalised
        self.z: list[tuple] = []                          # (t, reason, wk, mov_us, tot_us, top, avg, id)
        self.i: list[tuple] = []                          # (t, event, arg)
        self.b: list[tuple] = []                          # (t, event, conn, detail) in file order
        self.r: list[tuple[int, bytes]] = []              # RSC payloads sent
        self.x: list[tuple[int, bytes]] = []              # FTMS Treadmill Data sent
        self.wk_inputs: list[tuple] = []                  # (t, 'P'|'I', event) in file order
        self.g: list[tuple] = []
        self.d: list[tuple] = []
        self.l: list[tuple[int, int]] = []                # (t, level)
        self.ui_reset = None                              # (t, dim_us, off_us) from S ui;reset
        self.n: list[tuple] = []                          # (t, event, key, val)
        self.nvs_cmds: list[tuple[int, str]] = []         # (t, 'nX'|'nw'|'done') dev-command marks
        self.verb_changes: list[tuple[int, int]] = []
        self.status = 0
        self.other: dict[str, int] = {}
        self.malformed: list[tuple[int, str]] = []        # (lineno, text)
        self.noise = 0
        self.first_t = None
        self.last_t = None


def parse(lines):
    """lines: iterable of str (no line ends). Returns (pre_banner, [segments])."""
    pre = Segment()
    segs: list[Segment] = []
    cur = pre
    for lineno, line in enumerate(lines, 1):
        line = line.rstrip("\r")              # CRLF endings (a lone CR mid-line stays: malformed)
        if line.startswith("#") or not line.strip():
            continue
        letter = line[0] if len(line) >= 2 and line[1] == "," else None
        if letter is None or not ("A" <= letter <= "Z"):
            cur.noise += 1                    # boot ROM text, junk without "X,"
            continue
        shape = SHAPES.get(letter)
        m = shape.match(line) if shape else None
        if shape is not None and m is None:
            cur.malformed.append((lineno, line))
            continue
        if shape is None:
            if GENERIC.match(line):
                cur.other[letter] = cur.other.get(letter, 0) + 1
                t = int(GENERIC.match(line).group(2))
                cur.last_t = t if cur.last_t is None else max(cur.last_t, t)
            else:
                cur.malformed.append((lineno, line))
            continue
        t = int(m.group(1))
        if letter == "W" and (norm_state(m.group(2)) in (None, "-") or norm_state(m.group(3)) in (None, "-")
                              or m.group(4) not in oracle.W_REASONS):
            cur.malformed.append((lineno, line))
            continue
        if letter == "I" and m.group(2) not in oracle.I_EVENTS:
            cur.malformed.append((lineno, line))
            continue
        if letter == "K" and norm_state(m.group(2)) is None:
            cur.malformed.append((lineno, line))
            continue
        if letter == "B" and m.group(2) not in B_EVENTS:
            cur.malformed.append((lineno, line))
            continue
        if letter == "S" and m.group(2).startswith(BANNER_TAGS):
            cur = Segment(t, m.group(2))
            segs.append(cur)
        if cur.first_t is None:
            cur.first_t = t
        cur.last_t = t if cur.last_t is None else max(cur.last_t, t)
        if letter == "E":
            cur.edges.append((t, int(m.group(2))))
        elif letter == "P":
            cur.pulses.append((t, int(m.group(2)), int(m.group(3))))
            cur.wk_inputs.append((t, "P", None))
        elif letter == "W":
            cur.w.append((t, norm_state(m.group(2)), norm_state(m.group(3)), m.group(4)))
        elif letter == "I":
            cur.i.append((t, m.group(2), m.group(3) if m.group(3) is not None else ""))
            cur.wk_inputs.append((t, "I", m.group(2)))
        elif letter == "Z":
            cur.z.append((t, m.group(2)) + tuple(int(x) for x in m.groups()[2:]))
        elif letter == "M":
            cur.markers.append((t, int(m.group(2))))
        elif letter == "S":
            cur.status += 1
            v = VERB_CHANGE.match(m.group(2))
            if v:
                cur.verb_changes.append((t, int(v.group(1))))
            txt = m.group(2)
            if txt.startswith("ui;reset "):
                kv = dict(KV.findall(txt))
                if kv.get("dim_us", "").isdigit() and kv.get("off_us", "").isdigit():
                    cur.ui_reset = (t, int(kv["dim_us"]), int(kv["off_us"]))
            elif txt == "cmd;nX":
                cur.nvs_cmds.append((t, "nX"))
            elif txt.startswith("cmd;nw"):
                cur.nvs_cmds.append((t, "nw"))
            elif txt.startswith("nvs;forced_done"):
                cur.nvs_cmds.append((t, "done"))
        elif letter == "L":
            cur.l.append((t, int(m.group(2))))
        elif letter == "N":
            cur.n.append((t, m.group(2), m.group(3), m.group(4)))
        elif letter == "H":
            cur.h.append((t, tuple(int(x) for x in m.groups()[1:]), len(cur.pulses), len(cur.edges)))
        elif letter == "K":
            g = m.groups()
            cur.k.append((t, norm_state(g[1])) + tuple(int(x) for x in g[2:]))
        elif letter == "G":   # (t, loop_max, p99, render_max, render_count, touch_late[, nvs_pass_max, nvs_passes])
            cur.g.append((t,) + tuple(int(x) for x in m.groups()[1:] if x is not None))
        elif letter == "D":
            cur.d.append((t,) + tuple(int(x) for x in m.groups()[1:]))
        elif letter == "F":
            cur.ftms.append((t, bytes.fromhex(m.group(2))))
        elif letter == "B":
            cur.b.append((t, m.group(2), m.group(3), m.group(4)))
        elif letter == "R":
            cur.r.append((t, bytes.fromhex(m.group(2))))
        elif letter == "X":
            cur.x.append((t, bytes.fromhex(m.group(2))))
    return pre, segs


def read_lines(path):
    """Split on \\n only (as the oracle and the C++ replay parser do)."""
    with open(path, "rb") as fh:
        data = fh.read()
    return [ln.rstrip(b"\r").decode("utf-8", errors="replace") for ln in data.split(b"\n")]


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------
class Report:
    def __init__(self):
        self.checks: list[tuple[str, str, str]] = []   # (name, status, detail)
        self.info: list[str] = []
        self.table: list[str] = []

    def add(self, name, status, detail=""):
        self.checks.append((name, status, detail))

    def status(self, name):
        for n, s, _ in self.checks:
            if n == name:
                return s
        return None

    def failed(self):
        return [c for c in self.checks if c[1] in ("FAIL", "INVALID")]


def _fmt_list(xs, n=5):
    xs = list(xs)
    return ", ".join(str(x) for x in xs[:n]) + (f", ... (+{len(xs) - n})" if len(xs) > n else "")


def check_segment(seg: Segment, rep: Report, args, label=""):
    p = label
    fl = seg.flags
    is_ot = seg.kind == "device"
    sim = fl.get("sim") == "1"

    # ---- banner / identity -------------------------------------------------
    rep.info.append(f"{p}banner: {seg.banner_text}")
    lvl = fl.get("line_level")
    if lvl not in ("0", "1"):
        rep.add(p + "banner", "FAIL", "banner has no line_level=<0|1>; cannot initialise the filter")
        return
    rr = fl.get("reset_reason")
    if rr is None or not rr.isdigit():
        rep.add(p + "reset_reason", "FAIL", f"reset_reason missing/invalid ({rr})")
    elif rr in ("1", "3"):
        rep.add(p + "reset_reason", "PASS", f"reset_reason={rr}")
    else:   # only 1 (power-on/EN) or 3 (esp_restart) is a pass
        rep.add(p + "reset_reason", "FAIL",
                f"reset_reason={rr} ({BAD_RESET.get(int(rr), 'not 1 or 3')}; investigate)")
    if is_ot:
        iram = fl.get("isr_iram")
        rep.add(p + "isr_iram", "PASS" if iram == "1" else "FAIL", f"isr_iram={iram}")
        if fl.get("build") in (None, "00000000"):
            rep.add(p + "build_id", "WARN", f"build={fl.get('build')} (unknown build ID)")
    if args.expect_build:
        b = fl.get("build")
        rep.add(p + "build", "PASS" if b == args.expect_build else "FAIL",
                f"build={b}, expected {args.expect_build}")
    if args.require_release:
        want = release_flags(args)
        bad = {k: fl.get(k) for k, v in want.items() if fl.get(k) != v}
        ok_txt = " ".join(f"{k}={v}" for k, v in want.items())
        if getattr(args, "allow_dev", False):
            ok_txt += f" (dev={fl.get('dev')} permitted by --allow-dev)"
        rep.add(p + "release_flags", "FAIL" if bad else "PASS",
                ("not the release flag set: " + " ".join(f"{k}={v}" for k, v in bad.items())
                 + " (wanted " + ok_txt + ")") if bad else ok_txt)
    else:
        rel = all(fl.get(k) == v for k, v in release_flags(args).items())
        rep.info.append(f"{p}release flag set: {'yes' if rel else 'no'}"
                        + ("" if is_ot else " (pulse-logger log: flags not applicable)"))

    # ---- counts / stats ----------------------------------------------------
    span = (seg.last_t - seg.banner_t) / 1e6 if seg.last_t is not None else 0.0
    rep.info.append(
        f"{p}lines: E={len(seg.edges)} P={len(seg.pulses)} M={len(seg.markers)} S={seg.status} "
        f"H={len(seg.h)} K={len(seg.k)} G={len(seg.g)} D={len(seg.d)} F={len(seg.ftms)} "
        f"W={len(seg.w)} I={len(seg.i)} Z={len(seg.z)} "
        f"other={seg.other or 0} malformed={len(seg.malformed)}; ESP span {span:.1f} s after the banner")
    if seg.h:
        de = max(h[1][2] for h in seg.h)
        d6 = max(h[1][4] for h in seg.h)
        last = seg.h[-1][1]
        rep.info.append(f"{p}H drops: drop_edges max={de}, field6 "
                        f"{'debug_drop' if is_ot else 'dropped_ftms (pulse logger)'} max={d6}; last H "
                        f"accepted={last[0]} isr_edges={last[1]} line_level={last[6]}")
    if seg.g:
        gn = [g for g in seg.g if len(g) >= 8]
        rep.info.append(f"{p}G ({len(seg.g)} lines): loop_max_us max={max(g[1] for g in seg.g)}, "
                        f"loop_p99_us max={max(g[2] for g in seg.g)}, render_max_us max="
                        f"{max(g[3] for g in seg.g)}, touch_poll_late_max_us max={max(g[5] for g in seg.g)}"
                        + (f", nvs_pass_max_us max={max(g[6] for g in gn)} over {sum(g[7] for g in gn)} "
                           f"NVS-write pass(es) (excluded from loop_max_us)" if gn else
                           " (G without nvs_pass fields: older firmware)"))
    if seg.d:
        rep.info.append(f"{p}D ({len(seg.d)} lines): free min={min(d[1] for d in seg.d)}, "
                        f"min_free min={min(d[2] for d in seg.d)}, largest_8bit min="
                        f"{min(d[3] for d in seg.d)}, largest_dma min={min(d[4] for d in seg.d)}")
    if seg.verb_changes:
        rep.info.append(f"{p}verbosity changes: {_fmt_list(seg.verb_changes)}")

    # ---- precondition --------------------------------------------------------
    why = []
    if is_ot:
        if fl.get("verb") != "2":
            why.append(f"banner verb={fl.get('verb')} (needs 2)")
        if any(v != 2 for _, v in seg.verb_changes):
            why.append("verbosity changed away from 2 during the session")
        if not seg.h:
            why.append("no H line (debug_drop unknown)")
        elif any(h[1][4] != 0 for h in seg.h):
            first = next(h for h in seg.h if h[1][4] != 0)
            why.append(f"debug_drop={first[1][4]} at t={first[0]}")
    if seg.malformed:
        why.append(f"{len(seg.malformed)} malformed line(s), first: line {seg.malformed[0][0]} "
                   f"{seg.malformed[0][1][:60]!r}")
    pre_ok = not why
    tail = ""
    if seg.h and seg.last_t is not None and seg.last_t > seg.h[-1][0]:
        tail = f" (the last {(seg.last_t - seg.h[-1][0]) / 1e6:.1f} s after the last H are not covered by a drop counter)"
    rep.add(p + "precondition", "PASS" if pre_ok else "FAIL",
            ("verb=2, debug_drop=0, no malformed lines" if is_ot else
             "pulse-logger log: every E/P printed, no malformed lines") + tail if pre_ok else "; ".join(why))
    exact = "PASS" if pre_ok else "INVALID"

    # ---- edge drops ------------------------------------------------------------
    if seg.h:
        de = max(h[1][2] for h in seg.h)
        rep.add(p + "edge_drops", "PASS" if de == 0 else "FAIL", f"drop_edges max={de}")
    else:
        rep.add(p + "edge_drops", "SKIP" if not is_ot else "FAIL", "no H line")

    # ---- P sequence ----------------------------------------------------------------
    bad_seq = []
    prev_t = None
    for i, (t, n, dt) in enumerate(seg.pulses, 1):
        want_dt = 0 if prev_t is None else t - prev_t
        if n != i or dt != want_dt:
            bad_seq.append(f"#{i}: n={n} dt={dt} (want n={i} dt={want_dt})")
        prev_t = t
    rep.add(p + "p_sequence", (exact if not bad_seq else ("FAIL" if pre_ok else "INVALID")),
            f"{len(seg.pulses)} P lines numbered 1..N, dt consistent" if not bad_seq
            else f"{len(bad_seq)} bad: {_fmt_list(bad_seq, 3)}")

    # ---- H counter truth ----------------------------------------------------------
    if seg.h:
        bad_p = [(h[0], h[1][0], h[2]) for h in seg.h if h[1][0] != h[2]]
        rep.add(p + "h_pulses", (exact if not bad_p else ("FAIL" if pre_ok else "INVALID")),
                f"{len(seg.h)} H lines, accepted_pulses == P lines so far" if not bad_p else
                f"{len(bad_p)} mismatches (t, H, P lines): {_fmt_list(bad_p, 3)}")
        # isr_edges counts reed edges only (BOOT-button marker edges return
        # before the counter in app.cpp handleEdge), so with the
        # precondition met a mismatch means E lines are missing from the log
        # (e.g. lost on the host side), which (a) alone can't always see.
        bad_e = [(h[0], h[1][1], h[3]) for h in seg.h if h[1][1] != h[3]]
        if not bad_e:
            rep.add(p + "h_edges", exact, f"isr_edges == E lines so far on {len(seg.h)} H lines")
        else:
            rep.add(p + "h_edges", "FAIL" if pre_ok else "INVALID",
                    f"{len(bad_e)} mismatches (t, H, E lines): {_fmt_list(bad_e, 3)}")
    else:
        rep.add(p + "h_pulses", "SKIP" if not is_ot else "INVALID", "no H line")

    # ---- (a) re-filter --------------------------------------------------------------
    dev_p = sorted(t for t, _, _ in seg.pulses)
    if sim:
        rep.add(p + "a_refilter", "SKIP", "sim=1: P lines are injected, the ISR/filter path is not used")
    elif not seg.edges and dev_p:
        rep.add(p + "a_refilter", "FAIL" if pre_ok else "INVALID", f"{len(dev_p)} P lines but no E lines")
    else:
        edges = sorted(seg.edges, key=lambda e: e[0])        # stable sort by t_us
        ref = oracle.filter_edges(seg.banner_t, int(lvl), edges)
        if ref == dev_p:
            rep.add(p + "a_refilter", exact, f"{len(seg.edges)} E -> {len(ref)} P, identical to the device's P lines")
        else:
            i = next((j for j in range(min(len(ref), len(dev_p))) if ref[j] != dev_p[j]),
                     min(len(ref), len(dev_p)))
            rep.add(p + "a_refilter", "FAIL" if pre_ok else "INVALID",
                    f"re-filter {len(ref)} P vs device {len(dev_p)} P; first difference at #{i + 1}: "
                    f"ref={ref[i] if i < len(ref) else '-'} device={dev_p[i] if i < len(dev_p) else '-'}")

    # ---- (b) oracle replay of the device's P lines ------------------------------------
    met = oracle.Metrics(dev_p)
    if not seg.k:
        if is_ot and fl.get("verb") in ("1", "2") and span >= 2.0:
            rep.add(p + "b_oracle_speed", "FAIL", "OpenTread log at verb>=1 without K lines")
        else:
            rep.add(p + "b_oracle_speed", "SKIP", "no K lines" + (" (pulse-logger log)" if not is_ot else ""))
    else:
        diffs, bad_v, bad_o = [], [], []
        for k in seg.k:
            t, mmps, odo = k[0], k[4], k[2]
            ov = met.speed(t)
            oo = met.odometer(t)
            diffs.append(abs(mmps - ov))
            if abs(mmps - ov) > SPEED_TOL_MMPS:
                bad_v.append(f"t={t} K={mmps} oracle={ov}")
            if odo != oo:
                bad_o.append(f"t={t} K={odo} oracle={oo}")
        exact_n = sum(1 for x in diffs if x == 0)
        st = "PASS" if not bad_v else "FAIL"
        rep.add(p + "b_oracle_speed", st if pre_ok else "INVALID",
                f"{len(seg.k)} K lines: max |K - oracle| = {max(diffs)} mm/s, {exact_n} exact"
                + (f"; {len(bad_v)} beyond +-{SPEED_TOL_MMPS}: {_fmt_list(bad_v, 3)}" if bad_v else ""))
        rep.add(p + "b_oracle_odo", ("PASS" if not bad_o else "FAIL") if pre_ok else "INVALID",
                f"K odo_pulses == oracle odometer on {len(seg.k)} K lines" if not bad_o
                else f"{len(bad_o)} mismatches: {_fmt_list(bad_o, 3)}")

        # ---- (c) console comparison table ------------------------------------------------
        every = args.table_every
        if every and every > 0:
            rep.table.append(f"{p}(c) K speed vs expected console (console ~= ours x {CONSOLE_RATIO}):")
            rep.table.append("     t_s   K_mmps  ours_km/h  console_km/h  console_mph")
            next_t = None
            for k in seg.k:
                t, mmps = k[0], k[4]
                if next_t is not None and t < next_t:
                    continue
                next_t = t + int(every * 1e6)
                kmh = mmps * 0.0036
                rep.table.append(f"  {t / 1e6:8.1f}  {mmps:6d}  {kmh:9.2f}  {kmh * CONSOLE_RATIO:12.2f}"
                                 f"  {kmh * CONSOLE_RATIO / 1.609344:11.2f}")

    # ---- workout: W/Z/I vs the oracle, K invariants --------------------------------------
    check_workout(seg, rep, p, pre_ok)
    # ---- backlight (L) and NVS (N) --------------------------------------------------------
    check_backlight(seg, rep, p, pre_ok)
    check_nvs(seg, rep, p, pre_ok)

    # ---- BLE: R/X payloads vs K, CP token rules from the B lines -------------------------
    check_ble(seg, rep, p)

    # ---- (c') measured console ratio when the log has console FTMS (pulse logger) -----
    if seg.ftms and dev_p:
        ratios = []
        for t, payload in seg.ftms:
            c = oracle.ftms_speed_centikmh(payload)
            ours = met.speed(t)
            if c and ours:
                ratios.append(Fraction(25 * c, 9 * ours))
        if ratios:
            rep.info.append(f"{p}(c) console FTMS / ours: median {float(statistics.median(ratios)):.4f} over "
                            f"{len(ratios)} F lines with both speeds > 0 (expected ~{CONSOLE_RATIO})")


def _seq_diff(dev, ref, last_t):
    """Compare device vs oracle item lists (tuples starting with t_us).
    Returns (ok, detail). Oracle items stamped within WK_GRACE_US of the end
    of the log may be missing at the END of the device list."""
    n = min(len(dev), len(ref))
    i = next((j for j in range(n) if dev[j] != ref[j]), n)
    if i == len(dev) == len(ref):
        return True, ""
    if i == len(dev) and all(r[0] > last_t - WK_GRACE_US for r in ref[i:]):
        return True, f" ({len(ref) - i} oracle item(s) in the last {WK_GRACE_US // 1000} ms not yet printed)"
    d = " ".join(map(str, dev[i])) if i < len(dev) else "-"
    r = " ".join(map(str, ref[i])) if i < len(ref) else "-"
    return False, f"first difference at #{i + 1}: device [{d}] oracle [{r}] (device {len(dev)}, oracle {len(ref)})"


def rsc_expected(mmps: int, odo: int) -> bytes:
    """RSC Measurement 0x2A53 from the K line's values, written from the
    RSC spec, not from the C++."""
    flags = 0x02 | (0x04 if mmps >= WALK_RUN_MMPS else 0)
    sp = min(0xFFFF, (mmps * 256 + 500) // 1000) if mmps > 0 else 0
    dm = min(0xFFFFFFFF, odo * 597400 // 100000)
    return bytes([flags, sp & 0xFF, sp >> 8, 0]) + dm.to_bytes(4, "little")


def td_expected(mmps: int, wk: int, total_ms: int) -> bytes:
    """FTMS Treadmill Data 0x2ACD (flags 0x0404) from the K line's values:
    speed 0.01 km/h, distance m (u24), elapsed s (u16)."""
    sp = min(0xFFFF, (mmps * 36 + 50) // 100) if mmps > 0 else 0
    dist = min(0xFFFFFF, wk * 597400 // 10 ** 6)
    el = min(0xFFFF, max(0, total_ms) // 1000)
    return bytes([0x04, 0x04]) + sp.to_bytes(2, "little") + dist.to_bytes(3, "little") + el.to_bytes(2, "little")


def check_ble(seg: Segment, rep: Report, p: str):
    fl = seg.flags
    ftms = fl.get("ftms") == "1"
    if not seg.r and not seg.x:
        rep.add(p + "ble_payload", "SKIP", "no R/X lines (firmware without BLE logging, or verbosity 0)")
    else:
        kby = {}
        for k in seg.k:
            kby.setdefault(k[0], k)
        bad, missing = [], []
        for t, payload in seg.r:
            k = kby.get(t)
            if k is None:
                missing.append(f"R t={t}")
                continue
            want = rsc_expected(k[4], k[2])
            if payload != want:
                bad.append(f"R t={t} {payload.hex()} != {want.hex()}")
        for t, payload in seg.x:
            k = kby.get(t)
            if k is None:
                missing.append(f"X t={t}")
                continue
            want = td_expected(k[4], k[3], k[6])
            if payload != want:
                bad.append(f"X t={t} {payload.hex()} != {want.hex()}")
        if not ftms and seg.x:
            bad.append(f"{len(seg.x)} X lines in an ftms=0 build")
        if seg.k:
            if len(seg.r) != len(seg.k):
                bad.append(f"{len(seg.r)} R lines for {len(seg.k)} K lines")
            if ftms and len(seg.x) != len(seg.k):
                bad.append(f"{len(seg.x)} X lines for {len(seg.k)} K lines")
        prev = -1
        for t, payload in seg.r:
            d = int.from_bytes(payload[4:8], "little")
            if d < prev:
                bad.append(f"RSC distance decreases at t={t}: {prev} -> {d} dm")
            prev = d
        # FTMS distance / elapsed within a workout (K state RUNNING/PAUSED/SUMMARY,
        # split at IDLE/WARMUP and at warmup_ok, as k_workout)
        confirms = [w[0] for w in seg.w if w[3] == "warmup_ok"]
        xby = dict(seg.x)
        prevx = None
        for k in sorted(seg.k, key=lambda x: x[0]):
            if k[1] not in (oracle.RUNNING, oracle.PAUSED, oracle.SUMMARY) or k[0] not in xby:
                prevx = None
                continue
            cur = xby[k[0]]
            dist, el = int.from_bytes(cur[4:7], "little"), int.from_bytes(cur[7:9], "little")
            if prevx is not None and not any(prevx[0] <= c <= k[0] for c in confirms):
                if dist < prevx[1] or el < prevx[2]:
                    bad.append(f"FTMS distance/elapsed decrease t={prevx[0]}->{k[0]}")
            prevx = (k[0], dist, el)
        st = "PASS" if not (bad or missing) else "FAIL"
        rep.add(p + "ble_payload", st,
                f"{len(seg.r)} R + {len(seg.x)} X lines == the K snapshots, RSC distance monotonic, FTMS "
                f"distance/elapsed monotonic per workout" if st == "PASS" else
                "; ".join(x for x in (f"{len(bad)} mismatches: {_fmt_list(bad, 3)}" if bad else "",
                                      f"{len(missing)} without a K line: {_fmt_list(missing, 3)}" if missing else "")
                          if x))
    cp = [b for b in seg.b if b[1].startswith("cp_") or b[1] == "disconnect"]
    if not any(b[1].startswith("cp_") for b in cp):
        rep.add(p + "ble_cp", "SKIP", "no CP lines")
        return
    bad = []
    inflight = None          # conn of the indication in flight
    last_write = {}
    n_ind = 0
    for t, ev, conn, detail in cp:
        if ev == "cp_write":
            last_write[conn] = t
        elif ev == "cp_ind":
            n_ind += 1
            if inflight is not None:
                bad.append(f"t={t} cp_ind to {conn} while {inflight} is in flight")
            if conn in last_write and t - last_write[conn] < CP_IND_DEFER_US:
                bad.append(f"t={t} cp_ind {t - last_write[conn]} us after the write (< {CP_IND_DEFER_US})")
            inflight = conn
        elif ev in ("cp_ack", "cp_timeout"):
            inflight = None
        elif ev == "cp_drop" and detail.split(";")[0] in ("status", "send_fail") and conn != "-":
            inflight = None
        elif ev == "disconnect" and "token=1" in detail:
            inflight = None
    rep.add(p + "ble_cp", "PASS" if not bad else "FAIL",
            f"{n_ind} cp_ind: one in flight at most, each >= {CP_IND_DEFER_US // 1000} ms after its write"
            if not bad else f"{len(bad)} violations: {_fmt_list(bad, 3)}")


def check_workout(seg: Segment, rep: Report, p: str, pre_ok: bool):
    has = bool(seg.w or seg.i or seg.z) or any(k[1] != "-" for k in seg.k)
    names = ("w_oracle", "z_oracle", "hold_result", "k_workout")
    if not has:
        for n in names:
            rep.add(p + n, "SKIP", "no W/I/Z lines and K state '-' (firmware without the workout)")
        return
    fl = seg.flags
    short = fl.get("short") == "1"
    prm = oracle.WorkoutParams(auto_end_us=SHORT_AUTO_END_US if short else oracle.AUTO_END_US,
                               summary_us=SHORT_SUMMARY_US if short else oracle.SUMMARY_US)
    inputs = sorted(seg.wk_inputs, key=lambda x: x[0])          # stable: file order on ties
    # last_t + 1: strict ('>') thresholds are due 1 us after their stamp, so a
    # log ending exactly on a W pause/warmup_fail/rearm/autoend line needs it.
    end = seg.last_t + 1
    wk = oracle.replay_workout(inputs, prm, end=end)
    ref_w = [o[1:] for o in wk.out if o[0] == "w"]
    ref_z = [o[1:] for o in wk.out if o[0] == "z"]
    dev_w = sorted(seg.w, key=lambda x: x[0])
    dev_z = sorted(seg.z, key=lambda x: x[0])
    exact = "PASS" if pre_ok else "INVALID"
    bad = "FAIL" if pre_ok else "INVALID"
    pinfo = f"params {prm.header()}"
    ok, det = _seq_diff(dev_w, ref_w, end)
    rep.add(p + "w_oracle", exact if ok else bad,
            (f"{len(dev_w)} W lines identical to the oracle on {sum(1 for x in inputs if x[1] == 'P')} P + "
             f"{len(seg.i)} I inputs{det}; {pinfo}") if ok else det + f"; {pinfo}")
    ok, det = _seq_diff(dev_z, ref_z, end)
    rep.add(p + "z_oracle", exact if ok else bad,
            f"{len(dev_z)} Z lines identical to the oracle{det}" if ok else det)
    dev_h = [(t, a) for t, e, a in sorted(seg.i, key=lambda x: x[0]) if e == "hold_complete"]
    ref_h = list(wk.hold_results)
    ok = dev_h == ref_h
    i = next((j for j in range(min(len(dev_h), len(ref_h))) if dev_h[j] != ref_h[j]), min(len(dev_h), len(ref_h)))
    rep.add(p + "hold_result", exact if ok else bad,
            f"{len(dev_h)} hold_complete results identical to the oracle" if ok else
            f"first difference at #{i + 1}: device {dev_h[i] if i < len(dev_h) else '-'} "
            f"oracle {ref_h[i] if i < len(ref_h) else '-'}")
    # K invariants: moving <= total everywhere; total non-decreasing within a workout.
    confirms = [w[0] for w in dev_w if w[3] == "warmup_ok"]
    bad_mt, bad_dec = [], []
    prev = None
    for k in sorted(seg.k, key=lambda x: x[0]):
        t, st, mov, tot = k[0], k[1], k[5], k[6]
        if mov > tot:
            bad_mt.append(f"t={t} moving={mov} total={tot}")
        if st not in (oracle.RUNNING, oracle.PAUSED, oracle.SUMMARY):
            prev = None
            continue
        if prev is not None and not any(prev[0] <= c <= t for c in confirms) and tot < prev[6]:
            bad_dec.append(f"t={prev[0]}->{t} total {prev[6]}->{tot}")
        prev = k
    if not seg.k:
        rep.add(p + "k_workout", "SKIP", "no K lines")
    else:
        rep.add(p + "k_workout", "PASS" if not (bad_mt or bad_dec) else "FAIL",
                f"{len(seg.k)} K lines: moving <= total, total non-decreasing within each workout"
                if not (bad_mt or bad_dec) else
                "; ".join(x for x in (f"{len(bad_mt)} moving > total: {_fmt_list(bad_mt, 3)}" if bad_mt else "",
                                      f"{len(bad_dec)} total decreases: {_fmt_list(bad_dec, 3)}" if bad_dec else "")
                          if x))


# --------------------------------------------------------------------------
# Backlight (L lines) and NVS (N lines). Written from the behaviour
# definitions, not from the C++.
# --------------------------------------------------------------------------
BL_LEVEL = {255: "full", 30: "dim", 0: "off"}     # config_hw.h BL_FULL / BL_DIM / BL_OFF
BL_TIMER_TOL_US = 50_000       # acceptance: timers within +-50 ms (L is stamped when applied: >= nominal)
BL_WAKE_TOL_US = 100_000       # acceptance: wake latency L - max(P, I touch_down) < 100 ms
NVS_SETTLE_US = 3_000_000      # units settle time before a commit
NVS_MIN_INTERVAL_US = 10_000_000
STOP_US = 2_500_000            # (the belt is stationary when now - last P > STOP)


BL_LEARN_P_US = 40_000         # a P is fed to the policy when the loop drains it (<= one long pass)
BL_LEARN_TOUCH_US = 60_000     # a touch DOWN (stamped at its 1st sample) is known at its 3rd sample (+ lateness)


def backlight_model(reset_t, dim_us, off_us, activity, w_lines, end, touch=()):
    """Expected backlight transitions [(t, level, cause, group)] after the
    ui;reset instant: activity = accepted pulse / touch DOWN times; SUMMARY
    (W into it) is FULL; leaving SUMMARY (W out of it) restarts the idle
    timers at its stamp; otherwise FULL < dim <= DIM < off <= OFF of idle
    time. The device learns of an activity a little after its stamp
    (`touch` = the subset of `activity` that are touch DOWNs), so a timer
    threshold inside that learn window MAY show on the device just before the
    wake: such transitions are OPTIONAL (group > 0; a group is matched whole
    or skipped whole); every other entry has group 0 (mandatory)."""
    touch = set(touch)
    ev = [(t, 0, "act") for t in activity if t >= reset_t]
    for t, frm, to, _ in w_lines:
        if t < reset_t:
            continue
        if to == oracle.SUMMARY and frm != oracle.SUMMARY:
            ev.append((t, 1, "sum_on"))
        elif frm == oracle.SUMMARY and to != oracle.SUMMARY:
            ev.append((t, 1, "sum_off"))
    ev.sort(key=lambda e: (e[0], e[1]))
    out, last, level, in_sum, groups = [], reset_t, "full", False, 0
    rank = {"full": 0, "dim": 1, "off": 2}

    def advance(to_t):          # timer thresholds strictly before to_t
        nonlocal level
        if in_sum:
            return
        for thr, lv in ((last + dim_us, "dim"), (last + off_us, "off")):
            if thr < to_t and rank[level] < rank[lv]:
                level = lv
                out.append((thr, lv, "timer", 0))

    for t, _, kind in ev:
        advance(t)
        if kind == "act":
            if not in_sum:      # thresholds inside the learn window: optional, then back to FULL
                win = BL_LEARN_TOUCH_US if t in touch else BL_LEARN_P_US
                opt = [(thr, lv) for thr, lv in ((last + dim_us, "dim"), (last + off_us, "off"))
                       if t <= thr <= t + win and rank[level] < rank[lv]]
                if opt:
                    groups += 1
                    for thr, lv in opt:
                        out.append((thr, lv, "learn", groups))
                    if level == "full":
                        out.append((t, "full", "learn", groups))
            last = max(last, t)
            if not in_sum and level != "full":
                out.append((t, "full", "activity", 0))
            level = "full"
        elif kind == "sum_on":
            in_sum = True
            if level != "full":
                out.append((t, "full", "summary", 0))
            level = "full"
        else:
            in_sum = False
            last = max(last, t)
    advance(end + 1)
    return out


def check_backlight(seg: Segment, rep: Report, p: str, pre_ok: bool):
    name = p + "l_backlight"
    if seg.ui_reset is None:
        rep.add(name, "SKIP", "no S ui;reset line (older firmware, or a bring-up build)")
        return
    reset_t, dim_us, off_us = seg.ui_reset
    bad_lv = [f"t={t} level={v}" for t, v in seg.l if v not in BL_LEVEL]
    if bad_lv:
        rep.add(name, "FAIL", f"L level not 255/30/0: {_fmt_list(bad_lv, 3)}")
        return
    dev = [(t, BL_LEVEL[v]) for t, v in sorted(seg.l) if t >= reset_t]
    touches = [t for t, e, _ in seg.i if e == "touch_down"]
    acts = [t for t, _, _ in seg.pulses] + touches
    ref = backlight_model(reset_t, dim_us, off_us, sorted(acts), sorted(seg.w), seg.last_t, touch=touches)
    bad = "FAIL" if pre_ok else "INVALID"
    late = {"timer": 0, "summary": 0, "activity": 0, "learn": 0}
    tols = {"timer": BL_TIMER_TOL_US, "summary": BL_TIMER_TOL_US, "activity": BL_WAKE_TOL_US,
            "learn": BL_LEARN_TOUCH_US + BL_WAKE_TOL_US}

    def ok(d, r):
        return d[1] == r[1] and 0 <= d[0] - r[0] <= tols[r[2]]

    i = j = learned = 0
    while i < len(ref):
        rt, rl, cause, g = ref[i]
        if g:   # optional group: matched whole or skipped whole
            n = 1
            while i + n < len(ref) and ref[i + n][3] == g:
                n += 1
            if j + n <= len(dev) and all(ok(dev[j + m], ref[i + m]) for m in range(n)):
                for m in range(n):
                    late["learn"] = max(late["learn"], dev[j + m][0] - ref[i + m][0])
                j += n
                learned += 1
            i += n
            continue
        if j >= len(dev):
            if all(r[0] > seg.last_t - WK_GRACE_US for r in ref[i:] if not r[3]):
                break   # the capture ended before the device printed it
            rep.add(name, bad, f"missing L #{j + 1}: model t={rt} {rl} ({cause}); device has {len(dev)}")
            return
        if not ok(dev[j], ref[i]):
            rep.add(name, bad, f"L #{j + 1}: device t={dev[j][0]} {dev[j][1]}, model t={rt} {rl} ({cause}; "
                               f"allowed lateness 0..{tols[cause] // 1000} ms)")
            return
        late[cause] = max(late[cause], dev[j][0] - rt)
        i += 1
        j += 1
    if j < len(dev):
        rep.add(name, bad, f"extra device L #{j + 1}: t={dev[j][0]} {dev[j][1]} (model exhausted)")
        return
    rep.add(name, "PASS" if pre_ok else "INVALID",
            f"{len(dev)} L lines match the backlight model (dim {dim_us // 1000000} s, off {off_us // 1000000} s): "
            f"max lateness timer {late['timer'] / 1000:.1f} ms, summary {late['summary'] / 1000:.1f} ms, "
            f"wake {late['activity'] / 1000:.1f} ms" + (f"; {learned} transition(s) inside an activity's learn "
                                                         f"window" if learned else ""))


def check_nvs(seg: Segment, rep: Report, p: str, pre_ok: bool):
    name = p + "n_nvs"
    if not seg.n:
        rep.add(name, "SKIP", "no N lines (older firmware, or a bring-up build)")
        return
    bad = "FAIL" if pre_ok else "INVALID"
    problems = []
    reads = [n for n in seg.n if n[1] == "read"]
    if len(reads) != 2:
        problems.append(f"{len(reads)} N read lines (want 2: schema + units, once at boot)")
    # Dev-command windows (OT_DEV nX / nw<k>) are exempt from the policy rules.
    windows, open_t = [], None
    for t, kind in sorted(seg.nvs_cmds):
        if kind == "nX":
            windows.append((t, t))
        elif kind == "nw":
            open_t = t
        elif kind == "done" and open_t is not None:
            windows.append((open_t, t))
            open_t = None
    if open_t is not None:
        windows.append((open_t, seg.last_t))
    exempt = lambda t: any(a <= t <= b for a, b in windows)  # noqa: E731
    toggles = sorted(t for t, e, _ in seg.i if e == "units_toggle")
    pulses = sorted(t for t, _, _ in seg.pulses)
    commits = sorted(t for t, ev, key, _ in seg.n if ev == "commit" and key == "units" and not exempt(t))
    defers = sorted(t for t, ev, _, _ in seg.n if ev == "defer")
    prev = None
    for t in commits:
        before = [x for x in toggles if x <= t]
        if not before:
            problems.append(f"commit at t={t} with no units_toggle before it (a write at boot?)")
        elif t - before[-1] < NVS_SETTLE_US:
            problems.append(f"commit at t={t} only {(t - before[-1]) / 1e6:.3f} s after the toggle (< 3 s)")
        if prev is not None and t - prev < NVS_MIN_INTERVAL_US:
            problems.append(f"commit at t={t} only {(t - prev) / 1e6:.3f} s after the previous one (< 10 s)")
        lp = [x for x in pulses if x <= t]
        if lp and t - lp[-1] <= STOP_US:
            problems.append(f"commit at t={t} while moving (last P {(t - lp[-1]) / 1e6:.3f} s before)")
        prev = t
    for a, b in zip([None] + toggles, toggles + [None]):
        n = sum(1 for d in defers if (a is None or d >= a) and (b is None or d < b))
        if n > 1:
            problems.append(f"{n} defer lines between toggles at {a} and {b} (want <= 1)")
    if problems:
        rep.add(name, bad, "; ".join(problems[:4]) + (f" (+{len(problems) - 4})" if len(problems) > 4 else ""))
    else:
        rep.add(name, "PASS" if pre_ok else "INVALID",
                f"{len(commits)} commits (+{len(seg.n) - len(commits) - len(reads) - len(defers)} other N), "
                f"{len(defers)} defers, {len(toggles)} toggles: >= 3 s settle, >= 10 s apart, stationary, "
                f"none at boot" + (f"; {len(windows)} dev-command window(s) exempt" if windows else ""))


def analyse(lines, args) -> Report:
    rep = Report()
    pre, segs = parse(lines)
    orphan = len(pre.edges) + len(pre.pulses) + len(pre.h) + len(pre.k) + len(pre.markers)
    if not segs:
        rep.add("reboots", "FAIL", "no boot banner ('M4 pulse logger' / 'OpenTread fw=') in the log")
        return rep
    bad = []
    if len(segs) > 1:
        bad.append(f"{len(segs)} boot banners (ESP rebooted {len(segs) - 1}x) at t_us "
                   + _fmt_list(s.banner_t for s in segs))
    if orphan:
        bad.append(f"{orphan} protocol lines before the first banner (captured with --no-reset?)")
    if pre.malformed:
        rep.info.append(f"{len(pre.malformed)} malformed-looking line(s) before the banner (boot noise)")
    rep.add("reboots", "FAIL" if (bad and not args.allow_reboots) else ("WARN" if bad else "PASS"),
            "; ".join(bad) if bad else "one boot banner")
    for i, seg in enumerate(segs):
        check_segment(seg, rep, args, f"[boot {i + 1}] " if len(segs) > 1 else "")
    return rep


def print_report(rep: Report, path, out=sys.stdout):
    w = out.write
    w(f"replay: {path}\n")
    for x in rep.info:
        w(f"  {x}\n")
    for x in rep.table:
        w(f"  {x}\n")
    w("  checks:\n")
    for name, st, detail in rep.checks:
        w(f"    {st:<7} {name:<16} {detail}\n")
    f = rep.failed()
    w(f"  RESULT: {'FAIL (' + ', '.join(n for n, _, _ in f) + ')' if f else 'PASS'}\n")


# --------------------------------------------------------------------------
# Self-test
# --------------------------------------------------------------------------
LOGGER_LOGS = {   # file -> expected P count; (a) must PASS on every one
    "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv": 428,
    "firmware/opentread/test/data/2026-10-18_1416_S1b_10rev.csv": 61,
    "firmware/opentread/test/data/2026-10-18_1323_S1_handturn.csv": 164,
    "firmware/opentread/test/data/2026-10-18_1315_bench_jumper.csv": 11,
    "firmware/opentread/test/data/2026-10-18_1320_S1_handturn.csv": 0,
}


def _ns(**kw):
    d = dict(expect_build=None, require_release=False, allow_dev=False, ftms="0", allow_reboots=False,
             table_every=0)
    d.update(kw)
    return argparse.Namespace(**d)


def synth_log(src_rel="firmware/opentread/test/data/2026-10-18_1416_S1b_10rev.csv", **opt):
    """Synthetic OpenTread log from a real pulse-logger log's E/P/M lines:
    OpenTread banner, H every 5 s with true counters, K at 1 Hz from the oracle, G/D every 60 s. `opt`
    injects faults. Returns a list of lines."""
    src = read_lines(os.path.join(REPO, src_rel))
    _, segs = parse(src)
    seg = segs[0]
    t0 = seg.banner_t
    ev = []   # (t, order, line); order keeps E before its P and drains before H/K
    sim = opt.get("sim", False)
    if not sim:
        for t, lv in seg.edges:
            ev.append((t, 0, f"E,{t},{lv}"))
    for t, n, dt in seg.pulses:
        ev.append((t, 1, f"P,{t},{n},{dt}"))
    for t, n in seg.markers:
        ev.append((t, 1, f"M,{t},{n}"))
    end = seg.last_t + opt.get("tail_us", 6_000_000)
    met = oracle.Metrics(sorted(t for t, _, _ in seg.pulses))
    holds = opt.get("holds", [])                  # [(t, event)] I inputs (workout mode)
    for t, e in holds:
        ev.append((t, 1, f"I,{t},{e}"))
    k_times = list(range(t0 + 1_000_000, end + 1, 1_000_000))
    wk = None
    if opt.get("workout"):
        prm = oracle.WorkoutParams(auto_end_us=SHORT_AUTO_END_US if opt.get("short") else oracle.AUTO_END_US,
                                   summary_us=SHORT_SUMMARY_US if opt.get("short") else oracle.SUMMARY_US)
        wk = oracle.replay_workout(oracle.merge_inputs(sorted(t for t, _, _ in seg.pulses), holds), prm,
                                   end=end, probes=k_times)
        snaps = dict(wk.probes)
        res = dict(wk.hold_results)
        ev = [(t, o, (f"{ln},{res[t]}" if ln.startswith("I,") and ln.endswith(",hold_complete") else ln))
              for t, o, ln in ev]
        for o in wk.out:
            if o[0] == "w":
                ev.append((o[1], 1, f"W,{o[1]},{o[2]},{o[3]},{o[4]}"))
            else:
                ev.append((o[1], 1, "Z," + ",".join(str(x) for x in o[1:])))
    for t in k_times:
        sp = met.speed(t) + (opt.get("k_bias", 0) if t == t0 + 1_000_000 * opt.get("k_bias_at", 30) else 0)
        odo = met.odometer(t) + (1 if opt.get("odo_bias") and t == t0 + 30_000_000 else 0)
        if wk is None:
            ev.append((t, 2, f"K,{t},-,{odo},0,{sp},0,0,0"))
        else:
            sn = snaps[t]
            mv, tot = sn["moving"] // 1000, sn["total"] // 1000
            if opt.get("k_mov_gt_tot") and t == opt["k_mov_gt_tot"]:
                mv = tot + 1
            if opt.get("k_tot_dec") and t == opt["k_tot_dec"]:
                tot, mv = tot - 2000, min(mv, tot - 2000)
            ev.append((t, 2, f"K,{t},{sn['state']},{odo},{sn['wk']},{sp},{mv},{tot},{sn['top']}"))
        if opt.get("ble"):
            k_wk = 0 if wk is None else snaps[t]["wk"]
            k_tot = 0 if wk is None else snaps[t]["total"] // 1000
            r = bytearray(rsc_expected(sp, odo))
            x = bytearray(td_expected(sp, k_wk, k_tot))
            if opt.get("r_bad") and t == t0 + 30_000_000:
                r[1] ^= 0x01
            if opt.get("x_bad") and t == t0 + 30_000_000:
                x[7] ^= 0x01
            ev.append((t, 2, f"R,{t},{r.hex()}"))
            if opt.get("ftms", 0):
                ev.append((t, 2, f"X,{t},{x.hex()}"))
    bl = opt.get("backlight")                     # ui;reset + L lines from the model, faults injected
    if bl:
        dim, off = bl.get("dim", 60_000_000), bl.get("off", 300_000_000)
        rt = t0 + 100_000
        ev.append((rt, 1, f"S,{rt},ui;reset dim_us={dim} off_us={off} bl_full=255 bl_dim=30 bl_off=0 short=0"))
        wl = [(o[1], norm_state(o[2]), norm_state(o[3]), o[4]) for o in wk.out if o[0] == "w"] if wk else []
        acts = sorted([t for t, _, _ in seg.pulses] + [t for t, e in holds if e == "touch_down"])
        lv = {"full": 255, "dim": 30, "off": 0}
        for k, (t, level, _, grp) in enumerate(backlight_model(rt, dim, off, acts, wl, end)):
            if grp:
                continue                          # optional (learn-window) transitions: not shown
            if k == bl.get("drop_at", -1):
                continue
            d = bl.get("late", 2_000) + (bl.get("extra_late", 0) if k == bl.get("late_at", -1) else 0)
            ev.append((t + d, 1, f"L,{t + d},{lv[level]}"))
        if bl.get("bogus"):
            ev.append((end - 1000, 1, f"L,{end - 1000},128"))
        for t, v in bl.get("extra", []):
            ev.append((t, 1, f"L,{t},{v}"))
    nv = opt.get("nvs")                           # N lines (read at boot, commits/defers as given)
    if nv:
        ev.append((t0 + 50_000, 1, f"N,{t0 + 50_000},read,schema,none"))
        ev.append((t0 + 50_000, 1, f"N,{t0 + 50_000},read,units,none"))
        ev.append((t0 + 50_000, 1, f"N,{t0 + 50_000},default,units,0"))
        for t in nv.get("commits", []):
            ev.append((t, 1, f"N,{t},commit,units,1"))
        for t in nv.get("defers", []):
            ev.append((t, 1, f"N,{t},defer,units,1"))
        for t, kind in nv.get("cmds", []):
            ev.append((t, 1, f"S,{t},{kind}"))
    t = t0 + 5_000_000
    while t <= end:
        pc = sum(1 for p in seg.pulses if p[0] <= t)
        ec = 0 if sim else sum(1 for e in seg.edges if e[0] <= t)
        dd = 1 if (opt.get("debug_drop") and t >= t0 + 20_000_000) else 0
        de = 1 if (opt.get("edge_drop") and t >= t0 + 20_000_000) else 0
        ev.append((t, 3, f"H,{t},{pc},{ec},{de},0,{dd},0,1"))
        t += 5_000_000
    for t in range(t0 + 60_000_000, end + 1, 60_000_000):
        ev.append((t, 3, f"D,{t},200000,190000,110000,100000"))
        ev.append((t, 3, f"G,{t},1500,900,0,0,0" + (",31000,1" if opt.get("g_nvs") else "")
                   + (",5" if opt.get("g_bad") else "")))
    ev.sort(key=lambda e: (e[0], e[1]))
    banner = (f"S,{t0},OpenTread fw=1.0.0 build=1a2b3c4d dirty=0 pin=27 lockout_us=20000 "
              f"release_us=5000 baud=115200 line_level=1 ring=1024 marker_pin=0 "
              f"reset_reason={opt.get('reset_reason', 1)} isr_iram=1 bt_addr=n/a lgfx=n/a nimble=n/a "
              f"sim={1 if sim else 0} dev={1 if opt.get('holds') else 0} short={1 if opt.get('short') else 0} "
              f"ftms={opt.get('ftms', 0)} loopback=0 bringup=0 "
              f"verb={opt.get('verb', 2)}")
    lines = ["# tools/capture.py synthetic", "ets Jul 29 2019 12:21:46", "rst:0x1 (POWERON_RESET),boot:0x13",
             banner]
    body = [e[2] for e in ev]
    if opt.get("drop_e"):
        # the first E line whose removal leaves the re-filtered P list unchanged
        # (the synthetic banner says line_level=1, so the checker filters from (t0, 1))
        dev_p = sorted(t for t, _, _ in seg.pulses)
        e_idx = [i for i, ln in enumerate(body) if ln.startswith("E,")]
        e_all = [(int(body[i].split(",")[1]), int(body[i].split(",")[2])) for i in e_idx]
        for k, idx in enumerate(e_idx):
            if oracle.filter_edges(t0, 1, e_all[:k] + e_all[k + 1:]) == dev_p:
                del body[idx]
                break
        else:
            raise AssertionError("drop_e: no invisible E line found")
    if opt.get("drop_p"):
        idx = [i for i, ln in enumerate(body) if ln.startswith("P,")][10]
        del body[idx]
    if opt.get("verb_change"):
        body.insert(len(body) // 2, f"S,{ev[len(ev) // 2][0]},verbosity;1")
    if opt.get("malformed"):
        body.insert(len(body) // 2, "P,123,x,4")
    if opt.get("other"):
        body.insert(len(body) // 3, f"Q,{ev[len(ev) // 3][0]},something;else,1")
        body.insert(len(body) // 3, f"B,{ev[len(ev) // 3][0]},connect,0,x")
    if opt.get("reboot"):
        body.insert(len(body) // 2, banner.replace(f"S,{t0},", "S,100,"))
    if opt.get("cp"):
        # one CP exchange per 10 s: write, indication 6 ms later, ack 30 ms later;
        # faults: cp_double (a second cp_ind before the ack), cp_early (1 ms deferral)
        tc = t0 + 2_000_000
        cpl = []
        while tc < end - 1_000_000:
            cpl += [f"B,{tc},cp_write,1,00", f"B,{tc + (1000 if opt.get('cp_early') else 6000)},cp_ind,1,800001"]
            if opt.get("cp_double") and tc == t0 + 12_000_000:
                cpl.append(f"B,{tc + 7000},cp_ind,1,800001")
            cpl.append(f"B,{tc + 36000},cp_ack,1,code=14")
            tc += 10_000_000
        cpl += [f"B,{end - 500_000},disconnect,1,reason=19;pending_drop=0;token=0"]
        merged = []
        for ln in body + cpl:
            merged.append(ln)
        body = sorted(merged, key=lambda ln: int(ln.split(",")[1]))
    if opt.get("b_unknown"):
        body.insert(len(body) // 2, f"B,{ev[len(ev) // 2][0]},zap,1,x")
    for tag in ("w_shift", "w_drop", "z_bump", "i_drop", "hold_arg"):
        if not opt.get(tag):
            continue
        letter = {"w_shift": "W", "w_drop": "W", "z_bump": "Z", "i_drop": "I", "hold_arg": "I"}[tag]
        idx = [i for i, ln in enumerate(body) if ln.startswith(letter + ",")
               and (tag != "hold_arg" or ",hold_complete," in ln)][opt[tag] - 1]
        f = body[idx].split(",")
        if tag == "w_shift":
            f[1] = str(int(f[1]) + 1)
            body[idx] = ",".join(f)
        elif tag == "z_bump":
            f[4] = str(int(f[4]) + 1)            # moving_us + 1
            body[idx] = ",".join(f)
        elif tag == "hold_arg":
            f[3] = "void"
            body[idx] = ",".join(f)
        else:
            del body[idx]
    if opt.get("w_lower"):                        # state spelling variants are accepted
        body = [ln.replace(",WARMUP,", ",warm_up,") if ln.startswith("W,") else ln for ln in body]
    if opt.get("cut"):
        # capture ends at cut_at (a final S line marks it); W/Z lines stamped
        # after drop_after were "not yet printed" when the capture stopped
        cut_at, drop_after = opt["cut"]
        body = [ln for ln in body if int(ln.split(",")[1]) <= cut_at
                and not (ln[0] in "WZ" and int(ln.split(",")[1]) > drop_after)]
        if not opt.get("cut_no_marker"):
            body.append(f"S,{cut_at},cmd;d")
    lines += body
    if opt.get("crlf"):
        lines = [ln + "\r" for ln in lines]
    return lines


def _k_fault(src_rel, holds, tag):
    """A synthetic workout log with one K line faulted at the first K instant
    inside the first workout's RUNNING state (found from the oracle)."""
    lines = synth_log(src_rel, workout=True, holds=holds)
    _, segs = parse(lines)
    t = next(k[0] for k in segs[0].k if k[1] == oracle.RUNNING and k[6] > 5000)
    return synth_log(src_rel, workout=True, holds=holds, **{tag: t})


def self_test() -> int:
    fails = 0

    def expect(name, rep, want: dict, overall):
        nonlocal fails
        got = {k: rep.status(k) for k in want}
        ok = got == want and (not rep.failed()) == overall
        fails += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'FAIL'} {name}: " + ", ".join(f"{k}={v}" for k, v in got.items())
              + f", overall={'PASS' if not rep.failed() else 'FAIL'}"
              + ("" if ok else f"   (wanted {want}, overall={'PASS' if overall else 'FAIL'})"))
        if not ok:
            print_report(rep, name)

    print("replay self-test: pulse-logger logs ((a) must pass, P counts must match test 1.1)")
    for rel, n in LOGGER_LOGS.items():
        rep = analyse(read_lines(os.path.join(REPO, rel)), _ns())
        _, segs = parse(read_lines(os.path.join(REPO, rel)))
        count_ok = len(segs) == 1 and len(segs[0].pulses) == n
        if not count_ok:
            fails += 1
            print(f"  FAIL {rel}: expected {n} P lines")
        expect(os.path.basename(rel), rep,
               {"a_refilter": "PASS", "precondition": "PASS", "h_pulses": "PASS", "h_edges": "PASS",
                "p_sequence": "PASS", "b_oracle_speed": "SKIP", "edge_drops": "PASS"}, True)

    print("replay self-test: synthetic OpenTread logs (built from S1b and S2 E/P lines)")
    base = {"a_refilter": "PASS", "b_oracle_speed": "PASS", "b_oracle_odo": "PASS", "precondition": "PASS"}
    expect("clean S1b", analyse(synth_log(), _ns()), base, True)
    expect("clean S2", analyse(synth_log("firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv"), _ns()), base, True)
    expect("clean + release/build", analyse(synth_log(), _ns(expect_build="1a2b3c4d", require_release=True)),
           dict(base, build="PASS", release_flags="PASS"), True)
    expect("wrong build id", analyse(synth_log(), _ns(expect_build="deadbeef")), {"build": "FAIL"}, False)
    expect("CRLF line ends", analyse(synth_log(crlf=True), _ns()), base, True)
    expect("unknown letters", analyse(synth_log(other=True), _ns()), base, True)
    expect("G with nvs_pass fields (7)", analyse(synth_log(g_nvs=True), _ns()), base, True)
    expect("G with 6 fields (malformed)", analyse(synth_log(g_bad=True), _ns()),
           {"precondition": "FAIL", "a_refilter": "INVALID"}, False)
    expect("K +1 mm/s (inside tolerance)", analyse(synth_log(k_bias=1), _ns()), base, True)
    expect("K +2 mm/s", analyse(synth_log(k_bias=2), _ns()), dict(base, b_oracle_speed="FAIL"), False)
    expect("K +2 mm/s at zero speed", analyse(synth_log(k_bias=2, k_bias_at=2), _ns()),
           dict(base, b_oracle_speed="FAIL"), False)
    expect("K odo +1", analyse(synth_log(odo_bias=True), _ns()), dict(base, b_oracle_odo="FAIL"), False)
    expect("one P line lost", analyse(synth_log(drop_p=True), _ns()),
           {"a_refilter": "FAIL", "h_pulses": "FAIL", "p_sequence": "FAIL"}, False)
    expect("debug_drop > 0", analyse(synth_log(debug_drop=True), _ns()),
           {"precondition": "FAIL", "a_refilter": "INVALID", "b_oracle_speed": "INVALID"}, False)
    expect("banner verb=1", analyse(synth_log(verb=1), _ns()),
           {"precondition": "FAIL", "a_refilter": "INVALID"}, False)
    expect("verbosity change", analyse(synth_log(verb_change=True), _ns()),
           {"precondition": "FAIL", "a_refilter": "INVALID"}, False)
    expect("malformed line", analyse(synth_log(malformed=True), _ns()),
           {"precondition": "FAIL", "a_refilter": "INVALID"}, False)
    expect("edge drops", analyse(synth_log(edge_drop=True), _ns()), dict(base, edge_drops="FAIL"), False)
    expect("one E line lost (re-filter blind to it)", analyse(synth_log(drop_e=True), _ns()),
           dict(base, h_edges="FAIL", h_pulses="PASS"), False)
    expect("one E line lost + debug_drop", analyse(synth_log(drop_e=True, debug_drop=True), _ns()),
           {"precondition": "FAIL", "h_edges": "INVALID"}, False)
    expect("reset_reason=4", analyse(synth_log(reset_reason=4), _ns()), dict(base, reset_reason="FAIL"), False)
    for rr in (0, 2, 8, 12, 15):
        expect(f"reset_reason={rr}", analyse(synth_log(reset_reason=rr), _ns()),
               dict(base, reset_reason="FAIL"), False)
    expect("reset_reason=3", analyse(synth_log(reset_reason=3), _ns()), dict(base, reset_reason="PASS"), True)
    expect("reboot mid-session", analyse(synth_log(reboot=True), _ns()), {"reboots": "FAIL"}, False)
    expect("sim build (no E)", analyse(synth_log(sim=True), _ns()),
           {"a_refilter": "SKIP", "b_oracle_speed": "PASS", "b_oracle_odo": "PASS"}, True)
    expect("sim flag + release required", analyse(synth_log(sim=True), _ns(require_release=True)),
           {"release_flags": "FAIL"}, False)
    expect("no workout lines: workout checks skipped", analyse(synth_log(), _ns()),
           {"w_oracle": "SKIP", "z_oracle": "SKIP", "hold_result": "SKIP", "k_workout": "SKIP"}, True)

    print("replay self-test: synthetic workout logs (W/I/Z/K from the oracle workout model, faults injected)")
    s2 = "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv"
    hold = [(1_105_000_000, "hold_begin"), (1_107_000_000, "hold_complete")]
    wbase = dict(base, w_oracle="PASS", z_oracle="PASS", hold_result="PASS", k_workout="PASS")
    expect("S2 + hold 1105/1107", analyse(synth_log(s2, workout=True, holds=hold), _ns()), wbase, True)
    expect("S2 to auto-end + summary timeout", analyse(synth_log(s2, workout=True, tail_us=1_100_000_000), _ns()),
           wbase, True)
    expect("S1b short timers (AUTO_END 30 s)", analyse(synth_log(workout=True, short=True, tail_us=60_000_000), _ns()),
           wbase, True)
    expect("short-timer log but banner short=0",
           analyse([ln.replace(" short=1 ", " short=0 ") for ln in
                    synth_log(workout=True, short=True, tail_us=60_000_000)], _ns()),
           dict(base, w_oracle="FAIL", z_oracle="FAIL"), False)
    expect("W state spelled warm_up", analyse(synth_log(s2, workout=True, holds=hold, w_lower=True), _ns()),
           wbase, True)
    # S1b: the oracle pauses at 70330484 (last pulse 67830484 + STOP)
    expect("capture ends 0.5 s after a pause, W not yet printed",
           analyse(synth_log(workout=True, cut=(70_830_484, 70_000_000)), _ns()), wbase, True)
    # The log ENDS on the W pause line (no line after it; last_t = the
    # pause stamp = last + STOP, due 1 us later): must pass, not report an extra W.
    expect("log ends exactly on a W pause line",
           analyse(synth_log(workout=True, cut=(70_330_484, 10 ** 12), cut_no_marker=True), _ns()), wbase, True)
    expect("capture ends 1.5 s after a pause, W missing",
           analyse(synth_log(workout=True, cut=(71_830_484, 70_000_000)), _ns()),
           dict(wbase, w_oracle="FAIL"), False)
    expect("W timestamp +1 us", analyse(synth_log(s2, workout=True, holds=hold, w_shift=3), _ns()),
           dict(wbase, w_oracle="FAIL"), False)
    expect("W line missing", analyse(synth_log(s2, workout=True, holds=hold, w_drop=2), _ns()),
           dict(wbase, w_oracle="FAIL"), False)
    expect("Z moving +1 us", analyse(synth_log(s2, workout=True, holds=hold, z_bump=1), _ns()),
           dict(wbase, z_oracle="FAIL"), False)
    expect("I hold_begin missing", analyse(synth_log(s2, workout=True, holds=hold, i_drop=1), _ns()),
           dict(wbase, w_oracle="FAIL", z_oracle="FAIL", hold_result="FAIL"), False)
    expect("hold_complete arg wrong", analyse(synth_log(s2, workout=True, holds=hold, hold_arg=1), _ns()),
           dict(wbase, hold_result="FAIL"), False)
    expect("K moving > total", analyse(_k_fault(s2, hold, "k_mov_gt_tot"), _ns()),
           dict(wbase, k_workout="FAIL"), False)
    expect("K total decreases in a workout", analyse(_k_fault(s2, hold, "k_tot_dec"), _ns()),
           dict(wbase, k_workout="FAIL"), False)
    # --allow-dev (a dev=1 build; every other release flag still required).
    held = synth_log(s2, workout=True, holds=hold)               # banner dev=1 (hold commands)
    expect("dev=1 + release required", analyse(held, _ns(require_release=True)),
           dict(wbase, release_flags="FAIL"), False)
    expect("dev=1 + release + --allow-dev", analyse(held, _ns(require_release=True, allow_dev=True)),
           dict(wbase, release_flags="PASS"), True)
    expect("dev=0 + release + --allow-dev", analyse(synth_log(), _ns(require_release=True, allow_dev=True)),
           dict(base, release_flags="PASS"), True)
    expect("sim=1 dev=1 + --allow-dev", analyse(synth_log(s2, workout=True, holds=hold, sim=True),
                                                _ns(require_release=True, allow_dev=True)),
           {"release_flags": "FAIL"}, False)
    for flag in ("short", "loopback", "bringup", "dirty"):
        bad = [ln.replace(f" {flag}=0", f" {flag}=1") if "OpenTread fw=" in ln else ln for ln in held]
        expect(f"{flag}=1 + --allow-dev", analyse(bad, _ns(require_release=True, allow_dev=True)),
               {"release_flags": "FAIL"}, False)
    expect("ftms=1 without --ftms 1", analyse(synth_log(ftms=1), _ns(require_release=True)),
           {"release_flags": "FAIL"}, False)
    expect("ftms=1 with --ftms 1", analyse(synth_log(ftms=1), _ns(require_release=True, ftms="1")),
           {"release_flags": "PASS"}, True)
    expect("ftms=0 with --ftms 1", analyse(synth_log(), _ns(require_release=True, ftms="1")),
           {"release_flags": "FAIL"}, False)

    print("replay self-test: synthetic backlight/NVS logs (L from the backlight model, N lines; faults injected)")
    expect("no L/N lines: L/N checks skipped", analyse(synth_log(), _ns()),
           {"l_backlight": "SKIP", "n_nvs": "SKIP"}, True)
    # S1b (pulses ~0-68 s) with short timers: auto-end at 97.8 s (OFF before it at 15 s
    # idle), summary to 109.8 s, then DIM/OFF from the clear.
    sbl = dict(dim=6_000_000, off=15_000_000)
    blbase = dict(wbase, l_backlight="PASS")
    expect("S1b short timers + backlight", analyse(synth_log(workout=True, short=True, tail_us=80_000_000,
                                                            backlight=sbl), _ns()), blbase, True)
    expect("S2 + hold + touch wake + backlight (production timers)",
           analyse(synth_log(s2, workout=True, holds=hold + [(1_000_000_000, "touch_down")],
                            tail_us=600_000_000, backlight=dict(dim=60_000_000)), _ns()), blbase, True)
    expect("L 45 ms late (inside +-50 ms)",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000, backlight=dict(sbl, late=45_000)), _ns()),
           blbase, True)
    expect("a timer L 60 ms late",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000,
                            backlight=dict(sbl, late_at=0, extra_late=58_000)), _ns()),
           dict(wbase, l_backlight="FAIL"), False)
    expect("an L line missing",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000, backlight=dict(sbl, drop_at=1)), _ns()),
           dict(wbase, l_backlight="FAIL"), False)
    expect("an L level 128",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000, backlight=dict(sbl, bogus=True)), _ns()),
           dict(wbase, l_backlight="FAIL"), False)
    expect("L lines but banner says longer timers",
           analyse([ln.replace("dim_us=6000000", "dim_us=7000000") for ln in
                    synth_log(workout=True, short=True, tail_us=80_000_000, backlight=sbl)], _ns()),
           dict(wbase, l_backlight="FAIL"), False)
    # Learn window: a touch DOWN stamped 0.2 ms before the DIM instant (73830484) is only
    # known ~40 ms later, so the device may dim and wake again (or not dim at all).
    tlearn = [(73_830_284, "touch_down")]
    expect("touch just before DIM: no DIM shown",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000, holds=tlearn, backlight=sbl), _ns()),
           blbase, True)
    expect("touch just before DIM: DIM then wake shown",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000, holds=tlearn,
                            backlight=dict(sbl, extra=[(73_830_500, 30), (73_870_400, 255)])), _ns()),
           blbase, True)
    expect("touch just before DIM: DIM shown, no wake",
           analyse(synth_log(workout=True, short=True, tail_us=80_000_000, holds=tlearn,
                            backlight=dict(sbl, extra=[(73_830_500, 30)])), _ns()),
           dict(wbase, l_backlight="FAIL"), False)
    # NVS: S1b moves until ~67.8 s; a toggle at 75 s commits at >= 78 s (stationary).
    tog = [(75_000_000, "units_toggle")]
    nbase = dict(wbase, n_nvs="PASS")
    expect("N: toggle -> commit 3 s later, stationary",
           analyse(synth_log(workout=True, holds=tog, tail_us=30_000_000, nvs=dict(commits=[78_000_000])), _ns()),
           nbase, True)
    expect("N: commit 2.9 s after the toggle",
           analyse(synth_log(workout=True, holds=tog, tail_us=30_000_000, nvs=dict(commits=[77_900_000])), _ns()),
           dict(wbase, n_nvs="FAIL"), False)
    expect("N: commit while moving",
           analyse(synth_log(workout=True, holds=[(30_000_000, "units_toggle")], tail_us=30_000_000,
                            nvs=dict(commits=[33_000_000])), _ns()), dict(wbase, n_nvs="FAIL"), False)
    expect("N: commit at boot (no toggle)",
           analyse(synth_log(workout=True, tail_us=30_000_000, nvs=dict(commits=[75_000_000])), _ns()),
           dict(wbase, n_nvs="FAIL"), False)
    expect("N: two defers for one toggle",
           analyse(synth_log(workout=True, holds=[(30_000_000, "units_toggle")], tail_us=30_000_000,
                            nvs=dict(defers=[33_000_000, 40_000_000], commits=[70_400_000])), _ns()),
           dict(wbase, n_nvs="FAIL"), False)
    expect("N: one defer, commit after the belt stops",
           analyse(synth_log(workout=True, holds=[(30_000_000, "units_toggle")], tail_us=30_000_000,
                            nvs=dict(defers=[33_000_000], commits=[70_400_000])), _ns()), nbase, True)
    expect("N: nw<k> forced commits are exempt",
           analyse(synth_log(workout=True, tail_us=30_000_000,
                            nvs=dict(commits=[20_000_000, 20_001_000, 20_002_000],
                                     cmds=[(19_999_000, "cmd;nw3"), (20_003_000, "nvs;forced_done n=3")])), _ns()),
           nbase, True)
    print("replay self-test: synthetic BLE logs (R/X lines from the K values, CP B lines, faults injected)")
    bb = dict(wbase, ble_payload="PASS", ble_cp="PASS")
    expect("S2 + hold, R/X + CP", analyse(synth_log(s2, workout=True, holds=hold, ble=True, ftms=1, cp=True), _ns()),
           bb, True)
    expect("no R/X lines: BLE checks skipped", analyse(synth_log(), _ns()),
           {"ble_payload": "SKIP", "ble_cp": "SKIP"}, True)
    expect("ftms=0: R only", analyse(synth_log(s2, workout=True, holds=hold, ble=True), _ns()),
           dict(wbase, ble_payload="PASS", ble_cp="SKIP"), True)
    expect("R speed byte flipped", analyse(synth_log(s2, workout=True, holds=hold, ble=True, ftms=1, r_bad=True), _ns()),
           dict(wbase, ble_payload="FAIL"), False)
    expect("X elapsed byte flipped", analyse(synth_log(s2, workout=True, holds=hold, ble=True, ftms=1, x_bad=True),
                                             _ns()), dict(wbase, ble_payload="FAIL"), False)
    xs = [ln for ln in synth_log(s2, workout=True, holds=hold, ble=True, ftms=1) if not ln.startswith("X,")]
    expect("ftms=1 but no X lines", analyse(xs, _ns()), dict(wbase, ble_payload="FAIL"), False)
    expect("two CP indications in flight", analyse(synth_log(s2, workout=True, ble=True, ftms=1, cp=True,
                                                            cp_double=True), _ns()), {"ble_cp": "FAIL"}, False)
    expect("CP indication 1 ms after the write", analyse(synth_log(s2, workout=True, ble=True, ftms=1, cp=True,
                                                                  cp_early=True), _ns()), {"ble_cp": "FAIL"}, False)
    expect("unknown B event", analyse(synth_log(b_unknown=True), _ns()),
           {"precondition": "FAIL"}, False)
    print(f"replay self-test: {'PASS' if fails == 0 else f'FAIL ({fails})'}")
    return 0 if fails == 0 else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Offline checks on a captured OpenTread (or pulse-logger) serial log.")
    ap.add_argument("log", nargs="?", help="captured log (logs/<date>_<time>_<session>.csv)")
    ap.add_argument("--expect-build", metavar="ID8", help="banner build= must equal this")
    ap.add_argument("--require-release", action="store_true",
                    help="fail unless the banner shows the release flag set")
    ap.add_argument("--allow-dev", action="store_true",
                    help="with --require-release: permit dev=1 (a build driven by serial hold commands); every "
                         "other release flag is still required")
    ap.add_argument("--ftms", choices=("0", "1"), default="1",
                    help="with --require-release: the banner ftms= the build must show (1 = FTMS build, "
                         "0 = RSC only); default 1 (the release build)")
    ap.add_argument("--allow-reboots", action="store_true", help="a second banner is a WARN, not a FAIL")
    ap.add_argument("--table-every", type=float, default=10.0, metavar="S",
                    help="(c) table: one K row every S seconds (0 = off, 1 = every K); default 10")
    ap.add_argument("--self-test", action="store_true", help="run the checker's self-test and exit")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    if not args.log:
        ap.error("a log file is required (or --self-test)")
    if args.expect_build and not re.fullmatch(r"[0-9a-f]{8}", args.expect_build):
        ap.error("--expect-build takes 8 lowercase hex digits")
    if args.allow_dev and not args.require_release:
        ap.error("--allow-dev only modifies --require-release")
    try:
        lines = read_lines(args.log)
    except OSError as e:
        print(f"replay: {e}", file=sys.stderr)
        return 2
    rep = analyse(lines, args)
    print_report(rep, args.log)
    return 1 if rep.failed() else 0


if __name__ == "__main__":
    sys.exit(main())
