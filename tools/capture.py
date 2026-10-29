#!/usr/bin/env python3
"""
Serial capture tool for the OpenTread head unit (firmware/opentread). It also
reads logs from the original pulse logger, which used the same line formats.

Reads the device's serial stream (reed-switch edges + accepted pulses + status
and heartbeat lines, plus raw FTMS packets from the original pulse logger, all
stamped on ONE ESP32 microsecond clock), writes it
VERBATIM to logs/YYYY-MM-DD_HHMM_<session>.csv in the repo root, and shows a
live status line so you can see the capture is healthy while you run a
session. All real analysis happens offline (e.g. tools/replay.py) from the
saved file.

Line format produced by the firmware (one event per line, CSV, 2nd field is the
ESP timestamp in microseconds from esp_timer_get_time()):

    E,<t_us>,<level>            raw edge (0 = closed/magnet, 1 = open)
    P,<t_us>,<n>,<dt_us>        accepted pulse, running count, dt since previous
    F,<t_us>,<hex>              FTMS Treadmill Data (0x2ACD) raw bytes as hex
    M,<t_us>,<n>                marker n (BOOT button)
    S,<t_us>,<text>             status text (everything after the 2nd comma)
    H,<t_us>,<pulses>,<edges>,<dropped_edges>,<ftms_pkts>,<dropped_ftms>,<connected>,<line_level>
                                heartbeat every 5 s

OpenTread head unit (banner "OpenTread fw=..."): same E/P/M/S/H formats. In
its H line field 5 is always 0 and field 6 is debug_drop (debug lines dropped
because the serial TX buffer was full), labelled as such here. Its other
letters (K, D, G, W, I, Z, B, R, X, T, L, N) are not decoded: any
well-formed "<Letter>,<int>[,...]" line with an unknown letter is counted as
"other", not as a malformed line.

Usage:
    .venv/bin/python tools/capture.py --port /dev/cu.usbserial-110 --session my_run
    .venv/bin/python tools/capture.py --session S3_steptest      # port auto-detected
    .venv/bin/python tools/capture.py --session S1_handturn --no-reset
    .venv/bin/python tools/capture.py --replay logs/2026-10-18_1412_S3_steptest.csv
    .venv/bin/python tools/capture.py --replay FILE --replay-speed 1   # real-time

Only ONE process may hold the serial port: close the Arduino serial
monitor / other readers first.

Sending commands (head unit `d`, `v`, `s10`, `g10,600`, ...): type the command in
this terminal and press Enter. Each line typed on stdin is written to the
port followed by '\n' (the firmware echoes it as `S,t,cmd;<text>` in the
log, which stays a verbatim copy of the port output). So never open a
second reader/monitor on the port to type commands.

Port auto-detect (no --port): a CH340 (USB VID 0x1A86, the CYD) is chosen
when exactly one is plugged in; otherwise the tool refuses to guess when the
candidates are more than one board. With a pulse_gen / dev board on USB as
well (CP2102, it shows up as /dev/cu.usbserial-0001 AND
/dev/cu.SLAB_USBtoUART), pass --port explicitly.

DESIGN NOTES:
 1. THE FILE IS THE PRODUCT. Bytes are written to the log exactly as they arrive
    from the port (raw chunks, before any decoding), so garbage, boot-ROM noise,
    torn lines and \\r\\n endings are all preserved. Parsing/display can never
    alter or drop logged data. The only line we add is the leading '#' header.
 2. FLUSHED EVERY 0.5 s (plus fsync every 5 s), so a crash or USB unplug loses
    under a second of data. The file is opened exclusive-create: an existing
    log is never overwritten (a _2, _3... suffix is added instead).
 3. THE PARSER NEVER RAISES. Every line goes through a parse that returns None
    on anything unexpected (wrong type letter, non-int fields, odd hex, bad
    UTF-8). Malformed lines are counted; before the first S line they are
    assumed to be boot noise (74880-baud ROM output read at 115200) and counted
    silently, after that each one is reported (rate-limited).
 4. FTMS DECODE IS FLAGS-DRIVEN and bounds-checked, ported from the validated
    FTMS Treadmill Data decoder (Bluetooth SIG FTMS spec), with sentinels masked. The
    expected packet length is computed from the flags; a length mismatch or an
    unexpected flags value (anything but 0x14a6) is warned about once per
    distinct value and the decode is shown with a '?' so it is not trusted.
 5. INTEGRITY CHECKS on the stream: P running-count gaps (lost serial lines),
    ESP reboots, and increases in the firmware's dropped_edges / dropped_ftms
    counters from H lines are all printed as warnings the moment they happen,
    and totalled in the summary (drop totals are summed across reboots). A
    REBOOT is only declared on real evidence: the boot banner ("OpenTread
    fw=", or "M4 pulse logger" from the original pulse logger) after earlier
    lines, the P count restarting at 1, or the clock jumping back more than REBOOT_BACKSTEP_S. Smaller backward steps are just
    out-of-order lines (the firmware is merge-sorted but not perfectly) and
    are counted, never treated as a reboot.
 6. RESET without entering the bootloader: DTR/RTS are deasserted BEFORE the
    port is opened (so opening does not glitch EN/IO0), then the standard
    sequence dtr=False, rts=True, 0.1 s, rts=False pulses EN with IO0 high
    -> normal boot, and the S boot banner lands in the log.
 7. DISPLAY: on a TTY a single status line is redrawn in place ~1 Hz and
    S/M/warning messages scroll above it; when stdout is not a TTY (piped,
    replay tests) status lines are printed one per refresh instead.
 8. The live speed/distance estimate uses --m-per-pulse (default 0.5974 = 10 belt
    slats, a physical measurement, NOT a fitted value) and the mean of the last
    6 pulse intervals (~one belt revolution at ~6 magnets/rev). It is a sanity display only.
"""
import argparse
import collections
import datetime as dt
import os
import re
import select
import shutil
import signal
import struct
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # parent of tools/
LOG_DIR = os.path.join(REPO, "logs")

KMH_PER_MPH = 1.609344
EXPECTED_FLAGS = 0x14A6
SPEED_WINDOW = 6            # pulse intervals averaged for the speed estimate
PULSE_STALE_S = 3.0         # no pulse for this long (ESP time) -> speed 0
FTMS_STALE_S = 3.0          # FTMS packet older than this -> "(stale)"
FLUSH_EVERY_S = 0.5
REBOOT_BACKSTEP_S = 30.0    # clock going back more than this = ESP reboot
# Boot banners (reboot detection). "M4 pulse logger" is the original pulse
# logger's banner; keep it so older logs still parse.
BANNER_TAGS = ("M4 pulse logger", "OpenTread fw=")
OT_BANNER_TAG = "OpenTread fw="
FSYNC_EVERY_S = 5.0

# --------------------------------------------------------------------------
# FTMS Treadmill Data (0x2ACD) decoder — field order/flags per the Bluetooth
# SIG FTMS spec, bounds-checked.
# (bit, name, struct fmt sequence)
# --------------------------------------------------------------------------
_FIELDS = [
    # bit 0 is "More Data": inverted — speed present when bit 0 == 0.
    (0,  "speed",     ["<H"]),
    (1,  "avg_speed", ["<H"]),
    (2,  "distance",  ["u24"]),
    (3,  "incline",   ["<h", "<h"]),
    (4,  "elevation", ["<H", "<H"]),
    (5,  "pace",      ["<B"]),
    (6,  "avg_pace",  ["<B"]),
    (7,  "energy",    ["<H", "<H", "<B"]),
    (8,  "hr",        ["<B"]),
    (9,  "met",       ["<B"]),
    (10, "elapsed",   ["<H"]),
    (11, "remaining", ["<H"]),
    (12, "force_pwr", ["<h", "<h"]),
]
_SIZES = {"<H": 2, "<h": 2, "<B": 1, "u24": 3}


def _present(flags, bit):
    return not (flags & 1) if bit == 0 else bool(flags & (1 << bit))


def ftms_expected_len(flags):
    n = 2
    for bit, _name, fmts in _FIELDS:
        if _present(flags, bit):
            n += sum(_SIZES[f] for f in fmts)
    return n


def decode_treadmill_data(data):
    """Return a dict of decoded fields (sentinels masked to None).

    Never raises: on a short packet it stops and sets out['truncated']=True.
    """
    out = {"flags": None, "len": len(data), "truncated": False}
    if len(data) < 2:
        out["truncated"] = True
        return out
    flags = struct.unpack_from("<H", data, 0)[0]
    out["flags"] = flags
    out["expected_len"] = ftms_expected_len(flags)
    i = 2
    for bit, name, fmts in _FIELDS:
        if not _present(flags, bit):
            continue
        vals = []
        for f in fmts:
            size = _SIZES[f]
            if i + size > len(data):
                out["truncated"] = True
                return out
            if f == "u24":
                vals.append(int.from_bytes(data[i:i + 3], "little"))
            else:
                vals.append(struct.unpack_from(f, data, i)[0])
            i += size
        if name == "speed":
            out["speed_kmh"] = vals[0] / 100.0
        elif name == "avg_speed":
            out["avg_speed_kmh"] = vals[0] / 100.0
        elif name == "distance":
            out["distance_m"] = None if vals[0] == 0xFFFFFF else vals[0]
        elif name == "incline":
            out["incline_pct"] = None if vals[0] == 0x7FFF else vals[0] / 10.0
        elif name == "pace":
            out["pace"] = vals[0]
        elif name == "energy":
            out["kcal"] = None if vals[0] == 0xFFFF else vals[0]
            out["kcal_per_h"] = None if vals[1] == 0xFFFF else vals[1]
            out["kcal_per_min"] = None if vals[2] == 0xFF else vals[2]
        elif name == "hr":
            out["hr"] = vals[0]
        elif name == "elapsed":
            out["elapsed_s"] = vals[0]
        elif name == "force_pwr":
            out["force_n"] = None if vals[0] == 0x7FFF else vals[0]
            out["power_w"] = None if vals[1] == 0x7FFF else vals[1]
    return out


# --------------------------------------------------------------------------
# Line parser. Returns (type, t_us, payload) or None. Never raises.
# --------------------------------------------------------------------------
_HEX_RE = re.compile(r"^[0-9a-fA-F]*$")
_INT_RE = re.compile(r"^-?\d+$")
_KNOWN = frozenset("SEPMHF")
# Well-formed line of a letter this tool doesn't decode (head unit: K, D, G, W, ...).
_OTHER_RE = re.compile(r"^[A-Z],\d+(?:,.*)?$", re.ASCII)


def _ints(fields, n):
    if len(fields) != n or not all(_INT_RE.match(f) for f in fields):
        return None
    return [int(f) for f in fields]


def parse_line(text):
    text = text.strip("\r\n \t")
    if len(text) < 3 or text[1] != ",":
        return None
    kind = text[0]
    if kind not in _KNOWN:
        if _OTHER_RE.match(text):
            return ("other", int(text.split(",", 2)[1]), kind)
        return None
    if kind == "S":
        parts = text.split(",", 2)
        if len(parts) < 2 or not _INT_RE.match(parts[1]):
            return None
        return ("S", int(parts[1]), parts[2] if len(parts) == 3 else "")
    parts = text.split(",")
    if not _INT_RE.match(parts[1]):
        return None
    t_us = int(parts[1])
    rest = parts[2:]
    if kind == "E":
        v = _ints(rest, 1)
        return ("E", t_us, v[0]) if v and v[0] in (0, 1) else None
    if kind == "P":
        v = _ints(rest, 2)
        return ("P", t_us, tuple(v)) if v else None
    if kind == "M":
        v = _ints(rest, 1)
        return ("M", t_us, v[0]) if v else None
    if kind == "H":
        v = _ints(rest, 7)
        return ("H", t_us, tuple(v)) if v else None
    if kind == "F":
        if len(rest) != 1:
            return None
        h = rest[0]
        if not h or len(h) % 2 or not _HEX_RE.match(h):
            return None
        return ("F", t_us, bytes.fromhex(h))
    return None


# --------------------------------------------------------------------------
# Display
# --------------------------------------------------------------------------
class Display:
    def __init__(self):
        self.tty = sys.stdout.isatty()
        self.status = ""

    def message(self, msg):
        if self.tty:
            sys.stdout.write("\r\x1b[K" + msg + "\n")
            if self.status:
                sys.stdout.write(self.status)
        else:
            sys.stdout.write(msg + "\n")
        sys.stdout.flush()

    def update(self, line):
        if self.tty:
            width = shutil.get_terminal_size((120, 24)).columns
            self.status = line[: max(20, width - 1)]
            sys.stdout.write("\r\x1b[K" + self.status)
        else:
            sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def finish(self):
        if self.tty and self.status:
            sys.stdout.write("\n")
            sys.stdout.flush()
        self.status = ""


def fmt_hms(seconds):
    seconds = max(0, int(seconds))
    return f"{seconds // 3600:d}:{seconds // 60 % 60:02d}:{seconds % 60:02d}"


def fmt_esp(t_us):
    return f"{t_us / 1e6:10.3f}s"


# --------------------------------------------------------------------------
# Session state
# --------------------------------------------------------------------------
class Session:
    def __init__(self, display, m_per_pulse):
        self.d = display
        self.m_per_pulse = m_per_pulse
        self.lines = 0
        self.garbage = 0
        self.garbage_reported = 0
        self.seen_status = False
        self.edges = 0
        self.pulses = 0
        self.ftms = 0
        self.markers = 0
        self.status_lines = 0
        self.heartbeats = 0
        self.first_t = None
        self.last_t = None
        self.boot_first_t = None    # first timestamp of the current ESP boot
        self.esp_span_prev = 0.0    # ESP seconds covered by earlier boots
        self.out_of_order = 0
        self.max_lag_us = 0
        self.reboots = 0
        self.pulse_gaps = 0
        self.last_n = None
        self.last_pulse_t = None
        self.dts = collections.deque(maxlen=SPEED_WINDOW)
        self.last_ftms = None
        self.last_ftms_t = None
        self.first_ftms_dist = None
        self.first_ftms_pulses = None
        self.warned_flags = set()
        self.warned_lens = set()
        self.ftms_bad = 0
        self.last_h = None          # (pulses, edges, dE, ftms, dF, conn, level)
        self.connected = None
        self.max_dropped_edges = 0  # per boot (H counters reset on reboot)
        self.max_dropped_ftms = 0
        self.dropped_edges_prev = 0  # summed over earlier boots
        self.dropped_ftms_prev = 0
        self.head_unit = False      # current boot's banner is the OpenTread head unit's
        self.other = collections.Counter()   # well-formed lines with undecoded letters

    # -- helpers --
    def warn(self, msg):
        self.d.message(f"!! WARNING: {msg}")

    def _reboot(self, t_us, why):
        """New ESP boot: bank per-boot totals, reset per-boot state."""
        self.reboots += 1
        self.warn(f"ESP REBOOT ({why}) at ESP {t_us / 1e6:.3f}s "
                  f"(previous boot reached {self.last_t / 1e6:.3f}s) — "
                  f"per-boot counters reset")
        self.esp_span_prev += (self.last_t - self.boot_first_t) / 1e6
        self.dropped_edges_prev += self.max_dropped_edges
        self.dropped_ftms_prev += self.max_dropped_ftms
        self.max_dropped_edges = self.max_dropped_ftms = 0
        self.last_n = None
        self.last_pulse_t = None
        self.dts.clear()
        self.last_h = None
        self.first_ftms_dist = None
        self.boot_first_t = self.last_t = t_us

    def _clock(self, t_us, is_banner=False):
        if self.last_t is None:
            self.first_t = self.boot_first_t = self.last_t = t_us
            return
        if is_banner:
            self._reboot(t_us, "boot banner")
        elif t_us + REBOOT_BACKSTEP_S * 1e6 < self.last_t:
            self._reboot(t_us, f"clock went back {(self.last_t - t_us) / 1e6:.1f}s")
        elif t_us < self.last_t:
            self.out_of_order += 1   # merge-order jitter, not a reboot
            self.max_lag_us = max(self.max_lag_us, self.last_t - t_us)
        else:
            self.last_t = t_us

    # -- feed one decoded text line --
    def feed(self, text):
        self.lines += 1
        if not text.strip():
            return
        rec = parse_line(text)
        if rec is None:
            self.garbage += 1
            if self.seen_status:
                self.garbage_reported += 1
                n = self.garbage_reported
                if n <= 5 or n % 100 == 0:
                    shown = text.strip()[:80].encode("unicode_escape").decode()
                    self.warn(f"malformed line #{n}: {shown!r}")
            return
        kind, t_us, payload = rec
        is_banner = kind == "S" and payload.startswith(BANNER_TAGS)
        self._clock(t_us, is_banner)
        if is_banner:
            self.head_unit = payload.startswith(OT_BANNER_TAG)
        handler = getattr(self, "_on_" + kind)
        handler(t_us, payload)

    def _on_S(self, t_us, text):
        self.seen_status = True
        self.status_lines += 1
        self.d.message(f"[ESP {fmt_esp(t_us)}] {text}")
        low = text.lower()
        if "disconnect" in low:
            self.connected = False
        elif "subscribed" in low:
            self.connected = True        # H lines are authoritative anyway

    def _on_E(self, t_us, level):
        self.edges += 1

    def _on_P(self, t_us, payload):
        n, dt_us = payload
        self.pulses += 1
        if n == 1 and self.last_n is not None and self.last_n >= 1:
            self._reboot(t_us, f"pulse count restarted ({self.last_n} -> 1)")
        elif self.last_n is not None and n != self.last_n + 1:
            self.pulse_gaps += 1
            self.warn(f"pulse count gap: n {self.last_n} -> {n} "
                      f"(lost serial lines?)")
        self.last_n = n
        if 0 < dt_us <= 5_000_000:
            self.dts.append(dt_us)
        elif dt_us > 5_000_000:
            self.dts.clear()       # start after a stop: old intervals are stale
        self.last_pulse_t = t_us

    def _on_F(self, t_us, data):
        self.ftms += 1
        dec = decode_treadmill_data(data)
        flags = dec["flags"]
        ok = True
        if flags != EXPECTED_FLAGS:
            ok = False
            if flags not in self.warned_flags:
                self.warned_flags.add(flags)
                f = "none" if flags is None else f"0x{flags:04x}"
                self.warn(f"FTMS flags {f} != expected 0x{EXPECTED_FLAGS:04x} "
                          f"(decoding by flags anyway)")
        if dec["truncated"] or dec.get("expected_len") != len(data):
            ok = False
            key = (flags, len(data))
            if key not in self.warned_lens:
                self.warned_lens.add(key)
                self.warn(f"FTMS packet len {len(data)} != {dec.get('expected_len')} "
                          f"implied by flags (truncated={dec['truncated']}): "
                          f"{data.hex()}")
        if not ok:
            self.ftms_bad += 1
        dec["ok"] = ok
        self.last_ftms = dec
        self.last_ftms_t = t_us
        if ok and self.first_ftms_dist is None and dec.get("distance_m") is not None:
            self.first_ftms_dist = dec["distance_m"]
            self.first_ftms_pulses = self.pulses

    def _on_other(self, t_us, letter):
        self.other[letter] += 1

    def _on_M(self, t_us, n):
        self.markers += 1
        self.d.message(f"[ESP {fmt_esp(t_us)}] *** MARKER {n} ***")

    def _on_H(self, t_us, h):
        pulses, edges, d_e, ftms, d_f, conn, level = h
        self.heartbeats += 1
        if d_e > self.max_dropped_edges:
            self.warn(f"ESP dropped edges: {self.max_dropped_edges} -> {d_e} "
                      f"(ISR ring overflow)")
            self.max_dropped_edges = d_e
        if d_f > self.max_dropped_ftms:
            if self.head_unit:
                self.warn(f"ESP dropped debug lines: {self.max_dropped_ftms} -> {d_f} "
                          f"(serial TX buffer full; exact-line checks invalid)")
            else:
                self.warn(f"ESP dropped FTMS packets: {self.max_dropped_ftms} -> {d_f} "
                          f"(queue overflow)")
            self.max_dropped_ftms = d_f
        if self.connected is not None and bool(conn) != self.connected:
            self.d.message(f"[ESP {fmt_esp(t_us)}] BLE "
                           f"{'connected' if conn else 'DISCONNECTED'} (per heartbeat)")
        self.connected = bool(conn)
        self.last_h = h

    # -- derived values --
    def pulse_speed_kmh(self):
        if not self.dts or self.last_pulse_t is None or self.last_t is None:
            return 0.0
        if (self.last_t - self.last_pulse_t) / 1e6 > PULSE_STALE_S:
            return 0.0
        mean_dt = sum(self.dts) / len(self.dts) / 1e6
        return self.m_per_pulse / mean_dt * 3.6

    def status_line(self, elapsed_s):
        kmh = self.pulse_speed_kmh()
        dist = self.pulses * self.m_per_pulse
        parts = [f"{fmt_hms(elapsed_s)}",
                 f"P {self.pulses} ~{kmh:5.2f}km/h {kmh / KMH_PER_MPH:5.2f}mph "
                 f"~{dist:.0f}m"]
        f = self.last_ftms
        if f is None:
            parts.append("FTMS --")
        else:
            spd = f.get("speed_kmh")
            spd_s = (f"{spd:5.2f}km/h {spd / KMH_PER_MPH:5.2f}mph"
                     if spd is not None else "--")
            dist_f = f.get("distance_m")
            el = f.get("elapsed_s")
            stale = (self.last_t - self.last_ftms_t) / 1e6 > FTMS_STALE_S
            parts.append(f"FTMS{'?' if not f['ok'] else ''} {spd_s} "
                         f"{dist_f if dist_f is not None else '--'}m "
                         f"{el if el is not None else '--'}s"
                         f"{' (stale)' if stale else ''}")
        ble = {None: "?", True: "up", False: "DOWN"}[self.connected]
        parts.append(f"BLE {ble}")
        parts.append(f"drop E{self.dropped_edges_prev + self.max_dropped_edges} "
                     f"{'dbg' if self.head_unit else 'F'}{self.dropped_ftms_prev + self.max_dropped_ftms}")
        parts.append(f"M{self.markers}")
        if self.garbage_reported:
            parts.append(f"bad {self.garbage_reported}")
        return " | ".join(parts)

    def esp_elapsed_s(self):
        """ESP seconds covered so far, summed over boots."""
        if self.first_t is None:
            return 0.0
        return self.esp_span_prev + (self.last_t - self.boot_first_t) / 1e6

    def summary(self, host_s, path):
        esp_s = self.esp_elapsed_s()
        boots = f" (summed over {self.reboots + 1} boots)" if self.reboots else ""
        out = ["", "=" * 60, f"{'OpenTread' if self.head_unit else 'Pulse logger'} capture summary", "=" * 60,
               f"  host duration     {fmt_hms(host_s)} ({host_s:.1f} s)",
               f"  ESP time span     {esp_s:.3f} s{boots}",
               f"  lines             {self.lines} "
               f"(malformed {self.garbage}, of which boot noise "
               f"{self.garbage - self.garbage_reported})",
               f"  pulses (P)        {self.pulses}"
               f"  -> ~{self.pulses * self.m_per_pulse:.1f} m @ "
               f"{self.m_per_pulse} m/pulse",
               f"  edges (E)         {self.edges}"
               + (f"  ({self.edges / self.pulses:.2f} edges/pulse; 2.00 = no bounce)"
                  if self.pulses else ""),
               f"  FTMS packets (F)  {self.ftms} (suspect decode {self.ftms_bad})",
               f"  markers (M)       {self.markers}",
               f"  status (S)        {self.status_lines}",
               f"  heartbeats (H)    {self.heartbeats}",
               f"  dropped edges     "
               f"{self.dropped_edges_prev + self.max_dropped_edges} "
               f"(ESP ring overflow){boots}",
               (f"  debug lines dropped "
                f"{self.dropped_ftms_prev + self.max_dropped_ftms} "
                f"(H field 6; serial TX buffer full){boots}" if self.head_unit else
                f"  dropped FTMS      "
                f"{self.dropped_ftms_prev + self.max_dropped_ftms} "
                f"(ESP queue overflow){boots}"),
               f"  pulse count gaps  {self.pulse_gaps}",
               f"  out-of-order      {self.out_of_order} lines "
               f"(max {self.max_lag_us / 1e3:.1f} ms behind; analysis sorts by t_us)",
               f"  ESP reboots seen  {self.reboots}"]
        if self.other:
            out.append("  other lines       "
                       + " ".join(f"{k}={v}" for k, v in sorted(self.other.items()))
                       + " (letters not decoded here)")
        if self.last_h is not None:
            hp, he, _de, hf, _df, _c, lvl = self.last_h
            if self.head_unit:
                out.append(f"  last heartbeat    pulses={hp} edges={he} debug_drop={_df} "
                           f"line_level={lvl}")
            else:
                out.append(f"  last heartbeat    pulses={hp} edges={he} ftms={hf} "
                           f"line_level={lvl}")
        f = self.last_ftms
        if (f is not None and f.get("ok") and self.first_ftms_dist is not None
                and f.get("distance_m") is not None):
            d_m = f["distance_m"] - self.first_ftms_dist
            d_p = self.pulses - self.first_ftms_pulses
            line = (f"  FTMS distance     {self.first_ftms_dist} -> "
                    f"{f['distance_m']} m (+{d_m} m over {d_p} pulses")
            if d_p > 0 and d_m > 0:
                line += f"; rough {d_m / d_p:.4f} m/pulse — fit properly offline"
            out.append(line + ")")
        out.append(f"  file              {path or '(none — replay without --session)'}")
        out.append("=" * 60)
        return "\n".join(out)


# --------------------------------------------------------------------------
# Log file
# --------------------------------------------------------------------------
def open_log(session, header):
    os.makedirs(LOG_DIR, exist_ok=True)
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", session).strip("_") or "session"
    stem = f"{dt.datetime.now():%Y-%m-%d_%H%M}_{safe}"
    for k in range(1, 1000):
        path = os.path.join(LOG_DIR, stem + (f"_{k}" if k > 1 else "") + ".csv")
        try:
            fh = open(path, "xb")
        except FileExistsError:
            continue
        fh.write(header.encode("utf-8") + b"\n")
        fh.flush()
        return fh, path
    raise RuntimeError(f"could not create a unique log file for {stem}")


# --------------------------------------------------------------------------
# Sources: both yield raw byte chunks (None = nothing this tick)
# --------------------------------------------------------------------------
def open_serial(args, display):
    import serial  # pyserial; imported lazily so --replay works without it

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.1
    ser.write_timeout = 1.0   # stdin commands: never hang the capture on a stuck write
    # Deassert both before open so opening the port doesn't glitch EN/IO0.
    ser.dtr = False
    ser.rts = False
    ser.open()
    if not args.no_reset:
        display.message("Resetting ESP32 (RTS pulse on EN, DTR deasserted so IO0 stays high -> normal boot)...")
        ser.dtr = False
        ser.rts = True      # EN low: hold in reset
        time.sleep(0.1)
        ser.rts = False     # release EN -> boots app (IO0 not held low)
    return ser


CH340_VID = 0x1A86   # the CYD's USB-serial chip


def pick_port(found, info):
    """Choose the capture port from the globbed candidates `found` (sorted).
    `info` maps a device path to (vid, pid, serial_number, location) from
    serial.tools.list_ports (missing = unknown). Returns (port, None) or
    (None, error). Never guesses between two physical boards: '0001' sorts
    before the CYD's 'usbserial-110', so a plain sorted()[0] would pick a
    CP2102 pulse_gen board instead of the CYD."""
    ch340 = [d for d in found if (info.get(d) or (None,))[0] == CH340_VID]
    if len(ch340) == 1:
        return ch340[0], None
    if len(ch340) > 1:
        return None, f"several CH340 ports ({', '.join(ch340)}); pass --port"
    boards = {}
    for d in found:
        i = info.get(d)
        key = tuple(i) if i and i[0] is not None and i[3] is not None else ("?", d)
        boards.setdefault(key, []).append(d)
    if len(boards) == 1:
        return found[0], None    # one board, possibly under two names (CP2102)
    return None, f"several serial ports, possibly different boards ({', '.join(found)}); pass --port"


def port_info():
    try:
        from serial.tools import list_ports
        return {p.device: (p.vid, p.pid, p.serial_number, p.location) for p in list_ports.comports()}
    except Exception:
        return {}


class StdinCommands:
    """Non-blocking reader of command lines typed on stdin (POSIX select).
    Stops polling on EOF (stdin closed or /dev/null)."""

    def __init__(self):
        try:
            self.fd = sys.stdin.fileno()
        except (AttributeError, ValueError, OSError):
            self.fd = None
        self.buf = b""

    def poll(self):
        if self.fd is None:
            return []
        try:
            ready, _, _ = select.select([self.fd], [], [], 0)
            if not ready:
                return []
            data = os.read(self.fd, 1024)
        except (OSError, ValueError):
            self.fd = None
            return []
        if not data:
            self.fd = None
            return []
        self.buf += data
        parts = self.buf.split(b"\n")
        self.buf = parts.pop()
        return [c for c in (x.strip() for x in parts) if c]


def serial_chunks(ser):
    import serial

    while True:
        try:
            n = ser.in_waiting
            chunk = ser.read(n if n else 1)
        except (serial.SerialException, OSError) as e:
            raise ConnectionError(f"serial port error: {e}") from e
        yield chunk or None


_T_RE = re.compile(rb"^[A-Z],(\d+)")


def replay_chunks(path, speed):
    """Yield the file line by line, optionally paced by the ESP timestamps."""
    t0_esp = t0_host = None
    with open(path, "rb") as fh:
        for raw in fh:
            if raw.startswith(b"#"):
                continue        # our own capture header, not ESP output
            if speed > 0:
                m = _T_RE.match(raw)
                if m:
                    t = int(m.group(1))
                    if t0_esp is None or t < t0_esp:
                        t0_esp, t0_host = t, time.monotonic()
                    wait = (t - t0_esp) / 1e6 / speed - (time.monotonic() - t0_host)
                    if wait > 0:
                        time.sleep(min(wait, 5.0))
            yield raw


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Capture the OpenTread head unit's serial stream to logs/.")
    ap.add_argument("--port", default=None,
                    help="serial port (default: auto-detect /dev/cu.usbserial-* "
                    "[CYD, CH340] or /dev/cu.SLAB_USBtoUART [dev board, CP2102])")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--session", help="session name, e.g. S3_steptest "
                    "(required unless --replay)")
    ap.add_argument("--no-reset", action="store_true",
                    help="don't pulse RTS to reset the ESP32 on open")
    ap.add_argument("--replay", metavar="FILE",
                    help="read a previously captured log instead of serial")
    ap.add_argument("--replay-speed", type=float, default=0.0,
                    help="replay pacing: 0 = as fast as possible (default), "
                    "1 = real time by ESP timestamps, 10 = 10x")
    ap.add_argument("--status-every", type=float, default=None,
                    help="status refresh period in seconds (default 1; in a "
                    "fast replay it is ESP-time seconds, default 10)")
    ap.add_argument("--m-per-pulse", type=float, default=0.5974,
                    help="belt metres per pulse, live display estimate only")
    args = ap.parse_args()
    if not args.replay and not args.session:
        ap.error("--session is required unless --replay is given")
    if args.replay and not os.path.isfile(args.replay):
        ap.error(f"--replay file not found: {args.replay}")
    if not args.replay and not args.port:
        import glob
        found = sorted(glob.glob("/dev/cu.usbserial-*")) + sorted(glob.glob("/dev/cu.SLAB_USBtoUART"))
        if not found:
            ap.error("no ESP32 serial port found (/dev/cu.usbserial-* or /dev/cu.SLAB_USBtoUART); pass --port")
        port, err = pick_port(found, port_info())
        if err:
            ap.error(err)
        args.port = port

    display = Display()
    sess = Session(display, args.m_per_pulse)
    fast_replay = bool(args.replay) and args.replay_speed <= 0
    status_every = args.status_every or (10.0 if fast_replay else 1.0)

    # SIGTERM -> same clean shutdown path as Ctrl-C.
    def _on_sigterm(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    host_start = dt.datetime.now().astimezone()
    t_host0 = time.monotonic()
    log_fh = log_path = None
    ser = None
    commands = None    # stdin -> serial command lines (serial capture only)
    if args.session:
        src = f"replay={args.replay}" if args.replay else f"port={args.port}"
        header = (f"# tools/capture.py host_start={host_start.isoformat(timespec='seconds')} "
                  f"{src} baud={args.baud} session={args.session} "
                  f"m_per_pulse={args.m_per_pulse}")
        log_fh, log_path = open_log(args.session, header)
        display.message(f"Logging to {log_path}")

    try:
        if args.replay:
            display.message(f"Replaying {args.replay}"
                            + ("" if fast_replay else f" at {args.replay_speed}x"))
            chunks = replay_chunks(args.replay, args.replay_speed)
        else:
            ser = open_serial(args, display)
            display.message(f"Reading {args.port} @ {args.baud}. Type a command + Enter to send it. Ctrl-C to stop.")
            chunks = serial_chunks(ser)
            commands = StdinCommands()
    except Exception as e:
        display.message(f"ERROR: could not open {args.replay or args.port}: {e}")
        if log_fh:                     # nothing captured: don't leave a stub log
            log_fh.close()
            os.remove(log_path)
        return 1

    buf = b""
    last_flush = last_fsync = last_status_host = time.monotonic()
    last_status_esp = None
    rc = 0
    try:
        for chunk in chunks:
            now = time.monotonic()
            if commands is not None:
                for cmd in commands.poll():
                    text = cmd.decode("ascii", errors="replace")
                    try:
                        ser.write(cmd + b"\n")  # the device echoes it into the log (S,t,cmd;...)
                        display.message(f"> sent {text!r}")
                    except Exception as e:   # write timeout etc.: report, keep capturing
                        display.message(f"WARNING: could not send {text!r}: {e}")
            if chunk:
                if log_fh:
                    log_fh.write(chunk)          # VERBATIM, before any parsing
                buf += chunk
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    line, buf = buf[:nl + 1], buf[nl + 1:]
                    sess.feed(line.decode("utf-8", errors="replace"))
                if len(buf) > 4096:              # newline-free junk: don't grow forever
                    sess.feed(buf.decode("utf-8", errors="replace"))
                    buf = b""
            if log_fh and now - last_flush >= FLUSH_EVERY_S:
                log_fh.flush()
                last_flush = now
                if now - last_fsync >= FSYNC_EVERY_S:
                    os.fsync(log_fh.fileno())
                    last_fsync = now
            if fast_replay:
                if sess.last_t is not None:
                    if last_status_esp is None or sess.last_t < last_status_esp:
                        last_status_esp = sess.last_t   # first line / ESP reboot
                    if (sess.last_t - last_status_esp) / 1e6 >= status_every:
                        last_status_esp = sess.last_t
                        display.update(sess.status_line(sess.esp_elapsed_s()))
            elif now - last_status_host >= status_every:
                last_status_host = now
                display.update(sess.status_line(now - t_host0))
    except KeyboardInterrupt:
        pass
    except ConnectionError as e:
        display.message(f"ERROR: {e} — stopping (data up to here is saved)")
        rc = 1
    finally:
        if buf:
            sess.feed(buf.decode("utf-8", errors="replace"))
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if log_fh:
            log_fh.flush()
            os.fsync(log_fh.fileno())
            log_fh.close()
        if fast_replay and sess.last_t is not None:
            display.update(sess.status_line(sess.esp_elapsed_s()))
        display.finish()
        host_s = time.monotonic() - t_host0
        print(sess.summary(host_s, log_path))
    return rc


if __name__ == "__main__":
    sys.exit(main())
