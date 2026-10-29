#!/usr/bin/env python3
"""Touch calibration from a touch bring-up log (OT_BRINGUP=1 build).

Run with the pinned interpreter, standard library only (writes nothing):

    .venv/bin/python -I tools/touch_cal.py logs/<date>_<time>_<session>.csv
    .venv/bin/python -I tools/touch_cal.py presses.csv --size 320x240 --reff 1
    .venv/bin/python -I tools/touch_cal.py LOG --require-grid      # acceptance run
    .venv/bin/python -I tools/touch_cal.py --self-test

What it computes (from presses whose raw values come from lcd.getTouchRaw()):
  1. Screen-frame fit at the chosen rotation, from the 4 corner + centre
     presses: which raw axis drives screen x / y (axis SWAP) and whether raw
     decreases as the screen coordinate grows (INVERSION); the raw value at
     each screen edge.
  2. The LovyanGFX Touch_XPT2046 config (x_min/x_max/y_min/y_max, i.e. the
     TOUCH_X/Y_MIN/MAX constants) in the library's NATIVE touch frame. These
     depend on the library's effective touch rotation r_eff, taken from touchcfg
     (rot/poff/toff), --reff, or inferred from logged sx/sy.
  3. The 3x3 grid error with those (integer, rounded) constants, through the
     exact LovyanGFX 1.2.26 pipeline (Panel_Device::setCalibrate +
     convertRawXY: per-axis linear map, truncation to int, then r_eff
     swap/flip). Acceptance: every grid point |dx| <= 15 and |dy| <= 15 px.
     Without grid presses: calibration residuals and leave-one-out errors.
  4. TOUCH_Z_MIN from the labelled light/normal/firm x dry/damp presses
     (rule below).

------------------------------------------------------------------------------
INPUT CONTRACT (the firmware's OT_BRINGUP=1 touch screen must produce this;
emitted by firmware/opentread/hw_display.cpp; summarised in its README)
------------------------------------------------------------------------------
Log input = a tools/capture.py log (verbatim serial). Only lines containing one
of the tags `touchcal;`, `touchraw;` or `touchcfg;` are used; every other line
(boot ROM noise, E/P/H/K/D/G/T lines, torn lines) is ignored. Normally these
are S lines, so commas in the text are already ';':

  S,<t_us>,touchcfg;rot=1 poff=2 toff=0 w=320 h=240 pw=240 ph=320 x_min=300 x_max=3900 y_min=3700 y_max=200 zth=0 inset=20 reff=3
  S,<t_us>,touchcal;tgt=tl tx=20 ty=20 x=3712 y=402 z=44 zmin=41 n=25 sx=19 sy=22
  S,<t_us>,touchcal;tgt=z press=light finger=damp x=2010 y=2050 z=39 zmin=37 n=31
  S,<t_us>,touchraw;tgt=g4 tx=160 ty=120 x=2050 y=1980 z=45 sx=161 sy=118

After the tag, the payload is key=value tokens separated by spaces and/or ';'
(order free, unknown keys ignored, keys case-insensitive). Values are
integers (decimals accepted).

touchcfg (optional but strongly recommended; print once at bring-up start and
again whenever any field changes; a press uses the last touchcfg before it):
  rot      lcd.setRotation() value used for the bring-up screen (0..7)
  poff     Panel offset_rotation compiled into LGFX_CYD (0..7)
  toff     Touch_XPT2046 offset_rotation compiled into LGFX_CYD (0..7)
  w,h      logical screen size at rot (lcd.width(), lcd.height())
  pw,ph    panel_width/panel_height in the panel config (240, 320)
  x_min,x_max,y_min,y_max  Touch_XPT2046 config values live NOW (compiled, or
           applied in RAM with setTouchRanges, e.g. the bring-up's 'ka')
  zth      the z threshold the bring-up applied before logging (0 = none)
  inset    corner-target inset in px (only used when tx/ty are absent)
  reff     optional: the effective touch rotation if the firmware computes it
           (else derived from rot/poff/toff with LovyanGFX's own formula)
touchcal (one line per press; the firmware aggregates the samples it took
while the finger was held):
  tgt      target name (required): tl tr bl br c (calibration);
           g0..g8 (3x3 grid, row-major from top-left; g<r><c> also accepted);
           z (pressure test, with press= and finger=; or z_<press>_<finger>);
           any other name with tx/ty = extra check point
  x,y      raw from lcd.getTouchRaw() (median over the held samples; the
           library's own value is already a median of 7 reads) (required)
  z        tp.size from getTouchRaw (median) (required for the z rule)
  zmin     smallest size seen while held, after dropping the first/last 2
           samples (optional; the z rule uses it when present)
  tx,ty    the drawn target centre in logical screen px at rot (strongly
           recommended; defaults: corners at the inset, centre w/2,h/2, grid
           at (2c+1)w/6, (2r+1)h/6)
  sx,sy    lcd.convertRawXY() of the SAME raw x,y (copy the touch_point_t,
           convert the copy), with the compiled constants. NOT a second
           getTouch() call (that is a new, different sample). Optional: used
           to infer/verify r_eff and the library model, and to report the
           device-side grid error.
  press    light | normal | firm   (z tests; optional on other targets)
  finger   dry | damp
  n        samples aggregated (informational)
touchraw (one line per sample while held; the tool groups samples into
presses): same keys as touchcal (z per sample, no zmin/n). A press = a run of
consecutive touchraw lines with the same tgt/press/finger (and id= if given)
whose S-line timestamps are <= 150 ms apart. The first/last 2 samples of a
press are dropped when it has >= 7; then x,y,z = median, zmin = minimum.
Samples with valid=0 are skipped.

RUNS: every `screen;<name>` S line (printed when the firmware switches screens,
e.g. on kc, kg, z<label>, m, r<n>) starts a new run. The firmware restarts the
target sequence on every kc/kg, so, like its own cal;suggest, the tool fits
ONLY the last complete calibration run (all 5 of tl tr br bl c) and scores
ONLY the last complete 3x3 grid run (all 9); earlier runs (a redone kc or kg)
are discarded and counted in the output. Without screen lines (CSV input or
older logs) everything is one run. z presses from every run are used.

CSV input (alternative): a header line with at least target,x_raw,y_raw,z
(aliases: tgt, x, y, size); optional columns tx,ty,sx,sy,zmin,press,finger,n.
'#' lines are comments; a '#' line containing `touchcfg;` is parsed as above.
Each row is one press.

------------------------------------------------------------------------------
TOUCH_Z_MIN RULE
------------------------------------------------------------------------------
z is LovyanGFX's tp.size = (0x3200 + y - x + ((z1 - z2) >> 1)) >> 8 (median
of 7), so it depends on POSITION as well as pressure (raw y - x spans ~+-3700,
i.e. ~+-14 size units, comparable to the light..firm spread). Therefore:
  a. Position model f(x,y) = bx*x + by*y. Fitted by least squares on the
     calibration/grid presses plus presses labelled normal, when there are
     >= 9 of them spanning >= 1500 raw units in both x and y; otherwise the
     library's explicit term (bx, by) = (-1/256, +1/256).
  b. Every deliberate press i (all labelled presses AND all calibration/grid
     presses) is moved to the worst position on the screen:
         z_worst_i = zuse_i + min over the 4 screen corners c of f(c) - f(p_i)
     where zuse = zmin if logged, else z, and the corners' raw values come
     from the screen-frame fit.
  c. L = min_i z_worst_i (the weakest deliberate press at the worst place).
     TOUCH_Z_MIN = max(1, floor(L) - Z_MARGIN), Z_MARGIN = 2 size units (one
     unit of >>8 quantisation plus one of headroom). The touch debounce
     (3 consecutive valid samples, 120 ms release bridge) handles noise, so
     the threshold is biased to ACCEPT light/damp presses.
  Warnings: a light/normal/firm x dry/damp class missing; per-finger medians
  not light <= normal <= firm; the weakest raw z within 1 unit of the
  bring-up's zth (light presses may have been censored: rerun with zth=0).

TOUCH_Z_MIN needs the screen fit (the kc presses) in the SAME capture as the
z presses: without it no recommendation is printed (FAIL).

Exit code: 0 = calibration computed and, if grid presses exist, all within
tolerance; 1 = grid out of tolerance, missing/insufficient calibration,
inconsistent rotation, or --require-grid without a full grid; 2 = usage or
file error. --self-test runs synthetic cases (all 8 r_eff values incl.
rotated and inverted axes, each input format, injected faults).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import random
import re
import statistics
import sys
from dataclasses import dataclass, field

sys.dont_write_bytecode = True

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------
DEFAULT_W, DEFAULT_H = 320, 240        # landscape (rotation 1 or 3) logical screen
DEFAULT_PW, DEFAULT_PH = 240, 320      # ILI9341 panel_width / panel_height
START_CFG = {"x_min": 300, "x_max": 3900, "y_min": 3700, "y_max": 200}  # uncalibrated start values
DEFAULT_INSET = 20
GRID_TOL_PX = 15                       # acceptance: 3x3 grid within +-15 px
RAW_VALID_LO, RAW_VALID_HI = 129, 3968 # Touch_XPT2046::getTouchRaw keeps 128 < v <= 3968
Z_MARGIN = 2
GAP_MS = 150
TRIM = 2
TRIM_MIN_SAMPLES = 2 * TRIM + 3
LIB_BX, LIB_BY = -1.0 / 256.0, 1.0 / 256.0
POS_FIT_MIN_N, POS_FIT_MIN_SPAN = 9, 1500
MODEL_TOL_PX = 2.5                     # sx/sy vs exact library model, rms
PRESS_CLASSES = ("light", "normal", "firm")
FINGERS = ("dry", "damp")
CAL_TARGETS = ("tl", "tr", "bl", "br", "c")
GRID_TARGETS = tuple(f"g{i}" for i in range(9))

_ALIASES = {"ul": "tl", "lt": "tl", "topleft": "tl", "top_left": "tl",
            "ur": "tr", "rt": "tr", "topright": "tr", "top_right": "tr",
            "ll": "bl", "lb": "bl", "bottomleft": "bl", "bottom_left": "bl",
            "lr": "br", "rb": "br", "bottomright": "br", "bottom_right": "br",
            "cc": "c", "ctr": "c", "center": "c", "centre": "c", "mid": "c", "m": "c"}
_KEY_ALIASES = {"target": "tgt", "x_raw": "x", "y_raw": "y", "xr": "x", "yr": "y",
                "rx": "x", "ry": "y", "size": "z", "z_raw": "z", "pressure": "press",
                "z_min": "zmin", "rotation": "rot"}
_TAG_RE = re.compile(r"(?:^|[\s;:,])(touchcal|touchraw|touchcfg)[;:\s]")
_SCREEN_RE = re.compile(r"(?:^|[\s;:,])screen;([a-z_]+)")
_S_RE = re.compile(r"^S,(\d+),")
_NUM_RE = re.compile(r"^[+-]?\d+(\.\d+)?$")


class InputError(Exception):
    pass


# --------------------------------------------------------------------------
# Data
# --------------------------------------------------------------------------
@dataclass
class Cfg:
    rot: int | None = None
    poff: int | None = None
    toff: int | None = None
    reff: int | None = None
    w: int = DEFAULT_W
    h: int = DEFAULT_H
    pw: int = DEFAULT_PW
    ph: int = DEFAULT_PH
    consts: dict | None = None       # x_min.. compiled in the firmware (None = unknown)
    zth: float | None = None
    inset: int = DEFAULT_INSET
    line: int = 0

    def geom_key(self):
        return (self.rot, self.poff, self.toff, self.reff, self.w, self.h, self.pw, self.ph)


@dataclass
class Press:
    tgt: str
    kind: str                      # cal | grid | z | check
    x: float
    y: float
    z: float | None
    zmin: float | None = None
    tx: float | None = None
    ty: float | None = None
    sx: float | None = None
    sy: float | None = None
    press: str | None = None
    finger: str | None = None
    n: int = 1
    line: int = 0
    cfg: Cfg = field(default_factory=Cfg)
    default_target: bool = False
    run: int = 0                   # screen run (see RUNS in the header)


@dataclass
class Capture:
    presses: list
    cfgs: list
    warnings: list
    n_bad: int = 0
    n_tagged: int = 0
    fmt: str = "log"


def combine_rot(a: int, b: int) -> int:
    """LovyanGFX: ((r + offset) & 3) | ((r & 4) ^ (offset & 4))."""
    return ((a + b) & 3) | ((a & 4) ^ (b & 4))


def vflip(r: int) -> bool:
    return bool((1 << r) & 0b10010110)          # r = 1, 2, 4, 7


def cfg_reff(cfg: Cfg):
    if cfg.reff is not None:
        return cfg.reff & 7
    if cfg.rot is None:
        return None
    panel_r = combine_rot(cfg.rot & 7, (cfg.poff or 0) & 7)
    return combine_rot(panel_r, (cfg.toff or 0) & 7)


def touch_dims(cfg: Cfg, toff=None):
    toff = cfg.toff if toff is None else toff
    return (cfg.ph, cfg.pw) if (toff or 0) & 1 else (cfg.pw, cfg.ph)


def reff_valid(r: int, cfg: Cfg) -> bool:
    tw, th = touch_dims(cfg)
    return ((th, tw) if r & 1 else (tw, th)) == (cfg.w, cfg.h)


# --------------------------------------------------------------------------
# Parsing (never raises on bad lines; counts them)
# --------------------------------------------------------------------------
def _num(v):
    v = v.strip()
    if not _NUM_RE.match(v):
        raise ValueError(v)
    f = float(v)
    return int(f) if f.is_integer() else f


def parse_kv(payload: str) -> dict:
    out = {}
    for tok in re.split(r"[;\s]+", payload.strip()):
        if "=" not in tok:
            continue
        k, _, v = tok.partition("=")
        k = k.strip().lower()
        k = _KEY_ALIASES.get(k, k)
        if k:
            out[k] = v.strip()
    return out


def canon_target(name: str, kv: dict):
    """-> (tgt, kind, press, finger)."""
    n = name.strip().lower()
    n = _ALIASES.get(n, n)
    press = (kv.get("press") or "").lower() or None
    finger = (kv.get("finger") or "").lower() or None
    if n in CAL_TARGETS:
        return n, "cal", press, finger
    m = re.fullmatch(r"g(\d)", n)
    if m:
        return f"g{m.group(1)}", "grid", press, finger
    m = re.fullmatch(r"g([0-2])([0-2])", n)
    if m:
        return f"g{3 * int(m.group(1)) + int(m.group(2))}", "grid", press, finger
    if n == "z" or n.startswith("z_") or n.startswith("z-"):
        for part in re.split(r"[_-]", n)[1:]:
            if part in PRESS_CLASSES:
                press = press or part
            elif part in FINGERS:
                finger = finger or part
        return "z", "z", press, finger
    if press in PRESS_CLASSES:
        return n, "z", press, finger
    return n, "check", press, finger


def _cfg_from_kv(kv: dict, base: Cfg, line: int) -> Cfg:
    c = Cfg(rot=base.rot, poff=base.poff, toff=base.toff, reff=base.reff, w=base.w, h=base.h,
            pw=base.pw, ph=base.ph, consts=dict(base.consts) if base.consts else None,
            zth=base.zth, inset=base.inset, line=line)
    for k in ("rot", "poff", "toff", "reff"):
        if k in kv:
            setattr(c, k, int(_num(kv[k])))
    for k in ("w", "h", "pw", "ph", "inset"):
        if k in kv:
            setattr(c, k, int(_num(kv[k])))
    if "zth" in kv:
        c.zth = _num(kv["zth"])
    ck = [k for k in ("x_min", "x_max", "y_min", "y_max") if k in kv]
    if ck:
        if len(ck) != 4:
            raise ValueError("touchcfg needs all of x_min x_max y_min y_max")
        c.consts = {k: int(_num(kv[k])) for k in ("x_min", "x_max", "y_min", "y_max")}
    return c


def _press_from_kv(kv: dict, cfg: Cfg, line: int, need_z=False) -> Press:
    if "tgt" not in kv:
        raise ValueError("no tgt")
    tgt, kind, press, finger = canon_target(kv["tgt"], kv)
    x, y = _num(kv["x"]), _num(kv["y"])
    z = _num(kv["z"]) if kv.get("z", "") != "" else None
    if need_z and z is None:
        raise ValueError("no z")

    def opt(k):
        return _num(kv[k]) if kv.get(k, "") != "" else None
    return Press(tgt=tgt, kind=kind, x=x, y=y, z=z, zmin=opt("zmin"), tx=opt("tx"), ty=opt("ty"),
                 sx=opt("sx"), sy=opt("sy"), press=press, finger=finger,
                 n=int(opt("n") or 1), line=line, cfg=cfg)


def _group_raw(samples, warnings):
    """samples: [(t_us|None, line, kv, cfg, run)] in file order -> [Press]."""
    presses, cur, last_key, last_t = [], [], None, None

    def flush():
        if not cur:
            return
        ps = cur
        if len(ps) >= TRIM_MIN_SAMPLES:
            ps = ps[TRIM:-TRIM]
        base = ps[0][2]
        p = _press_from_kv(base, ps[0][3], ps[0][1])
        p.x = statistics.median(_num(s[2]["x"]) for s in ps)
        p.y = statistics.median(_num(s[2]["y"]) for s in ps)
        zs = [_num(s[2]["z"]) for s in ps if s[2].get("z", "") != ""]
        p.z = statistics.median(zs) if zs else None
        p.zmin = min(zs) if zs else None
        for k in ("sx", "sy"):
            vs = [_num(s[2][k]) for s in ps if s[2].get(k, "") != ""]
            setattr(p, k, statistics.median(vs) if vs else None)
        p.n = len(ps)
        p.run = ps[0][4]
        presses.append(p)

    for t, line, kv, cfg, run in samples:
        key = (kv.get("tgt", "").lower(), kv.get("press"), kv.get("finger"), kv.get("id"), id(cfg), run)
        gap = (t is not None and last_t is not None and t - last_t > GAP_MS * 1000)
        if key != last_key or gap:
            flush()
            cur = []
        cur.append((t, line, kv, cfg, run))
        last_key, last_t = key, t
    flush()
    return presses


def parse_lines(lines, base_cfg: Cfg | None = None) -> Capture:
    base_cfg = base_cfg or Cfg()
    cap = Capture(presses=[], cfgs=[base_cfg], warnings=[])
    first = next((ln.strip() for ln in lines if ln.strip() and not ln.lstrip().startswith("#")), "")
    head = [c.strip().lower() for c in first.split(",")]
    if any(c in ("target", "tgt") for c in head) and any(c in ("x_raw", "x") for c in head):
        return _parse_csv(lines, base_cfg, cap)

    cfg = base_cfg
    raw_samples = []
    run = 0
    for i, raw in enumerate(lines, 1):
        ln = raw.rstrip("\r\n").replace("\r", "")
        m = _TAG_RE.search(ln)
        if not m:
            if _SCREEN_RE.search(ln):
                run += 1
            continue
        cap.n_tagged += 1
        tag = m.group(1)
        sm = _S_RE.match(ln)
        t = int(sm.group(1)) if sm else None
        kv = parse_kv(ln[m.end():])
        try:
            if tag == "touchcfg":
                cfg = _cfg_from_kv(kv, cfg, i)
                cap.cfgs.append(cfg)
            elif tag == "touchcal":
                pr = _press_from_kv(kv, cfg, i)
                pr.run = run
                cap.presses.append(pr)
            else:
                if kv.get("valid", "1") in ("0", "false"):
                    continue
                for k in ("tgt", "x", "y"):
                    if k not in kv:
                        raise ValueError(f"no {k}")
                _num(kv["x"]), _num(kv["y"])
                if kv.get("z", "") != "":
                    _num(kv["z"])
                raw_samples.append((t, i, kv, cfg, run))
        except (ValueError, KeyError) as e:
            cap.n_bad += 1
            if cap.n_bad <= 5:
                cap.warnings.append(f"line {i}: malformed {tag} line ignored ({e}): {ln[:100]}")
    cap.presses.extend(_group_raw(raw_samples, cap.warnings))
    cap.presses.sort(key=lambda p: p.line)
    return cap


def _parse_csv(lines, cfg: Cfg, cap: Capture) -> Capture:
    cap.fmt = "csv"
    body = []
    for i, raw in enumerate(lines, 1):
        ln = raw.rstrip("\r\n")
        if not ln.strip():
            continue
        if ln.lstrip().startswith("#"):
            m = _TAG_RE.search(ln)
            if m and m.group(1) == "touchcfg":
                try:
                    cfg = _cfg_from_kv(parse_kv(ln[m.end():]), cfg, i)
                    cap.cfgs.append(cfg)
                except ValueError as e:
                    cap.n_bad += 1
                    cap.warnings.append(f"line {i}: malformed touchcfg ignored ({e})")
            continue
        body.append((i, ln))
    rd = csv.reader([b for _, b in body])
    header = None
    for (i, _), row in zip(body, rd):
        if header is None:
            header = [_KEY_ALIASES.get(c.strip().lower(), c.strip().lower()) for c in row]
            continue
        kv = {k: v for k, v in zip(header, row) if v.strip() != ""}
        try:
            cap.presses.append(_press_from_kv(kv, cfg, i))
        except (ValueError, KeyError) as e:
            cap.n_bad += 1
            if cap.n_bad <= 5:
                cap.warnings.append(f"line {i}: malformed CSV row ignored ({e})")
    return cap


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------
def default_target(p: Press, cfg: Cfg):
    W, H, a = cfg.w, cfg.h, cfg.inset
    if p.kind == "cal":
        return {"tl": (a, a), "tr": (W - 1 - a, a), "bl": (a, H - 1 - a),
                "br": (W - 1 - a, H - 1 - a), "c": (W // 2, H // 2)}[p.tgt]
    if p.kind == "grid":
        k = int(p.tgt[1:])
        r, c = divmod(k, 3)
        return (round((2 * c + 1) * W / 6), round((2 * r + 1) * H / 6))
    return None


def fit_line(pairs):
    """Least squares screen = m*raw + c over [(raw, screen)] -> (m, c, sse)."""
    n = len(pairs)
    mr = sum(r for r, _ in pairs) / n
    ms = sum(s for _, s in pairs) / n
    sxx = sum((r - mr) ** 2 for r, _ in pairs)
    if sxx == 0:
        raise InputError("degenerate fit: identical raw values")
    m = sum((r - mr) * (s - ms) for r, s in pairs) / sxx
    c = ms - m * mr
    sse = sum((m * r + c - s) ** 2 for r, s in pairs)
    return m, c, sse


@dataclass
class ScreenCal:
    swap: bool                 # True: screen x <- raw y, screen y <- raw x
    mx: float
    cx: float
    my: float
    cy: float
    sse: float
    sse_other: float
    W: int
    H: int

    def src(self, axis):       # raw axis driving screen axis 'x'/'y'
        return ("y" if self.swap else "x") if axis == "x" else ("x" if self.swap else "y")

    def raw_at(self, axis, s):
        m, c = (self.mx, self.cx) if axis == "x" else (self.my, self.cy)
        return (s - c) / m

    def edges(self):
        return {"x0": self.raw_at("x", 0), "x1": self.raw_at("x", self.W - 1),
                "y0": self.raw_at("y", 0), "y1": self.raw_at("y", self.H - 1)}

    def int_edges(self):
        return {k: int(round(v)) for k, v in self.edges().items()}

    def map_int(self, rx, ry):
        """Screen-frame integer pipeline (same arithmetic as LovyanGFX)."""
        e = self.int_edges()
        ax, ay = (ry, rx) if self.swap else (rx, ry)
        sx = math.trunc((ax - e["x0"]) * (self.W - 1) / (e["x1"] - e["x0"]))
        sy = math.trunc((ay - e["y0"]) * (self.H - 1) / (e["y1"] - e["y0"]))
        return sx, sy

    def corners_raw(self):
        out = []
        for sx in (0, self.W - 1):
            for sy in (0, self.H - 1):
                a, b = self.raw_at("x", sx), self.raw_at("y", sy)
                out.append((b, a) if self.swap else (a, b))   # (raw x, raw y)
        return out


def fit_screen(points, W, H, swap=None) -> ScreenCal:
    """points: [(rx, ry, tx, ty)]."""
    if len({p[2] for p in points}) < 2 or len({p[3] for p in points}) < 2:
        raise InputError("calibration needs >= 2 distinct target x and >= 2 distinct target y")
    res = {}
    for sw in (False, True):
        ax = [((p[1] if sw else p[0]), p[2]) for p in points]
        ay = [((p[0] if sw else p[1]), p[3]) for p in points]
        mx, cx, ex = fit_line(ax)
        my, cy, ey = fit_line(ay)
        res[sw] = (mx, cx, my, cy, ex + ey)
    best = min(res, key=lambda s: res[s][4]) if swap is None else swap
    other = not best
    mx, cx, my, cy, sse = res[best]
    if mx == 0 or my == 0:
        raise InputError("degenerate fit: zero slope")
    return ScreenCal(best, mx, cx, my, cy, sse, res[other][4], W, H)


def lgfx_map(consts, r, cfg: Cfg, rx, ry):
    """Exact LovyanGFX 1.2.26 Panel_Device::convertRawXY for rectangular cfg."""
    tw, th = touch_dims(cfg)
    tx = math.trunc((rx - consts["x_min"]) * (tw - 1) / (consts["x_max"] - consts["x_min"]))
    ty = math.trunc((ry - consts["y_min"]) * (th - 1) / (consts["y_max"] - consts["y_min"]))
    if r:
        if r & 1:
            tx, ty = ty, tx
        if r & 2:
            tx = (cfg.w - 1) - tx
        if vflip(r):
            ty = (cfg.h - 1) - ty
    return tx, ty


def raw_axis_target(r):
    """For raw axis 'x'/'y': (screen axis it lands on, flipped?) under r_eff."""
    if r & 1:
        return {"x": ("y", vflip(r)), "y": ("x", bool(r & 2))}
    return {"x": ("x", bool(r & 2)), "y": ("y", vflip(r))}


def lgfx_consts(sc: ScreenCal, r: int, cfg: Cfg):
    """-> (consts float dict) or raise InputError when the axes contradict r."""
    out = {}
    for raw_ax in ("x", "y"):
        s_ax, flip = raw_axis_target(r)[raw_ax]
        if sc.src(s_ax) != raw_ax:
            raise InputError(
                f"r_eff={r} maps raw {raw_ax} to screen {s_ax}, but the presses show screen "
                f"{s_ax} driven by raw {sc.src(s_ax)} (touch axes swapped relative to this "
                f"rotation: an odd touch offset_rotation, or a different rot/poff)")
        n = sc.W if s_ax == "x" else sc.H
        lo, hi = (n - 1, 0) if flip else (0, n - 1)
        out[f"{raw_ax}_min"] = sc.raw_at(s_ax, lo)
        out[f"{raw_ax}_max"] = sc.raw_at(s_ax, hi)
    return out


def infer_reff(presses, cfg: Cfg):
    """From sx/sy logged with the compiled constants. -> (r, rms, ranking) or (None, ...)."""
    rows = [p for p in presses if p.sx is not None and p.sy is not None and p.cfg.consts]
    if len(rows) < 3:
        return None, None, []
    rank = []
    for r in range(8):
        if not reff_valid(r, cfg):
            continue
        sse = 0.0
        for p in rows:
            px, py = lgfx_map(p.cfg.consts, r, p.cfg, p.x, p.y)
            sse += (px - p.sx) ** 2 + (py - p.sy) ** 2
        rank.append((math.sqrt(sse / len(rows)), r))
    rank.sort()
    if not rank:
        return None, None, rank
    best_rms, best_r = rank[0]
    second = rank[1][0] if len(rank) > 1 else math.inf
    if best_rms < 0.25 * min(cfg.w, cfg.h) and second >= 2 * best_rms + 5:
        return best_r, best_rms, rank
    return None, best_rms, rank


# --------------------------------------------------------------------------
# Analysis
# --------------------------------------------------------------------------
@dataclass
class Result:
    lines: list = field(default_factory=list)
    fails: list = field(default_factory=list)
    warns: list = field(default_factory=list)
    sc: ScreenCal | None = None
    reff: int | None = None
    lg: dict | None = None            # integer LovyanGFX constants
    grid_max: float | None = None
    zmin_rec: int | None = None
    swap: bool | None = None
    inv: dict | None = None

    def out(self, s=""):
        self.lines.append(s)

    def code(self):
        return 1 if self.fails else 0


def _fmt(v):
    return f"{v:.1f}" if isinstance(v, float) else str(v)


def last_complete_run(presses, targets):
    """Split presses by run; keep the last run holding every target (else the last
    run). -> (kept presses, number of other runs discarded, kept run complete?)."""
    runs = {}
    for p in presses:
        runs.setdefault(p.run, []).append(p)
    if not runs:
        return [], 0, False
    order = sorted(runs)
    complete = [r for r in order if set(targets) <= {p.tgt for p in runs[r]}]
    pick = complete[-1] if complete else order[-1]
    return runs[pick], len(order) - 1, bool(complete)


def analyse(cap: Capture, args) -> Result:
    R = Result()
    R.warns.extend(cap.warnings)
    if not cap.presses:
        R.fails.append("no touchcal/touchraw presses found (see the input contract at the top of this file)")
        return R

    # Geometry segment: the last geometry that has calibration presses.
    with_cal = [p for p in cap.presses if p.kind == "cal"]
    gkey = (with_cal or cap.presses)[-1].cfg.geom_key()
    geo = [p for p in cap.presses if p.cfg.geom_key() == gkey]
    dropped = [p for p in cap.presses if p.cfg.geom_key() != gkey and p.kind != "z"]
    if dropped:
        R.warns.append(f"{len(dropped)} position press(es) under a different rotation/geometry "
                       "touchcfg were ignored for the x/y fit")
    cfg = geo[-1].cfg
    W, H = cfg.w, cfg.h
    for p in cap.presses:
        if p.tx is None or p.ty is None:
            d = default_target(p, p.cfg)
            if d is not None:
                p.tx, p.ty = d
                p.default_target = True
        if not (RAW_VALID_LO <= p.x <= RAW_VALID_HI and RAW_VALID_LO <= p.y <= RAW_VALID_HI):
            R.warns.append(f"line {p.line}: raw ({_fmt(p.x)},{_fmt(p.y)}) outside getTouchRaw's "
                           f"valid {RAW_VALID_LO}..{RAW_VALID_HI}: not a getTouchRaw value?")
    if any(p.default_target for p in geo if p.kind in ("cal", "grid")):
        R.warns.append(f"some cal/grid presses have no tx/ty: DEFAULT target positions used "
                       f"(inset {cfg.inset}); log tx/ty from the firmware to remove this assumption")

    R.out(f"input: {cap.fmt}, {len(cap.presses)} presses ({cap.n_tagged} tagged lines, "
          f"{cap.n_bad} malformed ignored)")
    rot_s = "?" if cfg.rot is None else str(cfg.rot)
    R.out(f"geometry: screen {W}x{H} at rot={rot_s} poff={_fmt(cfg.poff) if cfg.poff is not None else '?'} "
          f"toff={_fmt(cfg.toff) if cfg.toff is not None else '?'}, panel {cfg.pw}x{cfg.ph}; "
          f"compiled consts: {cfg.consts if cfg.consts else 'unknown'}")

    # ---- 1. screen-frame fit ------------------------------------------
    cal, cal_drop, cal_complete = last_complete_run([p for p in geo if p.kind == "cal"], CAL_TARGETS)
    if cal_drop:
        R.out(f"calibration: fitted the last {'complete ' if cal_complete else ''}kc run only "
              f"(lines {cal[0].line}-{cal[-1].line}); {cal_drop} other run(s) discarded, as the "
              "firmware's cal;suggest does")
    have = sorted({p.tgt for p in cal})
    missing = [t for t in CAL_TARGETS if t not in have]
    if missing:
        R.warns.append(f"calibration targets missing: {' '.join(missing)} (expected: 4 corners + centre)")
    try:
        sc = fit_screen([(p.x, p.y, p.tx, p.ty) for p in cal], W, H)
    except (InputError, ZeroDivisionError) as e:
        R.fails.append(f"calibration fit impossible: {e} (the grid is scored with constants fitted "
                       "from kc presses, so every capture, including the post-rebuild check capture, "
                       "needs a kc run before kg)")
        return _z_section(R, cap, None)
    R.sc, R.swap = sc, sc.swap
    e = sc.edges()
    inv = {"x": e["x1"] < e["x0"], "y": e["y1"] < e["y0"]}
    R.inv = inv
    R.out("")
    R.out("== screen-frame calibration (4 corners + centre) ==")
    R.out(f"axis swap: {'YES' if sc.swap else 'no'} (fit SSE {sc.sse:.2f} px^2 vs {sc.sse_other:.1f} "
          f"for the other assignment)")
    for ax, n in (("x", W), ("y", H)):
        R.out(f"screen {ax} <- raw {sc.src(ax)}: raw {e[ax + '0']:.0f} at {ax}=0 .. {e[ax + '1']:.0f} at "
              f"{ax}={n - 1}  ({'INVERTED: raw decreases' if inv[ax] else 'raw increases'} as {ax} grows)")
    if sc.sse_other < 4 * sc.sse + 1:
        R.fails.append("axis assignment ambiguous (both swap choices fit about equally): "
                       "check tx/ty and that the presses hit the targets")
    R.out("residuals at the calibration presses (new integer constants):")
    for p in cal:
        mx_, my_ = sc.map_int(p.x, p.y)
        R.out(f"  {p.tgt:>3} target ({_fmt(p.tx)},{_fmt(p.ty)}) raw ({_fmt(p.x)},{_fmt(p.y)}) -> "
              f"({mx_},{my_})  dx={mx_ - p.tx:+.0f} dy={my_ - p.ty:+.0f}")
    loo = []
    for t in have:
        rest = [(q.x, q.y, q.tx, q.ty) for q in cal if q.tgt != t]
        try:
            s2 = fit_screen(rest, W, H, swap=sc.swap)
        except (InputError, ZeroDivisionError):
            continue
        for q in (q for q in cal if q.tgt == t):
            a, b = s2.map_int(q.x, q.y)
            loo.append((t, a - q.tx, b - q.ty))
    if loo:
        w = max(loo, key=lambda r: max(abs(r[1]), abs(r[2])))
        R.out(f"leave-one-out (refit without the target, predict it): worst {w[0]} "
              f"dx={w[1]:+.0f} dy={w[2]:+.0f}  (conservative: corners are extrapolated)")

    # ---- 2. LovyanGFX constants -----------------------------------------
    R.out("")
    R.out("== LovyanGFX Touch_XPT2046 constants (native touch frame) ==")
    r_cfg = cfg_reff(cfg)
    r_inf, rms_inf, rank = infer_reff(geo, cfg)
    if args.reff is not None:
        r, src = args.reff, "--reff"
    elif r_cfg is not None:
        r, src = r_cfg, "touchcfg rot/poff/toff" if cfg.reff is None else "touchcfg reff"
    elif r_inf is not None:
        r, src = r_inf, f"inferred from sx/sy (rms {rms_inf:.1f} px)"
    else:
        r, src = None, None
    if rank:
        R.out("r_eff fit to logged sx/sy: " + ", ".join(f"r={rr}:{rm:.1f}px" for rm, rr in rank[:4]))
    if r is not None and r_inf is not None and r_inf != r:
        R.fails.append(f"r_eff from {src} = {r} but the logged sx/sy fit r_eff = {r_inf}")
    if r is not None and not reff_valid(r, cfg):
        R.fails.append(f"r_eff={r} is incompatible with screen {W}x{H} / panel {cfg.pw}x{cfg.ph} "
                       f"(toff={cfg.toff or 0})")
        r = None
    if r is not None and rank and rms_inf is not None:
        rms_r = next((rm for rm, rr in rank if rr == r), None)
        if rms_r is not None and rms_r > MODEL_TOL_PX:
            R.warns.append(f"logged sx/sy differ from the exact LovyanGFX model by rms {rms_r:.1f} px "
                           f"at r_eff={r}: stale touchcfg consts, or sx/sy not convertRawXY of the same "
                           "sample. LovyanGFX constants below are UNVERIFIED")
    if r is None:
        R.warns.append("r_eff unknown (no touchcfg rot, no --reff, no usable sx/sy): only the "
                       "screen-frame calibration above is valid; LovyanGFX constants not computed")
    else:
        try:
            lg = lgfx_consts(sc, r, cfg)
        except InputError as ex:
            R.fails.append(str(ex))
            lg = None
        if lg:
            R.reff = r
            R.lg = {k: int(round(v)) for k, v in lg.items()}
            R.out(f"r_eff = {r} ({src}); rotation {cfg.rot if cfg.rot is not None else '?'}")
            for k in ("x_min", "x_max", "y_min", "y_max"):
                v = R.lg[k]
                R.out(f"  {k:5} = {v:5d}   (exact {lg[k]:.1f})")
                if not 0 <= v <= 65535:
                    R.fails.append(f"{k}={v} does not fit Touch_XPT2046's uint16_t")
                elif not 0 <= v <= 4095:
                    R.warns.append(f"{k}={v} is outside the 12-bit ADC range (extrapolated far)")
            R.out(f"  x: {'inverted (x_min > x_max)' if R.lg['x_min'] > R.lg['x_max'] else 'normal'}, "
                  f"y: {'inverted (y_min > y_max)' if R.lg['y_min'] > R.lg['y_max'] else 'normal'}")
            if cfg.consts:
                R.out(f"  compiled now: {cfg.consts}")

    # ---- 3. grid --------------------------------------------------------
    R.out("")
    R.out(f"== 3x3 grid (tolerance +-{args.tol} px per axis) ==")
    grid_all = [p for p in geo if p.kind == "grid" and p.tx is not None and p.ty is not None]
    grid_run, grid_drop, grid_complete = last_complete_run(grid_all, GRID_TARGETS)
    if grid_drop:
        R.out(f"grid: scored the last {'complete ' if grid_complete else ''}kg run only; "
              f"{grid_drop} other run(s) discarded")
    grid = grid_run + [p for p in geo if p.kind == "check" and p.tx is not None and p.ty is not None]
    grid.sort(key=lambda p: p.line)

    def mapper(p):
        if R.lg is not None:
            return lgfx_map(R.lg, R.reff, cfg, p.x, p.y)
        return sc.map_int(p.x, p.y)
    if not grid:
        R.out("no grid presses logged: grid NOT MEASURED (acceptance needs a measured 3x3 grid)")
        if loo:
            worst = max(max(abs(a), abs(b)) for _, a, b in loo)
            R.out(f"predicted grid error <= ~{worst:.0f} px (leave-one-out bound)")
        if args.require_grid:
            R.fails.append("--require-grid: no grid presses")
    else:
        R.out("  tgt   target      raw          predicted  dx  dy | device(sx,sy) dx  dy")
        worst = 0.0
        for p in grid:
            a, b = mapper(p)
            dx, dy = a - p.tx, b - p.ty
            if p.kind == "grid":
                worst = max(worst, abs(dx), abs(dy))
            dev = ""
            if p.sx is not None and p.sy is not None:
                dev = f" | ({_fmt(p.sx)},{_fmt(p.sy)}) {p.sx - p.tx:+4.0f} {p.sy - p.ty:+4.0f}"
            flag = "" if max(abs(dx), abs(dy)) <= args.tol else "  <-- OUT"
            R.out(f"  {p.tgt:>4} ({_fmt(p.tx):>4},{_fmt(p.ty):>4}) ({_fmt(p.x):>5},{_fmt(p.y):>5}) "
                  f"({a:4d},{b:4d}) {dx:+4.0f} {dy:+4.0f}{dev}{flag}")
        R.grid_max = worst
        got = {p.tgt for p in grid if p.kind == "grid"}
        miss = [t for t in GRID_TARGETS if t not in got]
        R.out(f"grid max |error| = {worst:.0f} px over {len(got)}/9 grid targets "
              f"({'constants: LovyanGFX pipeline' if R.lg else 'constants: screen-frame'})")
        if worst > args.tol:
            R.fails.append(f"grid error {worst:.0f} px > +-{args.tol} px")
        if miss:
            msg = f"grid targets missing: {' '.join(miss)}"
            (R.fails if args.require_grid else R.warns).append(msg)
    return _z_section(R, cap, sc)


def _z_section(R: Result, cap: Capture, sc: ScreenCal | None) -> Result:
    R.out("")
    R.out("== TOUCH_Z_MIN (rule: see the file header) ==")
    zp = [p for p in cap.presses if p.z is not None]
    labelled = [p for p in zp if p.press in PRESS_CLASSES]
    posp = [p for p in zp if p.kind in ("cal", "grid") or p.press == "normal"]
    deliberate = [p for p in zp if p.press in PRESS_CLASSES or p.kind in ("cal", "grid")]
    unl = [p for p in zp if p.kind == "z" and p.press not in PRESS_CLASSES]
    if unl:
        R.warns.append(f"{len(unl)} z-test press(es) without a light/normal/firm label ignored "
                       f"(first at line {unl[0].line})")
    if not deliberate:
        R.warns.append("no z values: TOUCH_Z_MIN not computed")
        R.out("no presses with z")
        return R
    if not labelled:
        R.warns.append("no light/normal/firm labelled presses: TOUCH_Z_MIN from calibration presses only")
    # position model
    bx, by, how = LIB_BX, LIB_BY, "library term (y - x)/256"
    if (len(posp) >= POS_FIT_MIN_N and max(p.x for p in posp) - min(p.x for p in posp) >= POS_FIT_MIN_SPAN
            and max(p.y for p in posp) - min(p.y for p in posp) >= POS_FIT_MIN_SPAN):
        fb = _fit_plane([(p.x, p.y, p.z) for p in posp])
        if fb is not None:
            bx, by = fb
            how = f"fitted on {len(posp)} cal/grid/normal presses"
    R.out(f"position model f = {bx * 256:+.2f}*x/256 {by * 256:+.2f}*y/256 ({how}; library term "
          "is -1*x/256 +1*y/256)")
    if sc is not None:
        corners = sc.corners_raw()
    else:
        # The observed extent (z presses sit inside the inset box) is NOT the
        # worst screen position: the table below is informative only.
        xs, ys = [p.x for p in zp], [p.y for p in zp]
        corners = [(a, b) for a in (min(xs), max(xs)) for b in (min(ys), max(ys))]
    fmin = min(bx * a + by * b for a, b in corners)
    worst_p, L = None, math.inf
    for p in deliberate:
        zu = p.zmin if p.zmin is not None else p.z
        zw = zu + fmin - (bx * p.x + by * p.y)
        if zw < L:
            L, worst_p = zw, p
    R.out("  class         n   z min   median  max   worst-position min")
    for pr in PRESS_CLASSES:
        for fi in FINGERS:
            g = [p for p in labelled if p.press == pr and (p.finger or "dry") == fi]
            if not g:
                R.out(f"  {pr:6} {fi:4}   0   (missing)")
                continue
            zs = [p.z for p in g]
            ww = min((p.zmin if p.zmin is not None else p.z) + fmin - (bx * p.x + by * p.y) for p in g)
            R.out(f"  {pr:6} {fi:4} {len(g):3} {min(zs):7.1f} {statistics.median(zs):7.1f} {max(zs):6.1f}"
                  f"   {ww:6.1f}")
    cg = [p for p in deliberate if p.press not in PRESS_CLASSES]
    if cg:
        zs = [p.z for p in cg]
        R.out(f"  cal/grid    {len(cg):3} {min(zs):7.1f} {statistics.median(zs):7.1f} {max(zs):6.1f}")
    miss = [f"{pr}/{fi}" for pr in PRESS_CLASSES for fi in FINGERS
            if not any(p.press == pr and (p.finger or "dry") == fi for p in labelled)]
    if miss and labelled:
        R.warns.append(f"z classes missing: {' '.join(miss)} (expected: light/normal/firm, dry and damp)")
    for fi in FINGERS:
        med = [statistics.median([p.z for p in labelled if p.press == pr and (p.finger or "dry") == fi])
               for pr in PRESS_CLASSES
               if any(p.press == pr and (p.finger or "dry") == fi for p in labelled)]
        if len(med) >= 2 and any(b < a for a, b in zip(med, med[1:])):
            R.warns.append(f"{fi}: z medians not ordered light <= normal <= firm ({med}): labels or "
                           "press technique suspect")
    zths = [p.cfg.zth for p in zp if p.cfg.zth]
    if zths:
        zth = max(zths)
        if min(p.z for p in zp) <= zth + 1:
            R.warns.append(f"weakest raw z is within 1 of the bring-up threshold zth={zth}: light "
                           "presses may have been censored; rerun with zth=0")
    if sc is None:
        R.out("TOUCH_Z_MIN NOT COMPUTED: needs the kc calibration presses in the same capture "
              "(the worst screen position comes from the screen fit); the table above uses the "
              "observed press extent only and is not usable")
        R.fails.append("TOUCH_Z_MIN not computed: run kc (and kg) and every z<label> in ONE capture")
        return R
    rec = max(1, math.floor(L) - Z_MARGIN)
    R.zmin_rec = rec
    R.out(f"weakest deliberate press at the worst position: L = {L:.1f} (line {worst_p.line}, "
          f"tgt={worst_p.tgt} press={worst_p.press or '-'} finger={worst_p.finger or '-'})")
    R.out(f"TOUCH_Z_MIN = max(1, floor(L) - {Z_MARGIN}) = {rec}")
    if L - Z_MARGIN < 1:
        R.warns.append("no z headroom at the worst position: TOUCH_Z_MIN=1 (accept anything the "
                       "library reports)")
    return R


def _fit_plane(pts):
    """Least squares z = a + bx*x + by*y -> (bx, by) or None."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    mz = sum(p[2] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    sxz = sum((p[0] - mx) * (p[2] - mz) for p in pts)
    syz = sum((p[1] - my) * (p[2] - mz) for p in pts)
    det = sxx * syy - sxy * sxy
    if abs(det) < 1e-9 * max(1.0, sxx * syy):
        return None
    return (sxz * syy - syz * sxy) / det, (syz * sxx - sxz * sxy) / det


def snippet(R: Result, cfg_desc: str, src: str) -> list:
    if R.sc is None:
        return []
    e = R.sc.int_edges()
    out = ["", "== config_hw.h (paste after review) ==",
           f"// Touch calibration: tools/touch_cal.py {src} ({cfg_desc})"]
    if R.lg:
        out.append(f"// LovyanGFX Touch_XPT2046 config, native touch frame, r_eff={R.reff}")
        for k, nm in (("x_min", "TOUCH_X_MIN"), ("x_max", "TOUCH_X_MAX"),
                      ("y_min", "TOUCH_Y_MIN"), ("y_max", "TOUCH_Y_MAX")):
            out.append(f"constexpr uint16_t {nm} = {R.lg[k]};")
    else:
        out.append("// TOUCH_X/Y_MIN/MAX: r_eff unknown, LovyanGFX constants not computed")
    if R.zmin_rec is not None:
        out.append(f"constexpr uint16_t TOUCH_Z_MIN = {R.zmin_rec};  // getTouchRaw() tp.size units")
    out.append(f"// screen frame at this rotation: x <- raw {R.sc.src('x')} ({e['x0']}..{e['x1']}), "
               f"y <- raw {R.sc.src('y')} ({e['y0']}..{e['y1']})")
    return out


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------
def base_cfg_from_args(args) -> Cfg:
    c = Cfg()
    if args.size:
        c.w, c.h = _wh(args.size)
    if args.panel:
        c.pw, c.ph = _wh(args.panel)
    c.rot, c.poff, c.toff = args.rot, args.poff, args.toff
    c.inset = args.inset
    if args.cfg:
        kv = parse_kv(args.cfg)
        c.consts = {k: int(_num(kv[k])) for k in ("x_min", "x_max", "y_min", "y_max")}
    return c


def _wh(s):
    m = re.fullmatch(r"(\d+)[xX](\d+)", s.strip())
    if not m:
        raise argparse.ArgumentTypeError(f"bad WxH: {s}")
    return int(m.group(1)), int(m.group(2))


def run(lines, args, src="<input>"):
    cap = parse_lines(lines, base_cfg_from_args(args))
    R = analyse(cap, args)
    cfg = cap.presses[-1].cfg if cap.presses else cap.cfgs[-1]
    R.lines += snippet(R, f"screen {cfg.w}x{cfg.h} rot={'?' if cfg.rot is None else cfg.rot}", src)
    R.out("")
    for w in R.warns:
        R.out(f"WARN: {w}")
    for f in R.fails:
        R.out(f"FAIL: {f}")
    R.out(f"RESULT: {'FAIL' if R.fails else 'PASS'}")
    return R


def make_parser():
    ap = argparse.ArgumentParser(description="Touch calibration from a touch bring-up log or CSV.")
    ap.add_argument("input", nargs="?", help="tools/capture.py log or CSV (target,x_raw,y_raw,z,...)")
    ap.add_argument("--size", help="logical screen WxH at the chosen rotation (default 320x240)")
    ap.add_argument("--panel", help="panel_width x panel_height (default 240x320)")
    ap.add_argument("--rot", type=int, help="setRotation value (when no touchcfg line)")
    ap.add_argument("--poff", type=int, help="panel offset_rotation (when no touchcfg line)")
    ap.add_argument("--toff", type=int, help="touch offset_rotation (when no touchcfg line)")
    ap.add_argument("--reff", type=int, choices=range(8), metavar="0-7",
                    help="force LovyanGFX's effective touch rotation (overrides touchcfg)")
    ap.add_argument("--cfg", help="compiled consts when no touchcfg: 'x_min=300 x_max=3900 y_min=3700 y_max=200'")
    ap.add_argument("--inset", type=int, default=DEFAULT_INSET, help="default corner inset px (no tx/ty)")
    ap.add_argument("--tol", type=float, default=GRID_TOL_PX, help="grid tolerance px per axis (15)")
    ap.add_argument("--require-grid", action="store_true", help="fail unless all 9 grid targets were logged")
    ap.add_argument("--self-test", action="store_true", help="run synthetic self-tests and exit")
    return ap


def main(argv=None) -> int:
    ap = make_parser()
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    if not args.input:
        ap.print_usage(sys.stderr)
        return 2
    try:
        with open(args.input, "r", encoding="utf-8", errors="replace", newline="") as f:
            lines = f.read().splitlines()
    except OSError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    try:
        R = run(lines, args, os.path.basename(args.input))
    except (KeyError, ValueError, argparse.ArgumentTypeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print("\n".join(R.lines))
    return R.code()


# --------------------------------------------------------------------------
# Self-test (synthetic; independent ground truth = the forward LovyanGFX model)
# --------------------------------------------------------------------------
def _truth_raw(consts, r, cfg: Cfg, sx, sy):
    """Inverse of lgfx_map in float (no truncation): screen point -> raw (rx, ry)."""
    tw, th = touch_dims(cfg)
    u, v = float(sx), float(sy)
    if r:
        if vflip(r):
            v = (cfg.h - 1) - v
        if r & 2:
            u = (cfg.w - 1) - u
        if r & 1:
            u, v = v, u
    rx = consts["x_min"] + u * (consts["x_max"] - consts["x_min"]) / (tw - 1)
    ry = consts["y_min"] + v * (consts["y_max"] - consts["y_min"]) / (th - 1)
    return rx, ry


Z_BASE = {("light", "dry"): 44.0, ("normal", "dry"): 47.0, ("firm", "dry"): 50.0,
          ("light", "damp"): 43.0, ("normal", "damp"): 46.0, ("firm", "damp"): 49.0}


def _z_of(base, rx, ry):
    return base + (ry - rx) / 256.0


def synth(r, consts, w=320, h=240, toff=0, seed=1, fmt="cal", noise=6.0, sxsy=True,
          cfg_line=True, logged_consts=None, grid_fault=None, drop=(), zth=0, tx_ty=True,
          garbage=True, markers=False, cal_fault=None, with_z=True):
    """Return (lines, truth_cfg). consts = the TRUE calibration (what the panel does)."""
    rnd = random.Random(seed)
    cfg = Cfg(w=w, h=h, pw=240, ph=320, toff=toff, reff=r)
    compiled = logged_consts or START_CFG
    inset = DEFAULT_INSET
    targets = [("tl", inset, inset), ("tr", w - 1 - inset, inset), ("bl", inset, h - 1 - inset),
               ("br", w - 1 - inset, h - 1 - inset), ("c", w // 2, h // 2)]
    targets += [(f"g{3 * rr + cc}", round((2 * cc + 1) * w / 6), round((2 * rr + 1) * h / 6))
                for rr in range(3) for cc in range(3)]
    lines = ["# tools/capture.py host_start=synthetic session=selftest"]
    if garbage:
        lines += ["ets Jul 29 2019 12:21:46", "rst:0x1 (POWERON_RESET),boot:0x13", "\x00\xffgarbage",
                  "S,13518,OpenTread fw=1.0.0 build=00000000 bringup=1 lgfx=1.2.26",
                  "S,13519,touchcal;tgt=tl x=notanumber y=1 z=3", "S,13520,touchraw;tgt=c y=5"]
    t = 1_000_000
    if cfg_line:
        lines.append(f"S,{t},touchcfg;reff={r} toff={toff} w={w} h={h} pw=240 ph=320 "
                     f"x_min={compiled['x_min']} x_max={compiled['x_max']} y_min={compiled['y_min']} "
                     f"y_max={compiled['y_max']} zth={zth} inset={inset}")
    csv_rows = ["target,x_raw,y_raw,z,tx,ty"]

    def emit(name, tx, ty, base, extra="", shift=(0, 0)):
        nonlocal t
        rx, ry = _truth_raw(consts, r, cfg, tx + shift[0], ty + shift[1])
        samples = []
        for _ in range(15):
            sxr = min(RAW_VALID_HI, max(RAW_VALID_LO, round(rx + rnd.gauss(0, noise))))
            syr = min(RAW_VALID_HI, max(RAW_VALID_LO, round(ry + rnd.gauss(0, noise))))
            z = round(_z_of(base, sxr, syr) + rnd.gauss(0, 0.4))
            samples.append((sxr, syr, z))
        # touchdown/lift-off outliers (must be trimmed by touchraw grouping)
        def clamp(v):
            return min(RAW_VALID_HI, max(RAW_VALID_LO, v))
        samples[0] = (clamp(samples[0][0] + 400), clamp(samples[0][1] - 300), samples[0][2] - 9)
        samples[-1] = (clamp(samples[-1][0] - 350), clamp(samples[-1][1] + 250), samples[-1][2] - 9)
        core = samples[TRIM:-TRIM]
        mx = statistics.median(s[0] for s in core)
        my = statistics.median(s[1] for s in core)
        mz = statistics.median(s[2] for s in core)
        zmn = min(s[2] for s in core)
        tgt_kv = f"tgt={name}" + (f" tx={tx} ty={ty}" if tx_ty else "") + extra
        if fmt == "raw":
            for sxr, syr, z in samples:
                t += 20_000
                kv = f"{tgt_kv} x={sxr} y={syr} z={z}"
                if sxsy:
                    a, b = lgfx_map(compiled, r, cfg, sxr, syr)
                    kv += f" sx={a} sy={b}"
                lines.append(f"S,{t},touchraw;{kv}")
            t += 500_000
        else:
            t += 400_000
            kv = f"{tgt_kv} x={mx} y={my} z={mz} zmin={zmn} n={len(core)}"
            if sxsy:
                a, b = lgfx_map(compiled, r, cfg, mx, my)
                kv += f" sx={a} sy={b}"
            lines.append(f"S,{t},touchcal;{kv}".replace(" ", ";", 1) if seed % 2 else
                         f"S,{t},touchcal;{kv}")
            csv_rows.append(f"{name + extra.replace(' press=', '_').replace(' finger=', '_')},"
                            f"{mx},{my},{mz},{tx},{ty}")
        return mx, my

    last_screen = None

    def marker(scr):
        # the firmware's screen;<name> S line (a new run) when the screen changes
        nonlocal t, last_screen
        if markers and scr != last_screen:
            t += 1000
            lines.append(f"S,{t},screen;{scr}")
        last_screen = scr

    for name, tx, ty in targets:
        if name in drop:
            continue
        marker("cal" if name in CAL_TARGETS else "grid")
        shift = grid_fault if (grid_fault and name == "g4") else (0, 0)
        if cal_fault and name == "tl":
            shift = cal_fault
        emit(name, tx, ty, Z_BASE[("normal", "dry")] + 0.5, shift=shift)
    for (pr, fi), base in Z_BASE.items() if with_z else ():
        marker("zlog")
        emit("z", w // 2 + 30, h // 2 - 20, base, extra=f" press={pr} finger={fi}")
    if fmt == "csv":
        return csv_rows, cfg
    lines.append("S,99999999,cmd_err;" + "x" * 40)
    lines.append("H,99999999,0,0,0,0,0,0,1")
    return lines, cfg


def _expected_zmin(consts, r, cfg):
    """Independent: weakest true base (light damp) at the worst true screen corner, minus margin.
    The synthetic z has +-0.4 noise and integer rounding, so the tool may land 1-2 lower."""
    worst = math.inf
    for sx in (0, cfg.w - 1):
        for sy in (0, cfg.h - 1):
            rx, ry = _truth_raw(consts, r, cfg, sx, sy)
            worst = min(worst, _z_of(Z_BASE[("light", "damp")], rx, ry))
    return math.floor(worst) - Z_MARGIN


def self_test() -> int:
    fails = []
    ns = make_parser().parse_args([])

    def check(name, cond, detail=""):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}{(' - ' + detail) if detail and not cond else ''}")
        if not cond:
            fails.append(name)

    print("hand-derived LovyanGFX 1.2.26 mapping (Panel_Device::convertRawXY, r=1 and r=3)")
    land = Cfg(w=320, h=240)
    # r=1: affine (300,3700)->(0,0); swap; r&2=0; vflip(1) -> y = 239-0
    check("r=1 raw(300,3700) -> (0,239)", lgfx_map(START_CFG, 1, land, 300, 3700) == (0, 239))
    # r=1: (3900,200)->(239,319); swap (319,239); vflip -> y = 0
    check("r=1 raw(3900,200) -> (319,0)", lgfx_map(START_CFG, 1, land, 3900, 200) == (319, 0))
    # r=3: (300,3700)->(0,0); swap; r&2 -> x = 319; vflip(3) false
    check("r=3 raw(300,3700) -> (319,0)", lgfx_map(START_CFG, 3, land, 300, 3700) == (319, 0))
    # truncation toward zero, not floor: raw just below x_min gives tx = 0 (trunc(-0.06))
    check("trunc toward zero", lgfx_map(START_CFG, 0, Cfg(w=240, h=320), 299, 3700) == (0, 0))
    check("r_eff formula: rot1+poff2+toff0 = 3", cfg_reff(Cfg(rot=1, poff=2, toff=0)) == 3)
    check("r_eff formula: rot1+poff6 = 7 (mirror bit)", cfg_reff(Cfg(rot=1, poff=6)) == 7)

    truths = [  # (r_eff, true consts, screen w,h)
        (1, {"x_min": 280, "x_max": 3880, "y_min": 3720, "y_max": 230}, 320, 240),
        (3, {"x_min": 310, "x_max": 3910, "y_min": 3690, "y_max": 190}, 320, 240),
        (5, {"x_min": 3850, "x_max": 260, "y_min": 240, "y_max": 3750}, 320, 240),
        (7, {"x_min": 3900, "x_max": 300, "y_min": 3700, "y_max": 200}, 320, 240),
        (0, {"x_min": 300, "x_max": 3900, "y_min": 3700, "y_max": 200}, 240, 320),
        (2, {"x_min": 3870, "x_max": 330, "y_min": 210, "y_max": 3680}, 240, 320),
        (4, {"x_min": 300, "x_max": 3900, "y_min": 200, "y_max": 3700}, 240, 320),
        (6, {"x_min": 3900, "x_max": 300, "y_min": 3700, "y_max": 200}, 240, 320),
    ]
    for i, (r, tc, w, h) in enumerate(truths):
        for fmt in ("cal", "raw"):
            print(f"case r_eff={r} {w}x{h} consts={tc} fmt={fmt}")
            lines, cfg = synth(r, tc, w, h, seed=10 * i + (fmt == "raw") + 1, fmt=fmt)
            R = run(lines, ns)
            check("PASS overall", not R.fails, "; ".join(R.fails))
            check("r_eff recovered", R.reff == r, f"got {R.reff}")
            # independent truth for the screen-frame swap / inversion
            exp_swap = bool(r & 1)
            check("swap flag", R.swap == exp_swap, f"got {R.swap}")
            x0 = _truth_raw(tc, r, cfg, 0, 0)
            x1 = _truth_raw(tc, r, cfg, w - 1, 0)
            y1 = _truth_raw(tc, r, cfg, 0, h - 1)
            ax = 1 if exp_swap else 0
            check("x inversion", R.inv and R.inv["x"] == (x1[ax] < x0[ax]))
            check("y inversion", R.inv and R.inv["y"] == (y1[1 - ax] < x0[1 - ax]))
            if R.lg:
                err = max(abs(R.lg[k] - tc[k]) for k in tc)
                check("constants within 40 raw of truth", err <= 40, f"max err {err} {R.lg}")
            check("grid within 3 px", R.grid_max is not None and R.grid_max <= 3, f"{R.grid_max}")
            ez = _expected_zmin(tc, r, cfg)
            check("TOUCH_Z_MIN near independent value",
                  R.zmin_rec is not None and ez - 2 <= R.zmin_rec <= ez,
                  f"got {R.zmin_rec}, expected {ez - 2}..{ez}")

    tc = truths[0][1]
    print("case CSV input (no touchcfg, --reff 1)")
    lines, _ = synth(1, tc, fmt="csv", seed=3)
    R = run(lines, make_parser().parse_args(["--reff", "1"]))
    check("CSV PASS", not R.fails, "; ".join(R.fails))
    check("CSV constants", R.lg and max(abs(R.lg[k] - tc[k]) for k in tc) <= 40, f"{R.lg}")
    check("CSV z_<press>_<finger> labels", not any("missing" in w or "without a" in w for w in R.warns),
          "; ".join(R.warns))

    print("case no rotation info, no sx/sy -> screen frame only")
    lines, _ = synth(1, tc, seed=4, sxsy=False, cfg_line=False)
    R = run(lines, ns)
    check("no LovyanGFX consts", R.lg is None)
    check("still PASS (screen frame grid)", not R.fails and R.grid_max <= 3, "; ".join(R.fails))
    check("warns r_eff unknown", any("r_eff unknown" in w for w in R.warns))

    print("case inferred r_eff from sx/sy with stale start constants (no reff in touchcfg)")
    lines, _ = synth(3, truths[1][1], seed=5)
    lines = [ln.replace("touchcfg;reff=3 ", "touchcfg;") for ln in lines]
    R = run(lines, ns)
    check("inferred r_eff=3", R.reff == 3, f"got {R.reff}")
    check("PASS", not R.fails, "; ".join(R.fails))

    print("fault: grid press 25 px off (g4)")
    lines, _ = synth(1, tc, seed=6, grid_fault=(25, 0))
    R = run(lines, ns)
    check("grid FAIL detected", R.code() == 1 and any("grid error" in f for f in R.fails))

    print("fault: contradictory --reff (2 is invalid for 320x240)")
    lines, _ = synth(1, tc, seed=7)
    R = run(lines, make_parser().parse_args(["--reff", "2"]))
    check("FAIL on incompatible r_eff", R.code() == 1)
    R = run(lines, make_parser().parse_args(["--reff", "3"]))
    check("FAIL when --reff disagrees with sx/sy", R.code() == 1 and any("sx/sy fit" in f for f in R.fails))

    print("fault: stale touchcfg constants (sx/sy computed with other consts)")
    lines, _ = synth(1, tc, seed=8, logged_consts={"x_min": 500, "x_max": 3500, "y_min": 3500,
                                                   "y_max": 400})
    lines = [ln.replace("x_min=500 x_max=3500 y_min=3500 y_max=400",
                        "x_min=300 x_max=3900 y_min=3700 y_max=200") for ln in lines]
    R = run(lines, ns)
    check("model mismatch warned", any("UNVERIFIED" in w for w in R.warns), "; ".join(R.warns))

    print("fault: centre missing, grid missing, --require-grid")
    lines, _ = synth(1, tc, seed=9, drop=("c",) + GRID_TARGETS)
    R = run(lines, make_parser().parse_args(["--require-grid"]))
    check("missing c warned", any("missing: c" in w for w in R.warns))
    check("--require-grid FAIL", R.code() == 1)
    R = run(lines, ns)
    check("without --require-grid PASS", R.code() == 0, "; ".join(R.fails))

    print("fault: censored z (zth at the weakest press)")
    lines, _ = synth(1, tc, seed=11, zth=60)
    R = run(lines, ns)
    check("censoring warned", any("censored" in w for w in R.warns))

    print("defaults: no tx/ty logged")
    lines, _ = synth(1, tc, seed=12, tx_ty=False)
    R = run(lines, ns)
    check("default targets warned", any("DEFAULT target" in w for w in R.warns))
    check("still within tolerance", not R.fails, "; ".join(R.fails))

    print("runs: a redone kc (first run's TL press 45/40 px off) -> only the last run is fitted")
    bad, _ = synth(1, tc, seed=21, markers=True, cal_fault=(45, 40), drop=GRID_TARGETS, with_z=False)
    good, _ = synth(1, tc, seed=22, markers=True, garbage=False)
    R = run(bad + good, make_parser().parse_args(["--require-grid"]))
    check("PASS with the bad run discarded", not R.fails, "; ".join(R.fails))
    check("constants within 40 raw of truth", R.lg and max(abs(R.lg[k] - tc[k]) for k in tc) <= 40, f"{R.lg}")
    check("grid within 3 px", R.grid_max is not None and R.grid_max <= 3, f"{R.grid_max}")
    check("discarded run reported", any("1 other run(s) discarded" in ln for ln in R.lines))
    R1 = run(bad + good, make_parser().parse_args(["--require-grid"]))
    nomark = [ln for ln in bad + good if ",screen;" not in ln]
    R2 = run(nomark, make_parser().parse_args(["--require-grid"]))
    check("same log without screen lines = one run (old behaviour: the bad press skews the fit)",
          R2.lg is not None and R1.lg is not None and R2.lg != R1.lg, f"{R2.lg} vs {R1.lg}")

    print("runs: a failed kg (g4 25 px off) before a clean kg -> only the last grid run is scored")
    g_bad, _ = synth(1, tc, seed=23, markers=True, grid_fault=(25, 0), drop=CAL_TARGETS, with_z=False,
                     garbage=False)
    first, _ = synth(1, tc, seed=24, markers=True, drop=GRID_TARGETS, with_z=False)
    last, _ = synth(1, tc, seed=25, markers=True, drop=CAL_TARGETS, garbage=False)
    R = run(first + g_bad + last, make_parser().parse_args(["--require-grid"]))
    check("PASS with the failed grid run discarded", not R.fails, "; ".join(R.fails))
    check("grid within 3 px", R.grid_max is not None and R.grid_max <= 3, f"{R.grid_max}")
    R = run(first + last + g_bad, make_parser().parse_args(["--require-grid"]))
    check("the LAST grid run decides (failed run last -> FAIL)", R.code() == 1 and
          any("grid error" in f for f in R.fails), "; ".join(R.fails))

    print("z-only capture (no kc in the same capture) -> no TOUCH_Z_MIN")
    zonly, _ = synth(1, tc, seed=26, markers=True, drop=CAL_TARGETS + GRID_TARGETS)
    R = run(zonly, ns)
    check("no recommendation", R.zmin_rec is None, f"got {R.zmin_rec}")
    check("FAIL names the one-capture rule", R.code() == 1 and any("ONE capture" in f for f in R.fails),
          "; ".join(R.fails))
    check("no TOUCH_Z_MIN in the snippet", not any("TOUCH_Z_MIN =" in ln for ln in R.lines))

    print("grid-only capture (no kc) -> FAIL that says kc is needed")
    gonly, _ = synth(1, tc, seed=27, markers=True, drop=CAL_TARGETS, with_z=False)
    R = run(gonly, make_parser().parse_args(["--require-grid"]))
    check("FAIL mentions kc before kg", R.code() == 1 and any("needs a kc run before kg" in f for f in R.fails),
          "; ".join(R.fails))

    print("parser: separators, CR, aliases, tag variants")
    cap = parse_lines(["S,1,touchcal;tgt=UL;tx=20;ty=20;x_raw=100;y_raw=200;size=40\r",
                       "S,2,touchcal tgt=g12 x=1 y=2 z=3", "junk touchcal;tgt=z_firm_damp x=5 y=6 z=7",
                       "S,3,touchcal;tgt=c x=1"])
    got = [(p.tgt, p.kind, p.press, p.finger) for p in cap.presses]
    check("aliases + grid rc + z name", got == [("tl", "cal", None, None), ("g5", "grid", None, None),
                                                 ("z", "z", "firm", "damp")], f"{got}")
    check("malformed counted", cap.n_bad == 1)

    print(f"self-test: {'FAIL (' + ', '.join(fails) + ')' if fails else 'PASS'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
