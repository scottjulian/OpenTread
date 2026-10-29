#!/usr/bin/env bash
# tools/build.sh: build firmware/opentread.
#
# Release builds come from a committed, clean tree. The build ID is the first
# 8 hex digits of `git rev-parse HEAD`, passed as -DOT_BUILD_ID=0x<id8>
# together with -DOT_BUILD_DIRTY=<0|1> and the variant flags, through
# --build-property compiler.cpp.extra_flags (empty in core 3.3.11's
# platform.txt; the script checks that before overriding it).
#
# Output: firmware/opentread/builds/<fw>-<id8>[-<variant>...][-dirty]/
#   <sketch>.ino.bin / .bootloader.bin / .partitions.bin / .elf / .map / .merged.bin
#   build_info.txt   (commit, flags, sizes, libraries, warnings, checks)
# A release build (no variant flags, clean tree) also writes
#   dist/opentread-<fw>.bin (+ .sha256): the merged image, flashed at 0x0.
#
# The build FAILS (exit 1, nothing written to builds/) on:
#   - a dirty tree without --allow-dirty;
#   - any compiler warning from a sketch file (firmware/opentread/**), or a
#     warning that can't be attributed to a file (e.g. "cc1plus: warning:");
#     warnings from library/core files are tolerated and listed;
#   - an object from firmware/opentread/test/ in the linker map;
#   - `attachInterrupt` linked into the ELF (the pulse ISR is installed directly; never use it);
#   - a lint hit in the sketch sources (comments and strings stripped):
#     millis( or attachInterrupt anywhere; `long`, `float`, `double` in src/core.
#
# Usage:
#   tools/build.sh                      release build (+ dist/opentread-<fw>.bin)
#   tools/build.sh --sim                OT_SIM_INPUT=1 variant
#   tools/build.sh --loopback           OT_LOOPBACK=1 variant
#   tools/build.sh --dev                OT_DEV=1 variant (serial dev commands)
#   tools/build.sh --short              OT_SHORT_TIMERS=1 variant
#   tools/build.sh --bringup            OT_BRINGUP=1 variant
#   tools/build.sh --verb N             OT_VERBOSITY_DEFAULT=N (0..2)
#   tools/build.sh --ftms 0|1           OT_FTMS_ENABLE=0|1 (0 = footpod only)
#   tools/build.sh -D NAME=VAL          any other -D flag (repeatable)
#   tools/build.sh --allow-dirty        build from a dirty tree: OT_BUILD_DIRTY=1,
#                                          folder suffix -dirty (git-ignored)
#   tools/build.sh --force              replace an existing output folder
#   tools/build.sh --dry-run            print the plan (flags, folder) and stop
#
# "Dirty" = any staged, unstaged or untracked (not ignored) change anywhere in
# the repo, except firmware/opentread/builds/ (build outputs) logs/
# (captured sessions) and dist/ (release images). None of those can change
# the binary.
#
# Requires: arduino-cli + esp32:esp32 3.3.11, git, and Python 3 (stdlib only:
# the repo's .venv/bin/python if present, else python3) to parse arduino-cli's JSON.
set -euo pipefail

die() { echo "build: $*" >&2; exit 1; }

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null) || die "not inside a git repository"
SKETCH="$REPO/firmware/opentread"
BUILDS="$SKETCH/builds"
PY="$REPO/.venv/bin/python"; [[ -x "$PY" ]] || PY=$(command -v python3 || true)
FQBN="esp32:esp32:esp32:UploadSpeed=115200"

usage() { sed -n '2,/^set -euo pipefail$/p' "${BASH_SOURCE[0]}" | sed -e '$d' -e 's/^# \{0,1\}//'; }

allow_dirty=0; force=0; dry_run=0
v_sim=0; v_loopback=0; v_dev=0; v_short=0; v_bringup=0; v_verb=""; v_ftms=""
custom_flags=()

check_flag() {   # NAME or NAME=VAL; no spaces, quotes or shell metacharacters
  [[ "$1" =~ ^[A-Za-z_][A-Za-z0-9_]*(=[A-Za-z0-9_.x+-]*)?$ ]] || die "bad -D flag '$1' (NAME or NAME=VAL, [A-Za-z0-9_.x+-] only)"
  case "${1%%=*}" in
    OT_BUILD_ID|OT_BUILD_DIRTY) die "-D ${1%%=*} is set by this script, not by hand" ;;
    OT_SIM_INPUT|OT_LOOPBACK|OT_DEV|OT_SHORT_TIMERS|OT_BRINGUP|OT_VERBOSITY_DEFAULT|OT_FTMS_ENABLE)
      die "use the named option for ${1%%=*} (see --help)" ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --allow-dirty) allow_dirty=1 ;;
    --force) force=1 ;;
    --dry-run) dry_run=1 ;;
    --sim) v_sim=1 ;;
    --loopback) v_loopback=1 ;;
    --dev) v_dev=1 ;;
    --short) v_short=1 ;;
    --bringup) v_bringup=1 ;;
    --verb) [[ $# -ge 2 && "$2" =~ ^[0-2]$ ]] || die "--verb needs 0, 1 or 2"; v_verb=$2; shift ;;
    --ftms) [[ $# -ge 2 && "$2" =~ ^[01]$ ]] || die "--ftms needs 0 or 1"; v_ftms=$2; shift ;;
    -D) [[ $# -ge 2 ]] || die "-D needs NAME=VAL"; check_flag "$2"; custom_flags+=("$2"); shift ;;
    -D*) check_flag "${1#-D}"; custom_flags+=("${1#-D}") ;;
    *) die "unknown argument '$1' (see --help)" ;;
  esac
  shift
done

[[ -d "$SKETCH" ]] || die "sketch folder missing: $SKETCH"
[[ -f "$SKETCH/opentread.ino" ]] || die "missing $SKETCH/opentread.ino"
[[ -n "$PY" && -x "$PY" ]] || die "python3 not found (needed to parse arduino-cli output)"
command -v arduino-cli >/dev/null || die "arduino-cli not on PATH"

# ---- firmware version (version.h) -------------------------------------------
[[ -f "$SKETCH/version.h" ]] || die "missing $SKETCH/version.h"
FW=$(sed -nE 's/^[[:space:]]*#[[:space:]]*define[[:space:]]+OT_FW_VERSION[[:space:]]+"([^"]*)".*/\1/p' "$SKETCH/version.h")
[[ $(printf '%s\n' "$FW" | grep -c .) -eq 1 && "$FW" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] \
  || die "version.h must define OT_FW_VERSION \"<major>.<minor>.<patch>\" exactly once (got '${FW}')"

# ---- build identity ---------------------------------------------------------
HEAD_SHA=$(git -C "$REPO" rev-parse --verify HEAD 2>/dev/null) || die "no commit yet (git rev-parse HEAD failed)"
ID8=${HEAD_SHA:0:8}
[[ "$ID8" =~ ^[0-9a-f]{8}$ ]] || die "unexpected HEAD sha '$HEAD_SHA'"
DIRTY_LIST=$(git -C "$REPO" status --porcelain=v1 --untracked-files=all -- . \
  ':(exclude)firmware/opentread/builds' ':(exclude)logs' ':(exclude)dist')
DIRTY=0
if [[ -n "$DIRTY_LIST" ]]; then
  if [[ $allow_dirty -ne 1 ]]; then
    echo "$DIRTY_LIST" | sed 's/^/    /' >&2
    die "refusing to build: the tree is dirty (commit first, or pass --allow-dirty for a dev build marked dirty=1)"
  fi
  DIRTY=1
fi

# ---- flags and output folder ------------------------------------------------
FLAGS=("-DOT_BUILD_ID=0x$ID8" "-DOT_BUILD_DIRTY=$DIRTY")
SUFFIX=""
[[ $v_sim -eq 1 ]]      && { FLAGS+=("-DOT_SIM_INPUT=1");      SUFFIX+="-sim"; }
[[ $v_loopback -eq 1 ]] && { FLAGS+=("-DOT_LOOPBACK=1");       SUFFIX+="-loopback"; }
[[ $v_dev -eq 1 ]]      && { FLAGS+=("-DOT_DEV=1");            SUFFIX+="-dev"; }
[[ $v_short -eq 1 ]]    && { FLAGS+=("-DOT_SHORT_TIMERS=1");   SUFFIX+="-short"; }
[[ $v_bringup -eq 1 ]]  && { FLAGS+=("-DOT_BRINGUP=1");        SUFFIX+="-bringup"; }
[[ -n "$v_verb" ]]      && { FLAGS+=("-DOT_VERBOSITY_DEFAULT=$v_verb"); SUFFIX+="-v$v_verb"; }
[[ -n "$v_ftms" ]]      && { FLAGS+=("-DOT_FTMS_ENABLE=$v_ftms");       SUFFIX+="-ftms$v_ftms"; }
for f in ${custom_flags[@]+"${custom_flags[@]}"}; do
  FLAGS+=("-D$f")
  SUFFIX+="-x$(printf '%s' "$f" | tr 'A-Z=' 'a-z-' | tr -c 'a-z0-9.+-' '_')"
done
[[ $DIRTY -eq 1 ]] && SUFFIX+="-dirty"
EXTRA="${FLAGS[*]}"
OUT="$BUILDS/$FW-$ID8$SUFFIX"
RELEASE=1
[[ -n "$SUFFIX" ]] && RELEASE=0   # any variant flag, custom flag or dirty tree

echo "build: fw=$FW build=$ID8 dirty=$DIRTY release_flag_set=$RELEASE"
echo "build: flags: $EXTRA"
echo "build: output: ${OUT#"$REPO"/}"
if [[ -e "$OUT" ]]; then
  if [[ $force -ne 1 ]]; then
    die "output folder exists: ${OUT#"$REPO"/} (a rebuild changes the embedded compile time; pass --force to replace it)"
  fi
fi
[[ $dry_run -eq 1 ]] && { echo "build: dry run, nothing compiled"; exit 0; }

TMP=$(mktemp -d "${TMPDIR:-/tmp}/opentread_build.XXXXXX")
trap 'rm -rf -- "$TMP"' EXIT

# ---- the override must not drop anything (core defaults must be empty) -----
DEFAULT_EXTRA=$(arduino-cli compile --fqbn "$FQBN" --show-properties "$SKETCH" 2>/dev/null \
  | grep -E '^compiler\.cpp\.extra_flags=' || true)
[[ "$DEFAULT_EXTRA" == "compiler.cpp.extra_flags=" ]] \
  || die "compiler.cpp.extra_flags is not empty in this core ('$DEFAULT_EXTRA'); overriding it would drop flags"

# ---- compile (always --clean: a cached object prints no warnings) ----------
echo "build: compiling (clean, --warnings all)..."
set +e
arduino-cli compile --clean --format json --fqbn "$FQBN" --warnings all \
  --output-dir "$TMP/out" --build-property "compiler.cpp.extra_flags=$EXTRA" \
  "$SKETCH" >"$TMP/compile.json" 2>"$TMP/compile.stderr"
CC_RC=$?
set -e

# ---- analyse: warnings, sizes, libraries, map/ELF/lint checks --------------
set +e
"$PY" -I - "$TMP/compile.json" "$TMP/compile.stderr" "$CC_RC" "$SKETCH" "$TMP/out" \
  "$FW" "$ID8" "$HEAD_SHA" "$DIRTY" "$RELEASE" "$EXTRA" "$FQBN" <<'PY'
import json, os, re, subprocess, sys

(json_path, stderr_path, cc_rc, sketch, out_dir, fw, id8, sha, dirty,
 release, extra, fqbn) = sys.argv[1:]
sketch = os.path.realpath(sketch)
fail = []
info = []

raw = open(json_path, encoding="utf-8", errors="replace").read()
side = open(stderr_path, encoding="utf-8", errors="replace").read()
try:
    d = json.loads(raw)
except ValueError:
    sys.stdout.write(raw + side)
    print(f"build: FAIL: arduino-cli did not return JSON (rc={cc_rc})")
    sys.exit(1)

text = (d.get("compiler_out") or "") + "\n" + (d.get("compiler_err") or "") + "\n" + side
sys.stdout.write(d.get("compiler_out") or "")
sys.stdout.write(d.get("compiler_err") or "")
sys.stdout.write(side)
br = d.get("builder_result") or {}
props = {}
for p in br.get("build_properties") or []:
    k, _, v = p.partition("=")
    props[k] = v

if cc_rc != "0" or not d.get("success"):
    print(f"build: FAIL: compile failed (rc={cc_rc})")
    sys.exit(1)

# -- warnings ---------------------------------------------------------------
home = os.path.expanduser("~")
lib_roots = [os.path.join(home, "Library", "Arduino15"),
             os.path.join(home, "Documents", "Arduino", "libraries")]
for lib in br.get("used_libraries") or []:
    if lib.get("install_dir"):
        lib_roots.append(lib["install_dir"])
for k in ("runtime.platform.path", "compiler.sdk.path", "compiler.path"):
    if props.get(k):
        lib_roots.append(props[k])
lib_roots = [os.path.realpath(r) for r in lib_roots]
build_path = os.path.realpath(br.get("build_path") or props.get("build.path") or "/nonexistent")

loc_re = re.compile(r"^(?P<path>[^\s:][^:]*?):(?P<line>\d+):(?:\d+:)? warning: (?P<msg>.*)$")
tool_re = re.compile(r"^(?P<tool>[^\s:][^:]*?): warning: (?P<msg>.*)$")
sketch_w, lib_w, unknown_w = [], [], []
seen = set()
for line in text.splitlines():
    m = loc_re.match(line)
    if m:
        if line in seen:
            continue
        seen.add(line)
        path = os.path.realpath(m.group("path"))
        if path.startswith(sketch + os.sep) or path.startswith(os.path.join(build_path, "sketch") + os.sep):
            sketch_w.append(line)
        elif any(path.startswith(r + os.sep) for r in lib_roots):
            lib_w.append(line)
        else:
            unknown_w.append(line)
        continue
    m = tool_re.match(line)
    if m and line not in seen:
        seen.add(line)
        tool = os.path.realpath(m.group("tool")) if os.sep in m.group("tool") else m.group("tool")
        if any(tool.startswith(r + os.sep) for r in lib_roots):
            lib_w.append(line)      # e.g. the toolchain's ld
        else:
            unknown_w.append(line)  # e.g. "cc1plus: warning:" (a flag problem)
if sketch_w:
    fail.append(f"{len(sketch_w)} warning(s) from sketch files")
if unknown_w:
    fail.append(f"{len(unknown_w)} warning(s) not attributable to a library/core file")

# -- effective flags --------------------------------------------------------
eff = props.get("compiler.cpp.extra_flags")
if eff is not None and eff != extra:
    fail.append(f"effective compiler.cpp.extra_flags {eff!r} != requested {extra!r}")

# -- sizes and libraries ----------------------------------------------------
sizes = {s.get("name"): s for s in br.get("executable_sections_size") or []}
def pct(s):
    return f"{s['size']} B of {s['max_size']} ({100.0 * s['size'] / s['max_size']:.1f}%)" if s.get("max_size") else f"{s.get('size')} B"
flash = pct(sizes["text"]) if "text" in sizes else "?"
ram = pct(sizes["data"]) if "data" in sizes else "?"
libs = [f"{l.get('name')} {l.get('version')}" for l in br.get("used_libraries") or []]

# -- map: no object from test/ ---------------------------------------------
maps = [f for f in os.listdir(out_dir) if f.endswith(".map")]
elfs = [f for f in os.listdir(out_dir) if f.endswith(".elf")]
objs = set()
if len(maps) != 1:
    fail.append(f"expected 1 .map in the output, found {maps}")
else:
    # Sketch objects appear as .../sketch/<dir/><name>.o or, when arduino-cli
    # archives each folder's objects (a long link command line, e.g. from a
    # deep worktree path), as .../sketch/<dir/>objs.a(<name>.o); both are
    # recorded as <dir/><name>.o. The test/ check also matches the test
    # sources' basenames, in case an archive ever hides the folder.
    for m in re.finditer(r"/sketch/([^\s()]*?)(?:objs\.a\()?([^\s()/]+?\.o)\b",
                         open(os.path.join(out_dir, maps[0]), errors="replace").read()):
        objs.add(m.group(1) + m.group(2))
    test_dir = os.path.join(sketch, "test")
    test_names = {f + ".o" for f in os.listdir(test_dir) if f.endswith(".cpp")} if os.path.isdir(test_dir) else set()
    test_objs = sorted(o for o in objs if o.startswith("test/") or "/test/" in o or o in test_names)
    if test_objs:
        fail.append(f"test/ objects linked into the firmware: {test_objs}")
    if not objs:
        fail.append("no sketch objects found in the map (map format changed?)")

# -- ELF: attachInterrupt must not be linked; report the IRAM ISR service ---
isr_service = "?"
if len(elfs) != 1:
    fail.append(f"expected 1 .elf in the output, found {elfs}")
else:
    nm = os.path.join(props.get("compiler.path", ""), "xtensa-esp32-elf-nm")
    try:
        syms = subprocess.run([nm, os.path.join(out_dir, elfs[0])], capture_output=True,
                              text=True, check=True).stdout
        names = {ln.split()[-1] for ln in syms.splitlines() if ln.strip()}
        bad = sorted(n for n in names if "attachInterrupt" in n)
        if bad:
            fail.append(f"attachInterrupt linked into the ELF: {bad}")
        isr_service = "yes" if "gpio_install_isr_service" in names else "no"
    except (OSError, subprocess.CalledProcessError) as e:
        fail.append(f"nm failed: {e}")

# -- source lint (comments and string/char literals stripped) --------------
def strip_code(src):
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            seg = src[i: n if j < 0 else j + 2]
            out.append("\n" * seg.count("\n"))
            i = n if j < 0 else j + 2
        elif c == "'" and i > 0 and src[i - 1].isalnum():
            out.append(c)           # C++14 digit separator (1'000'000), not a char literal
            i += 1
        elif c in "\"'":
            j = i + 1
            while j < n and src[j] != c and src[j] != "\n":
                j += 2 if src[j] == "\\" else 1
            out.append(c + c)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)

lint = []
everywhere = [(re.compile(r"\bmillis\s*\("), "millis()"),
              (re.compile(r"\battachInterrupt\w*\b"), "attachInterrupt")]
core_only = [(re.compile(r"\blong\b"), "long"), (re.compile(r"\b(?:float|double)\b"), "float/double")]
for root, dirs, files in os.walk(sketch):
    rel_root = os.path.relpath(root, sketch)
    if rel_root == "." :
        dirs[:] = [x for x in dirs if x == "src"]   # arduino-cli compiles only the root and src/
    for fn in sorted(files):
        if not fn.endswith((".ino", ".h", ".hpp", ".c", ".cpp", ".S")):
            continue
        p = os.path.join(root, fn)
        rel = os.path.relpath(p, sketch)
        code = strip_code(open(p, encoding="utf-8", errors="replace").read())
        rules = everywhere + (core_only if rel.startswith("src" + os.sep + "core" + os.sep) else [])
        for no, ln in enumerate(code.splitlines(), 1):
            for rx, what in rules:
                if rx.search(ln):
                    lint.append(f"{rel}:{no}: {what}: {ln.strip()}")
if lint:
    fail.append(f"{len(lint)} lint hit(s) (millis/attachInterrupt; long/float/double in src/core)")

# -- report + build_info.txt -----------------------------------------------
lines = [
    f"fw={fw}", f"build={id8}", f"commit={sha}", f"dirty={dirty}",
    f"release_flag_set={release}", f"fqbn={fqbn}", f"extra_flags={extra}",
    f"flash={flash}", f"static_ram={ram}",
    f"libraries={', '.join(libs) if libs else '(none)'}",
    f"core={props.get('runtime.platform.path', '?').rstrip('/').split('/')[-1]}",
    f"sketch_objects={' '.join(sorted(objs))}",
    f"isr_service_linked={isr_service}",
    f"sketch_warnings={len(sketch_w)}", f"unattributed_warnings={len(unknown_w)}",
    f"library_warnings={len(lib_w)}",
]
lines += [f"library_warning: {w}" for w in lib_w]
lines += [f"sketch_warning: {w}" for w in sketch_w + unknown_w]
lines += [f"lint: {x}" for x in lint]
lines.append("result=" + ("FAIL: " + "; ".join(fail) if fail else "PASS"))
with open(os.path.join(out_dir, "build_info.txt"), "w", encoding="utf-8", newline="\n") as fh:
    fh.write("\n".join(lines) + "\n")
print("-" * 60)
for x in lines:
    print("build: " + x)
sys.exit(1 if fail else 0)
PY
AN_RC=$?
set -e
[[ $AN_RC -eq 0 ]] || die "build REJECTED (nothing written to ${BUILDS#"$REPO"/}/)"

# ---- publish -------------------------------------------------------------
mkdir -p "$BUILDS"
if [[ -e "$OUT" ]]; then
  case "$OUT" in "$BUILDS"/[0-9]*.*-????????*) rm -rf -- "$OUT" ;; *) die "refusing to remove unexpected path $OUT" ;; esac
fi
mv -- "$TMP/out" "$OUT"
echo "build: OK -> ${OUT#"$REPO"/}"
echo "build: upload: arduino-cli upload -p /dev/cu.usbserial-<n> --fqbn $FQBN --input-dir ${OUT#"$REPO"/}"

# ---- release image: one file (bootloader + partitions + app), flashed at 0x0 --
if [[ $RELEASE -eq 1 ]]; then
  MERGED="$OUT/opentread.ino.merged.bin"
  [[ -f "$MERGED" ]] || die "release build has no merged image ($MERGED)"
  mkdir -p "$REPO/dist"
  REL="$REPO/dist/opentread-$FW.bin"
  cp -- "$MERGED" "$REL"
  (cd "$REPO/dist" && shasum -a 256 "opentread-$FW.bin" > "opentread-$FW.bin.sha256")
  echo "build: release image -> ${REL#"$REPO"/} (flash at 0x0)"
fi
