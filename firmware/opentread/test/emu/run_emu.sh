#!/usr/bin/env bash
# run_emu.sh - host emulation of the real firmware loop with the BLE stack
# emulated. Builds test/emu/emu_main.cpp against the
# REAL app.cpp, debug_serial.cpp, hw_ble*.cpp and src/core (copied into a
# temp folder where test/emu/stubs/hw_display.h replaces the LovyanGFX
# renderer header; hw_display/hw_touch/hw_pulse/hw_loopback .cpp are replaced
# by test/emu/emu_hw.cpp; Arduino/ESP-IDF/FreeRTOS/NimBLE come from
# test/emu/stubs; hw_display.cpp's backlight rules, scripted touch contacts
# and an NVS map are emulated in emu_hw.cpp). Five variants:
#   emu_sim          OT_SIM_INPUT=1 OT_DEV=1 OT_SHORT_TIMERS=1: scripted BLE bench session
#   emu_release      release flags: the S2 log's edges with a watch + an app connected
#   emu_rsc_only     OT_FTMS_ENABLE=0 (+ sim): the RSC-only build (Layout B, no 0x1826)
#   emu_merged_short OT_SHORT_TIMERS=1 OT_DEV=1 (reed edges): UI + BLE together:
#                    touch units toggles (idle + main), wake-swallow while OFF,
#                    DIM/OFF/auto-end wake, two touch hold-stops, a bout
#                    confirming behind a summary, NVS commit / defer / nw3,
#                    all with a watch and an app connected
#   emu_merged       the same scenario with the release flags (production timers)
# Each variant's own checks must pass, then tools/replay.py must pass on
# its captured log (with --ftms 1).
#
# Usage: test/emu/run_emu.sh [--keep <dir>]   (--keep: also copy the logs there)
set -u -o pipefail

EMU_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$(cd "$EMU_DIR/.." && pwd)"
UNIT_DIR="$(cd "$TEST_DIR/.." && pwd)"
ROOT="$(cd "$UNIT_DIR/../.." && pwd)"
PY="$ROOT/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3 || true)"
CXX="${CXX:-clang++}"
KEEP=""
if [ "${1:-}" = "--keep" ] && [ -n "${2:-}" ]; then KEEP="$2"; fi

WORK="$(mktemp -d "${TMPDIR:-/tmp}/opentread_emu.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
B="$WORK/src"
mkdir -p "$B/src/core"
for f in app.cpp app.h config_hw.h version.h debug_serial.cpp debug_serial.h hw_ble.cpp hw_ble.h hw_ble_priv.h \
         hw_ble_rsc.cpp hw_ble_ftms.cpp hw_pulse.h hw_touch.h hw_loopback.h hw_nvs.h; do
  cp "$UNIT_DIR/$f" "$B/$f" || { echo "FAIL: missing $f"; exit 1; }
done
cp "$UNIT_DIR"/src/core/*.h "$UNIT_DIR"/src/core/*.cpp "$B/src/core/"
cp "$EMU_DIR/stubs/hw_display.h" "$B/hw_display.h"   # the renderer API stub

FLAGS=(-std=c++17 -g -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer)
INC=(-I"$B" -I"$B/src/core" -I"$EMU_DIR/stubs" -I"$EMU_DIR" -I"$TEST_DIR")
SRCS=("$B/app.cpp" "$B/debug_serial.cpp" "$B/hw_ble.cpp" "$B/hw_ble_rsc.cpp" "$B/hw_ble_ftms.cpp" "$B"/src/core/*.cpp
      "$EMU_DIR/emu_hw.cpp" "$EMU_DIR/emu_main.cpp" "$TEST_DIR/replay.cpp")
export ASAN_OPTIONS="${ASAN_OPTIONS:-abort_on_error=1}"
export UBSAN_OPTIONS="${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1}"

fail=0
run_variant() {  # name, extra -D flags...
  local name="$1"; shift
  echo "== emu: build $name"
  if ! "$CXX" "${FLAGS[@]}" "$@" "${INC[@]}" "${SRCS[@]}" -o "$WORK/$name"; then
    echo "FAIL: emu build $name"; fail=1; return
  fi
  echo "== emu: run $name"
  if ! "$WORK/$name" "$WORK/$name.log" --root "$ROOT"; then
    echo "FAIL: emu $name checks"; fail=1
  fi
  if [ -n "$KEEP" ]; then mkdir -p "$KEEP" && cp "$WORK/$name.log" "$KEEP/"; fi
  echo "== emu: replay $name"
  if ! "$PY" -I "$ROOT/tools/replay.py" "$WORK/$name.log" --ftms 1 > "$WORK/$name.replay.txt" 2>&1; then
    cat "$WORK/$name.replay.txt"
    echo "FAIL: replay on emu $name"; fail=1
  else
    grep -E '^ *(PASS|SKIP|WARN|FAIL|INVALID)' "$WORK/$name.replay.txt" | sed 's/^/   /' | head -40
  fi
}
run_variant emu_sim -DOT_SIM_INPUT=1 -DOT_DEV=1 -DOT_SHORT_TIMERS=1
run_variant emu_release
run_variant emu_rsc_only -DOT_SIM_INPUT=1 -DOT_FTMS_ENABLE=0
run_variant emu_merged_short -DEMU_MERGED=1 -DOT_SHORT_TIMERS=1 -DOT_DEV=1
run_variant emu_merged -DEMU_MERGED=1

if [ "$fail" -eq 0 ]; then echo "EMULATION PASSED"; else echo "EMULATION FAILED"; fi
exit "$fail"
