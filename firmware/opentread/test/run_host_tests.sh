#!/usr/bin/env bash
# run_host_tests.sh - host test suite for the OpenTread firmware.
#
#   0. checks that every expected test and core source exists (pulse filter,
#      metrics, debug serial, workout, format, touch gesture, ui policy,
#      persist policy, BLE payloads);
#   1. builds every test_*.cpp with src/core/*.cpp and test/replay.cpp using
#      clang++ -Werror + ASan/UBSan and runs it;
#   2. compiles src/core/*.cpp a second time for the ESP32 (32-bit Xtensa GCC
#      from the installed arduino-cli core, -Wall -Wextra -Wconversion -Werror),
#      if that toolchain is present; skipped with a notice otherwise;
#   3. regenerates the goldens with the oracle into a temp dir and fails if
#      `diff -r <tmp> test/golden` is non-empty. The oracle also writes
#      golden/README.md (the format contract), so the diff covers every file;
#   4. test/emu/run_emu.sh: the real loop + BLE adapters on the host
#      with an emulated NimBLE stack and centrals, emulated touch / backlight /
#      NVS (five variants incl. the merged UI + BLE scenario), each log then
#      checked by tools/replay.py; and tools/replay.py --self-test.
#
# Exit code 0 only if every stage passes. Usage: test/run_host_tests.sh
set -u -o pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="$(cd "$TEST_DIR/.." && pwd)"             # firmware/opentread
ROOT="$(cd "$UNIT_DIR/../.." && pwd)"              # repo root
CORE_DIR="$UNIT_DIR/src/core"
PY="$ROOT/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3 || true)"
CXX="${CXX:-clang++}"
FLAGS=(-std=c++17 -g -O1 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
       -Wshorten-64-to-32 -Werror -fsanitize=address,undefined -fno-omit-frame-pointer)

BUILD="$(mktemp -d "${TMPDIR:-/tmp}/opentread_host_build.XXXXXX")"
GOLD_TMP="$(mktemp -d "${TMPDIR:-/tmp}/opentread_golden.XXXXXX")"
trap 'rm -rf "$BUILD" "$GOLD_TMP"' EXIT

fail=0
note() { printf '== %s\n' "$*"; }

export ASAN_OPTIONS="${ASAN_OPTIONS:-abort_on_error=1}"
export UBSAN_OPTIONS="${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1}"

shopt -s nullglob
CORE_SRCS=("$CORE_DIR"/*.cpp)
TESTS=("$TEST_DIR"/test_*.cpp)

# Every expected test and core source must be present (the globs above would
# otherwise skip a missing file silently): filter, metrics, debug serial,
# workout, format (+ its two oracle goldens), touch gesture, ui policy,
# persist policy (their xtensa compiles are covered by the CORE_SRCS glob
# below).
for f in "$TEST_DIR"/test_pulse_filter.cpp "$TEST_DIR"/test_metrics.cpp "$TEST_DIR"/test_debug_serial.cpp \
         "$TEST_DIR"/test_workout.cpp "$TEST_DIR"/test_format.cpp "$TEST_DIR"/test_touch.cpp \
         "$TEST_DIR"/test_ui_policy.cpp "$TEST_DIR"/test_persist.cpp \
         "$CORE_DIR"/pulse_filter.cpp "$CORE_DIR"/metrics.cpp "$CORE_DIR"/workout.cpp "$CORE_DIR"/format.cpp \
         "$CORE_DIR"/touch_gesture.cpp "$CORE_DIR"/ui_policy.cpp "$CORE_DIR"/persist_policy.cpp \
         "$TEST_DIR"/golden/format_pace_sweep.txt "$TEST_DIR"/golden/format_misc.txt; do
  if [ ! -f "$f" ]; then echo "FAIL: missing $f"; fail=1; fi
done
# BLE payloads, link table, CP token (+ the emulation, stage 4).
for f in "$TEST_DIR"/test_ble_payload.cpp "$CORE_DIR"/ble_payload.cpp "$TEST_DIR"/emu/run_emu.sh \
         "$TEST_DIR"/emu/emu_main.cpp "$TEST_DIR"/emu/emu_hw.cpp; do
  if [ ! -f "$f" ]; then echo "FAIL: missing $f"; fail=1; fi
done

# --- 1. host build + run ---------------------------------------------------
for t in "${TESTS[@]}"; do
  name="$(basename "$t" .cpp)"
  note "build $name"
  if ! "$CXX" "${FLAGS[@]}" -I"$CORE_DIR" -I"$TEST_DIR" "${CORE_SRCS[@]}" "$TEST_DIR/replay.cpp" "$t" \
        -o "$BUILD/$name"; then
    echo "FAIL: build $name"; fail=1; continue
  fi
  note "run $name"
  if ! "$BUILD/$name" --root "$ROOT"; then
    echo "FAIL: $name"; fail=1
  fi
done

# --- 2. second compiler: 32-bit ESP32 target ----------------------------------
XT_CANDIDATES=("$HOME"/Library/Arduino15/packages/esp32/tools/esp-x32/*/bin/xtensa-esp32-elf-g++)  # nullglob is on
XT=""
if [ "${#XT_CANDIDATES[@]}" -gt 0 ]; then XT="${XT_CANDIDATES[${#XT_CANDIDATES[@]}-1]}"; fi
if [ -z "$XT" ]; then
  XT="$(command -v xtensa-esp32-elf-g++ || true)"
fi
if [ -n "$XT" ]; then
  note "xtensa (ILP32) compile of src/core with $(basename "$XT")"
  for s in "${CORE_SRCS[@]}"; do
    if ! "$XT" -std=gnu++17 -Os -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror \
          -I"$CORE_DIR" -c "$s" -o "$BUILD/$(basename "$s" .cpp).xt.o"; then
      echo "FAIL: xtensa compile $(basename "$s")"; fail=1
    fi
  done
else
  note "xtensa toolchain not found: 32-bit compile skipped"
fi

# --- 3. oracle-vs-golden drift check -------------------------------------------
note "oracle -> $GOLD_TMP"
if [ ! -x "$PY" ]; then
  echo "FAIL: no Python interpreter (.venv/bin/python or python3) found"; fail=1
elif ! (cd "$UNIT_DIR" && "$PY" -I test/oracle/oracle.py --out "$GOLD_TMP"); then
  echo "FAIL: oracle run"; fail=1
elif ! diff -r "$GOLD_TMP" "$TEST_DIR/golden"; then
  echo "FAIL: oracle output differs from committed test/golden"; fail=1
else
  note "goldens match the oracle"
fi

# --- 4. emulation of the real loop + BLE, and the log checker's self-test ---------
note "emulation (test/emu/run_emu.sh): real app.cpp + hw_ble*.cpp, emulated NimBLE + centrals, touch, backlight, NVS"
if ! "$TEST_DIR/emu/run_emu.sh"; then
  echo "FAIL: emulation"; fail=1
fi
note "tools/replay.py --self-test"
if [ ! -x "$PY" ] || ! "$PY" -I "$ROOT/tools/replay.py" --self-test > "$BUILD/replay_selftest.txt" 2>&1; then
  tail -20 "$BUILD/replay_selftest.txt" 2>/dev/null
  echo "FAIL: replay self-test"; fail=1
else
  tail -1 "$BUILD/replay_selftest.txt"
fi

if [ "$fail" -eq 0 ]; then echo "ALL HOST TESTS PASSED"; else echo "HOST TESTS FAILED"; fi
exit "$fail"
