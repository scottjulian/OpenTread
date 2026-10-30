# Developing OpenTread

How the firmware is laid out, how to test it, and how releases are built.
For flashing a finished build, see the [README](../README.md).

## Layout

```
firmware/opentread/        Arduino sketch (folder name = sketch name)
├── opentread.ino          forwards setup()/loop() to app.cpp
├── app.cpp                the main loop: owns every core object, wires inputs to outputs
├── config_hw.h            pins, build flags, measured hardware constants
├── hw_*.cpp/.h            hardware adapters: pulse ISR, display, touch, BLE, NVS, loopback
├── debug_serial.*         the single serial writer (115200 baud log + dev commands)
├── version.h              OT_FW_VERSION, the semantic version
├── src/core/              pure C++ logic, no Arduino headers (host-testable):
│                          pulse filter, metrics, workout state machine, formatting,
│                          touch gestures, UI policy, settings persistence, BLE payloads
└── test/                  host tests (never linked into the firmware)
    ├── test_*.cpp         unit tests for src/core
    ├── oracle/oracle.py   independent Python reference model; writes test/golden/
    ├── golden/            expected outputs (README.md there is the format contract)
    ├── data/              real sensor captures used as test input
    └── emu/               runs the real app.cpp + BLE code on the host against
                           an emulated NimBLE stack, touch, backlight and NVS
tools/
├── build.sh               checked build (warnings, lint, map/ELF checks) + release image
├── capture.py             record the head unit's serial log to logs/
├── replay.py              offline checks of a captured log against the reference model
├── ble_check.py           connect from a computer and check the BLE services
├── touch_cal.py           touch calibration from a bring-up log
├── pulse_gen/             sketch for a second ESP32 that emulates the reed sensor
└── render_screens/        renders the real screens to PNG on a desktop (docs/img/)
```

Rule of thumb: logic goes in `src/core/` (testable on the host); `hw_*` files
only move data between the hardware and the core.

## Setup

- [`arduino-cli`](https://arduino.github.io/arduino-cli/), ESP32 core **3.3.11**,
  **NimBLE-Arduino 2.5.0**, **LovyanGFX 1.2.26** (see the README for the
  install commands). The firmware checks the library versions at compile time.
- Python 3 for the tools and tests:
  ```bash
  python3 -m venv .venv
  .venv/bin/pip install -r requirements.txt
  ```
- `clang++` for the host tests.

## Tests

```bash
firmware/opentread/test/run_host_tests.sh
```

This takes a few minutes and must end with `ALL HOST TESTS PASSED`. It runs:

1. every `test_*.cpp` with clang++ `-Werror` and ASan/UBSan;
2. a second compile of `src/core` with the ESP32 (Xtensa) compiler, if the
   Arduino core is installed;
3. the oracle, which regenerates the goldens, and a diff against `test/golden/`;
4. the emulation (`test/emu/run_emu.sh`): five build variants of the real loop,
   each log then checked by `tools/replay.py`, plus `replay.py --self-test`.

If you change behavior on purpose, update the oracle as well, regenerate the
goldens, and commit both:

```bash
cd firmware/opentread
T=$(mktemp -d) && ../../.venv/bin/python -I test/oracle/oracle.py --out "$T" && cp "$T"/* test/golden/
```

## Building

```bash
tools/build.sh            # release build from a clean, committed tree
tools/build.sh --help     # variants and options
```

`build.sh` compiles with all warnings on, and rejects the build on any warning
from the sketch, a test object in the link map, `attachInterrupt` in the ELF,
or a lint hit. Output goes to `firmware/opentread/builds/<version>-<commit>/`
(git-ignored). The build ID (first 8 hex digits of the commit) is embedded in the binary and
reported in the serial banner and the BLE firmware revision (`1.0.0+<id>`).

Variants for bench work (each adds a suffix to the output folder):

| Option | Effect |
|---|---|
| `--sim` | simulated belt input, no sensor needed |
| `--dev` | enables serial dev commands |
| `--short` | short timers (dim / off / auto-end) for quick tests |
| `--loopback` | built-in pulse generator on GPIO 22, jumpered to GPIO 27 (sensor unplugged) |
| `--bringup` | display/touch bring-up and touch calibration screens |
| `--ftms 0` | footpod (RSC) only, no FTMS service |
| `--verb N` | default serial verbosity 0..2 |
| `--allow-dirty` | build from uncommitted changes (marked dirty) |

Flash a build folder with:

```bash
arduino-cli upload -p <port> --fqbn esp32:esp32:esp32:UploadSpeed=115200 \
  --input-dir firmware/opentread/builds/<folder>
```

## Screenshots

```bash
tools/render_screens/render.sh            # writes docs/img/screen_*.png (2x scale)
```

This compiles the real renderer (`hw_display.cpp`, unchanged) with LovyanGFX on the
desktop. The panel is a 320×240 sprite instead of the SPI display, and the
test/emu stubs supply the clock. It feeds the renderer sample frames the way
`app.cpp` does, then saves the panel as PNG. The scenes and their numbers are in
`tools/render_screens/main.cpp`. It needs clang, zlib and the same LovyanGFX
1.2.26 (`LGFX_DIR` overrides where it's found), but no SDL. Re-run it after any UI
change and commit the images.

## Releasing

1. Set `OT_FW_VERSION` in `firmware/opentread/version.h` and commit.
2. Run the host tests.
3. `tools/build.sh` on the clean tree. A release build (no variant flags) also
   writes `dist/opentread-<version>.bin` and a `.sha256` file. That is the merged
   image (bootloader + partitions + app), flashed at `0x0`.
4. Tag `v<version>` and attach the `dist/` files to a GitHub release.

## Serial log

At 115200 baud the head unit writes one record per line, `<type>,<t_us>,...`:
raw edges (`E`), accepted pulses (`P`), metrics (`K`), workout events (`W`),
BLE payloads (`R`, `X`), status text (`S`), health (`H`, `G`), and more. The
first `S` line of each boot is the banner, `OpenTread fw=<version> build=<id> ...`.
`test/golden/README.md` and the comments in `debug_serial.h` and `app.cpp`
describe the formats. Capture a log with `tools/capture.py`, then check it with
`tools/replay.py`.
