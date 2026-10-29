# OpenTread

An open-source replacement console for the **AssaultRunner** curved manual treadmill.

A cheap ESP32 touch-screen board reads the treadmill's speed sensor directly, shows
your run, and broadcasts it over Bluetooth as a **running footpod** (for your watch)
and as an **FTMS treadmill** (for apps). Your watch records a native treadmill run
with real belt speed and distance, then syncs it to Garmin Connect, Strava, or any
other service your watch uses.

## Why

The AssaultRunner was sold with the promise of a companion app for recording
workouts and syncing them to Garmin and Strava. That app was never released. The
stock console does have Bluetooth, but:

- it only advertises after you press **CONNECT**, and drops the link if the belt
  doesn't move within about 30 s,
- its distance reads about **14% long** compared with true belt travel,
- watches can't use it as a footpod, so your watch guesses treadmill pace from your
  wrist.

OpenTread fixes this with about $15 of hardware and two soldered wires.

## Features

- Big **pace** display, plus speed, distance, moving time and total time
- **True belt distance** (counted from the belt's reed sensor, 0.5974 m per pulse)
- **Auto start** once you've been moving for 5 s, **auto pause** when you stop,
  **hold-to-stop**, and a 2-minute summary screen
- Tap the pace to switch **mi / km** (remembered after power-off)
- Screen dims and sleeps on its own; the workout auto-ends after 15 min idle
- Bluetooth **RSC footpod** and **FTMS treadmill** at the same time
- Touch screen only; no buttons, no app, no account

## Compatibility

| | Status |
|---|---|
| AssaultRunner Pro console (sticker *AirRunner 23-AS-514*) | ✅ Tested |
| Garmin Forerunner 955 (as a BLE footpod) | ✅ Tested |
| Other curved manual treadmills with a 2-wire reed speed sensor | Likely, but you may need to change the distance-per-pulse constant |
| Other watches that pair a BLE running footpod (RSC) | Likely |
| Apps that connect to an FTMS treadmill | Likely |

If you try it on something new, please open an issue and tell us how it went.

## Hardware

| Part | Notes |
|---|---|
| **CYD** board, ESP32-2432S028R ("Cheap Yellow Display") | ESP32 with a 2.8" 320×240 resistive touch screen. The common micro-USB version uses an ILI9341 panel; for the ST7789 two-USB variant, see [Build from source](#option-b-build-from-source). |
| Two short wires, and the CN1 cable that comes with the CYD | Connect the treadmill's sensor line to the CYD |
| 5 V USB wall adapter (≥ 1 A) and a USB cable | Powers the CYD |
| Soldering iron, multimeter | For the tap and to check the wiring |

You don't need to buy anything from Assault Fitness, and the stock console can
stay connected.

### Wiring

The treadmill's speed sensor is a reed switch in the base. Its cable plugs into a
2-pin socket on the back of the console's circuit board. Solder two wires to the
back of that socket (the **tap**) and run them to the CYD's **CN1** connector:

| CYD CN1 | Tap wire |
|---|---|
| **GND** | sensor GND |
| **GPIO 27** | sensor SIG |
| GPIO 22, 3.3V | leave unconnected |

> [!IMPORTANT]
> **Check which wire is which with a meter before connecting.** With the console
> powered, put the black probe on the GND wire: the SIG wire should read about
> **+3.2 V**, and drop to 0 V when the reed closes. If GND and SIG are swapped, the
> CYD may still count pulses, but the stock console stops seeing the belt.
>
> **Never connect the console's 3.3 V to the CYD.** Two regulators would fight.

You can leave the stock console plugged in. OpenTread just listens on the same
line, and the console keeps working. If you remove the console, the line has no
pull-up. If the CYD's internal pull-up isn't enough (this hasn't been tested yet),
add a 10 kΩ resistor from GPIO 27 to 3.3 V on the CYD side.

Power the CYD from the USB wall adapter, and keep the cable clear of the belt.

## Install the firmware

### Option A: prebuilt binary (easiest)

1. Download `opentread-<version>.bin` (for example `opentread-1.0.0.bin`) from the
   [Releases](https://github.com/scottjulian/OpenTread/releases) page.
2. Plug the CYD into your computer over USB. It uses a CH340 USB-serial chip, and
   some systems need its driver installed. The port shows up as
   `/dev/cu.usbserial-*` (macOS), `/dev/ttyUSB0` (Linux) or `COMx` (Windows).
3. Flash it, using either
   - the browser: open [ESP Tool](https://espressif.github.io/esptool-js/) in
     Chrome or Edge, connect at **115200** baud, and write the file at address
     **0x0**, or
   - the command line:
     ```bash
     pip install esptool
     python -m esptool --chip esp32 --port <port> --baud 115200 write_flash 0x0 opentread-<version>.bin
     ```

> [!NOTE]
> Use **115200** baud. Faster speeds garble on the CYD's CH340 chip.

### Option B: build from source

Requires [`arduino-cli`](https://arduino.github.io/arduino-cli/). The pinned versions
below are the ones OpenTread is tested with.

```bash
# one-time setup
arduino-cli core install esp32:esp32@3.3.11 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "NimBLE-Arduino@2.5.0" "LovyanGFX@1.2.26"

# build and flash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/opentread
arduino-cli upload  --fqbn esp32:esp32:esp32:UploadSpeed=115200 -p <port> firmware/opentread
```

- **ST7789 CYD variant:** add
  `--build-property "compiler.cpp.extra_flags=-DCYD_PANEL_ST7789=1"` to the compile
  command.
- **Use NimBLE-Arduino 2.x.** Version 1.4.x compiles against ESP32 core 3.x, but
  crash-loops at boot.

## Pair your watch

**Garmin:** Settings → Sensors & Accessories → Add New → **Foot Pod** → `OpenTread`.
Leave calibration at 100%. Then start a **Treadmill** activity as usual. The watch
takes speed and distance from OpenTread and records heart rate itself.

**Apps:** look for an FTMS treadmill called `OpenTread`.

## Using it

Just start running. The workout starts after 5 s of movement and pauses when you
stop. To finish, **press and hold the HOLD TO STOP strip** at the bottom of the
screen for 2 s. Tap the big pace number to switch units.

The watch's start/stop is separate from the console's, so start and stop each one
yourself.

The [user guide](docs/USER_GUIDE.md) has the details, plus how to report a
problem. To work on the firmware, see [DEVELOPMENT.md](docs/DEVELOPMENT.md).

## Disclaimer

OpenTread is a hobby project and is **not affiliated with or endorsed by Assault
Fitness**. AssaultRunner and other trademarks belong to their owners. Opening the
console and soldering to it may void your warranty. You use this at your own risk.
The software comes with **no warranty**; see the license.

## License

Copyright (C) 2026 Scott Julian

OpenTread is free software, licensed under the
[GNU General Public License v3.0](LICENSE). You may use, modify and share it, but
anything you distribute that is based on it must also be released under the GPL.
