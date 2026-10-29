# pulse_gen: bench pulse generator (second ESP32)

An independent reed-switch emulator for bench-testing the OpenTread head unit
without a treadmill. It runs on a **second plain ESP32 dev board**
(ESP32-WROOM-32, CP2102, FQBN `esp32:esp32:esp32`), never on the CYD. Because
it is independent of the device under test, it is the preferred source for
stress tests (the 1/3/6/10/15 pulses/s sweep and 30 min at 15 pulses/s).

## Wiring

| pulse_gen board | CYD (head unit) | Notes |
|---|---|---|
| **GPIO 25** | **GPIO 27** (CN1 signal) | The generator's output. Open-drain: it only ever pulls the line to GND or lets go. |
| **GND** | **GND** (CN1 ground) | **Common ground is required.** Don't rely on the two USB cables for it. |
| USB | | Its own serial port (`/dev/cu.SLAB_USBtoUART` or `/dev/cu.usbserial-*`, CP2102), 115200 baud |

- **Unplug the treadmill harness from CN1 first.** The generator replaces the
  reed switch. Don't run it in parallel with the console.
- The HIGH level comes from the CYD's GPIO 27 pull-up (`INPUT_PULLUP`), as it
  does with the real reed. The generator never drives the line HIGH, so a
  wiring mistake can't push 3.3 V into anything.
- Power both boards from USB **before** connecting the jumper wires.
- GPIO 25 is not a strapping pin on the plain ESP32 dev board, so it is safe
  at reset. (On the CYD, GPIO 25 is the touch CLK. That doesn't matter here:
  this sketch never runs on the CYD.)

## Build and flash

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all tools/pulse_gen
arduino-cli upload -p /dev/cu.SLAB_USBtoUART --fqbn esp32:esp32:esp32 tools/pulse_gen
```

No libraries are needed. The sketch compiles with zero warnings on core 3.3.11
with `--warnings all`. Only one process may hold a serial port at a time.

## Commands (its own serial port, 115200, one per line)

| Command | Effect |
|---|---|
| `g<rate>,<count>` | Start a train of `<count>` closures at `<rate>` closures/s. Rate 0.1–40, up to 3 decimals (`g10,600`, `g2.5,100`). Count 1–10,000,000. Rejected while a train runs. |
| `g0` | Stop now. Releases the line. |
| `b0` / `b1` | Bounce bursts off / on (default on). Not while running. |
| `?` | Status line. |

Every accepted command is echoed as `S,t,cmd;<text>`. Anything else gets
`S,t,cmd_err;<text>` and has no effect. Commas inside S text are shown as `;`,
following the head unit's log format.

## Output

```
S,<t>,pulse_gen v1 pin=25 mode=open_drain baud=115200 bounce=1 ...   boot banner
S,<t>,gen_start;rate_mhz=10000;count=600;period_us=100000;dwell_us=40000;bounce=1
C,<t>,<k>                     closure k started (scheduled time of its first falling edge)
S,<t>,gen_done;600;edges=3280;aborted=0;print_drop=0;max_late_us=<l>
```

- `gen_done;<n>` is the generator's own count: closures whose first falling
  edge was emitted. An aborted train counts the closure in progress, because
  the head unit counts a closure at its first falling edge.
- `edges` = the number of level changes driven.
- `print_drop` = `C` lines dropped because the TX buffer was full. The count
  in `gen_done` is never affected.
- `max_late_us` = the worst lateness of any edge versus its schedule. A few
  hundred µs is harmless. A value approaching 5000 µs (the filter's re-arm
  time) means the run isn't trustworthy.
- Timestamps are this board's clock. They are not comparable with the CYD's.

## Waveform (one closure)

Closure k starts at `T0 + k·10⁹/rate_mHz` µs (integer schedule, no drift).

| Phase | Edges |
|---|---|
| Closure burst, 75% | L@0, H@400+j, L@900+j (3 edges) |
| Closure burst, 25% | L@0, H@300+j, L@800+j, H@1700+j, L@2600+j (5 edges) |
| Dwell | closed until max(40% of the period, 4 ms) |
| Release, 50% | H (1 edge) |
| Release, 50% | H, L@+250, H@+600 (chatter, 3 edges) |

`j` is a seeded jitter of 0–150 µs. That gives about 3.5 edges within 3 ms on
closure and about 5.5 edges per closure overall. The PRNG (xorshift32) is
re-seeded at every start, so the same `g` command always produces the same
waveform.

Every open blip inside a burst or a chatter lasts less than 5 ms, so the head
unit's filter (5 ms release re-arm, 20 ms lockout) must count exactly one pulse per
closure. At the 40/s maximum, the line is open for ≥ 14 ms before each
closure, and closures are 25 ms apart (more than the 20 ms lockout). A
host simulation of this schedule, fed through the reference pulse filter,
gave exactly 600 pulses from 600 closures at 0.1,
1, 3, 6, 10, 15, 25 and 40/s, and 27,000 from 27,000 at 15/s (30 min).

## Pass rule

For every run: **CYD P count == `gen_done` count**, CYD `H` `drop_edges` = 0,
and `debug_drop` = 0. Check the CYD log offline with `tools/replay.py`
(the re-filter check (a) also proves that the device filter saw the same
edges).

**Two boards on USB: always name the CYD port.** This board (CP2102) shows
up as `/dev/cu.usbserial-0001` *and* `/dev/cu.SLAB_USBtoUART`; the CYD
(CH340) is `/dev/cu.usbserial-<n>` (the number depends on the USB port; check with
`ls /dev/cu.usbserial-*` with only the CYD plugged in). `0001` sorts first,
so never rely on auto-detect here (tools/capture.py refuses to guess between
two boards, but be explicit anyway):

```bash
# terminal 1: the CYD capture (type CYD commands here, e.g. d)
.venv/bin/python tools/capture.py --port /dev/cu.usbserial-<CYD n> --session OT_1_g15_30min
# terminal 2: the generator (its own port; type g15,27000 here)
arduino-cli monitor -p /dev/cu.SLAB_USBtoUART -c baudrate=115200
```

Each port has exactly one process on it: don't open the CYD
port in a monitor while tools/capture.py holds it (type CYD commands into
tools/capture.py), and don't open `/dev/cu.usbserial-0001` while the monitor
holds `/dev/cu.SLAB_USBtoUART` (same board).

Sweep: `g1,600`, `g3,600`, `g6,600`, `g10,600`, `g15,600`, then `g15,27000`
(30 min).
