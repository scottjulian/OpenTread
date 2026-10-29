// hw_touch.h - XPT2046 touch polling.
//
// Every TOUCH_POLL_US (20 ms) the loop calls poll(now): one lcd.getTouchRaw()
// (LovyanGFX 1.2.26: the median of 7 XPT2046 reads with its pressure check),
// the hardcoded calibration of config_hw.h applied through the panel's own
// convertRawXY (the same affine + rotation the bring-up 3x3 grid verified), and
// one TouchSample{t = now, valid = size >= TOUCH_Z_MIN, x, y, z}.
//  - The sample is stamped with the loop pass's `now` (never a later clock
//    read): every input to the core then has t <= now, the same rule as the
//    edge drain, so the replay of P + I lines stays exact.
//  - IRQ GPIO 36 is NEVER attached (GPIO 36/39 input-glitch erratum): touch is
//    polled only; LGFX_CYD passes pin_int = -1 and nothing here touches GPIO 36.
//  - T lines: T,<t>,<x_raw>,<y_raw>,<z>,<kind>;<sx>;<sy> for valid
//    samples: kind 's' = every valid sample at verbosity 2; kind 'c' = the
//    calibration mode ('c' command), 10 Hz whatever the verbosity (the v2
//    per-sample lines are then suppressed). <sx>;<sy> are the calibrated
//    screen coordinates.
// Loop task only. Not used by OT_BRINGUP builds (the bring-up has its own touch screens).
#pragma once

#include <stdint.h>

#include "src/core/touch_gesture.h"

namespace touchhw {

// The 20 ms poll schedule and its lateness (shared with the host emulation,
// test/emu/emu_hw.cpp, so the boot re-baseline below is tested there).
//  - lateness of a poll = now - its due instant; G touch_poll_late_max_us is
//    the worst one since the previous G line (takeLate).
//  - loopStart(now), called once at the END of setup(): the time between
//    begin() and the first loop() (the ~270 ms NimBLE init, measured on the device)
//    is boot time, not poll lateness. It is returned as the boot gap (its own
//    S line field, boot_gap_us, like the display's boot_render_max_us) and the
//    schedule restarts at `now`, so the first G line covers loop() polls only.
struct PollClock {
  bool     armed = false;
  int64_t  next = 0;
  uint32_t lateMax = 0;      // since the last takeLate()
  uint32_t lateMaxRun = 0;   // since loopStart() (dump)
  uint32_t bootGapUs = 0;    // begin() -> loopStart()

  static uint32_t clampUs(int64_t d) {
    return d < 0 ? 0u : (d > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(d));
  }
  void begin(int64_t now) {
    armed = true;
    next = now;
  }
  uint32_t loopStart(int64_t now) {
    bootGapUs = armed ? clampUs(now - next) : 0u;
    next = now;
    lateMax = 0;
    lateMaxRun = 0;
    return bootGapUs;
  }
  // True when a poll is due at `now` (then the schedule advances; after a long
  // block the next poll is one period later, no burst).
  bool due(int64_t now, int64_t periodUs) {
    if (!armed || now < next) return false;
    const uint32_t late = clampUs(now - next);
    if (late > lateMax) lateMax = late;
    if (late > lateMaxRun) lateMaxRun = late;
    next += periodUs;
    if (next <= now) next = now + periodUs;
    return true;
  }
  uint32_t takeLate() {
    const uint32_t v = lateMax;
    lateMax = 0;
    return v;
  }
};

void begin(int64_t now);

// End of setup(): re-baseline the poll schedule (PollClock::loopStart) and
// log the boot gap: S,t,touch;loop_start boot_gap_us=<us>.
void loopStart(int64_t now);

// True when a sample was taken this pass (a poll was due); `s` is filled.
bool poll(int64_t now, ot::TouchSample& s);

// 'c' calibration mode (T lines at 10 Hz).
void setCalMode(int64_t now, bool on);
bool calMode();

// G line: worst poll lateness (us) since the previous call, then reset.
uint32_t takeLateMaxUs();

// 'd' dump line.
void dump(int64_t now);

}  // namespace touchhw
