// hw_touch.cpp - see hw_touch.h.
#include "config_hw.h"

#if !OT_BRINGUP

#include "hw_touch.h"

#include <inttypes.h>

#include "debug_serial.h"
#include "hw_display.h"
#include "src/core/config.h"

namespace touchhw {

namespace {

constexpr int64_t CAL_LINE_US = 100000;  // 'c' mode: T lines at 10 Hz

PollClock gClock;
bool     gCal = false;
int64_t  gLastCalLine = 0;
uint32_t gPolls = 0, gValid = 0;
disp::TouchRaw gLast;

void tLine(int64_t t, const disp::TouchRaw& r, char kind) {
  dbg::line(dbg::LV_ALWAYS, "T,%" PRId64 ",%u,%u,%u,%c;%d;%d", t, static_cast<unsigned>(r.xRaw),
            static_cast<unsigned>(r.yRaw), static_cast<unsigned>(r.z), kind, static_cast<int>(r.x),
            static_cast<int>(r.y));
}

}  // namespace

void begin(int64_t now) {
  gClock.begin(now);
  dbg::status(now,
              "touch;poll_us=%" PRId64 " press_n=%" PRIu32 " release_ms=%" PRId64 " z_min=%u x_min=%u x_max=%u"
              " y_min=%u y_max=%u rotation=%u irq_pin=%d(unused)",
              ot::TOUCH_POLL_US, ot::TOUCH_PRESS_N, ot::TOUCH_RELEASE_MS, static_cast<unsigned>(hw::TOUCH_Z_MIN),
              static_cast<unsigned>(hw::TOUCH_X_MIN), static_cast<unsigned>(hw::TOUCH_X_MAX),
              static_cast<unsigned>(hw::TOUCH_Y_MIN), static_cast<unsigned>(hw::TOUCH_Y_MAX),
              static_cast<unsigned>(hw::DISPLAY_ROTATION), static_cast<int>(hw::TOUCH_IRQ_PIN_UNUSED));
}

void loopStart(int64_t now) {
  const uint32_t gap = gClock.loopStart(now);
  dbg::status(now, "touch;loop_start boot_gap_us=%" PRIu32 " (setup time since touch begin, e.g. the BLE init: not"
                   " poll lateness; G touch_poll_late_max_us counts loop() polls only)", gap);
}

bool poll(int64_t now, ot::TouchSample& s) {
  if (!gClock.due(now, ot::TOUCH_POLL_US)) return false;

  disp::TouchRaw r;
  disp::touchRead(r);
  ++gPolls;
  gLast = r;
  s.t = now;
  s.valid = r.valid;
  s.x = r.x;
  s.y = r.y;
  s.z = r.z;
  if (r.valid) {
    ++gValid;
    if (gCal) {
      if (now - gLastCalLine >= CAL_LINE_US) {
        gLastCalLine = now;
        tLine(now, r, 'c');
      }
    } else if (dbg::enabled(dbg::LV_EDGE)) {
      tLine(now, r, 's');
    }
  }
  return true;
}

void setCalMode(int64_t now, bool on) {
  gCal = on;
  gLastCalLine = now - CAL_LINE_US;
  dbg::status(now, "touch;cal_mode=%u x_min=%u x_max=%u y_min=%u y_max=%u z_min=%u rotation=%u (T lines at 10 Hz)",
              on ? 1u : 0u, static_cast<unsigned>(hw::TOUCH_X_MIN), static_cast<unsigned>(hw::TOUCH_X_MAX),
              static_cast<unsigned>(hw::TOUCH_Y_MIN), static_cast<unsigned>(hw::TOUCH_Y_MAX),
              static_cast<unsigned>(hw::TOUCH_Z_MIN), static_cast<unsigned>(hw::DISPLAY_ROTATION));
}

bool calMode() { return gCal; }

uint32_t takeLateMaxUs() { return gClock.takeLate(); }

void dump(int64_t now) {
  dbg::status(now,
              "dump;touch armed=%u cal=%u polls=%" PRIu32 " valid=%" PRIu32 " late_max_since_boot_us=%" PRIu32
              " boot_gap_us=%" PRIu32 " last valid=%u raw=%u;%u z=%u xy=%d;%d",
              gClock.armed ? 1u : 0u, gCal ? 1u : 0u, gPolls, gValid, gClock.lateMaxRun, gClock.bootGapUs, gLast.valid ? 1u : 0u,
              static_cast<unsigned>(gLast.xRaw), static_cast<unsigned>(gLast.yRaw), static_cast<unsigned>(gLast.z),
              static_cast<int>(gLast.x), static_cast<int>(gLast.y));
}

}  // namespace touchhw

#endif  // !OT_BRINGUP
