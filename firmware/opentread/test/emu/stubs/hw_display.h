// hw_display.h - emulation stub (test/emu): the renderer API app.cpp uses in
// non-bring-up builds, recording every Frame (the watch-icon state, the
// summary record, the units among them). Shadows the real hw_display.h
// because run_emu.sh copies this file over it in the emulation build folder.
// The backlight half (setBacklight / backlight / backlightWakePending)
// is emulated in emu_hw.cpp with the semantics of hw_display.cpp: OFF is
// applied at once, a wake from OFF first redraws (a few passes), then raises
// the brightness; every applied level is an L line.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <vector>

#include "config_hw.h"
#include "src/core/format.h"
#include "src/core/touch_gesture.h"
#include "src/core/workout.h"

#define OT_LGFX_VERSION_STR "emu"

namespace disp {

struct Frame {
  int64_t           t         = 0;
  ot::WorkoutState  state     = ot::WorkoutState::Idle;
  int32_t           speedMmps = 0;
  uint32_t          wkPulses  = 0;
  int64_t           movingUs  = 0;
  int64_t           totalUs   = 0;
  bool              hasSummary = false;
  ot::SummaryRecord summary;
  int64_t           summaryUs  = 0;
  ot::Units         units     = ot::Units::Mi;
  bool              linked    = false;
  bool              rearmBlocked = false;  // idle screen "Stop the belt to start again"
};

bool begin(const Frame& first);
bool frameDue(int64_t now);
void frame(const Frame& f);
uint32_t service(int64_t now);
void setForceRedraw(bool on);
void setHold(int64_t now, bool active, int32_t permille);
void unitsChanged(int64_t now);
void setBacklight(int64_t now, uint8_t level);
uint8_t backlight();
bool    backlightWakePending();
void takeStats(uint32_t& renderMaxUs, uint32_t& renderCount, uint32_t& touchLateMaxUs);
void dump(int64_t now);

}  // namespace disp

namespace emu {
inline std::vector<disp::Frame> frames;  // every frame the app passed
}
