// emu_hooks.h - driver-side handles into the emulation stubs (test/emu).
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "emu_platform.h"
#include "hw_pulse.h"

namespace emu {
extern std::vector<hwp::EdgeEvent> edges;  // preloaded reed edges (sorted by t)
extern size_t edgeNext;
extern uint8_t initPulseLevel;

// Touch: scripted contacts. A poll at t in [a, b) returns a valid
// sample at the point interpolated from (x0,y0) to (x1,y1); every poll
// instant is recorded.
struct Contact {
  int64_t a, b;
  int x0, y0, x1, y1;
};
extern std::vector<Contact> contacts;
extern std::vector<int64_t> polls;

// Backlight: every applied level / wake request, in order.
struct BlChange {
  int64_t t;
  uint8_t applied;  // the level on the panel after this change
  bool    pending;  // a wake redraw is in progress (the panel is still dark)
};
extern std::vector<BlChange> backlight;
extern uint32_t wakes, darkFrames, summaryFrames, frameErrors;

// NVS: the "opentread" namespace (keys "schema", "units"); writes advance
// the emulated clock (2 ms per entry write, 30 ms for every 16th: a page
// erase with the flash cache off). The pulse ring keeps its edges meanwhile
// (the ISR is IRAM-safe on the device).
extern std::map<std::string, uint8_t> nvs;
extern uint32_t nvsWrites, nvsReads;
// Render passes: the emulated renderer "pushes" (3 ms) in the pass after
// every frame and during a wake redraw. An NVS write in a pass that pushed
// is counted (app.cpp must not stack a flash write on a push).
extern uint32_t nvsInDrawPass, drawPasses;
extern bool passDrew;
}  // namespace emu
