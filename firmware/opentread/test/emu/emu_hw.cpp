// emu_hw.cpp - emulation stubs of the hardware adapters app.cpp calls
// (hw_display incl. the backlight, hw_touch, hw_pulse, hw_nvs): no
// LovyanGFX, no GPIO, no flash. The pulse ring is preloaded by the driver
// (emu::edges) and drained by the real app.cpp exactly like the ISR ring
// (peek, only t <= now). Touch contacts are scripted (emu::contacts); NVS is a
// map (emu::nvs).
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#include "debug_serial.h"
#include "emu_hooks.h"
#include "hw_display.h"
#include "hw_nvs.h"
#include "hw_pulse.h"
#include "hw_touch.h"
#include "src/core/persist_policy.h"
#include "src/core/ui_policy.h"

namespace emu {
std::vector<hwp::EdgeEvent> edges;
size_t edgeNext = 0;
uint8_t initPulseLevel = 1;
std::vector<Contact> contacts;
std::vector<int64_t> polls;
std::vector<BlChange> backlight;
uint32_t wakes = 0, darkFrames = 0, summaryFrames = 0, frameErrors = 0;
std::map<std::string, uint8_t> nvs;
uint32_t nvsWrites = 0, nvsReads = 0, nvsInDrawPass = 0, drawPasses = 0;
bool passDrew = false;
}  // namespace emu

namespace {
void frameError(const char* what, int64_t t) {
  if (emu::frameErrors < 20) std::fprintf(stderr, "EMU frame error: %s at t=%lld\n", what, static_cast<long long>(t));
  ++emu::frameErrors;
}
}  // namespace

// ---- display (backlight semantics of hw_display.cpp) --------------------------------
namespace disp {
namespace {
int64_t gNextFrame = 0;
uint8_t gTarget = 0, gApplied = 0;
bool gWake = false;
int gWakeSteps = 0;
bool gPushPending = false;  // a frame was built: the next service() pushes its dirty fields
std::map<int64_t, ot::SummaryRecord> gSummaries;  // by summaryStartUs: the first frame's record

void apply(uint8_t lv) {
  gApplied = lv;
  const int64_t t = esp_timer_get_time();
  dbg::line(dbg::LV_K, "L,%lld,%u", static_cast<long long>(t), static_cast<unsigned>(lv));
  emu::backlight.push_back({t, gApplied, gWake});
}
}  // namespace

bool begin(const Frame& first) {
  emu::frames.push_back(first);
  gTarget = hw::BL_FULL;
  apply(hw::BL_FULL);  // boot order step 6: the first frame is drawn, then lit
  gNextFrame = esp_timer_get_time() + 100000;
  return true;
}
bool frameDue(int64_t now) {
  emu::passDrew = false;  // called once per loop pass, before service()
  return now >= gNextFrame;
}
void frame(const Frame& f) {
  emu::frames.push_back(f);
  gPushPending = true;
  gNextFrame += 100000;  // RENDER_MAX_HZ = 10
  if (gNextFrame <= f.t) gNextFrame = f.t + 100000;
  if (gTarget == hw::BL_OFF) ++emu::darkFrames;
  if (f.state == ot::WorkoutState::Summary) {
    ++emu::summaryFrames;
    if (!f.hasSummary) frameError("summary frame without the record", f.t);
    else {
      if (ot::summaryRemainingUs(f.summary, f.summaryUs, f.t) <= 0) frameError("summary frame with 0 remaining", f.t);
      auto it = gSummaries.find(f.summary.summaryStartUs);
      if (it == gSummaries.end()) gSummaries[f.summary.summaryStartUs] = f.summary;
      else if (std::memcmp(&it->second, &f.summary, sizeof(f.summary)) != 0) frameError("summary record changed", f.t);
      ot::SummaryView v;
      ot::makeSummaryView(f.summary, f.units, v);
    }
  } else if (f.hasSummary) {
    frameError("non-summary frame with a summary", f.t);
  }
}
uint32_t service(int64_t) {
  if (gTarget == hw::BL_OFF) {  // OFF: nothing is pushed
    gPushPending = false;
    return 0;
  }
  if (gWake) {  // the full redraw: ~4 passes of ~8 ms, then the brightness
    emu::gNow += 8000;
    if (++gWakeSteps >= 4) {
      gWake = false;
      apply(gTarget);
    }
    gPushPending = false;
    emu::passDrew = true;
    ++emu::drawPasses;
    return 1;
  }
  if (gPushPending) {  // the frame's dirty fields: one ~3 ms push
    gPushPending = false;
    emu::gNow += 3000;
    emu::passDrew = true;
    ++emu::drawPasses;
    return 1;
  }
  return 0;
}
void setForceRedraw(bool) {}
void setHold(int64_t, bool, int32_t) {}
void unitsChanged(int64_t now) { gNextFrame = now; }
void setBacklight(int64_t now, uint8_t level) {
  if (level == gTarget) return;
  gTarget = level;
  if (level == hw::BL_OFF) {
    gWake = false;
    apply(level);
    return;
  }
  if (gApplied == hw::BL_OFF) {
    if (gWake) return;
    gWake = true;
    gWakeSteps = 0;
    ++emu::wakes;
    emu::backlight.push_back({esp_timer_get_time(), gApplied, true});
    gNextFrame = now;
    return;
  }
  apply(level);
}
uint8_t backlight() { return gApplied; }
bool backlightWakePending() { return gWake; }
void takeStats(uint32_t& a, uint32_t& b, uint32_t& c) { a = b = c = 0; }
void dump(int64_t) {}
}  // namespace disp

// ---- touch ----------------------------------------------------------------------
namespace touchhw {
namespace {
PollClock gClock;  // the real schedule + lateness (hw_touch.h)
}
void begin(int64_t now) { gClock.begin(now); }
void loopStart(int64_t now) {
  const uint32_t gap = gClock.loopStart(now);
  dbg::status(now, "touch;loop_start boot_gap_us=%u (emu)", static_cast<unsigned>(gap));
}
bool poll(int64_t now, ot::TouchSample& s) {
  if (!gClock.due(now, ot::TOUCH_POLL_US)) return false;
  emu::polls.push_back(now);
  s = ot::TouchSample{};
  s.t = now;
  for (const emu::Contact& c : emu::contacts) {
    if (now >= c.a && now < c.b) {
      const double f = static_cast<double>(now - c.a) / static_cast<double>(c.b - c.a);
      s.valid = true;
      s.x = static_cast<int16_t>(c.x0 + (c.x1 - c.x0) * f);
      s.y = static_cast<int16_t>(c.y0 + (c.y1 - c.y0) * f);
      s.z = 10;
      break;
    }
  }
  return true;
}
void setCalMode(int64_t, bool) {}
bool calMode() { return false; }
uint32_t takeLateMaxUs() { return gClock.takeLate(); }
void dump(int64_t) {}
}  // namespace touchhw

// ---- NVS --------------------------------------------------------------------------
namespace nvshw {
Read readUnits() {
  Read r;
  ++emu::nvsReads;
  r.opened = !emu::nvs.empty();
  if (emu::nvs.count(KEY_SCHEMA)) {
    r.hasSchema = true;
    r.schema = emu::nvs[KEY_SCHEMA];
  }
  if (emu::nvs.count(KEY_UNITS)) {
    r.hasUnits = true;
    r.units = emu::nvs[KEY_UNITS];
  }
  emu::gNow += 500;
  r.us = 500;
  return r;
}
Write writeUnits(uint8_t units, bool withSchema) {
  Write w;
  ++emu::nvsWrites;
  if (emu::passDrew) ++emu::nvsInDrawPass;  // a flash write on top of a push: loop budget
  const int64_t d = (emu::nvsWrites % 16 == 0) ? 30000 : 2000;  // an occasional page erase
  emu::gNow += d;
  emu::nvs[KEY_UNITS] = units;
  w.opened = true;
  if (withSchema) {
    emu::nvs[KEY_SCHEMA] = ot::NVS_SCHEMA;
    w.schemaPut = true;
  }
  w.ok = true;
  w.us = static_cast<uint32_t>(d);
  return w;
}
Stats stats() {  // a 5-page partition (5 x 126 entries); live entries only, no page model
  Stats st;
  st.ok = true;
  st.total = 630;
  st.used = static_cast<uint32_t>(emu::nvs.size()) + 1u;  // + the namespace entry
  st.free = st.total - st.used;
  st.available = st.free > 126u ? st.free - 126u : 0u;     // minus the reserved page
  st.namespaces = 1;
  return st;
}
}  // namespace nvshw

// ---- pulse ring -------------------------------------------------------------------
namespace hwp {
void begin(bool installPulse, InitInfo& info) {
  info = InitInfo{};
  info.t0 = esp_timer_get_time();
  info.pulseLevel = emu::initPulseLevel;
  info.markerLevel = 1;
  info.pulseInstalled = installPulse ? 1 : 0;
  info.isrIram = 1;
  info.pulseIsrInIram = info.markerIsrInIram = info.pushEdgeInIram = info.ringInDram = 1;
  info.intrFlags = 0x400;
}
bool peekEdge(EdgeEvent& out) {
  if (emu::edgeNext >= emu::edges.size()) return false;
  out = emu::edges[emu::edgeNext];
  return true;
}
void popEdge() {
  if (emu::edgeNext < emu::edges.size()) ++emu::edgeNext;
}
uint32_t droppedEdges() { return 0; }
uint32_t occupancy() { return 0; }
uint8_t readPulseLevel() {
  return emu::edgeNext == 0 ? emu::initPulseLevel : emu::edges[emu::edgeNext - 1].level;
}
}  // namespace hwp
