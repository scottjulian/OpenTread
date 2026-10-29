// emu_main.cpp - host emulation of the real firmware loop (app.cpp, debug_serial,
// hw_ble*.cpp, src/core) with stubbed hardware (emu_hw.cpp: renderer with the
// backlight rules, scripted touch, NVS map, pulse ring) and an emulated
// NimBLE stack (stubs/NimBLEDevice.h) and centrals, plus a merged UI + BLE
// scenario (EMU_MERGED=1, see the MERGED section below: touch units toggles,
// backlight DIM/OFF/wake, summary, NVS commit/defer with a watch and an app
// connected).
//
// Scenarios, chosen by the build flags (test/emu/run_emu.sh builds them all):
//  - OT_SIM_INPUT=1 (sim + dev + short timers): a scripted bench session:
//    advertising restarts and the 3-link ceiling, RSC / FTMS isolation, the
//    FTMS Control Point matrix from a link that is NOT handle 0, concurrent CP requests
//    on two links (one indication in flight), a synchronous indicate failure,
//    a disconnect with an indication outstanding, a lost completion (35 s
//    fallback), Training/Machine Status over a workout, the watch icon and H
//    subscribed_peers for 0/1/2 subscribers incl. an FTMS-only subscriber.
//  - release flags: the S2 walk log's reed edges replayed through the real
//    loop with a watch (RSC) and an app (FTMS + CP) connected for the whole
//    session; the watch-integrated distance (sum of the notified RSC speed x
//    the time between notifications, like the FR955) is compared with
//    pulses x M, the app's integrated TD speed likewise.
// Every run also checks: notifications byte-identical to the R / X lines,
// no NimBLE call under the firmware's spinlock, no nested lock, no
// indication without the CCCD / overlapping / < 5 ms after its write.
// The captured serial log is written to argv[1] for tools/replay.py.
//
// Usage: emu_<variant> <out.log> --root <repo root>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "NimBLEDevice.h"
#include "app.h"
#include "check.h"
#include "config_hw.h"
#include "emu_hooks.h"
#include "hw_ble.h"
#include "hw_display.h"
#include "replay.h"
#include "src/core/ble_payload.h"
#include "src/core/ui_policy.h"
#include "version.h"

#ifndef EMU_MERGED
#define EMU_MERGED 0
#endif

namespace eb = emu::ble;
namespace ble = ot::ble;

namespace {

[[maybe_unused]] constexpr int64_t SEC = 1000000;
constexpr int64_t MS = 1000;
int64_t gT0 = 0;

void loopUntil(int64_t t) {
  while (emu::gNow < t) {
    eb::service();  // the host task's turn (acks due)
    app_loop();
  }
}
void at(int64_t rel) { loopUntil(gT0 + rel); }

std::string hexs(const std::vector<uint8_t>& b) {
  char buf[128];
  ble::toHex(b.data(), b.size(), buf, sizeof(buf));
  return buf;
}
[[maybe_unused]] std::vector<uint8_t> bytes(std::initializer_list<int> v) {
  std::vector<uint8_t> b;
  for (int x : v) b.push_back(static_cast<uint8_t>(x));
  return b;
}

struct Line {
  char letter;
  int64_t t;
  std::vector<std::string> f;  // all comma fields
};
std::vector<Line> lines() {
  std::vector<Line> out;
  std::istringstream in(emu::serialOut);
  std::string s;
  while (std::getline(in, s)) {
    if (s.size() < 3 || s[1] != ',') continue;
    Line l;
    l.letter = s[0];
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ',')) l.f.push_back(x);
    if (l.f.size() < 2) continue;
    l.t = std::strtoll(l.f[1].c_str(), nullptr, 10);
    out.push_back(l);
  }
  return out;
}
std::vector<Line> only(const std::vector<Line>& v, char c) {
  std::vector<Line> o;
  for (const Line& l : v)
    if (l.letter == c) o.push_back(l);
  return o;
}
size_t countB(const std::vector<Line>& v, const std::string& ev, const std::string& detailPrefix = "") {
  size_t n = 0;
  for (const Line& l : v)
    if (l.letter == 'B' && l.f.size() == 5 && l.f[2] == ev && l.f[4].compare(0, detailPrefix.size(), detailPrefix) == 0)
      ++n;
  return n;
}

// Every notification of `uuid` a peer got equals the payload of the R (RSC)
// or X (TD) line of the same tick (printed at the tick's t, notified a few
// us later in the same pass).
void checkNotesMatchLines(const eb::Peer& p, uint16_t uuid, const std::vector<Line>& rx, const char* label) {
  size_t matched = 0, bad = 0;
  for (const eb::Notification& n : p.notes) {
    if (n.uuid != uuid) continue;
    const Line* best = nullptr;
    for (const Line& l : rx)
      if (l.t <= n.t && n.t - l.t < 10 * MS) best = &l;
    if (best != nullptr && best->f.size() == 3 && best->f[2] == hexs(n.bytes)) ++matched;
    else ++bad;
  }
  CHECK_EQ(bad, static_cast<size_t>(0));
  CHECK(matched > 0u);
  std::printf("  %s: %zu notifications, all byte-identical to the %c lines\n", label, matched, rx.empty() ? '?' : rx[0].letter);
}

[[maybe_unused]] size_t notesOf(const eb::Peer& p, uint16_t uuid) {
  size_t n = 0;
  for (const eb::Notification& x : p.notes)
    if (x.uuid == uuid) ++n;
  return n;
}

void commonChecks(const std::vector<Line>& L) {
  CHECK_EQ(emu::lockViolations, 0);
  CHECK_EQ(emu::nvsInDrawPass, 0u);  // no NVS write stacked on a display push
  CHECK_EQ(emu::frameErrors, 0u);
  CHECK_EQ(eb::nimbleUnderLock, 0);
  CHECK_EQ(eb::indNoCccd, 0);
  CHECK_EQ(eb::indDouble, 0);
  CHECK_EQ(eb::indEarly, 0);
  // R / X once per K, same t
  const auto K = only(L, 'K'), R = only(L, 'R'), X = only(L, 'X');
  CHECK_EQ(R.size(), K.size());
  CHECK_EQ(X.size(), OT_FTMS_ENABLE ? K.size() : static_cast<size_t>(0));
  size_t same = 0;
  for (size_t i = 0; i < std::min(K.size(), R.size()); ++i)
    if (K[i].t == R[i].t && (i >= X.size() || X[i].t == K[i].t)) ++same;
  CHECK_EQ(same, K.size());
  // RSC distance monotonic
  uint32_t prev = 0;
  bool mono = true;
  for (const Line& r : R) {
    const uint32_t d = static_cast<uint32_t>(std::stoul(r.f[2].substr(8, 8), nullptr, 16));
    const uint32_t dLE = ((d & 0xFF) << 24) | ((d & 0xFF00) << 8) | ((d >> 8) & 0xFF00) | (d >> 24);
    mono = mono && dLE >= prev;
    prev = dLE;
  }
  CHECK(mono);
  // no malformed B line (5 fields, known event)
  const char* evs[] = {"connect", "disconnect", "sub", "unsub", "adv_restart", "cp_write", "cp_ind", "cp_ack",
                       "cp_drop", "cp_timeout", "ts", "ms"};
  size_t badB = 0;
  for (const Line& l : only(L, 'B')) {
    bool known = false;
    for (const char* e : evs) known = known || (l.f.size() == 5 && l.f[2] == e);
    if (!known) ++badB;
  }
  CHECK_EQ(badB, static_cast<size_t>(0));
  CHECK(emu::serialOut.find("ERROR") == std::string::npos);
  // Boot touch-poll lateness (seen in a device log, 2026-10-26):
  // the emulated ~270 ms NimBLE init lies between touchhw::begin and loop();
  // it is logged as the boot gap, and no G line counts it as poll lateness.
  {
    const size_t at0 = emu::serialOut.find("touch;loop_start boot_gap_us=");
    CHECK(at0 != std::string::npos);
    if (at0 != std::string::npos)
      CHECK(std::strtoll(emu::serialOut.c_str() + at0 + std::strlen("touch;loop_start boot_gap_us="), nullptr, 10) >=
            270000);
  }
  // G: 7 fields after t; touch_poll_late_max_us < 50 ms (the
  // longest emulated pass is a 30 ms NVS page erase); nvs_passes = the NVS
  // writes (N commit units lines) up to that G line.
  size_t nvsLinesSeen = 0, nvsPassSum = 0, nG = 0;
  const auto G = only(L, 'G');
  size_t gi = 0;
  for (const Line& l : L) {
    if (l.letter == 'N' && l.f.size() >= 4 && l.f[2] == "commit" && l.f[3] == "units") ++nvsLinesSeen;
    if (l.letter != 'G') continue;
    ++nG;
    CHECK_EQ(l.f.size(), static_cast<size_t>(9));
    if (l.f.size() != 9) continue;
    CHECK(std::stoll(l.f[6]) < 50000);
    nvsPassSum += static_cast<size_t>(std::stoul(l.f[8]));
    // (a write in the G line's own pass is counted in the next G window)
    CHECK(nvsPassSum <= nvsLinesSeen && nvsLinesSeen - nvsPassSum <= 1u);
    ++gi;
  }
  CHECK_EQ(nG, G.size());
  std::printf("  G lines: %zu (touch late < 50 ms, nvs_passes == N commits)\n", gi);
}

[[maybe_unused]] bool linkedAt(int64_t rel) {  // the last frame at or before gT0 + rel
  bool v = false;
  for (const disp::Frame& f : emu::frames)
    if (f.t <= gT0 + rel) v = f.linked;
  return v;
}
[[maybe_unused]] uint32_t hPeersAt(const std::vector<Line>& L, int64_t rel) {  // H subscribed_peers (field 7)
  uint32_t v = 99;
  for (const Line& l : only(L, 'H'))
    if (l.t <= gT0 + rel && l.f.size() == 9) v = static_cast<uint32_t>(std::stoul(l.f[7]));
  return v;
}

// ===================================================================== MERGED (UI + BLE)
#if EMU_MERGED
// Summary / idle / backlight and NVS running together with RSC and FTMS:
// reed edges through the real ring + filter, scripted touch contacts, a watch
// (RSC) and an app (FTMS + CP) connected.
// Built twice: short timers + dev (nw<k> forced commits while notifying) and
// release flags (production timers). Times derive from the build's timers.
//   1. boot with km in NVS; the watch and the app connect; the app takes control
//   2. tap the idle units label (awake): units -> mi, NVS commit 3 s later
//      (stationary), while BLE keeps notifying
//   3. no activity: DIM, then OFF; RSC/TD notifications continue while OFF
//   4. a tap while OFF only wakes (no toggle); the panel lights after a redraw
//   5. walk: WARMUP -> RUNNING (ms 04, ts 0d); a pace tap -> km, the NVS
//      commit is DEFERRED while moving; a touch hold-stop on the strip
//      -> SUMMARY (Z, ms 0201, ts 0f) with both centrals connected; the
//      commit happens once stationary; the summary stays FULL, clears
//      (summary_timeout, idle timers restart at the clear)
//   6. walk again, stop: DIM, OFF, auto-end -> SUMMARY wakes the panel FULL
//      (ms 0201); a new bout confirms behind the summary (SUMMARY -> RUNNING,
//      a W from SUMMARY: ms 04, the idle timers restart there), pause,
//      hold-stop again; (dev) nw3 forced commits with both subscribed
//   7. the app leaves, the watch unsubscribes: icon off; summary clears; DIM
namespace {
int64_t gNextEdgeT = 0;
void walk(int64_t fromRel, int64_t toRel, int64_t periodUs) {  // closures with a contact bounce
  for (int64_t c = gT0 + fromRel; c < gT0 + toRel; c += periodUs) {
    if (c <= gNextEdgeT) continue;
    emu::edges.push_back({c, hwp::SRC_PULSE, 0});
    emu::edges.push_back({c + 300, hwp::SRC_PULSE, 1});
    emu::edges.push_back({c + 600, hwp::SRC_PULSE, 0});
    emu::edges.push_back({c + 60000, hwp::SRC_PULSE, 1});
    gNextEdgeT = c + 60000;
  }
}
void contact(int64_t rel, int64_t durUs, int x, int y) {
  emu::contacts.push_back({gT0 + rel, gT0 + rel + durUs, x, y, x + 2, y + 1});
}
std::vector<std::string> iEvents(const std::vector<Line>& L, const std::string& ev) {  // I args
  std::vector<std::string> v;
  for (const Line& l : only(L, 'I'))
    if (l.f.size() >= 4 && l.f[2] == ev) v.push_back(l.f[3]);
  return v;
}
int64_t lastW(const std::vector<Line>& L, const std::string& to, const std::string& reason) {
  int64_t t = -1;
  for (const Line& l : only(L, 'W'))
    if (l.f.size() == 5 && l.f[3] == to && l.f[4] == reason) t = l.t;
  return t;
}
uint8_t blAt(int64_t t) {  // the panel level at absolute t
  uint8_t v = hw::BL_FULL;
  for (const emu::BlChange& b : emu::backlight)
    if (b.t <= t) v = b.applied;
  return v;
}
}  // namespace

void scenario() {
  const uint16_t TD = ble::UUID_TREADMILL_DATA, RSC = ble::UUID_RSC_MEASUREMENT, CP = ble::UUID_FTMS_CONTROL_POINT,
                 TS = ble::UUID_TRAINING_STATUS, MSU = ble::UUID_MACHINE_STATUS;
#if OT_SHORT_TIMERS
  const int64_t D = hw::SHORT_DIM_US, O = hw::SHORT_OFF_US, A = hw::SHORT_AUTO_END_US, S = hw::SHORT_SUMMARY_US;
#else
  const int64_t D = ot::UiConfig{}.dimUs, O = ot::UiConfig{}.offUs, A = ot::AUTO_END_US, S = ot::SUMMARY_US;
#endif
  const int64_t P = 600 * MS;  // 1.0 m/s
  CHECK(emu::serialOut.find("units=km") != std::string::npos);  // NVS km read at boot
  CHECK_EQ(emu::nvsWrites, 0u);                                  // never written at boot
  CHECK(emu::serialOut.find("ftms=1 adv=A") != std::string::npos);
  // ---- 1. centrals ----
  at(500 * MS);
  CHECK(eb::connect(1));
  eb::subscribe(1, RSC, 1);
  at(800 * MS);
  CHECK(eb::connect(2));
  eb::subscribe(2, TD, 1);
  eb::subscribe(2, TS, 1);
  eb::subscribe(2, MSU, 1);
  eb::subscribe(2, CP, 2);
  at(1000 * MS);
  CHECK(eb::write(2, CP, bytes({0x00})));
  size_t cpWrites = 1;
  // ---- 2. idle units tap ----
  contact(2000 * MS, 250 * MS, 160, 70);
  at(2600 * MS);
  CHECK(linkedAt(2600 * MS));
  CHECK(!emu::frames.empty() && emu::frames.back().units == ot::Units::Mi);
  CHECK_EQ(emu::nvsWrites, 0u);  // settling
  at(5500 * MS);
  CHECK_EQ(emu::nvsWrites, 1u);  // 3 s after the toggle, stationary
  CHECK(emu::nvs["units"] == 0 && emu::nvs["schema"] == 1);
  // ---- 3. DIM: a tap on the DIM screen toggles and brightens; a
  //      second tap (FULL) toggles back inside the settle window: no write.
  //      Then DIM, OFF; BLE keeps going ----
  const int64_t tDown0 = 2000 * MS + 2 * ot::TOUCH_POLL_US;  // ~ the DOWN (3rd valid sample)
  at(tDown0 + D + 200 * MS);
  CHECK_EQ(disp::backlight(), hw::BL_DIM);
  const uint32_t wakesDim = emu::wakes;
  contact(tDown0 + D + 300 * MS, 250 * MS, 160, 70);
  at(tDown0 + D + 800 * MS);
  CHECK_EQ(disp::backlight(), hw::BL_FULL);              // the DOWN brightened the panel
  CHECK_EQ(emu::wakes, wakesDim);                        // DIM -> FULL is not a wake from OFF
  CHECK(emu::frames.back().units == ot::Units::Km);      // the DIM tap toggled (no wake-swallow)
  contact(tDown0 + D + 1300 * MS, 250 * MS, 160, 70);
  at(tDown0 + D + 1800 * MS);
  CHECK(emu::frames.back().units == ot::Units::Mi);      // toggled back on the FULL screen
  const int64_t tDown1 = tDown0 + D + 1300 * MS;         // the last activity (+ <= 2 polls)
  at(tDown1 + D + 200 * MS);
  CHECK_EQ(emu::nvsWrites, 1u);                          // net no change: nothing written
  CHECK_EQ(disp::backlight(), hw::BL_DIM);
  at(tDown1 + O + 200 * MS);
  CHECK_EQ(disp::backlight(), hw::BL_OFF);
  const size_t watchNotesOff = notesOf(eb::peers[1], RSC), appNotesOff = notesOf(eb::peers[2], TD);
  const size_t darkOff = emu::darkFrames;
  // ---- 4. a tap while OFF only wakes ----
  const int64_t tWake = tDown1 + O + 5000 * MS;
  at(tWake - 100 * MS);
  CHECK(notesOf(eb::peers[1], RSC) >= watchNotesOff + 4u);  // ~5 s OFF: notifications continue
  CHECK(notesOf(eb::peers[2], TD) >= appNotesOff + 4u);
  CHECK(emu::darkFrames > darkOff);                          // frames computed, never pushed
  const uint32_t wakes0 = emu::wakes;
  contact(tWake, 250 * MS, 160, 70);
  at(tWake + 600 * MS);
  CHECK_EQ(emu::wakes, wakes0 + 1);
  CHECK_EQ(disp::backlight(), hw::BL_FULL);
  CHECK(emu::frames.back().units == ot::Units::Mi);  // swallowed: no toggle
  // ---- 5. walk, pace tap (deferred commit), touch hold-stop ----
  const int64_t w1 = tWake + 1000 * MS;
  walk(w1, w1 + 20000 * MS, P);
  at(w1 + 8000 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000d"));
  contact(w1 + 10000 * MS, 250 * MS, 160, 70);  // pace tap on the main screen
  at(w1 + 14000 * MS);
  CHECK(emu::frames.back().units == ot::Units::Km);
  CHECK_EQ(emu::nvsWrites, 1u);  // deferred: the belt is moving
  contact(w1 + 15000 * MS, 2300 * MS, 160, 214);  // hold on the strip
  at(w1 + 17600 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000f"));  // SUMMARY
  CHECK(linkedAt(w1 + 17600 * MS));                   // the icon on the summary screen
  at(w1 + 20000 * MS + ot::STOP_US + 500 * MS);       // the last pulse + STOP: stationary
  CHECK_EQ(emu::nvsWrites, 2u);
  CHECK(emu::nvs["units"] == 1);
  const auto L1 = lines();
  const int64_t tStop1 = lastW(L1, "SUMMARY", "hold_stop");
  CHECK(tStop1 > 0);
  at(tStop1 - gT0 + S + 500 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("0001"));
  const int64_t tClear1 = lastW(lines(), "IDLE", "summary_timeout");
  CHECK(tClear1 > 0);
  for (const emu::BlChange& b : emu::backlight)
    if (b.t > tStop1 && b.t < tClear1) CHECK_EQ(b.applied, hw::BL_FULL);  // FULL for the whole summary
  // ---- 6. second walk, auto-end from OFF, a bout confirming behind the summary ----
  const int64_t w2 = tClear1 - gT0 + 1000 * MS;
  walk(w2, w2 + 12000 * MS, P);
  const int64_t last2 = w2 + 12000 * MS;  // >= the last closure
  at(last2 + O + 1000 * MS);
  CHECK_EQ(disp::backlight(), hw::BL_OFF);
  at(last2 + A + 1000 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000f"));
  const int64_t tAuto = lastW(lines(), "SUMMARY", "autoend");
  CHECK(tAuto > 0);
  CHECK_EQ(blAt(tAuto + 100 * MS), hw::BL_FULL);  // the auto-end lit the panel
  const int64_t w3 = tAuto - gT0 + 2000 * MS;
  walk(w3, w3 + 10000 * MS, P);
  at(w3 + 7000 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000d"));  // confirmed behind the summary
  const int64_t tConf3 = lastW(lines(), "RUNNING", "warmup_ok");
  CHECK(tConf3 > tAuto);
  contact(w3 + 13000 * MS, 2300 * MS, 160, 214);  // paused: hold-stop again
  at(w3 + 16000 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000f"));
#if OT_DEV
  const uint32_t nw0 = emu::nvsWrites;
  const size_t notes0 = notesOf(eb::peers[1], RSC);
  emu::serialIn("nw3\n");
  at(w3 + 19000 * MS);
  CHECK_EQ(emu::nvsWrites, nw0 + 3u);
  CHECK(emu::nvs["units"] == 1);                           // the last forced commit leaves the RAM units
  // The NVS entry statistics before / after, the slow-write count
  CHECK(emu::serialOut.find("nvs;stats when=before used=") != std::string::npos);
  CHECK(emu::serialOut.find("nvs;stats when=after used=") != std::string::npos);
  CHECK(emu::serialOut.find("nvs;forced_done n=3 fails=0 max_us=") != std::string::npos);
  CHECK(emu::serialOut.find(" slow_n=") != std::string::npos);
  CHECK(notesOf(eb::peers[1], RSC) >= notes0 + 2u);         // the 1 Hz tick did not stall
#endif
  // ---- 7. the centrals leave ----
  at(w3 + 20000 * MS);
  CHECK(eb::write(2, CP, bytes({0x07})));
  ++cpWrites;
  at(w3 + 20500 * MS);
  eb::disconnect(2);
  at(w3 + 21000 * MS);
  eb::subscribe(1, RSC, 0);
  at(w3 + 21500 * MS);
  CHECK(!linkedAt(w3 + 21500 * MS));
  const int64_t tStop3 = lastW(lines(), "SUMMARY", "hold_stop");
  at(tStop3 - gT0 + S + D + 1000 * MS);
  CHECK_EQ(disp::backlight(), hw::BL_DIM);
  // ---- 8. hold-stop while the belt keeps moving, and
  //      it keeps moving past the summary clear: the idle screen asks to stop
  //      the belt (Frame.rearmBlocked) until the re-arm, then "Start moving" ----
  const int64_t w4 = tStop3 - gT0 + S + D + 2000 * MS;
  const int64_t w4End = w4 + 10000 * MS + S + 6000 * MS;
  walk(w4, w4End, P);
  at(w4 + 8000 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000d"));  // RUNNING
  CHECK(!emu::frames.back().rearmBlocked);
  contact(w4 + 8000 * MS, 2300 * MS, 160, 214);         // hold-stop on the strip while walking
  at(w4 + 11000 * MS);
  const int64_t tStop4 = lastW(lines(), "SUMMARY", "hold_stop");
  CHECK(tStop4 > tStop3);
  at(tStop4 - gT0 + S + 2000 * MS);                     // the summary cleared, the belt still moving
  CHECK(lastW(lines(), "IDLE", "summary_timeout") > tStop4);
  CHECK(emu::frames.back().state == ot::WorkoutState::Idle);
  CHECK(emu::frames.back().rearmBlocked);                // "Stop the belt to start again"
  CHECK(lastW(lines(), "WARMUP", "bout_start") < tStop4);  // the moving belt started nothing
  at(w4End + ot::STOP_US + 1000 * MS);                  // stopped: re-armed
  CHECK(lastW(lines(), "IDLE", "rearm") > tStop4);
  CHECK(emu::frames.back().state == ot::WorkoutState::Idle);
  CHECK(!emu::frames.back().rearmBlocked);               // "Start moving to begin"

  const auto L = lines();
  commonChecks(L);
  checkNotesMatchLines(eb::peers[1], RSC, only(L, 'R'), "watch RSC");
  checkNotesMatchLines(eb::peers[2], TD, only(L, 'X'), "app TD");
  CHECK_EQ(eb::peers[2].inds.size(), cpWrites);  // every CP request answered
  CHECK_EQ(countB(L, "cp_ind"), cpWrites);
  // units: four toggles (idle label FULL, idle label DIM, idle label
  // FULL, pace), the OFF wake tap swallowed
  const auto tog = iEvents(L, "units_toggle");
  CHECK_EQ(tog.size(), static_cast<size_t>(4));
  if (tog.size() == 4) {
    CHECK_EQ(tog[0], std::string("mi"));
    CHECK_EQ(tog[1], std::string("km"));
    CHECK_EQ(tog[2], std::string("mi"));
    CHECK_EQ(tog[3], std::string("km"));
  }
  CHECK_EQ(iEvents(L, "hold_complete").size(), static_cast<size_t>(3));
  for (const std::string& r : iEvents(L, "hold_complete")) CHECK_EQ(r, std::string("ok"));
  size_t defers = 0, commits = 0;
  for (const Line& l : only(L, 'N')) {
    if (l.f.size() >= 4 && l.f[2] == "defer") ++defers;
    if (l.f.size() >= 4 && l.f[2] == "commit" && l.f[3] == "units") ++commits;
  }
  CHECK_EQ(defers, static_cast<size_t>(1));
  CHECK_EQ(commits, static_cast<size_t>(OT_DEV ? 5 : 2));
  CHECK(emu::nvs["units"] == 1 && emu::nvs["schema"] == 1);
  // FTMS status vs the W lines: Training Status = the class of every W's
  // target state, notified on a class change only; Machine Status 04 on
  // every warmup_ok, 0201 on every hold_stop / autoend.
  std::string wantTs, wantMs, bts, bms;
  uint8_t code = ble::TS_IDLE;
  for (const Line& l : only(L, 'W')) {
    const std::string& to = l.f[3];
    const ot::WorkoutState st = to == "IDLE" ? ot::WorkoutState::Idle
                                : to == "WARMUP" ? ot::WorkoutState::Warmup
                                : to == "RUNNING" ? ot::WorkoutState::Running
                                : to == "PAUSED" ? ot::WorkoutState::Paused : ot::WorkoutState::Summary;
    const uint8_t c = ble::trainingStatusCode(st);
    if (c != code) {
      char h[8];
      std::snprintf(h, sizeof(h), "00%02x ", c);
      wantTs += h;
      code = c;
    }
    if (l.f[4] == "warmup_ok") wantMs += "04 ";
    if (l.f[4] == "hold_stop" || l.f[4] == "autoend") wantMs += "0201 ";
  }
  for (const Line& l : only(L, 'B')) {
    if (l.f[2] == "ts") bts += l.f[4] + " ";
    if (l.f[2] == "ms") bms += l.f[4] + " ";
  }
  CHECK_EQ(bts, wantTs);
  CHECK_EQ(bms, wantMs);
  CHECK_EQ(wantMs, std::string("04 0201 04 0201 04 0201 04 0201 "));
  {  // the W from SUMMARY (a later walk, step 8, has its own warmup_ok)
    bool fromSummary = false;
    for (const Line& l : only(L, 'W'))
      fromSummary = fromSummary || (l.t == tConf3 && l.f.size() == 5 && l.f[2] == "SUMMARY" && l.f[4] == "warmup_ok");
    CHECK(fromSummary);
  }
  CHECK_EQ(emu::frameErrors, 0u);
  CHECK(emu::summaryFrames > 0u);
  CHECK(emu::wakes >= 2u);  // the OFF tap, the auto-end
  std::printf("  merged: %zu B, %zu R, %zu X, %zu L, %zu N, %zu W, %zu I; wakes %u, dark frames %u, summary frames %u,"
              " nvs writes %u\n",
              only(L, 'B').size(), only(L, 'R').size(), only(L, 'X').size(), only(L, 'L').size(), only(L, 'N').size(),
              only(L, 'W').size(), only(L, 'I').size(), emu::wakes, emu::darkFrames, emu::summaryFrames,
              emu::nvsWrites);
}

// ===================================================================== RSC ONLY
#elif !OT_FTMS_ENABLE
// OT_FTMS_ENABLE=0 (+ sim): the RSC-only build. Layout B, no FTMS service, R lines only.
void scenario() {
  const uint16_t RSC = ble::UUID_RSC_MEASUREMENT;
  ble::AdvPair p;
  CHECK(ble::buildAdvert(ble::AdvLayout::B, ble::BLE_NAME, p));
  CHECK_EQ(hexs(eb::advData), hexs(std::vector<uint8_t>(p.primary.b, p.primary.b + p.primary.len)));
  CHECK_EQ(hexs(eb::advData), std::string("0201060a094f70656e54726561640303141803194004"));
  CHECK(!eb::scanRespEnabled);
  CHECK(eb::findChr(ble::UUID_TREADMILL_DATA) == nullptr && eb::findChr(ble::UUID_FTMS_CONTROL_POINT) == nullptr &&
        eb::findChr(ble::UUID_FTMS_FEATURE) == nullptr);
  CHECK(emu::serialOut.find("ftms=0 adv=B") != std::string::npos);
  at(1000 * MS);
  CHECK(eb::connect(1));
  eb::subscribe(1, RSC, 1);
  at(2000 * MS);
  emu::serialIn("s8\n");
  at(20000 * MS);
  emu::serialIn("s0\n");
  at(25000 * MS);
  eb::disconnect(1);  // the watch walks out of range ...
  at(27000 * MS);
  CHECK(eb::connect(1));  // ... and comes back: the RSC distance never goes backwards
  eb::subscribe(1, RSC, 1);
  at(30000 * MS);
  const auto L = lines();
  commonChecks(L);
  CHECK(only(L, 'X').empty());
  checkNotesMatchLines(eb::peers[1], RSC, only(L, 'R'), "watch RSC (ftms=0)");
  CHECK(countB(L, "ms") == 0u && countB(L, "ts") == 0u);
}

// ===================================================================== SIM
#elif OT_SIM_INPUT
void scenario() {
  const uint16_t TD = ble::UUID_TREADMILL_DATA, RSC = ble::UUID_RSC_MEASUREMENT, CP = ble::UUID_FTMS_CONTROL_POINT,
                 TS = ble::UUID_TRAINING_STATUS, MSU = ble::UUID_MACHINE_STATUS;
  // ---- boot: Layout A, advertising, banner ----
  {
    ble::AdvPair p;
    CHECK(ble::buildAdvert(ble::AdvLayout::A, ble::BLE_NAME, p));
    CHECK_EQ(hexs(eb::advData), hexs(std::vector<uint8_t>(p.primary.b, p.primary.b + p.primary.len)));
    CHECK_EQ(hexs(eb::scanData), std::string("03194004"));
    CHECK(eb::scanRespEnabled);
  }
  CHECK(eb::advActive);
  CHECK(emu::serialOut.find("nimble=2.5.0") != std::string::npos);
  CHECK(emu::serialOut.find("ftms=1 adv=A") != std::string::npos);
  CHECK_EQ(hexs(eb::read(TS)), std::string("0001"));
  CHECK_EQ(hexs(eb::read(ble::UUID_FTMS_FEATURE)), std::string("0410000000000000"));
  CHECK_EQ(hexs(eb::read(ble::UUID_RSC_FEATURE)), std::string("0600"));
  CHECK_EQ(hexs(eb::read(ble::UUID_SENSOR_LOCATION)), std::string("00"));
  {
    const auto fw = eb::read(ble::UUID_DIS_FIRMWARE);
    CHECK(std::string(fw.begin(), fw.end()).rfind(OT_FW_VERSION "+", 0) == 0);
    const auto mf = eb::read(ble::UUID_DIS_MANUFACTURER);
    CHECK_EQ(std::string(mf.begin(), mf.end()), std::string("OpenTread"));
  }
  at(500 * MS);
  CHECK(!linkedAt(500 * MS));
  // ---- three centrals; advertising restarts after each, none at 3 links ----
  at(1000 * MS);
  CHECK(eb::connect(0));  // dummy holds handle 0 (bench rule for the CP matrix)
  CHECK(!eb::advActive);
  at(1010 * MS);
  CHECK(eb::advActive);   // restarted by the callback flag within a pass
  at(1500 * MS);
  CHECK(eb::connect(1));  // the watch
  eb::subscribe(1, RSC, 1);
  at(1700 * MS);
  CHECK(linkedAt(1700 * MS));
  at(2000 * MS);
  CHECK(eb::connect(2));  // the app: FTMS data + status, CP indications later
  eb::subscribe(2, TD, 1);
  eb::subscribe(2, TS, 1);
  eb::subscribe(2, MSU, 1);
  const uint32_t startsAt3 = eb::advStarts;
  at(2500 * MS);
  CHECK(!eb::advActive);
  CHECK(!eb::connect(3));  // a 4th central finds no advert
  // A stray advert at 3 links (e.g. a start racing the
  // 3rd connect) is stopped by the next watchdog pass (<= 1 s), never left running.
  CHECK(emu::serialOut.find("ble;adv_stop") == std::string::npos);
  eb::advActive = true;
  at(3000 * MS);
  emu::serialIn("s8\n");
  at(3900 * MS);
  CHECK(!eb::advActive);
  CHECK(emu::serialOut.find("ble;adv_stop links=3 ok=1") != std::string::npos);
  CHECK_EQ(eb::advStarts, startsAt3);  // the watchdog never restarts at 3 links
  // ---- CP without indications: dropped, no effect ----
  at(4000 * MS);
  CHECK(eb::write(2, CP, bytes({0x00})));
  at(4200 * MS);
  CHECK(eb::peers[2].inds.empty());
  eb::subscribe(2, CP, 2);
  at(5000 * MS);
  CHECK(eb::write(2, CP, bytes({0x00})));
  at(5200 * MS);
  CHECK_EQ(eb::peers[2].inds.size(), static_cast<size_t>(1));
  if (!eb::peers[2].inds.empty()) CHECK_EQ(hexs(eb::peers[2].inds[0].bytes), std::string("800001"));
  // ---- disconnect: advertising resumes at once; a "new connection" on the same handle ----
  at(6000 * MS);
  eb::disconnect(2);
  at(6010 * MS);
  CHECK(eb::advActive);
  at(6500 * MS);
  CHECK(eb::connect(2));
  at(8000 * MS);
  CHECK_EQ(bleh::subscribedPeers(), 1);  // only the watch: the new link inherited nothing
  CHECK_EQ(notesOf(eb::peers[2], TD), static_cast<size_t>(0));
  eb::subscribe(2, TD, 1);
  eb::subscribe(2, CP, 2);
  // ---- the CP matrix, from handle 2 with handle 0 held ----
  struct Step { int64_t t; std::vector<uint8_t> req; const char* want; };
  const std::vector<Step> matrix = {
      {8500 * MS, bytes({0x07}), "800705"},          // before 00: not permitted
      {9000 * MS, bytes({0x02, 0x64, 0x00}), "800202"},
      {9500 * MS, bytes({0x08, 0x03}), "800803"},
      {10000 * MS, bytes({0x00}), "800001"},
      {10500 * MS, bytes({0x07}), "800701"},
      {11000 * MS, bytes({0x08, 0x01}), "800801"},
  };
  for (const Step& s : matrix) {
    at(s.t);
    const size_t before = eb::peers[2].inds.size();
    CHECK(eb::write(2, CP, s.req));
    at(s.t + 100 * MS);
    if (CHECK_EQ(eb::peers[2].inds.size(), before + 1)) CHECK_EQ(hexs(eb::peers[2].inds.back().bytes), std::string(s.want));
  }
  // ---- two links ask at once: one indication in flight at most ----
  at(12000 * MS);
  eb::subscribe(1, CP, 2);
  eb::peers[2].ackDelayUs = 200 * MS;
  CHECK(eb::write(2, CP, bytes({0x00})));
  at(12001 * MS);
  CHECK(eb::write(1, CP, bytes({0x00})));
  at(12500 * MS);
  if (CHECK(!eb::peers[1].inds.empty() && !eb::peers[2].inds.empty())) {
    const int64_t t2 = eb::peers[2].inds.back().t, t1 = eb::peers[1].inds.back().t;
    CHECK(t1 >= t2 + 200 * MS);  // waited for link 2's ack
    CHECK(t1 - t2 < 220 * MS);   // ... and no longer
  }
  eb::peers[2].ackDelayUs = 30 * MS;
  // ---- synchronous indicate failure: token released, next request answered at once ----
  at(14000 * MS);
  eb::peers[1].policy = eb::AckPolicy::FailSync;
  CHECK(eb::write(1, CP, bytes({0x07})));
  at(14100 * MS);
  eb::peers[1].policy = eb::AckPolicy::Auto;
  const size_t n2 = eb::peers[2].inds.size();
  CHECK(eb::write(2, CP, bytes({0x00})));
  at(14130 * MS);
  CHECK_EQ(eb::peers[2].inds.size(), n2 + 1);
  // ---- disconnect with an indication outstanding: token freed by ENOTCONN ----
  at(16000 * MS);
  eb::peers[2].ackDelayUs = 5 * SEC;
  CHECK(eb::write(2, CP, bytes({0x00})));
  at(16050 * MS);
  CHECK(eb::outstanding.active);
  eb::disconnect(2);
  const size_t n1 = eb::peers[1].inds.size();
  CHECK(eb::write(1, CP, bytes({0x00})));
  at(16100 * MS);
  CHECK_EQ(eb::peers[1].inds.size(), n1 + 1);
  at(17000 * MS);
  CHECK(eb::connect(2));
  eb::subscribe(2, TD, 1);
  eb::subscribe(2, TS, 1);
  eb::subscribe(2, MSU, 1);
  eb::subscribe(2, CP, 2);
  // ---- workout: pause, resume, hold-stop, summary, idle ----
  at(20000 * MS);
  {
    const auto r = only(lines(), 'R');
    bool saw569 = false;
    for (const Line& l : r) saw569 = saw569 || l.f[2].substr(0, 6) == "063902";  // 8 km/h ideal: 569
    CHECK(saw569);
  }
  CHECK_EQ(hexs(eb::read(TS)), std::string("000d"));  // RUNNING
  emu::serialIn("s0\n");
  at(25000 * MS);
  emu::serialIn("s6\n");
  at(30000 * MS);
  emu::serialIn("hb\n");
  at(32000 * MS);
  emu::serialIn("hc\n");
  at(33000 * MS);
  CHECK_EQ(hexs(eb::read(TS)), std::string("000f"));  // SUMMARY
  at(45000 * MS);                                     // short SUMMARY = 12 s
  CHECK_EQ(hexs(eb::read(TS)), std::string("0001"));
  {
    std::string ms, ts;
    for (const eb::Notification& n : eb::peers[2].notes) {
      if (n.uuid == MSU) ms += hexs(n.bytes) + " ";
      if (n.uuid == TS) ts += hexs(n.bytes) + " ";
    }
    CHECK_EQ(ms, std::string("0201 "));             // subscribed after the confirm: only the stop
    CHECK_EQ(ts, std::string("000f 0001 "));
    const auto L = lines();
    std::string bts, bms;
    for (const Line& l : only(L, 'B')) {
      if (l.f[2] == "ts") bts += l.f[4] + " ";
      if (l.f[2] == "ms") bms += l.f[4] + " ";
    }
    CHECK_EQ(bts, std::string("000d 000f 0001 "));   // RUNNING<->PAUSED never notified
    CHECK_EQ(bms, std::string("04 0201 "));
  }
  // ---- icon: FTMS-only subscriber lights it; 0 subscribers = outline ----
  at(45500 * MS);
  eb::subscribe(1, RSC, 0);
  at(46000 * MS);
  CHECK(linkedAt(46000 * MS));
  at(46200 * MS);
  CHECK_EQ(bleh::subscribedPeers(), 1);  // the app's TD subscription alone
  eb::subscribe(2, TD, 0);
  at(46500 * MS);
  CHECK(!linkedAt(46500 * MS));
  // ---- a lost completion: the 35 s fallback releases the token ----
  at(47000 * MS);
  eb::peers[1].policy = eb::AckPolicy::Never;
  CHECK(eb::write(1, CP, bytes({0x07})));
  at(47500 * MS);
  const size_t m2 = eb::peers[2].inds.size();
  CHECK(eb::write(2, CP, bytes({0x00})));
  at(80000 * MS);
  CHECK_EQ(eb::peers[2].inds.size(), m2);  // still waiting (< 35 s)
  at(84000 * MS);
  if (CHECK_EQ(eb::peers[2].inds.size(), m2 + 1))
    CHECK(eb::peers[2].inds.back().t - eb::peers[1].inds.back().t >= ble::CP_IND_TIMEOUT_US);
  // ---- everyone leaves ----
  at(85000 * MS);
  eb::disconnect(0);
  eb::disconnect(1);
  eb::disconnect(2);
  at(86000 * MS);
  CHECK(eb::advActive);
  CHECK(!linkedAt(86000 * MS));

  const auto L = lines();
  commonChecks(L);
  checkNotesMatchLines(eb::peers[1], RSC, only(L, 'R'), "watch RSC");
  checkNotesMatchLines(eb::peers[2], TD, only(L, 'X'), "app TD");
  CHECK_EQ(notesOf(eb::peers[1], TD), static_cast<size_t>(0));   // isolation
  CHECK_EQ(notesOf(eb::peers[2], RSC), static_cast<size_t>(0));
  CHECK_EQ(notesOf(eb::peers[0], RSC) + notesOf(eb::peers[0], TD), static_cast<size_t>(0));
  CHECK(countB(L, "cp_drop", "no_cccd") >= 1u);
  CHECK(countB(L, "cp_drop", "status;code=6") == 1u);   // the synchronous failure
  CHECK(countB(L, "cp_drop", "status;code=7") == 1u);   // ENOTCONN at the disconnect
  CHECK(countB(L, "cp_timeout", "fallback") == 1u);
  CHECK(countB(L, "adv_restart", "flag;ok=1") >= 4u);
  CHECK(countB(L, "connect") == 5u);  // the 4th central never found an advert
  CHECK_EQ(hPeersAt(L, 30000 * MS), 2u);  // H subscribed_peers: the watch (RSC) + the app (TD)
  CHECK_EQ(hPeersAt(L, 51000 * MS), 0u);  // the H line after both unsubscribed
  std::printf("  sim: %zu B lines, %zu R, %zu X, cp_ind %zu, cp_ack %zu, adv starts %u\n", only(L, 'B').size(),
              only(L, 'R').size(), only(L, 'X').size(), countB(L, "cp_ind"), countB(L, "cp_ack"), eb::advStarts);
}

// ===================================================================== RELEASE
#else
void scenario() {
  const uint16_t TD = ble::UUID_TREADMILL_DATA, RSC = ble::UUID_RSC_MEASUREMENT, CP = ble::UUID_FTMS_CONTROL_POINT;
  CHECK(emu::serialOut.find("sim=0 dev=0 short=0 ftms=1 adv=A") != std::string::npos);
  at(1000 * MS);
  CHECK(eb::connect(1));
  eb::subscribe(1, RSC, 1);
  at(2000 * MS);
  CHECK(eb::connect(2));
  eb::subscribe(2, TD, 1);
  eb::subscribe(2, ble::UUID_TRAINING_STATUS, 1);
  eb::subscribe(2, ble::UUID_MACHINE_STATUS, 1);
  eb::subscribe(2, CP, 2);
  at(2500 * MS);
  CHECK(eb::write(2, CP, bytes({0x00})));
  const int64_t lastEdge = emu::edges.empty() ? gT0 : emu::edges.back().t;
  // the app pokes the CP every 60 s through the run
  int64_t next = 60 * SEC;
  const uint8_t ops[3][2] = {{0x07, 0}, {0x08, 0x01}, {0x00, 0}};
  int k = 0;
  while (emu::gNow < lastEdge + 1100 * SEC) {
    at(next);
    std::vector<uint8_t> q = {ops[k % 3][0]};
    if (ops[k % 3][0] == 0x08) q.push_back(ops[k % 3][1]);
    CHECK(eb::write(2, CP, q));
    ++k;
    next += 60 * SEC;
  }
  loopUntil(emu::gNow + 100 * MS);  // the last request's answer
  const auto L = lines();
  commonChecks(L);
  checkNotesMatchLines(eb::peers[1], RSC, only(L, 'R'), "watch RSC");
  checkNotesMatchLines(eb::peers[2], TD, only(L, 'X'), "app TD");
  CHECK_EQ(notesOf(eb::peers[1], TD), static_cast<size_t>(0));
  CHECK_EQ(notesOf(eb::peers[2], RSC), static_cast<size_t>(0));
  // every CP request answered (the app's 00 at +2.5 s + the periodic ones)
  CHECK_EQ(eb::peers[2].inds.size(), static_cast<size_t>(k + 1));
  // Watch / app integration of the broadcast speed vs the pulse truth.
  const auto P = only(L, 'P');
  const double truthM = static_cast<double>(P.size()) * 0.5974;
  auto integrate = [](const std::vector<eb::Notification>& v, uint16_t uuid, bool rsc) {
    double m = 0;
    const eb::Notification* prev = nullptr;
    for (const eb::Notification& n : v) {
      if (n.uuid != uuid) continue;
      if (prev != nullptr) {
        const double sp = rsc ? (prev->bytes[1] | (prev->bytes[2] << 8)) / 256.0
                              : (prev->bytes[2] | (prev->bytes[3] << 8)) / 360.0;  // 0.01 km/h -> m/s
        m += sp * static_cast<double>(n.t - prev->t) / 1e6;
      }
      prev = &n;
    }
    return m;
  };
  const double watchM = integrate(eb::peers[1].notes, RSC, true);
  const double appM = integrate(eb::peers[2].notes, TD, false);
  std::printf("  release S2: truth %zu P = %.3f m; watch-integrated RSC %.3f m (%+.3f m, %+.3f%%); app-integrated TD"
              " %.3f m (%+.3f%%)\n",
              P.size(), truthM, watchM, watchM - truthM, 100.0 * (watchM - truthM) / truthM, appM,
              100.0 * (appM - truthM) / truthM);
  CHECK(std::fabs(watchM - truthM) <= 0.01 * truthM);
  CHECK(std::fabs(appM - truthM) <= 0.015 * truthM);
  // final RSC distance = floor(P x 5.974) dm; last TD distance = the workout's (Z) metres
  const auto R = only(L, 'R');
  if (CHECK(!R.empty() && !P.empty())) {
    const std::string h = R.back().f[2];
    const uint32_t dm = static_cast<uint32_t>(std::stoul(h.substr(14, 2) + h.substr(12, 2) + h.substr(10, 2) + h.substr(8, 2), nullptr, 16));
    CHECK_EQ(static_cast<uint64_t>(dm), static_cast<uint64_t>(P.size()) * 597400u / 100000u);
  }
  CHECK(countB(L, "ms", "04") == 1u && countB(L, "ms", "0201") == 1u);
}
#endif

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <out.log> [--root <repo>]\n", argv[0]);
    return 2;
  }
  const std::string root = replay::repoRootFromArgs(argc, argv);
#if EMU_MERGED
  emu::nvs = {{"schema", 1}, {"units", 1}};  // km stored by an earlier session
#elif !OT_SIM_INPUT
  {
    replay::Log log;
    std::string err;
    if (!replay::loadFile(root + "/firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv", log, err) || log.banners.size() != 1) {
      std::fprintf(stderr, "cannot load S2: %s\n", err.c_str());
      return 2;
    }
    emu::initPulseLevel = log.banners[0].lineLevel;
    const int64_t shift = (emu::gNow + 3 * SEC) - log.edges.front().t;
    for (const replay::Edge& e : log.edges) emu::edges.push_back(hwp::EdgeEvent{e.t + shift, hwp::SRC_PULSE, e.level});
  }
#endif
  app_setup();
  gT0 = emu::gNow;
  scenario();
  std::ofstream out(argv[1], std::ios::binary);
  out << "# emulated session (test/emu), "
      << (EMU_MERGED ? "merged: touch, backlight, NVS with BLE"
                     : (OT_SIM_INPUT ? "sim+dev+short" : "release flags, S2 edges"))
      << "\n";
  out << emu::serialOut;
  out.close();
  return check::summary(EMU_MERGED ? (OT_SHORT_TIMERS ? "emu_merged_short" : "emu_merged")
                                   : (!OT_FTMS_ENABLE ? "emu_rsc_only" : (OT_SIM_INPUT ? "emu_sim" : "emu_release")));
}
