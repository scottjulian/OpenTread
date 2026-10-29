// test_ui_policy.cpp - host tests for src/core/ui_policy.*: tests
// 5.1-5.8 (backlight timers, wake, auto-end / hold-stop / summary
// interplay with the real Workout, summary math, frozen summary), plus the
// wake-swallow integration with the real TouchGesture, the chatter rule with the real PulseFilter, the portable
// 128-bit mul-div against __int128, and a seeded property test.
//
// Times are exact (int64 us); "not one us earlier" is checked literally.
//
// Usage: test_ui_policy [--root <repo root>]
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "fixed.h"
#include "format.h"
#include "metrics.h"
#include "pulse_filter.h"
#include "replay.h"
#include "touch_gesture.h"
#include "ui_policy.h"
#include "workout.h"

using ot::Backlight;
using ot::UiConfig;
using ot::UiPolicy;
using ot::UiScreen;
using ot::Units;
using ot::WorkoutEvent;
using ot::WorkoutEventType;
using ot::WorkoutState;

namespace {

std::string gRoot = ".";
constexpr int64_t SEC = 1000000;
constexpr int64_t MS = 1000;
constexpr int64_t DIM = ot::DIM_US;
constexpr int64_t OFF = ot::OFF_US;
const char* const kGoldenDir = "firmware/opentread/test/golden/";
const char* const kS2 = "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv";

const char* bl(Backlight b) { return ot::backlightName(b); }

struct Rng {  // splitmix64
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed) {}
  uint64_t next() {
    uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  int64_t range(int64_t lo, int64_t hi) { return lo + static_cast<int64_t>(next() % static_cast<uint64_t>(hi - lo + 1)); }
};

// The app's wiring (app.cpp), host side: accepted pulse -> Metrics ->
// Workout -> drain (W from SUMMARY -> summaryEnded(t)) -> activity(t);
// touch DOWN -> activity(t); each query ticks the Workout to `now` first.
struct Rig {
  ot::Config  cfg;
  ot::Metrics met;
  ot::Workout wk;
  UiPolicy    ui;
  std::deque<WorkoutEvent> ev;  // deque: pointers from find() stay valid

  explicit Rig(const ot::Config& c = ot::Config{}, const UiConfig& u = UiConfig{}) : cfg(c), met(c), wk(c), ui(u) {
    ui.reset(0);
  }
  void drain() {
    WorkoutEvent e;
    while (wk.pollEvent(e)) {
      if (e.from == WorkoutState::Summary && e.to != WorkoutState::Summary) ui.summaryEnded(e.t);
      ev.push_back(e);
    }
  }
  void pulse(int64_t t) {
    met.onPulse(t);
    wk.onPulse(t, met.speedAtLastPulseMmps());
    drain();
    ui.activity(t);
  }
  void walk(int64_t from, int64_t to, int64_t step) {
    for (int64_t t = from; t <= to; t += step) pulse(t);
  }
  void touch(int64_t t) { ui.activity(t); }
  bool summary() const { return wk.state() == WorkoutState::Summary; }
  Backlight at(int64_t now) {
    wk.tick(now);
    drain();
    return ui.level(now, summary());
  }
  const WorkoutEvent* find(WorkoutEventType type) const {
    for (const WorkoutEvent& e : ev)
      if (e.type == type) return &e;
    return nullptr;
  }
};

// ---------------------------------------------------------------- basics
void testBasics() {
  CHECK(ot::screenForState(WorkoutState::Idle) == UiScreen::Idle);
  CHECK(ot::screenForState(WorkoutState::Warmup) == UiScreen::Main);
  CHECK(ot::screenForState(WorkoutState::Running) == UiScreen::Main);
  CHECK(ot::screenForState(WorkoutState::Paused) == UiScreen::Main);
  CHECK(ot::screenForState(WorkoutState::Summary) == UiScreen::Summary);
  CHECK_EQ(std::string(ot::uiScreenName(UiScreen::Summary)), std::string("summary"));
  CHECK_EQ(std::string(bl(Backlight::Dim)), std::string("dim"));
  CHECK_EQ(DIM, static_cast<int64_t>(60) * SEC);
  CHECK_EQ(OFF, static_cast<int64_t>(300) * SEC);
  // A summary forces FULL whatever the idle time, even 10^12 s.
  UiPolicy p;
  p.reset(0);
  CHECK(p.level(static_cast<int64_t>(1) << 60, true) == Backlight::Full);
  CHECK(p.level(static_cast<int64_t>(1) << 60, false) == Backlight::Off);
  // Activity in the past never moves the clock backwards; in the "future"
  // (idle < 0) it is FULL.
  p.activity(100 * SEC);
  p.activity(50 * SEC);
  CHECK_EQ(p.lastActivityUs(), 100 * SEC);
  CHECK(p.level(99 * SEC, false) == Backlight::Full);
  p.summaryEnded(10 * SEC);  // older than the last activity: no effect
  CHECK_EQ(p.lastActivityUs(), 100 * SEC);
  // Large timestamps (an always-on device, ~3000 years of us): no overflow.
  UiPolicy q;
  const int64_t big = static_cast<int64_t>(1) << 62;
  q.reset(big);
  CHECK(q.level(big + DIM - 1, false) == Backlight::Full);
  CHECK(q.level(big + DIM, false) == Backlight::Dim);
  CHECK(q.level(big + OFF, false) == Backlight::Off);
}

// 5.1 No pulse or touch: DIM at 60.000 s, OFF at 300.000 s; not one us earlier.
void test51() {
  UiPolicy p;
  p.reset(0);
  CHECK(p.level(0, false) == Backlight::Full);
  CHECK(p.level(DIM - 1, false) == Backlight::Full);
  CHECK(p.level(DIM, false) == Backlight::Dim);
  CHECK(p.level(OFF - 1, false) == Backlight::Dim);
  CHECK(p.level(OFF, false) == Backlight::Off);
  int64_t at = 0;
  CHECK(p.nextChange(0, false, at) && at == DIM);
  CHECK(p.nextChange(DIM, false, at) && at == OFF);
  CHECK(!p.nextChange(OFF, false, at));
  CHECK(!p.nextChange(0, true, at));  // a summary has no policy timer
  // From an arbitrary activity instant A.
  const int64_t A = 1234567890;
  p.activity(A);
  CHECK(p.level(A + DIM - 1, false) == Backlight::Full);
  CHECK(p.level(A + DIM, false) == Backlight::Dim);
  CHECK(p.level(A + OFF - 1, false) == Backlight::Dim);
  CHECK(p.level(A + OFF, false) == Backlight::Off);
  // Short-timer bench set (OT_SHORT_TIMERS=1: DIM 6 s, OFF 15 s).
  UiConfig sc;
  sc.dimUs = 6 * SEC;
  sc.offUs = 15 * SEC;
  UiPolicy s(sc);
  s.reset(7);
  CHECK(s.level(7 + 6 * SEC - 1, false) == Backlight::Full);
  CHECK(s.level(7 + 6 * SEC, false) == Backlight::Dim);
  CHECK(s.level(7 + 15 * SEC - 1, false) == Backlight::Dim);
  CHECK(s.level(7 + 15 * SEC, false) == Backlight::Off);
  // With the real Workout: a walk, then nothing.
  Rig r;
  r.walk(1 * SEC, 61 * SEC, SEC / 2);
  const int64_t L = 61 * SEC;
  CHECK(r.at(L + DIM - 1) == Backlight::Full);
  CHECK(r.at(L + DIM) == Backlight::Dim);
  CHECK(r.at(L + OFF - 1) == Backlight::Dim);
  CHECK(r.at(L + OFF) == Backlight::Off);
}

// 5.2 Pulse or touch while DIM/OFF: FULL immediately; the timers restart.
void test52() {
  for (int kind = 0; kind < 2; ++kind) {        // 0 = pulse, 1 = touch DOWN
    for (int from = 0; from < 2; ++from) {      // 0 = from DIM, 1 = from OFF
      Rig r;
      r.walk(1 * SEC, 31 * SEC, SEC / 2);
      const int64_t L = 31 * SEC;
      const int64_t w = L + (from == 0 ? DIM + 5 * SEC : OFF + 5 * SEC);
      CHECK(r.at(w - 1) == (from == 0 ? Backlight::Dim : Backlight::Off));
      if (kind == 0) r.pulse(w);
      else r.touch(w);
      CHECK(r.at(w) == Backlight::Full);  // the same instant
      CHECK(r.at(w + DIM - 1) == Backlight::Full);
      CHECK(r.at(w + DIM) == Backlight::Dim);
      CHECK(r.at(w + OFF - 1) == Backlight::Dim);
      CHECK(r.at(w + OFF) == Backlight::Off);
    }
  }
}

// 5.3 Raw edges with no accepted pulse (chatter): no wake. Only accepted
// pulses are activity (app: onAcceptedPulse -> activity). A magnet resting on
// the reed with the line bouncing open < RELEASE never re-arms the filter.
void test53() {
  ot::PulseFilter f;
  UiPolicy p;
  f.reset(0, 1);
  p.reset(0);
  // One real closure at 1 s (line open since boot), then the line stays closed.
  if (f.onEdge(1 * SEC, 0)) p.activity(1 * SEC);
  CHECK_EQ(f.acceptedCount(), 1u);
  const int64_t T = 1 * SEC + DIM + 10 * SEC;  // DIM by now
  CHECK(p.level(T, false) == Backlight::Dim);
  uint32_t edges = 0;
  for (int64_t t = T; t < T + 2 * SEC; t += 3 * MS) {  // open 1 ms, closed 2 ms: never open >= 5 ms
    if (f.onEdge(t, 1)) p.activity(t);
    if (f.onEdge(t + 1 * MS, 0)) p.activity(t + 1 * MS);
    edges += 2;
  }
  CHECK(edges > 1000u);
  CHECK_EQ(f.acceptedCount(), 1u);
  CHECK_EQ(p.lastActivityUs(), 1 * SEC);
  CHECK(p.level(T + 2 * SEC, false) == Backlight::Dim);
  // ... and a real pulse (open >= RELEASE, then a closure) does wake it.
  const int64_t U = T + 3 * SEC;
  if (f.onEdge(U, 1)) p.activity(U);
  if (f.onEdge(U + 10 * MS, 0)) p.activity(U + 10 * MS);
  CHECK_EQ(f.acceptedCount(), 2u);
  CHECK(p.level(U + 10 * MS, false) == Backlight::Full);
}

// 5.4 Auto-end while OFF: last pulse at L, no touch -> OFF at L+300 s; FULL at
// L+900 s (auto-end) and for the whole summary; IDLE at L+1020 s (clear); DIM
// at L+1080 s; OFF at L+1320 s. Never DIM during the summary.
void test54() {
  Rig r;
  r.walk(1 * SEC, 61 * SEC, SEC / 2);
  const int64_t L = 61 * SEC;
  CHECK(r.wk.state() == WorkoutState::Running);
  CHECK(r.at(L + OFF - 1) == Backlight::Dim);
  CHECK(r.at(L + OFF) == Backlight::Off);
  CHECK(r.at(L + 900 * SEC) == Backlight::Off);  // auto-end fires when now - last > AUTO_END
  CHECK(r.wk.state() == WorkoutState::Paused);
  CHECK(r.at(L + 900 * SEC + 1) == Backlight::Full);
  CHECK(r.wk.state() == WorkoutState::Summary);
  const WorkoutEvent* ae = r.find(WorkoutEventType::AutoEnded);
  if (CHECK(ae != nullptr)) CHECK_EQ(ae->t, L + 900 * SEC);  // stamped at the threshold
  bool allFull = true, allSummary = true;
  for (int64_t t = L + 900 * SEC + 100 * MS; t < L + 1020 * SEC; t += 100 * MS) {
    allFull = allFull && r.at(t) == Backlight::Full;
    allSummary = allSummary && r.summary();
  }
  CHECK(allFull);
  CHECK(allSummary);
  CHECK(r.at(L + 1020 * SEC - 1) == Backlight::Full);
  CHECK(r.summary());
  CHECK(r.at(L + 1020 * SEC) == Backlight::Full);
  CHECK(r.wk.state() == WorkoutState::Idle);
  const WorkoutEvent* cl = r.find(WorkoutEventType::SummaryCleared);
  if (CHECK(cl != nullptr)) CHECK_EQ(cl->t, L + 1020 * SEC);
  CHECK_EQ(r.ui.lastActivityUs(), L + 1020 * SEC);  // timers restart at the clear
  CHECK(r.at(L + 1080 * SEC - 1) == Backlight::Full);
  CHECK(r.at(L + 1080 * SEC) == Backlight::Dim);
  CHECK(r.at(L + 1320 * SEC - 1) == Backlight::Dim);
  CHECK(r.at(L + 1320 * SEC) == Backlight::Off);
  // D3 on screen: the summary's total = auto-end instant - confirm (6 s).
  if (ae != nullptr) CHECK_EQ(ae->summary.totalUs, L + 900 * SEC - 6 * SEC);

  // The same with a late tick (the loop only looks at L+1000 s): the summary
  // is still FULL, and the clear is still stamped L+1020 (DIM at L+1080).
  Rig q;
  q.walk(1 * SEC, 61 * SEC, SEC / 2);
  CHECK(q.at(L + 1000 * SEC) == Backlight::Full);
  CHECK(q.summary());
  CHECK(q.at(L + 1079 * SEC) == Backlight::Full);
  CHECK(q.wk.state() == WorkoutState::Idle);
  CHECK(q.at(L + 1080 * SEC) == Backlight::Dim);
}

// 5.5 Workout open while OFF (paused 6 min, then resume): the same workout
// continues; total includes the 6 min, moving doesn't.
void test55() {
  Rig r;
  r.walk(1 * SEC, 61 * SEC, SEC / 2);  // confirm at 6 s
  const uint32_t id = r.wk.snapshot(61 * SEC).workoutId;
  CHECK(id != 0u);
  const int64_t resume = 61 * SEC + 360 * SEC;
  CHECK(r.at(resume - 1) == Backlight::Off);
  CHECK(r.wk.state() == WorkoutState::Paused);
  r.walk(resume, resume + 60 * SEC, SEC / 2);
  CHECK(r.at(resume + 60 * SEC) == Backlight::Full);
  const ot::WorkoutSnapshot s = r.wk.snapshot(resume + 60 * SEC);
  CHECK(s.state == WorkoutState::Running);
  CHECK_EQ(s.workoutId, id);
  CHECK_EQ(s.totalUs, resume + 60 * SEC - 6 * SEC);  // 475 s: the 6 min pause included
  CHECK(s.movingUs >= 115 * SEC && s.movingUs <= 117 * SEC);  // 55 + tail + 60 s: the pause excluded
  CHECK(r.find(WorkoutEventType::AutoEnded) == nullptr);
  CHECK(r.find(WorkoutEventType::Stopped) == nullptr);
  // The resume pulse itself is the wake.
  Rig q;
  q.walk(1 * SEC, 61 * SEC, SEC / 2);
  CHECK(q.at(resume - 1) == Backlight::Off);
  q.pulse(resume);
  CHECK(q.at(resume) == Backlight::Full);
}

// 5.6 Summary math (S2 golden: 249.116 m / 217.726 s = 4.119 km/h; avg pace
// moving/distance in integer us/um: 874.0 s/km -> 14:34 /km, 1406.6 s/mi ->
// 23:27 /mi; a 0-pulse workout shows --:-- and 0.0).
void test56() {
  // The record from the golden (written by the independent oracle) ...
  replay::Golden g;
  std::string err;
  ot::SummaryRecord z;
  if (CHECK(replay::loadGolden(gRoot + "/" + kGoldenDir + "workout_S2_walk.txt", g, err))) {
    const std::string zl = g.get("z");
    char reason[16] = {};
    long long t = 0, mv = 0, tot = 0;
    unsigned pulses = 0, id = 0;
    int top = 0, avg = 0;
    CHECK_EQ(std::sscanf(zl.c_str(), "%lld %15s %u %lld %lld %d %d %u", &t, reason, &pulses, &mv, &tot, &top, &avg, &id),
             8);
    z.pulses = pulses;
    z.movingUs = mv;
    z.totalUs = tot;
    z.topMmps = top;
    z.avgMmps = avg;
    z.workoutId = id;
    z.summaryStartUs = t;
  }
  CHECK_EQ(z.pulses, 417u);
  CHECK_EQ(z.movingUs, static_cast<int64_t>(217726508));
  // ... equals the one the C++ Workout produces from the S2 log.
  replay::Log log;
  if (CHECK(replay::loadFile(gRoot + "/" + kS2, log, err))) {
    const replay::WorkoutTrace tr = replay::replayWorkout(replay::workoutInputs(log), ot::Config{});
    const ot::SummaryRecord* rec = nullptr;
    for (const WorkoutEvent& e : tr.events)
      if (replay::isZEvent(e)) rec = &e.summary;
    if (CHECK(rec != nullptr)) {
      CHECK_EQ(rec->pulses, z.pulses);
      CHECK_EQ(rec->movingUs, z.movingUs);
      CHECK_EQ(rec->totalUs, z.totalUs);
      CHECK_EQ(rec->avgMmps, z.avgMmps);
      CHECK_EQ(rec->topMmps, z.topMmps);
    }
  }
  // distance 249.1158 m, avg 1144 mm/s = 4.118 km/h.
  CHECK_EQ(ot::pulsesToUm(z.pulses), static_cast<uint64_t>(249115800));
  CHECK_EQ(z.avgMmps, 1144);
  CHECK_EQ(ot::summaryPaceSeconds(z.pulses, z.movingUs, Units::Km), 874u);
  CHECK_EQ(ot::summaryPaceSeconds(z.pulses, z.movingUs, Units::Mi), 1407u);  // 1406.59 rounded
  ot::SummaryView v;
  ot::makeSummaryView(z, Units::Km, v);
  CHECK_EQ(std::string(v.distance), std::string("0.24"));
  CHECK_EQ(std::string(v.moving), std::string("3:37"));
  CHECK_EQ(std::string(v.total), std::string("35:30"));  // 2130.799 s
  CHECK_EQ(std::string(v.avgPace), std::string("14:34"));
  CHECK_EQ(std::string(v.avgSpeed), std::string("4.1"));
  CHECK_EQ(std::string(v.topSpeed), std::string("12.4"));  // 3440 mm/s = 12.384 km/h
  CHECK(v.paceShown);
  CHECK_EQ(v.paceS, 874u);
  ot::makeSummaryView(z, Units::Mi, v);
  CHECK_EQ(std::string(v.distance), std::string("0.15"));
  CHECK_EQ(std::string(v.avgPace), std::string("23:27"));
  CHECK_EQ(std::string(v.avgSpeed), std::string("2.6"));  // 2.559 mph
  CHECK_EQ(std::string(v.topSpeed), std::string("7.7"));  // 7.695 mph
  // 0-pulse workout (e.g. a hold-stop right after the confirm).
  ot::SummaryRecord e0;
  ot::makeSummaryView(e0, Units::Km, v);
  CHECK_EQ(std::string(v.avgPace), std::string("--:--"));
  CHECK_EQ(std::string(v.avgSpeed), std::string("0.0"));
  CHECK_EQ(std::string(v.topSpeed), std::string("0.0"));
  CHECK_EQ(std::string(v.distance), std::string("0.00"));
  CHECK_EQ(std::string(v.moving), std::string("0:00"));
  CHECK(!v.paceShown);
  ot::makeSummaryView(e0, Units::Mi, v);
  CHECK_EQ(std::string(v.avgPace), std::string("--:--"));
  CHECK_EQ(std::string(v.avgSpeed), std::string("0.0"));
  // Distance < 0.01 of a unit: --:-- (km: < 10 m = 16 pulses; mi: < 16.09 m = 26 pulses).
  ot::SummaryRecord d;
  d.movingUs = 10 * SEC;
  d.pulses = 16;  // 9.5584 m
  CHECK(!ot::summaryPaceShown(d.pulses, d.movingUs, Units::Km));
  d.pulses = 17;  // 10.1558 m -> 0.01 km, 984.7 s/km
  CHECK(ot::summaryPaceShown(d.pulses, d.movingUs, Units::Km));
  CHECK_EQ(ot::summaryPaceSeconds(d.pulses, d.movingUs, Units::Km), 985u);
  d.pulses = 26;  // 15.5324 m < 16.09344
  CHECK(!ot::summaryPaceShown(d.pulses, d.movingUs, Units::Mi));
  d.pulses = 27;  // 16.1298 m
  CHECK(ot::summaryPaceShown(d.pulses, d.movingUs, Units::Mi));
  // PACE_MAX_S guard, exactly at the rounding edge: 1000 pulses = 597.4 m;
  // s/km = moving / 597400.
  CHECK_EQ(ot::summaryPaceSeconds(1000, 3583802600, Units::Km), 5999u);          // 5999.0
  CHECK(ot::summaryPaceShown(1000, 3583802600, Units::Km));
  CHECK_EQ(ot::summaryPaceSeconds(1000, 3584101299, Units::Km), 5999u);          // 5999.4999983
  CHECK(ot::summaryPaceShown(1000, 3584101299, Units::Km));
  CHECK_EQ(ot::summaryPaceSeconds(1000, 3584101300, Units::Km), 6000u);          // 5999.5 -> 6000 (half up)
  CHECK(!ot::summaryPaceShown(1000, 3584101300, Units::Km));
  ot::SummaryRecord slow;
  slow.pulses = 1000;
  slow.movingUs = 3584101300;
  ot::makeSummaryView(slow, Units::Km, v);
  CHECK_EQ(std::string(v.avgPace), std::string("--:--"));
  // moving <= 0 with a distance (defensive): --:--, avg speed from the record.
  CHECK(!ot::summaryPaceShown(1000, 0, Units::Km));
  CHECK(!ot::summaryPaceShown(1000, -5, Units::Mi));
  CHECK_EQ(ot::summaryPaceSeconds(1000, -5, Units::Mi), 0u);
  // Extremes: no overflow, no UB (UBSan), saturating.
  CHECK_EQ(ot::summaryPaceSeconds(1, INT64_MAX, Units::Mi), UINT32_MAX);
  CHECK(!ot::summaryPaceShown(1, INT64_MAX, Units::Mi));
  CHECK_EQ(ot::summaryPaceSeconds(UINT32_MAX, 1, Units::Km), 0u);
  ot::SummaryRecord ext;
  ext.pulses = UINT32_MAX;
  ext.movingUs = INT64_MAX;
  ext.totalUs = INT64_MAX;
  ext.topMmps = INT32_MAX;
  ext.avgMmps = INT32_MAX;
  ot::makeSummaryView(ext, Units::Mi, v);
  CHECK(std::strlen(v.distance) < sizeof(v.distance));
  CHECK(std::strlen(v.total) < sizeof(v.total));
  CHECK_EQ(std::string(v.avgPace), std::string("--:--"));  // 2.4e7 s/mi
}

// 5.7 Summary frozen: pulses during SUMMARY don't change any summary value.
void test57() {
  Rig r;
  r.walk(1 * SEC, 61 * SEC, SEC / 2);
  r.at(61200 * MS);
  r.touch(61200 * MS);
  r.wk.holdBegan(61200 * MS);
  r.drain();
  r.at(63200 * MS);
  CHECK(r.wk.holdCompleted(63200 * MS) == ot::HoldResult::Ok);
  r.drain();
  CHECK(r.summary());
  const ot::SummaryRecord* sr = r.wk.summary();
  if (!CHECK(sr != nullptr)) return;
  const ot::SummaryRecord before = *sr;
  ot::SummaryView vKm0, vMi0, v;
  ot::makeSummaryView(before, Units::Km, vKm0);
  ot::makeSummaryView(before, Units::Mi, vMi0);
  // The cool-down walk that was already under way (re-arm block, D4) ...
  r.walk(63500 * MS, 75 * SEC, SEC / 2);
  // ... still > STOP, then a new bout that does not reach the 5 s confirm ...
  r.walk(80 * SEC, 83 * SEC, SEC / 2);
  // ... and a hand-nudge.
  r.walk(100 * SEC, 100 * SEC, SEC);
  CHECK(r.at(110 * SEC) == Backlight::Full);
  CHECK(r.summary());
  sr = r.wk.summary();
  if (!CHECK(sr != nullptr)) return;
  CHECK_EQ(sr->pulses, before.pulses);
  CHECK_EQ(sr->movingUs, before.movingUs);
  CHECK_EQ(sr->totalUs, before.totalUs);
  CHECK_EQ(sr->topMmps, before.topMmps);
  CHECK_EQ(sr->avgMmps, before.avgMmps);
  CHECK_EQ(sr->workoutId, before.workoutId);
  CHECK_EQ(sr->summaryStartUs, before.summaryStartUs);
  ot::makeSummaryView(*sr, Units::Km, v);
  CHECK(std::memcmp(&v, &vKm0, sizeof(v)) == 0);
  ot::makeSummaryView(*sr, Units::Mi, v);
  CHECK(std::memcmp(&v, &vMi0, sizeof(v)) == 0);
  // The snapshot's workout fields in SUMMARY are the record's.
  const ot::WorkoutSnapshot s = r.wk.snapshot(110 * SEC);
  CHECK_EQ(s.wkPulses, before.pulses);
  CHECK_EQ(s.movingUs, before.movingUs);
  CHECK_EQ(s.totalUs, before.totalUs);
  // The summary countdown uses the record's own start and the Workout's SUMMARY.
  CHECK_EQ(ot::summaryRemainingUs(before, ot::SUMMARY_US, before.summaryStartUs), ot::SUMMARY_US);
  CHECK_EQ(ot::summaryRemainingUs(before, ot::SUMMARY_US, before.summaryStartUs + 1), ot::SUMMARY_US - 1);
  CHECK_EQ(ot::summaryRemainingUs(before, ot::SUMMARY_US, before.summaryStartUs + ot::SUMMARY_US), 0);
  CHECK_EQ(ot::summaryRemainingUs(before, ot::SUMMARY_US, before.summaryStartUs - SEC), ot::SUMMARY_US);
  CHECK_EQ(ot::summaryRemainingUs(before, ot::SUMMARY_US, INT64_MAX), 0);
  // The clear happens exactly when the countdown reaches 0.
  CHECK(r.at(before.summaryStartUs + ot::SUMMARY_US - 1) == Backlight::Full);
  CHECK(r.summary());
  r.at(before.summaryStartUs + ot::SUMMARY_US);
  CHECK(!r.summary());
}

// 5.8 Hold-stop: hold DOWN at t (no later pulse or touch) -> FULL through the
// hold and the whole summary until t+2+120 s (clear); DIM at t+182 s; OFF at t+422 s.
void test58() {
  Rig r;
  r.walk(1 * SEC, 61 * SEC, SEC / 2);
  const int64_t t = 61200 * MS;
  r.at(t);
  r.touch(t);  // the hold's touch DOWN is activity
  CHECK_EQ(r.wk.holdBegan(t), t);
  r.drain();
  bool full = true;
  for (int64_t x = t; x < t + 2 * SEC; x += 20 * MS) full = full && r.at(x) == Backlight::Full;
  r.at(t + 2 * SEC);
  CHECK(r.wk.holdCompleted(t + 2 * SEC) == ot::HoldResult::Ok);
  r.drain();
  CHECK(r.summary());
  for (int64_t x = t + 2 * SEC; x < t + 122 * SEC; x += 100 * MS) full = full && r.at(x) == Backlight::Full && r.summary();
  CHECK(full);
  CHECK(r.at(t + 122 * SEC) == Backlight::Full);
  CHECK(r.wk.state() == WorkoutState::Idle);
  CHECK(r.at(t + 182 * SEC - 1) == Backlight::Full);
  CHECK(r.at(t + 182 * SEC) == Backlight::Dim);
  CHECK(r.at(t + 422 * SEC - 1) == Backlight::Dim);
  CHECK(r.at(t + 422 * SEC) == Backlight::Off);
}

// D4: a new bout confirming behind the summary replaces it early (SUMMARY ->
// RUNNING): the screen goes MAIN, the summary ends at the confirm instant.
// A summary that clears into a background warm-up (SUMMARY -> WARMUP) restarts
// the timers at the clear.
void testSummaryReplaced() {
  Rig r;
  r.walk(1 * SEC, 61 * SEC, SEC / 2);
  r.at(62 * SEC);
  r.touch(62 * SEC);
  r.wk.holdBegan(62 * SEC);
  r.at(64 * SEC);
  CHECK(r.wk.holdCompleted(64 * SEC) == ot::HoldResult::Ok);
  r.drain();
  CHECK(r.summary());
  r.walk(80 * SEC, 90 * SEC, SEC / 2);  // still since 61 s: the re-arm block is gone; confirms at 85 s
  CHECK(r.wk.state() == WorkoutState::Running);
  CHECK(ot::screenForState(r.wk.state()) == UiScreen::Main);
  const WorkoutEvent* c = nullptr;
  for (const WorkoutEvent& e : r.ev)
    if (e.type == WorkoutEventType::Confirmed && e.from == WorkoutState::Summary) c = &e;
  if (CHECK(c != nullptr)) CHECK_EQ(c->t, 85 * SEC);
  CHECK(r.at(90 * SEC + DIM - 1) == Backlight::Full);
  CHECK(r.at(90 * SEC + DIM) == Backlight::Dim);

  // A background bout still warming up at the clear: SUMMARY -> WARMUP.
  Rig q;
  q.walk(1 * SEC, 61 * SEC, SEC / 2);
  q.at(62 * SEC);
  q.wk.holdBegan(62 * SEC);
  q.at(64 * SEC);
  CHECK(q.wk.holdCompleted(64 * SEC) == ot::HoldResult::Ok);
  q.drain();
  const int64_t clr = 64 * SEC + ot::SUMMARY_US;
  q.walk(clr - 2 * SEC, clr + 1 * SEC, SEC / 2);  // bout from clr-2 s, would confirm at clr+3 s
  CHECK(q.wk.state() == WorkoutState::Warmup);
  const WorkoutEvent* sc = q.find(WorkoutEventType::SummaryCleared);
  if (CHECK(sc != nullptr)) {
    CHECK_EQ(sc->t, clr);
    CHECK(sc->to == WorkoutState::Warmup);
  }
  CHECK(ot::screenForState(q.wk.state()) == UiScreen::Main);
  CHECK(q.ui.lastActivityUs() >= clr);
}

// Wake-swallow (tested here with the real recognizer): the app sets screenAwake = ot::touchAwake(panel level, wake
// redraw pending) before every sample and reports activity at every
// TouchDown. Only an OFF panel
// swallows; on a DIM panel taps and holds work, and the DOWN brightens it.
struct TouchRig {
  ot::TouchGesture g;
  UiPolicy ui;
  ot::GestureContext ctx;
  std::vector<ot::GestureEvent> ev;
  int64_t t = 0;
  explicit TouchRig(bool mainScreen) {
    ui.reset(0);
    ctx.screen = mainScreen ? ot::TOUCH_SCREEN_MAIN : ot::TOUCH_SCREEN_IDLE;
    ctx.paceRect = ot::PACE_RECT;
    ctx.stopRect = ot::STOP_RECT;
    ctx.stopVisible = mainScreen;
    sample(0, false, 0, 0);  // no contact at boot
  }
  void sample(int64_t at, bool valid, int x, int y) {
    t = at;
    ctx.screenAwake = ot::touchAwake(ui.level(at, false), false);
    g.setContext(ctx, at);
    ot::TouchSample s;
    s.t = at;
    s.valid = valid;
    s.x = static_cast<int16_t>(x);
    s.y = static_cast<int16_t>(y);
    g.onSample(s);
    ot::GestureEvent e;
    while (g.pollEvent(e)) {
      if (e.type == ot::GestureType::TouchDown) ui.activity(e.t);
      ev.push_back(e);
    }
  }
  // A contact of `ms` at (x0,y0) -> (x1,y1) linearly, then released.
  void press(int64_t at, int64_t ms, int x0, int y0, int x1, int y1) {
    const int64_t n = ms * MS / ot::TOUCH_POLL_US;
    for (int64_t k = 0; k <= n; ++k)
      sample(at + k * ot::TOUCH_POLL_US, true, x0 + static_cast<int>((x1 - x0) * k / (n > 0 ? n : 1)),
             y0 + static_cast<int>((y1 - y0) * k / (n > 0 ? n : 1)));
    for (int64_t k = 1; k <= 10; ++k) sample(at + (n + k) * ot::TOUCH_POLL_US, false, 0, 0);
  }
  int count(ot::GestureType ty) const {
    int c = 0;
    for (const ot::GestureEvent& e : ev) c += e.type == ty ? 1 : 0;
    return c;
  }
};

void testWakeSwallow() {
  // The helper itself: only a dark panel (OFF, or waking from OFF) swallows.
  CHECK(ot::touchAwake(Backlight::Full, false));
  CHECK(ot::touchAwake(Backlight::Dim, false));
  CHECK(!ot::touchAwake(Backlight::Off, false));
  CHECK(!ot::touchAwake(Backlight::Full, true));  // the wake redraw is still in progress
  CHECK(!ot::touchAwake(Backlight::Dim, true));
  for (int off = 0; off < 2; ++off) {
    for (int mainScreen = 0; mainScreen < 2; ++mainScreen) {
      TouchRig r(mainScreen != 0);
      const int64_t t0 = off ? OFF + 10 * SEC : DIM + 10 * SEC;
      CHECK(r.ui.level(t0, false) == (off ? Backlight::Off : Backlight::Dim));
      r.press(t0, 200, 160, 70, 160, 70);  // a perfect tap on the pace / idle units label
      CHECK_EQ(r.count(ot::GestureType::Wake), off ? 1 : 0);
      CHECK_EQ(r.count(ot::GestureType::TouchDown), 1);
      CHECK_EQ(r.count(ot::GestureType::TapPace), off ? 0 : 1);  // OFF: no toggle; DIM: toggles
      CHECK(r.ui.level(r.t, false) == Backlight::Full);  // brightened / woken by the DOWN
      CHECK_EQ(r.ui.lastActivityUs(), t0);                  // tDown = the first DOWN sample
      // The next tap (screen now FULL) toggles.
      r.press(r.t + 300 * MS, 200, 160, 70, 160, 70);
      CHECK_EQ(r.count(ot::GestureType::TapPace), off ? 1 : 2);
      CHECK_EQ(r.count(ot::GestureType::Wake), off ? 1 : 0);
    }
    // A contact that lands on a cell (DIM or OFF) and slides onto the strip:
    // no hold (it landed outside the strip).
    TouchRig r(true);
    const int64_t t0 = off ? OFF + 10 * SEC : DIM + 10 * SEC;
    r.press(t0, 3000, 53, 177, 160, 214);
    CHECK_EQ(r.count(ot::GestureType::HoldStart), 0);
    CHECK_EQ(r.count(ot::GestureType::HoldComplete), 0);
    // One that lands right on the strip: OFF -> swallowed, no hold; DIM -> a
    // normal hold that completes, and the screen is FULL again.
    TouchRig s(true);
    s.press(t0, 3000, 160, 214, 160, 214);
    CHECK_EQ(s.count(ot::GestureType::Wake), off ? 1 : 0);
    CHECK_EQ(s.count(ot::GestureType::HoldStart), off ? 0 : 1);
    CHECK_EQ(s.count(ot::GestureType::HoldComplete), off ? 0 : 1);
    CHECK(s.ui.level(s.t, false) == Backlight::Full);
    // Awake: the same press on the strip does start and complete a hold (control).
    TouchRig c(true);
    c.press(10 * SEC, 2100, 160, 214, 160, 214);
    CHECK_EQ(c.count(ot::GestureType::HoldStart), 1);
    CHECK_EQ(c.count(ot::GestureType::HoldComplete), 1);
  }
}

// mulDivRoundU64 vs a 128-bit reference (host only: __int128).
void testMulDiv() {
  using u128 = unsigned __int128;
  auto ref = [](uint64_t a, uint64_t b, uint64_t c) -> uint64_t {
    if (c == 0) return UINT64_MAX;
    const u128 q = (static_cast<u128>(a) * b + c / 2) / c;
    return q > UINT64_MAX ? UINT64_MAX : static_cast<uint64_t>(q);
  };
  const uint64_t edge[] = {0, 1, 2, 3, 999, 1000, 1609344, 0xFFFFFFFFull, 0x100000000ull, 0x7FFFFFFFFFFFFFFFull,
                           0x8000000000000000ull, UINT64_MAX - 1, UINT64_MAX};
  uint32_t mism = 0, n = 0;
  for (uint64_t a : edge)
    for (uint64_t b : edge)
      for (uint64_t c : edge) {
        ++n;
        if (ot::mulDivRoundU64(a, b, c) != ref(a, b, c)) ++mism;
      }
  Rng rng(0x5EED55);
  for (int i = 0; i < 1000000; ++i) {
    const int sa = static_cast<int>(rng.next() % 65), sb = static_cast<int>(rng.next() % 65),
              sc = static_cast<int>(rng.next() % 65);
    const uint64_t a = sa == 64 ? rng.next() : (rng.next() & ((1ull << sa) - 1u));
    const uint64_t b = sb == 64 ? rng.next() : (rng.next() & ((1ull << sb) - 1u));
    const uint64_t c = sc == 64 ? rng.next() : (rng.next() & ((1ull << sc) - 1u));
    ++n;
    if (ot::mulDivRoundU64(a, b, c) != ref(a, b, c)) ++mism;
  }
  std::printf("  mulDivRoundU64: %u cases vs __int128, %u mismatches\n", n, mism);
  CHECK_EQ(mism, 0u);
  CHECK_EQ(ot::mulDivRoundU64(5, 1, 2), 3u);  // 2.5 -> 3 (half up)
  CHECK_EQ(ot::mulDivRoundU64(7, 1, 2), 4u);  // 3.5 -> 4
  CHECK_EQ(ot::mulDivRoundU64(UINT64_MAX, UINT64_MAX, UINT64_MAX), UINT64_MAX);
}

// Property test: random walks, nudges, touches and holds through the real
// Workout + UiPolicy, queried at random (increasing) instants. Invariants:
//  - SUMMARY => FULL;
//  - otherwise the level is exactly the idle-time rule from the last
//    activity / summary end (independent re-computation);
//  - never DIM/OFF before lastActivity + DIM; after a summary ends at c,
//    FULL until c + DIM at least.
void testProperty() {
  Rng rng(0xBACC11);
  uint64_t queries = 0, summaries = 0, dims = 0, offs = 0;
  uint32_t bad = 0;
  for (int run = 0; run < 200; ++run) {
    ot::Config cfg;
    UiConfig uc;
    if (run % 2 == 1) {  // the short-timer set
      cfg.autoEndUs = 30 * SEC;
      cfg.summaryUs = 12 * SEC;
      uc.dimUs = 6 * SEC;
      uc.offUs = 15 * SEC;
    }
    Rig r(cfg, uc);
    int64_t t = 0, lastAct = 0, lastSummaryEnd = INT64_MIN;
    size_t seen = 0;
    for (int step = 0; step < 400; ++step) {
      const int64_t kind = rng.range(0, 9);
      if (kind <= 3) {  // a walk
        const int64_t n = rng.range(1, 60), dt = rng.range(300, 1500) * MS;
        for (int64_t k = 0; k < n; ++k) {
          t += dt;
          r.pulse(t);
          lastAct = t;
        }
      } else if (kind == 4) {  // a touch DOWN
        t += rng.range(1, 30000) * MS;
        r.at(t);
        r.touch(t);
        lastAct = t;
      } else if (kind == 5) {  // a hold-stop
        t += rng.range(1, 3000) * MS;
        r.at(t);
        r.touch(t);
        lastAct = t;
        r.wk.holdBegan(t);
        t += 2 * SEC;
        r.at(t);
        r.wk.holdCompleted(t);
        r.drain();
      } else {  // time passes
        t += rng.range(1, 400) * SEC;
      }
      const Backlight lv = r.at(t);
      for (; seen < r.ev.size(); ++seen) {
        const WorkoutEvent& e = r.ev[seen];
        if (e.from == WorkoutState::Summary && e.to != WorkoutState::Summary) lastSummaryEnd = e.t;
        if (e.to == WorkoutState::Summary && e.from != WorkoutState::Summary) ++summaries;
      }
      ++queries;
      const int64_t ref0 = lastSummaryEnd > lastAct ? lastSummaryEnd : lastAct;
      Backlight want = Backlight::Full;
      if (!r.summary()) {
        const int64_t idle = t - ref0;
        want = idle < uc.dimUs ? Backlight::Full : (idle < uc.offUs ? Backlight::Dim : Backlight::Off);
      }
      if (lv != want) ++bad;
      if (lv == Backlight::Dim) ++dims;
      if (lv == Backlight::Off) ++offs;
    }
  }
  std::printf("  property: 200 runs, %llu queries (%llu dim, %llu off), %llu summaries, %u mismatches\n",
              static_cast<unsigned long long>(queries), static_cast<unsigned long long>(dims),
              static_cast<unsigned long long>(offs), static_cast<unsigned long long>(summaries), bad);
  CHECK_EQ(bad, 0u);
  CHECK(summaries > 100u && dims > 100u && offs > 100u);
}

}  // namespace

int main(int argc, char** argv) {
  gRoot = replay::repoRootFromArgs(argc, argv);
  testBasics();
  test51();
  test52();
  test53();
  test54();
  test55();
  test56();
  test57();
  test58();
  testSummaryReplaced();
  testWakeSwallow();
  testMulDiv();
  testProperty();
  return check::summary("test_ui_policy");
}
