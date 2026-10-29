// test_workout.cpp - host tests for the Workout state machine: tests
// 2.1-2.15, plus the golden comparison against test/golden/workout_*.txt
// (written by the independent oracle; the C++ only reads them).
//
// Literal expected values are hardcoded where they are known, within
// tolerances: pulses/distance exact, times +-1 ms, speeds
// +-0.01 km/h. Floating point appears only in those tolerance checks.
//
// Usage: test_workout --root <repo root> [--dump]   (--dump prints every
// golden case's W/Z lines in the golden-contract format, for diagnosis)
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "fixed.h"
#include "metrics.h"
#include "replay.h"
#include "workout.h"

using ot::HoldResult;
using ot::Workout;
using ot::WorkoutEvent;
using ot::WorkoutEventType;
using ot::WorkoutSnapshot;
using ot::WorkoutState;
using replay::InputKind;
using replay::WorkoutInput;
using replay::WorkoutTrace;

namespace {

std::string gRoot = ".";
bool gDump = false;
const char* const kGoldenDir = "firmware/opentread/test/golden/";
constexpr int64_t SEC = 1000000;
constexpr int64_t MS = 1000;
const char* const kS2 = "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv";
const char* const kS1b = "firmware/opentread/test/data/2026-10-18_1416_S1b_10rev.csv";
const char* const kS1 = "firmware/opentread/test/data/2026-10-18_1323_S1_handturn.csv";

// mm/s -> km/h for the +-0.01 km/h tolerance checks.
double kmh(int32_t mmps) { return static_cast<double>(mmps) * 0.0036; }

struct Rng {  // splitmix64
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed) {}
  uint64_t next() {
    uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  int64_t range(int64_t lo, int64_t hi) {  // inclusive
    return lo + static_cast<int64_t>(next() % static_cast<uint64_t>(hi - lo + 1));
  }
};

bool loadLog(const char* path, replay::Log& log) {
  std::string err;
  if (!replay::loadFile(gRoot + "/" + path, log, err)) {
    std::fprintf(stderr, "  %s\n", err.c_str());
    return false;
  }
  if (log.stats.malformed != 0) std::fprintf(stderr, "  %s: %u malformed lines\n", path, log.stats.malformed);
  return true;
}

// 1-based index of the pulse at t (0 if none).
size_t pulseIndex(const std::vector<int64_t>& pulses, int64_t t) {
  for (size_t i = 0; i < pulses.size(); ++i)
    if (pulses[i] == t) return i + 1;
  return 0;
}

std::vector<int64_t> train(int64_t from, int64_t to, int64_t step) {
  std::vector<int64_t> v;
  for (int64_t t = from; t <= to; t += step) v.push_back(t);
  return v;
}

WorkoutInput hold(InputKind k, int64_t t) { return WorkoutInput{t, k, std::string()}; }

ot::Config cfgWith(int64_t stopUs, int64_t autoEndUs) {
  ot::Config c;
  c.stopUs = stopUs;
  c.autoEndUs = autoEndUs;
  return c;
}

std::vector<const WorkoutEvent*> eventsOf(const WorkoutTrace& tr, WorkoutEventType type) {
  std::vector<const WorkoutEvent*> v;
  for (const WorkoutEvent& e : tr.events)
    if (e.type == type) v.push_back(&e);
  return v;
}

std::vector<const WorkoutEvent*> zEvents(const WorkoutTrace& tr) {
  std::vector<const WorkoutEvent*> v;
  for (const WorkoutEvent& e : tr.events)
    if (replay::isZEvent(e)) v.push_back(&e);
  return v;
}

bool hasW(const std::vector<WorkoutEvent>& evs, int64_t t, WorkoutState from, WorkoutState to, WorkoutEventType type) {
  for (const WorkoutEvent& e : evs)
    if (e.t == t && e.from == from && e.to == to && e.type == type) return true;
  return false;
}
bool hasW(const WorkoutTrace& tr, int64_t t, WorkoutState from, WorkoutState to, WorkoutEventType type) {
  return hasW(tr.events, t, from, to, type);
}

void dumpTrace(const char* name, const WorkoutTrace& tr) {
  if (!gDump) return;
  std::printf("--- %s (P=%u I=%u)\n", name, tr.inputsP, tr.inputsI);
  for (const WorkoutEvent& e : tr.events) {
    std::printf("w=%s\n", replay::formatW(e).c_str());
    if (replay::isZEvent(e)) std::printf("z=%s\n", replay::formatZ(e).c_str());
  }
  std::printf("workouts=%u warmup_fails=%u pauses=%u resumes=%u\n", tr.workouts, tr.warmupFails, tr.pauses,
              tr.resumes);
}

// ------------------------------------------------------------------ cases
// The golden cases (contract: test/golden/README.md, workout_<case>.txt).
struct Case {
  std::string name;
  std::string source;  // repo-relative log path or "synthetic"
  ot::Config cfg;
  std::vector<WorkoutInput> inputs;
};

bool buildLogCase(const char* name, const char* path, const ot::Config& cfg, Case& c) {
  replay::Log log;
  if (!loadLog(path, log)) return false;
  c.name = name;
  c.source = path;
  c.cfg = cfg;
  c.inputs = replay::workoutInputs(log);
  return true;
}

std::vector<WorkoutInput> synth28(bool withWalk) {
  std::vector<WorkoutInput> in = replay::pulseInputs(train(10 * SEC, 80 * SEC, SEC / 2));
  replay::mergeInputs(in, replay::pulseInputs({90 * SEC, 90 * SEC + SEC / 2, 91 * SEC, 91 * SEC + SEC / 2}));
  if (withWalk) replay::mergeInputs(in, replay::pulseInputs(train(110 * SEC, 130 * SEC, SEC / 2)));
  replay::mergeInputs(in, {hold(InputKind::HoldBegin, 50250000), hold(InputKind::HoldComplete, 52250000)});
  return in;
}

std::vector<WorkoutInput> synth213() {
  std::vector<WorkoutInput> in = replay::pulseInputs(train(10 * SEC, 40 * SEC, SEC / 2));
  replay::mergeInputs(in, replay::pulseInputs({50 * SEC}));
  replay::mergeInputs(in, {hold(InputKind::HoldBegin, 11 * SEC), hold(InputKind::HoldComplete, 13 * SEC)});
  return in;
}

// Holds that BEGAN with no stop button
// shown complete in WARMUP -> Ignored (goldens workout_synth_hold_*.txt).
std::vector<WorkoutInput> synthHoldIdleToWarmup() {
  std::vector<WorkoutInput> in = replay::pulseInputs(train(1500000, 20 * SEC, SEC / 2));
  replay::mergeInputs(in, {hold(InputKind::HoldBegin, SEC), hold(InputKind::HoldComplete, 3 * SEC)});
  return in;
}

std::vector<WorkoutInput> synthHoldSummaryToWarmup() {
  std::vector<WorkoutInput> in = replay::pulseInputs(train(10 * SEC, 20 * SEC, SEC / 2));
  replay::mergeInputs(in, replay::pulseInputs(train(141 * SEC, 160 * SEC, SEC / 2)));
  replay::mergeInputs(in, {hold(InputKind::HoldBegin, 21 * SEC), hold(InputKind::HoldComplete, 23 * SEC),
                           hold(InputKind::HoldBegin, 142 * SEC), hold(InputKind::HoldComplete, 144 * SEC)});
  return in;
}

bool s2Hold1105(Case& c) {
  if (!buildLogCase("S2_hold_1105", kS2, ot::Config{}, c)) return false;
  replay::mergeInputs(c.inputs, {hold(InputKind::HoldBegin, 1105 * SEC), hold(InputKind::HoldComplete, 1107 * SEC)});
  return true;
}

bool s2Nudges(Case& c) {
  replay::Log log;
  if (!loadLog(kS2, log)) return false;
  std::vector<int64_t> p;
  for (const replay::PulseLine& x : log.pulses)
    if (x.t > 264 * SEC && x.t < 272 * SEC) p.push_back(x.t);
  c.name = "S2_nudges_264_272";
  c.source = kS2;
  c.cfg = ot::Config{};
  c.inputs = replay::pulseInputs(p);
  return true;
}

std::vector<Case> allCases() {
  std::vector<Case> cs;
  Case c;
  if (buildLogCase("S2_walk", kS2, ot::Config{}, c)) cs.push_back(c);
  if (buildLogCase("S2_walk_auto600", kS2, cfgWith(ot::STOP_US, 600 * SEC), c)) cs.push_back(c);
  if (buildLogCase("S1b_10rev", kS1b, ot::Config{}, c)) cs.push_back(c);
  if (buildLogCase("S1_handturn_stop2000", kS1, cfgWith(2000000, ot::AUTO_END_US), c)) cs.push_back(c);
  if (buildLogCase("S1_handturn_stop2500", kS1, cfgWith(2500000, ot::AUTO_END_US), c)) cs.push_back(c);
  if (buildLogCase("S1_handturn_stop3000", kS1, cfgWith(3000000, ot::AUTO_END_US), c)) cs.push_back(c);
  if (s2Nudges(c)) cs.push_back(c);
  cs.push_back(Case{"synth_2_8a", "synthetic", ot::Config{}, synth28(true)});
  cs.push_back(Case{"synth_2_8b", "synthetic", ot::Config{}, synth28(false)});
  if (s2Hold1105(c)) cs.push_back(c);
  cs.push_back(Case{"synth_2_13", "synthetic", ot::Config{}, synth213()});
  cs.push_back(Case{"synth_hold_idle_to_warmup", "synthetic", ot::Config{}, synthHoldIdleToWarmup()});
  cs.push_back(Case{"synth_hold_summary_to_warmup", "synthetic", ot::Config{}, synthHoldSummaryToWarmup()});
  return cs;
}

const Case* findCase(const std::vector<Case>& cs, const char* name) {
  for (const Case& c : cs)
    if (c.name == name) return &c;
  std::fprintf(stderr, "  case %s not built\n", name);
  return nullptr;
}

WorkoutTrace run(const Case& c) {
  WorkoutTrace tr = replay::replayWorkout(c.inputs, c.cfg);
  dumpTrace(c.name.c_str(), tr);
  CHECK(!tr.overflow);
  CHECK_EQ(tr.holdMismatch, 0u);
  return tr;
}

// Common checks for a single-workout log case ending by auto-end.
struct Expect {
  int64_t confirmT;        // exact (a pulse timestamp)
  size_t confirmIndex;     // 1-based pulse number
  uint32_t pulses;
  int64_t movingUs;        // +-1 ms
  int64_t totalUs;         // +-1 ms
  int64_t autoEndT;        // exact
  double topKmh;           // +-0.01 (negative = not given)
};

void checkWorkout(const char* what, const std::vector<int64_t>& pulses, const WorkoutEvent& conf,
                  const WorkoutEvent& z, const Expect& x) {
  const uint64_t failsBefore = static_cast<uint64_t>(check::failures());
  CHECK_EQ(conf.t, x.confirmT);
  CHECK_EQ(pulseIndex(pulses, conf.t), x.confirmIndex);
  CHECK(z.type == WorkoutEventType::AutoEnded);
  CHECK_EQ(z.t, x.autoEndT);
  CHECK_EQ(z.summary.pulses, x.pulses);
  CHECK_NEAR(z.summary.movingUs, x.movingUs, MS);
  CHECK_NEAR(z.summary.totalUs, x.totalUs, MS);
  CHECK_EQ(z.summary.totalUs, x.autoEndT - x.confirmT);  // total = auto-end - confirm
  if (x.topKmh >= 0) CHECK_NEAR(kmh(z.summary.topMmps), x.topKmh, 0.01);
  CHECK(z.summary.movingUs <= z.summary.totalUs);
  if (static_cast<uint64_t>(check::failures()) != failsBefore)
    std::fprintf(stderr, "  %s: confirm %s; Z %s\n", what, replay::formatW(conf).c_str(), replay::formatZ(z).c_str());
}

// ---------------------------------------------------------------- 2.1
void test21(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "S2_walk");
  if (!c) { CHECK(false); return; }
  std::vector<int64_t> p;
  for (const WorkoutInput& i : c->inputs) p.push_back(i.t);
  CHECK_EQ(p.size(), static_cast<size_t>(428));
  const WorkoutTrace tr = run(*c);
  CHECK_EQ(tr.workouts, 1u);
  const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
  const auto z = zEvents(tr);
  CHECK_EQ(conf.size(), static_cast<size_t>(1));
  CHECK_EQ(z.size(), static_cast<size_t>(1));
  if (conf.size() != 1 || z.size() != 1) return;
  // Bout start 17.626 s (the bout the confirm belongs to).
  const auto bs = eventsOf(tr, WorkoutEventType::BoutStarted);
  CHECK(!bs.empty() && bs.back()->t <= conf[0]->t);
  if (!bs.empty()) CHECK_NEAR(bs.back()->t, 17626000, MS);
  checkWorkout("2.1", p, *conf[0], *z[0], {23003940, 11, 417, 217726000, 2130799000, 2153803054, 12.384});
  CHECK_EQ(ot::pulsesToUm(z[0]->summary.pulses), static_cast<uint64_t>(249115800));  // 249.116 m
  CHECK_EQ(pulseIndex(p, 1253803054), static_cast<size_t>(428));                      // last pulse
  CHECK_NEAR(kmh(z[0]->summary.avgMmps), 4.119, 0.01);
  CHECK_EQ(z[0]->summary.avgMmps, ot::averageMmps(417, z[0]->summary.movingUs));
  // "6 pauses": the in-workout pauses that were resumed (the final pause
  // before the auto-end is the 7th PAUSED entry).
  CHECK_EQ(tr.resumes, 6u);
  CHECK_EQ(tr.pauses, 7u);
}

// ---------------------------------------------------------------- 2.2
void test22(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "S2_walk_auto600");
  if (!c) { CHECK(false); return; }
  std::vector<int64_t> p;
  for (const WorkoutInput& i : c->inputs) p.push_back(i.t);
  const WorkoutTrace tr = run(*c);
  CHECK_EQ(tr.workouts, 2u);
  const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
  const auto z = zEvents(tr);
  CHECK_EQ(conf.size(), static_cast<size_t>(2));
  CHECK_EQ(z.size(), static_cast<size_t>(2));
  if (conf.size() != 2 || z.size() != 2) return;
  checkWorkout("2.2#1", p, *conf[0], *z[0], {23003940, 11, 97, 64848000, 848145000, 271148699 + 600 * SEC, 4.317});
  CHECK_EQ(ot::pulsesToUm(97) / 1000u, static_cast<uint64_t>(57947));  // 57.948 m (57.9478)
  checkWorkout("2.2#2", p, *conf[1], *z[1], {1070617884, 117, 311, 147662000, 783185000, 1853803054, 12.384});
  CHECK_EQ(z[0]->summary.workoutId, 1u);
  CHECK_EQ(z[1]->summary.workoutId, 2u);
}

// ---------------------------------------------------------------- 2.3
void test23(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "S1b_10rev");
  if (!c) { CHECK(false); return; }
  std::vector<int64_t> p;
  for (const WorkoutInput& i : c->inputs) p.push_back(i.t);
  CHECK_EQ(p.size(), static_cast<size_t>(61));
  const WorkoutTrace tr = run(*c);
  CHECK_EQ(tr.workouts, 1u);
  const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
  const auto z = zEvents(tr);
  if (conf.size() != 1 || z.size() != 1) { CHECK(false); return; }
  checkWorkout("2.3", p, *conf[0], *z[0], {23676924, 6, 55, 46654000, 944154000, 67830484 + 900 * SEC, 3.709});
  CHECK_EQ(ot::pulsesToUm(55), static_cast<uint64_t>(32857000));  // 32.857 m
  CHECK_EQ(tr.resumes, 0u);  // "0 pauses" (one continuous bout; only the final pause before auto-end)
  CHECK_EQ(tr.pauses, 1u);
}

// ---------------------------------------------------------------- 2.4
void test24(const std::vector<Case>& cs) {
  const Case* c20 = findCase(cs, "S1_handturn_stop2000");
  const Case* c25 = findCase(cs, "S1_handturn_stop2500");
  const Case* c30 = findCase(cs, "S1_handturn_stop3000");
  if (!c20 || !c25 || !c30) { CHECK(false); return; }
  std::vector<int64_t> p;
  for (const WorkoutInput& i : c25->inputs) p.push_back(i.t);
  CHECK_EQ(p.size(), static_cast<size_t>(164));
  {
    const WorkoutTrace tr = run(*c20);
    CHECK_EQ(tr.warmupFails, 10u);
    CHECK_EQ(tr.workouts, 1u);
    const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
    const auto z = zEvents(tr);
    if (conf.size() == 1 && z.size() == 1) {
      CHECK_EQ(pulseIndex(p, conf[0]->t), static_cast<size_t>(17));
      CHECK_NEAR(conf[0]->t, 71771000, MS);
      CHECK_EQ(z[0]->summary.pulses, 147u);
      CHECK_NEAR(z[0]->summary.movingUs, 107852000, MS);
    } else {
      CHECK(false);
    }
    CHECK_EQ(tr.resumes, 7u);
  }
  {
    const WorkoutTrace tr = run(*c25);
    CHECK_EQ(tr.workouts, 1u);
    const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
    const auto z = zEvents(tr);
    if (conf.size() == 1 && z.size() == 1) {
      checkWorkout("2.4/2.5", p, *conf[0], *z[0], {41869449, 5, 159, 129437000, 1051489000, 193358584 + 900 * SEC, -1});
      CHECK_EQ(ot::pulsesToUm(159), static_cast<uint64_t>(94986600));  // 94.987 m
    } else {
      CHECK(false);
    }
    CHECK_EQ(tr.resumes, 11u);
  }
  {
    const WorkoutTrace tr = run(*c30);
    CHECK_EQ(tr.workouts, 1u);
    const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
    const auto z = zEvents(tr);
    if (conf.size() == 1 && z.size() == 1) {
      CHECK_EQ(pulseIndex(p, conf[0]->t), static_cast<size_t>(5));
      CHECK_EQ(z[0]->summary.pulses, 159u);
      CHECK_NEAR(z[0]->summary.movingUs, 141856000, MS);
    } else {
      CHECK(false);
    }
    CHECK_EQ(tr.resumes, 6u);
  }
}

// ---------------------------------------------------------------- 2.5
void test25(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "S2_nudges_264_272");
  if (!c) { CHECK(false); return; }
  CHECK_EQ(c->inputs.size(), static_cast<size_t>(6));
  const WorkoutTrace tr = run(*c);
  CHECK_EQ(tr.workouts, 0u);
  CHECK_EQ(tr.warmupFails, 2u);
  CHECK(zEvents(tr).empty());
  // 2 and 4 pulses: count pulses per bout (bout_start .. warmup_fail).
  std::vector<uint32_t> perBout;
  size_t ev = 0;
  uint32_t n = 0;
  for (const WorkoutInput& in : c->inputs) {
    while (ev < tr.events.size() && tr.events[ev].t < in.t) {
      if (tr.events[ev].type == WorkoutEventType::WarmupFailed) { perBout.push_back(n); n = 0; }
      ++ev;
    }
    ++n;
  }
  for (; ev < tr.events.size(); ++ev)
    if (tr.events[ev].type == WorkoutEventType::WarmupFailed) { perBout.push_back(n); n = 0; }
  CHECK(perBout == std::vector<uint32_t>({2u, 4u}));
}

// ---------------------------------------------------------------- 2.6
WorkoutEvent lastEvent(Workout& w, std::vector<WorkoutEvent>& all) {
  WorkoutEvent e;
  while (w.pollEvent(e)) all.push_back(e);
  return all.empty() ? WorkoutEvent{} : all.back();
}

void test26() {
  // Confirm: t - boutStart = 4.999999 s -> no; 5.000000 s -> yes (at that pulse).
  for (int yes = 0; yes < 2; ++yes) {
    Workout w;
    std::vector<WorkoutEvent> ev;
    const int64_t t0 = 100 * SEC;
    for (int64_t t = t0; t <= t0 + 4 * SEC; t += SEC) w.onPulse(t, 597);
    const int64_t tc = t0 + (yes ? 5 * SEC : 5 * SEC - 1);
    w.onPulse(tc, 597);
    lastEvent(w, ev);
    CHECK(w.state() == (yes ? WorkoutState::Running : WorkoutState::Warmup));
    if (yes) CHECK(hasW(ev, tc, WorkoutState::Warmup, WorkoutState::Running,
                        WorkoutEventType::Confirmed));
  }
  // Pause: now - last = 2.500000 s -> no; 2.500001 s -> yes, stamped last + 2.5 s.
  // A pulse exactly 2.500000 s after the last one continues the bout.
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    for (int64_t t = 0; t <= 6 * SEC; t += SEC) w.onPulse(t, 597);  // confirm at 5 s
    CHECK(w.state() == WorkoutState::Running);
    w.tick(6 * SEC + 2500000);
    CHECK(w.state() == WorkoutState::Running);
    w.onPulse(6 * SEC + 2500000, 400);  // still RUNNING: no pause, no resume
    lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Running);
    CHECK(ev.back().type == WorkoutEventType::Confirmed);
    const int64_t last = 6 * SEC + 2500000;
    w.tick(last + 2500000);
    CHECK(w.state() == WorkoutState::Running);
    w.tick(last + 2500001);
    const WorkoutEvent e = lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Paused);
    CHECK(e.type == WorkoutEventType::Paused);
    CHECK_EQ(e.t, last + 2500000);
    // Auto-end: 900 s exactly -> no; +1 us -> yes, stamped last + 900 s.
    w.tick(last + 900 * SEC);
    CHECK(w.state() == WorkoutState::Paused);
    w.tick(last + 900 * SEC + 1);
    const WorkoutEvent a = lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Summary);
    CHECK(a.type == WorkoutEventType::AutoEnded);
    CHECK_EQ(a.t, last + 900 * SEC);
    CHECK_EQ(a.summary.totalUs, last + 900 * SEC - 5 * SEC);
    // Summary clears at exactly 120.000000 s.
    w.tick(a.t + 120 * SEC - 1);
    CHECK(w.state() == WorkoutState::Summary);
    w.tick(a.t + 120 * SEC);
    const WorkoutEvent s = lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Idle);
    CHECK(s.type == WorkoutEventType::SummaryCleared);
    CHECK_EQ(s.t, a.t + 120 * SEC);
    CHECK(w.summary() == nullptr);
  }
  // Warm-up fail boundary: a pulse 2.500000 s after the last keeps the bout;
  // 2.500001 s of stillness fails it, stamped last + 2.5 s.
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    w.onPulse(10 * SEC, 0);
    w.onPulse(12 * SEC + 500000, 597);
    CHECK(w.state() == WorkoutState::Warmup);
    w.tick(15 * SEC + 1);
    const WorkoutEvent e = lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Idle);
    CHECK(e.type == WorkoutEventType::WarmupFailed);
    CHECK_EQ(e.t, 15 * SEC);
  }
  // A pulse exactly at the confirm instant of a bout whose previous pulse is
  // 2.5 s earlier confirms (both boundaries inclusive at once).
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    w.onPulse(0, 0);
    w.onPulse(2500000, 239);
    w.onPulse(5 * SEC, 239);
    lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Running);
    CHECK_EQ(ev.back().t, 5 * SEC);
  }
  // Two transitions in one tick after a 20 min stall: pause and auto-end
  // stamped at their own instants, in order; then summary_timeout too.
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    for (int64_t t = 0; t <= 8 * SEC; t += SEC / 2) w.onPulse(t, 1195);
    lastEvent(w, ev);
    const size_t before = ev.size();
    w.tick(8 * SEC + 1200 * SEC);
    lastEvent(w, ev);
    CHECK_EQ(ev.size(), before + 3);
    if (ev.size() == before + 3) {
      CHECK(ev[before].type == WorkoutEventType::Paused);
      CHECK_EQ(ev[before].t, 8 * SEC + 2500000);
      CHECK(ev[before + 1].type == WorkoutEventType::AutoEnded);
      CHECK_EQ(ev[before + 1].t, 8 * SEC + 900 * SEC);
      CHECK(ev[before + 2].type == WorkoutEventType::SummaryCleared);
      CHECK_EQ(ev[before + 2].t, 8 * SEC + 1020 * SEC);
    }
  }
  // Tie: a bout warming up behind the summary fails at the same instant the
  // summary clears -> summary_timeout (">=") first (SUMMARY->WARMUP), then
  // warmup_fail (WARMUP->IDLE), both at that instant.
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    for (int64_t t = 0; t <= 8 * SEC; t += SEC / 2) w.onPulse(t, 1195);
    w.tick(8 * SEC + 900 * SEC + 1);  // auto-end at 908 s, summary until 1028 s
    CHECK(w.state() == WorkoutState::Summary);
    w.onPulse(1025 * SEC + 500000, 0);  // bout start; fails at 1028 s = summary clear
    w.tick(1028 * SEC + 1);
    lastEvent(w, ev);
    const size_t n = ev.size();
    CHECK(n >= 2);
    if (n >= 2) {
      CHECK(ev[n - 2].type == WorkoutEventType::SummaryCleared && ev[n - 2].to == WorkoutState::Warmup);
      CHECK_EQ(ev[n - 2].t, 1028 * SEC);
      CHECK(ev[n - 1].type == WorkoutEventType::WarmupFailed && ev[n - 1].from == WorkoutState::Warmup &&
            ev[n - 1].to == WorkoutState::Idle);
      CHECK_EQ(ev[n - 1].t, 1028 * SEC);
    }
  }
}

// ---------------------------------------------------------------- 2.7
// A running workout: pulses every 0.5 s from 0 (confirm at 5.0 s), up to `end`.
void runTo(Workout& w, int64_t end) {
  for (int64_t t = 0; t <= end; t += SEC / 2) w.onPulse(t, t == 0 ? 0 : 1195);
}

void test27() {
  std::vector<WorkoutEvent> ev;
  // IDLE: ignored.
  {
    Workout w;
    w.holdBegan(SEC);
    CHECK(w.snapshot(SEC).holdActive);
    CHECK(w.holdCompleted(3 * SEC) == HoldResult::Ignored);
    CHECK(w.state() == WorkoutState::Idle);
    CHECK(!w.snapshot(3 * SEC).rearmBlocked);
    WorkoutEvent e;
    CHECK(!w.pollEvent(e));
  }
  // No active hold: ignored.
  {
    Workout w;
    runTo(w, 10 * SEC);
    CHECK(w.holdCompleted(10 * SEC) == HoldResult::Ignored);
    CHECK(w.state() == WorkoutState::Running);
  }
  // WARMUP -> IDLE, nothing recorded, re-arm block.
  {
    Workout w;
    w.onPulse(0, 0);
    w.onPulse(SEC / 2, 1195);
    w.holdBegan(SEC);
    const WorkoutSnapshot s = w.snapshot(SEC + SEC / 2);
    CHECK(s.holdActive && s.wkPulses == 0 && s.movingUs == 0 && s.totalUs == 0);
    w.onPulse(SEC, 1195);
    w.onPulse(SEC + SEC / 2, 1195);
    CHECK(w.holdCompleted(3 * SEC) == HoldResult::Ok);
    ev.clear();
    lastEvent(w, ev);
    CHECK(w.state() == WorkoutState::Idle);
    CHECK(w.summary() == nullptr);
    CHECK(w.snapshot(3 * SEC).rearmBlocked);
    CHECK(hasW(ev, 3 * SEC, WorkoutState::Warmup, WorkoutState::Idle,
               WorkoutEventType::WarmupCancelled));
    // Belt already still > STOP at the hold-complete instant? No (last 1.5 s,
    // hold at 3.0 s): the re-arm comes at 1.5 + 2.5 = 4.0 s.
    w.tick(4 * SEC + 1);
    const WorkoutEvent r = lastEvent(w, ev);
    CHECK(r.type == WorkoutEventType::Rearmed);
    CHECK_EQ(r.t, 4 * SEC);
    CHECK(!w.snapshot(4 * SEC + 1).rearmBlocked);
  }
  // RUNNING -> SUMMARY with the hold-begin snapshot; pulses during the hold
  // are excluded; the snapshot is frozen during the hold.
  {
    Workout w;
    runTo(w, 20 * SEC);  // confirm 5.0, pulses 5.5..20.0 -> 30
    const WorkoutSnapshot pre = w.snapshot(20 * SEC + 100000);
    CHECK_EQ(pre.wkPulses, 30u);
    w.holdBegan(20 * SEC + 100000);
    for (int64_t t = 20 * SEC + SEC / 2; t <= 22 * SEC; t += SEC / 2) w.onPulse(t, 1195);  // 4 during the hold
    const WorkoutSnapshot mid = w.snapshot(22 * SEC);
    CHECK(mid.holdActive);
    CHECK_EQ(mid.wkPulses, 30u);
    CHECK_EQ(mid.movingUs, pre.movingUs);
    CHECK_EQ(mid.totalUs, pre.totalUs);
    CHECK_EQ(mid.totalUs, 15 * SEC + 100000);
    CHECK(w.holdCompleted(22 * SEC + 100000) == HoldResult::Ok);
    ev.clear();
    const WorkoutEvent e = lastEvent(w, ev);
    CHECK(e.type == WorkoutEventType::Stopped);
    CHECK_EQ(e.t, 22 * SEC + 100000);
    CHECK_EQ(e.summary.pulses, 30u);
    CHECK_EQ(e.summary.totalUs, 15 * SEC + 100000);
    CHECK_EQ(e.summary.movingUs, pre.movingUs);
    CHECK(w.summary() != nullptr && w.summary()->pulses == 30u);
    const WorkoutSnapshot post = w.snapshot(22 * SEC + 100000);
    CHECK(post.state == WorkoutState::Summary && post.rearmBlocked && !post.holdActive);
    CHECK_EQ(post.wkPulses, 30u);
    CHECK_EQ(post.totalUs, pre.totalUs);
  }
  // PAUSED -> SUMMARY with the snapshot; belt still > STOP at completion ->
  // re-arm at the hold-complete instant.
  {
    Workout w;
    runTo(w, 20 * SEC);
    w.tick(30 * SEC);
    CHECK(w.state() == WorkoutState::Paused);
    ev.clear();
    lastEvent(w, ev);  // drain bout_start / warmup_ok / pause
    w.holdBegan(30 * SEC);
    const WorkoutSnapshot pre = w.snapshot(30 * SEC);
    CHECK_EQ(pre.totalUs, 25 * SEC);
    CHECK(w.holdCompleted(32 * SEC) == HoldResult::Ok);
    ev.clear();
    lastEvent(w, ev);
    CHECK_EQ(ev.size(), static_cast<size_t>(2));
    if (ev.size() == 2) {
      CHECK(ev[0].type == WorkoutEventType::Stopped && ev[0].from == WorkoutState::Paused);
      CHECK_EQ(ev[0].summary.totalUs, 25 * SEC);
      CHECK(ev[1].type == WorkoutEventType::Rearmed);
      CHECK_EQ(ev[1].t, 32 * SEC);
    }
  }
  // SUMMARY: ignored.
  {
    Workout w;
    runTo(w, 20 * SEC);
    w.holdBegan(21 * SEC);
    CHECK(w.holdCompleted(23 * SEC) == HoldResult::Ok);
    w.holdBegan(24 * SEC);
    CHECK(w.holdCompleted(26 * SEC) == HoldResult::Ignored);
    CHECK(w.state() == WorkoutState::Summary);
  }
  // A hold voided by a confirm during the hold.
  {
    Workout w;
    for (int64_t t = 0; t <= 4 * SEC; t += SEC / 2) w.onPulse(t, 1195);
    w.holdBegan(4 * SEC + 100000);
    w.onPulse(4 * SEC + SEC / 2, 1195);
    w.onPulse(5 * SEC, 1195);  // confirm
    CHECK(w.state() == WorkoutState::Running);
    w.onPulse(5 * SEC + SEC / 2, 1195);
    CHECK(w.holdCompleted(6 * SEC + 100000) == HoldResult::Void);
    CHECK(w.state() == WorkoutState::Running);
    CHECK(!w.snapshot(6 * SEC + 100000).rearmBlocked);
    CHECK(!w.snapshot(6 * SEC + 100000).holdActive);
  }
  // A hold voided by an auto-end during the hold (hold begun in PAUSED).
  {
    Workout w;
    runTo(w, 20 * SEC);
    w.holdBegan(919 * SEC);
    CHECK(w.state() == WorkoutState::Paused);
    CHECK(w.holdCompleted(921 * SEC) == HoldResult::Void);  // auto-end at 920 s
    CHECK(w.state() == WorkoutState::Summary);
    CHECK(w.summary() != nullptr && w.summary()->endReason == ot::EndReason::AutoEnd);
    CHECK(!w.snapshot(921 * SEC).rearmBlocked);
  }
  // RUNNING <-> PAUSED during a hold doesn't void it.
  {
    Workout w;
    runTo(w, 20 * SEC);
    w.holdBegan(21 * SEC);
    w.tick(22 * SEC + 600000);  // pause at 22.5
    CHECK(w.state() == WorkoutState::Paused);
    w.onPulse(22 * SEC + 700000, 0);  // resume
    CHECK(w.state() == WorkoutState::Running);
    CHECK(w.holdCompleted(23 * SEC) == HoldResult::Ok);
    CHECK(w.summary() != nullptr && w.summary()->pulses == 30u);
  }
  // Abort: live values return (>= the frozen ones), completion is ignored.
  {
    Workout w;
    runTo(w, 20 * SEC);
    w.holdBegan(20 * SEC);
    w.onPulse(20 * SEC + SEC / 2, 1195);
    CHECK_EQ(w.snapshot(20 * SEC + SEC / 2).wkPulses, 30u);
    CHECK_EQ(w.holdAborted(20 * SEC + SEC / 2), 20 * SEC + SEC / 2);
    CHECK_EQ(w.snapshot(20 * SEC + SEC / 2).wkPulses, 31u);
    CHECK(w.holdCompleted(22 * SEC) == HoldResult::Ignored);
    CHECK(w.state() == WorkoutState::Running);
  }
}

// ---------------------------------------------------------------- 2.8
void test28(const std::vector<Case>& cs) {
  const Case* a = findCase(cs, "synth_2_8a");
  const Case* b = findCase(cs, "synth_2_8b");
  if (!a || !b) { CHECK(false); return; }
  const WorkoutTrace ta = run(*a);
  using S = WorkoutState;
  using E = WorkoutEventType;
  struct W { int64_t t; S from, to; E type; };
  const W want[] = {
      {10 * SEC, S::Idle, S::Warmup, E::BoutStarted},
      {15 * SEC, S::Warmup, S::Running, E::Confirmed},
      {52250000, S::Running, S::Summary, E::Stopped},
      {82500000, S::Summary, S::Summary, E::Rearmed},
      {90 * SEC, S::Summary, S::Summary, E::BoutStarted},
      {94 * SEC, S::Summary, S::Summary, E::WarmupFailed},
      {110 * SEC, S::Summary, S::Summary, E::BoutStarted},
      {115 * SEC, S::Summary, S::Running, E::Confirmed},
  };
  const size_t nw = sizeof(want) / sizeof(want[0]);
  CHECK(ta.events.size() >= nw);
  for (size_t i = 0; i < nw && i < ta.events.size(); ++i) {
    CHECK_EQ(ta.events[i].t, want[i].t);
    CHECK(ta.events[i].from == want[i].from && ta.events[i].to == want[i].to && ta.events[i].type == want[i].type);
  }
  const auto z = zEvents(ta);
  CHECK(!z.empty());
  if (!z.empty()) {
    CHECK_EQ(z[0]->t, 52250000);
    CHECK_EQ(z[0]->summary.pulses, 70u);
    CHECK_EQ(ot::pulsesToUm(70), static_cast<uint64_t>(41818000));  // 41.818 m
    CHECK_EQ(z[0]->summary.movingUs, 35250000);
    CHECK_EQ(z[0]->summary.totalUs, 35250000);
    CHECK_EQ(z[0]->summary.topMmps, 1195);
  }
  const WorkoutTrace tb = run(*b);
  CHECK(tb.events.size() >= 7);
  for (size_t i = 0; i < 6 && i < tb.events.size(); ++i) CHECK_EQ(tb.events[i].t, want[i].t);
  if (tb.events.size() >= 7) {
    CHECK_EQ(tb.events[6].t, 172250000);
    CHECK(tb.events[6].type == E::SummaryCleared && tb.events[6].from == S::Summary && tb.events[6].to == S::Idle);
  }
  CHECK_EQ(tb.events.size(), static_cast<size_t>(7));
}

// ---------------------------------------------------------------- 2.9
// The same inputs with tick() every `step` (plus the inputs themselves) give
// the same events as the event-driven replay.
WorkoutTrace tickedReplay(const Case& c, int64_t step) {
  replay::WorkoutReplay r(c.cfg);
  size_t next = 0;
  const int64_t start = c.inputs.empty() ? 0 : c.inputs.front().t - step;
  int64_t end = c.inputs.empty() ? 0 : c.inputs.back().t;
  for (int64_t t = start; t <= end; t += step) r.runUntil(c.inputs, next, t);
  while (next < c.inputs.size()) r.feed(c.inputs[next++]);
  // Then keep ticking at `step` until nothing is pending.
  int64_t d = 0;
  int64_t t = end;
  while (r.workout.nextDeadline(d)) {
    t += step;
    r.tickTo(t);
  }
  return r.trace;
}

bool sameEvents(const WorkoutTrace& a, const WorkoutTrace& b) {
  if (a.events.size() != b.events.size()) return false;
  for (size_t i = 0; i < a.events.size(); ++i) {
    if (replay::formatW(a.events[i]) != replay::formatW(b.events[i])) return false;
    if (replay::isZEvent(a.events[i]) && replay::formatZ(a.events[i]) != replay::formatZ(b.events[i])) return false;
  }
  return true;
}

void test29(const std::vector<Case>& cs) {
  for (const char* name : {"S2_hold_1105", "synth_2_8a", "synth_2_13", "S1_handturn_stop2000"}) {
    const Case* c = findCase(cs, name);
    if (!c) { CHECK(false); continue; }
    const WorkoutTrace ref = replay::replayWorkout(c->inputs, c->cfg);
    const WorkoutTrace fine = tickedReplay(*c, MS);
    const WorkoutTrace coarse = tickedReplay(*c, 3 * SEC);
    if (!CHECK(sameEvents(ref, fine))) std::fprintf(stderr, "  2.9 %s: 1 ms ticks differ\n", name);
    if (!CHECK(sameEvents(ref, coarse))) std::fprintf(stderr, "  2.9 %s: 3 s ticks differ\n", name);
  }
}

// ---------------------------------------------------------------- 2.10 / 2.14
bool legal(const WorkoutEvent& e) {
  using S = WorkoutState;
  using E = WorkoutEventType;
  switch (e.type) {
    case E::BoutStarted: return (e.from == S::Idle && e.to == S::Warmup) || (e.from == S::Summary && e.to == S::Summary);
    case E::Confirmed: return (e.from == S::Warmup || e.from == S::Summary) && e.to == S::Running;
    case E::WarmupFailed: return (e.from == S::Warmup && e.to == S::Idle) || (e.from == S::Summary && e.to == S::Summary);
    case E::WarmupCancelled: return e.from == S::Warmup && e.to == S::Idle;
    case E::Paused: return e.from == S::Running && e.to == S::Paused;
    case E::Resumed: return e.from == S::Paused && e.to == S::Running;
    case E::Stopped: return (e.from == S::Running || e.from == S::Paused) && e.to == S::Summary;
    case E::AutoEnded: return e.from == S::Paused && e.to == S::Summary;
    case E::SummaryCleared: return e.from == S::Summary && (e.to == S::Idle || e.to == S::Warmup);
    case E::Rearmed: return e.from == e.to && (e.from == S::Idle || e.from == S::Summary);
  }
  return false;
}

// Snapshot invariants between consecutive samples (2.10 / 2.14).
struct Monitor {
  bool have = false;
  WorkoutSnapshot prev;
  int64_t prevT = 0;
  uint64_t samples = 0, violations = 0;
  void sample(const WorkoutSnapshot& s, int64_t t) {
    ++samples;
    bool ok = s.movingUs >= 0 && s.movingUs <= s.totalUs;
    if (have && s.workoutId != 0 && s.workoutId == prev.workoutId && t >= prevT) {
      ok = ok && s.wkPulses >= prev.wkPulses && s.movingUs >= prev.movingUs && s.totalUs >= prev.totalUs;
      // Distance frozen in PAUSED. (A hold abort may legally jump the frozen
      // hold-begin values forward to the live ones, so not across a hold.)
      if (s.state == WorkoutState::Paused && prev.state == WorkoutState::Paused && !s.holdActive && !prev.holdActive)
        ok = ok && s.wkPulses == prev.wkPulses;
    }
    if (s.state == WorkoutState::Idle || s.state == WorkoutState::Warmup)
      ok = ok && s.wkPulses == 0 && s.movingUs == 0 && s.totalUs == 0 && s.workoutId == 0;
    if (!ok) {
      if (++violations <= 5)
        std::fprintf(stderr, "  invariant: t=%lld state=%s id=%u wk=%u mv=%lld tot=%lld (prev id=%u wk=%u mv=%lld tot=%lld)\n",
                     static_cast<long long>(t), ot::workoutStateName(s.state), s.workoutId, s.wkPulses,
                     static_cast<long long>(s.movingUs), static_cast<long long>(s.totalUs), prev.workoutId,
                     prev.wkPulses, static_cast<long long>(prev.movingUs), static_cast<long long>(prev.totalUs));
    }
    prev = s;
    prevT = t;
    have = true;
  }
};

struct EventMonitor {
  bool have = false;
  WorkoutState state = WorkoutState::Idle;
  int64_t lastT = 0;
  uint32_t lastId = 0;
  uint64_t events = 0, bad = 0;
  void check(const std::vector<WorkoutEvent>& evs, size_t from) {
    for (size_t i = from; i < evs.size(); ++i) {
      const WorkoutEvent& e = evs[i];
      ++events;
      bool ok = legal(e) && e.from == state && (!have || e.t >= lastT);
      if (replay::isZEvent(e)) {
        ok = ok && e.summary.workoutId > lastId && e.summary.movingUs <= e.summary.totalUs;
        lastId = e.summary.workoutId;  // ids strictly increase: never reused
      }
      if (!ok && ++bad <= 5)
        std::fprintf(stderr, "  illegal/out-of-order event: %s (state was %s)\n", replay::formatW(e).c_str(),
                     ot::workoutStateName(state));
      state = e.to;
      lastT = e.t;
      have = true;
    }
  }
};

// outOfOrder = false: the original 2.10 run (every input at a later t).
// outOfOrder = true (input time clamp): inputs are also fed at
// times EARLIER than what the core already processed: ticks run ahead of the
// next pulse (a pulse earlier than an already-fired tick), and hold calls
// carry a touch-DOWN instant up to 100 ms (sometimes 3 s) in the past. The
// core must process each at t_eff = max(t, latestUs): W timestamps never go
// backwards, moving <= total, workout fields non-decreasing. Snapshots are
// sampled at the clock (= latestUs).
void propertyRun(uint64_t seed, bool outOfOrder) {
  Rng rng(seed);
  replay::WorkoutReplay r;
  Monitor mon;
  EventMonitor em;
  int64_t clock = 1000;     // max t fed so far (what latestUs must be)
  int64_t lastP = 1000;     // last pulse t fed (pulses stay ordered among themselves)
  uint64_t late = 0, latencyBad = 0;
  constexpr uint32_t N = 1000000;
  auto feedAt = [&](int64_t t, InputKind k) {
    if (t < clock) ++late;
    r.feed({t, k, std::string()});
    if (k == InputKind::Pulse) lastP = t;
    clock = std::max(clock, t);
    if (r.workout.latestUs() != clock) ++latencyBad;
  };
  for (uint32_t i = 0; i < N; ++i) {
    const uint64_t k = rng.next() % 1000;
    if (k < 900) {
      // A pulse after a gap drawn from a mix: walking/running intervals,
      // stop-boundary gaps, short and long stops (incl. > AUTO_END).
      const uint64_t g = rng.next() % 1000;
      int64_t gap;
      if (g < 800) gap = rng.range(60000, 1500000);
      else if (g < 900) gap = rng.range(2400000, 2600000);
      else if (g < 970) gap = rng.range(2600000, 30 * SEC);
      else if (g < 995) gap = rng.range(30 * SEC, 200 * SEC);
      else gap = rng.range(800 * SEC, 1300 * SEC);
      // In-order: after the clock. Out-of-order (25%): after the last pulse
      // only, so it may land before a tick / hold that was already processed.
      const int64_t base = (outOfOrder && rng.next() % 4 == 0) ? lastP : clock;
      feedAt(base + gap, InputKind::Pulse);
    } else if (k < 940) {
      const int64_t t = clock + rng.range(0, 3 * SEC);
      r.tickTo(t);
      clock = t;
    } else {
      int64_t t;
      if (!outOfOrder) {
        t = clock + rng.range(0, k < 970 ? 200000 : (k < 990 ? 2500000 : 500000));
      } else {
        const uint64_t m = rng.next() % 10;
        if (m < 5) t = clock + rng.range(0, 2500000);
        else if (m < 9) t = clock - rng.range(0, 100000);   // touch-DOWN up to 100 ms in the past
        else t = clock - rng.range(0, 3 * SEC);             // far in the past
      }
      if (t < 0) t = 0;
      feedAt(t, k < 970 ? InputKind::HoldBegin : (k < 990 ? InputKind::HoldComplete : InputKind::HoldAbort));
    }
    em.check(r.trace.events, static_cast<size_t>(em.events));
    mon.sample(r.workout.snapshot(clock), clock);
  }
  r.finish();
  em.check(r.trace.events, static_cast<size_t>(em.events));
  std::printf("2.10%s: %u random inputs (%llu earlier than latestUs), %llu events, %u workouts, %llu snapshots\n",
              outOfOrder ? " out-of-order" : "", N, static_cast<unsigned long long>(late),
              static_cast<unsigned long long>(em.events), r.trace.workouts,
              static_cast<unsigned long long>(mon.samples));
  CHECK(!r.trace.overflow);
  CHECK_EQ(em.bad, static_cast<uint64_t>(0));       // legal transitions, W timestamps never backwards
  CHECK_EQ(mon.violations, static_cast<uint64_t>(0));  // moving <= total, fields non-decreasing
  CHECK_EQ(latencyBad, static_cast<uint64_t>(0));    // latestUs = max of every input and tick
  CHECK(r.trace.workouts > 1000u);
  CHECK(eventsOf(r.trace, WorkoutEventType::Stopped).size() > 100u);
  CHECK(eventsOf(r.trace, WorkoutEventType::AutoEnded).size() > 100u);
  CHECK(eventsOf(r.trace, WorkoutEventType::WarmupCancelled).size() > 10u);
  CHECK(eventsOf(r.trace, WorkoutEventType::Rearmed).size() > 100u);
  if (outOfOrder) CHECK(late > 20000u);
}

void test210() {
  propertyRun(0xC0FFEE2010ull, false);
  propertyRun(0xC0FFEE2011ull, true);
}

// 2.14: sample every 100 ms (and after every input) over S2 and every
// synthetic case, including across auto-end and hold completion.
void test214(const std::vector<Case>& cs) {
  for (const char* name : {"S2_walk", "S2_hold_1105", "S2_walk_auto600", "synth_2_8a", "synth_2_8b", "synth_2_13",
                           "S1_handturn_stop2500"}) {
    const Case* c = findCase(cs, name);
    if (!c) { CHECK(false); continue; }
    replay::WorkoutReplay r(c->cfg);
    Monitor mon;
    size_t next = 0;
    int64_t end = c->inputs.back().t;
    int64_t t = c->inputs.front().t - SEC;
    for (; t <= end + 1100 * SEC; t += 100 * MS) {
      while (next < c->inputs.size() && c->inputs[next].t <= t) {
        r.feed(c->inputs[next]);
        mon.sample(r.workout.snapshot(c->inputs[next].t), c->inputs[next].t);
        ++next;
      }
      r.tickTo(t);
      mon.sample(r.workout.snapshot(t), t);
    }
    if (!CHECK_EQ(mon.violations, static_cast<uint64_t>(0))) std::fprintf(stderr, "  2.14 %s\n", name);
    CHECK(mon.samples > 1000u);
  }
}

// ---------------------------------------------------------------- 2.11
void test211(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "S2_walk");
  if (!c) { CHECK(false); return; }
  // Top = max of v at pulse instants while RUNNING (the confirm pulse is the
  // baseline and resume pulses carry v = 0).
  auto runTop = [](const std::vector<WorkoutInput>& in, int32_t& maxV, int32_t& maxLastInterval) {
    replay::WorkoutReplay r;
    maxV = 0;
    maxLastInterval = 0;
    int64_t prevT = -1;
    for (const WorkoutInput& x : in) {
      const WorkoutState before = r.workout.state();
      r.feed(x);
      if (before == WorkoutState::Running && r.workout.state() == WorkoutState::Running) {
        maxV = std::max(maxV, r.metrics.speedAtLastPulseMmps());
        if (prevT >= 0 && x.t > prevT)
          maxLastInterval = std::max(maxLastInterval, ot::spanSpeedMmps(1, static_cast<uint64_t>(x.t - prevT)));
      }
      prevT = x.t;
    }
    r.finish();
    const auto z = zEvents(r.trace);
    return z.empty() ? -1 : z.back()->summary.topMmps;
  };
  int32_t maxV = 0, maxLi = 0;
  const int32_t top = runTop(c->inputs, maxV, maxLi);
  CHECK_EQ(top, maxV);
  CHECK_NEAR(kmh(top), 12.384, 0.01);
  // +25 ms double count on the REAL sprint plateau: [1139, 1143] s after the
  // first S2 pulse (test 1.12's window, test/INTEGRATION_NOTES.md P3), i.e.
  // ~1156.6-1160.6 s of ESP time. (At 1141 s of ESP time the belt is still
  // accelerating and a glitch there doesn't even reach the clean top.)
  int64_t firstP = -1;
  for (const WorkoutInput& x : c->inputs)
    if (x.kind == InputKind::Pulse) { firstP = x.t; break; }
  std::vector<WorkoutInput> in = c->inputs;
  int64_t glitchAt = 0;
  for (const WorkoutInput& x : in)
    if (x.kind == InputKind::Pulse && x.t >= firstP + 1141 * SEC) { glitchAt = x.t + 25000; break; }
  CHECK(glitchAt > firstP + 1139 * SEC && glitchAt < firstP + 1143 * SEC);
  CHECK(glitchAt > 1155 * SEC && glitchAt < 1161 * SEC);
  replay::mergeInputs(in, {WorkoutInput{glitchAt, InputKind::Pulse, std::string()}});
  int32_t gMaxV = 0, gMaxLi = 0;
  const int32_t gTop = runTop(in, gMaxV, gMaxLi);
  std::printf("2.11: top %d mm/s (%.3f km/h); with a +25 ms double count at %.6f s: %d mm/s (%.3f km/h), "
              "last-interval max %d mm/s (%.1f km/h)\n",
              top, kmh(top), static_cast<double>(glitchAt) / 1e6, gTop, kmh(gTop), gMaxLi, kmh(gMaxLi));
  CHECK_EQ(gTop, gMaxV);
  CHECK(gTop > top);            // the glitch is on the plateau: it does raise the top ...
  CHECK(kmh(gTop) <= 13.1);     // ... but span3 caps it (~+6%)
  CHECK(kmh(gMaxLi) > 80.0);    // a last-interval estimator would report ~86 km/h
}

// ---------------------------------------------------------------- 2.12
void test212(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "S2_hold_1105");
  if (!c) { CHECK(false); return; }
  std::vector<int64_t> p;
  for (const WorkoutInput& i : c->inputs)
    if (i.kind == InputKind::Pulse) p.push_back(i.t);
  CHECK_EQ(pulseIndex(p, 1115411139), static_cast<size_t>(143));
  CHECK_EQ(pulseIndex(p, 1128075942), static_cast<size_t>(144));
  const WorkoutTrace tr = run(*c);
  // Snapshot at 1105 s from a reference run without the hold.
  replay::WorkoutReplay ref;
  std::vector<WorkoutInput> plain;
  for (const WorkoutInput& i : c->inputs)
    if (i.kind == InputKind::Pulse) plain.push_back(i);
  size_t next = 0;
  ref.runUntil(plain, next, 1105 * SEC);
  const WorkoutSnapshot at1105 = ref.workout.snapshot(1105 * SEC);
  CHECK(at1105.state == WorkoutState::Running);

  const auto stops = eventsOf(tr, WorkoutEventType::Stopped);
  CHECK_EQ(stops.size(), static_cast<size_t>(1));
  if (stops.size() == 1) {
    const WorkoutEvent& s = *stops[0];
    CHECK_EQ(s.t, 1107 * SEC);
    CHECK(s.from == WorkoutState::Running);
    CHECK_EQ(s.summary.pulses, at1105.wkPulses);
    CHECK_EQ(s.summary.movingUs, at1105.movingUs);
    CHECK_EQ(s.summary.totalUs, at1105.totalUs);
    CHECK_EQ(s.summary.totalUs, 1105 * SEC - 23003940);
    CHECK_EQ(s.summary.topMmps, at1105.topMmps);
  }
  // No bout_start while the train continues; rearm at #143 + 2.5 s; the next
  // bout_start only at #144.
  bool sawStop = false, sawRearm = false;
  for (const WorkoutEvent& e : tr.events) {
    if (e.type == WorkoutEventType::Stopped) sawStop = true;
    if (sawStop && !sawRearm && e.type == WorkoutEventType::BoutStarted) CHECK(false);
    if (sawStop && !sawRearm && e.type == WorkoutEventType::Rearmed) {
      sawRearm = true;
      CHECK_EQ(e.t, 1117911139);
      CHECK(e.from == WorkoutState::Summary && e.to == WorkoutState::Summary);
    }
  }
  CHECK(sawRearm);
  const auto bs = eventsOf(tr, WorkoutEventType::BoutStarted);
  const WorkoutEvent* firstAfter = nullptr;
  for (const WorkoutEvent* e : bs)
    if (e->t > 1107 * SEC) { firstAfter = e; break; }
  CHECK(firstAfter != nullptr);
  if (firstAfter) {
    CHECK_EQ(firstAfter->t, 1128075942);
    CHECK(firstAfter->from == WorkoutState::Summary);
  }
  // That bout confirms within the 120 s summary -> SUMMARY -> RUNNING.
  const auto conf = eventsOf(tr, WorkoutEventType::Confirmed);
  CHECK_EQ(conf.size(), static_cast<size_t>(2));
  if (conf.size() == 2) CHECK(conf[1]->from == WorkoutState::Summary && conf[1]->t < 1227 * SEC);
}

// ---------------------------------------------------------------- 2.13
void test213(const std::vector<Case>& cs) {
  const Case* c = findCase(cs, "synth_2_13");
  if (!c) { CHECK(false); return; }
  const WorkoutTrace tr = run(*c);
  using S = WorkoutState;
  using E = WorkoutEventType;
  CHECK(hasW(tr, 10 * SEC, S::Idle, S::Warmup, E::BoutStarted));
  CHECK(hasW(tr, 13 * SEC, S::Warmup, S::Idle, E::WarmupCancelled));
  CHECK(hasW(tr, 42500000, S::Idle, S::Idle, E::Rearmed));
  CHECK(hasW(tr, 50 * SEC, S::Idle, S::Warmup, E::BoutStarted));
  for (const WorkoutEvent& e : tr.events)
    if (e.type == E::BoutStarted) CHECK(e.t == 10 * SEC || e.t == 50 * SEC);
  CHECK_EQ(tr.workouts, 0u);
  CHECK(zEvents(tr).empty());
}

// ---------------------------------------------------------------- 2.15
void test215(const std::vector<Case>& cs) {
  struct Win { const char* name; int64_t from, to; };
  for (const Win& win : {Win{"S2_walk", 17626000, 23003940}, Win{"synth_2_13", 10 * SEC, 13 * SEC}}) {
    const Case* c = findCase(cs, win.name);
    if (!c) { CHECK(false); continue; }
    replay::WorkoutReplay r(c->cfg);
    size_t next = 0;
    uint32_t samples = 0, bad = 0;
    int64_t secondPulse = -1;
    int64_t boutStart = -1;
    for (int64_t t = win.from - 100 * MS; t < win.to; t += 100 * MS) {
      r.runUntil(c->inputs, next, t);
      const WorkoutSnapshot s = r.workout.snapshot(t);
      if (s.state != WorkoutState::Warmup) continue;
      if (boutStart < 0) boutStart = t;
      ++samples;
      if (secondPulse < 0) {
        int pulsesInBout = 0;
        for (size_t i = 0; i < next; ++i)
          if (c->inputs[i].kind == InputKind::Pulse && c->inputs[i].t >= win.from - 1000) ++pulsesInBout;
        if (pulsesInBout >= 2) secondPulse = t;
      }
      const int32_t v = r.metrics.speedMmps(t);
      const bool ok = s.wkPulses == 0 && s.movingUs == 0 && s.totalUs == 0 && (secondPulse < 0 || v > 0);
      if (!ok) ++bad;
    }
    std::printf("2.15 %s: %u WARMUP samples\n", win.name, samples);
    CHECK(samples >= 25u);
    CHECK(secondPulse > 0);
    CHECK_EQ(bad, 0u);
  }
}

// ---------------------------------------------------------------- misc
void testHelpers() {
  CHECK_EQ(ot::movingTailUs(0, ot::STOP_US), static_cast<int64_t>(0));    // resume pulse: vLast = 0
  CHECK_EQ(ot::movingTailUs(-5, ot::STOP_US), static_cast<int64_t>(0));
  CHECK_EQ(ot::movingTailUs(1195, ot::STOP_US), static_cast<int64_t>(999832));
  CHECK_EQ(ot::movingTailUs(1, ot::STOP_US), ot::STOP_US);
  CHECK_EQ(ot::movingTailUs(INT32_MAX, ot::STOP_US), static_cast<int64_t>(0));  // 1194800000 / 2^31 -> 0
  CHECK_EQ(ot::movingTailUs(477, ot::STOP_US), static_cast<int64_t>(2500000));  // 2504821 capped
  CHECK_EQ(ot::movingTailUs(478, ot::STOP_US), static_cast<int64_t>(2499581));
  CHECK_EQ(ot::averageMmps(0, 0), 0);
  CHECK_EQ(ot::averageMmps(70, 35250000), 1186);                    // 41818000000 / 35250000 = 1186.33
  CHECK_EQ(ot::averageMmps(UINT32_MAX, 1), ot::SPEED_SAT_MMPS);     // saturates, no overflow
  CHECK_EQ(ot::averageMmps(1, 1000000), 597);                       // 597.4 -> 597
  CHECK_EQ(ot::averageMmps(1, 1194800), 500);                       // exact
  // Event queue: 8 entries, overflow latches.
  Workout w;
  w.onPulse(0, 0);  // bout_start
  for (int i = 1; i <= 8; ++i) {
    w.tick(static_cast<int64_t>(i) * 10 * SEC);  // warmup_fail
    w.onPulse(static_cast<int64_t>(i) * 10 * SEC + 1, 0);  // bout_start
  }
  CHECK(w.eventOverflow());
  WorkoutEvent e;
  int n = 0;
  while (w.pollEvent(e)) ++n;
  CHECK_EQ(n, 8);
  // reset() restarts ids at 1.
  w.reset();
  CHECK(!w.eventOverflow());
  runTo(w, 6 * SEC);
  CHECK_EQ(w.snapshot(6 * SEC).workoutId, 1u);
  // Names.
  CHECK_EQ(std::string(ot::workoutStateName(WorkoutState::Summary)), std::string("SUMMARY"));
  CHECK_EQ(std::string(ot::workoutEventReason(WorkoutEventType::WarmupCancelled)), std::string("hold_cancel_warmup"));
  CHECK_EQ(std::string(ot::holdResultName(HoldResult::Void)), std::string("void"));
  // Uptime: the same train near 10 days gives the same events shifted.
  const int64_t base = 864000LL * SEC;
  replay::WorkoutReplay a, b;
  for (int64_t t = 0; t <= 40 * SEC; t += SEC / 2) {
    a.feed({t + 1000, InputKind::Pulse, std::string()});
    b.feed({t + base, InputKind::Pulse, std::string()});
  }
  a.finish();
  b.finish();
  CHECK_EQ(a.trace.events.size(), b.trace.events.size());
  for (size_t i = 0; i < a.trace.events.size() && i < b.trace.events.size(); ++i) {
    CHECK_EQ(b.trace.events[i].t - a.trace.events[i].t, base - 1000);
    CHECK_EQ(a.trace.events[i].summary.movingUs, b.trace.events[i].summary.movingUs);
    CHECK_EQ(a.trace.events[i].summary.totalUs, b.trace.events[i].summary.totalUs);
  }
}

// Replay parser: I lines (log grammar) and P+I merge order.
void testReplayInputs() {
  const std::string text =
      "S,100,OpenTread fw=1.0.0 build=00000000 line_level=1\n"
      "P,1000,1,0\n"
      "I,2000,hold_begin,-\n"
      "P,2000,2,1000\n"
      "I,2000,hold_abort,-\n"
      "I,3000,hold_complete,ok\n"
      "I,3000,hold_complete,void\n"
      "I,3000,hold_complete,ignored\n"
      "I,4000,units_toggle,metric\n"
      "I,5000,touch_down,10;20\n"
      "I,5000,touch_up,0;0\n"
      "I,6000,hold_begin\n"            // missing arg
      "I,6000,hold_begin,x\n"          // arg must be '-'
      "I,6000,hold_complete,OK\n"      // case
      "I,6000,hold_complete,ok,1\n"    // extra field
      "I,6000,hold_stop,-\n"           // unknown event
      "I,-6,hold_begin,-\n"            // sign
      "I,6000,touch_down,1;\n"         // empty y
      "I,6000,units_toggle,\n"         // empty
      "I,1500,hold_begin,-\n";         // out of order: sorted before P 2000
  replay::Log log;
  replay::parseText(text, log);
  CHECK_EQ(log.stats.i, 9u);
  CHECK_EQ(log.stats.malformed, 8u);
  CHECK_EQ(log.stats.p, 2u);
  const std::vector<WorkoutInput> in = replay::workoutInputs(log);
  CHECK_EQ(in.size(), static_cast<size_t>(8));  // 2 P + 6 hold I (units/touch skipped)
  if (in.size() == 8) {
    CHECK(in[0].kind == InputKind::Pulse && in[0].t == 1000);
    CHECK(in[1].kind == InputKind::HoldBegin && in[1].t == 1500);
    CHECK(in[2].kind == InputKind::HoldBegin && in[2].t == 2000);  // file order at equal t
    CHECK(in[3].kind == InputKind::Pulse && in[3].t == 2000);
    CHECK(in[4].kind == InputKind::HoldAbort);
    CHECK(in[5].kind == InputKind::HoldComplete && in[5].expect == "ok");
  }
  // A logged result that the replay doesn't reproduce is counted.
  replay::WorkoutReplay r;
  r.feed({10, InputKind::HoldComplete, "ok"});
  CHECK_EQ(r.trace.holdMismatch, 1u);
}

// ------------------------------------------------------------ hold edge cases
// A hold that BEGAN in IDLE or SUMMARY is Ignored even
// though the state is WARMUP when it completes; the warm-up continues.
void testHoldBeganNoButton() {
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    CHECK_EQ(w.holdBegan(SEC), SEC);
    for (int64_t t = SEC + SEC / 2; t <= 2500000; t += SEC / 2) w.onPulse(t, 1195);
    CHECK(w.state() == WorkoutState::Warmup);
    int64_t te = 0;
    CHECK(w.holdCompleted(3 * SEC, &te) == HoldResult::Ignored);
    CHECK_EQ(te, 3 * SEC);
    CHECK(w.state() == WorkoutState::Warmup);
    CHECK(!w.snapshot(3 * SEC).rearmBlocked);
    lastEvent(w, ev);
    for (const WorkoutEvent& e : ev) CHECK(e.type != WorkoutEventType::WarmupCancelled);
    for (int64_t t = 3 * SEC; t <= 6500000; t += SEC / 2) w.onPulse(t, 1195);
    CHECK(w.state() == WorkoutState::Running);  // confirmed at 1.5 + 5 s
    lastEvent(w, ev);
    CHECK(hasW(ev, 6500000, WorkoutState::Warmup, WorkoutState::Running, WorkoutEventType::Confirmed));
  }
  {
    // SUMMARY (re-armed, a bout warming up behind it) -> summary_timeout
    // during the hold -> completes in WARMUP -> Ignored.
    Workout w;
    std::vector<WorkoutEvent> ev;
    runTo(w, 20 * SEC);
    w.holdBegan(21 * SEC);
    CHECK(w.holdCompleted(23 * SEC) == HoldResult::Ok);  // PAUSED at 22.5 -> SUMMARY, re-armed at once
    CHECK(!w.snapshot(23 * SEC).rearmBlocked);
    w.onPulse(141 * SEC, 0);
    w.onPulse(141 * SEC + SEC / 2, 1195);
    w.holdBegan(142 * SEC);
    CHECK(w.state() == WorkoutState::Summary);
    for (int64_t t = 142 * SEC; t <= 144 * SEC; t += SEC / 2) w.onPulse(t, 1195);
    CHECK(w.state() == WorkoutState::Warmup);  // summary_timeout at 143 s, the bout continues
    CHECK(w.holdCompleted(144 * SEC) == HoldResult::Ignored);
    CHECK(w.state() == WorkoutState::Warmup);
    CHECK(!w.snapshot(144 * SEC).rearmBlocked);
    for (int64_t t = 144 * SEC + SEC / 2; t <= 146 * SEC; t += SEC / 2) w.onPulse(t, 1195);
    CHECK(w.state() == WorkoutState::Running);
    lastEvent(w, ev);
    CHECK(hasW(ev, 143 * SEC, WorkoutState::Summary, WorkoutState::Warmup, WorkoutEventType::SummaryCleared));
    CHECK(hasW(ev, 146 * SEC, WorkoutState::Warmup, WorkoutState::Running, WorkoutEventType::Confirmed));
    for (const WorkoutEvent& e : ev) CHECK(e.type != WorkoutEventType::WarmupCancelled);
  }
  // A hold begun in WARMUP still cancels (contrast).
  {
    Workout w;
    w.onPulse(SEC, 0);
    w.holdBegan(SEC + 1);
    CHECK(w.holdCompleted(3 * SEC) == HoldResult::Ok);
    CHECK(w.state() == WorkoutState::Idle);
  }
}

// The input time clamp.
std::string zOf(Workout& w) {
  WorkoutEvent e;
  std::string z;
  while (w.pollEvent(e))
    if (replay::isZEvent(e)) z = replay::formatZ(e);
  return z;
}

void testInputClamp() {
  // (A) Device order: a pulse at 20.25 s is processed, then holdBegan with the
  // touch-DOWN instant 20.24 s (in the past). With and without a tick in
  // between. The hold-begin snapshot is taken at t_eff (never before the
  // processed pulse): moving <= total, and the replay of the logged P + I
  // lines (I at the returned t_eff) gives the same Z.
  for (int withTick = 0; withTick < 2; ++withTick) {
    Workout w;
    ot::Metrics met;                // the device path: Metrics -> speedAtLastPulse -> Workout
    std::vector<WorkoutInput> log;  // what the device would print, in print order
    for (int64_t t = 0; t <= 20 * SEC + SEC / 4; t += (t < 20 * SEC ? SEC / 2 : SEC / 4)) {
      met.onPulse(t);
      w.onPulse(t, met.speedAtLastPulseMmps());
      log.push_back({t, InputKind::Pulse, std::string()});
    }
    CHECK_EQ(log.back().t, 20250000);
    {
      // A snapshot asked for before the last processed input is evaluated at
      // latestUs (20.25 s), so moving (which includes 20.0-20.25) <= total.
      const WorkoutSnapshot sp = w.snapshot(20240000);
      CHECK_EQ(sp.totalUs, 15250000);
      CHECK_EQ(sp.movingUs, 15250000);
    }
    const int64_t want = withTick ? 20300000 : 20250000;
    if (withTick) w.tick(20300000);
    const int64_t tb = w.holdBegan(20240000);
    CHECK_EQ(tb, want);
    log.push_back({tb, InputKind::HoldBegin, std::string()});
    const WorkoutSnapshot s = w.snapshot(20300000);
    CHECK(s.holdActive);
    CHECK_EQ(s.wkPulses, 31u);  // the 20.25 s pulse was processed before the hold
    CHECK(s.movingUs <= s.totalUs);
    CHECK_EQ(s.totalUs, want - 5 * SEC);
    // Even a snapshot asked for in the past is evaluated at latestUs.
    const WorkoutSnapshot past = w.snapshot(20 * SEC);
    CHECK(past.movingUs <= past.totalUs);
    int64_t tc = 0;
    CHECK(w.holdCompleted(22240000, &tc) == HoldResult::Ok);
    CHECK_EQ(tc, 22240000);
    log.push_back({tc, InputKind::HoldComplete, "ok"});
    const std::string z = zOf(w);
    CHECK(!z.empty());
    // Replay of the device's own P + I lines (stable by t: file order on ties).
    std::vector<WorkoutInput> sorted = log;
    std::stable_sort(sorted.begin(), sorted.end(), [](const WorkoutInput& a, const WorkoutInput& b) { return a.t < b.t; });
    const WorkoutTrace tr = replay::replayWorkout(sorted, ot::Config{});
    const auto zr = zEvents(tr);
    CHECK_EQ(zr.size(), static_cast<size_t>(1));
    if (zr.size() == 1) CHECK_EQ(replay::formatZ(*zr[0]), z);
    CHECK_EQ(tr.holdMismatch, 0u);
    if (zr.size() == 1) {
      CHECK(zr[0]->summary.movingUs <= zr[0]->summary.totalUs);
      CHECK_EQ(zr[0]->summary.pulses, 31u);
    }
  }
  // (C) holdBegan with t inside a pause tail that a tick already committed:
  // snapshot at t_eff = 23 s, moving (committed incl. tail) <= total.
  {
    Workout w;
    for (int64_t t = 0; t <= 20 * SEC; t += SEC / 2) w.onPulse(t, t ? 1195 : 0);
    w.tick(23 * SEC);  // pause stamped 22.5 s, tail 0.999832 s committed
    CHECK(w.state() == WorkoutState::Paused);
    CHECK_EQ(w.holdBegan(20 * SEC + SEC / 2), 23 * SEC);
    const WorkoutSnapshot s = w.snapshot(23 * SEC);
    CHECK_EQ(s.movingUs, 15999832);
    CHECK_EQ(s.totalUs, 18 * SEC);
    CHECK(s.movingUs <= s.totalUs);
    CHECK_EQ(w.holdAborted(SEC), 23 * SEC);  // abort in the past -> t_eff too
    CHECK(!w.snapshot(23 * SEC).holdActive);
    w.holdBegan(23 * SEC);
    int64_t te = 0;
    CHECK(w.holdCompleted(22 * SEC, &te) == HoldResult::Ok);  // completion "in the past"
    CHECK_EQ(te, 23 * SEC);                                    // -> reported (and used) at t_eff
    CHECK(w.summary() != nullptr && w.summary()->summaryStartUs == 23 * SEC);
  }
  // (D) A pulse earlier than an already-fired tick: the resume is stamped at
  // t_eff (23 s), never before the pause (22.5 s): W timestamps monotone.
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    for (int64_t t = 0; t <= 20 * SEC; t += SEC / 2) w.onPulse(t, t ? 1195 : 0);
    w.tick(23 * SEC);
    w.onPulse(21 * SEC, 0);
    lastEvent(w, ev);
    CHECK(ev.size() >= 2);
    for (size_t i = 1; i < ev.size(); ++i) CHECK(ev[i].t >= ev[i - 1].t);
    if (ev.size() >= 2) {
      CHECK(ev[ev.size() - 2].type == WorkoutEventType::Paused && ev[ev.size() - 2].t == 22500000);
      CHECK(ev.back().type == WorkoutEventType::Resumed && ev.back().t == 23 * SEC);
    }
    CHECK_EQ(w.latestUs(), 23 * SEC);
    const WorkoutSnapshot s = w.snapshot(23 * SEC);
    CHECK(s.movingUs <= s.totalUs);
    // In-order inputs: t_eff = t.
    CHECK_EQ(w.holdBegan(24 * SEC), 24 * SEC);
    int64_t te = 0;
    CHECK(w.holdCompleted(25 * SEC, &te) == HoldResult::Ok);
    CHECK_EQ(te, 25 * SEC);
  }
  // A tick in the past is a no-op; the pulse that follows uses latestUs.
  {
    Workout w;
    w.tick(10 * SEC);
    w.tick(5 * SEC);
    CHECK_EQ(w.latestUs(), 10 * SEC);
    std::vector<WorkoutEvent> ev;
    w.onPulse(7 * SEC, 0);
    lastEvent(w, ev);
    CHECK(!ev.empty() && ev.back().type == WorkoutEventType::BoutStarted && ev.back().t == 10 * SEC);
    w.reset();
    CHECK_EQ(w.latestUs(), static_cast<int64_t>(0));
  }
}

// Tests that kill mutants that survived a mutation-testing pass.
void testMutantKillers(const std::vector<Case>& cs) {
  // The confirm pulse's speed is the baseline, not part of top.
  {
    Workout w;
    std::vector<WorkoutEvent> ev;
    for (int64_t t = 0; t < 5 * SEC; t += SEC / 2) w.onPulse(t, t ? 1000 : 0);
    w.onPulse(5 * SEC, 3000);  // confirm: fast
    CHECK(w.state() == WorkoutState::Running);
    w.onPulse(5 * SEC + SEC / 2, 1000);
    w.onPulse(6 * SEC, 1100);
    CHECK_EQ(w.snapshot(6 * SEC).topMmps, 1100);
    w.holdBegan(6 * SEC);
    CHECK(w.holdCompleted(8 * SEC) == HoldResult::Ok);
    WorkoutEvent e;
    int32_t ztop = -1;
    while (w.pollEvent(e))
      if (replay::isZEvent(e)) ztop = e.summary.topMmps;
    CHECK_EQ(ztop, 1100);
    // Right after the confirm (no RUNNING pulse yet) top is 0.
    Workout w2;
    for (int64_t t = 0; t <= 5 * SEC; t += SEC / 2) w2.onPulse(t, t == 5 * SEC ? 3000 : 1000);
    CHECK(w2.state() == WorkoutState::Running);
    CHECK_EQ(w2.snapshot(5 * SEC).topMmps, 0);
  }
  // A hold-stop's Z top is the hold-begin snapshot's top, even
  // if the belt speeds up during the hold; the snapshot's top is frozen too.
  {
    Workout w;
    runTo(w, 20 * SEC);  // top 1195
    w.holdBegan(20 * SEC + 100000);
    CHECK_EQ(w.snapshot(20 * SEC + 100000).topMmps, 1195);
    for (int64_t t = 20 * SEC + SEC / 2; t <= 22 * SEC; t += SEC / 2) w.onPulse(t, 2500);
    CHECK_EQ(w.snapshot(22 * SEC).topMmps, 1195);
    CHECK(w.holdCompleted(22 * SEC + 100000) == HoldResult::Ok);
    WorkoutEvent e;
    int32_t ztop = -1;
    while (w.pollEvent(e))
      if (replay::isZEvent(e)) ztop = e.summary.topMmps;
    CHECK_EQ(ztop, 1195);
    CHECK(w.summary() != nullptr && w.summary()->topMmps == 1195);
    CHECK_EQ(w.snapshot(22 * SEC + 100000).topMmps, 1195);
    // ... while an aborted hold lets the live top (2500) through.
    Workout w2;
    runTo(w2, 20 * SEC);
    w2.holdBegan(20 * SEC + 100000);
    w2.onPulse(20 * SEC + SEC / 2, 2500);
    w2.holdAborted(20 * SEC + SEC / 2);
    CHECK_EQ(w2.snapshot(20 * SEC + SEC / 2).topMmps, 2500);
  }
  // nextDeadline is exact (the first `now` at which tick(now) fires).
  {
    Workout w;
    int64_t d = 0;
    CHECK(!w.nextDeadline(d));
    w.onPulse(0, 0);
    CHECK(w.nextDeadline(d) && d == 2500001);  // WARMUP: warmup_fail
    runTo(w, 10 * SEC);
    CHECK(w.state() == WorkoutState::Running);
    CHECK(w.nextDeadline(d) && d == 12500001);  // RUNNING: pause
    w.tick(d - 1);
    CHECK(w.state() == WorkoutState::Running);
    w.tick(d);
    CHECK(w.state() == WorkoutState::Paused);
    CHECK(w.nextDeadline(d) && d == 910 * SEC + 1);  // PAUSED: auto-end
    w.tick(d - 1);
    CHECK(w.state() == WorkoutState::Paused);
    w.tick(d);
    CHECK(w.state() == WorkoutState::Summary);
    CHECK(w.nextDeadline(d) && d == 1030 * SEC);  // SUMMARY: timeout (>=)
    w.tick(d - 1);
    CHECK(w.state() == WorkoutState::Summary);
    w.tick(d);
    CHECK(w.state() == WorkoutState::Idle);
    CHECK(!w.nextDeadline(d));
    // Re-arm: hold-stop while the belt still moves -> due at last + STOP + 1.
    Workout w2;
    runTo(w2, 20 * SEC);
    w2.holdBegan(20 * SEC);
    CHECK(w2.holdCompleted(20 * SEC + SEC / 2) == HoldResult::Ok);
    w2.onPulse(21 * SEC, 1195);
    CHECK(w2.nextDeadline(d) && d == 23500001);
    w2.tick(d - 1);
    CHECK(w2.snapshot(d - 1).rearmBlocked);
    w2.tick(d);
    CHECK(!w2.snapshot(d).rearmBlocked);
  }
  // The confirm at 115 s in 2.8 discards the summary.
  {
    const Case* a = findCase(cs, "synth_2_8a");
    if (!a) { CHECK(false); } else {
      replay::WorkoutReplay r(a->cfg);
      size_t next = 0;
      r.runUntil(a->inputs, next, 115 * SEC - 1);
      CHECK(r.workout.state() == WorkoutState::Summary);
      CHECK(r.workout.summary() != nullptr);
      r.runUntil(a->inputs, next, 115 * SEC);
      CHECK(r.workout.state() == WorkoutState::Running);
      CHECK(r.workout.summary() == nullptr);
      const WorkoutSnapshot s = r.workout.snapshot(115 * SEC);
      CHECK_EQ(s.workoutId, 2u);
      CHECK_EQ(s.wkPulses, 0u);
    }
  }
  // On overflow the NEWEST event is dropped (the queue keeps the first 8).
  {
    Workout w;
    w.onPulse(0, 0);
    for (int i = 1; i <= 8; ++i) {
      w.tick(static_cast<int64_t>(i) * 10 * SEC);
      w.onPulse(static_cast<int64_t>(i) * 10 * SEC + 1, 0);
    }
    CHECK(w.eventOverflow());
    std::vector<WorkoutEvent> q;
    WorkoutEvent e;
    while (w.pollEvent(e)) q.push_back(e);
    CHECK_EQ(q.size(), static_cast<size_t>(8));
    if (q.size() == 8) {
      CHECK(q.front().type == WorkoutEventType::BoutStarted && q.front().t == 0);
      CHECK(q.back().type == WorkoutEventType::WarmupFailed && q.back().t == 32500001);
    }
  }
}

// ---------------------------------------------------------------- goldens
bool tokensInts(const std::string& s, std::vector<int64_t>& out) {
  out.clear();
  std::string cur;
  for (size_t i = 0; i <= s.size(); ++i) {
    const char ch = i < s.size() ? s[i] : ' ';
    if (ch >= '0' && ch <= '9') cur.push_back(ch);
    else if (!cur.empty()) {
      int64_t v = 0;
      if (!replay::parseI64(cur, v)) return false;
      out.push_back(v);
      cur.clear();
    }
  }
  return true;
}

void testGoldens(const std::vector<Case>& cs) {
  uint32_t present = 0, missing = 0;
  for (const Case& c : cs) {
    const std::string file = "workout_" + c.name + ".txt";
    replay::Golden g;
    std::string err;
    if (!replay::loadGolden(gRoot + "/" + kGoldenDir + file, g, err)) {
      ++missing;
      std::fprintf(stderr, "  golden: %s\n", err.c_str());
      CHECK(false);
      continue;
    }
    ++present;
    const WorkoutTrace tr = replay::replayWorkout(c.inputs, c.cfg);
    const std::string params = "STOP_US=" + std::to_string(c.cfg.stopUs) + " WARMUP_US=" +
                               std::to_string(c.cfg.warmupUs) + " AUTO_END_US=" + std::to_string(c.cfg.autoEndUs) +
                               " SUMMARY_US=" + std::to_string(c.cfg.summaryUs) +
                               " SPAN_US=" + std::to_string(c.cfg.spanUs);
    CHECK_EQ(g.get("case"), c.name);
    CHECK_EQ(g.get("source"), c.source);
    CHECK_EQ(g.get("params"), params);
    std::vector<int64_t> nin;
    CHECK(tokensInts(g.get("inputs"), nin));
    CHECK(nin.size() == 2 && nin[0] == tr.inputsP && nin[1] == tr.inputsI);
    std::vector<std::string> w, z;
    for (const WorkoutEvent& e : tr.events) {
      w.push_back(replay::formatW(e));
      if (replay::isZEvent(e)) z.push_back(replay::formatZ(e));
    }
    const std::vector<std::string> gw = g.all("w"), gz = g.all("z");
    const bool wOk = CHECK(gw == w);
    const bool zOk = CHECK(gz == z);
    if (!wOk || !zOk) {
      std::fprintf(stderr, "  golden %s: W %zu (golden %zu), Z %zu (golden %zu)\n", file.c_str(), w.size(), gw.size(),
                   z.size(), gz.size());
      for (size_t i = 0; i < std::max(w.size(), gw.size()); ++i) {
        const std::string a = i < w.size() ? w[i] : "(none)";
        const std::string b = i < gw.size() ? gw[i] : "(none)";
        if (a != b) { std::fprintf(stderr, "    first W diff #%zu: c++ '%s' vs golden '%s'\n", i, a.c_str(), b.c_str()); break; }
      }
      for (size_t i = 0; i < std::max(z.size(), gz.size()); ++i) {
        const std::string a = i < z.size() ? z[i] : "(none)";
        const std::string b = i < gz.size() ? gz[i] : "(none)";
        if (a != b) { std::fprintf(stderr, "    first Z diff #%zu: c++ '%s' vs golden '%s'\n", i, a.c_str(), b.c_str()); break; }
      }
    }
    CHECK_EQ(g.get("workouts"), std::to_string(tr.workouts));
    CHECK_EQ(g.get("warmup_fails"), std::to_string(tr.warmupFails));
    CHECK_EQ(g.get("pauses"), std::to_string(tr.pauses));
  }
  std::printf("goldens: %u workout_*.txt compared, %u missing\n", present, missing);
}

}  // namespace

int main(int argc, char** argv) {
  gRoot = replay::repoRootFromArgs(argc, argv);
  for (int i = 1; i < argc; ++i)
    if (std::strcmp(argv[i], "--dump") == 0) gDump = true;
  const std::vector<Case> cs = allCases();
  CHECK_EQ(cs.size(), static_cast<size_t>(13));
  testHelpers();
  testReplayInputs();
  test21(cs);
  test22(cs);
  test23(cs);
  test24(cs);
  test25(cs);
  test26();
  test27();
  test28(cs);
  test29(cs);
  test210();
  test211(cs);
  test212(cs);
  test213(cs);
  test214(cs);
  test215(cs);
  testHoldBeganNoButton();
  testInputClamp();
  testMutantKillers(cs);
  testGoldens(cs);
  return check::summary("test_workout");
}
