// test_persist.cpp - host tests for src/core/persist_policy.*:
// tests 8.1-8.6 (validation, settle, double toggle, storm, rate limit,
// stationary deferral), plus the hold deferral, failed writes, the "never at
// boot" rule, and a seeded property test with a simulated 1 ms loop.
//
// Usage: test_persist [--root <repo root>]   (the root is unused; accepted for run_host_tests.sh)
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "persist_policy.h"

using ot::LoadStatus;
using ot::PersistAction;
using ot::PersistPolicy;
using ot::Units;

namespace {

constexpr int64_t SEC = 1000000;
constexpr int64_t MS = 1000;
constexpr int64_t STOP = ot::STOP_US;

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

Units flip(Units u) { return u == Units::Mi ? Units::Km : Units::Mi; }

// The app's loop, host side: every `step` us it asks decide() and, on Commit,
// "writes" (ok unless failWrites) and calls written(). Inputs: toggle times,
// pulse times, hold windows [a, b).
struct Loop {
  PersistPolicy p;
  Units current = Units::Mi;
  std::vector<int64_t> toggles, pulses;
  std::vector<std::pair<int64_t, int64_t>> holds;
  bool failWrites = false;
  struct Write { int64_t t; Units v; };
  std::vector<Write> writes;
  std::vector<int64_t> defers;

  explicit Loop(Units boot = Units::Mi) : current(boot) { p.begin(boot); }
  void run(int64_t from, int64_t to, int64_t step) {
    size_t ti = 0, pi = 0;
    bool hasPulse = false;
    int64_t lastPulse = 0;
    for (int64_t t = from; t <= to; t += step) {
      while (ti < toggles.size() && toggles[ti] <= t) {
        current = flip(current);
        p.onToggle(toggles[ti]);
        ++ti;
      }
      while (pi < pulses.size() && pulses[pi] <= t) {
        hasPulse = true;
        lastPulse = pulses[pi];
        ++pi;
      }
      bool hold = false;
      for (const auto& h : holds) hold = hold || (t >= h.first && t < h.second);
      const PersistAction a = p.decide(t, current, hasPulse, lastPulse, hold);
      if (a == PersistAction::Defer) defers.push_back(t);
      if (a == PersistAction::Commit) {
        writes.push_back({t, current});
        p.written(t, current, !failWrites);
      }
    }
  }
};

std::vector<int64_t> train(int64_t from, int64_t to, int64_t step) {
  std::vector<int64_t> v;
  for (int64_t t = from; t <= to; t += step) v.push_back(t);
  return v;
}

// 8.1 Validate: missing -> imperial; 0/1 -> as stored; 2/255 -> imperial;
// schema 0/2 -> imperial.
void test81() {
  struct Row { bool hs; uint8_t s; bool hu; uint8_t u; Units want; LoadStatus st; };
  const Row rows[] = {
      {false, 0, false, 0, Units::Mi, LoadStatus::Missing},   // empty NVS / missing namespace
      {true, 1, false, 0, Units::Mi, LoadStatus::Missing},    // units key missing
      {false, 0, true, 1, Units::Mi, LoadStatus::Missing},    // schema key missing (even with units=1)
      {true, 1, true, 0, Units::Mi, LoadStatus::Ok},
      {true, 1, true, 1, Units::Km, LoadStatus::Ok},
      {true, 1, true, 2, Units::Mi, LoadStatus::BadValue},
      {true, 1, true, 7, Units::Mi, LoadStatus::BadValue},    // the OT_DEV nX value
      {true, 1, true, 255, Units::Mi, LoadStatus::BadValue},
      {true, 0, true, 1, Units::Mi, LoadStatus::BadSchema},
      {true, 2, true, 1, Units::Mi, LoadStatus::BadSchema},
      {true, 255, true, 0, Units::Mi, LoadStatus::BadSchema},
  };
  for (const Row& r : rows) {
    const ot::UnitsLoad l = ot::validateUnits(r.hs, r.s, r.hu, r.u);
    CHECK(l.units == r.want);
    CHECK(l.status == r.st);
  }
  // The default is a parameter (OT_UNITS_DEFAULT=OT_UNITS_KM builds).
  CHECK(ot::validateUnits(false, 0, false, 0, Units::Km).units == Units::Km);
  CHECK(ot::validateUnits(true, 1, true, 0, Units::Km).units == Units::Mi);
  CHECK(ot::UNITS_DEFAULT == Units::Mi);  // imperial by default
  CHECK_EQ(ot::unitsByte(Units::Mi), static_cast<uint8_t>(0));
  CHECK_EQ(ot::unitsByte(Units::Km), static_cast<uint8_t>(1));
  CHECK_EQ(std::string(ot::loadStatusName(LoadStatus::BadSchema)), std::string("bad_schema"));
}

// Never at boot: with no toggle nothing is ever written, whatever the NVS held.
void testNoBootWrite() {
  for (int b = 0; b < 2; ++b) {
    Loop l(b == 0 ? Units::Mi : Units::Km);
    l.run(0, 60 * SEC, MS);
    CHECK(l.writes.empty());
    CHECK(l.defers.empty());
  }
}

// 8.2 Settle: toggle at t=0 -> commit at 3.000 s, not 2.999.
void test82() {
  PersistPolicy p;
  p.begin(Units::Mi);
  p.onToggle(0);
  CHECK(p.decide(0, Units::Km, false, 0, false) == PersistAction::None);
  CHECK(p.decide(3 * SEC - 1, Units::Km, false, 0, false) == PersistAction::None);
  CHECK(p.decide(3 * SEC, Units::Km, false, 0, false) == PersistAction::Commit);
  p.written(3 * SEC, Units::Km, true);
  CHECK(p.stored() == Units::Km);
  CHECK(p.decide(3 * SEC + 1, Units::Km, false, 0, false) == PersistAction::None);
  Loop l;
  l.toggles = {0};
  l.run(0, 30 * SEC, MS);  // 3.000 s is on the 1 ms grid
  if (CHECK_EQ(l.writes.size(), static_cast<size_t>(1))) {
    CHECK_EQ(l.writes[0].t, 3 * SEC);
    CHECK(l.writes[0].v == Units::Km);
  }
}

// 8.3 Double toggle within 3 s: no write.
void test83() {
  for (int64_t gap : {1 * MS, 1 * SEC, 3 * SEC - 1}) {
    Loop l;
    l.toggles = {0, gap};
    l.run(0, 60 * SEC, MS / 2);
    CHECK(l.writes.empty());
  }
  // A toggle back AFTER the commit window is a second, separate change.
  Loop l;
  l.toggles = {0, 3 * SEC + MS};
  l.run(0, 60 * SEC, MS);
  CHECK_EQ(l.writes.size(), static_cast<size_t>(2));  // Km at 3 s, Mi at 13 s
  if (l.writes.size() == 2) {
    CHECK_EQ(l.writes[0].t, 3 * SEC);
    CHECK(l.writes[0].v == Units::Km);
    CHECK_EQ(l.writes[1].t, 13 * SEC);
    CHECK(l.writes[1].v == Units::Mi);
  }
}

// 8.4 Storm: 50 toggles over 60 s, ending on a net change -> <= 7 writes (one
// per 10 s window), last value correct.
void test84() {
  // (a) Even spacing 1.2 s: never settles during the storm; ends on a net
  // change (50 toggles from Mi with an extra pending toggle before the storm).
  {
    Loop l;
    l.toggles = train(0, 58800 * MS, 1200 * MS);  // 50 toggles
    l.toggles.insert(l.toggles.begin(), -500 * MS);  // pending change from just before
    CHECK_EQ(l.toggles.size(), static_cast<size_t>(51));
    l.run(-500 * MS, 120 * SEC, MS);
    CHECK(l.writes.size() <= 7u);
    if (CHECK(!l.writes.empty())) CHECK(l.writes.back().v == l.current);
    CHECK(l.p.stored() == l.current);
  }
  // (b) Worst case for the rate limit: gaps just over the settle window so a
  // commit is due after every toggle.
  {
    Loop l;
    // Deterministic storm: toggle times with gaps 0.2 s, but every 5th gap 3.1 s.
    int64_t t = 0;
    for (int k = 0; k < 50; ++k) {
      l.toggles.push_back(t);
      t += (k % 5 == 4) ? 3100 * MS : 200 * MS;
    }
    CHECK(l.toggles.back() <= 60 * SEC);
    l.run(0, 120 * SEC, MS);
    uint32_t inStorm = 0;
    for (const auto& w : l.writes) inStorm += w.t <= 60 * SEC + 3 * SEC ? 1u : 0u;
    CHECK(inStorm <= 7u);
    for (size_t i = 1; i < l.writes.size(); ++i) CHECK(l.writes[i].t - l.writes[i - 1].t >= ot::NVS_MIN_INTERVAL_US);
    for (size_t i = 1; i < l.writes.size(); ++i) CHECK(l.writes[i].v != l.writes[i - 1].v);  // only net changes
    CHECK(l.p.stored() == l.current);
    std::printf("  8.4 storm: %zu toggles, %zu writes (%u by 63 s)\n", l.toggles.size(), l.writes.size(), inStorm);
  }
  // (c) Seeded random storms: 50 toggles within 60 s.
  Rng rng(0x570A4);
  uint32_t maxWrites = 0;
  for (int run = 0; run < 200; ++run) {
    Loop l;
    std::vector<int64_t> ts;
    for (int k = 0; k < 50; ++k) ts.push_back(rng.range(0, 60 * SEC));
    std::sort(ts.begin(), ts.end());
    l.toggles = ts;
    l.run(0, 90 * SEC, 5 * MS);
    uint32_t n = 0;
    for (const auto& w : l.writes) n += w.t <= 63 * SEC ? 1u : 0u;
    if (n > maxWrites) maxWrites = n;
    CHECK(n <= 7u);
    for (size_t i = 1; i < l.writes.size(); ++i) CHECK(l.writes[i].t - l.writes[i - 1].t >= ot::NVS_MIN_INTERVAL_US);
    CHECK(l.p.stored() == l.current);
  }
  std::printf("  8.4 random storms: 200 runs, max %u writes by 63 s\n", maxWrites);
}

// 8.5 Rate limit: two net changes 4 s apart -> the second commit >= 10 s after the first.
void test85() {
  Loop l;
  l.toggles = {0, 4 * SEC};
  l.run(0, 40 * SEC, MS);
  if (CHECK_EQ(l.writes.size(), static_cast<size_t>(2))) {
    CHECK_EQ(l.writes[0].t, 3 * SEC);
    CHECK_EQ(l.writes[1].t, 13 * SEC);  // settle would allow 7 s; the rate limit says 3 + 10
    CHECK(l.writes[1].v == Units::Mi);
  }
  PersistPolicy p;
  p.begin(Units::Mi);
  p.onToggle(0);
  CHECK(p.decide(3 * SEC, Units::Km, false, 0, false) == PersistAction::Commit);
  p.written(3 * SEC, Units::Km, true);
  p.onToggle(4 * SEC);
  CHECK(p.decide(13 * SEC - 1, Units::Mi, false, 0, false) == PersistAction::None);
  CHECK(p.decide(13 * SEC, Units::Mi, false, 0, false) == PersistAction::Commit);
}

// 8.6 Stationary deferral: toggle at t=0 with pulses every 0.5 s until 20.0 s
// -> no commit before 22.500001 s (20.0 + STOP), then exactly one commit; a
// toggle while stationary commits at 3.000 s as in 8.2.
void test86() {
  Loop l;
  l.toggles = {0};
  l.pulses = train(-10 * SEC, 20 * SEC, SEC / 2);
  l.run(0, 40 * SEC, MS);
  if (CHECK_EQ(l.writes.size(), static_cast<size_t>(1))) CHECK(l.writes[0].t >= 22500001);
  if (CHECK_EQ(l.defers.size(), static_cast<size_t>(1))) CHECK_EQ(l.defers[0], 3 * SEC);  // logged once
  // Exact edge: a pulse AT last + STOP is still movement.
  PersistPolicy p;
  p.begin(Units::Mi);
  p.onToggle(0);
  CHECK(p.decide(3 * SEC, Units::Km, true, 2 * SEC, false) == PersistAction::Defer);
  CHECK(p.decide(20 * SEC + STOP, Units::Km, true, 20 * SEC, false) == PersistAction::None);  // = STOP: moving
  CHECK(p.decide(20 * SEC + STOP + 1, Units::Km, true, 20 * SEC, false) == PersistAction::Commit);
  // Toggle while stationary (last pulse long ago).
  Loop s;
  s.toggles = {30 * SEC};
  s.pulses = train(0, 20 * SEC, SEC / 2);
  s.run(0, 60 * SEC, MS);
  if (CHECK_EQ(s.writes.size(), static_cast<size_t>(1))) CHECK_EQ(s.writes[0].t, 33 * SEC);
  CHECK(s.defers.empty());
  // A pulse train that resumes after the deferral was reported: still one defer line.
  Loop m;
  m.toggles = {0};
  std::vector<int64_t> pp = train(0, 10 * SEC, SEC / 2);
  const std::vector<int64_t> more = train(14 * SEC, 30 * SEC, SEC / 2);  // stops 4 s at 10..14 (> STOP)
  pp.insert(pp.end(), more.begin(), more.end());
  m.pulses = pp;
  m.run(0, 60 * SEC, MS);
  if (CHECK_EQ(m.writes.size(), static_cast<size_t>(1))) CHECK_EQ(m.writes[0].t, 10 * SEC + STOP + MS);  // the gap
  CHECK_EQ(m.defers.size(), static_cast<size_t>(1));
}

// No commit while a touch hold is in progress (the gesture's critical window).
void testHold() {
  Loop l;
  l.toggles = {0};
  l.holds = {{2500 * MS, 5 * SEC}};
  l.run(0, 30 * SEC, MS);
  if (CHECK_EQ(l.writes.size(), static_cast<size_t>(1))) CHECK_EQ(l.writes[0].t, 5 * SEC);
  if (CHECK_EQ(l.defers.size(), static_cast<size_t>(1))) CHECK_EQ(l.defers[0], 3 * SEC);
}

// A toggle restarts the settle window and re-arms the defer report; toggling
// back to the stored value cancels the pending write.
void testToggleBack() {
  Loop l;
  l.toggles = {0, 5 * SEC};  // Km pending (deferred by movement), then back to Mi
  l.pulses = train(0, 30 * SEC, SEC / 2);
  l.run(0, 60 * SEC, MS);
  CHECK(l.writes.empty());
  CHECK_EQ(l.defers.size(), static_cast<size_t>(1));
  Loop r;
  r.toggles = {0, 5 * SEC, 6 * SEC};  // Km, Mi, Km: pending again, defer reported again
  r.pulses = train(0, 30 * SEC, SEC / 2);
  r.run(0, 60 * SEC, MS);
  CHECK_EQ(r.defers.size(), static_cast<size_t>(2));
  if (CHECK_EQ(r.writes.size(), static_cast<size_t>(1))) {
    CHECK_EQ(r.writes[0].t, 30 * SEC + STOP + MS);
    CHECK(r.writes[0].v == Units::Km);
  }
}

// A failing write: stored unchanged, retried at most once per 10 s.
void testFailedWrite() {
  Loop l;
  l.failWrites = true;
  l.toggles = {0};
  l.run(0, 45 * SEC, MS);
  if (CHECK_EQ(l.writes.size(), static_cast<size_t>(5))) {  // 3, 13, 23, 33, 43 s
    CHECK_EQ(l.writes[0].t, 3 * SEC);
    CHECK_EQ(l.writes[4].t, 43 * SEC);
  }
  CHECK(l.p.stored() == Units::Mi);
}

// Property: random toggles / pulse trains / holds, 1 ms loop. Invariants:
// a commit only on a net change, >= 3 s after the last toggle, >= 10 s after
// the previous write, never while moving or holding; <= 1 defer between two
// toggles; after a quiet tail the stored value equals the current one.
void testProperty() {
  Rng rng(0xFE5157);
  uint64_t commits = 0, defersN = 0;
  uint32_t bad = 0;
  for (int run = 0; run < 150; ++run) {
    PersistPolicy p;
    Units cur = Units::Mi;
    p.begin(cur);
    std::vector<int64_t> toggles, pulses;
    std::vector<std::pair<int64_t, int64_t>> holds;
    int64_t t = 0;
    for (int k = 0; k < 30; ++k) {
      t += rng.range(10, 8000) * MS;
      toggles.push_back(t);
    }
    for (int64_t x = 0; x < t + 5 * SEC;) {
      x += rng.range(1, 12000) * MS;
      const int64_t n = rng.range(0, 30), dt = rng.range(300, 1500) * MS;
      for (int64_t k = 0; k < n; ++k, x += dt) pulses.push_back(x);
    }
    for (int k = 0; k < 6; ++k) {
      const int64_t a = rng.range(0, t);
      holds.push_back({a, a + rng.range(100, 2500) * MS});
    }
    const int64_t end = t + 60 * SEC + 60 * SEC;
    size_t ti = 0, pi = 0;
    bool hasPulse = false;
    int64_t lastPulse = 0, lastToggle = INT64_MIN, lastWrite = INT64_MIN;
    uint32_t defersSinceToggle = 0;
    for (int64_t now = 0; now <= end; now += MS) {
      while (ti < toggles.size() && toggles[ti] <= now) {
        cur = flip(cur);
        p.onToggle(toggles[ti]);
        lastToggle = toggles[ti];
        defersSinceToggle = 0;
        ++ti;
      }
      while (pi < pulses.size() && pulses[pi] <= now) {
        hasPulse = true;
        lastPulse = pulses[pi];
        ++pi;
      }
      bool hold = false;
      for (const auto& h : holds) hold = hold || (now >= h.first && now < h.second);
      const Units before = p.stored();
      const PersistAction a = p.decide(now, cur, hasPulse, lastPulse, hold);
      if (a == PersistAction::Defer) {
        ++defersN;
        if (++defersSinceToggle > 1) ++bad;
      }
      if (a == PersistAction::Commit) {
        ++commits;
        const bool moving = hasPulse && now - lastPulse <= STOP;
        if (cur == before || moving || hold || now - lastToggle < ot::NVS_SETTLE_US ||
            (lastWrite != INT64_MIN && now - lastWrite < ot::NVS_MIN_INTERVAL_US))
          ++bad;
        lastWrite = now;
        p.written(now, cur, true);
      }
    }
    if (p.stored() != cur) ++bad;  // the quiet tail (60 s, no pulses after t + ~17 s) lets it settle
  }
  std::printf("  property: 150 runs, %llu commits, %llu defers, %u violations\n",
              static_cast<unsigned long long>(commits), static_cast<unsigned long long>(defersN), bad);
  CHECK_EQ(bad, 0u);
  CHECK(commits > 150u && defersN > 100u);
}

}  // namespace

int main(int, char**) {
  test81();
  testNoBootWrite();
  test82();
  test83();
  test84();
  test85();
  test86();
  testHold();
  testToggleBack();
  testFailedWrite();
  testProperty();
  return check::summary("test_persist");
}
