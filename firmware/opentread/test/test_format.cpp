// test_format.cpp - host tests for src/core/format.*: tests
// 3.1-3.8, plus the comparison against the oracle's goldens
// test/golden/format_pace_sweep.txt and test/golden/format_misc.txt (written
// by test/oracle/oracle.py; the C++ only reads them).
//
// Literal expected values are hardcoded. Golden contract:
//   format_pace_sweep.txt  case=format_pace_sweep, range=361..12000, then
//                          p=<mmps> <km> <mi> for every integer mm/s 361..12000
//                          (plain fmtPace, visible, no hysteresis state)
//   format_misc.txt        speed=<mmps> <km/h tenths> <mph tenths>, mmps 0..12000
//                          dist=<pulses> <km> <mi>, pulses 0..20000
//
// Usage: test_format --root <repo root> [--verbose]
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "format.h"
#include "metrics.h"
#include "replay.h"
#include "workout.h"

using ot::PaceDisplayFilter;
using ot::SpeedDisplayFilter;
using ot::Units;
using ot::WorkoutState;

namespace {

std::string gRoot = ".";
bool gVerbose = false;
const char* const kGoldenDir = "firmware/opentread/test/golden/";
const char* const kS2 = "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv";
constexpr int64_t SEC = 1000000;
constexpr int64_t MS = 1000;

std::string pace(int32_t mmps, Units u, bool visible = true) {
  char b[ot::PACE_STR_SIZE];
  ot::fmtPace(b, sizeof(b), mmps, u, visible);
  return b;
}
std::string speed(int32_t mmps, Units u) {
  char b[ot::SPEED_STR_SIZE];
  ot::fmtSpeed(b, sizeof(b), mmps, u);
  return b;
}
std::string dist(uint32_t pulses, Units u) {
  char b[ot::DIST_STR_SIZE];
  ot::fmtDistance(b, sizeof(b), pulses, u);
  return b;
}
std::string dur(int64_t us) {
  char b[ot::DUR_STR_SIZE];
  ot::fmtDuration(b, sizeof(b), us);
  return b;
}
std::string shown(const PaceDisplayFilter& f) {
  char b[ot::PACE_STR_SIZE];
  f.format(b, sizeof(b));
  return b;
}
std::string shown(const SpeedDisplayFilter& f) {
  char b[ot::SPEED_STR_SIZE];
  f.format(b, sizeof(b));
  return b;
}

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

// "m:ss" with ss < 60 and m <= 99, or "--:--".
bool paceWellFormed(const std::string& s) {
  if (s == "--:--") return true;
  const size_t c = s.find(':');
  if (c == std::string::npos || c == 0 || c > 2 || s.size() != c + 3) return false;
  for (size_t i = 0; i < s.size(); ++i)
    if (i != c && (s[i] < '0' || s[i] > '9')) return false;
  if (c == 2 && s[0] == '0') return false;  // no leading zero on minutes
  return (s[c + 1] - '0') * 10 + (s[c + 2] - '0') < 60;
}

std::vector<std::string> splitSpaces(const std::string& s) {
  std::vector<std::string> v;
  size_t i = 0;
  while (i <= s.size()) {
    const size_t j = s.find(' ', i);
    v.push_back(s.substr(i, j == std::string::npos ? std::string::npos : j - i));
    if (j == std::string::npos) break;
    i = j + 1;
  }
  return v;
}

bool loadGolden(const char* file, replay::Golden& g) {
  std::string err;
  if (!replay::loadGolden(gRoot + "/" + kGoldenDir + file, g, err)) {
    std::fprintf(stderr, "  %s\n", err.c_str());
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 3.1 Pace rounding (+ the sweep against the oracle)
// ---------------------------------------------------------------------------
void test31() {
  CHECK_EQ(pace(2084, Units::Km), std::string("8:00"));  // 479.846 s
  CHECK_EQ(pace(2086, Units::Km), std::string("7:59"));  // 479.386 s
  CHECK_EQ(pace(3353, Units::Mi), std::string("8:00"));  // 479.971 s
  CHECK_EQ(pace(3358, Units::Mi), std::string("7:59"));  // 479.257 s
  CHECK_EQ(ot::paceSeconds(2084, Units::Km), 480u);
  CHECK_EQ(ot::paceSeconds(2086, Units::Km), 479u);
  // mi uses 1609.344 m exactly: 1609.000 would give 4445 s here.
  CHECK_EQ(pace(362, Units::Mi), std::string("74:06"));
  // A whole-minute boundary: rounded before the split, so never "x:60".
  CHECK_EQ(pace(1000000 / 299, Units::Km), std::string("4:59"));  // 3344 mm/s -> 299.04 s
  CHECK_EQ(pace(2778, Units::Km), std::string("6:00"));           // 359.97 s rounds up to 6:00, not 5:60

  // Sweep every integer mm/s 361..12000, both units: well-formed (never x:60).
  uint32_t bad = 0;
  for (int32_t v = 361; v <= 12000; ++v)
    for (Units u : {Units::Km, Units::Mi})
      if (!paceWellFormed(pace(v, u))) ++bad;
  CHECK_EQ(bad, 0u);

  // ... and equal to the oracle's strings.
  replay::Golden g;
  if (!CHECK(loadGolden("format_pace_sweep.txt", g))) return;
  CHECK_EQ(g.get("case"), std::string("format_pace_sweep"));
  CHECK_EQ(g.get("range"), std::string("361..12000"));
  const std::vector<std::string> rows = g.all("p");
  CHECK_EQ(rows.size(), static_cast<size_t>(12000 - 361 + 1));
  uint32_t mismatch = 0, malformed = 0;
  int32_t expectV = 361;
  for (const std::string& r : rows) {
    const std::vector<std::string> f = splitSpaces(r);
    uint32_t v = 0;
    if (f.size() != 3 || !replay::parseU32(f[0], v) || static_cast<int32_t>(v) != expectV) {
      if (++malformed <= 5) std::fprintf(stderr, "  bad sweep row (expected mm/s %d): p=%s\n", expectV, r.c_str());
      ++expectV;
      continue;
    }
    const std::string km = pace(expectV, Units::Km), mi = pace(expectV, Units::Mi);
    if (km != f[1] || mi != f[2]) {
      if (++mismatch <= 10)
        std::fprintf(stderr, "  pace %d mm/s: C++ km=%s mi=%s, oracle km=%s mi=%s\n", expectV, km.c_str(), mi.c_str(),
                     f[1].c_str(), f[2].c_str());
    }
    ++expectV;
  }
  CHECK_EQ(malformed, 0u);
  CHECK_EQ(mismatch, 0u);
}

// ---------------------------------------------------------------------------
// 3.2 Pace visibility hysteresis
// ---------------------------------------------------------------------------
void test32() {
  int64_t t = 10 * SEC;
  auto step = [&](PaceDisplayFilter& f, int32_t v, Units u = Units::Km, bool st = true) {
    t += SEC;  // 1 Hz: every sample is past the 1 s refresh window
    f.update(t, v, u, st);
    return f.shown();
  };
  {  // rising
    PaceDisplayFilter f;
    CHECK(!step(f, 0));
    CHECK(!step(f, 443));
    CHECK(step(f, 444));
    CHECK_EQ(shown(f), pace(444, Units::Km));
  }
  {  // falling
    PaceDisplayFilter f;
    CHECK(step(f, 1000));
    CHECK(step(f, 361));
    CHECK(step(f, 361));
    CHECK(!step(f, 360));
    CHECK_EQ(shown(f), std::string("--:--"));
  }
  {  // 361..443 keeps the previous state, both ways
    PaceDisplayFilter hidden, visible;
    CHECK(!step(hidden, 100));
    CHECK(step(visible, 2000));
    for (int32_t v : {443, 400, 361, 442, 362}) {
      CHECK(!step(hidden, v));
      CHECK(step(visible, v));
    }
    CHECK(!hidden.hysteresisVisible());
    CHECK(visible.hysteresisVisible());
    // The same thresholds in imperial mode (km/h-based in both modes).
    PaceDisplayFilter mi;
    CHECK(!step(mi, 443, Units::Mi));
    CHECK(step(mi, 444, Units::Mi));
    CHECK(step(mi, 361, Units::Mi));
    CHECK_EQ(shown(mi), std::string("74:18"));  // 1609344/361 = 4458 s: never --:-- from PACE_MAX_S
    CHECK(!step(mi, 360, Units::Mi));
  }
  {  // WARMUP shows a pace; PAUSED / IDLE / SUMMARY never do
    CHECK(ot::paceShownInState(WorkoutState::Warmup));
    CHECK(ot::paceShownInState(WorkoutState::Running));
    CHECK(!ot::paceShownInState(WorkoutState::Paused));
    CHECK(!ot::paceShownInState(WorkoutState::Idle));
    CHECK(!ot::paceShownInState(WorkoutState::Summary));
    PaceDisplayFilter f;
    CHECK(step(f, 1200, Units::Km, ot::paceShownInState(WorkoutState::Warmup)));
    CHECK_EQ(shown(f), std::string("13:53"));  // 833 s
    for (WorkoutState s : {WorkoutState::Paused, WorkoutState::Idle, WorkoutState::Summary}) {
      PaceDisplayFilter g;
      CHECK(!step(g, 3000, Units::Km, ot::paceShownInState(s)));
      CHECK_EQ(shown(g), std::string("--:--"));
      CHECK(g.hysteresisVisible());  // the speed is high; the state alone hides it
    }
    // A state without pace hides at once, even inside the refresh window.
    PaceDisplayFilter h;
    h.update(t, 3000, Units::Km, true);
    CHECK(h.shown());
    CHECK(h.update(t + 1, 3000, Units::Km, false));
    CHECK_EQ(shown(h), std::string("--:--"));
  }
  // Imperial-mode sweep 430..460 mm/s with +-2% seeded jitter, 10 min of 1 Hz
  // samples: at most one visibility change, and a shown pace is never --:--
  // because of PACE_MAX_S (shown == hysteresis state at every 1 Hz sample).
  for (Units u : {Units::Mi, Units::Km}) {
    for (int shape = 0; shape < 3; ++shape) {
      for (uint64_t seed = 1; seed <= 20; ++seed) {
        Rng rng(seed * 7919u + static_cast<uint64_t>(shape));
        PaceDisplayFilter f;
        uint32_t changes = 0, mism = 0;
        bool prev = false;
        for (int i = 0; i < 600; ++i) {
          int32_t base = 0;
          if (shape == 0) base = 430 + 30 * i / 599;                                 // ramp up
          else if (shape == 1) base = 460 - 30 * i / 599;                            // ramp down
          else base = 430 + static_cast<int32_t>(rng.range(0, 30));                  // random in band
          const int64_t jit = rng.range(-20000, 20000);                              // ppm
          const int32_t v = static_cast<int32_t>(base + base * jit / 1000000);
          f.update(static_cast<int64_t>(i) * SEC, v, u, true);
          if (i > 0 && f.shown() != prev) ++changes;
          if (f.shown() != f.hysteresisVisible()) ++mism;
          prev = f.shown();
        }
        CHECK(changes <= 1u);
        CHECK_EQ(mism, 0u);
      }
    }
  }
  CHECK_EQ(pace(166, Units::Km), std::string("--:--"));  // 6024 s > PACE_MAX_S
  CHECK_EQ(pace(167, Units::Km), std::string("99:48"));  // 5988 s
  CHECK_EQ(pace(0, Units::Km), std::string("--:--"));
  CHECK_EQ(pace(-5, Units::Mi), std::string("--:--"));
  CHECK_EQ(pace(3000, Units::Km, false), std::string("--:--"));
  char b[ot::PACE_STR_SIZE];
  ot::fmtPaceSeconds(b, sizeof(b), ot::PACE_MAX_S);
  CHECK_EQ(std::string(b), std::string("99:59"));
  ot::fmtPaceSeconds(b, sizeof(b), ot::PACE_MAX_S + 1u);
  CHECK_EQ(std::string(b), std::string("--:--"));
}

// ---------------------------------------------------------------------------
// 3.3 Pace dead-band and refresh rate
// ---------------------------------------------------------------------------
void test33() {
  CHECK_EQ(ot::paceExactMs(2083, Units::Km), 480077u);
  CHECK_EQ(ot::paceExactMs(2077, Units::Km), 481464u);
  CHECK_EQ(ot::paceExactMs(2076, Units::Km), 481696u);
  CHECK_EQ(ot::paceExactMs(3353, Units::Mi), 479971u);
  {
    PaceDisplayFilter f;
    const int64_t t0 = 100 * SEC;
    CHECK(f.update(t0, 2083, Units::Km, true));
    CHECK_EQ(shown(f), std::string("8:00"));
    CHECK(!f.update(t0 + SEC, 2077, Units::Km, true));  // delta 1464 ms < 1500
    CHECK_EQ(shown(f), std::string("8:00"));
    CHECK(f.update(t0 + 2 * SEC, 2076, Units::Km, true));  // delta 1696 ms
    CHECK_EQ(shown(f), std::string("8:02"));
  }
  {  // dead-band boundary rows: exactly 1500 ms changes ...
    CHECK_EQ(ot::paceExactMs(1996, Units::Km), 501002u);
    CHECK_EQ(ot::paceExactMs(2002, Units::Km), 499500u);
    PaceDisplayFilter f;
    CHECK(f.update(0, 1996, Units::Km, true));
    CHECK_EQ(shown(f), std::string("8:21"));
    CHECK(f.update(SEC, 2002, Units::Km, true));  // |499500 - 501000| = 1500 ms
    CHECK_EQ(shown(f), std::string("8:20"));
  }
  {  // ... and 1499 ms keeps
    CHECK_EQ(ot::paceExactMs(1554, Units::Km), 643501u);
    PaceDisplayFilter f;
    CHECK(f.update(0, 1550, Units::Km, true));
    CHECK_EQ(f.shownSeconds(), 645u);
    CHECK(!f.update(SEC, 1554, Units::Km, true));  // |643501 - 645000| = 1499 ms
    CHECK_EQ(f.shownSeconds(), 645u);
  }
  {  // 10 Hz rate window: in-band samples (Δ < dead-band) must not
     // restart the 1 s window. Every 7th sample is out of band (2060 mm/s =
     // 485.4 s/km vs shown 8:00): the change must land at the first out-of-band
     // sample >= 1 s, i.e. at 1.4 s.
    PaceDisplayFilter f;
    CHECK(f.update(0, 2083, Units::Km, true));
    int64_t when = -1;
    for (int i = 1; i <= 30 && when < 0; ++i) {
      const int32_t v = (i % 7) != 0 ? 2083 : 2060;
      if (f.update(i * 100 * MS, v, Units::Km, true)) when = i * 100 * MS;
    }
    CHECK_EQ(when, 1400 * MS);
    CHECK_EQ(shown(f), pace(2060, Units::Km));
  }
  {  // a change due inside the window waits for the window; the dead-band is two-sided
    PaceDisplayFilter f;
    const int64_t t0 = 100 * SEC;
    f.update(t0, 2083, Units::Km, true);
    CHECK(!f.update(t0 + 500 * MS, 2076, Units::Km, true));
    CHECK(!f.update(t0 + SEC - 1, 2076, Units::Km, true));
    CHECK(f.update(t0 + SEC, 2076, Units::Km, true));
    CHECK_EQ(shown(f), std::string("8:02"));
    CHECK(!f.update(t0 + 3 * SEC, 2081, Units::Km, true));  // 480538 ms: |-1462| < 1500
    CHECK(f.update(t0 + 4 * SEC, 2089, Units::Km, true));   // 478698 ms: |-3302|
    CHECK_EQ(shown(f), std::string("7:59"));
  }
  {  // a units change re-formats at once (no window, no dead-band)
    PaceDisplayFilter f;
    f.update(0, 2083, Units::Km, true);
    CHECK(f.update(1, 2083, Units::Mi, true));
    CHECK_EQ(shown(f), pace(2083, Units::Mi));
    CHECK(f.update(2, 2083, Units::Km, true));
    CHECK_EQ(shown(f), std::string("8:00"));
  }
  // At most one change per PACE_REFRESH_US: 10 Hz samples with wild speed
  // swings (and visibility flips) for 10 min; every change is >= 1 s after
  // the previous one (state-forced hides excepted: none here).
  for (Units u : {Units::Km, Units::Mi}) {
    Rng rng(42);
    PaceDisplayFilter f;
    int64_t last = -10 * SEC;
    uint32_t tooSoon = 0, n = 0;
    uint32_t notPlain = 0;
    for (int64_t t = 0; t < 600 * SEC; t += 100 * MS) {
      const int32_t v = static_cast<int32_t>(rng.range(0, 1) ? rng.range(300, 6000) : rng.range(1500, 1700));
      if (f.update(t, v, u, true)) {
        if (n > 0 && t - last < ot::PACE_REFRESH_US) ++tooSoon;
        if (f.shown() && shown(f) != pace(v, u)) ++notPlain;  // a new number is fmtPace of its sample
        last = t;
        ++n;
      }
    }
    CHECK_EQ(tooSoon, 0u);
    CHECK_EQ(notPlain, 0u);
    CHECK(n > 100u);  // it did change, often
  }
}

// ---------------------------------------------------------------------------
// 3.4 Speed
// ---------------------------------------------------------------------------
void test34() {
  CHECK_EQ(speed(1389, Units::Km), std::string("5.0"));
  CHECK_EQ(speed(2235, Units::Mi), std::string("5.0"));
  CHECK_EQ(speed(2034, Units::Mi), std::string("4.5"));  // 4.4993 mph; 1.609 km/mi would give 4.6
  CHECK_EQ(speed(0, Units::Km), std::string("0.0"));
  CHECK_EQ(speed(-3, Units::Mi), std::string("0.0"));
  CHECK_EQ(speed(6944, Units::Km), std::string("25.0"));
  // hundredths used by the hysteresis
  CHECK_EQ(ot::speedCenti(1409, Units::Km), 507u);
  CHECK_EQ(ot::speedCenti(1410, Units::Km), 508u);
  CHECK_EQ(ot::speedCenti(1369, Units::Km), 493u);
  CHECK_EQ(ot::speedCenti(1368, Units::Km), 492u);
  CHECK_EQ(ot::speedCenti(2268, Units::Mi), 507u);
  CHECK_EQ(ot::speedCenti(2269, Units::Mi), 508u);
  CHECK_EQ(ot::speedCenti(2202, Units::Mi), 493u);
  CHECK_EQ(ot::speedCenti(2201, Units::Mi), 492u);
  // mph hundredths use 1609.344 m exactly: 1609.0 gives 2685 / 1119
  CHECK_EQ(ot::speedCenti(12000, Units::Mi), 2684u);
  CHECK_EQ(ot::speedCenti(5000, Units::Mi), 1118u);

  struct Row { Units u; int32_t set; int32_t v; const char* expect; };
  const Row rows[] = {
      {Units::Km, 1389, 1409, "5.0"}, {Units::Km, 1389, 1410, "5.1"},
      {Units::Km, 1389, 1369, "5.0"}, {Units::Km, 1389, 1368, "4.9"},
      {Units::Mi, 2235, 2268, "5.0"}, {Units::Mi, 2235, 2269, "5.1"},
      {Units::Mi, 2235, 2202, "5.0"}, {Units::Mi, 2235, 2201, "4.9"},
  };
  for (const Row& r : rows) {
    SpeedDisplayFilter f;
    CHECK(f.update(0, r.set, r.u));
    CHECK_EQ(shown(f), std::string("5.0"));
    f.update(ot::SPEED_REFRESH_US, r.v, r.u);
    CHECK_EQ(shown(f), std::string(r.expect));
  }
  {  // stopping always reaches 0.0 (|0 - 10 s| >= 10 for any s >= 1)
    SpeedDisplayFilter f;
    f.update(0, 300, Units::Km);
    CHECK_EQ(shown(f), std::string("1.1"));
    f.update(SEC, 0, Units::Km);
    CHECK_EQ(shown(f), std::string("0.0"));
  }
  {  // units change: at once. 2235 mm/s = 8.046 km/h: one rounding -> 8.0 (a
     // double rounding (c+5)/10 with c = 805 would show 8.1)
    CHECK_EQ(ot::speedCenti(2235, Units::Km), 805u);
    SpeedDisplayFilter f;
    f.update(0, 2235, Units::Km);
    CHECK_EQ(shown(f), std::string("8.0"));
    CHECK_EQ(shown(f), speed(2235, Units::Km));
    CHECK(f.update(1, 2235, Units::Mi));
    CHECK_EQ(shown(f), std::string("5.0"));
  }
  // At most one change per SPEED_REFRESH_US (10 Hz samples, big swings).
  for (Units u : {Units::Km, Units::Mi}) {
    Rng rng(7);
    SpeedDisplayFilter f;
    int64_t last = 0;
    uint32_t tooSoon = 0, n = 0;
    f.update(0, 1000, u);
    uint32_t notPlain = 0;
    for (int64_t t = 100 * MS; t < 600 * SEC; t += 100 * MS) {
      const int32_t v = static_cast<int32_t>(rng.range(0, 7000));
      if (f.update(t, v, u)) {
        if (t - last < ot::SPEED_REFRESH_US) ++tooSoon;
        if (shown(f) != speed(v, u)) ++notPlain;  // a new value is always fmtSpeed of its sample
        last = t;
        ++n;
      }
    }
    CHECK_EQ(tooSoon, 0u);
    CHECK_EQ(notPlain, 0u);
    CHECK(n > 100u);
  }
  {  // 10 Hz rate window: in-band samples must not restart the
     // 0.5 s window. Every 7th sample out of band (1430 mm/s, c 515 vs shown
     // 5.0): the change lands at the first out-of-band sample >= 0.5 s = 0.7 s
     // (and in any case within 1.4 s).
    SpeedDisplayFilter f;
    CHECK(f.update(0, 1389, Units::Km));
    int64_t when = -1;
    for (int i = 1; i <= 30 && when < 0; ++i) {
      const int32_t v = (i % 7) != 0 ? 1389 : 1430;
      if (f.update(i * 100 * MS, v, Units::Km)) when = i * 100 * MS;
    }
    CHECK_EQ(when, 700 * MS);
    CHECK(when > 0 && when <= 1400 * MS);
    CHECK_EQ(shown(f), std::string("5.1"));
  }
  // Steady jitter of +-0.07 km/h around a value never moves the display.
  {
    SpeedDisplayFilter f;
    f.update(0, 1389, Units::Km);  // 5.00 -> 5.0
    uint32_t n = 0;
    for (int64_t t = 1; t < 300; ++t)
      if (f.update(t * SEC, 1389 + static_cast<int32_t>((t % 3) - 1) * 19, Units::Km)) ++n;  // c 495..505
    CHECK_EQ(n, 0u);
  }

  // Plain fmtSpeed against the oracle (0..12000 mm/s, both units) and
  // distance (pulses 0..20000).
  replay::Golden g;
  if (!CHECK(loadGolden("format_misc.txt", g))) return;
  if (g.has("case")) CHECK_EQ(g.get("case"), std::string("format_misc"));
  const std::vector<std::string> sp = g.all("speed");
  const std::vector<std::string> di = g.all("dist");
  CHECK_EQ(sp.size(), static_cast<size_t>(12001));
  CHECK_EQ(di.size(), static_cast<size_t>(20001));
  uint32_t bad = 0, mism = 0;
  for (size_t i = 0; i < sp.size(); ++i) {
    const std::vector<std::string> f = splitSpaces(sp[i]);
    uint32_t v = 0;
    if (f.size() != 3 || !replay::parseU32(f[0], v) || v != i) {
      if (++bad <= 5) std::fprintf(stderr, "  bad speed row %zu: %s\n", i, sp[i].c_str());
      continue;
    }
    const std::string km = speed(static_cast<int32_t>(v), Units::Km), mi = speed(static_cast<int32_t>(v), Units::Mi);
    if (km != f[1] || mi != f[2])
      if (++mism <= 10)
        std::fprintf(stderr, "  speed %u: C++ %s %s, oracle %s %s\n", v, km.c_str(), mi.c_str(), f[1].c_str(), f[2].c_str());
  }
  for (size_t i = 0; i < di.size(); ++i) {
    const std::vector<std::string> f = splitSpaces(di[i]);
    uint32_t p = 0;
    if (f.size() != 3 || !replay::parseU32(f[0], p) || p != i) {
      if (++bad <= 5) std::fprintf(stderr, "  bad dist row %zu: %s\n", i, di[i].c_str());
      continue;
    }
    const std::string km = dist(p, Units::Km), mi = dist(p, Units::Mi);
    if (km != f[1] || mi != f[2])
      if (++mism <= 10)
        std::fprintf(stderr, "  dist %u: C++ %s %s, oracle %s %s\n", p, km.c_str(), mi.c_str(), f[1].c_str(), f[2].c_str());
  }
  CHECK_EQ(bad, 0u);
  CHECK_EQ(mism, 0u);
}

// ---------------------------------------------------------------------------
// 3.5 Distance floor, 3.6 durations
// ---------------------------------------------------------------------------
void test35() {
  CHECK_EQ(dist(2693, Units::Mi), std::string("0.99"));  // 1608.798 m
  CHECK_EQ(dist(2694, Units::Mi), std::string("1.00"));  // 1609.396 m
  CHECK_EQ(dist(16, Units::Km), std::string("0.00"));    // 9.558 m
  CHECK_EQ(dist(17, Units::Km), std::string("0.01"));    // 10.156 m
  CHECK_EQ(dist(1239, Units::Mi), std::string("0.45"));  // 0.45993 mi; 1609 m/mi would give 0.46
  CHECK_EQ(dist(0, Units::Km), std::string("0.00"));
  CHECK_EQ(dist(167392, Units::Km), std::string("99.99"));  // 99.9999808 km: worst case for the cell
  CHECK_EQ(dist(167393, Units::Km), std::string("100.00"));
  CHECK_EQ(dist(UINT32_MAX, Units::Km), std::string("2565813.46"));
  // monotonic in pulses
  uint32_t nonMono = 0;
  for (Units u : {Units::Km, Units::Mi}) {
    uint64_t prev = 0;
    for (uint32_t p = 0; p <= 200000; ++p) {
      const uint64_t h = ot::distanceHundredths(p, u);
      if (h < prev) ++nonMono;
      prev = h;
    }
  }
  CHECK_EQ(nonMono, 0u);
}

void test36() {
  CHECK_EQ(dur(3599999999), std::string("59:59"));
  CHECK_EQ(dur(3600000000), std::string("1:00:00"));
  CHECK_EQ(dur(35999000000), std::string("9:59:59"));
  CHECK_EQ(dur(36000000000), std::string("10:00:00"));
  CHECK_EQ(dur(0), std::string("0:00"));
  CHECK_EQ(dur(999999), std::string("0:00"));  // floored
  CHECK_EQ(dur(1000000), std::string("0:01"));
  CHECK_EQ(dur(61 * SEC), std::string("1:01"));
  CHECK_EQ(dur(-5), std::string("0:00"));
  CHECK_EQ(dur(INT64_MAX), std::string("2562047788:00:54"));
}

// ---------------------------------------------------------------------------
// 3.7 Buffer bounds: exactly the needed size and one byte less, for every
// formatter. Heap buffers of the exact size, so ASan sees any overflow.
// ---------------------------------------------------------------------------
template <class F>
void checkBounds(const char* what, F fmt) {
  char big[64];
  std::memset(big, 'X', sizeof(big));
  const size_t len = fmt(big, sizeof(big));
  const std::string full(big);
  CHECK_EQ(full.size(), len);
  CHECK(len > 0);
  {
    std::vector<char> exact(len + 1, 'X');
    CHECK_EQ(fmt(exact.data(), exact.size()), len);
    CHECK_EQ(std::string(exact.data()), full);
  }
  {
    std::vector<char> shortBuf(len, 'X');  // one byte less
    CHECK_EQ(fmt(shortBuf.data(), shortBuf.size()), len);
    CHECK_EQ(shortBuf[len - 1], '\0');
    CHECK_EQ(std::string(shortBuf.data()), full.substr(0, len - 1));
  }
  {
    std::vector<char> one(1, 'X');
    CHECK_EQ(fmt(one.data(), 1), len);
    CHECK_EQ(one[0], '\0');
  }
  {
    char untouched = 'X';
    CHECK_EQ(fmt(&untouched, 0), len);  // n == 0: nothing written
    CHECK_EQ(untouched, 'X');
    CHECK_EQ(fmt(nullptr, 0), len);
  }
  if (gVerbose) std::printf("  bounds %s: \"%s\" (%zu)\n", what, full.c_str(), len);
}

void test37() {
  for (int32_t v : {0, 167, 361, 444, 2084, 12000, INT32_MAX})
    for (Units u : {Units::Km, Units::Mi}) {
      checkBounds("pace", [&](char* b, size_t n) { return ot::fmtPace(b, n, v, u, true); });
      checkBounds("speed", [&](char* b, size_t n) { return ot::fmtSpeed(b, n, v, u); });
    }
  for (uint32_t s : {0u, 59u, 600u, ot::PACE_MAX_S, ot::PACE_MAX_S + 1u, UINT32_MAX})
    checkBounds("pace_s", [&](char* b, size_t n) { return ot::fmtPaceSeconds(b, n, s); });
  for (uint32_t t : {0u, 9u, 250u, UINT32_MAX})
    checkBounds("tenths", [&](char* b, size_t n) { return ot::fmtTenths(b, n, t); });
  for (uint32_t p : {0u, 2694u, 167393u, UINT32_MAX})
    for (Units u : {Units::Km, Units::Mi})
      checkBounds("dist", [&](char* b, size_t n) { return ot::fmtDistance(b, n, p, u); });
  for (int64_t us : {static_cast<int64_t>(0), static_cast<int64_t>(3599999999), static_cast<int64_t>(36000000000),
                     static_cast<int64_t>(-1), INT64_MAX})
    checkBounds("dur", [&](char* b, size_t n) { return ot::fmtDuration(b, n, us); });
  {
    PaceDisplayFilter pf;
    checkBounds("pace_filter_hidden", [&](char* b, size_t n) { return pf.format(b, n); });
    pf.update(0, 444, Units::Mi, true);
    checkBounds("pace_filter_shown", [&](char* b, size_t n) { return pf.format(b, n); });
    SpeedDisplayFilter sf;
    sf.update(0, INT32_MAX, Units::Km);
    checkBounds("speed_filter", [&](char* b, size_t n) { return sf.format(b, n); });
  }
  // The *_STR_SIZE buffers hold the longest strings of each formatter.
  char b[64];
  CHECK(ot::fmtPace(b, sizeof(b), 167, Units::Km, true) < ot::PACE_STR_SIZE);
  CHECK(ot::fmtPaceSeconds(b, sizeof(b), ot::PACE_MAX_S) < ot::PACE_STR_SIZE);
  CHECK(ot::fmtSpeed(b, sizeof(b), INT32_MAX, Units::Km) < ot::SPEED_STR_SIZE);
  CHECK(ot::fmtSpeed(b, sizeof(b), INT32_MAX, Units::Mi) < ot::SPEED_STR_SIZE);
  CHECK(ot::fmtDistance(b, sizeof(b), UINT32_MAX, Units::Km) < ot::DIST_STR_SIZE);
  CHECK(ot::fmtDuration(b, sizeof(b), INT64_MAX) < ot::DUR_STR_SIZE);
  CHECK_EQ(std::string(ot::unitsName(Units::Mi)), std::string("mi"));
  CHECK_EQ(std::string(ot::unitsName(Units::Km)), std::string("km"));
  CHECK_EQ(std::string(ot::speedUnitName(Units::Mi)), std::string("mph"));
  CHECK_EQ(std::string(ot::speedUnitName(Units::Km)), std::string("km/h"));
  CHECK_EQ(std::string(ot::paceUnitName(Units::Mi)), std::string("/mi"));
  CHECK_EQ(std::string(ot::paceUnitName(Units::Km)), std::string("/km"));
  CHECK_EQ(static_cast<int>(Units::Mi), 0);  // NVS "units" byte: 0 imperial, 1 metric
  CHECK_EQ(static_cast<int>(Units::Km), 1);
}

// ---------------------------------------------------------------------------
// 3.8 Replay S2 through Format: P lines -> Metrics -> Workout ->
// PaceDisplayFilter, sampled at 1 Hz AND at 10 Hz (the
// device's frame rate, RENDER_MAX_HZ). Envelope (walk
// 4.6 km/h): <= 60 pace changes and <= 20 reversals per minute, as rates over
// RUNNING time (a reversal = the shown number moves in the opposite direction
// to its previous move). The worst sliding minute inside RUNNING is checked
// against 60 changes and reported (reversals: diagnostic only).
// At 1 Hz the "<= 60 changes per minute" checks are structural (one sample
// per second can change at most 60 times a minute) and cannot catch a broken
// PACE_REFRESH_US; the 10 Hz run is where the rate limit is really exercised.
// ---------------------------------------------------------------------------
void run38(const std::vector<int64_t>& pulses, const std::vector<replay::WorkoutInput>& in, Units u, int hz) {
  const int64_t step = SEC / hz;
  const size_t win = static_cast<size_t>(60 * hz);  // samples per minute
  replay::WorkoutReplay rp;
  PaceDisplayFilter pf;
  SpeedDisplayFilter sf;
  size_t next = 0;
  struct S { bool running; bool changed; bool reversal; };
  std::vector<S> samples;
  std::string prev;
  int prevDir = 0;
  bool havePrevNum = false;
  uint32_t prevNum = 0;
  uint32_t speedChanges = 0;
  for (int64_t t = pulses.front(); t <= pulses.back() + 5 * SEC; t += step) {
    rp.runUntil(in, next, t);
    const ot::WorkoutSnapshot ws = rp.workout.snapshot(t);
    const int32_t v = rp.metrics.speedMmps(t);
    pf.update(t, v, u, ot::paceShownInState(ws.state));
    if (sf.update(t, v, u)) ++speedChanges;
    const std::string cur = shown(pf);
    S s{ws.state == WorkoutState::Running, !samples.empty() && cur != prev, false};
    if (pf.shown()) {
      if (havePrevNum && pf.shownSeconds() != prevNum) {
        const int dir = pf.shownSeconds() > prevNum ? 1 : -1;
        if (prevDir != 0 && dir != prevDir) s.reversal = true;
        prevDir = dir;
      }
      havePrevNum = true;
      prevNum = pf.shownSeconds();
    } else {
      havePrevNum = false;
      prevDir = 0;
    }
    samples.push_back(s);
    prev = cur;
  }
  // Prefix sums -> sliding one-minute windows entirely in RUNNING.
  std::vector<uint32_t> pc(samples.size() + 1, 0), pr(samples.size() + 1, 0), pn(samples.size() + 1, 0);
  for (size_t i = 0; i < samples.size(); ++i) {
    pc[i + 1] = pc[i] + (samples[i].running && samples[i].changed ? 1u : 0u);
    pr[i + 1] = pr[i] + (samples[i].running && samples[i].reversal ? 1u : 0u);
    pn[i + 1] = pn[i] + (samples[i].running ? 1u : 0u);
  }
  const uint32_t runningSamples = pn[samples.size()], totalCh = pc[samples.size()], totalRev = pr[samples.size()];
  uint32_t windows = 0, maxCh = 0, maxRev = 0;
  for (size_t i = 0; i + win <= samples.size(); ++i) {
    if (pn[i + win] - pn[i] != win) continue;
    ++windows;
    const uint32_t ch = pc[i + win] - pc[i], rev = pr[i + win] - pr[i];
    if (ch > maxCh) maxCh = ch;
    if (rev > maxRev) maxRev = rev;
  }
  // rates per minute of RUNNING time: count * 60 * hz / runningSamples
  const double mins = static_cast<double>(runningSamples) / (60.0 * hz);
  std::printf("  3.8 S2 %s @%d Hz: %.2f running min, %u full-minute windows; pace changes/min max %u (avg %.1f),"
              " reversals/min max %u (avg %.1f); speed changes %u\n",
              ot::unitsName(u), hz, mins, windows, maxCh, mins > 0 ? totalCh / mins : 0.0, maxRev,
              mins > 0 ? totalRev / mins : 0.0, speedChanges);
  CHECK(windows > 0u);
  CHECK(runningSamples >= 120u * static_cast<uint32_t>(hz));
  CHECK(maxCh <= 60u);
  // avg changes <= 60/min and reversals <= 20/min of RUNNING time
  const uint64_t perMin = 60u * static_cast<uint64_t>(hz);  // samples per minute
  CHECK(static_cast<uint64_t>(totalCh) * perMin <= 60u * static_cast<uint64_t>(runningSamples));
  CHECK(static_cast<uint64_t>(totalRev) * perMin <= 20u * static_cast<uint64_t>(runningSamples));
}

void test38() {
  replay::Log log;
  std::string err;
  if (!CHECK(replay::loadFile(gRoot + "/" + kS2, log, err))) {
    std::fprintf(stderr, "  %s\n", err.c_str());
    return;
  }
  std::vector<int64_t> pulses;
  for (const replay::PulseLine& p : log.pulses) pulses.push_back(p.t);
  if (!CHECK(pulses.size() > 100u)) return;
  const std::vector<replay::WorkoutInput> in = replay::pulseInputs(pulses);
  // S2 results: 1 Hz avg 54.7 changes / 15.7 reversals per min, worst
  // sliding minute 24 reversals; 10 Hz avg ~57 changes / ~15.5 reversals.
  for (int hz : {1, 10})
    for (Units u : {Units::Km, Units::Mi}) run38(pulses, in, u, hz);
}

}  // namespace

int main(int argc, char** argv) {
  gRoot = replay::repoRootFromArgs(argc, argv);
  for (int i = 1; i < argc; ++i)
    if (std::strcmp(argv[i], "--verbose") == 0) gVerbose = true;
  test31();
  test32();
  test33();
  test34();
  test35();
  test36();
  test37();
  test38();
  return check::summary("test_format");
}
