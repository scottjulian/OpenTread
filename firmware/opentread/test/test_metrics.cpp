// test_metrics.cpp - host tests for fixed.h and Metrics:
// tests 1.5-1.15 (1.14 metrics part) plus a seeded property test (~10^6
// pulses) against a direct restatement of the span-estimator definition.
//
// Floating point appears only here, for reporting and for the test-1.11
// continuous integral (defined with M/(t - t_k)).
//
// Usage: test_metrics --root <repo root>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "fixed.h"
#include "metrics.h"
#include "replay.h"

using ot::Metrics;

namespace {

std::string gRoot = ".";
const char* const kGoldenDir = "firmware/opentread/test/golden/";
constexpr int64_t SEC = 1000000;
constexpr int64_t M_UM = ot::M_PER_PULSE_UM;

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

bool loadPulses(const char* name, replay::Log& log, std::vector<int64_t>& pulses) {
  for (const replay::NamedLog& nl : replay::kLogs) {
    if (std::string(nl.name) != name) continue;
    std::string err;
    if (!replay::loadFile(gRoot + "/" + nl.path, log, err) || !replay::refilter(log, ot::Config{}, pulses, err)) {
      std::fprintf(stderr, "  %s: %s\n", name, err.c_str());
      return false;
    }
    return true;
  }
  return false;
}

const char* logPath(const char* name) {
  for (const replay::NamedLog& nl : replay::kLogs) if (std::string(nl.name) == name) return nl.path;
  return "";
}

bool loadGoldenFile(const std::string& file, replay::Golden& g) {
  std::string err;
  if (replay::loadGolden(gRoot + "/" + kGoldenDir + file, g, err)) return true;
  std::fprintf(stderr, "  %s\n", err.c_str());
  return false;
}

double goldenNum(const replay::Golden& g, const std::string& key) {
  double v = NAN;
  if (!replay::parseDouble(g.get(key), v)) {
    std::fprintf(stderr, "  golden key '%s' missing or not a number\n", key.c_str());
    return NAN;
  }
  return v;
}

// Metrics fed with a list of pulses; returns speedAtLastPulse after each.
std::vector<int32_t> pulseSpeeds(const std::vector<int64_t>& pulses) {
  Metrics m;
  std::vector<int32_t> v;
  v.reserve(pulses.size());
  for (int64_t t : pulses) { m.onPulse(t); v.push_back(m.speedAtLastPulseMmps()); }
  return v;
}

// ---------------------------------------------------------------- 1.5
void testOdometer() {
  struct Row { uint32_t pulses; uint64_t um, m, dm; };
  const Row rows[] = {
      {0, 0, 0, 0},
      {1, 597400, 0, 5},
      {2, 1194800, 1, 11},
      {1674, 1000047600, 1000, 10000},
      {20665, 12345271000ull, 12345, 123452},
      {4000000, 2389600000000ull, 2389600, 23896000},
      {UINT32_MAX, 2565813462033000ull, 2565813462ull, 25658134620ull},
  };
  for (const Row& r : rows) {
    CHECK_EQ(ot::pulsesToUm(r.pulses), r.um);
    CHECK_EQ(ot::ftmsDistanceM(r.pulses), r.m);
    CHECK_EQ(ot::rscDistanceDm(r.pulses), r.dm);
  }
  // Odometer through Metrics: 4,000,000 pulses at 10 pulses/s, no overflow.
  Metrics m;
  int64_t t = 1000;
  for (uint32_t i = 0; i < 4000000u; ++i, t += 100000) m.onPulse(t);
  CHECK_EQ(m.odometerPulses(), 4000000u);
  CHECK_EQ(ot::pulsesToUm(m.odometerPulses()), static_cast<uint64_t>(2389600000000ull));
  CHECK_EQ(m.speedAtLastPulseMmps(), ot::spanSpeedMmps(30, 3000000));  // 30 x 0.1 s = SPAN
  m.reset();
  CHECK_EQ(m.odometerPulses(), 0u);
  CHECK(!m.hasPulse());
  CHECK_EQ(m.speedMmps(t), 0);
  // Output speed conversions on a few values.
  CHECK_EQ(ot::ftmsSpeedCentiKmh(2778), 1000u);  // 10.0 km/h
  CHECK_EQ(ot::ftmsSpeedCentiKmh(0), 0u);
  CHECK_EQ(ot::rscSpeed256(1000), 256u);
  CHECK_EQ(ot::rscSpeed256(2778), 711u);  // (2778*256+500)/1000 = 711.668 -> 711
}

// ---------------------------------------------------------------- 1.6
void testSpeedUnits() {
  const int64_t dts[] = {1340000, 170000, 500000};
  const int32_t want[] = {446, 3514, 1195};
  for (int i = 0; i < 3; ++i) {
    Metrics m;
    m.onPulse(5 * SEC);
    m.onPulse(5 * SEC + dts[i]);
    CHECK_EQ(m.speedAtLastPulseMmps(), want[i]);
    CHECK_EQ(m.speedMmps(5 * SEC + dts[i]), want[i]);
    CHECK_EQ(ot::spanSpeedMmps(1, static_cast<uint64_t>(dts[i])), want[i]);
  }
  // Rounding is half-up: 597400000 / 1340000 = 445.82 -> 446 (floor would give 445).
  CHECK_EQ(ot::spanSpeedMmps(2, 2 * 1340000), 446);
}

// ---------------------------------------------------------------- 1.7
void testFirstInterval() {
  Metrics m;
  CHECK_EQ(m.speedMmps(0), 0);
  m.onPulse(10 * SEC);
  CHECK_EQ(m.speedAtLastPulseMmps(), 0);
  for (int64_t d = 0; d <= 3 * SEC; d += 10000) CHECK_EQ(m.speedMmps(10 * SEC + d), 0);
  m.onPulse(10 * SEC + 500000);
  CHECK_EQ(m.speedAtLastPulseMmps(), 1195);
  // First pulse after a gap > STOP is again a bout start (0); exactly STOP is not.
  m.onPulse(10 * SEC + 500000 + ot::STOP_US + 1);
  CHECK_EQ(m.speedAtLastPulseMmps(), 0);
  m.onPulse(m.lastPulseUs() + ot::STOP_US);
  CHECK_EQ(m.speedAtLastPulseMmps(), ot::spanSpeedMmps(1, ot::STOP_US));
  CHECK(m.speedAtLastPulseMmps() > 0);
}

// ---------------------------------------------------------------- 1.8
void testSpanStopGap() {
  auto run = [](int64_t gap) {
    std::vector<int64_t> p;
    int64_t t = SEC;
    for (int i = 0; i < 5; ++i, t += 500000) p.push_back(t);
    t = p.back() + gap;
    p.push_back(t);
    p.push_back(t + 500000);
    return pulseSpeeds(p);
  };
  const std::vector<int32_t> a = run(2600000);  // gap > STOP
  CHECK_EQ(a[5], 0);
  CHECK_EQ(a[6], 1195);  // only the last interval
  // Gap exactly STOP: not a stop gap, so the span may cross it. Pulse 6
  // (after the 2.5 s gap) reaches exactly SPAN at N = 2 (0.5 + 2.5 s) and
  // stops there (>=); pulse 7 likewise (2.5 + 0.5 s).
  const std::vector<int32_t> b = run(2500000);
  CHECK_EQ(b[5], ot::spanSpeedMmps(2, 3000000));
  CHECK_EQ(b[6], ot::spanSpeedMmps(2, 3000000));
  CHECK_EQ(b[6], 398);
  // Span boundary: steady 0.5 s pulses -> N = 6 (exactly 3.0 s).
  std::vector<int64_t> s;
  for (int i = 0; i < 20; ++i) s.push_back(SEC + i * 500000);
  CHECK_EQ(pulseSpeeds(s).back(), ot::spanSpeedMmps(6, 3000000));
  // Span 1 us short of SPAN at N = 3 must extend to N = 4.
  std::vector<int64_t> u = {SEC, SEC + 1, SEC + 1000000, SEC + 2000000, SEC + 3000000};
  CHECK_EQ(pulseSpeeds(u).back(), ot::spanSpeedMmps(4, 3000000));
  // ... and with the oldest interval 1 us longer it stops at N = 3 exactly.
  std::vector<int64_t> w = {SEC - 1, SEC, SEC + 1000000, SEC + 2000000, SEC + 3000000};
  CHECK_EQ(pulseSpeeds(w).back(), ot::spanSpeedMmps(3, 3000000));
}

// ---------------------------------------------------------------- 1.9
void testDecay() {
  Metrics m;
  int64_t t = 7 * SEC;
  for (int i = 0; i < 10; ++i, t += 500000) m.onPulse(t);
  const int64_t T = m.lastPulseUs();
  const int32_t v = m.speedAtLastPulseMmps();
  CHECK_EQ(v, 1195);
  int32_t prev = m.speedMmps(T);
  CHECK_EQ(prev, v);
  bool nonInc = true, capOk = true, positive = true;
  for (int64_t d = 1; d <= ot::STOP_US; d += 997) {
    const int32_t s = m.speedMmps(T + d);
    nonInc = nonInc && s <= prev;
    const int32_t cap = ot::decayCapMmps(static_cast<uint64_t>(d));  // floor(M/d)
    capOk = capOk && s == (cap < v ? cap : v);
    positive = positive && s > 0;
    prev = s;
  }
  CHECK(nonInc);
  CHECK(capOk);
  CHECK(positive);
  CHECK_EQ(m.speedMmps(T + ot::STOP_US), 238);  // exactly 2.500000 s: floor(238.96), still > 0
  CHECK_EQ(m.speedMmps(T + ot::STOP_US + 1), 0);
  CHECK_EQ(m.speedMmps(T + 3 * SEC), 0);
  CHECK_EQ(m.speedMmps(T + 3600 * SEC), 0);
  CHECK_EQ(m.speedMmps(T + 600000), 995);  // floor(995.67) < 1195
  CHECK_EQ(m.speedMmps(T + 400000), 1195);                           // cap 1494 > v
}

// ---------------------------------------------------------------- 1.10
void testOracleAgreement() {
  const char* names[] = {"S2_walk", "S1b_10rev", "S1_handturn"};
  for (const char* name : names) {
    replay::Log log;
    std::vector<int64_t> pulses;
    if (!CHECK(loadPulses(name, log, pulses)) || pulses.empty()) continue;
    replay::Golden g;
    if (!CHECK(loadGoldenFile(std::string("metrics_") + name + ".txt", g))) continue;
    const int64_t start = pulses.front() - SEC, end = pulses.back() + 5 * SEC, step = 50000;
    CHECK_EQ(g.get("log"), std::string(logPath(name)));
    CHECK_EQ(g.get("step_us"), std::to_string(step));
    CHECK_EQ(g.get("start_us"), std::to_string(start));
    CHECK_EQ(g.get("end_us"), std::to_string(end));
    const std::vector<replay::Sample> mine = replay::sampleMetrics(pulses, start, end, step);
    const std::vector<std::string> gs = g.all("s");
    CHECK_EQ(gs.size(), mine.size());
    long bad = 0, parseErr = 0;
    int64_t worst = 0;
    for (size_t i = 0; i < gs.size() && i < mine.size(); ++i) {
      int64_t gt = 0, gv = 0;
      uint32_t go = 0;
      const std::string& line = gs[i];
      const size_t a = line.find(' '), b = line.find(' ', a == std::string::npos ? a : a + 1);
      if (a == std::string::npos || b == std::string::npos || !replay::parseI64(line.substr(0, a), gt) ||
          !replay::parseI64(line.substr(a + 1, b - a - 1), gv) || !replay::parseU32(line.substr(b + 1), go)) {
        ++parseErr;
        continue;
      }
      const int64_t diff = gv - mine[i].speedMmps;
      const int64_t ad = diff < 0 ? -diff : diff;
      if (ad > worst) worst = ad;
      if (gt != mine[i].t || ad > 1 || go != mine[i].odometer) {
        if (bad < 5) std::fprintf(stderr, "  1.10 %s t=%lld golden=(%lld,%u) c++=(%d,%u)\n", name, static_cast<long long>(gt),
                                  static_cast<long long>(gv), go, mine[i].speedMmps, mine[i].odometer);
        ++bad;
      }
    }
    std::printf("  1.10 %-11s samples=%zu worst |c++ - oracle| = %lld mm/s\n", name, mine.size(), static_cast<long long>(worst));
    CHECK_EQ(parseErr, 0L);
    CHECK_EQ(bad, 0L);
  }
}

// ---------------------------------------------------------------- 1.11
struct Bias {
  double truthM = 0, continuousErrM = 0;
  double phaseErrM[10] = {};
  double minM = 0, maxM = 0, meanM = 0;
};

// Test 1.11 definition.
Bias computeBias(const std::vector<int64_t>& pulses) {
  Bias b;
  const int64_t n = static_cast<int64_t>(pulses.size());
  const int64_t truthUm = n * M_UM;
  b.truthM = static_cast<double>(truthUm) / 1e6;
  // Continuous: constant v_k until M/v_k after pulse k, then M/(t - t_k),
  // ending at the next pulse or at t_k + STOP.
  const std::vector<int32_t> v = pulseSpeeds(pulses);
  const double Mm = static_cast<double>(M_UM) / 1e6;  // metres
  double integral = 0;
  for (size_t k = 0; k < pulses.size(); ++k) {
    if (v[k] <= 0) continue;
    const int64_t endUs = (k + 1 < pulses.size()) ? std::min(pulses[k + 1], pulses[k] + ot::STOP_US) : pulses[k] + ot::STOP_US;
    const double D = static_cast<double>(endUs - pulses[k]) / 1e6;  // s
    const double vs = static_cast<double>(v[k]) / 1000.0;             // m/s
    const double tau = Mm / vs;
    integral += (D <= tau) ? vs * D : Mm + Mm * std::log(D / tau);
  }
  b.continuousErrM = integral - b.truthM;
  // 1 Hz sampling, phases p = 0..9: t = t_first - 2 s + p*0.1 s + i*1 s
  // until t_last + 5 s, integral = sum v x 1 s. Exact in integers (um).
  b.minM = 1e18;
  b.maxM = -1e18;
  double sum = 0;
  for (int p = 0; p < 10; ++p) {
    const int64_t start = pulses.front() - 2 * SEC + p * 100000;
    const std::vector<replay::Sample> s = replay::sampleMetrics(pulses, start, pulses.back() + 5 * SEC, SEC);
    int64_t mm = 0;
    for (const replay::Sample& x : s) mm += x.speedMmps;  // mm/s x 1 s
    const int64_t errUm = mm * 1000 - truthUm;
    b.phaseErrM[p] = static_cast<double>(errUm) / 1e6;
    b.minM = std::min(b.minM, b.phaseErrM[p]);
    b.maxM = std::max(b.maxM, b.phaseErrM[p]);
    sum += b.phaseErrM[p];
  }
  b.meanM = sum / 10.0;
  return b;
}

void testWatchBias() {
  struct RefRow { const char* name; double truth, cont, mn, mx, mean; bool bounded; };
  const RefRow rows[] = {
      {"S2_walk", 255.687, -1.171, -2.26, -0.41, -1.33, true},
      {"S1b_10rev", 36.441, -0.013, -0.28, 0.20, -0.02, true},
      {"S1_handturn", 97.974, -5.061, -5.42, -4.60, -5.01, false},  // characterisation only
  };
  // The reference figures must be reproduced to the digits quoted (half a
  // unit of the last quoted decimal: 3 dp for continuous, 2 dp for phases).
  // Against the oracle's goldens the tolerance is +-0.02 m.
  const double TOL_3DP = 0.0005 + 1e-9, TOL_2DP = 0.005 + 1e-9, TOL_ORACLE = 0.02;
  for (const RefRow& r : rows) {
    replay::Log log;
    std::vector<int64_t> pulses;
    if (!CHECK(loadPulses(r.name, log, pulses)) || pulses.empty()) continue;
    const Bias b = computeBias(pulses);
    std::printf("  1.11 %-11s truth=%.6f continuous=%+.6f phases min=%+.6f max=%+.6f mean=%+.6f [", r.name, b.truthM,
                b.continuousErrM, b.minM, b.maxM, b.meanM);
    for (double e : b.phaseErrM) std::printf(" %+.3f", e);
    std::printf(" ]\n");
    CHECK_NEAR(b.truthM, r.truth, 0.0005);
    CHECK_NEAR(b.continuousErrM, r.cont, TOL_3DP);
    CHECK_NEAR(b.minM, r.mn, TOL_2DP);
    CHECK_NEAR(b.maxM, r.mx, TOL_2DP);
    CHECK_NEAR(b.meanM, r.mean, TOL_2DP);
    if (r.bounded) {
      for (double e : b.phaseErrM) CHECK(std::fabs(e) <= 0.01 * b.truthM);
      CHECK(std::fabs(b.continuousErrM) <= 0.01 * b.truthM);
    }
    replay::Golden g;
    if (!CHECK(loadGoldenFile(std::string("bias_") + r.name + ".txt", g))) continue;
    CHECK_NEAR(goldenNum(g, "truth_m"), b.truthM, 5e-7);
    CHECK_NEAR(goldenNum(g, "continuous_err_m"), b.continuousErrM, TOL_ORACLE);
    for (int p = 0; p < 10; ++p) CHECK_NEAR(goldenNum(g, "phase" + std::to_string(p) + "_err_m"), b.phaseErrM[p], TOL_ORACLE);
    CHECK_NEAR(goldenNum(g, "phase_min_err_m"), b.minM, TOL_ORACLE);
    CHECK_NEAR(goldenNum(g, "phase_max_err_m"), b.maxM, TOL_ORACLE);
    CHECK_NEAR(goldenNum(g, "phase_mean_err_m"), b.meanM, TOL_ORACLE);
  }
}

// ---------------------------------------------------------------- 1.12
// HF noise: 20 Hz samples of speedMmps on [a, b] (seconds after the first
// accepted pulse); m(t) = mean of the 20 Hz samples within +-2 s of t (the
// 4 s centred mean, grid extended 2 s past both ends); noise = RMS of
// (v(t) - m(t)) / m(t) over the samples in [a, b].
struct Noise { double rms, meanKmh; };
Noise hfNoise(const std::vector<int64_t>& pulses, double a, double b) {
  const int64_t A = pulses.front() + static_cast<int64_t>(std::llround(a * 1e6));
  const int64_t B = pulses.front() + static_cast<int64_t>(std::llround(b * 1e6));
  const std::vector<replay::Sample> s = replay::sampleMetrics(pulses, A - 2 * SEC, B + 2 * SEC, 50000);
  double sq = 0, sumV = 0;
  long n = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i].t < A || s[i].t > B) continue;
    double sum = 0;
    long c = 0;
    for (size_t j = 0; j < s.size(); ++j) {
      const int64_t d = s[j].t - s[i].t;
      if (d >= -2 * SEC && d <= 2 * SEC) { sum += s[j].speedMmps; ++c; }
    }
    const double m = sum / static_cast<double>(c);
    const double r = (s[i].speedMmps - m) / m;
    sq += r * r;
    sumV += s[i].speedMmps;
    ++n;
  }
  return {std::sqrt(sq / static_cast<double>(n)), sumV / static_cast<double>(n) * 0.0036};
}

void testNoise() {
  replay::Log log;
  std::vector<int64_t> pulses;
  if (!CHECK(loadPulses("S2_walk", log, pulses)) || pulses.empty()) return;
  const Noise sprint = hfNoise(pulses, 1139.0, 1143.0);  // sprint plateau ~12.4 km/h
  const Noise walk = hfNoise(pulses, 1217.0, 1223.0);    // walk ~4.6-4.7 km/h
  std::printf("  1.12 sprint plateau noise=%.4f%% mean=%.2f km/h; walk noise=%.4f%% mean=%.2f km/h\n", sprint.rms * 100,
              sprint.meanKmh, walk.rms * 100, walk.meanKmh);
  CHECK(sprint.meanKmh > 12.0 && sprint.meanKmh < 12.7);  // the window really is the plateau
  CHECK(walk.meanKmh > 4.4 && walk.meanKmh < 4.9);
  CHECK(sprint.rms <= 0.010);
  CHECK(walk.rms <= 0.030);
}

// ---------------------------------------------------------------- 1.13
std::vector<int32_t> refSpeeds(const std::vector<int64_t>& p, uint32_t maxN);

void testSyntheticTrains() {
  // 25 km/h = 6944.4 mm/s -> 86026 us intervals, +-3% uniform jitter.
  {
    Rng rng(25);
    const int64_t base = 86026, jit = base * 3 / 100;
    std::vector<int64_t> p;
    int64_t t = SEC;
    for (int i = 0; i < 3000; ++i) { p.push_back(t); t += base + rng.range(-jit, jit); }
    const std::vector<int32_t> v = pulseSpeeds(p);
    double sumPulse = 0;
    long nPulse = 0;
    for (size_t k = 0; k < p.size(); ++k) if (p[k] - p[0] >= 4 * SEC) { sumPulse += v[k]; ++nPulse; }
    const std::vector<replay::Sample> s = replay::sampleMetrics(p, p[0] + 4 * SEC, p.back(), 50000);
    double sumS = 0;
    for (const replay::Sample& x : s) sumS += x.speedMmps;
    const double nominal = 25000.0 / 3.6;
    const double meanPulse = sumPulse / static_cast<double>(nPulse), meanS = sumS / static_cast<double>(s.size());
    std::printf("  1.13 25 km/h: mean at pulses %.1f, 20 Hz %.1f mm/s (nominal %.1f)\n", meanPulse, meanS, nominal);
    CHECK(std::fabs(meanPulse / nominal - 1.0) <= 0.003);
    CHECK(std::fabs(meanS / nominal - 1.0) <= 0.003);
  }
  // 1 mph at 1.34 s intervals: 446 +- 5 mm/s, never 0 between pulses.
  {
    std::vector<int64_t> p;
    for (int i = 0; i < 40; ++i) p.push_back(3 * SEC + i * 1340000);
    const std::vector<int32_t> v = pulseSpeeds(p);
    bool inBand = true;
    for (size_t k = 1; k < v.size(); ++k) inBand = inBand && v[k] >= 441 && v[k] <= 451;
    CHECK(inBand);
    const std::vector<replay::Sample> s = replay::sampleMetrics(p, p[1], p.back() + 1339999, 10000);
    bool never0 = true;
    for (const replay::Sample& x : s) never0 = never0 && x.speedMmps > 0;
    CHECK(never0);
  }
  // Ring of 64 never exhausted at 40 km/h (53766 us, +-3%): identical to an
  // unbounded reference. At 70 km/h it IS exhausted and N stops at 63.
  {
    Rng rng(40);
    std::vector<int64_t> p;
    int64_t t = SEC;
    const int64_t base = 53766, jit = base * 3 / 100;
    for (int i = 0; i < 5000; ++i) { p.push_back(t); t += base + rng.range(-jit, jit); }
    CHECK(pulseSpeeds(p) == refSpeeds(p, 1000000u));
    std::vector<int64_t> q;
    for (int i = 0; i < 500; ++i) q.push_back(SEC + i * 30723);  // 70 km/h
    const std::vector<int32_t> vq = pulseSpeeds(q);
    CHECK(vq == refSpeeds(q, ot::TS_RING - 1u));
    CHECK_EQ(vq.back(), ot::spanSpeedMmps(63, 63 * 30723));
  }
}

// ---------------------------------------------------------------- 1.14
void testUptime() {
  replay::Log log;
  std::vector<int64_t> pulses;
  if (!CHECK(loadPulses("S2_walk", log, pulses)) || pulses.empty()) return;
  const int64_t OFF = 864000000000LL;
  std::vector<int64_t> shifted = pulses;
  for (int64_t& t : shifted) t += OFF;
  const std::vector<replay::Sample> a = replay::sampleMetrics(pulses, pulses.front() - SEC, pulses.back() + 5 * SEC, 50000);
  const std::vector<replay::Sample> b =
      replay::sampleMetrics(shifted, shifted.front() - SEC, shifted.back() + 5 * SEC, 50000);
  bool same = a.size() == b.size();
  for (size_t i = 0; same && i < a.size(); ++i)
    same = a[i].t + OFF == b[i].t && a[i].speedMmps == b[i].speedMmps && a[i].odometer == b[i].odometer;
  CHECK(same);
  CHECK(pulseSpeeds(pulses) == pulseSpeeds(shifted));
}

// ---------------------------------------------------------------- 1.15
// Console ratio: every F line strictly after the first M marker, >= 4 bytes,
// flags (uint16 LE, bytes 0-1) bit 0 clear (Instantaneous Speed present);
// console = uint16 LE bytes 2-3 (0.01 km/h); ours = speedMmps(t_F) with the
// re-filtered pulses (a pulse at t_F counts); sample kept iff both > 0;
// ratio = console x 100 / (ours x 36); median (even count: mean of the two
// middle values).
void testConsoleRatio() {
  replay::Log log;
  std::vector<int64_t> pulses;
  if (!CHECK(loadPulses("S2_walk", log, pulses)) || !CHECK(!log.markers.empty())) return;
  const int64_t m1 = log.markers.front();
  Metrics m;
  size_t i = 0;
  std::vector<double> r;
  for (const replay::FtmsLine& f : log.ftms) {
    if (f.t <= m1 || f.bytes.size() < 4) continue;
    const uint32_t flags = static_cast<uint32_t>(f.bytes[0] | (f.bytes[1] << 8));
    if (flags & 1u) continue;
    const uint32_t console = static_cast<uint32_t>(f.bytes[2] | (f.bytes[3] << 8));
    while (i < pulses.size() && pulses[i] <= f.t) m.onPulse(pulses[i++]);
    const int32_t ours = m.speedMmps(f.t);
    if (console == 0 || ours <= 0) continue;
    r.push_back(static_cast<double>(console) * 100.0 / (static_cast<double>(ours) * 36.0));
  }
  std::sort(r.begin(), r.end());
  if (!CHECK(!r.empty())) return;
  const size_t n = r.size();
  const double median = (n % 2) ? r[n / 2] : (r[n / 2 - 1] + r[n / 2]) / 2.0;
  std::printf("  1.15 console/ours median=%.6f samples=%zu\n", median, n);
  CHECK_NEAR(median, 1.143, 0.02);
  replay::Golden g;
  if (!CHECK(loadGoldenFile("console_ratio_S2_walk.txt", g))) return;
  CHECK_EQ(g.get("samples"), std::to_string(n));
  CHECK_NEAR(goldenNum(g, "median_ratio"), median, 1e-6);
}

// ------------------------------------------------------------ properties
// Direct restatement of the span definition over an unbounded history,
// with N limited to maxN (TS_RING - 1 models "the ring runs out").
std::vector<int32_t> refSpeeds(const std::vector<int64_t>& p, uint32_t maxN) {
  std::vector<int32_t> out;
  out.reserve(p.size());
  for (size_t k = 0; k < p.size(); ++k) {
    if (k == 0 || p[k] - p[k - 1] > ot::STOP_US) { out.push_back(0); continue; }
    size_t N = 1;
    for (;;) {
      if (p[k] - p[k - N] >= ot::SPAN_US) break;
      if (N == k || N == maxN) break;
      if (p[k - N] - p[k - N - 1] > ot::STOP_US) break;
      ++N;
    }
    const int64_t span = p[k] - p[k - N];
    out.push_back(span <= 0 ? 0 : ot::spanSpeedMmps(static_cast<uint32_t>(N), static_cast<uint64_t>(span)));
  }
  return out;
}

void testPropertyRandomPulses() {
  Rng rng(0x5EED0002ull);
  const int64_t S = ot::STOP_US;
  std::vector<int64_t> p;
  p.reserve(1000000);
  int64_t t = static_cast<int64_t>(rng.next() % 1000000u);
  for (int i = 0; i < 1000000; ++i) {
    const uint64_t kind = rng.next() % 1000u;
    int64_t gap;
    if (kind < 5) gap = 0;                                   // duplicate timestamp
    else if (kind < 8) gap = -rng.range(1, 100000);          // out of order (clamped)
    else if (kind < 40) gap = rng.range(1, 20000);           // faster than the lockout (sim)
    else if (kind < 500) gap = rng.range(50000, 600000);     // run/walk
    else if (kind < 850) gap = rng.range(600000, 1500000);   // slow walk
    else if (kind < 900) gap = rng.range(S - 2, S + 2);      // STOP boundary
    else if (kind < 990) gap = rng.range(S + 3, 20 * SEC);   // pauses
    else gap = rng.range(20 * SEC, 100000 * SEC);            // long idle
    t += gap;
    p.push_back(t);
  }
  // Reference timestamps after the documented clamp.
  std::vector<int64_t> clamped = p;
  for (size_t k = 1; k < clamped.size(); ++k) clamped[k] = std::max(clamped[k], clamped[k - 1]);
  const std::vector<int32_t> ref = refSpeeds(clamped, ot::TS_RING - 1u);

  Metrics m;
  long badRef = 0, badOdo = 0, badMono = 0, badStop = 0, badNeg = 0, badAtPulse = 0;
  for (size_t k = 0; k < p.size(); ++k) {
    m.onPulse(p[k]);
    if (m.odometerPulses() != static_cast<uint32_t>(k + 1) || m.lastPulseUs() != clamped[k]) ++badOdo;
    const int32_t v = m.speedAtLastPulseMmps();
    if (v != ref[k]) ++badRef;
    if (v < 0) ++badNeg;
    const int64_t tk = clamped[k];
    if (m.speedMmps(tk) != v) ++badAtPulse;
    // Queries between this pulse and the next, in increasing time order.
    const int64_t next = k + 1 < p.size() ? std::max(p[k + 1], tk) : tk + 100 * SEC;
    int64_t q[8] = {tk, tk + 1, tk + rng.range(0, S), tk + S - 1, tk + S, tk + S + 1, next - 1, next};
    std::sort(q, q + 8);
    int32_t prev = INT32_MAX;
    for (int64_t qt : q) {
      if (qt < tk || qt >= next) continue;  // stay strictly before the next pulse
      const int32_t s = m.speedMmps(qt);
      if (s > prev || s > v || s < 0) ++badMono;
      if (qt > tk && s > ot::decayCapMmps(static_cast<uint64_t>(qt - tk))) ++badMono;  // v <= M/(now - t_last)
      if (qt - tk > S && s != 0) ++badStop;
      prev = s;
    }
  }
  std::printf("  property: 1e6 pulses, last t=%lld us\n", static_cast<long long>(clamped.back()));
  CHECK_EQ(badRef, 0L);
  CHECK_EQ(badOdo, 0L);
  CHECK_EQ(badMono, 0L);
  CHECK_EQ(badStop, 0L);
  CHECK_EQ(badNeg, 0L);
  CHECK_EQ(badAtPulse, 0L);
}

}  // namespace

int main(int argc, char** argv) {
  gRoot = replay::repoRootFromArgs(argc, argv);
  testOdometer();
  testSpeedUnits();
  testFirstInterval();
  testSpanStopGap();
  testDecay();
  testSyntheticTrains();
  testUptime();
  testNoise();
  testWatchBias();
  testOracleAgreement();
  testConsoleRatio();
  testPropertyRandomPulses();
  return check::summary("test_metrics");
}
