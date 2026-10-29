// test_pulse_filter.cpp - host tests for PulseFilter and the replay parser:
// tests 1.1, 1.2, 1.3, 1.4, 1.14 (filter part), 1.16, plus parser unit tests
// and a seeded property test (~10^6 edges) against a declarative
// restatement of the pulse logger's PULSE RULE.
//
// Usage: test_pulse_filter --root <repo root>
#include <cstdio>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "pulse_filter.h"
#include "replay.h"

using ot::PulseFilter;

namespace {

std::string gRoot = ".";
const char* const kGoldenDir = "firmware/opentread/test/golden/";

struct Rng {  // splitmix64: deterministic, seeded
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

struct Ev { int64_t t; uint8_t level; };

std::vector<int64_t> runFilter(int64_t t0, uint8_t lvl0, const std::vector<Ev>& evs) {
  PulseFilter pf;
  pf.reset(t0, lvl0);
  std::vector<int64_t> out;
  for (const Ev& e : evs) if (pf.onEdge(e.t, e.level)) out.push_back(e.t);
  return out;
}

// ---------------------------------------------------------------- parser
void testParser() {
  const std::string text =
      "# tools/capture.py host_start=x session=unit\n"
      "ets Jul 29 2019 12:21:46\n"
      "rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\n"
      "\r\n"
      "S,100,M4 pulse logger v1 pin=27 lockout_us=20000 release_us=5000 baud=115200 line_level=0 ring=2048\r\n"
      "S,150,scan match: a,b,c\n"
      "E,300,0\n"
      "E,200,1\n"
      "E,abc,1\n"
      "E,400,2\n"
      "E,500\n"
      "E,600,1,9\n"
      "E,-5,1\n"
      "P,300,1,0\n"
      "P,x,1,0\n"
      "F,700,a614ff\n"
      "F,700,a61\n"
      "H,1000,2,3,4,0,0,1,1\n"
      "H,1000,2,3\n"
      "M,900,1\n"
      "K,800,-,0,0,0,0,0,0\n"
      "S,1100\n"
      "e,1,1\n"
      // The oracle's ASCII regexes reject all of these, so the
      // C++ must count them malformed too (no sign, E level one char, hex
      // lowercase, H fields unsigned, ASCII digits, '\n'-only line ends).
      "E,1000,01\n"
      "E,-0,1\n"
      "P,100,1,-0\n"
      "H,1,-1,-2,-3,-4,-5,-6,-7\n"
      "F,1,A6\n"
      "M,-0,1\n"
      "S,-0,hello\n"
      "E,\xd9\xa1\xd9\xa0\xd9\xa0,1\n"  // U+0661 U+0660 U+0660 (Arabic-Indic "100")
      "E,5,1\rE,6,1\n"                    // a lone CR does not end a line
      "E,0001250,1\n"                      // leading zeros are allowed (\d+)
      "E,1200,1";  // no trailing newline
  replay::Log log;
  replay::parseText(text, log);
  const replay::Stats& st = log.stats;
  CHECK_EQ(st.header, 1u);
  CHECK_EQ(st.blank, 1u);
  CHECK_EQ(st.garbage, 3u);    // ets..., rst:..., e,1,1
  CHECK_EQ(st.malformed, 18u);  // E,abc E,400,2 E,500 E,600,1,9 E,-5 P,x F,a61 H(short) S,1100 + 9 oracle-regex lines
  CHECK_EQ(st.malformedLines.size(), static_cast<size_t>(18));
  CHECK_EQ(st.other, 1u);      // K
  CHECK_EQ(st.e, 4u);
  CHECK_EQ(st.p, 1u);
  CHECK_EQ(st.f, 1u);
  CHECK_EQ(st.h, 1u);
  CHECK_EQ(st.m, 1u);
  CHECK_EQ(st.s, 2u);
  CHECK(st.outOfOrder >= 1u);
  CHECK_EQ(log.banners.size(), static_cast<size_t>(1));
  if (log.banners.size() == 1) {
    CHECK_EQ(log.banners[0].t, static_cast<int64_t>(100));
    CHECK_EQ(log.banners[0].lineLevel, static_cast<uint8_t>(0));
  }
  CHECK_EQ(log.status.size(), static_cast<size_t>(2));
  if (log.status.size() == 2) CHECK_EQ(log.status[1].text, std::string("scan match: a,b,c"));  // split at most twice
  CHECK_EQ(log.edges.size(), static_cast<size_t>(4));
  if (log.edges.size() == 4) {  // sorted by t
    CHECK_EQ(log.edges[0].t, static_cast<int64_t>(200));
    CHECK_EQ(log.edges[1].t, static_cast<int64_t>(300));
    CHECK_EQ(log.edges[2].t, static_cast<int64_t>(1200));
    CHECK_EQ(log.edges[3].t, static_cast<int64_t>(1250));
  }
  CHECK_EQ(log.ftms.size(), static_cast<size_t>(1));
  if (log.ftms.size() == 1) {
    CHECK_EQ(log.ftms[0].bytes.size(), static_cast<size_t>(3));
    if (log.ftms[0].bytes.size() == 3) CHECK_EQ(log.ftms[0].bytes[2], static_cast<uint8_t>(0xFF));
  }
  std::vector<int64_t> p;
  std::string err;
  CHECK(replay::refilter(log, ot::Config{}, p, err));

  // Two banners (a reboot inside the log) must be refused, not mixed.
  replay::Log two;
  replay::parseText("S,1,M4 pulse logger v1 line_level=1\nS,2,OpenTread fw=1.0.0 line_level=1 ring=1024\n", two);
  CHECK_EQ(two.banners.size(), static_cast<size_t>(2));
  CHECK(!replay::refilter(two, ot::Config{}, p, err));
  // A banner without line_level is malformed (never silently defaulted).
  replay::Log nolvl;
  replay::parseText("S,1,M4 pulse logger v1 pin=27\n", nolvl);
  CHECK_EQ(nolvl.stats.malformed, 1u);
  CHECK_EQ(nolvl.banners.size(), static_cast<size_t>(0));
}

// ------------------------------------------------- 1.1 + 1.16 (real logs)
void testRealLogs() {
  const uint32_t expected[4] = {428, 61, 164, 11};  // test 1.1
  for (int i = 0; i < 4; ++i) {
    const replay::NamedLog& nl = replay::kLogs[i];
    replay::Log log;
    std::string err;
    if (!CHECK(replay::loadFile(gRoot + "/" + nl.path, log, err))) { std::fprintf(stderr, "  %s\n", err.c_str()); continue; }
    std::printf("  %-12s lines=%u header=%u blank=%u garbage=%u malformed=%u other=%u out_of_order=%u E=%u P=%u F=%u M=%u S=%u H=%u\n",
                nl.name, log.stats.lines, log.stats.header, log.stats.blank, log.stats.garbage, log.stats.malformed,
                log.stats.other, log.stats.outOfOrder, log.stats.e, log.stats.p, log.stats.f, log.stats.m, log.stats.s,
                log.stats.h);
    CHECK_EQ(log.stats.malformed, 0u);
    CHECK_EQ(log.stats.header, 1u);
    CHECK_EQ(log.banners.size(), static_cast<size_t>(1));
    std::vector<int64_t> pulses;
    if (!CHECK(replay::refilter(log, ot::Config{}, pulses, err))) continue;

    // 1.1: identical P list, timestamps included.
    CHECK_EQ(static_cast<uint32_t>(pulses.size()), expected[i]);
    CHECK_EQ(pulses.size(), log.pulses.size());
    bool same = pulses.size() == log.pulses.size();
    for (size_t k = 0; same && k < pulses.size(); ++k) {
      same = pulses[k] == log.pulses[k].t && log.pulses[k].n == static_cast<uint32_t>(k + 1) &&
             log.pulses[k].dt == (k == 0 ? 0 : log.pulses[k].t - log.pulses[k - 1].t);
    }
    CHECK(same);
    if (std::string(nl.name) == "bench_jumper") CHECK_EQ(log.edges.size(), static_cast<size_t>(1193));

    // 1.16: the oracle's independent re-filter equals the C++.
    replay::Golden g;
    const std::string gp = gRoot + "/" + kGoldenDir + "filter_" + nl.name + ".txt";
    if (!CHECK(replay::loadGolden(gp, g, err))) { std::fprintf(stderr, "  %s\n", err.c_str()); continue; }
    CHECK_EQ(g.get("log"), std::string(nl.path));
    CHECK_EQ(g.get("edges"), std::to_string(log.edges.size()));
    CHECK_EQ(g.get("pulses"), std::to_string(pulses.size()));
    const std::vector<std::string> gp_t = g.all("p");
    CHECK_EQ(gp_t.size(), pulses.size());
    bool eq = gp_t.size() == pulses.size();
    for (size_t k = 0; eq && k < pulses.size(); ++k) eq = gp_t[k] == std::to_string(pulses[k]);
    CHECK(eq);

    // 1.14 (filter part): 10 days of uptime changes nothing but the offset.
    const int64_t OFF = 864000000000LL;  // 8.64e11 us
    replay::Log shifted = log;
    shifted.banners[0].t += OFF;
    for (replay::Edge& e : shifted.edges) e.t += OFF;
    std::vector<int64_t> p2;
    CHECK(replay::refilter(shifted, ot::Config{}, p2, err));
    bool shiftedSame = p2.size() == pulses.size();
    for (size_t k = 0; shiftedSame && k < p2.size(); ++k) shiftedSame = p2[k] == pulses[k] + OFF;
    CHECK(shiftedSame);
  }
}

// ------------------------------------------------------- 1.2 boot on magnet
void testBootLow() {
  // Line LOW at boot (t0 = 1000): chatter and short releases give nothing.
  std::vector<Ev> ev = {{2000, 0}, {9000, 1}, {13999, 0},   // release 4999 us: no
                        {50000, 0},                          // repeated closed reading: no
                        {60000, 1}, {65000, 0}};             // release exactly 5000 us: pulse
  std::vector<int64_t> p = runFilter(1000, 0, ev);
  CHECK_EQ(p.size(), static_cast<size_t>(1));
  if (p.size() == 1) CHECK_EQ(p[0], static_cast<int64_t>(65000));

  // Parked on the magnet for 10 s with vibration blips (< 5 ms open): nothing.
  std::vector<Ev> parked;
  for (int64_t t = 10000; t < 10000000; t += 37000) { parked.push_back({t, 1}); parked.push_back({t + 1500, 0}); }
  CHECK_EQ(runFilter(0, 0, parked).size(), static_cast<size_t>(0));

  // Boot HIGH: the boot state is an open pseudo-edge at t0 (exact logger port):
  // a closure 4999 us after t0 is NOT accepted, 5000 us after t0 is.
  CHECK_EQ(runFilter(1000, 1, {{5999, 0}}).size(), static_cast<size_t>(0));
  CHECK_EQ(runFilter(1000, 1, {{6000, 0}}).size(), static_cast<size_t>(1));
}

// ---------------------------------------------------- 1.3 bounce torture
void testBounceTorture() {
  const int64_t periods[] = {200000, 86000, 61000};  // 2.6, 25, 35 km/h
  for (int64_t period : periods) {
    std::vector<Ev> ev;
    std::vector<int64_t> closures;
    int64_t T = 100000;
    for (int c = 0; c < 200; ++c, T += period) {
      closures.push_back(T);
      ev.push_back({T, 0});  // closure + 6 bounce edges within 3 ms, ends closed
      for (int k = 1; k <= 6; ++k) ev.push_back({T + 500 * k, static_cast<uint8_t>(k % 2)});
      // vibration blip while the magnet dwells (open < 5 ms)
      ev.push_back({T + 8000, 1});
      ev.push_back({T + 9000, 0});
      // release at T + 15 ms with 4 chatter edges within 2 ms, ends open
      const int64_t R = T + 15000;
      ev.push_back({R, 1});
      for (int k = 1; k <= 4; ++k) ev.push_back({R + 500 * k, static_cast<uint8_t>((k + 1) % 2)});
    }
    const std::vector<int64_t> p = runFilter(0, 1, ev);
    CHECK_EQ(p.size(), closures.size());
    CHECK(p == closures);
  }
}

// ----------------------------------------------------- 1.4 lockout boundary
void testLockoutBoundary() {
  // Accept at 10000 (open since t0 = 0), valid release at 15000 (>= 5 ms
  // before the next closure), then the next closure 19.999 / 20.000 ms after
  // the accept.
  CHECK_EQ(runFilter(0, 1, {{10000, 0}, {15000, 1}, {29999, 0}}).size(), static_cast<size_t>(1));
  const std::vector<int64_t> ok = runFilter(0, 1, {{10000, 0}, {15000, 1}, {30000, 0}});
  CHECK_EQ(ok.size(), static_cast<size_t>(2));
  if (ok.size() == 2) CHECK_EQ(ok[1], static_cast<int64_t>(30000));
  // Release boundary: 4999 us open rejects, 5000 us accepts.
  CHECK_EQ(runFilter(0, 1, {{10000, 0}, {100000, 1}, {104999, 0}}).size(), static_cast<size_t>(1));
  CHECK_EQ(runFilter(0, 1, {{10000, 0}, {100000, 1}, {105000, 0}}).size(), static_cast<size_t>(2));
  // A rejected (locked-out) closure still disarms: without a new release the
  // closure at 40 ms is not accepted even though the lockout has expired.
  CHECK_EQ(runFilter(0, 1, {{10000, 0}, {15000, 1}, {29999, 0}, {40000, 0}}).size(), static_cast<size_t>(1));
}

// --------------------------------------- property: declarative PULSE RULE
void testPropertyRandomEdges() {
  Rng rng(0x5EED0001ull);
  const int64_t R = ot::RELEASE_US, L = ot::LOCKOUT_US;
  for (int run = 0; run < 4; ++run) {
    const uint8_t lvl0 = static_cast<uint8_t>(rng.next() & 1u);
    const int64_t t0 = run == 3 ? 864000000000LL : static_cast<int64_t>(rng.next() % 1000000u);
    std::vector<Ev> ev;
    ev.reserve(250000);
    int64_t t = t0;
    uint8_t lvl = lvl0;
    for (int i = 0; i < 250000; ++i) {
      const uint64_t kind = rng.next() % 100u;
      int64_t gap;
      if (kind < 40) gap = rng.range(1, 3000);              // bounce
      else if (kind < 60) gap = rng.range(R - 3, R + 3);    // release boundary
      else if (kind < 75) gap = rng.range(3000, 25000);     // around the lockout
      else if (kind < 80) gap = rng.range(L - 3, L + 3);    // lockout boundary
      else if (kind < 97) gap = rng.range(25000, 2000000);  // real gaps
      else gap = 0;                                          // same-timestamp edge
      t += gap;
      lvl = (rng.next() % 10u) < 8u ? static_cast<uint8_t>(lvl ^ 1u) : lvl;  // occasional repeat level
      ev.push_back({t, lvl});
    }
    // Implementation under test.
    PulseFilter pf;
    pf.reset(t0, lvl0);
    std::vector<int64_t> got;
    std::vector<bool> acc(ev.size(), false);
    for (size_t i = 0; i < ev.size(); ++i) {
      acc[i] = pf.onEdge(ev[i].t, ev[i].level);
      if (acc[i]) got.push_back(ev[i].t);
    }
    // Declarative reference: closure i is accepted iff some open edge j after
    // the previous closure (or the boot state if there was none) was followed
    // by >= R of quiet (next edge at or before i), and i is >= L after the
    // previous accepted closure.
    long mismatches = 0;
    long prev0 = -1;  // index of the previous level-0 edge
    bool haveAcc = false;
    int64_t lastAcc = 0;
    for (size_t i = 0; i < ev.size(); ++i) {
      if (ev[i].level != 0) continue;
      bool open = false;
      if (prev0 < 0 && lvl0 == 1) {
        const int64_t nextT = ev[0].t;  // the edge after the boot pseudo-edge
        if (nextT - t0 >= R) open = true;
      }
      for (size_t j = static_cast<size_t>(prev0 + 1); !open && j < i; ++j) {
        if (ev[j].level == 1 && ev[j + 1].t - ev[j].t >= R) open = true;
      }
      const bool expect = open && (!haveAcc || ev[i].t - lastAcc >= L);
      if (expect) { haveAcc = true; lastAcc = ev[i].t; }
      if (expect != acc[i]) ++mismatches;
      prev0 = static_cast<long>(i);
    }
    for (size_t i = 0; i < ev.size(); ++i) if (ev[i].level != 0 && acc[i]) ++mismatches;  // only closures
    CHECK_EQ(mismatches, 0L);
    bool spaced = true;
    for (size_t k = 1; k < got.size(); ++k) spaced = spaced && got[k] - got[k - 1] >= L;
    CHECK(spaced);
    CHECK_EQ(static_cast<size_t>(pf.acceptedCount()), got.size());
    CHECK(got.size() > 1000u);  // the property is not vacuous
  }
}

}  // namespace

int main(int argc, char** argv) {
  gRoot = replay::repoRootFromArgs(argc, argv);
  testParser();
  testRealLogs();
  testBootLow();
  testBounceTorture();
  testLockoutBoundary();
  testPropertyRandomEdges();
  return check::summary("test_pulse_filter");
}
