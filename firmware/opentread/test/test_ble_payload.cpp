// test_ble_payload.cpp - host tests for src/core/ble_payload
// (tests 6.1-6.4, 7.1-7.7), plus:
//   6.5  the RSC speed AS SENT (1/256 m/s) integrated at 1 Hz over S2 / S1b:
//        the watch-integration bias through the wire format (characterisation
//        + the ±1 % bound of test 1.11);
//   7.8  a seeded random-operation property test of LinkTable (token,
//        deferral, CCCD, grant, slot reuse) against an independent model.
//
// Byte vectors are stated in the encoders' real inputs. Floating point appears only in the
// 6.5 report and its tolerance checks.
//
// Usage: test_ble_payload --root <repo root>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "ble_payload.h"
#include "check.h"
#include "config.h"
#include "fixed.h"
#include "format.h"
#include "metrics.h"
#include "replay.h"
#include "workout.h"

namespace ble = ot::ble;
using ot::WorkoutEvent;
using ot::WorkoutEventType;
using ot::WorkoutSnapshot;
using ot::WorkoutState;
using replay::InputKind;
using replay::WorkoutInput;

namespace {

std::string gRoot = ".";
constexpr int64_t SEC = 1000000;
constexpr int64_t MS = 1000;
const char* const kS2 = "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv";

using Bytes = std::vector<uint8_t>;

Bytes hex(const char* s) {  // "06 c2 03" -> bytes (spaces ignored)
  Bytes b;
  int hi = -1;
  for (const char* p = s; *p; ++p) {
    int v = -1;
    if (*p >= '0' && *p <= '9') v = *p - '0';
    else if (*p >= 'a' && *p <= 'f') v = *p - 'a' + 10;
    else if (*p >= 'A' && *p <= 'F') v = *p - 'A' + 10;
    else continue;
    if (hi < 0) hi = v;
    else {
      b.push_back(static_cast<uint8_t>(hi * 16 + v));
      hi = -1;
    }
  }
  return b;
}
std::string hexs(const uint8_t* d, size_t n) {
  char buf[128];
  ble::toHex(d, n, buf, sizeof(buf));
  return buf;
}
std::string hexs(const Bytes& b) { return hexs(b.data(), b.size()); }

Bytes rsc(int32_t mmps, uint32_t pulses) {
  uint8_t o[ble::RSC_MEAS_LEN];
  std::memset(o, 0xAA, sizeof(o));
  ble::encodeRscMeasurement(mmps, pulses, o);
  return Bytes(o, o + sizeof(o));
}
Bytes td(int32_t mmps, uint32_t wkPulses, int64_t totalUs) {
  uint8_t o[ble::TD_LEN];
  std::memset(o, 0xAA, sizeof(o));
  ble::encodeTreadmillData(mmps, wkPulses, totalUs, o);
  return Bytes(o, o + sizeof(o));
}
uint32_t u16(const Bytes& b, size_t i) { return static_cast<uint32_t>(b[i]) | (static_cast<uint32_t>(b[i + 1]) << 8); }
uint32_t u24(const Bytes& b, size_t i) { return u16(b, i) | (static_cast<uint32_t>(b[i + 2]) << 16); }
uint32_t u32(const Bytes& b, size_t i) { return u24(b, i) | (static_cast<uint32_t>(b[i + 3]) << 24); }

bool loadLog(const char* path, replay::Log& log) {
  std::string err;
  if (!replay::loadFile(gRoot + "/" + path, log, err)) {
    std::fprintf(stderr, "  %s\n", err.c_str());
    return false;
  }
  return true;
}

WorkoutInput hold(InputKind k, int64_t t) { return WorkoutInput{t, k, std::string()}; }

// ------------------------------------------------------------------ 6.1
void test61() {
  // Reference vectors (encoder inputs (mmps, odometerPulses)).
  CHECK_EQ(hexs(rsc(3756, 20666)), hexs(hex("06 c2 03 00 42 e2 01 00")));
  CHECK_EQ(hexs(rsc(3755, 20665)), hexs(hex("06 c1 03 00 3c e2 01 00")));
  CHECK_EQ(hexs(rsc(2222, 0)), hexs(hex("06 39 02 00 00 00 00 00")));
  for (int32_t v = 2221; v <= 2224; ++v) CHECK_EQ(u16(rsc(v, 0), 1), 569u);  // 8.0 km/h -> 569
  CHECK_EQ(u16(rsc(2220, 0), 1), 568u);
  CHECK_EQ(u16(rsc(2225, 0), 1), 570u);
  // idle (0, p) -> 02 00 00 00 + floor(p x 5.974) dm LE
  for (uint32_t p : {0u, 1u, 2u, 1674u, 20665u, 4000000u}) {
    const Bytes b = rsc(0, p);
    CHECK_EQ(b[0], 0x02);
    CHECK_EQ(u16(b, 1), 0u);
    CHECK_EQ(b[3], 0x00);
    CHECK_EQ(static_cast<uint64_t>(u32(b, 4)), static_cast<uint64_t>(p) * 597400u / 100000u);
  }
  CHECK_EQ(u32(rsc(0, 1), 4), 5u);
  CHECK_EQ(u32(rsc(0, 2), 4), 11u);
  CHECK_EQ(u32(rsc(0, 20665), 4), 123452u);
  // running flag on the UNROUNDED mm/s: 1943 -> 02, 1944 -> 06
  CHECK_EQ(rsc(1943, 0)[0], 0x02);
  CHECK_EQ(rsc(1944, 0)[0], 0x06);
  CHECK_EQ(u16(rsc(1943, 0), 1), 497u);  // both round to ~1.94 m/s: the flag doesn't follow the rounded speed
  CHECK_EQ(u16(rsc(1944, 0), 1), 498u);
  // 0 / negative speed -> 00 00, flag 02
  CHECK_EQ(hexs(rsc(0, 0)), std::string("0200000000000000"));
  CHECK_EQ(hexs(rsc(-5, 0)), std::string("0200000000000000"));
  // speed clamp at u16: 255.998 m/s = 255998 mm/s -> 65535; INT32_MAX too
  CHECK_EQ(u16(rsc(255998, 0), 1), 65535u);
  CHECK_EQ(u16(rsc(255997, 0), 1), 65535u);  // (255997*256+500)/1000 = 65535.7 -> 65535
  CHECK_EQ(u16(rsc(255996, 0), 1), 65535u);
  CHECK_EQ(u16(rsc(255994, 0), 1), 65534u);
  CHECK_EQ(u16(rsc(INT32_MAX, 0), 1), 65535u);
  CHECK_EQ(u16(rsc(300000, 0), 1), 65535u);
  // distance clamp at u32 (only after ~429,000 km)
  CHECK_EQ(u32(rsc(0, UINT32_MAX), 4), 0xFFFFFFFFu);
  const uint32_t pMax = static_cast<uint32_t>(0xFFFFFFFFull * 100000ull / 597400ull);  // floor: last unclamped
  CHECK_EQ(static_cast<uint64_t>(u32(rsc(0, pMax), 4)), static_cast<uint64_t>(pMax) * 597400u / 100000u);
  CHECK_EQ(u32(rsc(0, pMax + 1), 4), 0xFFFFFFFFu);
  // cadence always 0
  for (int32_t v : {0, 500, 1944, 5000, 99999}) CHECK_EQ(rsc(v, 7)[3], 0x00);
}

// ------------------------------------------------------------------ 6.2
// RSC distance over the whole S2 replay sampled at 1 Hz (and at every W
// event): = floor(odometer x 5.974) dm, monotonic, unchanged by state changes.
void test62() {
  replay::Log log;
  if (!CHECK(loadLog(kS2, log))) return;
  std::vector<WorkoutInput> in = replay::workoutInputs(log);
  replay::mergeInputs(in, {hold(InputKind::HoldBegin, 1105 * SEC), hold(InputKind::HoldComplete, 1107 * SEC)});
  replay::WorkoutReplay r;
  size_t next = 0;
  uint32_t prevDm = 0;
  size_t samples = 0, evChecks = 0, seenEvents = 0;
  bool mono = true, exact = true, stable = true;
  for (int64_t t = in.front().t - 2 * SEC; t <= in.back().t + 1200 * SEC; t += SEC) {
    while (next < in.size() && in[next].t <= t) r.feed(in[next++]);
    r.tickTo(t);
    for (; seenEvents < r.trace.events.size(); ++seenEvents) ++evChecks;  // W events seen
    const Bytes b = rsc(r.metrics.speedMmps(t), r.metrics.odometerPulses());
    const uint32_t dm = u32(b, 4);
    exact = exact && dm == static_cast<uint32_t>(ot::rscDistanceDm(r.metrics.odometerPulses()));
    mono = mono && dm >= prevDm;
    prevDm = dm;
    ++samples;
  }
  // State changes without pulses (pause, auto-end, summary timeout, re-arm):
  // replay again and compare the encoded distance right before and right at
  // each time-based transition stamp.
  {
    replay::WorkoutReplay q;
    size_t nx = 0;
    std::vector<int64_t> stamps;
    for (const WorkoutEvent& e : r.trace.events)
      if (e.type == WorkoutEventType::Paused || e.type == WorkoutEventType::AutoEnded ||
          e.type == WorkoutEventType::SummaryCleared || e.type == WorkoutEventType::Rearmed ||
          e.type == WorkoutEventType::Stopped || e.type == WorkoutEventType::WarmupFailed)
        stamps.push_back(e.t);
    std::sort(stamps.begin(), stamps.end());
    CHECK(stamps.size() >= 8u);
    for (int64_t s : stamps) {
      while (nx < in.size() && in[nx].t < s) q.feed(in[nx++]);
      q.tickTo(s - 1);
      const uint32_t before = u32(rsc(q.metrics.speedMmps(s - 1), q.metrics.odometerPulses()), 4);
      uint32_t atPulses = 0;
      while (nx < in.size() && in[nx].t <= s) {
        if (in[nx].kind == InputKind::Pulse) ++atPulses;
        q.feed(in[nx++]);
      }
      q.tickTo(s);
      const uint32_t after = u32(rsc(q.metrics.speedMmps(s), q.metrics.odometerPulses()), 4);
      if (atPulses == 0) stable = stable && before == after;
    }
  }
  CHECK(exact);
  CHECK(mono);
  CHECK(stable);
  CHECK(samples > 2000u);
  CHECK(evChecks > 10u);
  CHECK_EQ(prevDm, static_cast<uint32_t>(ot::rscDistanceDm(static_cast<uint32_t>(log.pulses.size()))));
  std::printf("  6.2 S2: %zu samples, final RSC distance %u dm (%zu P), %zu W events\n", samples, prevDm,
              log.pulses.size(), evChecks);
}

// ------------------------------------------------------------------ 6.3 / 7.5
bool hasAd(const ble::AdvPacket& p, uint8_t type) {
  for (size_t i = 0; i < p.len;) {
    const size_t l = p.b[i];
    if (l == 0 || i + 1 + l > p.len) return false;
    if (p.b[i + 1] == type) return true;
    i += 1 + l;
  }
  return false;
}
bool wellFormed(const ble::AdvPacket& p) {
  size_t i = 0;
  while (i < p.len) {
    const size_t l = p.b[i];
    if (l == 0 || i + 1 + l > p.len) return false;
    i += 1 + l;
  }
  return i == p.len;
}
bool contains1826(const ble::AdvPacket& p) {
  for (size_t i = 0; i + 1 < p.len; ++i)
    if (p.b[i] == 0x26 && p.b[i + 1] == 0x18) return true;
  return false;
}
Bytes pk(const ble::AdvPacket& p) { return Bytes(p.b, p.b + p.len); }

void test63_75() {
  const Bytes name = hex("4f 70 65 6e 54 72 65 61 64");  // "OpenTread"
  Bytes layoutB = hex("02 01 06 0a 09");
  layoutB.insert(layoutB.end(), name.begin(), name.end());
  for (uint8_t x : hex("03 03 14 18 03 19 40 04")) layoutB.push_back(x);
  CHECK_EQ(layoutB.size(), static_cast<size_t>(22));

  ble::AdvPair p;
  // 6.3: Layout B = the 23 bytes exactly, no scan response
  CHECK(ble::buildAdvert(ble::AdvLayout::B, ble::BLE_NAME, p));
  CHECK_EQ(hexs(pk(p.primary)), hexs(layoutB));
  CHECK_EQ(p.scanResponse.len, 0);
  CHECK(!contains1826(p.primary));
  CHECK(!ble::advMentionsFtms(ble::AdvLayout::B));
  // the name is in the PRIMARY packet (a scan-response-only name can go missing), complete-name AD 0x09
  CHECK(hasAd(p.primary, 0x09));
  // 7.5: B' = B primary byte-identical + scan response 06 16 26 18 01 01 00, no 0x03 list there
  CHECK(ble::buildAdvert(ble::AdvLayout::BPrime, ble::BLE_NAME, p));
  CHECK_EQ(hexs(pk(p.primary)), hexs(layoutB));
  CHECK_EQ(hexs(pk(p.scanResponse)), std::string("06162618010100"));
  CHECK_EQ(p.scanResponse.len, 7);
  CHECK(!hasAd(p.scanResponse, 0x03));
  CHECK(!hasAd(p.scanResponse, 0x02));
  // B'': both packets use Incomplete lists (0x02)
  CHECK(ble::buildAdvert(ble::AdvLayout::BDouble, ble::BLE_NAME, p));
  Bytes bpp = layoutB;
  bpp[15] = 0x02;  // 03 03 14 18 -> 03 02 14 18
  CHECK_EQ(hexs(pk(p.primary)), hexs(bpp));
  CHECK_EQ(hexs(pk(p.scanResponse)), std::string("030226180616261801010" "0"));
  CHECK(!hasAd(p.primary, 0x03) && !hasAd(p.scanResponse, 0x03));
  // Layout A: 28 B primary, appearance in the scan response
  CHECK(ble::buildAdvert(ble::AdvLayout::A, ble::BLE_NAME, p));
  Bytes a = hex("02 01 06 0a 09");
  a.insert(a.end(), name.begin(), name.end());
  for (uint8_t x : hex("05 03 14 18 26 18 06 16 26 18 01 01 00")) a.push_back(x);
  CHECK_EQ(a.size(), static_cast<size_t>(27));
  CHECK_EQ(hexs(pk(p.primary)), hexs(a));
  CHECK_EQ(hexs(pk(p.scanResponse)), std::string("03194004"));
  // all layouts well formed, <= 31 B, flags first
  for (ble::AdvLayout l : {ble::AdvLayout::B, ble::AdvLayout::A, ble::AdvLayout::BPrime, ble::AdvLayout::BDouble}) {
    CHECK(ble::buildAdvert(l, ble::BLE_NAME, p));
    CHECK(wellFormed(p.primary) && wellFormed(p.scanResponse));
    CHECK(p.primary.len <= 31 && p.scanResponse.len <= 31);
    CHECK_EQ(hexs(Bytes(p.primary.b, p.primary.b + 3)), std::string("020106"));
    CHECK_EQ(ble::advMentionsFtms(l), contains1826(p.primary) || contains1826(p.scanResponse));
    // never two COMPLETE lists across the pair
    CHECK(!(hasAd(p.primary, 0x03) && hasAd(p.scanResponse, 0x03)));
  }
  // > 31 B is refused (a name change): Layout A fits names up to 13 chars,
  // B/B'/B'' up to 18.
  CHECK(ble::buildAdvert(ble::AdvLayout::A, "OpenTreadX123", p));
  CHECK_EQ(p.primary.len, 31);
  CHECK(!ble::buildAdvert(ble::AdvLayout::A, "OpenTreadX1234", p));
  CHECK_EQ(p.primary.len, 0);  // nothing half-built is returned
  CHECK(ble::buildAdvert(ble::AdvLayout::B, "OpenTreadX12345678", p));
  CHECK_EQ(p.primary.len, 31);
  CHECK(!ble::buildAdvert(ble::AdvLayout::B, "OpenTreadX123456789", p));
  CHECK(!ble::buildAdvert(ble::AdvLayout::BPrime, "OpenTreadX123456789", p));
  CHECK(!ble::buildAdvert(ble::AdvLayout::BDouble, "OpenTreadX123456789", p));
  CHECK(!ble::buildAdvert(ble::AdvLayout::B, "", p));
  CHECK(!ble::buildAdvert(ble::AdvLayout::B, nullptr, p));
  CHECK(!ble::buildAdvert(static_cast<ble::AdvLayout>(9), ble::BLE_NAME, p));
  // AdvPacket::add refuses a structure that would cross 31 B and keeps the packet
  ble::AdvPacket q;
  const uint8_t z[29] = {};
  CHECK(q.add(0xFF, z, 29));
  CHECK_EQ(q.len, 31);
  CHECK(!q.add(0x01, z, 0));
  CHECK_EQ(q.len, 31);
  ble::AdvPacket q2;
  CHECK(!q2.add(0xFF, z, 30));
  CHECK_EQ(q2.len, 0);
  CHECK_EQ(std::string(ble::advLayoutName(ble::AdvLayout::A)), std::string("A"));
  CHECK_EQ(std::string(ble::advLayoutName(ble::AdvLayout::BPrime)), std::string("Bp"));
  CHECK_EQ(std::string(ble::advLayoutName(ble::AdvLayout::BDouble)), std::string("Bpp"));
  CHECK_EQ(std::string(ble::advLayoutName(ble::AdvLayout::B)), std::string("B"));
}

// ------------------------------------------------------------------ 6.4 / 7.6
void test64_76() {
  CHECK_EQ(hexs(ble::RSC_FEATURE, 2), std::string("0600"));
  CHECK_EQ(ble::SENSOR_LOCATION, 0x00);
  CHECK_EQ(hexs(ble::FTMS_FEATURE, 8), std::string("0410000000000000"));
  CHECK_EQ(ble::TD_FLAGS, 0x0404);
  CHECK_EQ(ble::APPEARANCE, 0x0440);
  CHECK_EQ(std::string(ble::BLE_NAME), std::string("OpenTread"));
  CHECK_EQ(ble::BLE_NAME_LEN, static_cast<size_t>(9));
  CHECK(std::string(ble::DIS_MANUFACTURER).find("Assault") == std::string::npos);  // never the stock identity
  CHECK(std::string(ble::DIS_MODEL) == "OpenTread head unit");
  CHECK_EQ(ble::NOTIFY_PERIOD_US, static_cast<int64_t>(1000000));
  CHECK_EQ(ble::WALK_RUN_MMPS, 1944);
  CHECK_EQ(ble::CP_IND_DEFER_US, static_cast<int64_t>(5000));
  CHECK_EQ(ble::CP_IND_TIMEOUT_US, static_cast<int64_t>(35000000));
  CHECK_EQ(ble::MAX_LINKS, static_cast<size_t>(3));
}

// ------------------------------------------------------------------ 6.5
// The FR955 integrates the footpod's instantaneous speed.
// Test 1.11 characterised that bias on the mm/s estimate; here the same
// 10-phase 1 Hz sampling integrates the speed AS SENT (1/256 m/s).
void test65() {
  for (const char* name : {"S2_walk", "S1b_10rev"}) {
    const replay::NamedLog* nl = nullptr;
    for (const replay::NamedLog& x : replay::kLogs)
      if (std::string(x.name) == name) nl = &x;
    if (!CHECK(nl != nullptr)) continue;
    replay::Log log;
    std::vector<int64_t> pulses;
    std::string err;
    if (!CHECK(replay::loadFile(gRoot + "/" + nl->path, log, err) && replay::refilter(log, ot::Config{}, pulses, err)))
      continue;
    const int64_t truthUm = static_cast<int64_t>(pulses.size()) * static_cast<int64_t>(ot::M_PER_PULSE_UM);
    const double truthM = static_cast<double>(truthUm) / 1e6;
    double minE = 1e18, maxE = -1e18, sumE = 0, maxWireVsMm = 0;
    for (int p = 0; p < 10; ++p) {
      const int64_t start = pulses.front() - 2 * SEC + p * 100000;
      const std::vector<replay::Sample> s = replay::sampleMetrics(pulses, start, pulses.back() + 5 * SEC, SEC);
      int64_t wire256 = 0, mm = 0;  // sum of 1/256 m/s x 1 s, and of mm/s x 1 s
      uint32_t prevDm = 0;
      bool mono = true;
      for (const replay::Sample& x : s) {
        const Bytes b = rsc(x.speedMmps, x.odometer);
        wire256 += static_cast<int64_t>(u16(b, 1));
        mm += x.speedMmps;
        mono = mono && u32(b, 4) >= prevDm;
        prevDm = u32(b, 4);
      }
      CHECK(mono);
      const double wireM = static_cast<double>(wire256) / 256.0;
      const double mmM = static_cast<double>(mm) / 1000.0;
      const double e = wireM - truthM;
      minE = std::min(minE, e);
      maxE = std::max(maxE, e);
      sumE += e;
      maxWireVsMm = std::max(maxWireVsMm, std::fabs(wireM - mmM));
      CHECK(std::fabs(e) <= 0.01 * truthM);  // test 1.11 bound: every phase within ±1 %
      // quantisation bound: |round(v*0.256)/256 - v/1000| <= 0.5/256 m/s per sample
      CHECK(std::fabs(wireM - mmM) <= static_cast<double>(s.size()) * (0.5 / 256.0) + 1e-9);
    }
    std::printf("  6.5 %-9s truth=%.3f m  wire(1/256 m/s) 1 Hz phases min=%+.3f max=%+.3f mean=%+.3f m"
                "  |wire - mm/s| <= %.3f m\n",
                name, truthM, minE, maxE, sumE / 10.0, maxWireVsMm);
  }
}

// ------------------------------------------------------------------ 7.1
void test71() {
  CHECK_EQ(hexs(td(3756, 3869, 1145000000)), hexs(hex("04 04 48 05 07 09 00 79 04")));
  CHECK_EQ(hexs(td(0, 0, 0)), hexs(hex("04 04 00 00 00 00 00 00 00")));
  // speed (0.01 km/h): (mmps*36+50)/100
  CHECK_EQ(u16(td(2222, 0, 0), 2), 800u);
  CHECK_EQ(u16(td(1, 0, 0), 2), 0u);
  CHECK_EQ(u16(td(2, 0, 0), 2), 1u);  // 0.072 -> 0.0? (72+50)/100 = 1
  CHECK_EQ(u16(td(-100, 0, 0), 2), 0u);
  // speed clamp u16: 655.35 km/h = 182041.6 mm/s
  CHECK_EQ(u16(td(182040, 0, 0), 2), 65534u);  // (6553440+50)/100 = 65534
  CHECK_EQ(u16(td(182042, 0, 0), 2), 65535u);
  CHECK_EQ(u16(td(182043, 0, 0), 2), 65535u);  // 65535.98 -> clamp
  CHECK_EQ(u16(td(INT32_MAX, 0, 0), 2), 65535u);
  // distance u24 m = floor(wk x 0.5974), clamp at 16,777,215
  CHECK_EQ(u24(td(0, 1, 0), 4), 0u);
  CHECK_EQ(u24(td(0, 2, 0), 4), 1u);
  CHECK_EQ(u24(td(0, 1674, 0), 4), 1000u);
  CHECK_EQ(u24(td(0, 20665, 0), 4), 12345u);
  const uint32_t pLast = static_cast<uint32_t>((16777216ull * 1000000ull - 1) / 597400ull);  // floor(..)=16777215
  CHECK_EQ(u24(td(0, pLast, 0), 4), 16777215u);
  CHECK_EQ(static_cast<uint64_t>(u24(td(0, pLast - 2, 0), 4)), static_cast<uint64_t>(pLast - 2) * 597400u / 1000000u);
  CHECK_EQ(u24(td(0, pLast + 1, 0), 4), 16777215u);
  CHECK_EQ(u24(td(0, UINT32_MAX, 0), 4), 16777215u);
  // elapsed u16 s = floor(totalUs/1e6), clamp 65535, negative -> 0
  CHECK_EQ(u16(td(0, 0, 999999), 7), 0u);
  CHECK_EQ(u16(td(0, 0, 1000000), 7), 1u);
  CHECK_EQ(u16(td(0, 0, 65535999999), 7), 65535u);
  CHECK_EQ(u16(td(0, 0, 65536000000), 7), 65535u);
  CHECK_EQ(u16(td(0, 0, INT64_MAX), 7), 65535u);
  CHECK_EQ(u16(td(0, 0, -5), 7), 0u);
  // flags never change: no sentinel / optional fields
  for (int32_t v : {0, 1, 5000, INT32_MAX}) CHECK_EQ(u16(td(v, 5, 5), 0), 0x0404u);
}

// ------------------------------------------------------------------ 7.2
// Per-session semantics over the S2 replay (with the 2.12 hold inputs, and
// plain S2 ending by auto-end), sampled every 100 ms and after every input.
struct SessionMonitor {
  uint32_t lastId = 0;
  uint32_t lastDist = 0, lastEl = 0;
  uint64_t violations = 0, samples = 0, idleNonZero = 0, screenMismatch = 0, summaryMismatch = 0;
  uint64_t runningSamples = 0, summarySamples = 0, idleSamples = 0;
  void sample(const WorkoutSnapshot& ws, const ot::SummaryRecord* z) {
    ++samples;
    const Bytes b = td(1234, ws.wkPulses, ws.totalUs);
    const uint32_t dist = u24(b, 4), el = u16(b, 7);
    switch (ws.state) {
      case WorkoutState::Idle:
      case WorkoutState::Warmup:
        ++idleSamples;
        if (dist != 0 || el != 0) ++idleNonZero;
        break;
      case WorkoutState::Running:
      case WorkoutState::Paused: {
        ++runningSamples;
        // the screen shows the same snapshot: distance hundredths (km) and the
        // total's whole seconds agree exactly with the FTMS fields
        char a[32], c[32];
        if (ot::distanceHundredths(ws.wkPulses, ot::Units::Km) != dist / 10u) ++screenMismatch;
        ot::fmtDuration(a, sizeof(a), ws.totalUs);
        ot::fmtDuration(c, sizeof(c), static_cast<int64_t>(el) * 1000000);
        if (std::strcmp(a, c) != 0) ++screenMismatch;
        break;
      }
      case WorkoutState::Summary:
        ++summarySamples;
        if (z == nullptr || dist != static_cast<uint32_t>(ot::ftmsDistanceM(z->pulses)) ||
            el != static_cast<uint32_t>(z->totalUs / 1000000))
          ++summaryMismatch;
        break;
    }
    if (ws.workoutId != 0) {
      if (ws.workoutId == lastId && (dist < lastDist || el < lastEl)) ++violations;
      lastId = ws.workoutId;
      lastDist = dist;
      lastEl = el;
    }
  }
};

void run72(const char* label, std::vector<WorkoutInput> in, uint32_t expectWorkouts) {
  replay::WorkoutReplay r;
  SessionMonitor mon;
  size_t next = 0;
  const int64_t end = in.back().t + 1200 * SEC;
  // 0 before the confirm: the first samples are IDLE/WARMUP
  for (int64_t t = in.front().t - SEC; t <= end; t += 100 * MS) {
    while (next < in.size() && in[next].t <= t) {
      r.feed(in[next]);
      mon.sample(r.workout.snapshot(in[next].t), r.workout.summary());
      ++next;
    }
    r.tickTo(t);
    mon.sample(r.workout.snapshot(t), r.workout.summary());
  }
  CHECK_EQ(mon.violations, static_cast<uint64_t>(0));
  CHECK_EQ(mon.idleNonZero, static_cast<uint64_t>(0));
  CHECK_EQ(mon.screenMismatch, static_cast<uint64_t>(0));
  CHECK_EQ(mon.summaryMismatch, static_cast<uint64_t>(0));
  CHECK(mon.runningSamples > 1000u && mon.summarySamples > 100u && mon.idleSamples > 100u);
  CHECK_EQ(r.trace.workouts, expectWorkouts);
  CHECK(r.workout.state() == WorkoutState::Idle);  // after summary_timeout: 0 again
  const WorkoutSnapshot last = r.workout.snapshot(end);
  CHECK_EQ(hexs(td(0, last.wkPulses, last.totalUs)), std::string("040400000000000000"));
  std::printf("  7.2 %-14s %llu samples (running %llu, summary %llu, idle/warmup %llu), 0 decreases\n", label,
              static_cast<unsigned long long>(mon.samples), static_cast<unsigned long long>(mon.runningSamples),
              static_cast<unsigned long long>(mon.summarySamples), static_cast<unsigned long long>(mon.idleSamples));
}

void test72() {
  replay::Log log;
  if (!CHECK(loadLog(kS2, log))) return;
  std::vector<WorkoutInput> in = replay::workoutInputs(log);
  run72("S2_walk", in, 1);  // auto-end
  replay::mergeInputs(in, {hold(InputKind::HoldBegin, 1105 * SEC), hold(InputKind::HoldComplete, 1107 * SEC)});
  run72("S2_hold_1105", in, 2);  // hold-stop while walking, then a second workout that auto-ends
  // The hold freezes the fields at the hold-begin snapshot: the FTMS values
  // during the hold equal the frozen values, and stay at the final values.
  replay::WorkoutReplay r;
  size_t next = 0;
  r.runUntil(in, next, 1105 * SEC);
  const WorkoutSnapshot atBegin = r.workout.snapshot(1105 * SEC);
  r.runUntil(in, next, 1106 * SEC);
  const WorkoutSnapshot mid = r.workout.snapshot(1106 * SEC);
  CHECK_EQ(hexs(td(0, mid.wkPulses, mid.totalUs)), hexs(td(0, atBegin.wkPulses, atBegin.totalUs)));
  r.runUntil(in, next, 1107 * SEC);
  const WorkoutSnapshot after = r.workout.snapshot(1107 * SEC);
  CHECK(after.state == WorkoutState::Summary);
  CHECK_EQ(hexs(td(0, after.wkPulses, after.totalUs)), hexs(td(0, atBegin.wkPulses, atBegin.totalUs)));
}

// ------------------------------------------------------------------ 7.3
std::string cp(const char* reqHex, bool granted, bool* grantOut = nullptr) {
  const Bytes q = hex(reqHex);
  const ble::CpResult r = ble::cpRespond(q.empty() ? nullptr : q.data(), q.size(), granted);
  if (grantOut) *grantOut = r.grant;
  return r.respond ? hexs(r.rsp, 3) : std::string("none");
}

void test73() {
  bool g = false;
  CHECK_EQ(cp("00", false, &g), std::string("800001"));
  CHECK(g);
  CHECK_EQ(cp("00", true, &g), std::string("800001"));
  CHECK(g);
  CHECK_EQ(cp("00 55", false), std::string("800001"));  // lenient: trailing bytes ignored
  CHECK_EQ(cp("01", true, &g), std::string("800101"));
  CHECK(!g);
  CHECK_EQ(cp("01", false), std::string("800105"));
  CHECK_EQ(cp("07", true), std::string("800701"));
  CHECK_EQ(cp("07", false), std::string("800705"));
  CHECK_EQ(cp("07 00", true), std::string("800701"));
  CHECK_EQ(cp("08 01", true), std::string("800801"));
  CHECK_EQ(cp("08 02", true), std::string("800801"));
  CHECK_EQ(cp("08 01", false), std::string("800805"));
  CHECK_EQ(cp("08 02", false), std::string("800805"));
  CHECK_EQ(cp("08", true), std::string("800803"));      // missing param
  CHECK_EQ(cp("08 03", true), std::string("800803"));   // bad param
  CHECK_EQ(cp("08 00", true), std::string("800803"));
  CHECK_EQ(cp("08 ff", false), std::string("800803"));  // param check before permission
  CHECK_EQ(cp("08 03", false), std::string("800803"));
  CHECK_EQ(cp("08 01 99", true), std::string("800801"));
  CHECK_EQ(cp("02 64 00", true), std::string("800202"));
  CHECK_EQ(cp("02 64 00", false), std::string("800202"));
  CHECK_EQ(cp("", true), std::string("none"));           // empty write: ignored
  // every other op: not supported, whatever the grant
  for (int op = 0; op <= 0xFF; ++op) {
    if (op == 0x00 || op == 0x01 || op == 0x07 || op == 0x08) continue;
    for (bool gr : {false, true}) {
      const uint8_t req[3] = {static_cast<uint8_t>(op), 0x01, 0x00};
      for (size_t n = 1; n <= 3; ++n) {
        const ble::CpResult r = ble::cpRespond(req, n, gr);
        CHECK(r.respond && !r.grant && r.rsp[0] == 0x80 && r.rsp[1] == op && r.rsp[2] == 0x02);
      }
    }
  }
  CHECK_EQ(cp("80", true), std::string("808002"));
  CHECK_EQ(cp("14 00", true), std::string("801402"));
  CHECK_EQ(cp("15", true), std::string("801502"));
  CHECK_EQ(cp("ff", false), std::string("80ff02"));
  CHECK_EQ(cp("09 10 27", true), std::string("800902"));
  CHECK_EQ(cp("06", true), std::string("800602"));
}

// ------------------------------------------------------------------ 7.4
void test74() {
  // Training Status map
  CHECK_EQ(ble::trainingStatusCode(WorkoutState::Idle), 0x01);
  CHECK_EQ(ble::trainingStatusCode(WorkoutState::Warmup), 0x01);
  CHECK_EQ(ble::trainingStatusCode(WorkoutState::Running), 0x0D);
  CHECK_EQ(ble::trainingStatusCode(WorkoutState::Paused), 0x0D);
  CHECK_EQ(ble::trainingStatusCode(WorkoutState::Summary), 0x0F);
  uint8_t ts[2];
  ble::encodeTrainingStatus(WorkoutState::Summary, ts);
  CHECK_EQ(hexs(ts, 2), std::string("000f"));
  // Machine Status per event type
  uint8_t ms[2] = {0, 0};
  CHECK_EQ(ble::machineStatusFor(WorkoutEventType::Confirmed, ms), static_cast<size_t>(1));
  CHECK_EQ(hexs(ms, 1), std::string("04"));
  CHECK_EQ(ble::machineStatusFor(WorkoutEventType::Stopped, ms), static_cast<size_t>(2));
  CHECK_EQ(hexs(ms, 2), std::string("0201"));
  CHECK_EQ(ble::machineStatusFor(WorkoutEventType::AutoEnded, ms), static_cast<size_t>(2));
  CHECK_EQ(hexs(ms, 2), std::string("0201"));
  for (WorkoutEventType e : {WorkoutEventType::BoutStarted, WorkoutEventType::WarmupFailed,
                             WorkoutEventType::WarmupCancelled, WorkoutEventType::Paused, WorkoutEventType::Resumed,
                             WorkoutEventType::SummaryCleared, WorkoutEventType::Rearmed})
    CHECK_EQ(ble::machineStatusFor(e, ms), static_cast<size_t>(0));

  // Over replays: each start/stop event -> exactly one Machine Status; the
  // Training Status changes only on a state-class change (RUNNING <-> PAUSED
  // flaps notify nothing).
  replay::Log log;
  if (!CHECK(loadLog(kS2, log))) return;
  struct Run { const char* label; std::vector<WorkoutInput> in; std::string tsSeq, msSeq; };
  std::vector<WorkoutInput> s2 = replay::workoutInputs(log);
  std::vector<WorkoutInput> s2h = s2;
  replay::mergeInputs(s2h, {hold(InputKind::HoldBegin, 1105 * SEC), hold(InputKind::HoldComplete, 1107 * SEC)});
  // synthetic flapping: a run with 30 pause/resume flaps, then a hold-stop
  std::vector<WorkoutInput> flap;
  int64_t t = 10 * SEC;
  for (int i = 0; i < 40; ++i, t += SEC / 2) flap.push_back(WorkoutInput{t, InputKind::Pulse, std::string()});
  for (int k = 0; k < 30; ++k) {
    t += 3 * SEC;  // > STOP: pause
    for (int i = 0; i < 4; ++i, t += SEC / 2) flap.push_back(WorkoutInput{t, InputKind::Pulse, std::string()});
  }
  flap.push_back(hold(InputKind::HoldBegin, t));
  flap.push_back(hold(InputKind::HoldComplete, t + 2 * SEC));
  std::vector<Run> runs = {
      {"S2_walk", s2, "0d 0f 01", "04 0201"},
      {"S2_hold_1105", s2h, "0d 0f 0d 0f 01", "04 0201 04 0201"},
      {"synth_flap30", flap, "0d 0f 01", "04 0201"},
  };
  for (Run& run : runs) {
    replay::WorkoutReplay r;
    ble::FtmsStatus st;
    size_t seen = 0, starts = 0, stops = 0, msCount = 0, tsCount = 0, pauses = 0;
    std::string tsSeq, msSeq;
    auto drainEv = [&]() {
      for (; seen < r.trace.events.size(); ++seen) {
        const WorkoutEvent& e = r.trace.events[seen];
        const uint8_t before = st.trainingCode();
        const ble::FtmsStatus::Out o = st.onEvent(e);
        if (e.type == WorkoutEventType::Confirmed) ++starts;
        if (e.type == WorkoutEventType::Stopped || e.type == WorkoutEventType::AutoEnded) ++stops;
        if (e.type == WorkoutEventType::Paused) ++pauses;
        if (o.msLen > 0) {
          ++msCount;
          msSeq += (msSeq.empty() ? "" : " ") + hexs(o.ms, o.msLen);
        }
        if (o.tsChanged) {
          ++tsCount;
          tsSeq += (tsSeq.empty() ? "" : " ") + hexs(o.ts + 1, 1);
          CHECK(ble::trainingStatusCode(e.from) != ble::trainingStatusCode(e.to));
          CHECK(before != o.ts[1]);
        } else {
          CHECK_EQ(ble::trainingStatusCode(e.to), before);
        }
        CHECK_EQ(o.ts[1], ble::trainingStatusCode(e.to));
      }
    };
    for (const WorkoutInput& x : run.in) {
      r.feed(x);
      drainEv();
    }
    r.finish();
    drainEv();
    CHECK_EQ(msCount, starts + stops);
    CHECK_EQ(tsSeq, run.tsSeq);
    CHECK_EQ(msSeq, run.msSeq);
    CHECK_EQ(st.trainingCode(), 0x01);
    if (std::string(run.label) == "synth_flap30") CHECK(pauses >= 30u);
    std::printf("  7.4 %-13s TS %s | MS %s | %zu pauses -> %zu TS notifications\n", run.label, tsSeq.c_str(),
                msSeq.c_str(), pauses, tsCount);
  }
}

// ------------------------------------------------------------------ 7.7
const uint8_t kAddr[6] = {1, 2, 3, 4, 5, 6};

std::vector<ble::BleEvent> drainEvents(ble::LinkTable& lt) {
  std::vector<ble::BleEvent> v;
  ble::BleEvent e;
  while (lt.popEvent(e)) v.push_back(e);
  return v;
}
size_t countEv(const std::vector<ble::BleEvent>& v, ble::BleEv ev, ble::Why why = ble::Why::None, bool anyWhy = true) {
  size_t n = 0;
  for (const ble::BleEvent& e : v)
    if (e.ev == ev && (anyWhy || e.why == why)) ++n;
  return n;
}
void write(ble::LinkTable& lt, int64_t t, uint16_t conn, const char* h) {
  const Bytes b = hex(h);
  lt.onCpWrite(t, conn, b.data(), b.size());
}
std::string pollHex(ble::LinkTable& lt, int64_t now, uint16_t* conn = nullptr) {
  ble::CpSend s;
  if (!lt.pollCp(now, s)) return "none";
  if (conn) *conn = s.conn;
  return hexs(s.rsp, 3);
}

void test77() {
  // --- deferral, single token across links, ack release, handle != 0 -------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 0, kAddr);   // dummy central on handle 0 (bench rule)
    lt.onConnect(10, 1, kAddr);  // the app on handle 1
    lt.onConnect(20, 2, kAddr);
    CHECK_EQ(lt.links(), 3);
    lt.onSubscribe(30, 1, ble::Chr::Cp, 2);
    lt.onSubscribe(30, 2, ble::Chr::Cp, 2);
    write(lt, 1000, 1, "00");
    write(lt, 1100, 2, "00");
    CHECK_EQ(pollHex(lt, 1000 + 4999), std::string("none"));  // < 5 ms after the write
    uint16_t c = 0;
    CHECK_EQ(pollHex(lt, 1000 + 5000, &c), std::string("800001"));
    CHECK_EQ(c, 1);
    CHECK(lt.token().inFlight && lt.token().conn == 1);
    CHECK_EQ(pollHex(lt, 1000000), std::string("none"));  // link 2 waits: one indication in flight at most
    lt.onCpStatus(1100000, ble::HS_EDONE);                  // connInfo zeroed: only the code is used
    CHECK(!lt.token().inFlight);
    CHECK_EQ(pollHex(lt, 1100001, &c), std::string("800001"));
    CHECK_EQ(c, 2);
    lt.onCpStatus(1200000, ble::HS_EDONE);
    CHECK_EQ(pollHex(lt, 1300000), std::string("none"));
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpInd), static_cast<size_t>(2));
    CHECK_EQ(countEv(ev, ble::BleEv::CpAck), static_cast<size_t>(2));
    CHECK_EQ(countEv(ev, ble::BleEv::CpWrite), static_cast<size_t>(2));
    CHECK(lt.slot(1).controlGranted && lt.slot(2).controlGranted && !lt.slot(0).controlGranted);
  }
  // --- code 13 / other release; stray status ------------------------------
  for (int32_t code : {ble::HS_ETIMEOUT, ble::HS_ENOTCONN, 3, -1}) {
    ble::LinkTable lt;
    lt.onConnect(0, 5, kAddr);
    lt.onSubscribe(1, 5, ble::Chr::Cp, 2);
    write(lt, 100, 5, "07");
    CHECK_EQ(pollHex(lt, 6000), std::string("800705"));
    lt.onCpStatus(7000, code);
    CHECK(!lt.token().inFlight);
    write(lt, 8000, 5, "00");
    CHECK_EQ(pollHex(lt, 14000), std::string("800001"));
    const auto ev = drainEvents(lt);
    if (code == ble::HS_ETIMEOUT) CHECK_EQ(countEv(ev, ble::BleEv::CpTimeout), static_cast<size_t>(1));
    else CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::Status, false), static_cast<size_t>(1));
  }
  {
    ble::LinkTable lt;
    lt.onCpStatus(5, ble::HS_EDONE);  // nothing in flight: stray, logged, harmless
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::Stray, false), static_cast<size_t>(1));
  }
  // --- disconnect of the token's link releases it; another link's doesn't --
  {
    ble::LinkTable lt;
    lt.onConnect(0, 0, kAddr);
    lt.onConnect(0, 1, kAddr);
    lt.onSubscribe(0, 0, ble::Chr::Cp, 2);
    lt.onSubscribe(0, 1, ble::Chr::Cp, 2);
    write(lt, 0, 1, "00");
    write(lt, 10, 0, "00");
    uint16_t c = 99;
    CHECK_EQ(pollHex(lt, 5000, &c), std::string("800001"));
    CHECK_EQ(c, 1);
    lt.onDisconnect(6000, 0, 0x13);  // the OTHER link: token stays
    CHECK(lt.token().inFlight);
    lt.onConnect(7000, 0, kAddr);    // handle 0 reused: no inherited state
    CHECK(!lt.slot(0).cpIndicate && !lt.slot(0).pending && !lt.slot(0).controlGranted);
    lt.onDisconnect(8000, 1, 0x08);  // the token's link: released
    CHECK(!lt.token().inFlight);
    const auto ev = drainEvents(lt);
    bool sawTokenFree = false;
    for (const auto& e : ev)
      if (e.ev == ble::BleEv::Disconnect && e.conn == 1) sawTokenFree = e.data[1] == 1;
    CHECK(sawTokenFree);
  }
  // --- 35 s fallback --------------------------------------------------------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 3, kAddr);
    lt.onConnect(0, 4, kAddr);
    lt.onSubscribe(0, 3, ble::Chr::Cp, 2);
    lt.onSubscribe(0, 4, ble::Chr::Cp, 2);
    write(lt, 0, 3, "00");
    CHECK_EQ(pollHex(lt, 5000), std::string("800001"));
    write(lt, 10000, 4, "00");
    CHECK_EQ(pollHex(lt, 5000 + ble::CP_IND_TIMEOUT_US - 1), std::string("none"));
    uint16_t c = 0;
    CHECK_EQ(pollHex(lt, 5000 + ble::CP_IND_TIMEOUT_US, &c), std::string("800001"));  // released + next sent
    CHECK_EQ(c, 4);
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpTimeout, ble::Why::Fallback, false), static_cast<size_t>(1));
  }
  // --- a second write before the first is answered replaces it -------------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 1, kAddr);
    lt.onSubscribe(0, 1, ble::Chr::Cp, 2);
    write(lt, 0, 1, "00");
    write(lt, 3000, 1, "07");
    CHECK_EQ(pollHex(lt, 6000), std::string("none"));    // the replacement restarted the deferral
    CHECK_EQ(pollHex(lt, 8000), std::string("800705"));  // the 00 never happened: not granted
    lt.onCpStatus(9000, ble::HS_EDONE);
    CHECK_EQ(pollHex(lt, 100000), std::string("none"));
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::Replaced, false), static_cast<size_t>(1));
    CHECK_EQ(countEv(ev, ble::BleEv::CpInd), static_cast<size_t>(1));
  }
  // --- CCCD not enabled: dropped with no effect (no grant) ------------------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 1, kAddr);
    write(lt, 0, 1, "00");
    CHECK_EQ(pollHex(lt, 5000), std::string("none"));
    CHECK(!lt.slot(0).controlGranted && !lt.slot(0).pending);
    lt.onSubscribe(6000, 1, ble::Chr::Cp, 2);
    write(lt, 7000, 1, "07");
    CHECK_EQ(pollHex(lt, 12000), std::string("800705"));
    lt.onCpStatus(13000, ble::HS_EDONE);
    lt.onSubscribe(14000, 1, ble::Chr::Cp, 0);  // indications off again
    write(lt, 15000, 1, "00");
    CHECK_EQ(pollHex(lt, 21000), std::string("none"));
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::NoCccd, false), static_cast<size_t>(2));
  }
  // --- the bench CP matrix, as a LinkTable script (second central, handle 1) --
  {
    ble::LinkTable lt;
    lt.onConnect(0, 0, kAddr);  // dummy holds handle 0
    lt.onConnect(1, 1, kAddr);
    int64_t t = 1000;
    std::string got;
    auto step = [&](const char* req) {
      write(lt, t, 1, req);
      t += 6000;
      const std::string r = pollHex(lt, t);
      if (r != "none") lt.onCpStatus(t + 10000, ble::HS_EDONE);
      t += 20000;
      got += (got.empty() ? "" : " ") + r;
    };
    step("00");                                 // without indications: nothing (cp_drop)
    lt.onSubscribe(t, 1, ble::Chr::Cp, 2);
    step("00");                                 // 80 00 01
    lt.onDisconnect(t, 1, 0x13);                // "a new connection"
    lt.onConnect(t, 1, kAddr);
    lt.onSubscribe(t, 1, ble::Chr::Cp, 2);
    step("07");                                 // before 00: 80 07 05
    step("02 64 00");                           // 80 02 02
    step("08 03");                              // 80 08 03
    step("00");                                 // 80 00 01
    step("07");                                 // 80 07 01
    step("08 01");                              // 80 08 01
    CHECK_EQ(got, std::string("none 800001 800705 800202 800803 800001 800701 800801"));
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpInd), static_cast<size_t>(7));
    CHECK_EQ(countEv(ev, ble::BleEv::CpAck), static_cast<size_t>(7));
  }
  // --- send failure: matched by seq, no double release ----------------------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 1, kAddr);
    lt.onSubscribe(0, 1, ble::Chr::Cp, 2);
    write(lt, 0, 1, "00");
    ble::CpSend s;
    CHECK(lt.pollCp(5000, s));
    lt.cpSendFailed(5001, s.seq, 6);  // indicate() returned false
    CHECK(!lt.token().inFlight);
    write(lt, 6000, 1, "00");
    CHECK(lt.pollCp(11000, s));
    lt.onCpStatus(11000, ble::HS_ENOTCONN);  // synchronous failure path: onStatus first
    lt.cpSendFailed(11001, s.seq, 7);        // ... then the false return: no second release
    write(lt, 12000, 1, "00");
    CHECK(lt.pollCp(17000, s));
    lt.cpSendFailed(17001, s.seq - 1, 7);    // a stale seq never releases the current token
    CHECK(lt.token().inFlight);
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::SendFail, false), static_cast<size_t>(1));
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::Status, false), static_cast<size_t>(1));
  }
  // --- subscriptions, slot clearing, peers count, 4th link ------------------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 7, kAddr);
    lt.onConnect(0, 8, kAddr);
    lt.onConnect(0, 9, kAddr);
    lt.onConnect(0, 10, kAddr);  // no slot
    CHECK_EQ(lt.links(), 3);
    CHECK_EQ(lt.subscribedPeers(), 0);
    lt.onSubscribe(0, 7, ble::Chr::Rsc, 1);
    CHECK_EQ(lt.subscribedPeers(), 1);
    lt.onSubscribe(0, 8, ble::Chr::Td, 1);  // FTMS-only subscriber counts
    CHECK_EQ(lt.subscribedPeers(), 2);
    lt.onSubscribe(0, 8, ble::Chr::Ts, 1);
    lt.onSubscribe(0, 9, ble::Chr::Ms, 1);
    lt.onSubscribe(0, 9, ble::Chr::Cp, 2);
    CHECK_EQ(lt.subscribedPeers(), 2);  // status/CP subscriptions alone don't light the icon
    CHECK(lt.anySub(ble::Chr::Rsc) && lt.anySub(ble::Chr::Td) && lt.anySub(ble::Chr::Ts) && lt.anySub(ble::Chr::Ms) &&
          lt.anySub(ble::Chr::Cp));
    lt.onSubscribe(0, 8, ble::Chr::Td, 0);
    CHECK_EQ(lt.subscribedPeers(), 1);
    CHECK(!lt.anySub(ble::Chr::Td));
    lt.onDisconnect(0, 7, 0x13);
    CHECK_EQ(lt.subscribedPeers(), 0);
    CHECK(!lt.anySub(ble::Chr::Rsc));
    lt.onConnect(0, 7, kAddr);  // handle reused: no inherited RSC subscription
    CHECK(!lt.anySub(ble::Chr::Rsc));
    lt.onSubscribe(0, 42, ble::Chr::Rsc, 1);  // unknown handle: ignored
    CHECK(!lt.anySub(ble::Chr::Rsc));
    // a connect on a handle still in the table (its disconnect was missed)
    // starts that slot afresh: no inherited subscription, grant or request
    lt.onSubscribe(0, 7, ble::Chr::Rsc, 1);
    lt.onSubscribe(0, 7, ble::Chr::Cp, 2);
    write(lt, 0, 7, "00");
    lt.onConnect(1, 7, kAddr);
    CHECK_EQ(lt.links(), 3);
    CHECK(!lt.anySub(ble::Chr::Rsc));
    for (size_t i = 0; i < ble::LinkTable::SLOTS; ++i)
      if (lt.slot(i).conn == 7) CHECK(!lt.slot(i).rscSub && !lt.slot(i).cpIndicate && !lt.slot(i).pending);
    const auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::Connect, ble::Why::NoSlot, false), static_cast<size_t>(1));
    CHECK_EQ(countEv(ev, ble::BleEv::Sub, ble::Why::NoSlot, false), static_cast<size_t>(1));
  }
  // --- odd writes and the event queue bound ----------------------------------
  {
    ble::LinkTable lt;
    lt.onConnect(0, 1, kAddr);
    lt.onSubscribe(0, 1, ble::Chr::Cp, 2);
    lt.onCpWrite(0, 1, nullptr, 0);  // empty: ignored
    uint8_t big[21] = {};
    lt.onCpWrite(0, 1, big, sizeof(big));  // > 20: dropped
    write(lt, 0, 77, "00");                  // unknown link: dropped
    CHECK_EQ(pollHex(lt, 100000), std::string("none"));
    auto ev = drainEvents(lt);
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::Empty, false), static_cast<size_t>(1));
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::TooLong, false), static_cast<size_t>(1));
    CHECK_EQ(countEv(ev, ble::BleEv::CpDrop, ble::Why::NoSlot, false), static_cast<size_t>(1));
    for (int i = 0; i < 40; ++i) lt.logAdvRestart(i, ble::Why::Watchdog, true);
    CHECK_EQ(lt.eventDrops(), 8u);
    ev = drainEvents(lt);
    CHECK_EQ(ev.size(), ble::LinkTable::EVENT_QUEUE);
    CHECK(ev.front().t == 0 && ev.back().t == 31);  // oldest kept, newest dropped, FIFO order
  }
}

// ------------------------------------------------------------------ 7.8
// Random operations vs an independent model of the rules.
struct Model {
  struct L { bool up = false; bool cccd = false; bool granted = false; bool pend = false; Bytes req; int64_t at = 0; };
  L l[4];  // handles 0..3 (3 slots in use at most)
  bool inFlight = false;
  int conn = -1;
  int64_t sentAt = 0;
};

void test78() {
  uint64_t s = 0x7E57B1E0ull;
  auto rnd = [&s](uint64_t n) {
    s = s * 6364136223846793005ull + 1442695040888963407ull;
    return (s >> 33) % n;
  };
  uint64_t inds = 0, viol = 0, ops = 0;
  for (int run = 0; run < 300; ++run) {
    ble::LinkTable lt;
    Model m;
    int64_t t = 0;
    for (int i = 0; i < 400; ++i, ++ops) {
      t += static_cast<int64_t>(rnd(4000)) + (rnd(50) == 0 ? ble::CP_IND_TIMEOUT_US : 0);
      const uint16_t h = static_cast<uint16_t>(rnd(4));
      Model::L& L = m.l[h];
      int up = 0;
      for (const auto& x : m.l) up += x.up ? 1 : 0;
      switch (rnd(9)) {
        case 0:
          if (!L.up && up < 3) {
            lt.onConnect(t, h, kAddr);
            L = Model::L{};
            L.up = true;
          }
          break;
        case 1:
          if (L.up) {
            lt.onDisconnect(t, h, 0x13);
            L = Model::L{};
            if (m.inFlight && m.conn == h) m.inFlight = false;
          }
          break;
        case 2:
          if (L.up) {
            const bool on = rnd(3) != 0;
            lt.onSubscribe(t, h, ble::Chr::Cp, on ? 2 : 0);
            L.cccd = on;
          }
          break;
        case 3:
        case 4: {
          if (!L.up) break;
          const uint8_t ops8[] = {0x00, 0x01, 0x07, 0x08, 0x02, 0x80};
          Bytes q = {ops8[rnd(6)]};
          if (q[0] == 0x08 && rnd(4) != 0) q.push_back(static_cast<uint8_t>(rnd(4)));
          lt.onCpWrite(t, h, q.data(), q.size());
          L.pend = true;
          L.req = q;
          L.at = t;
          break;
        }
        case 5:
          if (m.inFlight || rnd(5) == 0) {
            const int32_t code = rnd(3) == 0 ? ble::HS_ETIMEOUT : (rnd(4) == 0 ? ble::HS_ENOTCONN : ble::HS_EDONE);
            lt.onCpStatus(t, code);
            m.inFlight = false;
          }
          break;
        default: {
          // model: fallback, no-CCCD drops, then the oldest ready request
          if (m.inFlight && t - m.sentAt >= ble::CP_IND_TIMEOUT_US) m.inFlight = false;
          for (auto& x : m.l)
            if (x.up && x.pend && t - x.at >= ble::CP_IND_DEFER_US && !x.cccd) x.pend = false;
          int best = -1;
          if (!m.inFlight)
            for (int k = 0; k < 4; ++k) {
              const auto& x = m.l[k];
              if (x.up && x.pend && x.cccd && t - x.at >= ble::CP_IND_DEFER_US &&
                  (best < 0 || x.at < m.l[best].at))
                best = k;
            }
          std::string want = "none";
          if (best >= 0) {
            Model::L& b = m.l[best];
            const ble::CpResult r = ble::cpRespond(b.req.data(), b.req.size(), b.granted);
            if (r.grant) b.granted = true;
            b.pend = false;
            m.inFlight = true;
            m.conn = best;
            m.sentAt = t;
            want = hexs(r.rsp, 3);
          }
          uint16_t c = 0xFFFF;
          const std::string got = pollHex(lt, t, &c);
          if (got != want || (best >= 0 && c != best)) ++viol;
          if (got != "none") ++inds;
          // invariants on the real table
          if (lt.token().inFlight != m.inFlight) ++viol;
          break;
        }
      }
      ble::BleEvent e;
      while (lt.popEvent(e)) {
      }
    }
  }
  CHECK_EQ(viol, static_cast<uint64_t>(0));
  CHECK(inds > 2000u);
  std::printf("  7.8 LinkTable vs model: %llu ops, %llu indications, %llu violations\n",
              static_cast<unsigned long long>(ops), static_cast<unsigned long long>(inds),
              static_cast<unsigned long long>(viol));
}

void testHelpers() {
  char b[8];
  const uint8_t d[4] = {0x0a, 0xff, 0x10, 0x01};
  ble::toHex(d, 4, b, sizeof(b));
  CHECK_EQ(std::string(b), std::string("0aff10"));  // truncated to fit (7 chars + NUL)
  ble::toHex(d, 2, b, sizeof(b));
  CHECK_EQ(std::string(b), std::string("0aff"));
  CHECK_EQ(std::string(ble::chrName(ble::Chr::Td)), std::string("td"));
  CHECK_EQ(std::string(ble::bleEvName(ble::BleEv::CpTimeout)), std::string("cp_timeout"));
  CHECK_EQ(std::string(ble::whyName(ble::Why::NoCccd)), std::string("no_cccd"));
  // BleSnapshot wrappers = the primitive encoders
  ble::BleSnapshot sn;
  sn.speedMmps = 3756;
  sn.odoPulses = 20666;
  sn.wkPulses = 3869;
  sn.totalUs = 1145000000;
  uint8_t r8[8], t9[9];
  ble::encodeRsc(sn, r8);
  ble::encodeTd(sn, t9);
  CHECK_EQ(hexs(r8, 8), std::string("06c2030042e20100"));
  CHECK_EQ(hexs(t9, 9), std::string("040448050709007904"));
}

}  // namespace

int main(int argc, char** argv) {
  gRoot = replay::repoRootFromArgs(argc, argv);
  testHelpers();
  test61();
  test62();
  test63_75();
  test64_76();
  test65();
  test71();
  test72();
  test73();
  test74();
  test77();
  test78();
  return check::summary("test_ble_payload");
}
