// fixed.h - integer unit helpers.
//
// Units: time int64 us, belt travel uint32 pulses, distance uint64 um,
// speed int32 mm/s. Every product goes through a uint64 intermediate so the
// helpers are overflow-free on ILP32 (ESP32: int/long/size_t are 32-bit) as
// well as LP64 hosts. No floating point.
//
// Pace and display-distance formatting belong to format.h.
#pragma once

#include <climits>
#include <cstdint>

#include "config.h"

static_assert(sizeof(int64_t) == 8 && sizeof(uint64_t) == 8, "64-bit integers required");
static_assert(sizeof(int32_t) == 4 && sizeof(uint32_t) == 4, "32-bit integers required");
static_assert(CHAR_BIT == 8, "8-bit bytes required");
static_assert(INT32_MAX == 2147483647, "int32 range");

namespace ot {

// Speeds saturate here rather than overflowing int32.
constexpr int32_t SPEED_SAT_MMPS = INT32_MAX;

// Distance in micrometres: pulses x 597400. Exact; max 4294967295 pulses
// -> 2.57e15 um, far below 2^64.
constexpr uint64_t pulsesToUm(uint32_t pulses) {
  return static_cast<uint64_t>(pulses) * static_cast<uint64_t>(M_PER_PULSE_UM);
}

// FTMS Total Distance (1 m): floor(pulses x 597400 / 10^6). The encoder
// narrows to the uint24 wire field.
constexpr uint64_t ftmsDistanceM(uint32_t pulses) { return pulsesToUm(pulses) / 1000000u; }

// RSC Total Distance (1 dm): floor(pulses x 597400 / 10^5). The encoder
// narrows to the uint32 wire field.
constexpr uint64_t rscDistanceDm(uint32_t pulses) { return pulsesToUm(pulses) / 100000u; }

// Span speed estimate: total distance over total time across
// `intervals` pulse intervals, rounded half up:
//   mmps = (N x 597400 x 1000 + dt/2) / dt        (dt in us, uint64)
// Saturates at SPEED_SAT_MMPS. dt == 0 (two pulses sharing a timestamp,
// impossible after the 20 ms lockout) yields 0 instead of dividing by zero.
// Bounds: N <= 2^32-1 -> numerator <= 2.57e18; dt/2 <= 2^62; sum < 2^64.
constexpr int32_t spanSpeedMmps(uint32_t intervals, uint64_t dtUs) {
  if (dtUs == 0) return 0;
  const uint64_t num = static_cast<uint64_t>(intervals) * static_cast<uint64_t>(M_PER_PULSE_UM) * 1000u;
  const uint64_t q   = (num + dtUs / 2u) / dtUs;
  return q > static_cast<uint64_t>(SPEED_SAT_MMPS) ? SPEED_SAT_MMPS : static_cast<int32_t>(q);
}

// Decay cap, enforcing the invariant v <= M/(now - t_last):
//   floor(K x 597400 x 1000 / dt)  mm/s
// FLOORED, unlike the span estimate: rounding half up could exceed the
// invariant by < 0.5 mm/s, and the test-1.11 phase figures
// (S2 -2.26 / -0.41 / -1.33 m) are reproduced only with a floored cap.
// dt == 0 -> saturated (no cap at the pulse instant).
constexpr int32_t decayCapMmps(uint64_t sinceLastPulseUs) {
  if (sinceLastPulseUs == 0) return SPEED_SAT_MMPS;
  const uint64_t q = static_cast<uint64_t>(DECAY_CAP_K) * static_cast<uint64_t>(M_PER_PULSE_UM) * 1000u / sinceLastPulseUs;
  return q > static_cast<uint64_t>(SPEED_SAT_MMPS) ? SPEED_SAT_MMPS : static_cast<int32_t>(q);
}

// FTMS Instantaneous Speed (0.01 km/h): (mmps x 36 + 50) / 100. Negative -> 0.
constexpr uint32_t ftmsSpeedCentiKmh(int32_t mmps) {
  if (mmps <= 0) return 0;
  return static_cast<uint32_t>((static_cast<uint64_t>(mmps) * 36u + 50u) / 100u);
}

// RSC Instantaneous Speed (1/256 m/s): (mmps x 256 + 500) / 1000. Negative -> 0.
constexpr uint32_t rscSpeed256(int32_t mmps) {
  if (mmps <= 0) return 0;
  return static_cast<uint32_t>((static_cast<uint64_t>(mmps) * 256u + 500u) / 1000u);
}

// --- compile-time spot checks (tests 1.5 / 1.6) -----------------------------
static_assert(pulsesToUm(1) == 597400u, "1 pulse");
static_assert(ftmsDistanceM(1) == 0u && rscDistanceDm(1) == 5u, "1 pulse -> 0 m / 5 dm");
static_assert(ftmsDistanceM(2) == 1u && rscDistanceDm(2) == 11u, "2 pulses -> 1 m / 11 dm");
static_assert(ftmsDistanceM(1674) == 1000u, "1674 pulses -> 1000 m");
static_assert(ftmsDistanceM(20665) == 12345u && rscDistanceDm(20665) == 123452u, "20665 pulses");
static_assert(spanSpeedMmps(1, 1340000) == 446, "1.340 s -> 446 mm/s");
static_assert(spanSpeedMmps(1, 170000) == 3514, "0.170 s -> 3514 mm/s");
static_assert(spanSpeedMmps(1, 500000) == 1195, "0.5 s -> 1195 mm/s");
static_assert(spanSpeedMmps(UINT32_MAX, 1) == SPEED_SAT_MMPS, "saturates");
static_assert(spanSpeedMmps(5, 0) == 0, "dt == 0 guarded");
static_assert(decayCapMmps(2500000) == 238, "cap at STOP: 238.96 floored");
static_assert(decayCapMmps(1340000) == 445, "cap is floored (span estimate rounds to 446)");
static_assert(decayCapMmps(0) == SPEED_SAT_MMPS, "no cap at the pulse instant");

}  // namespace ot
