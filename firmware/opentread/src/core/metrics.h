// metrics.h - odometer + span-period speed estimator + decay cap.
//
// The single source of truth for belt distance and speed.
//
// Span estimator, evaluated once per pulse k (cached as speedAtLastPulse):
//   - k is the first pulse of a bout (no predecessor, or t_k - t_{k-1} > STOP)
//     -> 0 (the first interval spans the stop gap);
//   - otherwise the smallest N >= 1 with t_k - t_{k-N} >= SPAN, stopping early
//     (keeping the last valid N) if t_{k-N} - t_{k-N-1} > STOP (never span a
//     stop gap), if the timestamp ring runs out (N = TS_RING-1), or if N = k;
//   - v = N x M / (t_k - t_{k-N}) in integer mm/s, rounded half up.
// speedMmps(now):
//   - 0 if there is no pulse or now - t_last > STOP (exactly STOP is still > 0);
//   - else speedAtLastPulse, capped by the decay cap floor(M / (now - t_last))
//     when now > t_last (floored so v <= M/(now - t_last) holds, see fixed.h).
// Between pulses the result is non-increasing in `now`.
//
// No EMA or other filter on top. No floating point. Pure C++17.
#pragma once

#include <cstdint>

#include "config.h"

namespace ot {

class Metrics {
 public:
  explicit Metrics(const Config& cfg = Config{});

  // Forget every pulse (odometer back to 0).
  void reset();

  // An accepted pulse at time t (int64 us). Pulses must arrive in
  // non-decreasing time order (the loop drains the ISR ring in order and sim
  // pulses are exclusive with ISR pulses). Defensive: a t earlier than the
  // previous pulse is clamped to the previous pulse time, so time never runs
  // backwards inside the estimator.
  void onPulse(int64_t t);

  int32_t  speedMmps(int64_t now) const;
  int32_t  speedAtLastPulseMmps() const { return vLast_; }
  uint32_t odometerPulses() const { return odometer_; }   // saturates at UINT32_MAX
  bool     hasPulse() const { return count_ != 0; }
  int64_t  lastPulseUs() const;                          // valid iff hasPulse()

 private:
  int64_t at(uint32_t back) const;  // back = 0: newest timestamp
  int32_t rawSpeedNewest() const;

  Config   cfg_;
  int64_t  ts_[TS_RING];
  uint32_t head_     = 0;  // next write slot
  uint32_t count_    = 0;  // valid timestamps in the ring (<= TS_RING)
  uint32_t odometer_ = 0;  // pulses since reset
  int32_t  vLast_    = 0;  // estimator at the newest pulse
};

}  // namespace ot
