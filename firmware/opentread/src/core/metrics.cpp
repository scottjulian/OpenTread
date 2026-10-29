// metrics.cpp - see metrics.h.
#include "metrics.h"

#include "fixed.h"

namespace ot {

namespace {
constexpr uint32_t TS_MASK = TS_RING - 1u;
}  // namespace

Metrics::Metrics(const Config& cfg) : cfg_(cfg), ts_{} { reset(); }

void Metrics::reset() {
  for (uint32_t i = 0; i < TS_RING; ++i) ts_[i] = 0;
  head_     = 0;
  count_    = 0;
  odometer_ = 0;
  vLast_    = 0;
}

int64_t Metrics::at(uint32_t back) const {
  // Caller guarantees back < count_ <= TS_RING.
  return ts_[(head_ + TS_RING - 1u - back) & TS_MASK];
}

int64_t Metrics::lastPulseUs() const { return count_ != 0 ? at(0) : 0; }

void Metrics::onPulse(int64_t t) {
  if (count_ != 0 && t < at(0)) t = at(0);  // defensive clamp, see header
  ts_[head_] = t;
  head_ = (head_ + 1u) & TS_MASK;
  if (count_ < TS_RING) ++count_;
  if (odometer_ != UINT32_MAX) ++odometer_;
  vLast_ = rawSpeedNewest();
}

int32_t Metrics::rawSpeedNewest() const {
  if (count_ < 2) return 0;                    // no predecessor: first of a bout
  const int64_t tk = at(0);
  if (tk - at(1) > cfg_.stopUs) return 0;      // first pulse after a stop gap
  const uint32_t maxN = count_ - 1u;           // N = k, or the ring runs out
  uint32_t n = 1;
  int64_t span = tk - at(1);
  while (span < cfg_.spanUs && n < maxN) {
    if (at(n) - at(n + 1u) > cfg_.stopUs) break;  // never span a stop gap
    ++n;
    span = tk - at(n);
  }
  if (span <= 0) return 0;                      // only with duplicate timestamps
  return spanSpeedMmps(n, static_cast<uint64_t>(span));
}

int32_t Metrics::speedMmps(int64_t now) const {
  if (count_ == 0) return 0;
  const int64_t tLast = at(0);
  if (now > tLast && now - tLast > cfg_.stopUs) return 0;
  int32_t v = vLast_;
  if (now > tLast) {
    const int32_t cap = decayCapMmps(static_cast<uint64_t>(now - tLast));
    if (cap < v) v = cap;
  }
  return v;
}

}  // namespace ot
