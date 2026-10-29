// pulse_filter.cpp - see pulse_filter.h. Mirrors the pulse logger's acceptClosure line by line.
#include "pulse_filter.h"

namespace ot {

PulseFilter::PulseFilter(const Config& cfg) : cfg_(cfg) { reset(0, 0); }

void PulseFilter::reset(int64_t t0, uint8_t initialLevel) {
  // logger setup(): gPulseFilter = {initLevel, t0, 0, false, false}
  prevLevel_   = initialLevel != 0 ? 1 : 0;
  prevT_       = t0;
  lastAcceptT_ = 0;
  armed_       = false;
  hasAccepted_ = false;
  accepted_    = 0;
}

bool PulseFilter::onEdge(int64_t t, uint8_t level) {
  const uint8_t lv = level != 0 ? 1 : 0;
  // logger: if (f.prevLevel == 1 && t - f.prevT >= releaseUs) f.armed = true;
  if (prevLevel_ == 1 && t - prevT_ >= cfg_.releaseUs) armed_ = true;  // line was open & quiet
  prevLevel_ = lv;
  prevT_     = t;
  if (lv != 0) return false;
  const bool wasArmed = armed_;
  armed_ = false;  // any closure disarms
  if (!wasArmed || (hasAccepted_ && t - lastAcceptT_ < cfg_.lockoutUs)) return false;
  hasAccepted_ = true;
  lastAcceptT_ = t;
  if (accepted_ != UINT32_MAX) ++accepted_;
  return true;
}

}  // namespace ot
