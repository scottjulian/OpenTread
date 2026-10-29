// pulse_filter.h - closure filter, an EXACT port of acceptClosure ("PULSE
// RULE") in the original pulse logger that captured the logs in test/data/.
//
// An accepted pulse is an edge with level 0 (closure) such that
//   (a) re-arm: since the previous level-0 edge the line was seen OPEN, i.e.
//       some edge with level 1 (or the boot state, if HIGH) was followed by
//       >= RELEASE_US with no other edge; every level-0 edge disarms; and
//   (b) lockout: it is >= LOCKOUT_US after the previous ACCEPTED pulse.
// The arming test runs at every edge (before the edge's own level is
// recorded), exactly like the logger: `prevLevel == 1 && t - prevT >= release`.
// At reset the boot state is a pseudo-edge (initialLevel at t0): HIGH means
// "open since t0" (a closure >= RELEASE_US after t0 is accepted), LOW
// (parked on a magnet) means the first pulse needs a real release first.
//
// Pure C++17, no clock reads: callers pass ISR timestamps (int64 us).
#pragma once

#include <cstdint>

#include "config.h"

namespace ot {

class PulseFilter {
 public:
  explicit PulseFilter(const Config& cfg = Config{});

  // Initialise from the boot banner: pin level and time sampled before the
  // ISR was attached. level != 0 is treated as 1 (open).
  void reset(int64_t t0, uint8_t initialLevel);

  // Feed EVERY edge in timestamp order. level is the pin level read in the
  // ISR right after the edge (0 = closed/magnet, 1 = open); level != 0 is
  // treated as 1. Returns true iff this edge is an accepted pulse.
  bool onEdge(int64_t t, uint8_t level);

  uint32_t acceptedCount() const { return accepted_; }
  bool     hasAccepted() const { return hasAccepted_; }
  int64_t  lastAcceptUs() const { return lastAcceptT_; }  // valid iff hasAccepted()

 private:
  Config   cfg_;
  uint8_t  prevLevel_   = 0;      // level of the previous edge (or at boot)
  int64_t  prevT_       = 0;      // time of the previous edge (or boot)
  int64_t  lastAcceptT_ = 0;      // time of the last accepted closure
  bool     armed_       = false;  // line seen open >= release since last closure
  bool     hasAccepted_ = false;  // at least one closure accepted
  uint32_t accepted_    = 0;      // accepted pulses since reset (saturating)
};

}  // namespace ot
