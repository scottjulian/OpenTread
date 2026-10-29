// ui_policy.cpp - see ui_policy.h.
#include "ui_policy.h"

#include "fixed.h"

namespace ot {

namespace {

constexpr uint64_t PACE_KM_MUL = 1000u;     // s/km = moving_us * 1000 / um
constexpr uint64_t PACE_MI_MUL = 1609344u;  // s/mi = moving_us * 1609344 / (um * 1000)
constexpr uint64_t PACE_MI_DIV = 1000u;

void copyStr(char* dst, size_t n, const char* src) {
  size_t i = 0;
  for (; i + 1 < n && src[i] != '\0'; ++i) dst[i] = src[i];
  if (n != 0) dst[i] = '\0';
}

}  // namespace

const char* uiScreenName(UiScreen s) {
  switch (s) {
    case UiScreen::Idle: return "idle";
    case UiScreen::Main: return "main";
    case UiScreen::Summary: return "summary";
  }
  return "?";
}

const char* backlightName(Backlight b) {
  switch (b) {
    case Backlight::Full: return "full";
    case Backlight::Dim: return "dim";
    case Backlight::Off: return "off";
  }
  return "?";
}

UiScreen screenForState(WorkoutState s) {
  switch (s) {
    case WorkoutState::Summary: return UiScreen::Summary;
    case WorkoutState::Idle: return UiScreen::Idle;
    case WorkoutState::Warmup:
    case WorkoutState::Running:
    case WorkoutState::Paused: return UiScreen::Main;
  }
  return UiScreen::Idle;
}

// ---------------------------------------------------------------------------
// UiPolicy
// ---------------------------------------------------------------------------
UiPolicy::UiPolicy(const UiConfig& cfg) : cfg_(cfg) {}

void UiPolicy::reset(int64_t t0) { last_ = t0; }

void UiPolicy::activity(int64_t t) {
  if (t > last_) last_ = t;
}

void UiPolicy::summaryEnded(int64_t t) {
  if (t > last_) last_ = t;
}

Backlight UiPolicy::level(int64_t now, bool summaryShown) const {
  if (summaryShown) return Backlight::Full;
  const int64_t idle = now - last_;  // signed: an activity "in the future" (idle < 0) is FULL
  if (idle < cfg_.dimUs) return Backlight::Full;
  if (idle < cfg_.offUs) return Backlight::Dim;
  return Backlight::Off;
}

bool UiPolicy::nextChange(int64_t now, bool summaryShown, int64_t& at) const {
  if (summaryShown) return false;
  switch (level(now, false)) {
    case Backlight::Full: at = last_ + cfg_.dimUs; return true;
    case Backlight::Dim: at = last_ + cfg_.offUs; return true;
    case Backlight::Off: return false;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Summary math
// ---------------------------------------------------------------------------
uint64_t mulDivRoundU64(uint64_t a, uint64_t b, uint64_t c) {
  if (c == 0u) return UINT64_MAX;
  // 128-bit product (hi:lo) from 32-bit limbs.
  constexpr uint64_t M32 = 0xFFFFFFFFu;
  const uint64_t a0 = a & M32, a1 = a >> 32, b0 = b & M32, b1 = b >> 32;
  const uint64_t p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
  const uint64_t mid = (p00 >> 32) + (p01 & M32) + (p10 & M32);  // < 3 * 2^32
  uint64_t lo = (p00 & M32) | (mid << 32);
  uint64_t hi = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
  // + c/2 (half up). a*b + c/2 < 2^128, so hi never wraps.
  const uint64_t half = c / 2u;
  lo += half;
  if (lo < half) ++hi;
  if (hi >= c) return UINT64_MAX;  // the quotient needs more than 64 bits
  // Restoring long division of (hi:lo) by c; invariant r < c.
  uint64_t r = hi, q = 0;
  for (uint32_t k = 64; k-- > 0;) {
    const bool carry = (r >> 63) != 0u;
    r = (r << 1) | ((lo >> k) & 1u);
    q <<= 1;
    if (carry || r >= c) {  // the true remainder 2r + bit is >= c (wraps correctly when carry)
      r -= c;
      q |= 1u;
    }
  }
  return q;
}

uint32_t summaryPaceSeconds(uint32_t pulses, int64_t movingUs, Units u) {
  if (pulses == 0u || movingUs <= 0) return 0;
  const uint64_t um = pulsesToUm(pulses);  // <= 2.57e15, so um * 1000 < 2^64
  const uint64_t mv = static_cast<uint64_t>(movingUs);
  const uint64_t s = u == Units::Km ? mulDivRoundU64(mv, PACE_KM_MUL, um)
                                    : mulDivRoundU64(mv, PACE_MI_MUL, um * PACE_MI_DIV);
  return s > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(s);
}

bool summaryPaceShown(uint32_t pulses, int64_t movingUs, Units u) {
  if (distanceHundredths(pulses, u) == 0u || movingUs <= 0) return false;  // < 0.01 unit (or no time)
  return summaryPaceSeconds(pulses, movingUs, u) <= PACE_MAX_S;
}

void makeSummaryView(const SummaryRecord& r, Units u, SummaryView& out) {
  out = SummaryView{};
  fmtDistance(out.distance, sizeof(out.distance), r.pulses, u);
  fmtDuration(out.total, sizeof(out.total), r.totalUs);
  fmtDuration(out.moving, sizeof(out.moving), r.movingUs);
  out.paceShown = summaryPaceShown(r.pulses, r.movingUs, u);
  out.paceS = out.paceShown ? summaryPaceSeconds(r.pulses, r.movingUs, u) : 0u;
  if (out.paceShown) fmtPaceSeconds(out.avgPace, sizeof(out.avgPace), out.paceS);
  else copyStr(out.avgPace, sizeof(out.avgPace), "--:--");
  fmtSpeed(out.avgSpeed, sizeof(out.avgSpeed), r.avgMmps, u);  // <= 0 -> "0.0"
  fmtSpeed(out.topSpeed, sizeof(out.topSpeed), r.topMmps, u);
}

int64_t summaryRemainingUs(const SummaryRecord& r, int64_t summaryUs, int64_t now) {
  const int64_t elapsed = now - r.summaryStartUs;
  if (elapsed <= 0) return summaryUs;
  if (elapsed >= summaryUs) return 0;
  return summaryUs - elapsed;
}

}  // namespace ot
