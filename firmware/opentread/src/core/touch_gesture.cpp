// touch_gesture.cpp - see touch_gesture.h.
#include "touch_gesture.h"

namespace ot {

namespace {

constexpr int64_t TOUCH_RELEASE_US  = TOUCH_RELEASE_MS * 1000;
constexpr int64_t TAP_MIN_US  = TAP_MIN_MS * 1000;
constexpr int64_t TAP_MAX_US  = TAP_MAX_MS * 1000;
constexpr int64_t TOUCH_STUCK_US    = STUCK_TOUCH_MS * 1000;
constexpr int64_t STOP_LOCKOUT_US  = POST_STOP_LOCKOUT_MS * 1000;
constexpr int32_t HOLD_SLOP2  = HOLD_SLOP_PX * HOLD_SLOP_PX;
constexpr int32_t TAP_SLOP2   = TAP_SLOP_PX * TAP_SLOP_PX;
static_assert(TOUCH_PRESS_N == 3, "the DOWN point is the median of 3 samples");

int16_t med3(int16_t a, int16_t b, int16_t c) {
  if (a > b) {
    const int16_t t = a;
    a = b;
    b = t;
  }
  // a <= b
  if (c <= a) return a;
  if (c >= b) return b;
  return c;
}

int16_t clampPx(int32_t v) {
  return static_cast<int16_t>(v < INT16_MIN ? INT16_MIN : (v > INT16_MAX ? INT16_MAX : v));
}

}  // namespace

const char* gestureTypeName(GestureType g) {
  switch (g) {
    case GestureType::TouchDown: return "touch_down";
    case GestureType::TouchUp: return "touch_up";
    case GestureType::Wake: return "wake";
    case GestureType::TapPace: return "tap_pace";
    case GestureType::HoldStart: return "hold_start";
    case GestureType::HoldCancel: return "hold_cancel";
    case GestureType::HoldComplete: return "hold_complete";
    case GestureType::Stuck: return "stuck";
  }
  return "?";
}

const char* ignoreReasonName(IgnoreReason r) {
  switch (r) {
    case IgnoreReason::None: return "none";
    case IgnoreReason::Boot: return "boot";
    case IgnoreReason::ScreenChange: return "screen_change";
    case IgnoreReason::Lockout: return "lockout";
    case IgnoreReason::Wake: return "wake";
    case IgnoreReason::Stuck: return "stuck";
    case IgnoreReason::HoldEnded: return "hold_ended";
  }
  return "?";
}

void TouchGesture::reset() {
  ctx_ = GestureContext{};
  sawInvalid_ = false;
  lockoutUntil_ = INT64_MIN;
  present_ = false;
  presenceT_ = 0;
  lastValidT_ = 0;
  lastX_ = lastY_ = 0;
  run_ = 0;
  down_ = false;
  tDown_ = 0;
  downX_ = downY_ = 0;
  maxDrift2_ = 0;
  holding_ = false;
  tapCand_ = false;
  stuckReported_ = false;
  ignore_ = IgnoreReason::None;
  qHead_ = qCount_ = 0;
  overflow_ = false;
}

void TouchGesture::push(GestureType type, int64_t t, int32_t x, int32_t y, IgnoreReason reason) {
  if (qCount_ >= EVENT_QUEUE) {
    overflow_ = true;
    return;
  }
  GestureEvent& e = q_[(qHead_ + qCount_) % EVENT_QUEUE];
  e.type = type;
  e.t = t;
  e.tDown = down_ ? tDown_ : 0;
  e.x = clampPx(x);
  e.y = clampPx(y);
  e.reason = reason;
  ++qCount_;
}

bool TouchGesture::pollEvent(GestureEvent& e) {
  if (qCount_ == 0) return false;
  e = q_[qHead_];
  qHead_ = (qHead_ + 1u) % EVENT_QUEUE;
  --qCount_;
  return true;
}

int32_t TouchGesture::holdPermille(int64_t now) const {
  if (!holding_) return -1;
  const int64_t e = now - tDown_;
  if (e <= 0) return 0;
  if (e >= HOLD_STOP_US) return 1000;
  return static_cast<int32_t>(e * 1000 / HOLD_STOP_US);
}

void TouchGesture::setContext(const GestureContext& ctx, int64_t now) {
  const bool screenChanged = ctx.screen != ctx_.screen;
  const bool stopAppeared = ctx.stopVisible && !ctx_.stopVisible;
  const bool stopGone = !ctx.stopVisible && ctx_.stopVisible;
  ctx_ = ctx;
  if (!present_) return;
  if (holding_ && (screenChanged || stopGone)) cancelHold(now, IgnoreReason::ScreenChange);
  if (screenChanged || stopAppeared) {
    tapCand_ = false;
    if (ignore_ == IgnoreReason::None) ignore_ = IgnoreReason::ScreenChange;
  }
}

void TouchGesture::cancelHold(int64_t t, IgnoreReason why) {
  holding_ = false;
  tapCand_ = false;
  push(GestureType::HoldCancel, t, lastX_, lastY_, why);
  if (ignore_ == IgnoreReason::None) ignore_ = why;
}

void TouchGesture::startPresence(int64_t t) {
  present_ = true;
  presenceT_ = t;
  run_ = 0;
  down_ = false;
  tDown_ = 0;
  maxDrift2_ = 0;
  holding_ = false;
  tapCand_ = false;
  stuckReported_ = false;
  // A contact already valid at the first sample(s) after reset (boot) is
  // ignored until UP.
  ignore_ = sawInvalid_ ? IgnoreReason::None : IgnoreReason::Boot;
}

void TouchGesture::checkStuck(int64_t t) {
  if (!present_ || stuckReported_ || t - presenceT_ <= TOUCH_STUCK_US) return;
  stuckReported_ = true;
  if (holding_) cancelHold(t, IgnoreReason::Stuck);  // unreachable in practice (a hold ends at 2 s)
  tapCand_ = false;
  ignore_ = IgnoreReason::Stuck;
  push(GestureType::Stuck, t, lastX_, lastY_, IgnoreReason::Stuck);
}

void TouchGesture::becomeDown(int64_t t) {
  down_ = true;
  tDown_ = runT_[0];
  downX_ = med3(runX_[0], runX_[1], runX_[2]);
  downY_ = med3(runY_[0], runY_[1], runY_[2]);
  maxDrift2_ = 0;
  bool wake = false, hold = false;
  if (ignore_ == IgnoreReason::None) {
    const int64_t lock = ctx_.lockoutUntil > lockoutUntil_ ? ctx_.lockoutUntil : lockoutUntil_;
    // A hold starts only if ALL 3 DOWN samples are inside stopRect (a
    // finger landing outside and sliding in never starts one; the
    // per-axis median of 3 inside points is then inside too).
    bool allInStop = ctx_.stopVisible;
    for (uint32_t i = 0; i < TOUCH_PRESS_N && allInStop; ++i) allInStop = ctx_.stopRect.contains(runX_[i], runY_[i]);
    if (!ctx_.screenAwake) {
      ignore_ = IgnoreReason::Wake;
      wake = true;
    } else if (presenceT_ < lock) {
      // The lockout applies to a contact whose PRESENCE began
      // inside the window (presenceT_ <= tDown_), even if a bouncy finger only
      // reaches DOWN after it.
      ignore_ = IgnoreReason::Lockout;
    } else if (allInStop) {
      hold = true;
    } else if (ctx_.paceRect.contains(downX_, downY_)) {
      tapCand_ = true;
    }
  }
  push(GestureType::TouchDown, tDown_, downX_, downY_, ignore_);
  if (wake) push(GestureType::Wake, tDown_, downX_, downY_, IgnoreReason::Wake);
  if (hold) {
    holding_ = true;
    push(GestureType::HoldStart, tDown_, downX_, downY_, IgnoreReason::None);
  }
  // The DOWN samples themselves count for drift (a fast sweep can already be
  // > 20 px from their median).
  for (uint32_t i = 0; i < TOUCH_PRESS_N; ++i) track(runX_[i], runY_[i], t);
}

void TouchGesture::track(int32_t x, int32_t y, int64_t t) {
  const int32_t dx = x - downX_, dy = y - downY_;
  const int32_t d2 = dx * dx + dy * dy;
  if (d2 > maxDrift2_) maxDrift2_ = d2;
  if (tapCand_ && d2 >= TAP_SLOP2) tapCand_ = false;
  if (!holding_) return;
  if (d2 > HOLD_SLOP2) {
    cancelHold(t, IgnoreReason::HoldEnded);
    return;
  }
  if (t - tDown_ >= HOLD_STOP_US) {
    holding_ = false;
    tapCand_ = false;
    ignore_ = IgnoreReason::HoldEnded;
    lockoutUntil_ = t + STOP_LOCKOUT_US;
    push(GestureType::HoldComplete, t, x, y, IgnoreReason::None);
  }
}

void TouchGesture::endPresence(int64_t t) {
  if (down_) {
    push(GestureType::TouchUp, t, lastX_, lastY_, ignore_);
    if (holding_) {
      cancelHold(t, IgnoreReason::HoldEnded);
    } else if (tapCand_ && ignore_ == IgnoreReason::None) {
      const int64_t dur = lastValidT_ - tDown_;
      if (dur >= TAP_MIN_US && dur <= TAP_MAX_US && ctx_.paceRect.contains(lastX_, lastY_))
        push(GestureType::TapPace, t, lastX_, lastY_, IgnoreReason::None);
    }
  }
  present_ = false;
  down_ = false;
  holding_ = false;
  tapCand_ = false;
  run_ = 0;
  ignore_ = IgnoreReason::None;
}

void TouchGesture::onSample(const TouchSample& s) {
  if (!s.valid) {
    sawInvalid_ = true;
    if (!present_) return;
    run_ = 0;  // DOWN needs CONSECUTIVE valid samples
    if (s.t - lastValidT_ >= TOUCH_RELEASE_US) {
      endPresence(s.t);
      return;
    }
    checkStuck(s.t);
    return;
  }
  if (!present_) startPresence(s.t);
  lastValidT_ = s.t;
  lastX_ = s.x;
  lastY_ = s.y;
  checkStuck(s.t);
  if (down_) {
    track(s.x, s.y, s.t);
    return;
  }
  runT_[run_] = s.t;
  runX_[run_] = s.x;
  runY_[run_] = s.y;
  if (++run_ >= TOUCH_PRESS_N) becomeDown(s.t);
}

}  // namespace ot
