// workout.cpp - see workout.h.
#include "workout.h"

#include "fixed.h"

namespace ot {

namespace {
// K * M * 1000: 2 x 597400 x 1000 = 1,194,800,000 (us * mm/s).
constexpr int64_t TAIL_NUM = static_cast<int64_t>(MOVING_FREEZE_K) * static_cast<int64_t>(M_PER_PULSE_UM) * 1000;
static_assert(TAIL_NUM == 1194800000, "2 x 597400 x 1000");
}  // namespace

const char* workoutStateName(WorkoutState s) {
  switch (s) {
    case WorkoutState::Idle: return "IDLE";
    case WorkoutState::Warmup: return "WARMUP";
    case WorkoutState::Running: return "RUNNING";
    case WorkoutState::Paused: return "PAUSED";
    case WorkoutState::Summary: return "SUMMARY";
  }
  return "?";
}

const char* workoutEventReason(WorkoutEventType e) {
  switch (e) {
    case WorkoutEventType::BoutStarted: return "bout_start";
    case WorkoutEventType::Confirmed: return "warmup_ok";
    case WorkoutEventType::WarmupFailed: return "warmup_fail";
    case WorkoutEventType::WarmupCancelled: return "hold_cancel_warmup";
    case WorkoutEventType::Paused: return "pause";
    case WorkoutEventType::Resumed: return "resume";
    case WorkoutEventType::Stopped: return "hold_stop";
    case WorkoutEventType::AutoEnded: return "autoend";
    case WorkoutEventType::SummaryCleared: return "summary_timeout";
    case WorkoutEventType::Rearmed: return "rearm";
  }
  return "?";
}

const char* endReasonName(EndReason r) { return r == EndReason::AutoEnd ? "autoend" : "hold_stop"; }

const char* holdResultName(HoldResult r) {
  switch (r) {
    case HoldResult::Ok: return "ok";
    case HoldResult::Void: return "void";
    case HoldResult::Ignored: return "ignored";
  }
  return "?";
}

int64_t movingTailUs(int32_t vMmps, int64_t stopUs) {
  if (vMmps <= 0) return 0;
  const int64_t t = TAIL_NUM / static_cast<int64_t>(vMmps);  // floor; v >= 1 -> t <= 1.19e9
  return t < stopUs ? t : stopUs;
}

int32_t averageMmps(uint32_t pulses, int64_t movingUs) {
  if (movingUs <= 0) return 0;
  const uint64_t num = pulsesToUm(pulses) * 1000u;  // <= 2.57e18 < 2^64
  const uint64_t mv = static_cast<uint64_t>(movingUs);
  const uint64_t q = (num + mv / 2u) / mv;
  return q > static_cast<uint64_t>(SPEED_SAT_MMPS) ? SPEED_SAT_MMPS : static_cast<int32_t>(q);
}

Workout::Workout(const Config& cfg) : cfg_(cfg) { reset(); }

void Workout::reset() {
  hasLatest_ = false;
  latest_ = 0;
  state_ = WorkoutState::Idle;
  stateSince_ = 0;
  hasLast_ = false;
  last_ = 0;
  boutActive_ = false;
  boutStart_ = 0;
  rearmBlocked_ = false;
  rearmFloor_ = 0;
  lastId_ = 0;
  curId_ = 0;
  tConf_ = 0;
  wkPulses_ = 0;
  committed_ = 0;
  vLast_ = 0;
  top_ = 0;
  holdActive_ = false;
  holdState_ = WorkoutState::Idle;
  holdOpenId_ = 0;
  holdPulses_ = 0;
  holdMoving_ = 0;
  holdTotal_ = 0;
  holdTop_ = 0;
  hasSummary_ = false;
  summary_ = SummaryRecord{};
  for (WorkoutEvent& e : q_) e = WorkoutEvent{};
  qHead_ = 0;
  qCount_ = 0;
  overflow_ = false;
}

void Workout::setState(WorkoutState s, int64_t t) {
  if (s != state_) stateSince_ = t;
  state_ = s;
}

void Workout::push(WorkoutEventType type, int64_t t, WorkoutState from, WorkoutState to) {
  if (qCount_ == EVENT_QUEUE) {
    overflow_ = true;  // drop the newest; host test 2.10 checks this never happens
    return;
  }
  WorkoutEvent& e = q_[(qHead_ + qCount_) % EVENT_QUEUE];
  e.type = type;
  e.t = t;
  e.from = from;
  e.to = to;
  e.summary = (type == WorkoutEventType::Stopped || type == WorkoutEventType::AutoEnded) ? summary_ : SummaryRecord{};
  ++qCount_;
}

bool Workout::pollEvent(WorkoutEvent& e) {
  if (qCount_ == 0) return false;
  e = q_[qHead_];
  qHead_ = (qHead_ + 1u) % EVENT_QUEUE;
  --qCount_;
  return true;
}

uint32_t Workout::openId() const {
  return (state_ == WorkoutState::Running || state_ == WorkoutState::Paused) ? curId_ : 0u;
}

void Workout::liveFields(int64_t now, uint32_t& pulses, int64_t& moving, int64_t& total, int32_t& top) const {
  pulses = 0;
  moving = 0;
  total = 0;
  top = 0;
  switch (state_) {
    case WorkoutState::Running: {
      const int64_t since = now > last_ ? now - last_ : 0;
      const int64_t tail = movingTailUs(vLast_, cfg_.stopUs);
      pulses = wkPulses_;
      moving = committed_ + (since < tail ? since : tail);
      total = now > tConf_ ? now - tConf_ : 0;
      top = top_;
      break;
    }
    case WorkoutState::Paused:
      pulses = wkPulses_;
      moving = committed_;
      total = now > tConf_ ? now - tConf_ : 0;
      top = top_;
      break;
    case WorkoutState::Summary:
      if (hasSummary_) {
        pulses = summary_.pulses;
        moving = summary_.movingUs;
        total = summary_.totalUs;
        top = summary_.topMmps;
      }
      break;
    case WorkoutState::Idle:
    case WorkoutState::Warmup:
      break;
  }
}

WorkoutSnapshot Workout::snapshot(int64_t nowIn) const {
  // Evaluated at max(now, latestUs), like every input (input time clamp).
  const int64_t now = (hasLatest_ && nowIn < latest_) ? latest_ : nowIn;
  WorkoutSnapshot s;
  s.state = state_;
  s.stateSince = stateSince_;
  s.workoutId = openId();
  if (state_ == WorkoutState::Summary && hasSummary_) s.workoutId = summary_.workoutId;
  s.holdActive = holdActive_;
  s.rearmBlocked = rearmBlocked_;
  // Frozen only for the workout the hold began on (a hold begun in
  // WARMUP/IDLE/SUMMARY has no open workout to freeze; after a void-making
  // confirm or auto-end the live / summary values are shown).
  if (holdActive_ && holdOpenId_ != 0 && openId() == holdOpenId_) {
    s.wkPulses = holdPulses_;
    s.movingUs = holdMoving_;
    s.totalUs = holdTotal_;
    s.topMmps = holdTop_;
  } else {
    liveFields(now, s.wkPulses, s.movingUs, s.totalUs, s.topMmps);
  }
  return s;
}

// --------------------------------------------------------------------------
// Time-based transitions
// --------------------------------------------------------------------------
bool Workout::nextDeadline(int64_t& now) const {
  bool any = false;
  int64_t best = 0;
  auto take = [&](int64_t t) {
    if (!any || t < best) best = t;
    any = true;
  };
  if (state_ == WorkoutState::Summary && hasSummary_) take(summary_.summaryStartUs + cfg_.summaryUs);
  if (hasLast_) {
    // Strict thresholds: due once now - last > X, i.e. at last + X + 1.
    if (rearmBlocked_) {
      const int64_t a = last_ + cfg_.stopUs + 1;
      take(a > rearmFloor_ ? a : rearmFloor_);
    }
    if (boutTracking() || state_ == WorkoutState::Running) take(last_ + cfg_.stopUs + 1);
    if (state_ == WorkoutState::Paused) take(last_ + cfg_.autoEndUs + 1);
  }
  if (any) now = best;
  return any;
}

void Workout::advanceTo(int64_t now) {
  // Each pass applies the earliest due transition at its stamp; a transition
  // can make another one due (pause -> auto-end -> summary_timeout after a
  // long stall), so loop. At most ~5 can chain; the bound is defensive.
  for (int guard = 0; guard < 16; ++guard) {
    Due best = Due::None;
    int64_t bestT = 0;
    auto consider = [&](Due d, int64_t stamp) {
      // Ties: SummaryTimeout (a ">=" threshold) is listed first and wins.
      if (best == Due::None || stamp < bestT) {
        best = d;
        bestT = stamp;
      }
    };
    if (state_ == WorkoutState::Summary && hasSummary_ && now - summary_.summaryStartUs >= cfg_.summaryUs)
      consider(Due::SummaryTimeout, summary_.summaryStartUs + cfg_.summaryUs);
    if (hasLast_ && now - last_ > cfg_.stopUs) {
      if (rearmBlocked_) {
        const int64_t a = last_ + cfg_.stopUs;
        consider(Due::Rearm, a > rearmFloor_ ? a : rearmFloor_);
      }
      if (boutTracking()) consider(Due::WarmupFail, last_ + cfg_.stopUs);
      if (state_ == WorkoutState::Running) consider(Due::Pause, last_ + cfg_.stopUs);
    }
    if (hasLast_ && state_ == WorkoutState::Paused && now - last_ > cfg_.autoEndUs)
      consider(Due::AutoEnd, last_ + cfg_.autoEndUs);
    if (best == Due::None) return;
    apply(best, bestT);
  }
}

void Workout::apply(Due d, int64_t at) {
  const WorkoutState from = state_;
  switch (d) {
    case Due::None:
      break;
    case Due::SummaryTimeout:
      hasSummary_ = false;
      setState(boutActive_ ? WorkoutState::Warmup : WorkoutState::Idle, at);
      push(WorkoutEventType::SummaryCleared, at, from, state_);
      break;
    case Due::Rearm:
      rearmBlocked_ = false;
      push(WorkoutEventType::Rearmed, at, from, from);
      break;
    case Due::WarmupFail:
      boutActive_ = false;
      if (state_ == WorkoutState::Warmup) setState(WorkoutState::Idle, at);
      push(WorkoutEventType::WarmupFailed, at, from, state_);
      break;
    case Due::Pause:
      committed_ += movingTailUs(vLast_, cfg_.stopUs);
      setState(WorkoutState::Paused, at);
      push(WorkoutEventType::Paused, at, from, state_);
      break;
    case Due::AutoEnd:
      // Total: autoEndT - tConf; moving = committed.
      finish(EndReason::AutoEnd, at, wkPulses_, committed_, at - tConf_, top_);
      break;
  }
}

void Workout::finish(EndReason r, int64_t at, uint32_t pulses, int64_t moving, int64_t total, int32_t top) {
  const WorkoutState from = state_;
  summary_.workoutId = curId_;
  summary_.pulses = pulses;
  summary_.movingUs = moving;
  summary_.totalUs = total;
  summary_.topMmps = top;
  summary_.avgMmps = averageMmps(pulses, moving);
  summary_.endReason = r;
  summary_.summaryStartUs = at;
  hasSummary_ = true;
  boutActive_ = false;
  setState(WorkoutState::Summary, at);
  push(r == EndReason::AutoEnd ? WorkoutEventType::AutoEnded : WorkoutEventType::Stopped, at, from, state_);
}

int64_t Workout::clampIn(int64_t t) {
  if (hasLatest_ && t < latest_) return latest_;
  hasLatest_ = true;
  latest_ = t;
  return t;
}

void Workout::tick(int64_t now) { advanceTo(clampIn(now)); }

// --------------------------------------------------------------------------
// Pulses
// --------------------------------------------------------------------------
void Workout::confirm(int64_t t, int32_t v) {
  const WorkoutState from = state_;
  // The confirm pulse is the distance and time baseline: not counted.
  tConf_ = t;
  wkPulses_ = 0;
  committed_ = 0;
  vLast_ = v;
  top_ = 0;
  curId_ = ++lastId_;
  hasSummary_ = false;  // a confirmed new bout replaces the summary
  boutActive_ = false;
  setState(WorkoutState::Running, t);
  push(WorkoutEventType::Confirmed, t, from, state_);
}

void Workout::onPulse(int64_t tIn, int32_t v) {
  const int64_t t = clampIn(tIn);  // >= last_ and >= every tick already processed
  advanceTo(t);
  const WorkoutState from = state_;
  switch (state_) {
    case WorkoutState::Idle:
      if (!rearmBlocked_) {
        boutActive_ = true;
        boutStart_ = t;
        setState(WorkoutState::Warmup, t);
        push(WorkoutEventType::BoutStarted, t, from, state_);
      }
      break;
    case WorkoutState::Summary:
      if (rearmBlocked_) break;  // the train running at stop time can't start anything
      if (!boutActive_) {
        boutActive_ = true;
        boutStart_ = t;
        push(WorkoutEventType::BoutStarted, t, from, from);
      } else if (t - boutStart_ >= cfg_.warmupUs) {
        confirm(t, v);
      }
      break;
    case WorkoutState::Warmup:
      // advanceTo(t) guarantees t - last <= STOP here (else the bout failed).
      if (t - boutStart_ >= cfg_.warmupUs) confirm(t, v);
      break;
    case WorkoutState::Running:
      committed_ += t - last_;
      if (wkPulses_ != UINT32_MAX) ++wkPulses_;
      if (v > top_) top_ = v;
      vLast_ = v;
      break;
    case WorkoutState::Paused:
      // All belt travel counts; the gap adds no moving time. v is 0 here by
      // the first-interval rule (Metrics shares STOP), so top is unaffected.
      if (wkPulses_ != UINT32_MAX) ++wkPulses_;
      vLast_ = v;
      setState(WorkoutState::Running, t);
      push(WorkoutEventType::Resumed, t, from, state_);
      break;
  }
  hasLast_ = true;
  last_ = t;
}

// --------------------------------------------------------------------------
// Hold-to-stop
// --------------------------------------------------------------------------
int64_t Workout::holdBegan(int64_t tIn) {
  const int64_t t = clampIn(tIn);
  advanceTo(t);
  holdActive_ = true;
  holdState_ = state_;
  holdOpenId_ = openId();
  liveFields(t, holdPulses_, holdMoving_, holdTotal_, holdTop_);
  return t;
}

int64_t Workout::holdAborted(int64_t tIn) {
  const int64_t t = clampIn(tIn);
  advanceTo(t);
  holdActive_ = false;
  return t;
}

HoldResult Workout::holdCompleted(int64_t nowIn, int64_t* tEffOut) {
  const int64_t now = clampIn(nowIn);
  if (tEffOut != nullptr) *tEffOut = now;
  advanceTo(now);
  if (!holdActive_) return HoldResult::Ignored;
  holdActive_ = false;
  if (openId() != holdOpenId_) return HoldResult::Void;
  if (holdState_ == WorkoutState::Idle || holdState_ == WorkoutState::Summary) return HoldResult::Ignored;
  const WorkoutState from = state_;
  switch (state_) {
    case WorkoutState::Warmup:
      boutActive_ = false;
      setState(WorkoutState::Idle, now);
      push(WorkoutEventType::WarmupCancelled, now, from, state_);
      break;
    case WorkoutState::Running:
    case WorkoutState::Paused:
      finish(EndReason::HoldStop, now, holdPulses_, holdMoving_, holdTotal_, holdTop_);
      break;
    case WorkoutState::Idle:
    case WorkoutState::Summary:
      return HoldResult::Ignored;
  }
  rearmBlocked_ = true;
  rearmFloor_ = now;
  advanceTo(now);  // belt already still for > STOP: re-arm at this instant
  return HoldResult::Ok;
}

}  // namespace ot
