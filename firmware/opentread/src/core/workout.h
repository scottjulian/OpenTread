// workout.h - workout state machine.
//
// States IDLE, WARMUP, RUNNING, PAUSED, SUMMARY. Inputs: accepted pulses
// (with Metrics' speed at that pulse), time ticks, and the hold-to-stop
// calls. Outputs: snapshot(now), the frozen summary record, and an event
// queue the app drains for W/Z lines and BLE status.
//
// Time rules (all int64 us, ISR pulse timestamps, never loop time):
//  - Every time-based transition is stamped at its exact threshold instant,
//    not at the `now` that discovered it. onPulse(t), holdBegan(t),
//    holdAborted(t) and holdCompleted(now) first run advanceTo(t), so a late
//    tick can never reorder events (test 2.9).
//  - Input time clamp: latestUs = the maximum
//    timestamp of every input and tick processed so far. onPulse, tick and
//    the three hold calls use t_eff = max(t, latestUs); the hold calls return
//    t_eff so the app logs the I line with the exact time the core used (the
//    replay then reproduces it). The app passes the touch-DOWN instant, which can
//    lie ~60 ms in the past: the clamp keeps the hold-begin snapshot from being
//    taken before an input that was already processed (no backwards W stamp,
//    moving <= total). snapshot(now) evaluates at max(now, latestUs) too.
//  - "still for > STOP" transitions (pause, warm-up fail, re-arm) fire when
//    now - last > STOP and are stamped last + STOP (re-arm: max(last + STOP,
//    hold-complete instant)); a pulse AT last + STOP still continues the bout.
//  - Auto-end fires when now - last > AUTO_END, stamped last + AUTO_END.
//  - The summary clears when now - summaryStart >= SUMMARY (stamped
//    summaryStart + SUMMARY): a ">=" threshold, so at an equal stamp it is
//    processed BEFORE the strict ones (summary_timeout, then warmup_fail /
//    rearm at the same instant).
//
// Moving time: pulse-interval accounting. In RUNNING each pulse commits
// t - last; on pause the tail min(STOP, 2*M/vLast) is committed (0 if
// vLast == 0). Displayed moving(now) in RUNNING = committed +
// min(now - last, tail(vLast)), so the committed tail is exactly the one
// that was being shown (monotonic).
// Total time = now - tConf in RUNNING/PAUSED; at auto-end the final total is
// autoEndT - tConf; at a hold-stop it is the hold-begin
// snapshot's total.
//
// Snapshot workout fields (wkPulses, moving, total, top): live in
// RUNNING/PAUSED, the summary record's values in SUMMARY (also while a new
// bout warms up behind it), 0 in IDLE/WARMUP. While a hold that began on an
// open workout is active and that workout is still open, they are the
// hold-begin snapshot (frozen, so they never step backwards when the hold
// completes). Speed is not part of Workout (Metrics stays live).
//
// Pure C++17, integer only, no allocation, no clock reads.
#pragma once

#include <cstdint>

#include "config.h"

namespace ot {

enum class WorkoutState : uint8_t { Idle, Warmup, Running, Paused, Summary };

// Every W transition has one event type; the reason string of the W line is
// eventReason(type).
enum class WorkoutEventType : uint8_t {
  BoutStarted,      // bout_start            IDLE->WARMUP or SUMMARY->SUMMARY
  Confirmed,        // warmup_ok             WARMUP->RUNNING or SUMMARY->RUNNING
  WarmupFailed,     // warmup_fail           WARMUP->IDLE or SUMMARY->SUMMARY
  WarmupCancelled,  // hold_cancel_warmup    WARMUP->IDLE
  Paused,           // pause                 RUNNING->PAUSED
  Resumed,          // resume                PAUSED->RUNNING
  Stopped,          // hold_stop             RUNNING/PAUSED->SUMMARY (+ Z)
  AutoEnded,        // autoend               PAUSED->SUMMARY (+ Z)
  SummaryCleared,   // summary_timeout       SUMMARY->IDLE or SUMMARY->WARMUP
  Rearmed,          // rearm                 same state -> same state
};

enum class EndReason : uint8_t { HoldStop, AutoEnd };

enum class HoldResult : uint8_t { Ok, Void, Ignored };

struct SummaryRecord {
  uint32_t  workoutId      = 0;
  uint32_t  pulses         = 0;  // distance = pulses x M_PER_PULSE_UM
  int64_t   movingUs       = 0;
  int64_t   totalUs        = 0;
  int32_t   topMmps        = 0;
  int32_t   avgMmps        = 0;  // (pulses*M*1000 + moving/2) / moving, 0 if moving == 0
  EndReason endReason      = EndReason::HoldStop;
  int64_t   summaryStartUs = 0;  // the instant SUMMARY was entered
};

struct WorkoutEvent {
  WorkoutEventType type = WorkoutEventType::BoutStarted;
  int64_t          t    = 0;     // exact threshold / input instant
  WorkoutState     from = WorkoutState::Idle;
  WorkoutState     to   = WorkoutState::Idle;
  SummaryRecord    summary;      // valid for Stopped / AutoEnded (the Z record)
};

struct WorkoutSnapshot {
  WorkoutState state        = WorkoutState::Idle;
  int64_t      stateSince   = 0;  // stamp of the last state change
  uint32_t     workoutId    = 0;  // open workout (RUNNING/PAUSED), the summary's in SUMMARY, else 0
  uint32_t     wkPulses     = 0;
  int64_t      movingUs     = 0;
  int64_t      totalUs      = 0;
  int32_t      topMmps      = 0;
  bool         holdActive   = false;
  bool         rearmBlocked = false;
};

const char* workoutStateName(WorkoutState s);       // "IDLE", "WARMUP", ...
const char* workoutEventReason(WorkoutEventType e);  // "bout_start", ... (W line reasons)
const char* endReasonName(EndReason r);              // "hold_stop" / "autoend"
const char* holdResultName(HoldResult r);            // "ok" / "void" / "ignored"

// tail(v) = v > 0 ? min(STOP, floor(K*M*1000 / v)) : 0   (us; K = MOVING_FREEZE_K)
int64_t movingTailUs(int32_t vMmps, int64_t stopUs);
// avg = moving > 0 ? (pulses*M*1000 + moving/2) / moving : 0   (mm/s, half up)
int32_t averageMmps(uint32_t pulses, int64_t movingUs);

class Workout {
 public:
  static constexpr uint32_t EVENT_QUEUE = 8;

  explicit Workout(const Config& cfg = Config{});
  void reset();  // IDLE, no bout, no summary, no block, empty queue; ids restart at 1

  // An accepted pulse at t with Metrics' speedAtLastPulse for it. Every
  // input is processed at t_eff = max(t, latestUs) (input time clamp).
  void onPulse(int64_t t, int32_t speedAtPulseMmps);
  void tick(int64_t now);

  // Hold-to-stop. holdBegan(t): t = touch-DOWN instant; stores the
  // workout fields at t_eff. holdCompleted(now):
  //   - no active hold                                   -> Ignored
  //   - open workout id changed since holdBegan (a
  //     confirm or an auto-end happened during the hold) -> Void
  //   - hold began in, or completes in, IDLE/SUMMARY      -> Ignored
  //     (a hold begun with no stop button shown is Ignored whatever the
  //     state at completion, e.g. a bout that is in WARMUP by then)
  //   - WARMUP  -> IDLE, nothing recorded, re-arm block   -> Ok
  //   - RUNNING/PAUSED -> SUMMARY with the hold-begin snapshot as the final
  //     values (pulses during the hold are not counted), re-arm block -> Ok
  // A holdBegan while a hold is active restarts it.
  // Each hold call returns / reports t_eff = max(t, latestUs): the time the
  // core used, which the app logs in the I line.
  int64_t    holdBegan(int64_t t);
  int64_t    holdAborted(int64_t t);
  HoldResult holdCompleted(int64_t now, int64_t* tEffOut = nullptr);

  // Call tick(now) first (the app and the replay always do).
  WorkoutSnapshot      snapshot(int64_t now) const;
  const SummaryRecord* summary() const { return hasSummary_ ? &summary_ : nullptr; }
  bool                 pollEvent(WorkoutEvent& e);
  bool                 eventOverflow() const { return overflow_; }

  WorkoutState state() const { return state_; }
  // latestUs: the maximum timestamp of every input/tick processed (0 before any).
  int64_t latestUs() const { return latest_; }
  // The smallest `now` at which tick(now) would fire a time-based
  // transition; false if none can happen without further input.
  bool nextDeadline(int64_t& now) const;

 private:
  enum class Due : uint8_t { None, SummaryTimeout, Rearm, WarmupFail, Pause, AutoEnd };

  int64_t clampIn(int64_t t);  // t_eff = max(t, latest_); latest_ = t_eff
  void advanceTo(int64_t now);
  void apply(Due d, int64_t stamp);
  void setState(WorkoutState s, int64_t t);
  void push(WorkoutEventType type, int64_t t, WorkoutState from, WorkoutState to);
  void confirm(int64_t t, int32_t v);
  void finish(EndReason r, int64_t at, uint32_t pulses, int64_t moving, int64_t total, int32_t top);
  void liveFields(int64_t now, uint32_t& pulses, int64_t& moving, int64_t& total, int32_t& top) const;
  uint32_t openId() const;
  bool     boutTracking() const { return state_ == WorkoutState::Warmup || (state_ == WorkoutState::Summary && boutActive_); }

  Config       cfg_;
  bool         hasLatest_  = false;
  int64_t      latest_     = 0;  // latestUs (input time clamp)
  WorkoutState state_      = WorkoutState::Idle;
  int64_t      stateSince_ = 0;

  // Bout tracker (IDLE/WARMUP/SUMMARY) and the shared last-pulse time.
  bool    hasLast_    = false;
  int64_t last_       = 0;
  bool    boutActive_ = false;  // true in WARMUP; in SUMMARY: a new bout is warming up behind the summary
  int64_t boutStart_  = 0;

  // Re-arm block (set by every completed hold-stop / warm-up cancel).
  bool    rearmBlocked_ = false;
  int64_t rearmFloor_   = 0;  // the hold-complete instant

  // Open workout (RUNNING/PAUSED).
  uint32_t lastId_    = 0;  // last id handed out (ids are never reused)
  uint32_t curId_     = 0;
  int64_t  tConf_     = 0;
  uint32_t wkPulses_  = 0;
  int64_t  committed_ = 0;
  int32_t  vLast_     = 0;
  int32_t  top_       = 0;

  // Hold-begin snapshot.
  bool         holdActive_ = false;
  WorkoutState holdState_  = WorkoutState::Idle;
  uint32_t     holdOpenId_ = 0;
  uint32_t     holdPulses_ = 0;
  int64_t      holdMoving_ = 0;
  int64_t      holdTotal_  = 0;
  int32_t      holdTop_    = 0;

  // Summary.
  bool          hasSummary_ = false;
  SummaryRecord summary_;

  // Event queue (FIFO, fixed size; overflow drops the newest and latches the flag).
  WorkoutEvent q_[EVENT_QUEUE];
  uint32_t     qHead_   = 0;
  uint32_t     qCount_  = 0;
  bool         overflow_ = false;
};

}  // namespace ot
