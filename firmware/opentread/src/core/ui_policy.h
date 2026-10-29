// ui_policy.h - screen selection, backlight FULL/DIM/OFF timers and the
// summary-screen values.
//
// Screen selection (pure function of the Workout state):
//   SUMMARY (a summary record is live, including a new bout warming up behind
//   it) -> SUMMARY;  IDLE -> IDLE;  WARMUP / RUNNING / PAUSED -> MAIN.
//
// Backlight (UiPolicy):
//  - Activity = an accepted pulse (never a raw edge: chatter can't keep the
//    screen on) or a touch DOWN (every debounced DOWN, ignored ones too).
//    lastActivity = max(lastActivity, t): an activity stamped in the past
//    (a touch DOWN is the first of its 3 samples) never moves it backwards.
//  - While a summary is shown the level is always FULL (the summary counts
//    as "screen on"), whatever the idle time: entering SUMMARY (hold-stop or
//    auto-end, even from OFF) is FULL at that instant.
//  - When the summary leaves the screen at t (summary_timeout, or a new bout
//    confirming behind it) the idle timers restart from t:
//    lastActivity = max(lastActivity, t) (a cleared summary dims and turns
//    off on the same idle timers as IDLE).
//  - Otherwise idle = now - lastActivity: FULL if idle < DIM_US, DIM if
//    idle < OFF_US, else OFF. So DIM begins at exactly lastActivity + DIM_US
//    and not one us earlier (test 5.1).
//  - Boot counts as activity (reset(t0)): the screen is FULL for DIM_US after
//    power-on.
//  The policy only decides a level. The app maps it to BL_FULL/BL_DIM/BL_OFF
//  (config_hw.h) and the renderer applies it through lcd.setBrightness()
//  (LovyanGFX Light_PWM, the only code that drives GPIO 21), with an L line.
//
// Summary values (makeSummaryView): every string from the FROZEN
// SummaryRecord (never live values), in the current units:
//   distance  fmtDistance(pulses)                 (floored hundredths)
//   total     fmtDuration(totalUs)
//   moving    fmtDuration(movingUs)
//   avg pace  moving / distance in integer us / um, rounded half up to whole
//             seconds per km / mi (summaryPaceSeconds); "--:--" when the
//             distance is < 0.01 of the unit (floored hundredths == 0), when
//             moving <= 0, or when the pace is > PACE_MAX_S (99:59)
//   avg speed fmtSpeed(record.avgMmps) = distance / moving (the Z line's
//             avg_mmps; 0.0 when moving == 0)
//   top speed fmtSpeed(record.topMmps)
//
// Pure C++17, integer only, no allocation, no clock reads.
#pragma once

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "format.h"
#include "workout.h"

namespace ot {

// No movement and no touch for 60 s -> DIM, for 5 min -> OFF.
// OT_SHORT_TIMERS=1 bench builds use DIM 6 s / OFF 15 s (config_hw.h).
constexpr int64_t DIM_US = 60000000;
constexpr int64_t OFF_US = 300000000;
static_assert(DIM_US > 0 && OFF_US > DIM_US, "backlight timers: 0 < DIM < OFF");

enum class UiScreen : uint8_t { Idle, Main, Summary };
enum class Backlight : uint8_t { Full, Dim, Off };

const char* uiScreenName(UiScreen s);   // "idle" / "main" / "summary"
const char* backlightName(Backlight b); // "full" / "dim" / "off"

UiScreen screenForState(WorkoutState s);

// Touch on a dimmed screen: wake-swallow applies only while the panel is OFF, or while the
// wake redraw from OFF is still in progress (the panel is still dark). On a
// FULL or DIM panel a touch acts normally (a tap toggles the units, a hold
// on the strip stops). Either way every touch DOWN is activity, so a DIM or
// OFF screen goes back to FULL. `panel` = the level actually on the panel.
constexpr bool touchAwake(Backlight panel, bool wakeRedrawPending) {
  return panel != Backlight::Off && !wakeRedrawPending;
}

struct UiConfig {
  int64_t dimUs = DIM_US;
  int64_t offUs = OFF_US;  // must be > dimUs
};

class UiPolicy {
 public:
  explicit UiPolicy(const UiConfig& cfg = UiConfig{});

  // Boot: the activity clock starts at t0 (FULL until t0 + dimUs).
  void reset(int64_t t0);
  // An accepted pulse or a touch DOWN at t.
  void activity(int64_t t);
  // The summary left the screen at t (W from SUMMARY to anything else): the
  // idle timers restart from t.
  void summaryEnded(int64_t t);

  // The level at `now`. summaryShown = the Workout is in SUMMARY.
  Backlight level(int64_t now, bool summaryShown) const;
  // The first instant > now at which level() changes without further input;
  // false if none (OFF already, or a summary is shown: its end is a Workout
  // event, not a policy timer).
  bool nextChange(int64_t now, bool summaryShown, int64_t& at) const;

  int64_t         lastActivityUs() const { return last_; }
  const UiConfig& config() const { return cfg_; }

 private:
  UiConfig cfg_;
  int64_t  last_ = 0;
};

// floor((a * b + c / 2) / c) with a full 128-bit intermediate (portable: no
// __int128, which the 32-bit Xtensa compiler lacks). c == 0 or a quotient
// above UINT64_MAX saturates to UINT64_MAX.
uint64_t mulDivRoundU64(uint64_t a, uint64_t b, uint64_t c);

// Average pace in whole seconds per km / mi from the frozen record's integer
// moving time (us) and distance (pulses x M_PER_PULSE_UM um), rounded half
// up: s/km = moving * 1000 / um, s/mi = moving * 1609344 / (um * 1000).
// 0 when pulses == 0 or moving <= 0 (callers show "--:--" then).
uint32_t summaryPaceSeconds(uint32_t pulses, int64_t movingUs, Units u);
// True when the summary shows a pace number: distance >= 0.01 unit, moving
// > 0 and the rounded pace <= PACE_MAX_S.
bool summaryPaceShown(uint32_t pulses, int64_t movingUs, Units u);

struct SummaryView {
  char     distance[DIST_STR_SIZE] = {};
  char     total[DUR_STR_SIZE]     = {};
  char     moving[DUR_STR_SIZE]    = {};
  char     avgPace[PACE_STR_SIZE]  = {};
  char     avgSpeed[SPEED_STR_SIZE] = {};
  char     topSpeed[SPEED_STR_SIZE] = {};
  bool     paceShown = false;
  uint32_t paceS     = 0;  // valid iff paceShown
};

void makeSummaryView(const SummaryRecord& r, Units u, SummaryView& out);

// Remaining summary time at `now` (0..summaryUs), from the record's own
// summaryStartUs and the Workout's SUMMARY duration: the same clock and the
// same threshold the Workout clears the summary with.
int64_t summaryRemainingUs(const SummaryRecord& r, int64_t summaryUs, int64_t now);

}  // namespace ot
