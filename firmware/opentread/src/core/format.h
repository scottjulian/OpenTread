// format.h - display strings and display filters.
//
// Integer only, no allocation, no clock reads, 32-bit safe (every product
// goes through a uint64 intermediate). Pure C++17.
//
// String formatters: fmtX(buf, n, ...) writes the string into buf (n bytes
// including the NUL) and returns its FULL length (excluding the NUL), like
// snprintf: if the return value is >= n the output was truncated to its first
// n-1 characters, always NUL-terminated (n == 0 writes nothing). The *_STR_SIZE
// constants are buffers that always hold the full string for any input.
//
//   fmtPace      "m:ss" (m = 0..99) or "--:--". Rounded to whole seconds BEFORE
//                the m:ss split (so "x:60" can't happen), from the UNROUNDED
//                mm/s: s/km = (10^6 + v/2)/v, s/mi = (1609344 + v/2)/v.
//                "--:--" when not visible, v <= 0, or s > PACE_MAX_S.
//   fmtSpeed     tenths of km/h or mph, rounded half up from the exact value
//                (one rounding): km/h (v*36 + 500)/1000, mph
//                (v*36000 + 804672)/1609344 (1 mi = 1609.344 m exactly).
//                "<int>.<d>"; v <= 0 -> "0.0".
//   fmtDistance  hundredths of km or mi, FLOORED (monotonic, never ahead of
//                the truth): km floor(um / 10^7), mi floor(um / 16093440).
//                "<int>.<dd>".
//   fmtDuration  whole seconds, FLOORED: "m:ss" below 1 h, "h:mm:ss" from
//                1 h (no upper limit: "10:00:00", "123:45:06"); us < 0 -> "0:00".
//
// Display filters (the renderer feeds them every frame, ~10 Hz):
//   PaceDisplayFilter   visibility hysteresis (show >= PACE_SHOW_MMPS, hide
//                       < PACE_HIDE_MMPS, km/h-based in both unit modes),
//                       state rule (a pace only in WARMUP/RUNNING), 1.5 s
//                       dead-band, at most one change per PACE_REFRESH_US.
//   SpeedDisplayFilter  hysteresis in hundredths c (shown tenths s changes
//                       only when |c - 10 s| >= SPEED_HYST_CENTI, then becomes
//                       speedTenths = fmtSpeed's value), at most one change per
//                       SPEED_REFRESH_US.
// A units change re-formats both at once (no rate limit, no dead-band).
#pragma once

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "workout.h"

namespace ot {

// Linked units: Mi = mph / mi / min per mi, Km = km/h / km / min per km.
// The values match the NVS "units" byte (0 imperial, 1 metric).
enum class Units : uint8_t { Mi = 0, Km = 1 };

const char* unitsName(Units u);       // "mi" / "km"   (distance unit, also the units id)
const char* speedUnitName(Units u);   // "mph" / "km/h"
const char* paceUnitName(Units u);    // "/mi" / "/km"

// Buffers that hold the longest possible string of each formatter (+ NUL).
constexpr size_t PACE_STR_SIZE  = 6;   // "99:59", "--:--"
constexpr size_t SPEED_STR_SIZE = 12;  // INT32_MAX mm/s = "7730941.1" km/h (9 chars)
constexpr size_t DIST_STR_SIZE  = 14;  // UINT32_MAX pulses = "2565813.46" km (10 chars)
constexpr size_t DUR_STR_SIZE   = 20;  // INT64_MAX us = "2562047788:00:54" (16 chars)

// Pace and speed in display units (integer). mmps <= 0 -> 0 for all of them.
uint32_t paceSeconds(int32_t mmps, Units u);  // rounded whole seconds per km / mi
uint32_t paceExactMs(int32_t mmps, Units u);  // rounded ms per km / mi: (10^9 + v/2)/v, (1609344000 + v/2)/v
uint32_t speedTenths(int32_t mmps, Units u);  // fmtSpeed's value, tenths of km/h / mph, half up
uint32_t speedCenti(int32_t mmps, Units u);   // hundredths (hysteresis compare): km/h (v*36+50)/100,
                                              // mph (v*360000 + 804672)/1609344
uint64_t distanceHundredths(uint32_t pulses, Units u);  // floored

// True if the workout state shows a pace (subject to the hysteresis): WARMUP
// and RUNNING. PAUSED, IDLE (and SUMMARY) show "--:--".
bool paceShownInState(WorkoutState s);

size_t fmtPace(char* buf, size_t n, int32_t mmps, Units u, bool visible);
size_t fmtPaceSeconds(char* buf, size_t n, uint32_t seconds);  // "m:ss", or "--:--" if > PACE_MAX_S
size_t fmtSpeed(char* buf, size_t n, int32_t mmps, Units u);
size_t fmtTenths(char* buf, size_t n, uint32_t tenths);         // "<int>.<d>"
size_t fmtDistance(char* buf, size_t n, uint32_t pulses, Units u);
size_t fmtDuration(char* buf, size_t n, int64_t us);

// ---------------------------------------------------------------------------
// PaceDisplayFilter. update() per frame with the live (unrounded) speed:
//  1. hysteresis: v >= PACE_SHOW_MMPS -> visible; v < PACE_HIDE_MMPS -> hidden;
//     in between the previous state is kept (updated on EVERY call, whatever
//     the workout state and the rate limit);
//  2. target: a pace iff the state shows one (paceShownInState) and the
//     hysteresis says visible and the rounded pace is <= PACE_MAX_S;
//  3. immediate changes (no rate limit): the first call, a units change
//     (re-formatted in the new units at once) and a state that shows no pace
//     (PAUSED/IDLE: "--:--" at once);
//  4. otherwise at most one change per PACE_REFRESH_US since the last change:
//     a show/hide whenever the target visibility differs from the shown one;
//     a new number only when |exact_ms - shown_s*1000| >= PACE_DEADBAND_MS
//     (exact_ms = paceExactMs), and it becomes the rounded paceSeconds.
// A change held back by the rate limit is not queued: the next update after
// the window re-evaluates with its own (newer) speed.
// ---------------------------------------------------------------------------
class PaceDisplayFilter {
 public:
  void reset();
  // Returns true if the shown value (number, "--:--" or units) changed.
  bool update(int64_t now, int32_t mmps, Units u, bool stateShowsPace);

  bool     shown() const { return shown_; }          // a number (false: "--:--")
  uint32_t shownSeconds() const { return shownS_; }  // valid iff shown()
  bool     hysteresisVisible() const { return hyst_; }
  Units    units() const { return units_; }
  size_t   format(char* buf, size_t n) const;        // the shown string

 private:
  bool     started_  = false;  // first update done
  bool     hyst_     = false;
  bool     shown_    = false;
  uint32_t shownS_   = 0;
  Units    units_    = Units::Mi;
  int64_t  lastChangeUs_ = 0;
};

// ---------------------------------------------------------------------------
// SpeedDisplayFilter. update() per frame with the live speed:
//  - the first call and a units change set s = speedTenths(mmps) at once;
//  - otherwise, at most once per SPEED_REFRESH_US since the last change, s
//    changes iff |c - 10 s| >= SPEED_HYST_CENTI and then becomes
//    speedTenths(mmps).
// c = speedCenti(mmps, units). A held-back change is not queued.
// The new value is NOT (c+5)/10: that rounds twice (exact -> hundredths ->
// tenths) and shows 0.1 too much whenever the exact hundredths fraction is
// in [.45, .50) (e.g. 2235 mm/s = 8.046 km/h, c = 805 -> "8.1"; plain
// fmtSpeed says "8.0"). One rounding keeps the filtered display equal to
// fmtSpeed of the sample that moved it (|c - 10 s| >= 8 implies
// |exact - 10 s| >= 7.5 hundredths, so the new value always differs from s).
// ---------------------------------------------------------------------------
class SpeedDisplayFilter {
 public:
  void reset();
  bool update(int64_t now, int32_t mmps, Units u);  // true if the shown value changed

  uint32_t shownTenths() const { return s_; }
  Units    units() const { return units_; }
  size_t   format(char* buf, size_t n) const;       // "<int>.<d>"

 private:
  bool     started_ = false;
  uint32_t s_       = 0;
  Units    units_   = Units::Mi;
  int64_t  lastChangeUs_ = 0;
};

}  // namespace ot
