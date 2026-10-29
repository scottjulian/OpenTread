// config.h - core constants and the overridable runtime Config.
//
// PURE C++17: no Arduino / ESP-IDF / FreeRTOS includes.
// Pins and other hardware constants go to config_hw.h.
#pragma once

#include <cstdint>

namespace ot {

// --- Belt geometry (decision 2026-10-18: hardcoded true belt travel) -------
// 1 pulse per 10 slats x 5.974 cm, confirmed with the meter.
constexpr uint32_t M_PER_PULSE_UM = 597400;  // 0.5974 m

// --- Pulse filter (exact port of the pulse logger's acceptClosure) -----------
constexpr int64_t RELEASE_US = 5000;   // line seen open this long re-arms
constexpr int64_t LOCKOUT_US = 20000;  // min time between accepted pulses

// --- Edge ring (hw_pulse); power of two ------------------------------------
constexpr uint32_t EDGE_RING_SIZE = 1024;

// --- Metrics -----------------------------------------------------------------
constexpr uint32_t TS_RING  = 64;         // pulse timestamps kept for the span estimator
constexpr int64_t  SPAN_US  = 3000000;    // speed estimator span
constexpr int64_t  STOP_US  = 2500000;    // gap > STOP => speed 0 / new bout
constexpr uint32_t DECAY_CAP_K = 1;       // v <= K*M/(now - t_last)

// --- Workout state machine ---------------------------------------------------
constexpr int64_t  WARMUP_US       = 5000000;     // confirm on the first pulse >= 5 s after the bout start
constexpr int64_t  AUTO_END_US     = 900000000;   // 15 min since the last pulse -> SUMMARY
constexpr int64_t  SUMMARY_US      = 120000000;   // summary shown 2 min
constexpr uint32_t MOVING_FREEZE_K = 2;           // moving-clock tail cap = min(STOP, K*M/v_last)

// --- Display formatting and display filters ----------------------------------
// Pace visibility hysteresis, in mm/s in BOTH unit modes (km/h-based).
constexpr int32_t  PACE_SHOW_MMPS   = 444;      // 1.6 km/h: hidden -> shown at >= this
constexpr int32_t  PACE_HIDE_MMPS   = 361;      // 1.3 km/h: shown -> hidden below this
constexpr uint32_t PACE_MAX_S       = 5999;     // 99:59; a longer rounded pace shows --:--
constexpr int64_t  PACE_REFRESH_US  = 1000000;  // shown pace changes at most once per second
constexpr int64_t  PACE_DEADBAND_MS = 1500;     // ... and only when |exact_ms - shown_s*1000| >= this
constexpr int64_t  SPEED_REFRESH_US = 500000;   // shown speed changes at most twice per second
constexpr uint32_t SPEED_HYST_CENTI = 8;        // ... and only when |c - 10*s| >= this (c: hundredths)
constexpr uint32_t RENDER_MAX_HZ    = 10;       // overall render (frame) cap

// --- Touch: units tap + hold-to-stop ------------------------------------------
constexpr int64_t  TOUCH_POLL_US        = 20000;    // 50 Hz raw XPT2046 polling
constexpr uint32_t TOUCH_PRESS_N        = 3;        // DOWN after 3 consecutive valid samples (60 ms)
constexpr int64_t  TOUCH_RELEASE_MS     = 120;      // UP after 120 ms without a valid sample
constexpr int64_t  TAP_MIN_MS           = 40;       // tap duration (last valid sample - DOWN) 40..700 ms: 40 = the 3-sample DOWN itself, so every registered contact qualifies (final session 2026-10-27: 3 of 17 quick taps were 40 ms and failed at 60)
constexpr int64_t  TAP_MAX_MS           = 700;
constexpr int32_t  TAP_SLOP_PX          = 40;       // tap: drift from the DOWN point < 40 px
constexpr int64_t  HOLD_STOP_US         = 2000000;  // "hold ~2 s": continuous contact >= 2.000 s
// HOLD_SLOP_PX: drift from the DOWN point (Euclidean) beyond which a hold
// cancels: 20 px from the DOWN point rather than a margin around the button
// rect (where a hold may START is limited to the strip: STOP_RECT in
// touch_gesture.h); a started hold may wobble 20 px, e.g. up into the cells. The 2 s continuous hold stays the safeguard.
constexpr int32_t  HOLD_SLOP_PX         = 20;
constexpr int64_t  STUCK_TOUCH_MS       = 10000;    // a contact lasting > 10 s is ignored until UP
constexpr int64_t  POST_STOP_LOCKOUT_MS = 1000;     // after a completed hold: ignore touches 1 s (and until UP)
constexpr int64_t  UNITS_TOAST_MS       = 1500;     // units-toggle confirmation

static_assert((EDGE_RING_SIZE & (EDGE_RING_SIZE - 1u)) == 0, "EDGE_RING_SIZE must be a power of two");
static_assert((TS_RING & (TS_RING - 1u)) == 0, "TS_RING must be a power of two");
static_assert(TS_RING >= 2, "TS_RING must hold at least two timestamps");
static_assert(DECAY_CAP_K == 1, "the decay cap constant is 1 (implicit)");
static_assert(RELEASE_US > 0 && LOCKOUT_US > 0 && SPAN_US > 0 && STOP_US > 0, "timing constants must be positive");
static_assert(WARMUP_US > 0 && SUMMARY_US > 0 && AUTO_END_US > STOP_US, "workout timers: AUTO_END must exceed STOP");
static_assert(MOVING_FREEZE_K == 2, "the moving-clock tail cap constant is 2");
static_assert(PACE_HIDE_MMPS > 0 && PACE_HIDE_MMPS < PACE_SHOW_MMPS, "pace hysteresis: 0 < hide < show");
// PACE_MAX_S: the speed hysteresis alone decides visibility in both unit
// modes, i.e. the slowest shown pace (at the hide threshold) is <= 99:59 in
// min/mi (the longer unit): round(1609344 / 361) = 4458 s = 74:18.
static_assert((1609344u + static_cast<uint32_t>(PACE_HIDE_MMPS) / 2u) / static_cast<uint32_t>(PACE_HIDE_MMPS) <= PACE_MAX_S,
              "pace at the hide threshold must be <= PACE_MAX_S in min/mi");
static_assert(PACE_REFRESH_US > 0 && PACE_DEADBAND_MS > 0 && SPEED_REFRESH_US > 0 && SPEED_HYST_CENTI > 0,
              "display filter constants must be positive");
static_assert(TOUCH_POLL_US > 0 && TOUCH_PRESS_N >= 1 && TOUCH_RELEASE_MS * 1000 > TOUCH_POLL_US,
              "touch: the release window must bridge at least one poll gap");
static_assert(TAP_MIN_MS > 0 && TAP_MIN_MS < TAP_MAX_MS && TAP_MAX_MS * 1000 < HOLD_STOP_US,
              "touch: 0 < tap min < tap max < hold");
static_assert(HOLD_SLOP_PX > 0 && HOLD_SLOP_PX < TAP_SLOP_PX && STUCK_TOUCH_MS * 1000 > HOLD_STOP_US &&
                  POST_STOP_LOCKOUT_MS > 0 && UNITS_TOAST_MS > 0,
              "touch constants");
static_assert(RENDER_MAX_HZ >= 10,"timers are checked at >= 10 Hz, so the frame rate must be >= 10 Hz");

// Runtime configuration. Production code uses the defaults; tests may
// override individual fields. All durations are int64 microseconds.
// stopUs is shared by Metrics (speed -> 0, first-of-bout rule) and Workout
// (pause / warm-up fail / re-arm), so one STOP value drives both.
// Workout requires autoEndUs > stopUs (otherwise PAUSED could be skipped).
struct Config {
  int64_t releaseUs = RELEASE_US;
  int64_t lockoutUs = LOCKOUT_US;
  int64_t spanUs    = SPAN_US;
  int64_t stopUs    = STOP_US;
  int64_t warmupUs  = WARMUP_US;
  int64_t autoEndUs = AUTO_END_US;
  int64_t summaryUs = SUMMARY_US;
};

}  // namespace ot
