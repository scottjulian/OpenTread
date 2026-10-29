// app.cpp - loop orchestrator: the renderer (main, idle, summary screens +
// backlight), the touch UI (units tap, hold-to-stop) and the NVS units in
// every build except OT_BRINGUP=1, which keeps the display/touch bring-up
// instead; the BLE peripheral (RSC + FTMS, hw_ble*) in every build.
//
// Pass order (no fixed period; idle passes yield one tick):
//   0. (OT_LOOPBACK builds) generator top-up and end-of-train check, BEFORE
//      `now` is read, so every edge of a finished train is drained before
//      its gen_done line
//   1. now = esp_timer_get_time()
//   2. drain the edge ring ONLY up to edges with t <= now (peek; newer edges
//      stay in the ring). An ISR edge that lands after `now` was read has
//      t >= now (same monotonic clock, the ISR timestamps first), so the
//      core never sees an event older than a tick it already processed.
//      Reed edges -> PulseFilter -> Metrics -> Workout (E/P lines, v2; W/Z
//      lines as the Workout reports transitions). Marker edges -> marker
//      filter (M lines).
//   3. sim pulses due at t <= now (OT_SIM_INPUT builds), stamped at their
//      ideal scheduled times, same path as reed pulses
//   3b. Workout.tick(now): time-based transitions, stamped at their exact
//      threshold instants (W/Z lines)
//   4. serial commands (each takes effect at `now`; OT_DEV hold commands log
//      their I line with the t_eff the core returns, = `now` here), then sim
//      pulses again (a speed change can schedule one at exactly `now`), then
//      Workout.tick(now)
//   4b. touch (not OT_BRINGUP): when a 20 ms poll is due, ONE sample
//      stamped `now` -> TouchGesture (context from the Workout state at `now`)
//      -> units toggle / Workout.holdBegan(tDown) / holdAborted / holdCompleted,
//      each logged as an I line with the t_eff the core returns (touch_down /
//      touch_up / units_toggle carry the gesture's own sample timestamps);
//      then the hold progress goes to the renderer (setHold, every pass)
//   4c. BLE (bleh::service; every build): B lines for the events
//      the NimBLE callbacks queued, advertising restart (callback flag or 1 s
//      watchdog), the FTMS CP scheduler (one indication in flight, >= 5 ms
//      after the write). Workout events reach BLE from drainWorkout (FTMS
//      Training / Machine Status, B ts / ms lines) wherever they happen.
//   4d. (not OT_BRINGUP) UiPolicy -> backlight level -> disp::setBacklight
//      (L lines; a wake from OFF redraws before raising the brightness), right
//      before the frame (a wake makes the frame due in this pass).
//      Activity = every accepted pulse (onAcceptedPulse) and every touch DOWN
//      (touchActivity); a W from SUMMARY restarts the idle timers at its stamp
//      (drainWorkout). BLE traffic is not activity.
//   5. gen_done line if step 0 saw the train end (OT_LOOPBACK builds)
//   5b. display: (OT_BRINGUP builds) the bring-up service: at most one
//      bounded draw step (< LOOP_BUDGET_US) and a touch poll when due;
//      (every other build) the renderer: when a frame is due (<= 10 Hz), ONE
//      Frame snapshot (Workout already ticked to `now`, live Metrics speed)
//      is formatted, then a bounded amount of dirty fields is pushed
//      (G render_max_us / render_count). Never from a callback.
//   5c. (not OT_BRINGUP) NVS: PersistPolicy decides; a units commit
//      only while the belt is stationary (> STOP since the last accepted
//      pulse) and no touch hold is in progress (N lines). Never at boot.
//      Never in a pass that pushed pixels (a page erase of tens of ms on top
//      of a ~14 ms hero push would break LOOP_BUDGET_US): it waits for the
//      next pass without a push (frames are <= 10 Hz, so that is ms away).
//   6. edge-overflow warning, K (1 Hz, v1) and from the SAME snapshot the BLE
//      tick (RSC + FTMS Treadmill Data setValue/notify, R/X lines, v1),
//      H (5 s), D + G (60 s)
//   7. record the pass duration (from `now`) for G; vTaskDelay(1) if no
//      edge, sim pulse or command byte was handled this pass
// Every time-based value printed at `now` is computed after everything with
// t <= now was fed to the core, so K replays exactly through the oracle.
#include "app.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <inttypes.h>
#include <string.h>

#include "config_hw.h"
#include "debug_serial.h"
#include "hw_ble.h"
#include "hw_loopback.h"
#include "hw_display.h"
#include "hw_nvs.h"
#include "hw_pulse.h"
#include "hw_touch.h"
#include "src/core/config.h"
#include "src/core/format.h"
#include "src/core/metrics.h"
#include "src/core/persist_policy.h"
#include "src/core/pulse_filter.h"
#include "src/core/touch_gesture.h"
#include "src/core/ui_policy.h"
#include "src/core/workout.h"
#include "version.h"

namespace {

// Banner lgfx=: the LovyanGFX version linked into this binary (every build
// links it: the release build runs the main screen).
constexpr const char* LGFX_BANNER = OT_LGFX_VERSION_STR;

// Display units: read from NVS at boot; this compile-time default (imperial)
// is used when NVS holds no valid value. Toggled by a tap on the pace / the
// idle units label, saved by the PersistPolicy.
constexpr ot::Units UNITS_BOOT = hw::UNITS_DEFAULT_KM ? ot::Units::Km : ot::Units::Mi;

// ---------------------------------------------------------------------------
// Core objects (sole owner) and counters. loop()-task only.
// ---------------------------------------------------------------------------
ot::Config markerConfig() {
  ot::Config c;
  c.releaseUs = hw::MARKER_DEBOUNCE_US;  // the BOOT marker uses 50 ms for both (as the pulse logger did)
  c.lockoutUs = hw::MARKER_DEBOUNCE_US;
  return c;
}

// Workout timers: production values, or the short-timer set's AUTO_END
// 30 s / SUMMARY 12 s in OT_SHORT_TIMERS=1 bench builds (banner short=1;
// tools/replay.py replays such a log with the same values). STOP and
// WARMUP never change, so Metrics and Workout share STOP.
ot::Config workoutConfig() {
  ot::Config c;
#if OT_SHORT_TIMERS
  c.autoEndUs = hw::SHORT_AUTO_END_US;
  c.summaryUs = hw::SHORT_SUMMARY_US;
#endif
  return c;
}

ot::PulseFilter gFilter;
ot::PulseFilter gMarkerFilter(markerConfig());
ot::Metrics     gMetrics;
ot::Workout     gWorkout(workoutConfig());
#if !OT_BRINGUP
ot::Units        gUnits = UNITS_BOOT;
ot::TouchGesture gGesture;
bool             gGestureOverflowReported = false;
int64_t          gLastTouchActivity = -1;  // the backlight timers read this (touchActivity)
uint32_t         gTouchDowns = 0, gUnitsToggles = 0;

// ---- UI policy (backlight timers) -------------------------------------------
// Production DIM 60 s / OFF 300 s, or the short-timer set (DIM 6 s / OFF
// 15 s) in OT_SHORT_TIMERS=1 bench builds (banner short=1).
ot::UiConfig uiConfig() {
  ot::UiConfig c;
#if OT_SHORT_TIMERS
  c.dimUs = hw::SHORT_DIM_US;
  c.offUs = hw::SHORT_OFF_US;
#endif
  return c;
}
ot::UiPolicy  gUi(uiConfig());
ot::Backlight gUiLevel = ot::Backlight::Full;  // the policy's level at the last uiService()

// ---- NVS persistence of the units --------------------------------------------
ot::PersistPolicy gPersist;
bool     gNvsSchemaOk = false;  // the "schema" key already holds NVS_SCHEMA (else written with the next commit)
uint32_t gNvsWrites = 0, gNvsFails = 0, gNvsMaxUs = 0;
#if OT_DEV
// nw<k> (bench test): k forced commits, one per loop pass
// at most every NVS_FORCE_PERIOD_US (600 commits take ~12 s: the N/S lines
// stay well inside the 115200 baud link, so no E/P line is dropped), with
// the NVS entry statistics before and after, and the count of writes slower
// than NVS_SLOW_WRITE_US (a page erase / relocation: the evidence that the
// run crossed page boundaries; a fresh 5-page NVS = 5 x 126 entries, so
// nw600 single-entry writes must reclaim at least one page).
constexpr int64_t  NVS_FORCE_PERIOD_US = 20000;
constexpr uint32_t NVS_SLOW_WRITE_US   = 20000;
uint32_t gNvsForceLeft = 0;     // forced commits still to do
int64_t  gNvsForceNextT = 0;
uint32_t gNvsForceN = 0, gNvsForceFails = 0, gNvsForceMaxUs = 0, gNvsForceSlow = 0;
#endif
#endif
hwp::InitInfo   gInit;
bool            gWorkoutOverflowReported = false;

uint32_t gPulseN    = 0;  // accepted pulses since boot (P n), all sources
int64_t  gLastPulseT = 0;
uint32_t gEdgeN     = 0;  // reed edges drained (H isr_edges), whatever the verbosity
uint32_t gMarkerN   = 0;
uint32_t gRingHwm   = 0;  // ring occupancy high-water mark seen by loop()
uint32_t gReportedEdgeDrops = 0;
int64_t  gLastDropWarnT     = -hw::DROP_WARN_MIN_US;  // first warning is never rate-limited

int64_t gNextK     = 0;
int64_t gNextH     = 0;
int64_t gNextStats = 0;

// The BLE tick rides on the K tick (one snapshot for K, RSC and FTMS).
static_assert(ot::ble::NOTIFY_PERIOD_US == hw::K_PERIOD_US, "RSC/FTMS notify period = K period (one snapshot)");

constexpr uint32_t CMD_BYTES_PER_PASS = 64;
constexpr uint32_t CMDS_PER_PASS      = 4;

// ---------------------------------------------------------------------------
// Loop-pass statistics for the G line (reset after each G).
// Histogram: 8 us buckets below 1024 us, 512 us buckets below 65536 us, one
// overflow bucket. p99 is the upper edge of the bucket holding the 99th
// percentile (never below the true value), capped at the window's max.
// ---------------------------------------------------------------------------
class LoopStats {
 public:
  void record(uint32_t us) {
    if (us > max_) max_ = us;
    ++hist_[bucket(us)];
    ++passes_;
  }
  uint32_t maxUs() const { return max_; }
  uint32_t passes() const { return passes_; }
  uint32_t p99Us() const {
    if (passes_ == 0) return 0;
    const uint32_t rank = passes_ - passes_ / 100u;  // ceil(0.99 n)
    uint32_t cum = 0;
    for (uint32_t i = 0; i < NB; ++i) {
      cum += hist_[i];
      if (cum >= rank) {
        const uint32_t up = upper(i);
        return up < max_ ? up : max_;
      }
    }
    return max_;
  }
  void reset() {
    memset(hist_, 0, sizeof(hist_));
    max_ = 0;
    passes_ = 0;
  }

 private:
  static constexpr uint32_t FINE_US = 8, FINE_N = 128;      // 0..1023
  static constexpr uint32_t COARSE_US = 512, COARSE_N = 126; // 1024..65535
  static constexpr uint32_t NB = FINE_N + COARSE_N + 1;
  static uint32_t bucket(uint32_t us) {
    if (us < FINE_US * FINE_N) return us / FINE_US;
    if (us < FINE_US * FINE_N + COARSE_US * COARSE_N) return FINE_N + (us - FINE_US * FINE_N) / COARSE_US;
    return NB - 1u;
  }
  static uint32_t upper(uint32_t i) {
    if (i < FINE_N) return (i + 1u) * FINE_US - 1u;
    if (i < FINE_N + COARSE_N) return FINE_US * FINE_N + (i - FINE_N + 1u) * COARSE_US - 1u;
    return UINT32_MAX;
  }
  uint32_t hist_[NB] = {};
  uint32_t max_ = 0;
  uint32_t passes_ = 0;
};
LoopStats gLoop;
// A pass that performed an NVS write (a units commit, an nw<k> forced
// commit, nX) is recorded HERE instead of gLoop, so a flash page erase (tens
// of ms, the flash cache off) does not fail the G
// loop_max_us <= LOOP_BUDGET_US check; the G line reports these passes in
// its own trailing fields nvs_pass_max_us / nvs_passes.
struct NvsPassStats {  // max + count only (no histogram: a few passes per window)
  uint32_t max = 0, passes = 0;
  void record(uint32_t us) {
    if (us > max) max = us;
    ++passes;
  }
  void reset() { max = passes = 0; }
};
NvsPassStats gNvsPass;
bool         gPassNvsWrite = false;  // set by nvsWrite(), consumed at the end of the pass

// ---------------------------------------------------------------------------
// Workout events -> W lines (+ Z on entry to SUMMARY). Drained after every
// call into the Workout, so W/Z lines interleave with P/I lines in the order
// the core saw its inputs (the replay merges P and I lines in that order).
// ---------------------------------------------------------------------------
void drainWorkout(int64_t now) {
  ot::WorkoutEvent e;
  while (gWorkout.pollEvent(e)) {
    dbg::workoutTransition(e.t, ot::workoutStateName(e.from), ot::workoutStateName(e.to),
                           ot::workoutEventReason(e.type));
#if !OT_BRINGUP
    // The summary left the screen (summary_timeout, or a new bout
    // confirming behind it): the idle timers restart at the exact stamp.
    if (e.from == ot::WorkoutState::Summary && e.to != ot::WorkoutState::Summary) gUi.summaryEnded(e.t);
#endif
    if (e.type == ot::WorkoutEventType::Stopped || e.type == ot::WorkoutEventType::AutoEnded) {
      const ot::SummaryRecord& z = e.summary;
      dbg::summary(e.t, ot::endReasonName(z.endReason), z.pulses, z.movingUs, z.totalUs, z.topMmps, z.avgMmps,
                   z.workoutId);
    }
    bleh::onWorkoutEvent(e.t, e);  // FTMS Training / Machine Status (B ts / ms lines)
  }
  if (gWorkout.eventOverflow() && !gWorkoutOverflowReported) {
    dbg::status(now, "ERROR workout event queue overflow: W/Z lines lost (replay will diverge)");
    gWorkoutOverflowReported = true;
  }
}

void workoutTick(int64_t now) {
  gWorkout.tick(now);
  drainWorkout(now);
}

#if !OT_BRINGUP
// ---------------------------------------------------------------------------
// The renderer's one snapshot per frame: the caller has ticked the
// Workout to `now`; speed is Metrics' live, unrounded estimate (pace is
// computed from it, never from the displayed speed). While a hold is active
// the workout fields are the hold-begin values (Workout::snapshot), as in K.
// ---------------------------------------------------------------------------
disp::Frame makeFrame(int64_t now) {
  const ot::WorkoutSnapshot ws = gWorkout.snapshot(now);
  disp::Frame f;
  f.t         = now;
  f.state     = ws.state;
  f.speedMmps = gMetrics.speedMmps(now);
  f.wkPulses  = ws.wkPulses;
  f.movingUs  = ws.movingUs;
  f.totalUs   = ws.totalUs;
  // The frozen summary record (never live values) while in SUMMARY
  const ot::SummaryRecord* sr = gWorkout.summary();
  if (ws.state == ot::WorkoutState::Summary && sr != nullptr) {
    f.hasSummary = true;
    f.summary    = *sr;
    f.summaryUs  = workoutConfig().summaryUs;
  }
  f.units     = gUnits;
  f.linked    = bleh::subscribedPeers() > 0;  // watch icon: >= 1 RSC or Treadmill Data subscriber
  f.rearmBlocked = ws.rearmBlocked;            // the idle screen asks to stop the belt first
  return f;
}
#endif

// ---------------------------------------------------------------------------
// Touch. The gesture context comes from the Workout state at the
// poll instant (the screen the renderer is showing or switching to):
//   WARMUP/RUNNING/PAUSED -> main screen: pace tap (hero rect) + hold button
//   IDLE                  -> idle screen: units tap on the hero rect's label
//   SUMMARY               -> no touch action
// screenAwake = ot::touchAwake: the panel is lit
// (FULL or DIM) with no wake redraw pending. A contact while OFF or still
// waking is a Wake (swallowed until UP; no units toggle, no hold); on a DIM
// screen taps and holds work. Every DOWN is activity (DIM/OFF -> FULL).
// ---------------------------------------------------------------------------
#if !OT_BRINGUP
// Activity: every debounced touch DOWN (ignored contacts too).
void touchActivity(int64_t t) {
  if (t > gLastTouchActivity) gLastTouchActivity = t;
  gUi.activity(t);
}

bool screenAwake() {
  const uint8_t bl = disp::backlight();
  const ot::Backlight panel =
      bl == hw::BL_OFF ? ot::Backlight::Off : (bl == hw::BL_FULL ? ot::Backlight::Full : ot::Backlight::Dim);
  return ot::touchAwake(panel, disp::backlightWakePending());
}

ot::GestureContext gestureContext() {
  const ot::WorkoutState st = gWorkout.state();
  const bool mainScreen = st == ot::WorkoutState::Warmup || st == ot::WorkoutState::Running ||
                          st == ot::WorkoutState::Paused;
  ot::GestureContext c;
  c.screen = mainScreen ? ot::TOUCH_SCREEN_MAIN
                        : (st == ot::WorkoutState::Summary ? ot::TOUCH_SCREEN_SUMMARY : ot::TOUCH_SCREEN_IDLE);
  c.screenAwake = screenAwake();  // the panel is lit (FULL or DIM), no wake redraw pending
  c.paceRect = st == ot::WorkoutState::Summary ? ot::TouchRect{} : ot::PACE_RECT;
  c.stopRect = ot::STOP_RECT;
  c.stopVisible = mainScreen;
  c.lockoutUntil = INT64_MIN;  // the post-stop lockout lives in the recognizer
  return c;
}

uint32_t handleGestures(int64_t now) {
  uint32_t n = 0;
  ot::GestureEvent e;
  char arg[24];
  while (gGesture.pollEvent(e)) {
    ++n;
    switch (e.type) {
      case ot::GestureType::TouchDown:
        ++gTouchDowns;
        snprintf(arg, sizeof(arg), "%d;%d", static_cast<int>(e.x), static_cast<int>(e.y));
        dbg::input(e.t, "touch_down", arg);
        touchActivity(e.t);
        if (e.reason != ot::IgnoreReason::None)
          dbg::status(e.t, "touch;ignored reason=%s", ot::ignoreReasonName(e.reason));
        break;
      case ot::GestureType::TouchUp:
        snprintf(arg, sizeof(arg), "%d;%d", static_cast<int>(e.x), static_cast<int>(e.y));
        dbg::input(e.t, "touch_up", arg);
        break;
      case ot::GestureType::Wake: dbg::status(e.t, "touch;wake (swallowed until UP)"); break;
      case ot::GestureType::Stuck: dbg::status(e.t, "touch;stuck > %" PRId64 " ms: ignored until UP", ot::STUCK_TOUCH_MS); break;
      case ot::GestureType::TapPace:
        gUnits = gUnits == ot::Units::Mi ? ot::Units::Km : ot::Units::Mi;
        ++gUnitsToggles;
        dbg::input(e.t, "units_toggle", ot::unitsName(gUnits));
        disp::unitsChanged(now);  // re-render every field + toast; no BLE effect (BLE is always SI)
        gPersist.onToggle(e.t);   // NVS commit after the settle window, when stationary
        break;
      case ot::GestureType::HoldStart: {
        const int64_t te = gWorkout.holdBegan(e.t);  // t_down; the core clamps to latestUs
        dbg::input(te, "hold_begin", "-");
        drainWorkout(now);
        break;
      }
      case ot::GestureType::HoldCancel: {
        const int64_t te = gWorkout.holdAborted(e.t);
        dbg::input(te, "hold_abort", "-");
        drainWorkout(now);
        break;
      }
      case ot::GestureType::HoldComplete: {
        int64_t te = e.t;
        const ot::HoldResult r = gWorkout.holdCompleted(e.t, &te);
        dbg::input(te, "hold_complete", ot::holdResultName(r));
        drainWorkout(now);
        break;
      }
    }
  }
  if (gGesture.eventOverflow() && !gGestureOverflowReported) {
    dbg::status(now, "ERROR touch gesture event queue overflow: gestures lost");
    gGestureOverflowReported = true;
  }
  return n;
}

// One touch poll when due (20 ms): context, sample, actions. Then the hold
// progress for the renderer (every pass).
uint32_t touchService(int64_t now) {
  uint32_t n = 0;
  ot::TouchSample s;
  if (touchhw::poll(now, s)) {
    gGesture.setContext(gestureContext(), now);
    n += handleGestures(now);
    gGesture.onSample(s);
    n += handleGestures(now);
  }
  disp::setHold(now, gGesture.holding(), gGesture.holdPermille(now));
  return n;
}

// ---------------------------------------------------------------------------
// Backlight from the UiPolicy. Every pass, after the touch poll
// (its activity is in) and before the frame (a wake makes the frame due now).
// ---------------------------------------------------------------------------
uint8_t backlightValue(ot::Backlight b) {
  switch (b) {
    case ot::Backlight::Full: return hw::BL_FULL;
    case ot::Backlight::Dim: return hw::BL_DIM;
    case ot::Backlight::Off: return hw::BL_OFF;
  }
  return hw::BL_FULL;
}

void uiService(int64_t now) {
  gUiLevel = gUi.level(now, gWorkout.state() == ot::WorkoutState::Summary);
  disp::setBacklight(now, backlightValue(gUiLevel));  // no-op unless the level changed; L line when applied
}

// ---------------------------------------------------------------------------
// Units in NVS. N,<t>,<event>,<key>,<val> lines, level 0.
// ---------------------------------------------------------------------------
void nLine(int64_t t, const char* event, const char* key, const char* val) {
  dbg::line(dbg::LV_ALWAYS, "N,%" PRId64 ",%s,%s,%s", t, event, key, val);
}
void nLineU(int64_t t, const char* event, const char* key, unsigned val) {
  dbg::line(dbg::LV_ALWAYS, "N,%" PRId64 ",%s,%s,%u", t, event, key, val);
}

// Boot read (setup, before the first frame): read-only, never writes.
void nvsBoot() {
  const nvshw::Read r = nvshw::readUnits();
  const int64_t t = esp_timer_get_time();
  if (r.hasSchema) nLineU(t, "read", nvshw::KEY_SCHEMA, r.schema);
  else nLine(t, "read", nvshw::KEY_SCHEMA, "none");
  if (r.hasUnits) nLineU(t, "read", nvshw::KEY_UNITS, r.units);
  else nLine(t, "read", nvshw::KEY_UNITS, "none");
  const ot::UnitsLoad ld = ot::validateUnits(r.hasSchema, r.schema, r.hasUnits, r.units, UNITS_BOOT);
  if (ld.status != ot::LoadStatus::Ok) nLineU(t, "default", nvshw::KEY_UNITS, ot::unitsByte(ld.units));
  gUnits = ld.units;
  gPersist.begin(ld.units);
  gNvsSchemaOk = r.hasSchema && r.schema == ot::NVS_SCHEMA;
  dbg::status(t, "nvs;namespace=%s opened=%u status=%s units=%s read_us=%" PRIu32 " settle_us=%" PRId64
                 " min_interval_us=%" PRId64 " (commits only while stationary; never at boot)",
              nvshw::NAMESPACE, r.opened ? 1u : 0u, ot::loadStatusName(ld.status), ot::unitsName(ld.units), r.us,
              ot::NVS_SETTLE_US, ot::NVS_MIN_INTERVAL_US);
}

// One write of the units byte (+ the schema key until it is known good).
// Returns the write result; logs N commit lines or an S ERROR.
bool nvsWrite(int64_t now, uint8_t units) {
  gPassNvsWrite = true;  // this pass goes to the G line's nvs_pass fields, not loop_max_us
  const nvshw::Write w = nvshw::writeUnits(units, !gNvsSchemaOk);
  if (w.schemaPut) gNvsSchemaOk = true;
  if (w.us > gNvsMaxUs) gNvsMaxUs = w.us;
  if (w.ok) {
    ++gNvsWrites;
    nLineU(now, "commit", nvshw::KEY_UNITS, units);
    if (w.schemaPut) nLineU(now, "commit", nvshw::KEY_SCHEMA, ot::NVS_SCHEMA);
  } else {
    ++gNvsFails;
    dbg::status(now, "ERROR nvs write failed: opened=%u units=%u schema=%u us=%" PRIu32 " (retry after %" PRId64 " us)",
                w.opened ? 1u : 0u, static_cast<unsigned>(units), w.schemaPut ? 1u : 0u, w.us,
                ot::NVS_MIN_INTERVAL_US);
  }
  dbg::status(now, "nvs;write us=%" PRIu32 " ok=%u", w.us, w.ok ? 1u : 0u);
  return w.ok;
}

#if OT_DEV
// nw<k> evidence: the NVS entry statistics (nvs_get_stats, loop only).
void nvsStatsLine(int64_t now, const char* when) {
  const nvshw::Stats st = nvshw::stats();
  if (!st.ok) {
    dbg::status(now, "nvs;stats when=%s err=%" PRId32, when, st.err);
    return;
  }
  dbg::status(now,
              "nvs;stats when=%s used=%" PRIu32 " free=%" PRIu32 " available=%" PRIu32 " total=%" PRIu32
              " namespaces=%" PRIu32,
              when, st.used, st.free, st.available, st.total, st.namespaces);
}
#endif

// Every pass, after the renderer (pass step 5c above). Returns 1 if it wrote.
uint32_t persistService(int64_t now) {
#if OT_DEV
  if (gNvsForceLeft > 0) {  // nw<k> stress test: one forced commit per pass, ignoring the policy on purpose
    if (now < gNvsForceNextT) return 0;  // paced (and no policy commit while a forced run is active)
    gNvsForceNextT = now + NVS_FORCE_PERIOD_US;
    --gNvsForceLeft;
    // Alternate the value so every write really changes the entry (NVS skips
    // identical writes); the last one leaves the current units stored.
    const uint8_t cur = ot::unitsByte(gUnits);
    const uint8_t v = (gNvsForceLeft % 2u == 0u) ? cur : static_cast<uint8_t>(cur ^ 1u);
    const int64_t t0 = esp_timer_get_time();
    const bool ok = nvsWrite(now, v);
    const int64_t d = esp_timer_get_time() - t0;
    const uint32_t us = d < 0 ? 0u : static_cast<uint32_t>(d > INT32_MAX ? INT32_MAX : d);
    ++gNvsForceN;
    if (!ok) ++gNvsForceFails;
    if (us > gNvsForceMaxUs) gNvsForceMaxUs = us;
    if (us >= NVS_SLOW_WRITE_US) ++gNvsForceSlow;
    if (gNvsForceLeft == 0) {
      gPersist.written(now, gUnits, ok);
      dbg::status(now, "nvs;forced_done n=%" PRIu32 " fails=%" PRIu32 " max_us=%" PRIu32 " slow_n=%" PRIu32
                       " (writes >= %" PRIu32 " us: page erase / relocation)",
                  gNvsForceN, gNvsForceFails, gNvsForceMaxUs, gNvsForceSlow, NVS_SLOW_WRITE_US);
      nvsStatsLine(now, "after");
    }
    return 1;
  }
#endif
  const bool hasPulse = gMetrics.hasPulse();
  const ot::PersistAction a =
      gPersist.decide(now, gUnits, hasPulse, hasPulse ? gMetrics.lastPulseUs() : 0, gGesture.holding());
  if (a == ot::PersistAction::Defer) {
    nLineU(now, "defer", nvshw::KEY_UNITS, ot::unitsByte(gUnits));
    return 0;
  }
  if (a != ot::PersistAction::Commit) return 0;
  const bool ok = nvsWrite(now, ot::unitsByte(gUnits));
  gPersist.written(now, gUnits, ok);
  return 1;
}

#if OT_DEV
// OT_DEV NVS commands: nX writes units=7 (invalid: the next boot falls
// back to imperial with N,default); nw<k> forces k commits, one per pass
// (bench test: edge capture during flash writes; ignores the stationary rule on
// purpose). Returns false when not an NVS command or malformed.
constexpr uint32_t NVS_FORCE_MAX = 1000;
bool nvsCommand(int64_t now, const char* c) {
  if (strcmp(c, "nX") == 0) {
    dbg::status(now, "cmd;%s", c);
    nvsWrite(now, 7);
    return true;
  }
  if (c[1] != 'w') return false;
  uint32_t k = 0;
  const char* end = nullptr;
  if (!dbg::parse::u32(c + 2, k, &end) || *end != '\0' || k == 0 || k > NVS_FORCE_MAX) return false;
  dbg::status(now, "cmd;%s", c);
  gNvsForceLeft = k;
  gNvsForceNextT = now;
  gNvsForceN = gNvsForceFails = gNvsForceMaxUs = gNvsForceSlow = 0;
  dbg::status(now, "nvs;forced_start k=%" PRIu32 " period_us=%" PRId64 " (one commit per loop pass, paced)", k,
              NVS_FORCE_PERIOD_US);
  nvsStatsLine(now, "before");
  return true;
}
#endif
#endif

// ---------------------------------------------------------------------------
// Pulses
// ---------------------------------------------------------------------------
void onAcceptedPulse(int64_t t) {
  const int64_t dt = (gPulseN == 0) ? 0 : (t - gLastPulseT);
  ++gPulseN;
  gLastPulseT = t;
  gMetrics.onPulse(t);
  dbg::line(dbg::LV_EDGE, "P,%" PRId64 ",%" PRIu32 ",%" PRId64, t, gPulseN, dt);
  gWorkout.onPulse(t, gMetrics.speedAtLastPulseMmps());
  drainWorkout(t);
#if !OT_BRINGUP
  gUi.activity(t);  // an accepted pulse (never a raw edge) wakes / keeps the screen on
#endif
}

void handleEdge(const hwp::EdgeEvent& e) {
  if (e.src == hwp::SRC_MARKER) {
    if (gMarkerFilter.onEdge(e.t, e.level)) {
      ++gMarkerN;
      dbg::line(dbg::LV_ALWAYS, "M,%" PRId64 ",%" PRIu32, e.t, gMarkerN);
    }
    return;
  }
  ++gEdgeN;
  dbg::line(dbg::LV_EDGE, "E,%" PRId64 ",%u", e.t, static_cast<unsigned>(e.level));
  if (gFilter.onEdge(e.t, e.level)) onAcceptedPulse(e.t);
}

uint32_t drainEdges(int64_t now) {
  const uint32_t occ = hwp::occupancy();
  if (occ > gRingHwm) gRingHwm = occ;
  uint32_t n = 0;
  hwp::EdgeEvent e;
  while (hwp::peekEdge(e)) {
    if (e.t > now) break;  // newer than this pass's tick: leave it for the next pass
    hwp::popEdge();
    handleEdge(e);
    ++n;
  }
  return n;
}

// ---------------------------------------------------------------------------
// Sim mode (OT_SIM_INPUT=1): ideal scheduled timestamps, straight into
// Metrics (bypasses the ring and PulseFilter), normal P lines continuing n.
// ---------------------------------------------------------------------------
#if OT_SIM_INPUT
constexpr uint32_t SIM_KMH_MIN_MILLI = 100;     // 0.1 km/h (interval 21.5 s)
constexpr uint32_t SIM_KMH_MAX_MILLI = 60000;   // 60 km/h (interval 35.8 ms)
constexpr int64_t  SIM_JITTER_PPM    = 30000;   // +-3%
constexpr uint32_t SIM_SEED          = 0x5EED0001u;
// interval_us = M_um * 3600 / kmh_milli  (M / v with v = kmh_milli/3.6 um/us)
constexpr uint64_t SIM_INTERVAL_NUM  = static_cast<uint64_t>(ot::M_PER_PULSE_UM) * 3600u;
static_assert(SIM_INTERVAL_NUM == 2150640000ull, "597400 um x 3600");

struct Sim {
  bool     running   = false;
  bool     jitter    = false;
  uint32_t kmhMilli  = 0;
  int64_t  baseUs    = 0;
  uint32_t rng       = SIM_SEED;
  bool     havePrev  = false;  // a sim pulse was injected in the current run
  int64_t  prevT     = 0;
  int64_t  nextT     = 0;
};
Sim gSim;

uint32_t xorshift32(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

int64_t simInterval() {
  if (!gSim.jitter) return gSim.baseUs;
  const int64_t r = static_cast<int64_t>(xorshift32(gSim.rng) % static_cast<uint32_t>(2 * SIM_JITTER_PPM + 1)) -
                    SIM_JITTER_PPM;                       // -30000..+30000 ppm
  const int64_t d = gSim.baseUs * r / 1000000;
  const int64_t iv = gSim.baseUs + d;
  return iv > 0 ? iv : 1;
}

uint32_t simInject(int64_t now) {
  uint32_t n = 0;
  while (gSim.running && gSim.nextT <= now) {
    const int64_t t = gSim.nextT;
    onAcceptedPulse(t);
    gSim.prevT = t;
    gSim.havePrev = true;
    gSim.nextT = t + simInterval();  // strictly increasing: no out-of-order, no duplicate t
    ++n;
  }
  return n;
}

// Returns false for a malformed/out-of-range command (caller emits cmd_err).
bool simCommand(int64_t now, const char* c) {
  const bool jit = (c[1] == 'j');
  uint32_t kmh = 0;
  if (!dbg::parse::kmhMilli(c + (jit ? 2 : 1), kmh)) return false;
  if (kmh != 0 && (kmh < SIM_KMH_MIN_MILLI || kmh > SIM_KMH_MAX_MILLI)) return false;
  dbg::status(now, "cmd;%s", c);
  if (kmh == 0) {
    gSim.running = false;
    dbg::status(now, "sim;stop last_pulse_us=%" PRId64, gSim.havePrev ? gSim.prevT : static_cast<int64_t>(-1));
    return true;
  }
  gSim.kmhMilli = kmh;
  gSim.jitter   = jit;
  gSim.baseUs   = static_cast<int64_t>((SIM_INTERVAL_NUM + kmh / 2u) / kmh);
  gSim.rng      = SIM_SEED;  // reseeded per command: a session is reproducible
  if (gSim.running && gSim.havePrev) {
    // Keep the phase; a pulse is never stamped before the command's instant.
    const int64_t nx = gSim.prevT + simInterval();
    gSim.nextT = nx > now ? nx : now;
  } else {
    gSim.havePrev = false;
    gSim.nextT = now + simInterval();  // the belt starts now; the first magnet one interval later
  }
  gSim.running = true;
  dbg::status(now, "sim;kmh_milli=%" PRIu32 " interval_us=%" PRId64 " jitter_ppm=%" PRId64 " seed=0x%08" PRIx32
              " next_us=%" PRId64,
              kmh, gSim.baseUs, jit ? SIM_JITTER_PPM : static_cast<int64_t>(0), SIM_SEED, gSim.nextT);
  return true;
}
#endif  // OT_SIM_INPUT

// ---------------------------------------------------------------------------
// Loopback generator (OT_LOOPBACK=1)
// ---------------------------------------------------------------------------
#if !OT_BRINGUP && (OT_SIM_INPUT || OT_LOOPBACK)
// Bench builds only: x1 = every frame pushes every field ("full redraws",
// for the loopback bench), x0 = normal (dirty fields only).
bool redrawCommand(int64_t now, const char* c) {
  if (strcmp(c, "x1") != 0 && strcmp(c, "x0") != 0) return false;
  dbg::status(now, "cmd;%s", c);
  disp::setForceRedraw(c[1] == '1');
  dbg::status(now, "display;force_redraw=%c", c[1]);
  return true;
}
#endif

#if OT_LOOPBACK
bool genCommand(int64_t now, const char* c) {
  if (strcmp(c, "g0") == 0) {
    dbg::status(now, "cmd;%s", c);
    if (lb::busy()) {
      lb::stop();
      dbg::status(now, "gen_stopping");
    }
    return true;
  }
  uint32_t rate = 0, count = 0;
  const char* p = nullptr;
  if (!dbg::parse::u32(c + 1, rate, &p) || *p != ',') return false;
  if (!dbg::parse::u32(p + 1, count, &p) || *p != '\0') return false;
  const lb::Start r = lb::start(rate, count);
  if (r != lb::Start::Ok) return false;
  dbg::status(now, "cmd;%s", c);
  lb::Info gi;
  lb::info(gi);
  dbg::status(now, "gen_start;rate=%" PRIu32 " count=%" PRIu32 " period_us=%" PRIu32, gi.rate, gi.count,
              gi.periodUs);
  return true;
}
#endif

// ---------------------------------------------------------------------------
// Hold-to-stop dev commands (OT_DEV=1): hb / hc / ha. Each logs the matching
// I line with the exact timestamp the core used: the hold calls return
// t_eff = max(now, latestUs) (input time clamp), so the replay of the
// logged P + I lines reproduces the device. On the device the touch gesture
// recognizer makes the same calls (passing the touch-DOWN instant).
// ---------------------------------------------------------------------------
#if OT_DEV
bool holdCommand(int64_t now, const char* c) {
  if (strcmp(c, "hb") == 0) {
    dbg::status(now, "cmd;%s", c);
    const int64_t te = gWorkout.holdBegan(now);
    dbg::input(te, "hold_begin", "-");
  } else if (strcmp(c, "hc") == 0) {
    dbg::status(now, "cmd;%s", c);
    int64_t te = now;
    const ot::HoldResult r = gWorkout.holdCompleted(now, &te);
    dbg::input(te, "hold_complete", ot::holdResultName(r));
  } else if (strcmp(c, "ha") == 0) {
    dbg::status(now, "cmd;%s", c);
    const int64_t te = gWorkout.holdAborted(now);
    dbg::input(te, "hold_abort", "-");
  } else {
    return false;
  }
  drainWorkout(now);
  return true;
}
#endif

// ---------------------------------------------------------------------------
// 'd': dump state
// ---------------------------------------------------------------------------
void dumpState(int64_t now) {
  dbg::status(now, "dump;fw=%s build=%08" PRIx32 " dirty=%u verb=%u uptime_us=%" PRId64, OT_FW_VERSION,
              hw::BUILD_ID, static_cast<unsigned>(hw::BUILD_DIRTY), static_cast<unsigned>(dbg::verbosity()), now);
  dbg::status(now,
              "dump;pulses=%" PRIu32 " edges=%" PRIu32 " drop_edges=%" PRIu32 " debug_drop=%" PRIu32
              " markers=%" PRIu32 " ring=%" PRIu32 " ring_occ=%" PRIu32 " ring_hwm=%" PRIu32 " line_level=%u",
              gPulseN, gEdgeN, hwp::droppedEdges(), dbg::droppedLines(), gMarkerN, ot::EDGE_RING_SIZE,
              hwp::occupancy(), gRingHwm, static_cast<unsigned>(hwp::readPulseLevel()));
  dbg::status(now,
              "dump;isr service_err=%d flags=0x%" PRIx32 " core=%u pulse_installed=%u pulse_type_err=%d"
              " pulse_add_err=%d marker_type_err=%d marker_add_err=%d isr_iram=%u",
              static_cast<int>(gInit.installErr), gInit.intrFlags, static_cast<unsigned>(gInit.isrCore),
              static_cast<unsigned>(gInit.pulseInstalled), static_cast<int>(gInit.pulseTypeErr),
              static_cast<int>(gInit.pulseAddErr), static_cast<int>(gInit.markerTypeErr),
              static_cast<int>(gInit.markerAddErr), static_cast<unsigned>(gInit.isrIram));
  dbg::status(now,
              "dump;placement pulseIsr=0x%08" PRIxPTR " iram=%u markerIsr=0x%08" PRIxPTR " iram=%u pushEdge=0x%08" PRIxPTR
              " iram=%u ring=0x%08" PRIxPTR " dram=%u dispatcher=gpio_intr_service(.iram1)",
              gInit.pulseIsrAddr, static_cast<unsigned>(gInit.pulseIsrInIram), gInit.markerIsrAddr,
              static_cast<unsigned>(gInit.markerIsrInIram), gInit.pushEdgeAddr,
              static_cast<unsigned>(gInit.pushEdgeInIram), gInit.ringAddr, static_cast<unsigned>(gInit.ringInDram));
  dbg::status(now, "dump;metrics odo_pulses=%" PRIu32 " v_last_mmps=%" PRId32 " speed_mmps=%" PRId32 " last_pulse_us=%" PRId64,
              gMetrics.odometerPulses(), gMetrics.speedAtLastPulseMmps(), gMetrics.speedMmps(now),
              gMetrics.hasPulse() ? gMetrics.lastPulseUs() : static_cast<int64_t>(-1));
  {
    workoutTick(now);
    const ot::WorkoutSnapshot ws = gWorkout.snapshot(now);
    const ot::SummaryRecord* sr = gWorkout.summary();
    dbg::status(now,
                "dump;workout state=%s since_us=%" PRId64 " id=%" PRIu32 " wk_pulses=%" PRIu32 " moving_us=%" PRId64
                " total_us=%" PRId64 " top_mmps=%" PRId32 " hold=%u rearm_blocked=%u summary=%u summary_start_us=%" PRId64
                " event_overflow=%u",
                ot::workoutStateName(ws.state), ws.stateSince, ws.workoutId, ws.wkPulses, ws.movingUs, ws.totalUs,
                ws.topMmps, ws.holdActive ? 1u : 0u, ws.rearmBlocked ? 1u : 0u, sr != nullptr ? 1u : 0u,
                sr != nullptr ? sr->summaryStartUs : static_cast<int64_t>(-1), gWorkout.eventOverflow() ? 1u : 0u);
  }
  dbg::status(now, "dump;heap free=%" PRIu32 " min_free=%" PRIu32 " largest_8bit=%" PRIu32 " largest_dma=%" PRIu32,
              static_cast<uint32_t>(esp_get_free_heap_size()), static_cast<uint32_t>(esp_get_minimum_free_heap_size()),
              static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
              static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)));
  dbg::status(now, "dump;loop passes=%" PRIu32 " max_us=%" PRIu32 " p99_us=%" PRIu32 " nvs_passes=%" PRIu32
                   " nvs_pass_max_us=%" PRIu32 " (since the last G line; NVS-write passes excluded from max/p99)",
              gLoop.passes(), gLoop.maxUs(), gLoop.p99Us(), gNvsPass.passes, gNvsPass.max);
#if OT_SIM_INPUT
  dbg::status(now, "dump;sim running=%u kmh_milli=%" PRIu32 " jitter=%u interval_us=%" PRId64 " next_us=%" PRId64,
              gSim.running ? 1u : 0u, gSim.kmhMilli, gSim.jitter ? 1u : 0u, gSim.baseUs, gSim.nextT);
#endif
  disp::dump(now);
#if !OT_BRINGUP
  touchhw::dump(now);
  dbg::status(now,
              "dump;gesture present=%u down=%u holding=%u hold_down_us=%" PRId64 " ignore=%s post_stop_lockout_until=%" PRId64
              " overflow=%u units=%s touch_downs=%" PRIu32 " units_toggles=%" PRIu32 " last_touch_activity_us=%" PRId64,
              gGesture.present() ? 1u : 0u, gGesture.contactDown() ? 1u : 0u, gGesture.holding() ? 1u : 0u,
              gGesture.holdDownUs(), ot::ignoreReasonName(gGesture.ignoreReason()),
              gGesture.postStopLockoutUntil() == INT64_MIN ? static_cast<int64_t>(-1) : gGesture.postStopLockoutUntil(),
              gGesture.eventOverflow() ? 1u : 0u, ot::unitsName(gUnits), gTouchDowns, gUnitsToggles,
              gLastTouchActivity);
  {
    const bool summary = gWorkout.state() == ot::WorkoutState::Summary;
    int64_t next = -1;
    if (!gUi.nextChange(now, summary, next)) next = -1;
    dbg::status(now,
                "dump;ui screen=%s level=%s last_activity_us=%" PRId64 " next_change_us=%" PRId64 " dim_us=%" PRId64
                " off_us=%" PRId64 " awake=%u",
                ot::uiScreenName(ot::screenForState(gWorkout.state())), ot::backlightName(gUi.level(now, summary)),
                gUi.lastActivityUs(), next, gUi.config().dimUs, gUi.config().offUs, screenAwake() ? 1u : 0u);
  }
  dbg::status(now,
              "dump;nvs units=%s stored=%s pending=%u last_toggle_us=%" PRId64 " last_write_us=%" PRId64
              " writes=%" PRIu32 " fails=%" PRIu32 " max_write_us=%" PRIu32 " schema_ok=%u defer_reported=%u",
              ot::unitsName(gUnits), ot::unitsName(gPersist.stored()), gPersist.pending(gUnits) ? 1u : 0u,
              gPersist.hasToggle() ? gPersist.lastToggleUs() : static_cast<int64_t>(-1),
              gPersist.hasWritten() ? gPersist.lastWriteUs() : static_cast<int64_t>(-1), gNvsWrites, gNvsFails,
              gNvsMaxUs, gNvsSchemaOk ? 1u : 0u, gPersist.deferReported() ? 1u : 0u);
#endif
#if OT_LOOPBACK
  lb::Info gi;
  lb::info(gi);
  dbg::status(now,
              "dump;gen ready=%u init_err=%d state=%u rate=%" PRIu32 " count=%" PRIu32 " enqueued=%" PRIu32
              " done=%" PRIu32 " last_tx_err=%d",
              static_cast<unsigned>(gi.ready), static_cast<int>(gi.initErr), static_cast<unsigned>(gi.state), gi.rate,
              gi.count, gi.enqueued, gi.done, static_cast<int>(gi.lastTxErr));
#endif
  bleh::dump(now);
  dbg::dumpInterrupts(now);
}

// ---------------------------------------------------------------------------
// Commands. Every accepted command is echoed as
// S,t,cmd;<text> at the instant it takes effect; anything else is
// S,t,cmd_err;<text> with no effect.
// ---------------------------------------------------------------------------
void handleCommand(int64_t now, const char* c) {
  if (strcmp(c, "v") == 0) {
    dbg::status(now, "cmd;%s", c);
    dbg::setVerbosity(static_cast<uint8_t>((dbg::verbosity() + 1u) % (dbg::LV_MAX + 1u)));
    dbg::status(now, "verbosity;%u", static_cast<unsigned>(dbg::verbosity()));
    return;
  }
  if (strcmp(c, "d") == 0) {
    dbg::status(now, "cmd;%s", c);
    dumpState(now);
    return;
  }
#if !OT_BRINGUP
  if (strcmp(c, "c") == 0) {  // raw-touch calibration mode (T lines at 10 Hz)
    dbg::status(now, "cmd;%s", c);
    touchhw::setCalMode(now, !touchhw::calMode());
    return;
  }
#endif
#if OT_DEV
  if (holdCommand(now, c)) return;
#endif
#if OT_DEV && !OT_BRINGUP
  if (c[0] == 'n' && nvsCommand(now, c)) return;  // nX, nw<k>
#endif
#if OT_BRINGUP
  if (disp::bringupCommand(now, c)) return;
#endif
#if !OT_BRINGUP && (OT_SIM_INPUT || OT_LOOPBACK)
  if (c[0] == 'x' && redrawCommand(now, c)) return;
#endif
#if OT_SIM_INPUT
  if (c[0] == 's' && simCommand(now, c)) return;
#endif
#if OT_LOOPBACK
  if (c[0] == 'g' && genCommand(now, c)) return;
#endif
  dbg::status(now, "cmd_err;%s", c);
}

uint32_t pollCommands(int64_t now) {
  uint32_t bytes = 0;
  char cmd[hw::CMD_MAX_CHARS + 1];
  for (uint32_t i = 0; i < CMDS_PER_PASS; ++i) {
    uint32_t got = 0;
    const dbg::ReadResult r = dbg::readCommand(cmd, sizeof(cmd), CMD_BYTES_PER_PASS - bytes, got);
    bytes += got;
    if (r == dbg::ReadResult::Line) handleCommand(now, cmd);
    else if (r == dbg::ReadResult::TooLong) dbg::status(now, "cmd_err;too_long");
    else break;
    if (bytes >= CMD_BYTES_PER_PASS) break;
  }
  return bytes;
}

// ---------------------------------------------------------------------------
// Periodic lines
// ---------------------------------------------------------------------------
void advance(int64_t& next, int64_t period, int64_t now) {
  next += period;
  if (next <= now) next = now + period;  // after a long block: no burst of catch-up lines
}

void periodic(int64_t now) {
  const uint32_t drops = hwp::droppedEdges();
  if (drops != gReportedEdgeDrops && now - gLastDropWarnT >= hw::DROP_WARN_MIN_US) {
    dbg::status(now, "WARNING edge ring overflow: %" PRIu32 " edges dropped total (+%" PRIu32 ")", drops,
                drops - gReportedEdgeDrops);
    gReportedEdgeDrops = drops;
    gLastDropWarnT = now;
  }
  if (now >= gNextK) {
    // K,t,state,odo_pulses,wk_pulses,speed_mmps,moving_ms,total_ms,top_mmps
    // One snapshot (the caller ticked the Workout to `now`); while a hold is
    // active the workout fields are the hold-begin values, speed stays live.
    // The BLE tick (NOTIFY_PERIOD_US = K_PERIOD_US) encodes RSC
    // and FTMS from exactly these values (R/X lines right after K).
    const ot::WorkoutSnapshot ws = gWorkout.snapshot(now);
    ot::ble::BleSnapshot bs;
    bs.t         = now;
    bs.speedMmps = gMetrics.speedMmps(now);
    bs.odoPulses = gMetrics.odometerPulses();
    bs.state     = ws.state;
    bs.wkPulses  = ws.wkPulses;
    bs.totalUs   = ws.totalUs;
    bs.workoutId = ws.workoutId;
    dbg::line(dbg::LV_K, "K,%" PRId64 ",%s,%" PRIu32 ",%" PRIu32 ",%" PRId32 ",%" PRId64 ",%" PRId64 ",%" PRId32, now,
              ot::workoutStateName(ws.state), bs.odoPulses, ws.wkPulses, bs.speedMmps, ws.movingUs / 1000,
              ws.totalUs / 1000, ws.topMmps);
    bleh::tick(bs);
    advance(gNextK, hw::K_PERIOD_US, now);
  }
  if (now >= gNextH) {
    // H,t,accepted_pulses,isr_edges,drop_edges,0,debug_drop,subscribed_peers,line_level
    dbg::line(dbg::LV_ALWAYS, "H,%" PRId64 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",0,%" PRIu32 ",%u,%u", now, gPulseN,
              gEdgeN, drops, dbg::droppedLines(), static_cast<unsigned>(bleh::subscribedPeers()),
              static_cast<unsigned>(hwp::readPulseLevel()));
    advance(gNextH, hw::HEARTBEAT_US, now);
  }
  if (now >= gNextStats) {
    dbg::heap(now);
    // G,t,loop_max_us,loop_p99_us,render_max_us,render_count,touch_poll_late_max_us,nvs_pass_max_us,nvs_passes
    // Renderer passes that drew (bring-up builds: the bring-up draws and touch
    // polls); touch_poll_late_max_us from hw_touch. loop_max_us /
    // loop_p99_us exclude the passes that wrote NVS: those are the
    // trailing nvs_pass_max_us / nvs_passes (appended fields: older parsers
    // that read the first 5 still work). Reset after each G.
    uint32_t renderMax = 0, renderCount = 0, touchLate = 0;
    disp::takeStats(renderMax, renderCount, touchLate);
#if !OT_BRINGUP
    touchLate = touchhw::takeLateMaxUs();  // the 20 ms touch poll's worst lateness
#endif
    dbg::line(dbg::LV_ALWAYS, "G,%" PRId64 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
              ",%" PRIu32, now, gLoop.maxUs(), gLoop.p99Us(), renderMax, renderCount, touchLate, gNvsPass.max,
              gNvsPass.passes);
    gLoop.reset();
    gNvsPass.reset();
    advance(gNextStats, hw::STATS_PERIOD_US, now);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
void app_setup() {
  // CYD RGB LED (active LOW): off. Never Wire.begin() (SDA = GPIO 21 = backlight).
  pinMode(hw::LED_R_PIN, OUTPUT);
  pinMode(hw::LED_G_PIN, OUTPUT);
  pinMode(hw::LED_B_PIN, OUTPUT);
  digitalWrite(hw::LED_R_PIN, HIGH);
  digitalWrite(hw::LED_G_PIN, HIGH);
  digitalWrite(hw::LED_B_PIN, HIGH);

  dbg::begin(static_cast<uint8_t>(OT_VERBOSITY_DEFAULT));

#if OT_LOOPBACK
  // Before the GPIO 27 handler exists, so a set-up glitch on GPIO 22 is never logged.
  lb::begin();
#endif

  hwp::begin(OT_SIM_INPUT == 0, gInit);  // sim: GPIO 27 handler not installed (exclusive sources)
  gFilter.reset(gInit.t0, gInit.pulseLevel);
  gMarkerFilter.reset(gInit.t0, gInit.markerLevel);
  gMetrics.reset();
  gWorkout.reset();

  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  esp_read_mac(mac, ESP_MAC_BT);
  const int64_t t0 = gInit.t0;
  dbg::status(t0,
              "OpenTread fw=%s build=%08" PRIx32 " dirty=%u pin=%u lockout_us=%" PRId64 " release_us=%" PRId64
              " baud=%" PRIu32 " line_level=%u ring=%" PRIu32 " marker_pin=%u reset_reason=%d isr_iram=%u"
              " bt_addr=%02x:%02x:%02x:%02x:%02x:%02x lgfx=%s nimble=%s sim=%u dev=%u short=%u ftms=%u adv=%s"
              " loopback=%u bringup=%u verb=%u",
              OT_FW_VERSION, hw::BUILD_ID, static_cast<unsigned>(hw::BUILD_DIRTY), static_cast<unsigned>(hw::PULSE_PIN),
              ot::LOCKOUT_US, ot::RELEASE_US, hw::SERIAL_BAUD, static_cast<unsigned>(gInit.pulseLevel),
              ot::EDGE_RING_SIZE, static_cast<unsigned>(hw::MARKER_PIN), static_cast<int>(esp_reset_reason()),
              static_cast<unsigned>(gInit.isrIram), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], LGFX_BANNER,
              bleh::nimbleVersion(), static_cast<unsigned>(OT_SIM_INPUT), static_cast<unsigned>(OT_DEV),
              static_cast<unsigned>(OT_SHORT_TIMERS), static_cast<unsigned>(OT_FTMS_ENABLE), bleh::advLayoutName(),
              static_cast<unsigned>(OT_LOOPBACK), static_cast<unsigned>(OT_BRINGUP),
              static_cast<unsigned>(dbg::verbosity()));
  dbg::status(t0,
              "format: E;t;level P;t;n;dt_us M;t;n S;t;text H;t;pulses;edges;drop_edges;0;debug_drop;peers;level"
              " K;t;state;odo;wk;speed_mmps;moving_ms;total_ms;top_mmps D;t;free;min_free;largest_8bit;largest_dma"
              " G;t;loop_max_us;loop_p99_us;render_max_us;render_count;touch_late_max_us;nvs_pass_max_us;nvs_passes");
  dbg::status(t0,
              "format: B;t;event;conn;detail R;t;rsc_hex X;t;ftms_treadmill_data_hex (R/X: the 1 Hz BLE payloads"
              " encoded from the K line's snapshot)");
  dbg::status(t0,
              "format: W;t;from;to;reason I;t;event;arg Z;t;reason;wk_pulses;moving_us;total_us;top_mmps;avg_mmps;"
              "workout_id T;t;x_raw;y_raw;z;kind/sx/sy");  // T: 6 comma fields, the last is <kind>;<sx>;<sy>
#if OT_DEV
  dbg::status(t0, "dev mode: hold commands hb (begin) hc (complete) ha (abort); each logs its I line");
#endif
#if OT_DEV && !OT_BRINGUP
  dbg::status(t0, "dev mode: NVS commands nX (write units=7: next boot defaults) nw<k> (k forced commits; 1..1000)");
#endif
  if (hw::BUILD_ID == 0)
    dbg::status(t0, "WARNING build id unknown (OT_BUILD_ID=0): not built by tools/build.sh; build not traceable");
  if (!gInit.isrIram)
    dbg::status(t0,
                "ERROR isr_iram=0: service_err=%d pulse_add_err=%d marker_add_err=%d placement=%u%u%u%u (build unusable)",
                static_cast<int>(gInit.installErr), static_cast<int>(gInit.pulseAddErr),
                static_cast<int>(gInit.markerAddErr), static_cast<unsigned>(gInit.pulseIsrInIram),
                static_cast<unsigned>(gInit.markerIsrInIram), static_cast<unsigned>(gInit.pushEdgeInIram),
                static_cast<unsigned>(gInit.ringInDram));
#if OT_SIM_INPUT
  dbg::status(t0, "sim mode: GPIO 27 handler NOT installed; commands s<kmh> sj<kmh> s0 (km/h 0.1..60)");
#endif
#if !OT_BRINGUP && (OT_SIM_INPUT || OT_LOOPBACK)
  dbg::status(t0, "bench display commands: x1 (every frame pushes every field) x0 (dirty fields only)");
#endif
#if !OT_BRINGUP
  // The units from NVS (read-only; the default when nothing valid is
  // stored), before the first frame so it shows the right units.
  nvsBoot();
  dbg::status(esp_timer_get_time(),
              "display;units=%s (NVS, default %s; tap the pace / idle units label to toggle; saved to NVS"
                  " after %" PRId64 " us when stationary)",
              ot::unitsName(gUnits), ot::unitsName(UNITS_BOOT), ot::NVS_SETTLE_US);
#endif
#if OT_LOOPBACK
  {
    lb::Info gi;
    lb::info(gi);
    dbg::status(t0,
                "loopback generator pin=%u ready=%u init_err=%d: jumper GPIO %u -> GPIO %u with the harness unplugged;"
                " commands g<rate 1..40>,<count> g0",
                static_cast<unsigned>(hw::LOOPBACK_PIN), static_cast<unsigned>(gi.ready), static_cast<int>(gi.initErr),
                static_cast<unsigned>(hw::LOOPBACK_PIN), static_cast<unsigned>(hw::PULSE_PIN));
  }
#endif
  // After the banner (and after hwp::begin: the ring buffers edges while the
  // panel initialises). Display, touch and UI timers first; BLE after.
#if OT_BRINGUP
  disp::bringupBegin();
#else
  // The first frame is the boot state (IDLE, no pulse yet). The Workout is
  // NOT ticked here: edges buffered during the panel init carry earlier
  // timestamps, and a tick at this later instant would clamp them (input
  // time clamp), so the device would diverge from the P-line replay.
  // snapshot() and speedMmps() are const.
  disp::begin(makeFrame(esp_timer_get_time()));
  // Touch after the panel init (the XPT2046 driver is set up by lcd.init()).
  // A contact already valid at the first polls is ignored until UP (boot guard).
  touchhw::begin(esp_timer_get_time());
  // The idle timers start when the first frame is lit. The S line
  // carries that exact instant (tools/replay.py l_backlight starts there).
  {
    const int64_t tr = esp_timer_get_time();
    gUi.reset(tr);
    dbg::status(tr, "ui;reset dim_us=%" PRId64 " off_us=%" PRId64 " bl_full=%u bl_dim=%u bl_off=%u short=%u",
                gUi.config().dimUs, gUi.config().offUs, static_cast<unsigned>(hw::BL_FULL),
                static_cast<unsigned>(hw::BL_DIM), static_cast<unsigned>(hw::BL_OFF),
                static_cast<unsigned>(OT_SHORT_TIMERS));
  }
#endif
  // BLE after the display, its sprites and the first frame (so
  // the sprites are allocated before NimBLEDevice::init). A failure is logged
  // (ERROR ble;...) and the head unit keeps running without BLE.
  bleh::begin(esp_timer_get_time());

  const int64_t now = esp_timer_get_time();
#if !OT_BRINGUP
  // The ~270 ms NimBLE init above lies between touchhw::begin and the first
  // loop() poll: re-baseline the 20 ms schedule here so the first G line's
  // touch_poll_late_max_us covers loop() polls only (the boot gap gets its
  // own S line field, like boot_render_max_us).
  touchhw::loopStart(now);
#endif
  gNextK     = now + hw::K_PERIOD_US;
  gNextH     = now + hw::HEARTBEAT_US;
  gNextStats = now + hw::STATS_PERIOD_US;
}

void app_loop() {
#if OT_LOOPBACK
  // Before `now` is read: if the train ended (done callback ran), every edge
  // it generated has t < now and is drained this pass, before gen_done.
  uint32_t generated = 0;
  const bool genEnded = lb::service(generated);
#endif
  const int64_t now = esp_timer_get_time();
  uint32_t work = drainEdges(now);
#if OT_SIM_INPUT
  work += simInject(now);
#endif
  workoutTick(now);
  work += pollCommands(now);
#if OT_SIM_INPUT
  work += simInject(now);  // a command may have scheduled a pulse at exactly `now`
#endif
  workoutTick(now);        // idempotent; K below reads the snapshot at `now`
#if OT_LOOPBACK
  if (genEnded) dbg::status(now, "gen_done;%" PRIu32, generated);
#endif
#if OT_BRINGUP
  work += bleh::service(now);
  work += disp::bringupService(now);
#else
  work += touchService(now);                             // may change the Workout (hold) and the units
  work += bleh::service(now);                            // B lines, advertising, CP indications
  uiService(now);                                        // backlight (a wake makes the frame due now)
  if (disp::frameDue(now)) disp::frame(makeFrame(now));  // one snapshot per frame
  const uint32_t drew = disp::service(now);              // bounded pushes; 1 if it drew
  work += drew;
  if (drew == 0) work += persistService(now);            // NVS commit when due (stationary, no push this pass)
#endif
  periodic(now);

  const int64_t passUs = esp_timer_get_time() - now;
  const uint32_t passU32 =
      passUs < 0 ? 0u : (passUs > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(passUs));
  if (gPassNvsWrite) gNvsPass.record(passU32);  // NVS-write passes reported separately
  else gLoop.record(passU32);
  gPassNvsWrite = false;
  if (work == 0) vTaskDelay(1);  // idle: yield one tick; edges keep buffering in the ring
}
