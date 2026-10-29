// touch_gesture.h - touch debounce + gesture recognizer ("tap the pace
// number" toggles the units, "hold ~2 s to stop" ends the workout).
//
// Input: one TouchSample per poll (every TOUCH_POLL_US = 20 ms on the device),
// already calibrated to screen pixels by hw_touch, with valid = pressure >=
// TOUCH_Z_MIN. The context (screen id, screenAwake, paceRect, stopRect,
// stopVisible, lockoutUntil) is set before every sample. Output: a queue of
// GestureEvents with exact timestamps (sample times), which the app turns into
// Workout / units actions and I lines.
//
// Definitions (all times are sample timestamps, int64 us):
//  - Presence: valid samples whose gaps are < TOUCH_RELEASE_MS. It begins at a
//    valid sample when no presence is active and ends (UP) at the first sample
//    with t - lastValid >= TOUCH_RELEASE_MS (so dropouts < 120 ms are
//    bridged). Every guard below that says "until UP" lasts until this end,
//    whether or not the presence ever reached DOWN.
//  - DOWN: TOUCH_PRESS_N (3) CONSECUTIVE valid samples within a presence
//    (an invalid sample before that restarts the count). tDown = the first of
//    the 3; the DOWN point = the per-axis median of the 3. At most one DOWN
//    per presence.
//  - Drift: Euclidean distance of a valid sample from the DOWN point (only
//    valid samples count; the 3 DOWN samples are checked too).
//  - Duration of a contact: lastValid - tDown.
//
// Gestures:
//  | Wake         | !screenAwake at DOWN: Wake event; the contact is swallowed until UP. |
//  | TapPace      | DOWN point and last valid point inside paceRect, duration 40..700 ms,  |
//  |              | max drift < TAP_SLOP_PX, nothing ignored; reported at UP.              |
//  | HoldStart    | ALL 3 DOWN samples (hence the DOWN point) inside stopRect while      |
//  |              | stopVisible (a finger that lands outside and slides in never starts a |
//  |              | hold), and the presence began while the button was visible (a        |
//  |              | presence that was active when the screen changed or the button       |
//  |              | appeared is ignored until UP). t = tDown.                             |
//  | HoldProgress | holdPermille(now) = min(1000, (now - tDown) * 1000 / HOLD_STOP_US)    |
//  |              | (a query for the renderer, not an event).                             |
//  | HoldCancel   | UP before completion, drift > HOLD_SLOP_PX (20 px), a screen          |
//  |              | change / the button disappearing, or the stuck guard; the rest of the |
//  |              | contact is then ignored until UP (no tap, no new hold).                |
//  | HoldComplete | a VALID sample with t - tDown >= HOLD_STOP_US while holding (a       |
//  |              | dropout being bridged never completes a hold: the finger must be seen |
//  |              | at >= 2.000 s). Then everything is ignored until UP, and any contact  |
//  |              | whose PRESENCE BEGAN before t + POST_STOP_LOCKOUT_MS is ignored until |
//  |              | its UP, even if its DOWN comes later.                                 |
//  | Stuck        | the first sample(s) after reset() are valid (contact at boot), or the |
//  |              | presence lasts > STUCK_TOUCH_MS: ignored until UP (Stuck event at the |
//  |              | 10 s crossing); it can never produce a tap or start a hold.            |
//  | Lockout      | a contact whose presence began before max(ctx.lockoutUntil, the      |
//  |              | post-stop lockout) is ignored until UP (checked at its DOWN; a bouncy |
//  |              | finger that only reaches DOWN after the window is still ignored).     |
//
// Every debounced contact reports TouchDown (t = tDown, the DOWN point, and
// the reason it is ignored, if any) and TouchUp (t = the UP sample, the last
// valid point). The app reports activity (backlight) at every TouchDown.
//
// Pure C++17, integer only, no allocation, no clock reads.
#pragma once

#include <cstdint>

#include "config.h"

namespace ot {

struct TouchSample {
  int64_t  t     = 0;      // poll instant (us)
  bool     valid = false;  // pressure >= TOUCH_Z_MIN
  int16_t  x     = 0;      // screen px (landscape 320x240), valid samples only
  int16_t  y     = 0;
  uint16_t z     = 0;      // pressure (diagnostic)
};

// Half-open pixel rectangle [x, x+w) x [y, y+h). w or h <= 0 = empty (never hit).
struct TouchRect {
  int16_t x = 0, y = 0, w = 0, h = 0;
  constexpr bool contains(int32_t px, int32_t py) const {
    return w > 0 && h > 0 && px >= x && px < x + w && py >= y && py < y + h;
  }
};

// Production touch targets (the renderer static_asserts that they match its
// layout):
//  - PACE_RECT: the hero rect y 24-119, full width. Main screen: the pace
//    number; idle screen: the units label (units can be set before a workout).
//  - STOP_RECT: "strip only": a hold may START only on the drawn HOLD TO
//    STOP strip, x 12-307, y 202-227. The cells, the speed row and the hero
//    never start a hold (a resting palm, forearm, phone or sweat drop there
//    must not stop the workout). The bezel edge is excluded by a 12 px margin
//    at the bottom and at the left/right panel edges (the resistive
//    calibration error is 6-9 px, so 12 px keeps a thumb resting on the bezel
//    lip out of the rect). The drawn box IS this rect (hw_display static_asserts
//    it). A started hold may still wobble 20 px from its DOWN point
//    (HOLD_SLOP_PX), i.e. up into the cells.
constexpr int16_t HOLD_EDGE_MARGIN_PX = 12;
constexpr int16_t HOLD_STRIP_TOP_Y = 202;  // the strip's top edge (cells end at y 199)
constexpr TouchRect PACE_RECT = {0, 24, 320, 96};
constexpr TouchRect STOP_RECT = {HOLD_EDGE_MARGIN_PX, HOLD_STRIP_TOP_Y, 320 - 2 * HOLD_EDGE_MARGIN_PX,
                                 240 - HOLD_STRIP_TOP_Y - HOLD_EDGE_MARGIN_PX};
static_assert(STOP_RECT.x == 12 && STOP_RECT.y == 202 && STOP_RECT.w == 296 && STOP_RECT.h == 26,
              "STOP_RECT = the strip x 12-307, y 202-227");

// Screen ids for GestureContext::screen (any change = "screen change").
enum : uint8_t { TOUCH_SCREEN_NONE = 0, TOUCH_SCREEN_IDLE = 1, TOUCH_SCREEN_MAIN = 2, TOUCH_SCREEN_SUMMARY = 3 };

struct GestureContext {
  uint8_t   screen       = TOUCH_SCREEN_NONE;
  bool      screenAwake  = true;   // panel lit, FULL or DIM, no wake redraw pending (ot::touchAwake)
  TouchRect paceRect;              // empty = no units tap on this screen
  TouchRect stopRect;              // where a hold may start (all 3 DOWN samples inside, if stopVisible)
  bool      stopVisible  = false;  // the HOLD TO STOP button is shown (WARMUP/RUNNING/PAUSED)
  int64_t   lockoutUntil = INT64_MIN;  // app-imposed: a contact whose presence began before this is ignored until UP
};

enum class GestureType : uint8_t {
  TouchDown,     // t = tDown; x/y = DOWN point; reason = why the contact is ignored (None = live)
  TouchUp,       // t = UP sample; x/y = last valid point; reason = the contact's final ignore reason
  Wake,          // t = tDown: contact while the screen is not awake (panel OFF / waking; swallowed until UP)
  TapPace,       // t = UP sample: toggle units
  HoldStart,     // t = tDown -> Workout.holdBegan(tDown)
  HoldCancel,    // t = the sample (or context change) that cancelled -> Workout.holdAborted(t)
  HoldComplete,  // t = the valid sample at >= tDown + HOLD_STOP_US -> Workout.holdCompleted(t)
  Stuck,         // t = the sample at which the presence passed STUCK_TOUCH_MS (diagnostic)
};

enum class IgnoreReason : uint8_t {
  None,          // live contact
  Boot,          // valid at the very first sample(s) after reset()
  ScreenChange,  // present when the screen changed / the button appeared / it vanished mid-hold
  Lockout,       // presence began inside the post-stop or app lockout
  Wake,          // swallowed: the screen was not awake at DOWN
  Stuck,         // presence > STUCK_TOUCH_MS
  HoldEnded,     // the rest of a contact whose hold was cancelled or completed
};

struct GestureEvent {
  GestureType  type   = GestureType::TouchDown;
  int64_t      t      = 0;
  int64_t      tDown  = 0;   // the contact's DOWN instant (0 if none)
  int16_t      x      = 0;
  int16_t      y      = 0;
  IgnoreReason reason = IgnoreReason::None;
};

const char* gestureTypeName(GestureType g);     // "touch_down", "tap_pace", ...
const char* ignoreReasonName(IgnoreReason r);   // "none", "boot", ...

class TouchGesture {
 public:
  static constexpr uint32_t EVENT_QUEUE = 8;

  TouchGesture() { reset(); }
  void reset();  // boot state: no presence, no lockout, the boot-contact guard armed

  // Set before every sample (the app derives it from the Workout state at the
  // same instant). A change of `screen`, or stopVisible going true, while a
  // presence is active ignores that presence until UP; a hold in progress is
  // cancelled (HoldCancel at `now`) on a screen change or when stopVisible
  // goes false.
  void setContext(const GestureContext& ctx, int64_t now);
  void onSample(const TouchSample& s);

  bool pollEvent(GestureEvent& e);
  bool eventOverflow() const { return overflow_; }

  bool         present() const { return present_; }
  bool         contactDown() const { return down_; }
  bool         holding() const { return holding_; }
  int64_t      holdDownUs() const { return holding_ ? tDown_ : 0; }
  int32_t      holdPermille(int64_t now) const;  // -1 if not holding, else 0..1000
  IgnoreReason ignoreReason() const { return ignore_; }
  int64_t      postStopLockoutUntil() const { return lockoutUntil_; }

 private:
  void startPresence(int64_t t);
  void endPresence(int64_t t);
  void becomeDown(int64_t t);
  void track(int32_t x, int32_t y, int64_t t);
  void checkStuck(int64_t t);
  void cancelHold(int64_t t, IgnoreReason why);
  void push(GestureType type, int64_t t, int32_t x, int32_t y, IgnoreReason reason);

  GestureContext ctx_;
  bool    sawInvalid_   = false;  // an invalid sample was seen since reset() (boot guard)
  int64_t lockoutUntil_ = INT64_MIN;

  // Presence / contact
  bool         present_   = false;
  int64_t      presenceT_ = 0;
  int64_t      lastValidT_ = 0;
  int16_t      lastX_ = 0, lastY_ = 0;
  uint32_t     run_ = 0;          // consecutive valid samples before DOWN
  int64_t      runT_[TOUCH_PRESS_N] = {};
  int16_t      runX_[TOUCH_PRESS_N] = {};
  int16_t      runY_[TOUCH_PRESS_N] = {};
  bool         down_ = false;
  int64_t      tDown_ = 0;
  int16_t      downX_ = 0, downY_ = 0;
  int32_t      maxDrift2_ = 0;
  bool         holding_ = false;
  bool         tapCand_ = false;
  bool         stuckReported_ = false;
  IgnoreReason ignore_ = IgnoreReason::None;

  GestureEvent q_[EVENT_QUEUE];
  uint32_t     qHead_ = 0;
  uint32_t     qCount_ = 0;
  bool         overflow_ = false;
};

}  // namespace ot
