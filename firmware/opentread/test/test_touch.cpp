// test_touch.cpp - host tests for src/core/touch_gesture.*: tests 4.1-4.13
// (table-driven sample streams). A hold may START only on the HOLD TO STOP
// strip, STOP_RECT = x 12-307, y 202-227 (12 px bezel margins), with all 3
// DOWN samples inside it; a started hold cancels on a drift > 20 px from the
// DOWN point. The post-stop lockout keys on the presence start. 4.12 has a
// fifth family (e): stationary contacts anywhere off the strip.
//
// Mutation resistance is designed in: every guard of the recognizer has a test
// that fails when the guard is removed or loosened, and 4.12 runs an adversarial set
// (0 HoldComplete allowed) AND a control set (every press must complete), so it
// fails both when a guard is missing and when one is too strict.
//
// Usage: test_touch [--root <repo root>] [--verbose]
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "config.h"
#include "touch_gesture.h"

using ot::GestureContext;
using ot::GestureEvent;
using ot::GestureType;
using ot::IgnoreReason;
using ot::TouchGesture;
using ot::TouchSample;

namespace {

bool gVerbose = false;
constexpr int64_t MS = 1000;
constexpr int64_t SEC = 1000000;
constexpr int64_t P = ot::TOUCH_POLL_US;  // 20 ms

// Points on the production layout.
constexpr int PACE_X = 160, PACE_Y = 70;    // hero (pace number)
constexpr int STOP_X = 160, STOP_Y = 214;   // HOLD TO STOP strip (STOP_RECT y 202-227)
constexpr int CELL_X = 53, CELL_Y = 177;    // distance cell: no action (never starts a hold)
constexpr int SPEED_X = 160, SPEED_Y = 136; // speed row: no action

GestureContext ctxMain() {
  GestureContext c;
  c.screen = ot::TOUCH_SCREEN_MAIN;
  c.screenAwake = true;
  c.paceRect = ot::PACE_RECT;
  c.stopRect = ot::STOP_RECT;
  c.stopVisible = true;
  return c;
}
GestureContext ctxIdle() {
  GestureContext c;
  c.screen = ot::TOUCH_SCREEN_IDLE;
  c.screenAwake = true;
  c.paceRect = ot::PACE_RECT;
  c.stopRect = ot::STOP_RECT;
  c.stopVisible = false;
  return c;
}

// A recognizer fed sample by sample; collects every event.
struct Rig {
  TouchGesture g;
  GestureContext ctx = ctxMain();
  std::vector<GestureEvent> ev;
  int64_t t = 0;  // time of the last sample fed

  explicit Rig(bool bootClean = true) {
    if (bootClean) none(0);  // an invalid first sample: no contact at boot
  }
  void drain() {
    GestureEvent e;
    while (g.pollEvent(e)) ev.push_back(e);
  }
  void feed(int64_t at, bool valid, int x, int y) {
    g.setContext(ctx, at);
    drain();
    TouchSample s;
    s.t = at;
    s.valid = valid;
    s.x = static_cast<int16_t>(x);
    s.y = static_cast<int16_t>(y);
    s.z = valid ? 40 : 0;
    g.onSample(s);
    drain();
    t = at;
  }
  void none(int64_t at) { feed(at, false, 0, 0); }
  // Invalid samples every P from `from` (inclusive) until `to` (exclusive).
  void gap(int64_t from, int64_t to) {
    for (int64_t k = from; k < to; k += P) none(k);
  }
  // A still press: valid samples at t0, t0+step, ... up to and including
  // t0 + dur (the last sample exactly at t0 + dur), then invalid samples
  // every P until 200 ms after the last valid one (UP at +120 ms).
  void press(int64_t t0, int64_t dur, int x, int y, int64_t step = P) {
    int64_t k = t0;
    for (; k < t0 + dur; k += step) feed(k, true, x, y);
    feed(t0 + dur, true, x, y);
    gap(t0 + dur + P, t0 + dur + 200 * MS);
  }
  int count(GestureType type) const {
    int n = 0;
    for (const GestureEvent& e : ev)
      if (e.type == type) ++n;
    return n;
  }
  const GestureEvent* first(GestureType type) const {
    for (const GestureEvent& e : ev)
      if (e.type == type) return &e;
    return nullptr;
  }
  void clear() { ev.clear(); }
};

// ---------------------------------------------------------------------------
// Deterministic RNG (splitmix64), so the seeded sets are identical everywhere.
// ---------------------------------------------------------------------------
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed) {}
  uint64_t next() {
    uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  // uniform integer in [lo, hi]
  int64_t range(int64_t lo, int64_t hi) {
    const uint64_t span = static_cast<uint64_t>(hi - lo) + 1u;
    return lo + static_cast<int64_t>(next() % span);
  }
  bool chance(int64_t pctTimes10) { return range(0, 999) < pctTimes10; }
};

// Integer trig-free direction: a random unit vector scaled by 1000 (rejection
// sampling in the unit disc, then normalised with an integer sqrt).
int64_t isqrt(int64_t v) {
  if (v <= 0) return 0;
  int64_t r = 0, bit = int64_t{1} << 62;
  while (bit > v) bit >>= 2;
  while (bit != 0) {
    if (v >= r + bit) {
      v -= r + bit;
      r = (r >> 1) + bit;
    } else {
      r >>= 1;
    }
    bit >>= 2;
  }
  return r;
}
void randomDir(Rng& r, int64_t& ux, int64_t& uy) {  // |(ux,uy)| ~= 1000
  for (;;) {
    const int64_t x = r.range(-1000, 1000), y = r.range(-1000, 1000);
    const int64_t n2 = x * x + y * y;
    if (n2 < 250000 || n2 > 1000000) continue;
    const int64_t n = isqrt(n2);
    ux = x * 1000 / n;
    uy = y * 1000 / n;
    return;
  }
}
// round(a / b) for b > 0
int64_t rdiv(int64_t a, int64_t b) { return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b); }

bool onPanel(int64_t x, int64_t y) { return x >= 0 && x < 320 && y >= 0 && y < 240; }

// ===========================================================================
// 4.1 Clean tap on pace, 150 ms -> 1 TapPace (+ boundaries, spam)
// ===========================================================================
void test41() {
  {
    Rig r;
    r.press(1 * SEC, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
    CHECK_EQ(r.count(GestureType::TouchDown), 1);
    CHECK_EQ(r.count(GestureType::TouchUp), 1);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    const GestureEvent* d = r.first(GestureType::TouchDown);
    if (CHECK(d != nullptr)) {
      CHECK_EQ(d->t, 1 * SEC);  // tDown = the first of the 3 DOWN samples
      CHECK_EQ(static_cast<int>(d->x), PACE_X);
      CHECK_EQ(static_cast<int>(d->y), PACE_Y);
      CHECK(d->reason == IgnoreReason::None);
    }
    const GestureEvent* tp = r.first(GestureType::TapPace);
    if (CHECK(tp != nullptr)) {
      CHECK_EQ(tp->t, 1 * SEC + 150 * MS + 120 * MS);  // reported at UP (120 ms after the last valid)
      CHECK_EQ(tp->tDown, 1 * SEC);
    }
  }
  // Duration boundaries (TAP_MIN_MS 40 .. 700 ms inclusive), custom sample times.
  // 40 ms = the 3-sample DOWN itself (0/20/40 ms): every registered contact
  // qualifies (final hardware session 2026-10-27: quick 40 ms taps failed at 60).
  struct Row { int64_t dur; int taps; };
  for (const Row& row : {Row{39 * MS, 0}, Row{40 * MS, 1}, Row{60 * MS, 1}, Row{700 * MS, 1}, Row{701 * MS, 0}}) {
    Rig r;
    const int64_t t0 = 1 * SEC;
    r.feed(t0, true, PACE_X, PACE_Y);
    r.feed(t0 + (row.dur < 40 * MS ? row.dur / 2 : 20 * MS), true, PACE_X, PACE_Y);
    for (int64_t k = t0 + 40 * MS; k < t0 + row.dur; k += P) r.feed(k, true, PACE_X, PACE_Y);
    r.feed(t0 + row.dur, true, PACE_X, PACE_Y);
    r.gap(t0 + row.dur + P, t0 + row.dur + 300 * MS);
    if (!CHECK_EQ(r.count(GestureType::TapPace), row.taps)) std::fprintf(stderr, "  tap dur %lld\n", static_cast<long long>(row.dur));
  }
  // Drift boundary: < 40 px taps, 40 px does not (Euclidean, from the DOWN point).
  for (const int d : {39, 40}) {
    Rig r;
    for (int k = 0; k < 3; ++k) r.feed(1 * SEC + k * P, true, PACE_X, PACE_Y);
    r.feed(1 * SEC + 3 * P, true, PACE_X + d, PACE_Y);
    r.feed(1 * SEC + 4 * P, true, PACE_X, PACE_Y);
    r.gap(1 * SEC + 5 * P, 1 * SEC + 400 * MS);
    CHECK_EQ(r.count(GestureType::TapPace), d < 40 ? 1 : 0);
  }
  {  // 3-4-5 triangle: drift 40 exactly in Euclidean terms (24, 32) -> no tap; (24, 31) -> 39.2 -> tap
    for (const int dy : {31, 32}) {
      Rig r;
      for (int k = 0; k < 3; ++k) r.feed(1 * SEC + k * P, true, PACE_X, PACE_Y);
      r.feed(1 * SEC + 3 * P, true, PACE_X + 24, PACE_Y + dy);
      r.gap(1 * SEC + 4 * P, 1 * SEC + 400 * MS);
      CHECK_EQ(r.count(GestureType::TapPace), dy == 31 ? 1 : 0);
    }
  }
  {  // Taps elsewhere do nothing: the stop strip (starts a hold that is cancelled), the speed row, a cell
     // (a cell never starts a hold).
    Rig r;
    r.press(1 * SEC, 150 * MS, SPEED_X, SPEED_Y);
    r.press(2 * SEC, 150 * MS, STOP_X, STOP_Y);
    r.press(3 * SEC, 150 * MS, CELL_X, CELL_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    CHECK_EQ(r.count(GestureType::HoldStart), 1);
    CHECK_EQ(r.count(GestureType::HoldCancel), 1);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
  }
  {  // Idle screen: the units label in the hero rect toggles too; summary: no tap (paceRect empty).
    Rig r;
    r.ctx = ctxIdle();
    r.press(1 * SEC, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
    r.ctx.screen = ot::TOUCH_SCREEN_SUMMARY;
    r.ctx.paceRect = ot::TouchRect{};
    r.none(2 * SEC);
    r.press(3 * SEC, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
  }
  {  // Toggle spam (adversarial brief): 100 taps in 30 s -> 100 toggles, no overflow.
    Rig r;
    for (int i = 0; i < 100; ++i) r.press(1 * SEC + i * 300 * MS, 100 * MS, PACE_X + (i % 7) - 3, PACE_Y + (i % 5) - 2);
    CHECK_EQ(r.count(GestureType::TapPace), 100);
    CHECK(!r.g.eventOverflow());
  }
}

// ===========================================================================
// 4.2 Bounce
// ===========================================================================
void test42() {
  {  // valid/invalid alternating every 20 ms for 200 ms: never 3 consecutive valid -> no DOWN, 0 taps
    Rig r;
    for (int k = 0; k < 10; ++k) r.feed(1 * SEC + k * P, (k % 2) == 0, PACE_X, PACE_Y);
    r.gap(1 * SEC + 10 * P, 2 * SEC);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    CHECK_EQ(r.count(GestureType::TouchDown), 0);
  }
  {  // 2 valid / 1 invalid repeating for 300 ms: still never 3 consecutive -> 0 taps (guards N = 3)
    Rig r;
    for (int k = 0; k < 15; ++k) r.feed(1 * SEC + k * P, (k % 3) != 2, PACE_X, PACE_Y);
    r.gap(1 * SEC + 15 * P, 2 * SEC);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    CHECK_EQ(r.count(GestureType::TouchDown), 0);
  }
  {  // 3 valid / 1 invalid repeating for 200 ms -> exactly 1 tap (the 20 ms gaps are bridged)
    Rig r;
    for (int k = 0; k < 10; ++k) r.feed(1 * SEC + k * P, (k % 4) != 3, PACE_X, PACE_Y);
    r.gap(1 * SEC + 10 * P, 2 * SEC);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
    CHECK_EQ(r.count(GestureType::TouchDown), 1);
    CHECK_EQ(r.count(GestureType::TouchUp), 1);
  }
  {  // DOWN point = per-axis median of the 3 DOWN samples (median at each position in turn)
    struct Row { int x[3], y[3], mx, my; };
    for (const Row& row : {Row{{100, 140, 120}, {60, 50, 90}, 120, 60},    // x: last,   y: first
                           Row{{120, 140, 100}, {50, 60, 90}, 120, 60},    // x: first,  y: middle
                           Row{{100, 120, 140}, {90, 50, 60}, 120, 60}}) { // x: middle, y: last
      Rig r;
      for (int k = 0; k < 3; ++k) r.feed(1 * SEC + k * P, true, row.x[k], row.y[k]);
      const GestureEvent* d = r.first(GestureType::TouchDown);
      if (CHECK(d != nullptr)) {
        CHECK_EQ(static_cast<int>(d->x), row.mx);
        CHECK_EQ(static_cast<int>(d->y), row.my);
      }
    }
  }
}

// ===========================================================================
// 4.3 Drag out of the pace rect (> 40 px) -> no tap
// ===========================================================================
void test43() {
  {  // drag down from the hero into the speed row (60 px)
    Rig r;
    for (int k = 0; k < 10; ++k) r.feed(1 * SEC + k * P, true, PACE_X, PACE_Y + 6 * k);
    r.gap(1 * SEC + 10 * P, 2 * SEC);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    CHECK_EQ(r.count(GestureType::TouchDown), 1);
  }
  {  // UP point outside the pace rect but drift < 40: no tap (UP must be inside)
    Rig r;
    for (int k = 0; k < 3; ++k) r.feed(1 * SEC + k * P, true, PACE_X, 110);
    r.feed(1 * SEC + 3 * P, true, PACE_X, 125);
    r.gap(1 * SEC + 4 * P, 2 * SEC);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
  }
  {  // a drag that leaves and comes back: max drift counts, not the end point
    Rig r;
    for (int k = 0; k < 3; ++k) r.feed(1 * SEC + k * P, true, PACE_X, PACE_Y);
    r.feed(1 * SEC + 3 * P, true, PACE_X + 45, PACE_Y);
    r.feed(1 * SEC + 4 * P, true, PACE_X, PACE_Y);
    r.gap(1 * SEC + 5 * P, 2 * SEC);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
  }
}

// ===========================================================================
// 4.4 Hold 1.99 s / 2.00 s -> Cancel / Complete (exact boundary)
// ===========================================================================
void test44() {
  for (const int64_t dur : {1990 * MS, 2000 * MS}) {
    Rig r;
    const int64_t t0 = 5 * SEC;
    r.press(t0, dur, STOP_X, STOP_Y, 10 * MS);
    CHECK_EQ(r.count(GestureType::HoldStart), 1);
    const GestureEvent* hs = r.first(GestureType::HoldStart);
    if (hs != nullptr) CHECK_EQ(hs->t, t0);  // holdBegan(t_down): the first DOWN sample
    if (dur < 2 * SEC) {
      CHECK_EQ(r.count(GestureType::HoldComplete), 0);
      CHECK_EQ(r.count(GestureType::HoldCancel), 1);
      const GestureEvent* c = r.first(GestureType::HoldCancel);
      // UP 120 ms after the last valid sample (the bridged window never completes a hold)
      if (c != nullptr) CHECK_EQ(c->t, t0 + dur + 120 * MS);
    } else {
      CHECK_EQ(r.count(GestureType::HoldComplete), 1);
      CHECK_EQ(r.count(GestureType::HoldCancel), 0);
      const GestureEvent* c = r.first(GestureType::HoldComplete);
      if (c != nullptr) CHECK_EQ(c->t, t0 + 2 * SEC);
    }
  }
  {  // the 3 DOWN samples count for drift: a brush landing at 1250 px/s (100 -> 150 px) that
     // stops at its median point is cancelled at DOWN, although every later sample is still
    Rig r;
    const int64_t t0 = 5 * SEC;
    r.feed(t0, true, 100, STOP_Y);
    r.feed(t0 + P, true, 125, STOP_Y);
    r.feed(t0 + 2 * P, true, 150, STOP_Y);
    for (int k = 3; k <= 110; ++k) r.feed(t0 + k * P, true, 125, STOP_Y);
    CHECK_EQ(r.count(GestureType::HoldStart), 1);
    CHECK_EQ(r.count(GestureType::HoldCancel), 1);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
  }
  {  // HoldProgress permille (renderer query)
    Rig r;
    const int64_t t0 = 5 * SEC;
    CHECK_EQ(r.g.holdPermille(t0), -1);
    for (int k = 0; k < 3; ++k) r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    CHECK(r.g.holding());
    CHECK_EQ(r.g.holdDownUs(), t0);
    CHECK_EQ(r.g.holdPermille(t0), 0);
    CHECK_EQ(r.g.holdPermille(t0 + 500 * MS), 250);
    CHECK_EQ(r.g.holdPermille(t0 + 1999 * MS), 999);
    CHECK_EQ(r.g.holdPermille(t0 + 2500 * MS), 1000);
  }
  {  // Strip only: a still 3 s press in a cell, on the speed row or the hero never starts a hold
    for (const int which : {0, 1, 2}) {
      Rig r;
      const int x = which == 0 ? CELL_X : (which == 1 ? SPEED_X : PACE_X);
      const int y = which == 0 ? CELL_Y : (which == 1 ? SPEED_Y : PACE_Y);
      r.press(5 * SEC, 3 * SEC, x, y);
      CHECK_EQ(r.count(GestureType::HoldStart), 0);
      CHECK_EQ(r.count(GestureType::HoldComplete), 0);
    }
  }
  {  // ... but a hold started on the strip may wobble up into the cells by <= 20 px (DOWN at the strip's
     // top edge y 202, then 20 px up to y 182, inside the cell row) and still completes
    Rig r;
    const int64_t t0 = 5 * SEC;
    for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) r.feed(k, true, STOP_X, k >= t0 + 1 * SEC ? 182 : 202);
    CHECK_EQ(r.count(GestureType::HoldStart), 1);
    CHECK_EQ(r.count(GestureType::HoldComplete), 1);
  }
  {  // Drift boundary: exactly 20 px keeps the hold, 21 px cancels (HOLD_SLOP_PX = 20)
    for (const int d : {20, 21}) {
      Rig r;
      const int64_t t0 = 5 * SEC;
      for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) r.feed(k, true, STOP_X + (k >= t0 + 1 * SEC ? d : 0), STOP_Y);
      r.gap(t0 + 2 * SEC + P, t0 + 2400 * MS);
      CHECK_EQ(r.count(GestureType::HoldComplete), d <= 20 ? 1 : 0);
      CHECK_EQ(r.count(GestureType::HoldCancel), d <= 20 ? 0 : 1);
      // the rest of the contact is ignored after a completed or cancelled hold
      const GestureEvent* up = r.first(GestureType::TouchUp);
      if (CHECK(up != nullptr)) CHECK(up->reason == IgnoreReason::HoldEnded);
    }
  }
}

// ===========================================================================
// 4.5 / 4.6 Dropouts: 100 ms bridged (completes at 2.0 s from DOWN); 150 ms cancels
// ===========================================================================
void dropoutHold(int64_t lastValidBefore, int64_t nextValid, int expectComplete) {
  Rig r;
  const int64_t t0 = 5 * SEC;
  for (int64_t k = t0; k <= t0 + 2400 * MS; k += 10 * MS) {
    const bool inDrop = k > t0 + lastValidBefore && k < t0 + nextValid;
    r.feed(k, !inDrop, STOP_X, STOP_Y);
  }
  r.gap(t0 + 2420 * MS, t0 + 2800 * MS);
  if (!CHECK_EQ(r.count(GestureType::HoldComplete), expectComplete))
    std::fprintf(stderr, "  dropout %lld..%lld\n", static_cast<long long>(lastValidBefore), static_cast<long long>(nextValid));
  if (expectComplete) {
    const GestureEvent* c = r.first(GestureType::HoldComplete);
    if (c != nullptr) CHECK_EQ(c->t, t0 + 2 * SEC);
    CHECK_EQ(r.count(GestureType::HoldCancel), 0);
  } else {
    // (the contact that resumes after the UP is a NEW presence: it may start a
    // new hold, which is too short to complete and cancels at its own UP)
    CHECK(r.count(GestureType::HoldCancel) >= 1);
    const GestureEvent* c = r.first(GestureType::HoldCancel);
    if (c != nullptr) CHECK_EQ(c->t, t0 + lastValidBefore + 120 * MS);  // UP 120 ms after the last valid
  }
}
void test45_46() {
  dropoutHold(1000 * MS, 1100 * MS, 1);  // 4.5: 100 ms dropout at 1.0 s (invalid 1.01..1.09)
  dropoutHold(1000 * MS, 1120 * MS, 1);  // invalid up to +110 ms: still bridged
  dropoutHold(1000 * MS, 1150 * MS, 0);  // 4.6: 150 ms dropout -> UP at +120 -> cancel
  dropoutHold(1000 * MS, 1130 * MS, 0);  // boundary: an invalid sample at exactly +120 ms -> UP
  {  // 4.5 with 20 ms polling: valid at 1.00, invalid 1.02..1.08, valid from 1.10
    Rig r;
    const int64_t t0 = 5 * SEC;
    for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) r.feed(k, !(k > t0 + 1 * SEC && k < t0 + 1100 * MS), STOP_X, STOP_Y);
    CHECK_EQ(r.count(GestureType::HoldComplete), 1);
  }
  {  // a dropout spanning the 2.0 s mark: completion waits for the next valid sample
    Rig r;
    const int64_t t0 = 5 * SEC;
    for (int64_t k = t0; k <= t0 + 2200 * MS; k += P) r.feed(k, !(k > t0 + 1950 * MS && k < t0 + 2040 * MS), STOP_X, STOP_Y);
    const GestureEvent* c = r.first(GestureType::HoldComplete);
    if (CHECK(c != nullptr)) CHECK_EQ(c->t, t0 + 2040 * MS);
  }
}

// ===========================================================================
// 4.7 Hold that starts outside and slides into stopRect -> no hold
// ===========================================================================
void test47() {
  const ot::TouchRect& R = ot::STOP_RECT;
  struct Start { int x, y; };
  // above the strip (a cell), the 12 px bezel margins left/right, the bottom margin
  for (const Start s : {Start{160, 190}, Start{4, STOP_Y}, Start{315, STOP_Y}, Start{160, 234}}) {
    Rig r;
    const int64_t t0 = 5 * SEC;
    int x = s.x, y = s.y;
    for (int k = 0; k < 3; ++k) r.feed(t0 + k * P, true, x, y);  // DOWN outside
    for (int k = 3; k < 160; ++k) {                                // slide 15 px in, then hold still 3 s
      if (k < 18) {
        if (s.y < R.y) ++y;
        else if (s.y >= R.y + R.h) --y;
        else if (s.x < R.x) ++x;
        else --x;
      }
      r.feed(t0 + k * P, true, x, y);
    }
    CHECK(R.contains(x, y));  // it did end inside
    r.gap(t0 + 160 * P, t0 + 160 * P + 300 * MS);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
  }
  // ALL 3 DOWN samples must be inside STOP_RECT. One sample just
  // outside an edge (the 1st, 2nd or 3rd) with the other two (and so the
  // median DOWN point) inside -> no hold, although the finger then stays still
  // inside for 3 s. Control: all 3 inside, at the very edge -> hold completes.
  {
    struct Edge { int out, in1, in2; bool vertical; };  // coordinate values along one axis
    const Edge edges[] = {{R.y - 1, R.y + 2, R.y + 5, true},                    // top (cells side)
                          {R.y + R.h, R.y + R.h - 3, R.y + R.h - 6, true},      // bottom margin
                          {R.x - 1, R.x + 2, R.x + 5, false},                   // left margin
                          {R.x + R.w, R.x + R.w - 3, R.x + R.w - 6, false}};    // right margin
    for (const Edge& e : edges) {
      for (int outAt = -1; outAt < 3; ++outAt) {  // -1 = control (all inside: out replaced by the edge pixel)
        Rig r;
        const int64_t t0 = 5 * SEC;
        const int edgeIn = e.out == R.y - 1 ? R.y : (e.out == R.x - 1 ? R.x : e.out - 1);
        int v[3] = {e.in1, e.in2, e.in1};
        if (outAt >= 0) v[outAt] = e.out;
        else v[0] = edgeIn;
        for (int k = 0; k < 3; ++k) r.feed(t0 + k * P, true, e.vertical ? STOP_X : v[k], e.vertical ? v[k] : STOP_Y);
        for (int k = 3; k <= 150; ++k) r.feed(t0 + k * P, true, e.vertical ? STOP_X : e.in1, e.vertical ? e.in1 : STOP_Y);
        r.gap(t0 + 151 * P, t0 + 151 * P + 300 * MS);
        const int want = outAt < 0 ? 1 : 0;
        if (!CHECK_EQ(r.count(GestureType::HoldStart), want) || !CHECK_EQ(r.count(GestureType::HoldComplete), want))
          std::fprintf(stderr, "  DOWN-sample edge out=%d outAt=%d\n", e.out, outAt);
      }
    }
  }
  {  // A finger landing just above the strip and sliding down from its very first sample
     // (y 196, 202, 208: median 202 = the strip's top edge, inside), then still on the strip 3 s -> no hold
    Rig r;
    const int64_t t0 = 5 * SEC;
    const int ys[] = {196, 202, 208, 214};
    for (int k = 0; k < 4; ++k) r.feed(t0 + k * P, true, STOP_X, ys[k]);
    for (int k = 4; k <= 150; ++k) r.feed(t0 + k * P, true, STOP_X, 214);
    r.gap(t0 + 151 * P, t0 + 151 * P + 300 * MS);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
    const GestureEvent* d = r.first(GestureType::TouchDown);
    if (CHECK(d != nullptr)) CHECK(ot::STOP_RECT.contains(d->x, d->y));  // the DOWN point itself is inside
  }
}

// ===========================================================================
// 4.8 Contact present when the button becomes visible -> no hold until UP + a new DOWN
// ===========================================================================
void test48() {
  for (int when = 0; when < 3; ++when) {  // switch after the 1st sample (pre-DOWN), after DOWN, mid-hold length
    Rig r;
    r.ctx = ctxIdle();
    const int64_t t0 = 5 * SEC;
    const int switchAt = when == 0 ? 1 : (when == 1 ? 3 : 50);
    for (int k = 0; k < 175; ++k) {  // 3.5 s still on the button area
      if (k == switchAt) r.ctx = ctxMain();
      r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    }
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
    const GestureEvent* d = r.first(GestureType::TouchDown);
    if (CHECK(d != nullptr) && when == 0) CHECK(d->reason == IgnoreReason::ScreenChange);
    r.gap(t0 + 175 * P, t0 + 175 * P + 200 * MS);  // UP
    r.clear();
    r.press(t0 + 5 * SEC, 2 * SEC, STOP_X, STOP_Y);  // a new DOWN holds normally
    CHECK_EQ(r.count(GestureType::HoldStart), 1);
    CHECK_EQ(r.count(GestureType::HoldComplete), 1);
  }
  {  // The button appearing on the SAME screen id (stopVisible false -> true) counts as an appearance too
    Rig r;
    r.ctx.stopVisible = false;
    const int64_t t0 = 5 * SEC;
    for (int k = 0; k < 150; ++k) {
      if (k == 1) r.ctx.stopVisible = true;
      r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    }
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
  }
  {  // A screen change during a hold cancels it (HoldCancel at the change instant)
    Rig r;
    const int64_t t0 = 5 * SEC;
    for (int k = 0; k < 50; ++k) r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    r.ctx = ctxIdle();
    r.feed(t0 + 50 * P, true, STOP_X, STOP_Y);
    const GestureEvent* c = r.first(GestureType::HoldCancel);
    if (CHECK(c != nullptr)) CHECK_EQ(c->t, t0 + 50 * P);
    for (int k = 51; k < 150; ++k) r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
  }
  {  // The button disappearing on the SAME screen id (stopVisible true -> false) cancels a hold at that instant
    Rig r;
    const int64_t t0 = 5 * SEC;
    for (int k = 0; k < 50; ++k) r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    r.ctx.stopVisible = false;
    r.feed(t0 + 50 * P, true, STOP_X, STOP_Y);
    const GestureEvent* c = r.first(GestureType::HoldCancel);
    if (CHECK(c != nullptr)) {
      CHECK_EQ(c->t, t0 + 50 * P);
      CHECK(c->reason == IgnoreReason::ScreenChange);
    }
    for (int k = 51; k < 150; ++k) r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
  }
  // A tap spanning a screen change (IDLE -> MAIN, MAIN -> IDLE; both screens have the
  // units tap) never toggles: 140 ms on the pace, the screen changes after DOWN. Control: no change -> 1 tap.
  for (int dir = 0; dir < 3; ++dir) {
    Rig r;
    r.ctx = dir == 1 ? ctxMain() : ctxIdle();
    const int64_t t0 = 5 * SEC;
    for (int k = 0; k < 8; ++k) {
      if (k == 5 && dir == 0) r.ctx = ctxMain();
      if (k == 5 && dir == 1) r.ctx = ctxIdle();
      r.feed(t0 + k * P, true, PACE_X, PACE_Y);
    }
    r.gap(t0 + 8 * P, t0 + 8 * P + 200 * MS);
    CHECK_EQ(r.count(GestureType::TouchDown), 1);
    CHECK_EQ(r.count(GestureType::TapPace), dir == 2 ? 1 : 0);
    const GestureEvent* u = r.first(GestureType::TouchUp);
    if (CHECK(u != nullptr)) CHECK(u->reason == (dir == 2 ? IgnoreReason::None : IgnoreReason::ScreenChange));
  }
}

// ===========================================================================
// 4.9 Touch while OFF / waking (screenAwake = false injected): Wake only. A DIM
// panel is awake (ot::touchAwake, test_ui_policy).
// ===========================================================================
void test49() {
  {
    Rig r;
    r.ctx.screenAwake = false;
    const int64_t t0 = 5 * SEC;
    for (int k = 0; k < 3; ++k) r.feed(t0 + k * P, true, 160, 140);  // speed row
    r.ctx.screenAwake = true;                                         // the wake turns the screen on
    int y = 140;
    for (int k = 3; k < 200; ++k) {                                   // ... and the finger slides onto the button
      if (y < STOP_Y) y += 4;
      r.feed(t0 + k * P, true, 160, y);
    }
    r.gap(t0 + 200 * P, t0 + 200 * P + 200 * MS);
    CHECK_EQ(r.count(GestureType::Wake), 1);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
  }
  {  // a wake touch ON the button never holds, even if the screen wakes during the contact
    Rig r;
    r.ctx.screenAwake = false;
    const int64_t t0 = 5 * SEC;
    for (int k = 0; k < 150; ++k) {
      if (k == 3) r.ctx.screenAwake = true;
      r.feed(t0 + k * P, true, STOP_X, STOP_Y);
    }
    CHECK_EQ(r.count(GestureType::Wake), 1);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
  }
  {  // a wake tap on the pace never toggles (does not leak into a tap)
    Rig r;
    r.ctx.screenAwake = false;
    r.press(5 * SEC, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::Wake), 1);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    const GestureEvent* d = r.first(GestureType::TouchDown);
    if (CHECK(d != nullptr)) CHECK(d->reason == IgnoreReason::Wake);
    r.ctx.screenAwake = true;  // the next tap works
    r.press(6 * SEC, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
  }
}

// ===========================================================================
// 4.10 Tap then hold within 300 ms -> toggle once, hold completes normally
// ===========================================================================
void test410() {
  for (const int64_t gapUs : {140 * MS, 300 * MS}) {
    Rig r;
    const int64_t t0 = 5 * SEC;
    for (int64_t k = t0; k <= t0 + 150 * MS; k += 10 * MS) r.feed(k, true, PACE_X, PACE_Y);
    const int64_t tHold = t0 + 150 * MS + gapUs;  // next valid sample = the hold's first sample
    r.gap(t0 + 160 * MS, tHold);
    for (int64_t k = tHold; k <= tHold + 2 * SEC; k += P) r.feed(k, true, STOP_X, STOP_Y);
    r.gap(tHold + 2 * SEC + P, tHold + 2400 * MS);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
    CHECK_EQ(r.count(GestureType::HoldStart), 1);
    CHECK_EQ(r.count(GestureType::HoldComplete), 1);
    const GestureEvent* c = r.first(GestureType::HoldComplete);
    if (c != nullptr) CHECK_EQ(c->t, tHold + 2 * SEC);
  }
}

// ===========================================================================
// 4.11 Phantom constant pressure (stuck, from boot) -> never taps, never holds
// ===========================================================================
void test411() {
  for (const int which : {0, 1}) {
    Rig r(false);  // the very first sample is valid: contact at boot
    const int x = which == 0 ? PACE_X : STOP_X, y = which == 0 ? PACE_Y : STOP_Y;
    for (int64_t k = 0; k <= 30 * SEC; k += P) r.feed(k, true, x, y);
    r.gap(30 * SEC + P, 30 * SEC + 400 * MS);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::Stuck), 1);
    const GestureEvent* st = r.first(GestureType::Stuck);
    if (CHECK(st != nullptr)) CHECK_EQ(st->t, 10 * SEC + P);  // "> 10 s": not at exactly 10.000 s
    const GestureEvent* d = r.first(GestureType::TouchDown);
    if (CHECK(d != nullptr)) CHECK(d->reason == IgnoreReason::Boot);
    // released: the next real hold works
    r.clear();
    r.press(32 * SEC, 2 * SEC, STOP_X, STOP_Y);
    CHECK_EQ(r.count(GestureType::HoldComplete), 1);
  }
  {  // a short boot contact (released quickly) is ignored too, then a tap works
    Rig r(false);
    r.press(0, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    r.press(1 * SEC, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(r.count(GestureType::TapPace), 1);
  }
  {  // STUCK_TOUCH_MS counts from the start of the PRESENCE (bridged bouncing included):
     // bounce (no DOWN) 7.9 s then a 2.0 s hold -> completes at 9.88 s (< 10 s);
     // bounce 8.1 s -> the presence passes 10 s before the 2 s mark -> Stuck cancels the hold;
     // bounce 10.1 s -> the DOWN itself is after 10 s -> ignored, no hold at all.
    struct Row { int64_t bounce; int starts, completes, stuck; };
    for (const Row row : {Row{7900 * MS, 1, 1, 0}, Row{8100 * MS, 1, 0, 1}, Row{10100 * MS, 0, 0, 1}}) {
      Rig r;
      const int64_t t0 = 5 * SEC;
      int64_t k = t0;
      for (int i = 0; k < t0 + row.bounce; k += P, ++i) r.feed(k, i % 2 == 0, STOP_X, STOP_Y);
      const int64_t steadyEnd = k + 2000 * MS;
      for (; k < steadyEnd; k += P) r.feed(k, true, STOP_X, STOP_Y);
      r.gap(k, k + 300 * MS);  // the 7.9 s row is released (UP at 10.00 s) before the presence passes 10 s
      if (!CHECK_EQ(r.count(GestureType::HoldStart), row.starts) ||
          !CHECK_EQ(r.count(GestureType::HoldComplete), row.completes) ||
          !CHECK_EQ(r.count(GestureType::Stuck), row.stuck))
        std::fprintf(stderr, "  stuck row bounce=%lld\n", static_cast<long long>(row.bounce));
    }
  }
  {  // The stuck guard re-arms for EVERY presence. A first stuck contact (a phantom in a
     // cell, 12 s), released; then a second one in the same session: bouncy (no DOWN) 10.5 s on the strip,
     // then still 2.5 s -> it is guarded too (Stuck at its own 10 s crossing, no hold).
    Rig r;
    for (int64_t k = 1 * SEC; k < 13 * SEC; k += P) r.feed(k, true, CELL_X, CELL_Y);
    r.gap(13 * SEC, 13 * SEC + 300 * MS);
    CHECK_EQ(r.count(GestureType::Stuck), 1);
    r.clear();
    const int64_t s0 = 14 * SEC;
    int64_t k = s0;
    for (int i = 0; k < s0 + 10500 * MS; k += P, ++i) r.feed(k, i % 2 == 0, STOP_X, STOP_Y);
    for (; k < s0 + 13 * SEC; k += P) r.feed(k, true, STOP_X, STOP_Y);
    r.gap(k, k + 300 * MS);
    CHECK_EQ(r.count(GestureType::Stuck), 1);
    CHECK_EQ(r.count(GestureType::HoldStart), 0);
    CHECK_EQ(r.count(GestureType::HoldComplete), 0);
    const GestureEvent* st = r.first(GestureType::Stuck);
    // the crossing (first sample > 10 s after the presence start) is an INVALID sample of the bounce:
    // the guard is checked on invalid samples of a presence too
    if (CHECK(st != nullptr)) CHECK_EQ(st->t, s0 + 10 * SEC + P);
  }
  {  // A tap can never come from a stuck contact. A bouncy presence on the pace (no DOWN)
     // for 9.86 s, then still: DOWN at 9.88 s (a tap candidate), the presence passes 10 s at 10.02 s
     // (Stuck), released at 10.20 s: a 320 ms, 0 px "tap" -> no toggle.
    Rig r;
    const int64_t t0 = 5 * SEC;
    int i = 0;
    for (; i < 494; ++i) r.feed(t0 + i * P, i % 2 == 0, PACE_X, PACE_Y);
    for (; i <= 510; ++i) r.feed(t0 + i * P, true, PACE_X, PACE_Y);
    r.gap(t0 + 511 * P, t0 + 511 * P + 300 * MS);
    CHECK_EQ(r.count(GestureType::TouchDown), 1);
    CHECK_EQ(r.count(GestureType::Stuck), 1);
    CHECK_EQ(r.count(GestureType::TapPace), 0);
    const GestureEvent* d = r.first(GestureType::TouchDown);
    if (CHECK(d != nullptr)) {
      CHECK_EQ(d->t, t0 + 494 * P);
      CHECK(d->reason == IgnoreReason::None);  // live at DOWN: only the stuck guard stops the tap
    }
    const GestureEvent* u = r.first(GestureType::TouchUp);
    if (CHECK(u != nullptr)) CHECK(u->reason == IgnoreReason::Stuck);
  }
}

// ===========================================================================
// 4.13 Post-stop lockout: touches within 1 s after a stop do nothing
// ===========================================================================
void test413() {
  Rig r;
  const int64_t t0 = 5 * SEC;
  for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) r.feed(k, true, STOP_X, STOP_Y);
  const int64_t tc = t0 + 2 * SEC;
  CHECK_EQ(r.count(GestureType::HoldComplete), 1);
  for (int64_t k = tc + P; k <= tc + 300 * MS; k += P) r.feed(k, true, STOP_X, STOP_Y);  // still pressed: nothing
  r.gap(tc + 320 * MS, tc + 500 * MS);                                                    // UP at +420 ms
  r.press(tc + 500 * MS, 150 * MS, PACE_X, PACE_Y);                                       // tap at +0.5 s
  r.press(tc + 700 * MS, 290 * MS, STOP_X, STOP_Y);  // DOWN at +0.7 s; still held past the lockout end
  CHECK_EQ(r.count(GestureType::TapPace), 0);
  CHECK_EQ(r.count(GestureType::HoldStart), 1);
  {  // a hold whose DOWN is in the lockout and lasts past it stays ignored until UP
    Rig q;
    for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) q.feed(k, true, STOP_X, STOP_Y);
    q.gap(tc + P, tc + 200 * MS);
    for (int64_t k = tc + 200 * MS; k <= tc + 4 * SEC; k += P) q.feed(k, true, STOP_X, STOP_Y);
    CHECK_EQ(q.count(GestureType::HoldStart), 1);
    CHECK_EQ(q.count(GestureType::HoldComplete), 1);
    const GestureEvent* d = nullptr;
    for (const GestureEvent& e : q.ev)
      if (e.type == GestureType::TouchDown) d = &e;
    if (CHECK(d != nullptr)) CHECK(d->reason == IgnoreReason::Lockout);
  }
  // Boundary: DOWN at exactly tc + 1 s is allowed; tc + 1 s - 20 ms is not.
  for (const int64_t at : {1 * SEC - P, 1 * SEC}) {
    Rig q;
    for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) q.feed(k, true, STOP_X, STOP_Y);
    q.gap(tc + P, tc + at);
    q.press(tc + at, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(q.count(GestureType::TapPace), at >= 1 * SEC ? 1 : 0);
  }
  {  // the app-imposed lockout (ctx.lockoutUntil) works the same way
    Rig q;
    q.ctx.lockoutUntil = 3 * SEC;
    q.press(2900 * MS, 150 * MS, PACE_X, PACE_Y);
    q.press(3 * SEC + 500 * MS, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(q.count(GestureType::TapPace), 1);
  }
  // The lockout keys on the PRESENCE start. A bouncy finger whose presence begins inside
  // the window (valid/invalid alternating from tc + 0.94 s) but whose DOWN (3 consecutive valid samples)
  // only comes after it (tDown = tc + 1.02 s) stays ignored until UP: no tap (idle screen, pace) and no
  // hold (main screen, strip). Control: the same bounce starting at exactly tc + 1 s works.
  for (const int64_t start : {940 * MS, 1000 * MS}) {
    for (const int kind : {0, 1}) {  // 0: a tap on the units label (idle); 1: a hold on the strip (main)
      Rig q;
      for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) q.feed(k, true, STOP_X, STOP_Y);
      q.gap(tc + P, tc + start);
      if (kind == 0) q.ctx = ctxIdle();
      const int x = kind == 0 ? PACE_X : STOP_X, y = kind == 0 ? PACE_Y : STOP_Y;
      int64_t k = tc + start;
      for (int i = 0; i < 4; ++i, k += P) q.feed(k, i % 2 == 0, x, y);  // valid, invalid, valid, invalid
      const int64_t tDown = k;
      const int64_t steady = kind == 0 ? 100 * MS : 2500 * MS;
      for (; k <= tDown + steady; k += P) q.feed(k, true, x, y);
      q.gap(k, k + 300 * MS);
      const bool locked = start < 1 * SEC;
      const GestureEvent* d = nullptr;
      for (const GestureEvent& e : q.ev)
        if (e.type == GestureType::TouchDown) d = &e;
      if (CHECK(d != nullptr)) {
        CHECK_EQ(d->t, tDown);
        CHECK(tDown >= tc + 1 * SEC);  // the DOWN itself is after the window
        CHECK(d->reason == (locked ? IgnoreReason::Lockout : IgnoreReason::None));
      }
      if (kind == 0) {
        CHECK_EQ(q.count(GestureType::TapPace), locked ? 0 : 1);
      } else {
        CHECK_EQ(q.count(GestureType::HoldStart), locked ? 1 : 2);     // (1 = the first, completed hold)
        CHECK_EQ(q.count(GestureType::HoldComplete), locked ? 1 : 2);
      }
    }
  }
  {  // Wake is checked before the lockout: a touch while the screen is asleep inside the lockout still
     // reports Wake (backlight activity), and is swallowed
    Rig q;
    for (int64_t k = t0; k <= t0 + 2 * SEC; k += P) q.feed(k, true, STOP_X, STOP_Y);
    q.gap(tc + P, tc + 300 * MS);
    q.ctx.screenAwake = false;
    q.press(tc + 300 * MS, 150 * MS, PACE_X, PACE_Y);
    CHECK_EQ(q.count(GestureType::Wake), 1);
    CHECK_EQ(q.count(GestureType::TapPace), 0);
  }
  // After the lockout: a tap works
  r.clear();
  r.press(tc + 1500 * MS, 150 * MS, PACE_X, PACE_Y);
  CHECK_EQ(r.count(GestureType::TapPace), 1);
}

// ===========================================================================
// 4.12 Accidental-stop torture + control set
// ===========================================================================
struct TortureStats {
  int contacts = 0, longContacts = 0, downs = 0, holdStarts = 0, holdCancels = 0, holdCompletes = 0, taps = 0;
};

// Poll times with jitter: k-th sample at base + k*P + j, j in [0, 3 ms] (the
// device's poll lateness), strictly increasing.
struct Clock {
  int64_t t = 1 * SEC;
  Rng* rng;
  int64_t step() {
    t += P + rng->range(-3, 3) * MS / 3;
    return t;
  }
};

// Position of a contact at time `dt` after its first sample (x1000 fixed point).
struct Motion {
  int64_t x0 = 0, y0 = 0;        // start (px)
  int64_t ux = 0, uy = 0;        // direction x1000
  int64_t startUs = 0;           // motion starts after this
  int64_t pxPerS = 0;            // speed
  int64_t maxPx = INT64_MAX;     // displacement cap (px)
  void at(int64_t dt, int64_t& x, int64_t& y) const {
    int64_t d1000 = dt > startUs ? (dt - startUs) * pxPerS / 1000 : 0;  // px x1000
    if (maxPx != INT64_MAX && d1000 > maxPx * 1000) d1000 = maxPx * 1000;
    x = x0 + rdiv(ux * d1000, 1000000);
    y = y0 + rdiv(uy * d1000, 1000000);
  }
};

// Runs one contact of duration `dur` through r (samples from c), with
// optional dropouts after `dropFrom` (each <= dropMaxUs), and the context
// switch callback. Then a gap of `gapUs` invalid samples.
template <class F>
void runContact(Rig& r, Clock& c, Rng& rng, const Motion& m, int64_t dur, int64_t dropFrom, int64_t dropMaxUs,
                int dropPct10, int64_t gapUs, F&& onSample) {
  const int64_t tStart = c.step();
  int64_t dropUntil = -1;
  for (int64_t t = tStart; t - tStart <= dur; t = c.step()) {
    const int64_t dt = t - tStart;
    onSample(dt);
    int64_t x = 0, y = 0;
    m.at(dt, x, y);
    bool valid = onPanel(x, y);
    // (at least one valid sample between two dropouts: a chain longer than
    // 120 ms would be a real release, splitting the contact in two)
    if (dt >= dropFrom && t >= dropUntil + P && dropMaxUs > 0 && rng.chance(dropPct10))
      dropUntil = t + rng.range(P, dropMaxUs);
    if (t < dropUntil && dt + P <= dur) valid = false;  // never drop the final sample
    r.feed(t, valid, valid ? static_cast<int>(x) : 0, valid ? static_cast<int>(y) : 0);
  }
  const int64_t tEnd = c.t;
  while (c.t - tEnd < gapUs) r.none(c.step());
}

void tally(const Rig& r, TortureStats& s) {
  s.downs += r.count(GestureType::TouchDown);
  s.holdStarts += r.count(GestureType::HoldStart);
  s.holdCancels += r.count(GestureType::HoldCancel);
  s.holdCompletes += r.count(GestureType::HoldComplete);
  s.taps += r.count(GestureType::TapPace);
}

int64_t contactDuration(Rng& rng) {  // 0.02..3.0 s, >= 30% above 2 s (40% drawn from (2, 3] s)
  return rng.chance(400) ? rng.range(2 * SEC + 1, 3 * SEC) : rng.range(20 * MS, 2 * SEC);
}

void insideStop(Rng& rng, int64_t& x, int64_t& y, int64_t inset) {
  const ot::TouchRect& s = ot::STOP_RECT;
  x = rng.range(s.x + inset, s.x + s.w - 1 - inset);
  y = rng.range(s.y + inset, s.y + s.h - 1 - inset);
}

// A point off the strip for family (e): the hero, the speed row, the cells
// (incl. the 2 px gap above the strip), or the 12 px bezel margins. Absolute
// layout coordinates (NOT derived from STOP_RECT), so a widened STOP_RECT
// (e.g. a larger band or a 6 px margin) makes (e) fail.
void offStrip(Rng& rng, int64_t& x, int64_t& y) {
  switch (rng.range(0, 4)) {
    case 0: x = rng.range(0, 319); y = rng.range(24, 119); break;    // hero (pace)
    case 1: x = rng.range(0, 319); y = rng.range(122, 151); break;   // speed row
    case 2: x = rng.range(0, 319); y = rng.range(154, 201); break;   // cells + gap
    case 3:                                                          // left / right margin
      x = rng.chance(500) ? rng.range(0, 11) : rng.range(308, 319);
      y = rng.range(154, 239);
      break;
    default: x = rng.range(0, 319); y = rng.range(228, 239); break;  // bottom margin
  }
}

void test412() {
  constexpr int PER_FAMILY = 500;
  constexpr int FAMILIES = 5;
  TortureStats fam[FAMILIES];
  const char* const names[FAMILIES] = {"a slide-in", "b drift>20px", "c sweep", "d present-at-appear",
                                       "e stationary off-strip"};
  int slideFromFirst = 0, slideFromFirstMedianIn = 0;
  const ot::TouchRect& R = ot::STOP_RECT;
  for (int f = 0; f < FAMILIES; ++f) {
    Rng rng(0x4120000ull + static_cast<uint64_t>(f));
    Clock c{1 * SEC, &rng};
    Rig r;
    TortureStats& st = fam[f];
    for (int i = 0; i < PER_FAMILY; ++i) {
      const int64_t dur = f == 4 ? rng.range(2 * SEC, 5 * SEC) : contactDuration(rng);
      ++st.contacts;
      if (dur > 2 * SEC) ++st.longContacts;
      Motion m;
      const int64_t gapUs = rng.range(200 * MS, 600 * MS);
      if (f == 0) {
        // (a) DOWN outside STOP_RECT (above the strip in the cells, or in a
        // 12 px bezel margin), then a slide INTO the rect, then still. Half:
        // still for the 3 DOWN samples, then a slide of 1..60 px (a third of
        // them < 20 px, which a hold would tolerate). Half:
        // sliding from the very first sample, which lands 1..s px outside
        // (s = px per sample, 2..6), so the 2nd/3rd samples and the median
        // DOWN point are inside; the slide is 4..20 px (within the hold slop).
        const int64_t where = rng.range(0, 3);
        const bool fromFirst = rng.chance(500);
        const int64_t speed = fromFirst ? rng.range(100, 300) : rng.range(20, 300);
        const int64_t out = fromFirst ? rng.range(1, speed / 50) : 0;  // px outside at the first sample
        if (where == 0) { m.x0 = rng.range(R.x + 4, R.x + R.w - 5); m.y0 = fromFirst ? R.y - out : rng.range(172, R.y - 1); m.ux = 0; m.uy = 1000; }
        else if (where == 1) { m.x0 = fromFirst ? R.x - out : rng.range(0, R.x - 1); m.y0 = rng.range(R.y + 2, R.y + R.h - 3); m.ux = 1000; m.uy = 0; }
        else if (where == 2) { m.x0 = fromFirst ? R.x + R.w - 1 + out : rng.range(R.x + R.w, 319); m.y0 = rng.range(R.y + 2, R.y + R.h - 3); m.ux = -1000; m.uy = 0; }
        else { m.x0 = rng.range(R.x + 4, R.x + R.w - 5); m.y0 = fromFirst ? R.y + R.h - 1 + out : rng.range(R.y + R.h, 239); m.ux = 0; m.uy = -1000; }
        m.startUs = fromFirst ? 0 : 3 * P + 5 * MS;
        m.pxPerS = speed;
        m.maxPx = fromFirst ? rng.range(4, 20) : rng.range(1, 60);
        if (fromFirst) {
          ++slideFromFirst;
          // (the median of the first 3 samples, at the nominal 20 ms grid, is the 2nd: inside?)
          int64_t x1 = 0, y1 = 0;
          m.at(P, x1, y1);
          if (R.contains(static_cast<int32_t>(x1), static_cast<int32_t>(y1))) ++slideFromFirstMedianIn;
        }
        runContact(r, c, rng, m, dur, fromFirst ? 3 * P + 10 * MS : m.startUs + 2 * P, 60 * MS, 30, gapUs,
                   [](int64_t) {});
      } else if (f == 1) {
        // (b) DOWN inside, then a drift of 21..80 px that passes 20 px before
        // 2 s (motion starts 60 ms..1.3 s, 40..400 px/s), then still.
        insideStop(rng, m.x0, m.y0, 0);
        randomDir(rng, m.ux, m.uy);
        m.startUs = rng.range(3 * P, 1300 * MS);
        m.pxPerS = rng.range(40, 400);
        m.maxPx = rng.range(21, 80);
        runContact(r, c, rng, m, dur, 3 * P + 10 * MS, 80 * MS, 20, gapUs, [](int64_t) {});
      } else if (f == 2) {
        // (c) a sweep across the screen at 50..500 px/s from anywhere
        // (off-panel samples are invalid: the finger left the glass).
        m.x0 = rng.range(0, 319);
        m.y0 = rng.range(100, 239);
        randomDir(rng, m.ux, m.uy);
        m.startUs = 0;
        m.pxPerS = rng.range(50, 500);
        runContact(r, c, rng, m, dur, 0, 0, 0, gapUs, [](int64_t) {});
      } else if (f == 4) {
        // (e) a contact that appears mid-workout (button
        // visible) OFF the strip - a resting palm, forearm, phone or sweat
        // drop on the hero, the speed row, the cells or the 12 px bezel
        // margins - and stays still (half) or creeps at 1..8 px/s in any
        // direction, for 2..5 s, with occasional <= 100 ms dropouts.
        offStrip(rng, m.x0, m.y0);
        randomDir(rng, m.ux, m.uy);
        m.startUs = 0;
        m.pxPerS = rng.chance(500) ? 0 : rng.range(1, 8);
        runContact(r, c, rng, m, dur, 3 * P + 10 * MS, 100 * MS, 20, gapUs, [](int64_t) {});
      } else {
        // (d) the contact is already present when the button appears:
        // idle screen (no button), a still contact inside STOP_RECT, then the
        // main screen appears 1) right after the first sample (before DOWN),
        // 2) during a bouncy pre-DOWN phase, or 3) after DOWN; up to 3 s.
        insideStop(rng, m.x0, m.y0, 3);
        m.ux = 0;
        m.uy = 0;
        const int64_t kind = rng.range(0, 3);  // 0,1: before DOWN; 2: bouncy start; 3: after DOWN
        const int64_t tSwitch = kind <= 1 ? rng.range(1, 2 * P - 1)
                                : kind == 2 ? rng.range(2 * P, 400 * MS)
                                            : rng.range(3 * P, 1500 * MS);
        const int64_t bounceUntil = kind == 2 ? tSwitch + rng.range(0, 200 * MS) : 0;
        r.ctx = ctxIdle();
        // Bouncy start: drop every other sample until bounceUntil (pre-DOWN presence).
        const int64_t tStart = c.step();
        bool switched = false;
        int64_t idx = 0;
        for (int64_t t = tStart; t - tStart <= dur; t = c.step(), ++idx) {
          const int64_t dt = t - tStart;
          if (!switched && dt >= tSwitch) {
            r.ctx = ctxMain();
            switched = true;
          }
          const bool valid = dt >= bounceUntil || idx % 2 == 0;
          const int jx = static_cast<int>(rng.range(-2, 2)), jy = static_cast<int>(rng.range(-2, 2));
          r.feed(t, valid, static_cast<int>(m.x0) + jx, static_cast<int>(m.y0) + jy);
        }
        const int64_t tEnd = c.t;
        while (c.t - tEnd < gapUs) {
          if (c.t - tEnd > 150 * MS) r.ctx = ctxIdle();  // after UP: back to idle for the next contact
          r.none(c.step());
        }
      }
    }
    tally(r, st);
    CHECK(!r.g.eventOverflow());
    CHECK_EQ(st.holdCompletes, 0);
    if (gVerbose || st.holdCompletes != 0)
      std::printf("  4.12 (%s): contacts=%d >2s=%d downs=%d hold_start=%d hold_cancel=%d hold_complete=%d taps=%d\n",
                  names[f], st.contacts, st.longContacts, st.downs, st.holdStarts, st.holdCancels, st.holdCompletes,
                  st.taps);
  }
  int total = 0, longs = 0, completes = 0;
  for (const TortureStats& s : fam) {
    total += s.contacts;
    longs += s.longContacts;
    completes += s.holdCompletes;
  }
  CHECK_EQ(total, PER_FAMILY * FAMILIES);
  CHECK(longs * 10 >= total * 3);  // >= 30% lasting > 2 s
  CHECK_EQ(completes, 0);
  // Families (b) and (c) must actually start holds (otherwise they test nothing).
  CHECK(fam[1].holdStarts >= PER_FAMILY * 9 / 10);
  CHECK(fam[2].holdStarts > 25);
  CHECK_EQ(fam[0].holdStarts, 0);
  CHECK_EQ(fam[3].holdStarts, 0);
  CHECK_EQ(fam[4].holdStarts, 0);
  CHECK_EQ(fam[4].taps, 0);
  CHECK(fam[4].longContacts >= PER_FAMILY - 5);  // (e) contacts last 2..5 s
  CHECK(fam[4].downs >= PER_FAMILY * 9 / 10);    // ... and are real debounced contacts
  // (a)'s slide-from-first-sample half must contain contacts whose median DOWN point is inside (else it tests nothing).
  CHECK(slideFromFirst >= PER_FAMILY / 3);
  CHECK(slideFromFirstMedianIn * 2 >= slideFromFirst);
  std::printf("  4.12 adversarial: %d contacts (%d = %.1f%% > 2 s): hold_complete=%d [a %d/%d/%d b %d/%d/%d c %d/%d/%d"
              " d %d/%d/%d e %d/%d/%d (start/cancel/complete)]; (a) slide-from-1st-sample %d (median inside %d);"
              " (e) downs %d taps %d\n",
              total, longs, 100.0 * longs / total, completes, fam[0].holdStarts, fam[0].holdCancels,
              fam[0].holdCompletes, fam[1].holdStarts, fam[1].holdCancels, fam[1].holdCompletes, fam[2].holdStarts,
              fam[2].holdCancels, fam[2].holdCompletes, fam[3].holdStarts, fam[3].holdCancels, fam[3].holdCompletes,
              fam[4].holdStarts, fam[4].holdCancels, fam[4].holdCompletes, slideFromFirst, slideFromFirstMedianIn,
              fam[4].downs, fam[4].taps);

  // ---- control sets: legitimate presses must ALL complete ----------------
  // Start inside STOP_RECT = the strip (incl. its extreme corners), still for
  // the 3 DOWN samples, then wander within a disc of radius R around the DOWN
  // point, held 2.0..3.0 s (exact 20 ms grid), with 0..3 dropouts of
  // 20..100 ms after DOWN (never covering the last sample).
  struct ControlSet { const char* name; int n; int64_t radius; uint64_t seed; };
  for (const ControlSet cs : {ControlSet{"drift <= 8 px", 200, 8, 0x4120C01ull},
                              ControlSet{"drift 9..19 px", 100, 19, 0x4120C02ull}}) {
    Rng rng(cs.seed);
    Rig r;
    int64_t t = 2 * SEC;
    int completes2 = 0, starts = 0, cancels = 0, edgeCases = 0;
    for (int i = 0; i < cs.n; ++i) {
      r.clear();
      int64_t x0 = 0, y0 = 0;
      if (i % 10 == 0) {  // the rect's extreme corners
        const ot::TouchRect& s = ot::STOP_RECT;
        x0 = (i / 10) % 2 == 0 ? s.x : s.x + s.w - 1;
        y0 = (i / 20) % 2 == 0 ? s.y : s.y + s.h - 1;
        ++edgeCases;
      } else {
        insideStop(rng, x0, y0, 0);
      }
      const int64_t samples = i % 7 == 0 ? 100 : rng.range(100, 150);  // 2.00..3.00 s (some exactly 2.00)
      const int64_t t0 = t;
      int64_t dropLeft = rng.range(0, 3);
      int64_t dropUntil = -1;
      int64_t ox = 0, oy = 0;
      for (int64_t k = 0; k <= samples; ++k) {
        const int64_t tk = t0 + k * P;
        bool valid = true;
        if (k >= 3) {
          // wander: a bounded random walk; stay within the radius (integer disc)
          const int64_t nx = ox + rng.range(-1, 1), ny = oy + rng.range(-1, 1);
          if (nx * nx + ny * ny <= cs.radius * cs.radius) {
            ox = nx;
            oy = ny;
          }
          if (cs.radius > 8 && k == 20) {  // the 9..19 px set: jump out to 9..19 px once
            const int64_t d = rng.range(9, cs.radius);
            ox = d;
            oy = 0;
          }
          // dropouts end before the 2.0 s sample (k = 100), so every press completes at exactly 2.000 s
          if (dropLeft > 0 && tk >= dropUntil + P && k + 6 < 100 && rng.chance(30)) {
            dropUntil = tk + rng.range(1, 5) * P;  // the next valid sample is 20..100 ms after this one's predecessor
            --dropLeft;
          }
          if (tk < dropUntil) valid = false;
        }
        r.feed(tk, valid, static_cast<int>(x0 + ox), static_cast<int>(y0 + oy));
      }
      r.gap(t0 + (samples + 1) * P, t0 + (samples + 1) * P + 200 * MS);
      starts += r.count(GestureType::HoldStart);
      cancels += r.count(GestureType::HoldCancel);
      const int done = r.count(GestureType::HoldComplete);
      completes2 += done;
      if (done != 1) std::fprintf(stderr, "  control %s #%d at (%lld,%lld) did not complete\n", cs.name, i,
                                  static_cast<long long>(x0), static_cast<long long>(y0));
      else {
        const GestureEvent* e = r.first(GestureType::HoldComplete);
        if (e != nullptr) CHECK_EQ(e->t, t0 + 2 * SEC);  // completes exactly 2.000 s after DOWN
      }
      t = t0 + (samples + 1) * P + 1500 * MS;  // the next press after the post-stop lockout
    }
    CHECK_EQ(completes2, cs.n);
    CHECK_EQ(starts, cs.n);
    CHECK_EQ(cancels, 0);
    std::printf("  4.12 control (%s): %d presses (%d at the rect corners) -> %d complete, %d cancel\n", cs.name, cs.n,
                edgeCases, completes2, cancels);
  }
}

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i)
    if (std::strcmp(argv[i], "--verbose") == 0) gVerbose = true;
  // Layout sanity: the production targets (the strip x 12-307,
  // y 202-227) and the test points.
  CHECK(ot::PACE_RECT.contains(PACE_X, PACE_Y));
  CHECK(ot::STOP_RECT.contains(STOP_X, STOP_Y));
  CHECK(!ot::STOP_RECT.contains(CELL_X, CELL_Y));
  CHECK(!ot::STOP_RECT.contains(SPEED_X, SPEED_Y));
  CHECK(!ot::PACE_RECT.contains(SPEED_X, SPEED_Y));
  CHECK(ot::STOP_RECT.contains(12, 202) && ot::STOP_RECT.contains(307, 227));
  CHECK(!ot::STOP_RECT.contains(11, 214) && !ot::STOP_RECT.contains(308, 214) && !ot::STOP_RECT.contains(160, 228) &&
        !ot::STOP_RECT.contains(160, 201));
  {  // every panel pixel: inside STOP_RECT exactly on the strip (nothing above it, nothing in the margins)
    int wrong = 0;
    for (int y = 0; y < 240; ++y)
      for (int x = 0; x < 320; ++x)
        if (ot::STOP_RECT.contains(x, y) != (y >= 202 && y <= 227 && x >= 12 && x <= 307)) ++wrong;
    CHECK_EQ(wrong, 0);
  }
  test41();
  test42();
  test43();
  test44();
  test45_46();
  test47();
  test48();
  test49();
  test410();
  test411();
  test412();
  test413();
  return check::summary("test_touch");
}
