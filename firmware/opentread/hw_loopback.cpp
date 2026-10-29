// hw_loopback.cpp - see hw_loopback.h. Compiled to nothing unless OT_LOOPBACK=1.
#include "config_hw.h"
#include "hw_loopback.h"

#if OT_LOOPBACK

#include <driver/rmt_tx.h>
#include <esp_attr.h>

namespace lb {

namespace {

constexpr uint32_t RES_HZ       = 1000000;  // 1 tick = 1 us (APB 80 MHz / 80)
constexpr uint32_t DEPTH        = 4;        // RMT transaction queue depth = max closures in flight
constexpr uint32_t VARIANTS     = 4;        // closure bounce variants (and 2 release variants)
constexpr uint32_t MAX_HALVES   = 48;
constexpr uint32_t MAX_SYMS     = MAX_HALVES / 2;
constexpr uint32_t HALF_MAX_US  = 32767;    // 15-bit duration field
constexpr uint32_t CLOSED_PCT   = 35;       // closed share of the period (incl. the bounce burst)

static_assert(VARIANTS % 2 == 0, "release variants (i % 2) must align with closure variants (i % 4)");

// Closure bounce bursts, as alternating LOW/HIGH durations (us) before the
// final LOW. 2 halves = 3 edges, 4 halves = 5 edges. Cycle 3,5,3,3 -> 3.5.
struct Burst { uint8_t n; uint16_t d[4]; };
constexpr Burst CLOSE_BURST[VARIANTS] = {
  {2, {400, 300, 0, 0}},
  {4, {250, 400, 600, 900}},  // 2.15 ms before the final LOW
  {2, {300, 500, 0, 0}},
  {2, {500, 200, 0, 0}},
};
// Release chatter: HIGH 300 us, LOW 200 us before the final HIGH (3 edges),
// on odd closures; even closures release cleanly (1 edge).
constexpr uint16_t CHATTER_HIGH_US = 300;
constexpr uint16_t CHATTER_LOW_US  = 200;

rmt_channel_handle_t gChan = nullptr;
rmt_encoder_handle_t gEnc  = nullptr;
int32_t  gInitErr  = 0;
bool     gReady    = false;

// Payload: one period per variant. Rebuilt only by start() while idle, then
// constant for the whole train (the copy encoder reads it from the RMT ISR).
rmt_symbol_word_t gSyms[VARIANTS][MAX_SYMS];
uint32_t gNSyms[VARIANTS];

enum : uint8_t { IDLE = 0, RUNNING = 1, STOPPING = 2 };
uint8_t  gState    = IDLE;
uint32_t gRate     = 0;
uint32_t gCount    = 0;
uint32_t gEnq      = 0;
int32_t  gLastTxErr = 0;
uint32_t gPeriodUs = 0;
volatile uint32_t gDone = 0;  // written only by the RMT TX ISR (done callback)

bool IRAM_ATTR onDone(rmt_channel_handle_t /*ch*/, const rmt_tx_done_event_data_t* /*ev*/, void* /*ctx*/) {
  gDone = gDone + 1u;
  return false;  // no task woken
}

// Builds one period as RMT symbols. Returns false if it doesn't fit.
struct Builder {
  uint8_t  lv[MAX_HALVES];
  uint32_t du[MAX_HALVES];
  uint32_t n = 0;
  bool     ok = true;
  void add(uint8_t level, uint32_t us) {
    while (us > 0 && ok) {
      const uint32_t d = us > HALF_MAX_US ? HALF_MAX_US : us;
      if (n == MAX_HALVES) { ok = false; return; }
      lv[n] = level;
      du[n] = d;
      ++n;
      us -= d;
    }
  }
};

bool buildPattern(uint32_t variant, uint32_t periodUs, rmt_symbol_word_t* out, uint32_t& nSyms) {
  const Burst& b = CLOSE_BURST[variant];
  uint32_t burstUs = 0;
  for (uint32_t i = 0; i < b.n; ++i) burstUs += b.d[i];
  const bool chatter = (variant % 2u) == 1u;
  const uint32_t chatterUs = chatter ? (CHATTER_HIGH_US + CHATTER_LOW_US) : 0u;
  const uint32_t closedUs = periodUs * CLOSED_PCT / 100u;
  if (closedUs <= burstUs || periodUs - closedUs <= chatterUs) return false;

  Builder bd;
  for (uint32_t i = 0; i < b.n; ++i) bd.add((i % 2u) == 0u ? 0 : 1, b.d[i]);  // starts LOW: falling edge
  bd.add(0, closedUs - burstUs);                                             // dwell (closed)
  if (chatter) {
    bd.add(1, CHATTER_HIGH_US);
    bd.add(0, CHATTER_LOW_US);
  }
  bd.add(1, periodUs - closedUs - chatterUs);                                // open rest
  if (!bd.ok) return false;
  if (bd.n % 2u == 1u) {  // pad to whole symbols: split the last (long, HIGH) half in two
    if (bd.n == MAX_HALVES || bd.du[bd.n - 1] < 2u) return false;
    const uint32_t d = bd.du[bd.n - 1];
    bd.du[bd.n - 1] = d / 2u;
    bd.lv[bd.n] = bd.lv[bd.n - 1];
    bd.du[bd.n] = d - d / 2u;
    ++bd.n;
  }
  nSyms = bd.n / 2u;
  for (uint32_t s = 0; s < nSyms; ++s) {
    const uint32_t d0 = bd.du[2 * s], d1 = bd.du[2 * s + 1];
    const uint32_t l0 = bd.lv[2 * s], l1 = bd.lv[2 * s + 1];
    out[s].val = (d0 & 0x7fffu) | (l0 << 15) | ((d1 & 0x7fffu) << 16) | (l1 << 31);
  }
  return true;
}

}  // namespace

bool begin() {
  rmt_tx_channel_config_t cc = {};
  cc.gpio_num          = static_cast<gpio_num_t>(hw::LOOPBACK_PIN);
  cc.clk_src           = RMT_CLK_SRC_DEFAULT;
  cc.resolution_hz     = RES_HZ;
  cc.mem_block_symbols = 64;  // one ESP32 channel block; the driver ping-pongs longer payloads
  cc.trans_queue_depth = DEPTH;
  cc.intr_priority     = 0;
  cc.flags.io_od_mode  = 1;   // open-drain: pulls LOW like the reed, GPIO 27's pull-up gives HIGH
  cc.flags.init_level  = 1;   // released (open)
  esp_err_t e = rmt_new_tx_channel(&cc, &gChan);
  if (e == ESP_OK) {
    rmt_copy_encoder_config_t ec = {};
    e = rmt_new_copy_encoder(&ec, &gEnc);
  }
  if (e == ESP_OK) {
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = onDone;
    e = rmt_tx_register_event_callbacks(gChan, &cbs, nullptr);
  }
  if (e == ESP_OK) e = rmt_enable(gChan);
  gInitErr = static_cast<int32_t>(e);
  gReady = (e == ESP_OK);
  return gReady;
}

Start start(uint32_t rate, uint32_t count) {
  if (!gReady) return Start::NotReady;
  if (gState != IDLE) return Start::Busy;
  if (rate < RATE_MIN || rate > RATE_MAX || count < 1u || count > COUNT_MAX) return Start::BadArgs;
  const uint32_t period = RES_HZ / rate;
  for (uint32_t v = 0; v < VARIANTS; ++v)
    if (!buildPattern(v, period, gSyms[v], gNSyms[v])) return Start::BadArgs;
  // Idle: nothing in flight, so the done callback cannot run concurrently.
  gDone      = 0;
  gEnq       = 0;
  gRate      = rate;
  gCount     = count;
  gPeriodUs  = period;
  gLastTxErr = 0;
  gState     = RUNNING;
  return Start::Ok;
}

void stop() {
  if (gState == RUNNING) gState = STOPPING;
}

bool service(uint32_t& doneCount) {
  if (gState == RUNNING) {
    rmt_transmit_config_t tc = {};
    tc.loop_count = 0;
    tc.flags.eot_level = 1;          // idle HIGH (released) between and after closures
    tc.flags.queue_nonblocking = 1;  // never block loop()
    while (gEnq < gCount && gEnq - gDone < DEPTH) {
      const uint32_t v = gEnq % VARIANTS;
      const esp_err_t e = rmt_transmit(gChan, gEnc, gSyms[v], gNSyms[v] * sizeof(rmt_symbol_word_t), &tc);
      if (e != ESP_OK) {
        gLastTxErr = static_cast<int32_t>(e);
        gState = STOPPING;
        break;
      }
      ++gEnq;
    }
    if (gEnq >= gCount) gState = STOPPING;
  }
  if (gState == STOPPING && gDone == gEnq) {
    gState = IDLE;
    doneCount = gDone;
    return true;
  }
  return false;
}

bool busy() { return gState != IDLE; }

void info(Info& out) {
  out.initErr   = gInitErr;
  out.ready     = gReady ? 1 : 0;
  out.state     = gState;
  out.rate      = gRate;
  out.count     = gCount;
  out.enqueued  = gEnq;
  out.done      = gDone;
  out.lastTxErr = gLastTxErr;
  out.periodUs  = gPeriodUs;
}

}  // namespace lb

#endif  // OT_LOOPBACK
