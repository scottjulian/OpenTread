// hw_loopback.h - OT_LOOPBACK=1 bench builds only: an RMT pulse-train
// generator on LOOPBACK_PIN (GPIO 22), jumpered to GPIO 27 with the
// treadmill harness unplugged. The production ISR, ring and PulseFilter see the
// train exactly as they would see the reed switch.
//
// Each closure is one RMT transaction of exactly one period (1/rate s):
//   closure bounce burst  -> 3, 5, 3, 3 edges (cycling; mean 3.5) within < 3 ms
//   closed dwell          -> 35% of the period in total
//   release chatter       -> 1 or 3 edges (alternating; the LOW blip is 200 us)
//   open rest             -> the remainder (>= 5 ms at the maximum rate)
// The output is OPEN-DRAIN (like the reed switch: it only pulls LOW, GPIO
// 27's pull-up makes the HIGH). Expected result: exactly 1 accepted P per
// closure, so "CYD P count == generated count".
//
// Transactions are queued (depth 4) from loop(); the hardware keeps the
// timing, so a loop stall of up to 4 periods changes nothing, and a longer
// one only lengthens an open phase (the line idles HIGH between
// transactions). The count is exact by construction: the RMT done callback
// (ISR) counts completed closures.
//
// All channel/encoder allocation happens in begin() (setup). No dynamic
// allocation afterwards. This header has no Arduino/IDF includes.
#pragma once

#include <stdint.h>

#include "config_hw.h"  // OT_LOOPBACK

namespace lb {

constexpr uint32_t RATE_MIN = 1;   // closures per second
constexpr uint32_t RATE_MAX = 40;  // 25 ms period: still > the 20 ms lockout
constexpr uint32_t COUNT_MAX = 10000000;

enum class Start : uint8_t { Ok, Busy, BadArgs, NotReady };

struct Info {
  int32_t  initErr;     // first failing RMT call in begin() (0 = ESP_OK)
  uint8_t  ready;       // channel + encoder created and enabled
  uint8_t  state;       // 0 idle, 1 running, 2 stopping
  uint32_t rate;
  uint32_t count;       // requested
  uint32_t enqueued;    // transactions accepted by rmt_transmit
  uint32_t done;        // transactions completed (ISR count)
  int32_t  lastTxErr;   // last rmt_transmit error (0 = none)
  uint32_t periodUs;
};

#if OT_LOOPBACK
// Create the TX channel (GPIO 22, 1 MHz, open-drain, idle HIGH) and encoder.
bool  begin();
// Start a train of `count` closures at `rate` per second. Busy while a
// previous train (or its graceful stop) is still in flight.
Start start(uint32_t rate, uint32_t count);
// Graceful stop: no new closures are queued; the up to 4 already queued
// finish (so the line always ends HIGH and every counted closure is whole).
void  stop();
// Call every loop pass: tops up the queue. Returns true exactly once when a
// train has ended (all queued closures done); doneCount = closures generated.
bool  service(uint32_t& doneCount);
bool  busy();
void  info(Info& out);
#endif

}  // namespace lb
