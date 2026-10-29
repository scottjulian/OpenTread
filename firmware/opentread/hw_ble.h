// hw_ble.h - the NimBLE peripheral: RSC footpod `OpenTread` and the FTMS
// treadmill server next to it.
//
// Files: hw_ble.cpp (stack init, server callbacks, the link table + its lock,
// advertising and its watchdog, B lines, dump), hw_ble_rsc.cpp (RSC + Device
// Information services, RSC notify), hw_ble_ftms.cpp (FTMS service, Treadmill
// Data notify, Training/Machine Status, the Control Point responder with the
// global indication token). All wire formats and decisions are in the pure
// core: src/core/ble_payload.* (host tests: test/test_ble_payload.cpp).
//
// Concurrency:
//  - NimBLE host callbacks (core 0) only update the link table under a portMUX
//    spinlock (subscriptions, pending CP request + arrival time, the CP token
//    release, B-line events) and set an atomic advertising flag. They never
//    call NimBLE, Serial, the display, or allocate. connInfo is never read in
//    onStatus (NimBLE-Arduino 2.5.0 passes a zeroed descriptor there).
//  - Everything else runs in loop() (core 1): advertising restarts (flag +
//    1 s watchdog), the 1 Hz setValue + notify() from ONE snapshot, CP
//    indications (one in flight across all links, >= CP_IND_DEFER_US after
//    the write), Training/Machine Status notifies, every B/R/X line.
//  - notify() is always called WITHOUT a buffer (it goes through
//    ble_gatts_chr_updated and respects each peer's CCCD); notify(buf,len)
//    would bypass the CCCD in 2.5.0 and is never used.
//
// Heap: begin() runs after the display sprites and the first frame.
#pragma once

#include <stdint.h>

#include "config_hw.h"
#include "src/core/ble_payload.h"
#include "src/core/workout.h"

namespace bleh {

// NimBLEDevice::init("OpenTread") + GATT (RSC, DIS, FTMS when enabled) +
// advertising (the build's layout). Logs S lines (ble;init ..., the advert
// bytes) and a D heap line. Returns false if anything failed (logged as
// ERROR; the head unit keeps running without BLE).
bool begin(int64_t now);

// Every loop pass: B lines from the callbacks' events, advertising restart
// (flag or 1 s watchdog), the CP scheduler (indication when due). Returns
// the number of actions taken (0 = idle).
uint32_t service(int64_t now);

// The 1 Hz tick (same instant and values as the K line): RSC Measurement and
// Treadmill Data from ONE snapshot: setValue, notify() if anyone subscribed,
// R / X lines (verbosity 1).
void tick(const ot::ble::BleSnapshot& s);

// Every Workout event in order (drainWorkout): Training Status (on a class
// change) and Machine Status (start/stop) notifications, B ts / ms lines.
void onWorkoutEvent(int64_t now, const ot::WorkoutEvent& e);

// Peers subscribed to RSC Measurement or Treadmill Data (the watch icon and
// H subscribed_peers). 0 before begin().
uint8_t subscribedPeers();

// Banner tokens: the linked NimBLE-Arduino version and the advert layout.
const char* nimbleVersion();
const char* advLayoutName();

// 'd' dump lines.
void dump(int64_t now);

}  // namespace bleh
