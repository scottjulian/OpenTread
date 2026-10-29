// hw_ble_priv.h - shared between hw_ble.cpp, hw_ble_rsc.cpp and
// hw_ble_ftms.cpp only (see hw_ble.h for the rules).
#pragma once

#include <NimBLEDevice.h>
#include <stdint.h>

#include "config_hw.h"
#include "src/core/ble_payload.h"
#include "src/core/workout.h"

namespace bleh {
namespace detail {

// The link table and its spinlock (hw_ble.cpp). Every LinkTable call, from
// a callback (core 0) or loop() (core 1), holds the lock; the critical
// sections are a few hundred ns of memory work, no calls out.
void lock();
void unlock();
struct Guard {
  Guard() { lock(); }
  ~Guard() { unlock(); }
  Guard(const Guard&) = delete;
  Guard& operator=(const Guard&) = delete;
};
ot::ble::LinkTable& table();  // only under the lock

// Host-task time stamp for the B lines (esp_timer_get_time is callable from
// any context).
int64_t nowUs();

// Subscription callbacks for a notify characteristic (RSC Measurement,
// Treadmill Data, Training Status, Machine Status): onSubscribe -> the table.
class SubCallbacks : public NimBLECharacteristicCallbacks {
 public:
  explicit SubCallbacks(ot::ble::Chr c) : chr_(c) {}
  void onSubscribe(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo, uint16_t subValue) override;

 private:
  ot::ble::Chr chr_;
};

// hw_ble_rsc.cpp: RSC 0x1814 (Measurement NOTIFY, Feature, Sensor Location)
// and Device Information 0x180A. Setup only. Returns false on any failure.
bool createRscAndDis(NimBLEServer* server, const char* fwRevision);
// 1 Hz: setValue(8 B) + notify() if anyone subscribed. Fills `out`.
void rscTick(const ot::ble::BleSnapshot& s, bool anySubscriber, uint8_t out[ot::ble::RSC_MEAS_LEN]);

#if OT_FTMS_ENABLE
// hw_ble_ftms.cpp: FTMS 0x1826 (all five characteristics). Setup only.
bool createFtms(NimBLEServer* server);
// 1 Hz: Treadmill Data setValue(9 B) + notify() if anyone subscribed.
void ftmsTick(const ot::ble::BleSnapshot& s, bool anySubscriber, uint8_t out[ot::ble::TD_LEN]);
// Workout event -> Training / Machine Status (loop task). `ready` = GATT up.
void ftmsWorkoutEvent(int64_t now, const ot::WorkoutEvent& e, bool ready);
// The CP scheduler: at most one indication per call. Returns 1 if it sent.
uint32_t ftmsServiceCp(int64_t now);
// dump helper
uint8_t ftmsTrainingCode();
#endif

}  // namespace detail
}  // namespace bleh
