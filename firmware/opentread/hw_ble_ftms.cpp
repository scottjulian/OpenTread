// hw_ble_ftms.cpp - FTMS 0x1826 treadmill server next to the RSC footpod.
// Compiled only with OT_FTMS_ENABLE=1.
//
//   0x2ACC Feature         READ    04 10 00 00 00 00 00 00 (only what we send)
//   0x2ACD Treadmill Data  NOTIFY  9 B at 1 Hz in every state (flags 0x0404)
//   0x2AD3 Training Status READ + NOTIFY  [00, code], notify on a class change
//   0x2AD9 Control Point   WRITE + WRITE_NR + INDICATE, max 20 B
//   0x2ADA Machine Status  NOTIFY  04 on a confirmed start, 02 01 on a stop
//
// Control Point (the responder and the token logic are pure:
// ot::ble::cpRespond and ot::ble::LinkTable):
//  - onWrite (host task, core 0) copies the request into that link's slot
//    with its arrival time; nothing else.
//  - loop() indicates the response only when no CP indication is in flight
//    on ANY link (one global token), the link has CP indications enabled
//    (indicate() bypasses the CCCD in 2.5.0, so this check is ours), and
//    the request arrived >= CP_IND_DEFER_US ago (the ATT Write Response,
//    sent by the host after onWrite returns, goes out first).
//  - onStatus releases the token: 14 ack, 13 ATT timeout, anything else
//    drop. Its connInfo is NEVER read (2.5.0 passes a zeroed descriptor, so
//    conn handle 0 whatever the link). Also released on the token link's
//    disconnect and by the 35 s fallback.
//  - No CCCD: the response is dropped (B cp_drop no_cccd) with no effect.
//    The spec wants ATT error 0xFD, which onWrite can't return in 2.5.0
//    (documented deviation).
#include "config_hw.h"

#if OT_FTMS_ENABLE

#include <NimBLEDevice.h>
#include <string.h>

#include "hw_ble_priv.h"

namespace bleh {
namespace detail {

namespace {
namespace ble = ot::ble;

// onWrite copies the value without allocating: getValue<T>(nullptr, true)
// reads sizeof(T) bytes from the value buffer. The buffer's capacity is the
// initial length (CONFIG_NIMBLE_CPP_ATT_VALUE_INIT_LENGTH, 20) and only grows,
// and NimBLE refuses writes longer than the characteristic's max (20), so 20
// bytes are always readable; getLength() says how many are the request.
struct CpBuf {
  uint8_t b[ble::CP_MAX_LEN];
};
static_assert(CONFIG_NIMBLE_CPP_ATT_VALUE_INIT_LENGTH >= ble::CP_MAX_LEN,
              "the CP value buffer must hold CP_MAX_LEN bytes from the start");

NimBLECharacteristic* gTd = nullptr;
NimBLECharacteristic* gTs = nullptr;
NimBLECharacteristic* gCp = nullptr;
NimBLECharacteristic* gMs = nullptr;
ble::FtmsStatus       gStatus;  // loop task only

class CpCallbacks : public NimBLECharacteristicCallbacks {
 public:
  using NimBLECharacteristicCallbacks::onStatus;  // the deprecated 2-argument overload stays the library's no-op
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    const int64_t t = nowUs();
    const size_t n = chr->getLength();
    const CpBuf v = chr->getValue<CpBuf>(nullptr, true);
    Guard g;
    table().onCpWrite(t, connInfo.getConnHandle(), v.b, n);  // n > 20 cannot happen (max_len); the table drops it anyway
  }
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    const int64_t t = nowUs();
    Guard g;
    table().onSubscribe(t, connInfo.getConnHandle(), ble::Chr::Cp, subValue);
  }
  // Indication completion. connInfo is deliberately unused (zeroed in 2.5.0).
  void onStatus(NimBLECharacteristic*, NimBLEConnInfo& /*unused*/, int code) override {
    const int64_t t = nowUs();
    Guard g;
    table().onCpStatus(t, static_cast<int32_t>(code));
  }
};

CpCallbacks  gCpCb;
SubCallbacks gTdCb(ble::Chr::Td);
SubCallbacks gTsCb(ble::Chr::Ts);
SubCallbacks gMsCb(ble::Chr::Ms);

void logStatus(int64_t now, ble::BleEv ev, const uint8_t* d, size_t n) {
  Guard g;
  table().logStatus(now, ev, d, n);
}

}  // namespace

bool createFtms(NimBLEServer* server) {
  NimBLEService* svc = server->createService(NimBLEUUID(ble::UUID_FTMS_SERVICE));
  if (svc == nullptr) return false;
  NimBLECharacteristic* feat = svc->createCharacteristic(NimBLEUUID(ble::UUID_FTMS_FEATURE), NIMBLE_PROPERTY::READ,
                                                         static_cast<uint16_t>(sizeof(ble::FTMS_FEATURE)));
  if (feat == nullptr) return false;
  feat->setValue(ble::FTMS_FEATURE, sizeof(ble::FTMS_FEATURE));
  // Treadmill Data: NOTIFY only (the spec excludes READ).
  gTd = svc->createCharacteristic(NimBLEUUID(ble::UUID_TREADMILL_DATA), NIMBLE_PROPERTY::NOTIFY,
                                  static_cast<uint16_t>(ble::TD_LEN));
  if (gTd == nullptr) return false;
  gTd->setCallbacks(&gTdCb);
  uint8_t td[ble::TD_LEN];
  ble::encodeTreadmillData(0, 0, 0, td);
  gTd->setValue(td, sizeof(td));
  gTs = svc->createCharacteristic(NimBLEUUID(ble::UUID_TRAINING_STATUS), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
                                  2);
  if (gTs == nullptr) return false;
  gTs->setCallbacks(&gTsCb);
  uint8_t ts[2];
  ble::encodeTrainingStatus(ot::WorkoutState::Idle, ts);  // 00 01 at boot
  gTs->setValue(ts, sizeof(ts));
  gCp = svc->createCharacteristic(NimBLEUUID(ble::UUID_FTMS_CONTROL_POINT),
                                  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::INDICATE,
                                  static_cast<uint16_t>(ble::CP_MAX_LEN));
  if (gCp == nullptr) return false;
  gCp->setCallbacks(&gCpCb);
  gMs = svc->createCharacteristic(NimBLEUUID(ble::UUID_MACHINE_STATUS), NIMBLE_PROPERTY::NOTIFY, 2);
  if (gMs == nullptr) return false;
  gMs->setCallbacks(&gMsCb);
  gStatus.reset();
  return true;
}

void ftmsTick(const ble::BleSnapshot& s, bool anySubscriber, uint8_t out[ble::TD_LEN]) {
  ble::encodeTd(s, out);
  if (gTd == nullptr) return;
  gTd->setValue(out, ble::TD_LEN);
  if (anySubscriber) gTd->notify();  // NO buffer argument: CCCD respected
}

void ftmsWorkoutEvent(int64_t now, const ot::WorkoutEvent& e, bool ready) {
  const ble::FtmsStatus::Out o = gStatus.onEvent(e);
  if (!ready || gTs == nullptr || gMs == nullptr) return;
  bool tsSub, msSub;
  {
    Guard g;
    tsSub = table().anySub(ble::Chr::Ts);
    msSub = table().anySub(ble::Chr::Ms);
  }
  if (o.msLen > 0) {
    gMs->setValue(o.ms, o.msLen);
    if (msSub) gMs->notify();
    logStatus(now, ble::BleEv::Ms, o.ms, o.msLen);
  }
  if (o.tsChanged) {
    gTs->setValue(o.ts, sizeof(o.ts));  // READ returns the new code even with no subscriber
    if (tsSub) gTs->notify();
    logStatus(now, ble::BleEv::Ts, o.ts, sizeof(o.ts));
  }
}

uint32_t ftmsServiceCp(int64_t now) {
  if (gCp == nullptr) return 0;
  ble::CpSend s;
  bool send;
  {
    Guard g;
    send = table().pollCp(now, s);
  }
  if (!send) return 0;
  // Outside the lock: a synchronous failure calls onStatus in this task,
  // which takes the lock itself (and releases the token).
  if (!gCp->indicate(s.rsp, ble::CP_RSP_LEN, s.conn)) {
    Guard g;
    table().cpSendFailed(now, s.seq, -1);  // no-op if onStatus already released it
  }
  return 1;
}

uint8_t ftmsTrainingCode() { return gStatus.trainingCode(); }

}  // namespace detail
}  // namespace bleh

#endif  // OT_FTMS_ENABLE
