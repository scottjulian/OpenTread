// hw_ble_rsc.cpp - RSC 0x1814 footpod service + Device Information 0x180A
// Reuses the encoding of the FR955-proven earlier footpod bridge:
// Measurement NOTIFY only, Feature 06 00, Sensor Location 00, no SC Control
// Point (a known deviation from RSCS C.1; the FR955 accepts it without one).
#include <NimBLEDevice.h>
#include <string.h>

#include "hw_ble_priv.h"

namespace bleh {
namespace detail {

namespace {
namespace ble = ot::ble;

NimBLECharacteristic* gMeas = nullptr;
SubCallbacks          gMeasCb(ble::Chr::Rsc);

bool readOnly(NimBLEService* svc, uint16_t uuid, const uint8_t* v, size_t n) {
  NimBLECharacteristic* c = svc->createCharacteristic(NimBLEUUID(uuid), NIMBLE_PROPERTY::READ, static_cast<uint16_t>(n));
  if (c == nullptr) return false;
  c->setValue(v, n);
  return true;
}
bool readOnlyStr(NimBLEService* svc, uint16_t uuid, const char* s) {
  return readOnly(svc, uuid, reinterpret_cast<const uint8_t*>(s), strlen(s));
}
}  // namespace

bool createRscAndDis(NimBLEServer* server, const char* fwRevision) {
  NimBLEService* rsc = server->createService(NimBLEUUID(ble::UUID_RSC_SERVICE));
  if (rsc == nullptr) return false;
  gMeas = rsc->createCharacteristic(NimBLEUUID(ble::UUID_RSC_MEASUREMENT), NIMBLE_PROPERTY::NOTIFY,
                                    static_cast<uint16_t>(ble::RSC_MEAS_LEN));
  if (gMeas == nullptr) return false;
  gMeas->setCallbacks(&gMeasCb);
  uint8_t idle[ble::RSC_MEAS_LEN];
  ble::encodeRscMeasurement(0, 0, idle);
  gMeas->setValue(idle, sizeof(idle));
  if (!readOnly(rsc, ble::UUID_RSC_FEATURE, ble::RSC_FEATURE, sizeof(ble::RSC_FEATURE))) return false;
  const uint8_t loc = ble::SENSOR_LOCATION;
  if (!readOnly(rsc, ble::UUID_SENSOR_LOCATION, &loc, 1)) return false;

  NimBLEService* dis = server->createService(NimBLEUUID(ble::UUID_DIS_SERVICE));
  if (dis == nullptr) return false;
  return readOnlyStr(dis, ble::UUID_DIS_MANUFACTURER, ble::DIS_MANUFACTURER) &&
         readOnlyStr(dis, ble::UUID_DIS_MODEL, ble::DIS_MODEL) && readOnlyStr(dis, ble::UUID_DIS_FIRMWARE, fwRevision) &&
         readOnlyStr(dis, ble::UUID_DIS_HARDWARE, ble::DIS_HARDWARE);
}

void rscTick(const ble::BleSnapshot& s, bool anySubscriber, uint8_t out[ble::RSC_MEAS_LEN]) {
  ble::encodeRsc(s, out);
  if (gMeas == nullptr) return;
  gMeas->setValue(out, ble::RSC_MEAS_LEN);  // capacity 20 B: never reallocates
  if (anySubscriber) gMeas->notify();       // NO buffer argument: CCCD respected (ble_gatts_chr_updated)
}

}  // namespace detail
}  // namespace bleh
