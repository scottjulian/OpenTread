// NimBLEDevice.h - emulation stub of the NimBLE-Arduino 2.5.0 API subset the
// firmware uses (test/emu). It models the behaviour the firmware depends on,
// as read in the 2.5.0 sources:
//  - notify() with no buffer = ble_gatts_chr_updated: sent only to peers
//    whose CCCD has the notify bit. notify(buf,len) is NOT provided, so any
//    use of it fails to compile here.
//  - indicate(buf,len,conn) = ble_gattc_indicate_custom: NO CCCD check; a
//    failure (unknown link, injected error) calls onStatus synchronously in
//    the caller and returns false.
//  - onStatus(chr, connInfo, code) for indications only on completion
//    (14 ack, 13 timeout, 7 not connected, ...) with a ZEROED connInfo
//    (conn handle 0 whatever the link).
//  - a disconnect first fails the link's pending indication (ENOTCONN), then
//    calls onDisconnect (ble_gap_conn_broken order: gatts/gattc first).
//  - advertising stops when a central connects; a central can only connect
//    while advertising (so a missing restart is visible).
//  - writes longer than the characteristic's max length are rejected
//    before onWrite.
// The emulator also counts protocol violations by the firmware: an
// indication to a peer without the indicate CCCD, two indications in
// flight at once (across all links), an indication < 5 ms after the write
// it answers, NimBLE calls made while the firmware holds its spinlock.
#pragma once

#include <cstdint>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

#include "emu_platform.h"

#define NIMBLE_CPP_VERSION_MAJOR 2
#define NIMBLE_CPP_VERSION_MINOR 5
#define NIMBLE_CPP_VERSION_PATCH 0
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 3
#define CONFIG_NIMBLE_CPP_ATT_VALUE_INIT_LENGTH 20
#define BLE_HS_IO_NO_INPUT_OUTPUT 3
#define BLE_HS_CONN_HANDLE_NONE 0xffff
#define BLE_ATT_ATTR_MAX_LEN 512

typedef enum {
  READ = 0x0002,
  WRITE_NR = 0x0004,
  WRITE = 0x0008,
  NOTIFY = 0x0010,
  INDICATE = 0x0020,
} NIMBLE_PROPERTY;

class NimBLEUUID {
 public:
  explicit NimBLEUUID(uint16_t u) : u16(u) {}
  uint16_t u16;
};

class NimBLEAddress {
 public:
  NimBLEAddress() = default;
  explicit NimBLEAddress(const uint8_t v[6]) { std::memcpy(val_, v, 6); }
  const uint8_t* getVal() const { return val_; }

 private:
  uint8_t val_[6] = {};
};

class NimBLEConnInfo {
 public:
  NimBLEConnInfo() = default;
  NimBLEConnInfo(uint16_t h, const NimBLEAddress& a) : h_(h), a_(a) {}
  uint16_t getConnHandle() const { return h_; }
  NimBLEAddress getAddress() const { return a_; }

 private:
  uint16_t h_ = 0;  // a default (zeroed) descriptor reports handle 0, like 2.5.0's onStatus
  NimBLEAddress a_;
};

class NimBLECharacteristic;
class NimBLEServer;

class NimBLECharacteristicCallbacks {
 public:
  virtual ~NimBLECharacteristicCallbacks() {}
  virtual void onRead(NimBLECharacteristic*, NimBLEConnInfo&) {}
  virtual void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) {}
  virtual void onStatus(NimBLECharacteristic*, int) {}
  virtual void onStatus(NimBLECharacteristic*, NimBLEConnInfo&, int) {}
  virtual void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t) {}
};

class NimBLEServerCallbacks {
 public:
  virtual ~NimBLEServerCallbacks() {}
  virtual void onConnect(NimBLEServer*, NimBLEConnInfo&) {}
  virtual void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) {}
};

namespace emu {
namespace ble {

struct Notification {
  int64_t t;
  uint16_t uuid;
  std::vector<uint8_t> bytes;
};
enum class AckPolicy : uint8_t { Auto, Never, FailSync };
struct Peer {
  bool up = false;
  uint8_t addr[6] = {};
  AckPolicy policy = AckPolicy::Auto;
  int64_t ackDelayUs = 30000;
  std::vector<Notification> notes;       // notifications received
  std::vector<Notification> inds;        // indications received
  int64_t lastCpWriteT = -1;
};
struct Outstanding {
  bool active = false;
  uint16_t conn = 0;
  int64_t ackAt = 0;
  bool neverAck = false;
  NimBLECharacteristic* chr = nullptr;
};

inline std::map<uint16_t, Peer> peers;
inline Outstanding outstanding;
inline bool advActive = false;
inline uint32_t advStarts = 0;
inline std::vector<uint8_t> advData, scanData;
inline bool scanRespEnabled = false;
// violations
inline int indNoCccd = 0, indDouble = 0, indEarly = 0, nimbleUnderLock = 0, badWrites = 0;
inline uint32_t indicateCalls = 0, notifyCalls = 0;
inline std::vector<NimBLECharacteristic*> chars;
inline NimBLEServer* server = nullptr;

inline void checkLock() {
  if (::emu::lockDepth != 0) ++nimbleUnderLock;
}

}  // namespace ble
}  // namespace emu

class NimBLECharacteristic {
 public:
  NimBLECharacteristic(uint16_t uuid, uint32_t props, uint16_t maxLen) : uuid_(uuid), props_(props), maxLen_(maxLen) {}
  void setCallbacks(NimBLECharacteristicCallbacks* cb) { cb_ = cb != nullptr ? cb : &defaultCb(); }
  NimBLECharacteristicCallbacks* cb() { return cb_; }
  void setValue(const uint8_t* d, size_t n) {
    emu::ble::checkLock();
    if (n > maxLen_) return;  // NimBLE logs "val > max" and keeps the old value
    value_.assign(d, d + n);
  }
  size_t getLength() const { return value_.size(); }
  template <typename T>
  T getValue(time_t* = nullptr, bool skipSizeCheck = false) const {
    T out{};
    if (!skipSizeCheck && value_.size() < sizeof(T)) return out;
    // capacity >= 20 B (init length): emulate reading the whole buffer
    uint8_t buf[sizeof(T)] = {};
    std::memcpy(buf, value_.data(), value_.size() < sizeof(T) ? value_.size() : sizeof(T));
    std::memcpy(&out, buf, sizeof(T));
    return out;
  }
  const std::vector<uint8_t>& value() const { return value_; }
  uint16_t uuid() const { return uuid_; }
  uint32_t props() const { return props_; }
  uint16_t maxLen() const { return maxLen_; }
  uint16_t cccd(uint16_t conn) const {
    auto it = cccd_.find(conn);
    return it == cccd_.end() ? 0 : it->second;
  }
  void setCccd(uint16_t conn, uint16_t v) { cccd_[conn] = v; }
  void clearCccd(uint16_t conn) { cccd_.erase(conn); }

  bool notify(uint16_t connHandle = BLE_HS_CONN_HANDLE_NONE) const {
    emu::ble::checkLock();
    ++emu::ble::notifyCalls;
    for (auto& kv : emu::ble::peers) {
      if (!kv.second.up) continue;
      if (connHandle != BLE_HS_CONN_HANDLE_NONE && kv.first != connHandle) continue;
      if ((cccd(kv.first) & 0x1u) == 0) continue;
      kv.second.notes.push_back({emu::gNow, uuid_, value_});
    }
    return true;
  }
  bool indicate(const uint8_t* v, size_t n, uint16_t conn) {
    emu::ble::checkLock();
    ++emu::ble::indicateCalls;
    auto it = emu::ble::peers.find(conn);
    if (it == emu::ble::peers.end() || !it->second.up || it->second.policy == emu::ble::AckPolicy::FailSync) {
      NimBLEConnInfo zero;
      const int rc = (it == emu::ble::peers.end() || !it->second.up) ? 7 : 6;  // ENOTCONN / ENOMEM
      cb_->onStatus(this, zero, rc);
      return false;
    }
    emu::ble::Peer& p = it->second;
    if ((cccd(conn) & 0x2u) == 0) ++emu::ble::indNoCccd;
    if (emu::ble::outstanding.active) ++emu::ble::indDouble;
    if (p.lastCpWriteT >= 0 && emu::gNow - p.lastCpWriteT < 5000) ++emu::ble::indEarly;
    p.inds.push_back({emu::gNow, uuid_, std::vector<uint8_t>(v, v + n)});
    const bool never = p.policy == emu::ble::AckPolicy::Never;
    // Never: the completion is lost (no onStatus at all); the stack forgets the
    // indication after 30 s, so only the firmware's 35 s fallback frees the token.
    emu::ble::outstanding = {true, conn, emu::gNow + (never ? 30000000 : p.ackDelayUs), never, this};
    return true;
  }

 private:
  static NimBLECharacteristicCallbacks& defaultCb() {
    static NimBLECharacteristicCallbacks d;
    return d;
  }
  uint16_t uuid_;
  uint32_t props_;
  uint16_t maxLen_;
  NimBLECharacteristicCallbacks* cb_ = &defaultCb();
  std::vector<uint8_t> value_;
  std::map<uint16_t, uint16_t> cccd_;
};

class NimBLEService {
 public:
  explicit NimBLEService(uint16_t u) : uuid(u) {}
  NimBLECharacteristic* createCharacteristic(const NimBLEUUID& u, uint32_t props, uint16_t maxLen = 512) {
    auto* c = new NimBLECharacteristic(u.u16, props, maxLen);
    emu::ble::chars.push_back(c);
    return c;
  }
  uint16_t uuid;
};

class NimBLEServer {
 public:
  void setCallbacks(NimBLEServerCallbacks* cb, bool = true) { cb_ = cb; }
  NimBLEServerCallbacks* cb() { return cb_; }
  void advertiseOnDisconnect(bool on) { advOnDisc = on; }
  NimBLEService* createService(const NimBLEUUID& u) {
    services.push_back(new NimBLEService(u.u16));
    return services.back();
  }
  uint8_t getConnectedCount() const {
    uint8_t n = 0;
    for (auto& kv : emu::ble::peers)
      if (kv.second.up) ++n;
    return n;
  }
  std::vector<NimBLEService*> services;
  bool advOnDisc = true;

 private:
  NimBLEServerCallbacks* cb_ = nullptr;
};

class NimBLEAdvertisementData {
 public:
  bool addData(const uint8_t* d, size_t n) {
    if (payload_.size() + n > 31) return false;
    payload_.insert(payload_.end(), d, d + n);
    return true;
  }
  std::vector<uint8_t> getPayload() const { return payload_; }

 private:
  std::vector<uint8_t> payload_;
};

class NimBLEAdvertising {
 public:
  void enableScanResponse(bool on) { emu::ble::scanRespEnabled = on; }
  bool setAdvertisementData(const NimBLEAdvertisementData& d) {
    emu::ble::advData = d.getPayload();
    return true;
  }
  bool setScanResponseData(const NimBLEAdvertisementData& d) {
    emu::ble::scanData = d.getPayload();
    return true;
  }
  bool start() {
    emu::ble::checkLock();
    if (emu::ble::advActive) return true;
    emu::ble::advActive = true;
    ++emu::ble::advStarts;
    return true;
  }
  bool stop() {
    emu::ble::advActive = false;
    return true;
  }
  bool isAdvertising() { return emu::ble::advActive; }
};

class NimBLEDevice {
 public:
  // A device log (2026-10-26) shows ~270 ms
  // from ble;init to ble;ready: emulate it, so a schedule baselined before
  // the BLE init shows up as lateness in the first G line.
  static bool init(const std::string&) {
    emu::gNow += 270000;
    return true;
  }
  static void setSecurityAuth(bool, bool, bool) {}
  static void setSecurityIOCap(uint8_t) {}
  static NimBLEServer* createServer() {
    if (emu::ble::server == nullptr) emu::ble::server = new NimBLEServer();
    return emu::ble::server;
  }
  static NimBLEAdvertising* getAdvertising() {
    static NimBLEAdvertising a;
    return &a;
  }
  static NimBLEAddress getAddress() {
    const uint8_t v[6] = {0x02, 0x00, 0x00, 0xa7, 0xdb, 0xcc};
    return NimBLEAddress(v);
  }
};

// ---- the emulated centrals (the test driver calls these) --------------------
namespace emu {
namespace ble {

inline NimBLECharacteristic* findChr(uint16_t uuid) {
  for (auto* c : chars)
    if (c->uuid() == uuid) return c;
  return nullptr;
}
inline NimBLEAddress addrOf(uint16_t h) {
  const uint8_t a[6] = {static_cast<uint8_t>(h), 0x11, 0x22, 0x33, 0x44, 0x55};
  return NimBLEAddress(a);
}

// A central connects (only possible while advertising).
inline bool connect(uint16_t h) {
  if (!advActive || server == nullptr) return false;
  if (server->getConnectedCount() >= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) return false;
  advActive = false;  // the controller stops advertising on a connection
  Peer& p = peers[h];
  p = Peer{};
  p.up = true;
  NimBLEConnInfo ci(h, addrOf(h));
  server->cb()->onConnect(server, ci);
  return true;
}
inline void disconnect(uint16_t h, int reason = 0x13) {
  auto it = peers.find(h);
  if (it == peers.end() || !it->second.up) return;
  if (outstanding.active && outstanding.conn == h) {  // gatts/gattc connection_broken first
    outstanding.active = false;
    NimBLEConnInfo zero;
    outstanding.chr->cb()->onStatus(outstanding.chr, zero, 7);
  }
  for (auto* c : chars) c->clearCccd(h);
  it->second.up = false;
  NimBLEConnInfo ci(h, addrOf(h));
  server->cb()->onDisconnect(server, ci, reason);
}
inline void subscribe(uint16_t h, uint16_t uuid, uint16_t value) {
  NimBLECharacteristic* c = findChr(uuid);
  if (c == nullptr) return;
  c->setCccd(h, value);
  NimBLEConnInfo ci(h, addrOf(h));
  c->cb()->onSubscribe(c, ci, value);
}
inline bool write(uint16_t h, uint16_t uuid, const std::vector<uint8_t>& v) {
  NimBLECharacteristic* c = findChr(uuid);
  if (c == nullptr || !peers[h].up) return false;
  if ((c->props() & (WRITE | WRITE_NR)) == 0 || v.size() > c->maxLen()) {
    ++badWrites;
    return false;  // ATT error, no onWrite
  }
  c->setValue(v.data(), v.size());
  peers[h].lastCpWriteT = gNow;
  NimBLEConnInfo ci(h, addrOf(h));
  c->cb()->onWrite(c, ci);
  return true;
}
inline std::vector<uint8_t> read(uint16_t uuid) {
  NimBLECharacteristic* c = findChr(uuid);
  return c == nullptr ? std::vector<uint8_t>() : c->value();
}
// The host task's turn: acknowledge a due indication.
inline void service() {
  if (outstanding.active && gNow >= outstanding.ackAt) {
    outstanding.active = false;
    if (outstanding.neverAck) return;  // lost completion: no callback
    NimBLEConnInfo zero;
    outstanding.chr->cb()->onStatus(outstanding.chr, zero, 14);
  }
}
// An ATT timeout on the outstanding indication (status 13).
inline void timeoutOutstanding() {
  if (!outstanding.active) return;
  outstanding.active = false;
  NimBLEConnInfo zero;
  outstanding.chr->cb()->onStatus(outstanding.chr, zero, 13);
}

}  // namespace ble
}  // namespace emu
