// hw_ble.cpp - NimBLE stack, server callbacks, link table, advertising,
// B lines (see hw_ble.h).
#include "hw_ble.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <atomic>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "debug_serial.h"
#include "hw_ble_priv.h"
#include "version.h"

// The pinned library. 1.4.x crash-loops on core 3.x at runtime;
// another 2.x is a recorded decision, not a silent switch.
static_assert(NIMBLE_CPP_VERSION_MAJOR == 2 && NIMBLE_CPP_VERSION_MINOR == 5 && NIMBLE_CPP_VERSION_PATCH == 0,
              "NimBLE-Arduino must be 2.5.0");
// Library default of 3 links (nimconfig.h is never edited); the link
// table has exactly that many slots.
static_assert(CONFIG_BT_NIMBLE_MAX_CONNECTIONS == ot::ble::MAX_LINKS, "link table size = NimBLE max connections (3)");

namespace bleh {

namespace {

namespace ble = ot::ble;

#define OT_BLE_STR2_(x) #x
#define OT_BLE_STR_(x) OT_BLE_STR2_(x)
constexpr const char* NIMBLE_VERSION_STR = OT_BLE_STR_(NIMBLE_CPP_VERSION_MAJOR) "." OT_BLE_STR_(
    NIMBLE_CPP_VERSION_MINOR) "." OT_BLE_STR_(NIMBLE_CPP_VERSION_PATCH);

constexpr ble::AdvLayout LAYOUT = static_cast<ble::AdvLayout>(OT_ADV_LAYOUT);
static_assert(ble::advMentionsFtms(LAYOUT) == (OT_FTMS_ENABLE != 0), "0x1826 advertised iff the FTMS service exists");

portMUX_TYPE        gMux = portMUX_INITIALIZER_UNLOCKED;
ble::LinkTable      gTable;
std::atomic<bool>   gAdvWanted{false};
bool                gReady = false;  // GATT + advertising set up
NimBLEServer*       gServer = nullptr;
NimBLEAdvertising*  gAdv = nullptr;
int64_t             gNextAdvWatchdog = 0;
uint32_t            gAdvRestarts = 0, gAdvFails = 0;
uint32_t            gTicks = 0;
uint32_t            gReportedEvDrops = 0;
ble::AdvPair        gAdvPair;
char                gFwRev[40];

// Server callbacks (host task, core 0): table + flag only.
class ServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer*, NimBLEConnInfo& connInfo) override {
    const NimBLEAddress a = connInfo.getAddress();
    const int64_t t = detail::nowUs();
    {
      detail::Guard g;
      gTable.onConnect(t, connInfo.getConnHandle(), a.getVal());
    }
    gAdvWanted.store(true);  // advertising stops on every connect: resume for the next central
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& connInfo, int reason) override {
    const int64_t t = detail::nowUs();
    {
      detail::Guard g;
      gTable.onDisconnect(t, connInfo.getConnHandle(), static_cast<int32_t>(reason));
    }
    gAdvWanted.store(true);  // advertiseOnDisconnect is false: loop() restarts it
  }
};
ServerCallbacks gServerCb;

void fmtAddr(const uint8_t* a, char* out, size_t n) {  // NimBLE stores the address little endian
  snprintf(out, n, "%02x:%02x:%02x:%02x:%02x:%02x", a[5], a[4], a[3], a[2], a[1], a[0]);
}

// One B line per event: B,<t>,<event>,<conn|->,<detail>  (no commas in detail)
void printEvent(const ble::BleEvent& e) {
  char conn[8];
  if (e.conn == ble::CONN_NONE) snprintf(conn, sizeof(conn), "-");
  else snprintf(conn, sizeof(conn), "%u", static_cast<unsigned>(e.conn));
  char hex[2 * ble::CP_MAX_LEN + 1];
  char d[96];
  switch (e.ev) {
    case ble::BleEv::Connect: {
      char a[20];
      fmtAddr(e.data, a, sizeof(a));
      if (e.why == ble::Why::NoSlot) snprintf(d, sizeof(d), "%s;no_slot", a);
      else snprintf(d, sizeof(d), "%s;links=%" PRId32, a, e.code);
      break;
    }
    case ble::BleEv::Disconnect:
      snprintf(d, sizeof(d), "reason=%" PRId32 ";pending_drop=%u;token=%u", e.code, static_cast<unsigned>(e.data[0]),
               static_cast<unsigned>(e.data[1]));
      break;
    case ble::BleEv::Sub:
      snprintf(d, sizeof(d), "%s=%" PRId32 "%s", ble::chrName(e.chr), e.code, e.why == ble::Why::NoSlot ? ";no_slot" : "");
      break;
    case ble::BleEv::Unsub:
      snprintf(d, sizeof(d), "%s%s", ble::chrName(e.chr), e.why == ble::Why::NoSlot ? ";no_slot" : "");
      break;
    case ble::BleEv::AdvRestart:
      snprintf(d, sizeof(d), "%s;ok=%" PRId32, ble::whyName(e.why), e.code);
      break;
    case ble::BleEv::CpWrite:
      ble::toHex(e.data, e.len, hex, sizeof(hex));
      if (e.code > static_cast<int32_t>(e.len)) snprintf(d, sizeof(d), "%s;len=%" PRId32, e.len ? hex : "-", e.code);
      else snprintf(d, sizeof(d), "%s", e.len ? hex : "-");
      break;
    case ble::BleEv::CpInd:
    case ble::BleEv::Ts:
    case ble::BleEv::Ms:
      ble::toHex(e.data, e.len, hex, sizeof(hex));
      snprintf(d, sizeof(d), "%s", e.len ? hex : "-");
      break;
    case ble::BleEv::CpAck:
      snprintf(d, sizeof(d), "code=%" PRId32, e.code);
      break;
    case ble::BleEv::CpDrop:
      switch (e.why) {
        case ble::Why::Replaced:
        case ble::Why::NoCccd: snprintf(d, sizeof(d), "%s;op=%02" PRIx32, ble::whyName(e.why), static_cast<uint32_t>(e.code)); break;
        case ble::Why::Empty:
        case ble::Why::NoSlot: snprintf(d, sizeof(d), "%s", ble::whyName(e.why)); break;
        default: snprintf(d, sizeof(d), "%s;code=%" PRId32, ble::whyName(e.why), e.code); break;
      }
      break;
    case ble::BleEv::CpTimeout:
      if (e.why == ble::Why::Fallback) snprintf(d, sizeof(d), "fallback");
      else snprintf(d, sizeof(d), "%s;code=%" PRId32, ble::whyName(e.why), e.code);
      break;
    default: snprintf(d, sizeof(d), "-"); break;
  }
  dbg::line(dbg::LV_ALWAYS, "B,%" PRId64 ",%s,%s,%s", e.t, ble::bleEvName(e.ev), conn, d);
}

uint32_t drainEvents() {
  uint32_t n = 0;
  ble::BleEvent e;
  for (;;) {
    bool got;
    {
      detail::Guard g;
      got = gTable.popEvent(e);
    }
    if (!got) break;
    printEvent(e);
    ++n;
  }
  return n;
}

bool applyAdvert(int64_t now) {
  if (!ble::buildAdvert(LAYOUT, ble::BLE_NAME, gAdvPair)) {
    dbg::status(now, "ERROR ble;advert layout %s does not fit 31 B", ble::advLayoutName(LAYOUT));
    return false;
  }
  NimBLEAdvertisementData primary, scan;
  if (!primary.addData(gAdvPair.primary.b, gAdvPair.primary.len)) {
    dbg::status(now, "ERROR ble;addData(primary) refused");
    return false;
  }
  const std::vector<uint8_t> pp = primary.getPayload();
  if (pp.size() != gAdvPair.primary.len || memcmp(pp.data(), gAdvPair.primary.b, pp.size()) != 0) {
    dbg::status(now, "ERROR ble;primary payload differs from the builder's bytes");
    return false;
  }
  const bool hasScan = gAdvPair.scanResponse.len > 0;
  if (hasScan && !scan.addData(gAdvPair.scanResponse.b, gAdvPair.scanResponse.len)) {
    dbg::status(now, "ERROR ble;addData(scan response) refused");
    return false;
  }
  // enableScanResponse() resets the "data set" flag, so it goes first.
  gAdv->enableScanResponse(hasScan);
  if (!gAdv->setAdvertisementData(primary)) {
    dbg::status(now, "ERROR ble;setAdvertisementData failed");
    return false;
  }
  if (hasScan && !gAdv->setScanResponseData(scan)) {
    dbg::status(now, "ERROR ble;setScanResponseData failed");
    return false;
  }
  return true;
}

// Links up = the larger of NimBLE's count and the link table's (either one
// at the 3-link max means no advertising).
unsigned linksUp() {
  unsigned tableLinks;
  {
    detail::Guard g;
    tableLinks = gTable.links();
  }
  const unsigned nimbleLinks = gServer != nullptr ? static_cast<unsigned>(gServer->getConnectedCount()) : 0u;
  return tableLinks > nimbleLinks ? tableLinks : nimbleLinks;
}

void restartAdvertising(int64_t now, ble::Why why) {
  if (gAdv == nullptr || gServer == nullptr) return;
  const unsigned links = linksUp();
  if (links >= ble::MAX_LINKS) {  // 3 links up: never (re)start, and stop a stray advert
    if (gAdv->isAdvertising()) {
      const bool ok = gAdv->stop();
      dbg::status(now, "ble;adv_stop links=%u ok=%u (advertising while at the %u-link max)", links, ok ? 1u : 0u,
                  static_cast<unsigned>(ble::MAX_LINKS));
    }
    return;
  }
  if (gAdv->isAdvertising()) return;
  const bool ok = gAdv->start();
  ++gAdvRestarts;
  if (!ok) ++gAdvFails;
  detail::Guard g;
  gTable.logAdvRestart(now, why, ok);
}

}  // namespace

// ---------------------------------------------------------------------------
namespace detail {
void lock() { portENTER_CRITICAL(&gMux); }
void unlock() { portEXIT_CRITICAL(&gMux); }
ot::ble::LinkTable& table() { return gTable; }
int64_t nowUs() { return esp_timer_get_time(); }

void SubCallbacks::onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& connInfo, uint16_t subValue) {
  const int64_t t = nowUs();
  Guard g;
  gTable.onSubscribe(t, connInfo.getConnHandle(), chr_, subValue);
}
}  // namespace detail

// ---------------------------------------------------------------------------
bool begin(int64_t now) {
  gTable.reset();
  snprintf(gFwRev, sizeof(gFwRev), "%s+%08" PRIx32 "%s", OT_FW_VERSION, hw::BUILD_ID, hw::BUILD_DIRTY ? "-dirty" : "");
  dbg::status(now, "ble;init name=%s nimble=%s ftms=%u adv=%s max_links=%u fw_rev=%s", ble::BLE_NAME,
              NIMBLE_VERSION_STR, static_cast<unsigned>(OT_FTMS_ENABLE), ble::advLayoutName(LAYOUT),
              static_cast<unsigned>(ble::MAX_LINKS), gFwRev);
  if (!NimBLEDevice::init(ble::BLE_NAME)) {
    dbg::status(esp_timer_get_time(), "ERROR ble;NimBLEDevice::init failed: BLE disabled");
    return false;
  }
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  // Default MTU (23): every packet is <= 9 B.
  gServer = NimBLEDevice::createServer();
  if (gServer == nullptr) {
    dbg::status(esp_timer_get_time(), "ERROR ble;createServer failed: BLE disabled");
    return false;
  }
  gServer->setCallbacks(&gServerCb, false);  // static object: never deleted
  gServer->advertiseOnDisconnect(false);     // loop() restarts it (one rule)
  // GATT, created once, in this order: RSC, DIS, then FTMS.
  bool ok = detail::createRscAndDis(gServer, gFwRev);
#if OT_FTMS_ENABLE
  ok = ok && detail::createFtms(gServer);
#endif
  if (!ok) {
    dbg::status(esp_timer_get_time(), "ERROR ble;GATT creation failed: BLE disabled");
    return false;
  }
  gAdv = NimBLEDevice::getAdvertising();
  const int64_t t1 = esp_timer_get_time();
  if (gAdv == nullptr || !applyAdvert(t1)) {
    dbg::status(t1, "ERROR ble;advertising setup failed: BLE disabled");
    return false;
  }
  char p[2 * 31 + 1], s[2 * 31 + 1];
  ble::toHex(gAdvPair.primary.b, gAdvPair.primary.len, p, sizeof(p));
  ble::toHex(gAdvPair.scanResponse.b, gAdvPair.scanResponse.len, s, sizeof(s));
  dbg::status(t1, "ble;adv layout=%s primary=%s (%u B) scan_rsp=%s (%u B)", ble::advLayoutName(LAYOUT), p,
              static_cast<unsigned>(gAdvPair.primary.len), gAdvPair.scanResponse.len ? s : "-",
              static_cast<unsigned>(gAdvPair.scanResponse.len));
  const bool started = gAdv->start();  // also starts the GATT server (handles fixed from here on)
  const int64_t t2 = esp_timer_get_time();
  char addr[20];
  fmtAddr(NimBLEDevice::getAddress().getVal(), addr, sizeof(addr));
  dbg::status(t2, "ble;ready addr=%s advertising=%u", addr, started ? 1u : 0u);
  dbg::status(t2, "heap;point=after_ble");
  dbg::heap(t2);
  gReady = true;
  gNextAdvWatchdog = t2 + ble::ADV_WATCHDOG_US;
  return true;
}

uint32_t service(int64_t now) {
  uint32_t n = drainEvents();
  if (!gReady) return n;
  // Advertising: the callbacks' flag at once, else the 1 s watchdog
  // (covers a failed connect, a host reset, a failed start).
  if (gAdvWanted.exchange(false)) {
    restartAdvertising(now, ble::Why::Flag);
    ++n;
  }
  if (now >= gNextAdvWatchdog) {
    gNextAdvWatchdog += ble::ADV_WATCHDOG_US;
    if (gNextAdvWatchdog <= now) gNextAdvWatchdog = now + ble::ADV_WATCHDOG_US;
    restartAdvertising(now, ble::Why::Watchdog);
  }
#if OT_FTMS_ENABLE
  n += detail::ftmsServiceCp(now);
#endif
  n += drainEvents();
  uint32_t drops;
  {
    detail::Guard g;
    drops = gTable.eventDrops();
  }
  if (drops != gReportedEvDrops) {
    dbg::status(now, "WARNING ble;event queue overflow: %" PRIu32 " B lines lost total", drops);
    gReportedEvDrops = drops;
  }
  return n;
}

void tick(const ble::BleSnapshot& s) {
  if (!gReady) return;
  ++gTicks;
  bool rscSub, tdSub;
  {
    detail::Guard g;
    rscSub = gTable.anySub(ble::Chr::Rsc);
    tdSub = gTable.anySub(ble::Chr::Td);
  }
  char hex[2 * ble::TD_LEN + 1];
  uint8_t r[ble::RSC_MEAS_LEN];
  detail::rscTick(s, rscSub, r);
  ble::toHex(r, sizeof(r), hex, sizeof(hex));
  dbg::line(dbg::LV_K, "R,%" PRId64 ",%s", s.t, hex);
#if OT_FTMS_ENABLE
  uint8_t x[ble::TD_LEN];
  detail::ftmsTick(s, tdSub, x);
  ble::toHex(x, sizeof(x), hex, sizeof(hex));
  dbg::line(dbg::LV_K, "X,%" PRId64 ",%s", s.t, hex);
#else
  (void)tdSub;
#endif
}

void onWorkoutEvent(int64_t now, const ot::WorkoutEvent& e) {
#if OT_FTMS_ENABLE
  detail::ftmsWorkoutEvent(now, e, gReady);  // the status tracker sees every event, ready or not
#else
  (void)now;
  (void)e;
#endif
}

uint8_t subscribedPeers() {
  detail::Guard g;
  return gTable.subscribedPeers();
}

const char* nimbleVersion() { return NIMBLE_VERSION_STR; }
const char* advLayoutName() { return ble::advLayoutName(LAYOUT); }

void dump(int64_t now) {
  // A consistent copy of the slots and the token (no event queue), printed
  // outside the lock.
  ble::LinkSlot slots[ble::LinkTable::SLOTS];
  ble::CpToken tok;
  uint32_t nInd, evDrops;
  uint8_t links, peers;
  {
    detail::Guard g;
    for (size_t i = 0; i < ble::LinkTable::SLOTS; ++i) slots[i] = gTable.slot(i);
    tok = gTable.token();
    nInd = gTable.cpIndications();
    evDrops = gTable.eventDrops();
    links = gTable.links();
    peers = gTable.subscribedPeers();
  }
  const bool adv = gReady && gAdv != nullptr && gAdv->isAdvertising();
  const unsigned nimbleLinks = gReady && gServer != nullptr ? static_cast<unsigned>(gServer->getConnectedCount()) : 0u;
  dbg::status(now,
              "dump;ble ready=%u adv=%u layout=%s links=%u nimble_links=%u subscribed_peers=%u ticks=%" PRIu32
              " adv_restarts=%" PRIu32 " adv_fails=%" PRIu32 " cp_ind=%" PRIu32 " token=%u token_conn=%u ev_drops=%" PRIu32,
              gReady ? 1u : 0u, adv ? 1u : 0u, ble::advLayoutName(LAYOUT), static_cast<unsigned>(links), nimbleLinks,
              static_cast<unsigned>(peers), gTicks, gAdvRestarts, gAdvFails, nInd, tok.inFlight ? 1u : 0u,
              static_cast<unsigned>(tok.conn), evDrops);
  for (size_t i = 0; i < ble::LinkTable::SLOTS; ++i) {
    const ble::LinkSlot& s = slots[i];
    if (!s.inUse) continue;
    char a[20];
    fmtAddr(s.addr, a, sizeof(a));
    dbg::status(now, "dump;ble_link slot=%u conn=%u addr=%s rsc=%u td=%u ts=%u ms=%u cp_ind=%u granted=%u pending=%u",
                static_cast<unsigned>(i), static_cast<unsigned>(s.conn), a, s.rscSub ? 1u : 0u, s.tdSub ? 1u : 0u,
                s.tsSub ? 1u : 0u, s.msSub ? 1u : 0u, s.cpIndicate ? 1u : 0u, s.controlGranted ? 1u : 0u,
                s.pending ? 1u : 0u);
  }
#if OT_FTMS_ENABLE
  dbg::status(now, "dump;ftms training_status=%02x", static_cast<unsigned>(detail::ftmsTrainingCode()));
#endif
}

}  // namespace bleh
