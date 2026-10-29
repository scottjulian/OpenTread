// ble_payload.cpp - see ble_payload.h.
#include "ble_payload.h"

#include <cstring>

namespace ot {
namespace ble {

namespace {

void putU16(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}
void putU24(uint8_t* p, uint32_t v) {
  putU16(p, v);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
}
void putU32(uint8_t* p, uint32_t v) {
  putU24(p, v);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}
uint32_t clampU(uint64_t v, uint32_t max) { return v > max ? max : static_cast<uint32_t>(v); }

}  // namespace

// ---------------------------------------------------------------------------
// Encoders
// ---------------------------------------------------------------------------
void encodeRscMeasurement(int32_t mmps, uint32_t odometerPulses, uint8_t out[RSC_MEAS_LEN]) {
  uint8_t flags = RSC_FLAG_TOTAL_DISTANCE;
  if (mmps >= WALK_RUN_MMPS) flags = static_cast<uint8_t>(flags | RSC_FLAG_RUNNING);
  out[0] = flags;
  putU16(out + 1, clampU(rscSpeed256(mmps), 0xFFFFu));
  out[3] = 0;  // cadence: none (no fabricated data)
  putU32(out + 4, clampU(rscDistanceDm(odometerPulses), 0xFFFFFFFFu));
}

void encodeTreadmillData(int32_t mmps, uint32_t wkPulses, int64_t totalUs, uint8_t out[TD_LEN]) {
  putU16(out, TD_FLAGS);
  putU16(out + 2, clampU(ftmsSpeedCentiKmh(mmps), 0xFFFFu));
  putU24(out + 4, clampU(ftmsDistanceM(wkPulses), 0xFFFFFFu));
  const uint64_t s = totalUs <= 0 ? 0u : static_cast<uint64_t>(totalUs) / 1000000u;
  putU16(out + 7, clampU(s, 0xFFFFu));
}

uint8_t trainingStatusCode(WorkoutState s) {
  switch (s) {
    case WorkoutState::Running:
    case WorkoutState::Paused: return TS_MANUAL_MODE;
    case WorkoutState::Summary: return TS_POST_WORKOUT;
    case WorkoutState::Idle:
    case WorkoutState::Warmup:
    default: return TS_IDLE;
  }
}

void encodeTrainingStatus(WorkoutState s, uint8_t out[2]) {
  out[0] = 0x00;  // flags: no Training Status String, no Extended String
  out[1] = trainingStatusCode(s);
}

size_t machineStatusFor(WorkoutEventType e, uint8_t out[2]) {
  switch (e) {
    case WorkoutEventType::Confirmed:
      out[0] = MS_STARTED_OR_RESUMED_BY_USER;
      return 1;
    case WorkoutEventType::Stopped:
    case WorkoutEventType::AutoEnded:
      out[0] = MS_STOPPED_OR_PAUSED_BY_USER;
      out[1] = 0x01;  // control information: stop
      return 2;
    default: return 0;
  }
}

FtmsStatus::Out FtmsStatus::onEvent(const WorkoutEvent& e) {
  Out o;
  o.msLen = machineStatusFor(e.type, o.ms);
  const uint8_t c = trainingStatusCode(e.to);
  if (c != code_) {
    code_ = c;
    o.tsChanged = true;
  }
  o.ts[0] = 0x00;
  o.ts[1] = code_;
  return o;
}

// ---------------------------------------------------------------------------
// Control Point
// ---------------------------------------------------------------------------
CpResult cpRespond(const uint8_t* req, size_t len, bool granted) {
  CpResult r;
  if (req == nullptr || len == 0) return r;  // empty write: ignored
  const uint8_t op = req[0];
  uint8_t result = CP_RESULT_NOT_SUPPORTED;
  switch (op) {
    case CP_OP_REQUEST_CONTROL:
      result = CP_RESULT_SUCCESS;
      r.grant = true;
      break;
    case CP_OP_RESET:
    case CP_OP_START_RESUME:
      result = granted ? CP_RESULT_SUCCESS : CP_RESULT_NOT_PERMITTED;
      break;
    case CP_OP_STOP_PAUSE:
      if (len < 2 || (req[1] != 0x01 && req[1] != 0x02)) result = CP_RESULT_INVALID_PARAM;
      else result = granted ? CP_RESULT_SUCCESS : CP_RESULT_NOT_PERMITTED;
      break;
    default: result = CP_RESULT_NOT_SUPPORTED; break;
  }
  r.respond = true;
  r.rsp[0] = CP_OP_RESPONSE;
  r.rsp[1] = op;
  r.rsp[2] = result;
  return r;
}

// ---------------------------------------------------------------------------
// Advertising
// ---------------------------------------------------------------------------
const char* advLayoutName(AdvLayout l) {
  switch (l) {
    case AdvLayout::B: return "B";
    case AdvLayout::A: return "A";
    case AdvLayout::BPrime: return "Bp";
    case AdvLayout::BDouble: return "Bpp";
  }
  return "?";
}

bool AdvPacket::add(uint8_t type, const uint8_t* data, size_t n) {
  if (n > MAX || static_cast<size_t>(len) + 2u + n > MAX) return false;
  b[len] = static_cast<uint8_t>(n + 1u);
  b[len + 1u] = type;
  if (n > 0) std::memcpy(b + len + 2u, data, n);
  len = static_cast<uint8_t>(len + 2u + n);
  return true;
}

bool buildAdvert(AdvLayout layout, const char* name, AdvPair& out) {
  out = AdvPair{};
  if (name == nullptr) return false;
  const size_t nameLen = std::strlen(name);
  if (nameLen == 0 || nameLen > AdvPacket::MAX - 2u) return false;
  const uint8_t* nm = reinterpret_cast<const uint8_t*>(name);
  const uint8_t flags[1] = {0x06};  // LE General Discoverable + BR/EDR not supported
  const uint8_t rsc[2] = {0x14, 0x18};
  const uint8_t ftms[2] = {0x26, 0x18};
  const uint8_t both[4] = {0x14, 0x18, 0x26, 0x18};
  const uint8_t appearance[2] = {static_cast<uint8_t>(APPEARANCE & 0xFFu), static_cast<uint8_t>(APPEARANCE >> 8)};
  constexpr uint8_t AD_FLAGS = 0x01, AD_UUID16_INCOMPLETE = 0x02, AD_UUID16_COMPLETE = 0x03, AD_NAME_COMPLETE = 0x09,
                    AD_SERVICE_DATA16 = 0x16, AD_APPEARANCE = 0x19;
  AdvPair p;
  bool ok = p.primary.add(AD_FLAGS, flags, 1) && p.primary.add(AD_NAME_COMPLETE, nm, nameLen);
  switch (layout) {
    case AdvLayout::B:
      ok = ok && p.primary.add(AD_UUID16_COMPLETE, rsc, 2) && p.primary.add(AD_APPEARANCE, appearance, 2);
      break;
    case AdvLayout::BPrime:
      ok = ok && p.primary.add(AD_UUID16_COMPLETE, rsc, 2) && p.primary.add(AD_APPEARANCE, appearance, 2) &&
           p.scanResponse.add(AD_SERVICE_DATA16, FTMS_SERVICE_DATA, sizeof(FTMS_SERVICE_DATA));
      break;
    case AdvLayout::BDouble:
      ok = ok && p.primary.add(AD_UUID16_INCOMPLETE, rsc, 2) && p.primary.add(AD_APPEARANCE, appearance, 2) &&
           p.scanResponse.add(AD_UUID16_INCOMPLETE, ftms, 2) &&
           p.scanResponse.add(AD_SERVICE_DATA16, FTMS_SERVICE_DATA, sizeof(FTMS_SERVICE_DATA));
      break;
    case AdvLayout::A:
      ok = ok && p.primary.add(AD_UUID16_COMPLETE, both, 4) &&
           p.primary.add(AD_SERVICE_DATA16, FTMS_SERVICE_DATA, sizeof(FTMS_SERVICE_DATA)) &&
           p.scanResponse.add(AD_APPEARANCE, appearance, 2);
      break;
    default: ok = false; break;
  }
  if (!ok) return false;
  out = p;
  return true;
}

// ---------------------------------------------------------------------------
// LinkTable
// ---------------------------------------------------------------------------
const char* chrName(Chr c) {
  switch (c) {
    case Chr::Rsc: return "rsc";
    case Chr::Td: return "td";
    case Chr::Ts: return "ts";
    case Chr::Ms: return "ms";
    case Chr::Cp: return "cp";
    case Chr::Other: return "other";
  }
  return "?";
}

const char* bleEvName(BleEv e) {
  switch (e) {
    case BleEv::Connect: return "connect";
    case BleEv::Disconnect: return "disconnect";
    case BleEv::Sub: return "sub";
    case BleEv::Unsub: return "unsub";
    case BleEv::AdvRestart: return "adv_restart";
    case BleEv::CpWrite: return "cp_write";
    case BleEv::CpInd: return "cp_ind";
    case BleEv::CpAck: return "cp_ack";
    case BleEv::CpDrop: return "cp_drop";
    case BleEv::CpTimeout: return "cp_timeout";
    case BleEv::Ts: return "ts";
    case BleEv::Ms: return "ms";
  }
  return "?";
}

const char* whyName(Why w) {
  switch (w) {
    case Why::None: return "-";
    case Why::NoCccd: return "no_cccd";
    case Why::Replaced: return "replaced";
    case Why::Disconnect: return "disconnect";
    case Why::SendFail: return "send_fail";
    case Why::Status: return "status";
    case Why::Stray: return "stray";
    case Why::Empty: return "empty";
    case Why::NoSlot: return "no_slot";
    case Why::TooLong: return "too_long";
    case Why::Flag: return "flag";
    case Why::Watchdog: return "watchdog";
    case Why::Fallback: return "fallback";
  }
  return "?";
}

void LinkTable::reset() {
  for (LinkSlot& s : slots_) s = LinkSlot{};
  token_ = CpToken{};
  nInd_ = 0;
  qHead_ = qTail_ = 0;
  evDrops_ = 0;
}

LinkSlot* LinkTable::find(uint16_t conn) {
  for (LinkSlot& s : slots_)
    if (s.inUse && s.conn == conn) return &s;
  return nullptr;
}
const LinkSlot* LinkTable::find(uint16_t conn) const {
  for (const LinkSlot& s : slots_)
    if (s.inUse && s.conn == conn) return &s;
  return nullptr;
}

void LinkTable::push(const BleEvent& e) {
  if (qTail_ - qHead_ >= EVENT_QUEUE) {
    ++evDrops_;
    return;
  }
  q_[qTail_ % EVENT_QUEUE] = e;
  ++qTail_;
}

void LinkTable::pushSimple(int64_t t, BleEv ev, uint16_t conn, Why why, int32_t code) {
  BleEvent e;
  e.t = t;
  e.ev = ev;
  e.conn = conn;
  e.why = why;
  e.code = code;
  push(e);
}

bool LinkTable::popEvent(BleEvent& e) {
  if (qHead_ == qTail_) return false;
  e = q_[qHead_ % EVENT_QUEUE];
  ++qHead_;
  return true;
}

void LinkTable::onConnect(int64_t t, uint16_t conn, const uint8_t addr[6]) {
  LinkSlot* s = find(conn);  // a stale slot with this handle (missed disconnect): start it afresh
  if (s == nullptr) {
    for (LinkSlot& x : slots_)
      if (!x.inUse) {
        s = &x;
        break;
      }
  }
  BleEvent e;
  e.t = t;
  e.ev = BleEv::Connect;
  e.conn = conn;
  e.len = 6;
  if (addr != nullptr) std::memcpy(e.data, addr, 6);
  if (s == nullptr) {
    e.why = Why::NoSlot;
    e.code = -1;
    push(e);
    return;
  }
  *s = LinkSlot{};
  s->inUse = true;
  s->conn = conn;
  if (addr != nullptr) std::memcpy(s->addr, addr, 6);
  e.code = links();
  push(e);
}

void LinkTable::onDisconnect(int64_t t, uint16_t conn, int32_t reason) {
  LinkSlot* s = find(conn);
  bool pendingDrop = false;
  if (s != nullptr) {
    pendingDrop = s->pending;
    *s = LinkSlot{};  // everything goes: subscriptions, grant, pending request
  }
  bool tokenFreed = false;
  if (token_.inFlight && token_.conn == conn) {
    token_.inFlight = false;
    tokenFreed = true;
  }
  BleEvent e;
  e.t = t;
  e.ev = BleEv::Disconnect;
  e.conn = conn;
  e.code = reason;
  e.len = 2;
  e.data[0] = pendingDrop ? 1 : 0;
  e.data[1] = tokenFreed ? 1 : 0;
  push(e);
}

void LinkTable::onSubscribe(int64_t t, uint16_t conn, Chr c, uint16_t subValue) {
  LinkSlot* s = find(conn);
  BleEvent e;
  e.t = t;
  e.ev = subValue != 0 ? BleEv::Sub : BleEv::Unsub;
  e.conn = conn;
  e.chr = c;
  e.code = subValue;
  if (s == nullptr) {
    e.why = Why::NoSlot;  // not a known link (connect missed or already gone): ignored
    push(e);
    return;
  }
  const bool notify = (subValue & 0x1u) != 0;
  const bool indicate = (subValue & 0x2u) != 0;
  switch (c) {
    case Chr::Rsc: s->rscSub = notify; break;
    case Chr::Td: s->tdSub = notify; break;
    case Chr::Ts: s->tsSub = notify; break;
    case Chr::Ms: s->msSub = notify; break;
    case Chr::Cp: s->cpIndicate = indicate; break;
    case Chr::Other: break;
  }
  push(e);
}

void LinkTable::onCpWrite(int64_t t, uint16_t conn, const uint8_t* d, size_t n) {
  BleEvent e;
  e.t = t;
  e.ev = BleEv::CpWrite;
  e.conn = conn;
  const size_t k = n < CP_MAX_LEN ? n : CP_MAX_LEN;
  e.len = static_cast<uint8_t>(k);
  if (k > 0 && d != nullptr) std::memcpy(e.data, d, k);
  e.code = static_cast<int32_t>(n);
  push(e);
  if (n == 0 || d == nullptr) {
    pushSimple(t, BleEv::CpDrop, conn, Why::Empty, 0);
    return;
  }
  if (n > CP_MAX_LEN) {
    pushSimple(t, BleEv::CpDrop, conn, Why::TooLong, static_cast<int32_t>(n));
    return;
  }
  LinkSlot* s = find(conn);
  if (s == nullptr) {
    pushSimple(t, BleEv::CpDrop, conn, Why::NoSlot, 0);
    return;
  }
  if (s->pending) pushSimple(t, BleEv::CpDrop, conn, Why::Replaced, s->pendingCp[0]);
  s->pending = true;
  s->pendingLen = static_cast<uint8_t>(n);
  std::memcpy(s->pendingCp, d, n);
  s->pendingAtUs = t;
}

void LinkTable::onCpStatus(int64_t t, int32_t code) {
  if (!token_.inFlight) {
    pushSimple(t, BleEv::CpDrop, CONN_NONE, Why::Stray, code);
    return;
  }
  const uint16_t conn = token_.conn;
  token_.inFlight = false;
  if (code == HS_EDONE) pushSimple(t, BleEv::CpAck, conn, Why::None, code);
  else if (code == HS_ETIMEOUT) pushSimple(t, BleEv::CpTimeout, conn, Why::Status, code);
  else pushSimple(t, BleEv::CpDrop, conn, Why::Status, code);
}

bool LinkTable::pollCp(int64_t now, CpSend& out) {
  if (token_.inFlight && now - token_.sentAtUs >= CP_IND_TIMEOUT_US) {
    token_.inFlight = false;
    pushSimple(now, BleEv::CpTimeout, token_.conn, Why::Fallback, 0);
  }
  // Ready requests on links without CP indications: dropped, no effect.
  for (LinkSlot& s : slots_) {
    if (s.inUse && s.pending && now - s.pendingAtUs >= CP_IND_DEFER_US && !s.cpIndicate) {
      s.pending = false;
      pushSimple(now, BleEv::CpDrop, s.conn, Why::NoCccd, s.pendingCp[0]);
    }
  }
  if (token_.inFlight) return false;
  LinkSlot* best = nullptr;
  for (LinkSlot& s : slots_) {
    if (!s.inUse || !s.pending || !s.cpIndicate || now - s.pendingAtUs < CP_IND_DEFER_US) continue;
    if (best == nullptr || s.pendingAtUs < best->pendingAtUs) best = &s;
  }
  if (best == nullptr) return false;
  const CpResult r = cpRespond(best->pendingCp, best->pendingLen, best->controlGranted);
  best->pending = false;
  if (!r.respond) return false;  // (unreachable: empty writes never become pending)
  if (r.grant) best->controlGranted = true;
  token_.inFlight = true;
  token_.conn = best->conn;
  token_.sentAtUs = now;
  ++token_.seq;
  ++nInd_;
  out.conn = best->conn;
  std::memcpy(out.rsp, r.rsp, CP_RSP_LEN);
  out.seq = token_.seq;
  BleEvent e;
  e.t = now;
  e.ev = BleEv::CpInd;
  e.conn = best->conn;
  e.len = CP_RSP_LEN;
  std::memcpy(e.data, r.rsp, CP_RSP_LEN);
  push(e);
  return true;
}

void LinkTable::cpSendFailed(int64_t now, uint32_t seq, int32_t rc) {
  if (!token_.inFlight || token_.seq != seq) return;  // already released (e.g. a synchronous onStatus)
  token_.inFlight = false;
  pushSimple(now, BleEv::CpDrop, token_.conn, Why::SendFail, rc);
}

void LinkTable::logAdvRestart(int64_t t, Why why, bool ok) { pushSimple(t, BleEv::AdvRestart, CONN_NONE, why, ok ? 1 : 0); }

void LinkTable::logStatus(int64_t t, BleEv ev, const uint8_t* d, size_t n) {
  BleEvent e;
  e.t = t;
  e.ev = ev;
  const size_t k = n < CP_MAX_LEN ? n : CP_MAX_LEN;
  e.len = static_cast<uint8_t>(k);
  if (k > 0 && d != nullptr) std::memcpy(e.data, d, k);
  push(e);
}

uint8_t LinkTable::links() const {
  uint8_t n = 0;
  for (const LinkSlot& s : slots_)
    if (s.inUse) ++n;
  return n;
}

uint8_t LinkTable::subscribedPeers() const {
  uint8_t n = 0;
  for (const LinkSlot& s : slots_)
    if (s.inUse && (s.rscSub || s.tdSub)) ++n;
  return n;
}

bool LinkTable::anySub(Chr c) const {
  for (const LinkSlot& s : slots_) {
    if (!s.inUse) continue;
    switch (c) {
      case Chr::Rsc: if (s.rscSub) return true; break;
      case Chr::Td: if (s.tdSub) return true; break;
      case Chr::Ts: if (s.tsSub) return true; break;
      case Chr::Ms: if (s.msSub) return true; break;
      case Chr::Cp: if (s.cpIndicate) return true; break;
      case Chr::Other: break;
    }
  }
  return false;
}

void toHex(const uint8_t* d, size_t n, char* out, size_t outSize) {
  static const char kHex[] = "0123456789abcdef";
  if (outSize == 0) return;
  size_t j = 0;
  for (size_t i = 0; i < n && j + 2 < outSize; ++i) {
    out[j++] = kHex[d[i] >> 4];
    out[j++] = kHex[d[i] & 0x0Fu];
  }
  out[j] = '\0';
}

}  // namespace ble
}  // namespace ot
