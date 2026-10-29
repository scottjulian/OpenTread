// ble_payload.h - BLE wire formats and BLE link logic, pure C++17.
//
// Everything the NimBLE adapters (hw_ble*.cpp) send or decide lives here, so
// the host tests (test/test_ble_payload.cpp, tests 6.x / 7.x) cover it:
//   - RSC Measurement 0x2A53 encoder (8 B), RSC Feature / Sensor Location;
//   - FTMS Treadmill Data 0x2ACD encoder (flags 0x0404, 9 B), Feature 0x2ACC;
//   - FTMS Training Status / Machine Status from Workout states and events;
//   - the FTMS Control Point responder (one request -> one response);
//   - advertising payload builder (Layouts B, B', B'', A) with the 31 B limit;
//   - LinkTable: the per-connection table (subscriptions, control grant, one
//     pending CP request per link), the single global CP-indication token, and
//     the queue of B-line events. The adapter wraps every LinkTable call in
//     its portMUX; this class itself has no locking and no clock.
//
// No allocation, no floating point, no clock reads, fixed-width integers only.
#pragma once

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "fixed.h"
#include "workout.h"

namespace ot {
namespace ble {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
inline constexpr char     BLE_NAME[] = "OpenTread";             // project name; <= 13 chars for Layout A
inline constexpr size_t   BLE_NAME_LEN = sizeof(BLE_NAME) - 1;   // 9
inline constexpr int64_t  NOTIFY_PERIOD_US  = 1000000;           // RSC and FTMS both 1 Hz
inline constexpr int32_t  WALK_RUN_MMPS     = 1944;              // 7.0 km/h: RSC "running" flag
inline constexpr int64_t  ADV_WATCHDOG_US   = 1000000;           // advertising watchdog period
inline constexpr int64_t  CP_IND_DEFER_US   = 5000;              // CP response >= 5 ms after the write
inline constexpr int64_t  CP_IND_TIMEOUT_US = 35000000;          // token fallback (ATT timeout is 30 s)
inline constexpr uint16_t APPEARANCE        = 0x0440;            // generic running-walking sensor
inline constexpr size_t   MAX_LINKS         = 3;                 // NimBLE-Arduino 2.5.0 default (nimconfig.h untouched)

// SIG UUIDs (16-bit)
inline constexpr uint16_t UUID_RSC_SERVICE     = 0x1814;
inline constexpr uint16_t UUID_RSC_MEASUREMENT = 0x2A53;
inline constexpr uint16_t UUID_RSC_FEATURE     = 0x2A54;
inline constexpr uint16_t UUID_SENSOR_LOCATION = 0x2A5D;
inline constexpr uint16_t UUID_DIS_SERVICE     = 0x180A;
inline constexpr uint16_t UUID_DIS_MODEL       = 0x2A24;
inline constexpr uint16_t UUID_DIS_FIRMWARE    = 0x2A26;
inline constexpr uint16_t UUID_DIS_HARDWARE    = 0x2A27;
inline constexpr uint16_t UUID_DIS_MANUFACTURER = 0x2A29;
inline constexpr uint16_t UUID_FTMS_SERVICE    = 0x1826;
inline constexpr uint16_t UUID_FTMS_FEATURE    = 0x2ACC;
inline constexpr uint16_t UUID_TREADMILL_DATA  = 0x2ACD;
inline constexpr uint16_t UUID_TRAINING_STATUS = 0x2AD3;
inline constexpr uint16_t UUID_FTMS_CONTROL_POINT = 0x2AD9;
inline constexpr uint16_t UUID_MACHINE_STATUS  = 0x2ADA;

// Device Information strings (never the stock "AssaultRunner" identity).
inline constexpr char DIS_MANUFACTURER[] = "OpenTread";
inline constexpr char DIS_MODEL[]        = "OpenTread head unit";
inline constexpr char DIS_HARDWARE[]     = "ESP32-2432S028R";

// Characteristic values / sizes
inline constexpr size_t  RSC_MEAS_LEN = 8;
inline constexpr uint8_t RSC_FEATURE[2] = {0x06, 0x00};  // Total Distance + Walking/Running status
inline constexpr uint8_t SENSOR_LOCATION = 0x00;         // Other
inline constexpr uint8_t RSC_FLAG_TOTAL_DISTANCE = 0x02;
inline constexpr uint8_t RSC_FLAG_RUNNING = 0x04;

inline constexpr size_t   TD_LEN = 9;
inline constexpr uint16_t TD_FLAGS = 0x0404;  // Inst. Speed (bit 0 = 0) + Total Distance (bit 2) + Elapsed (bit 10)
inline constexpr uint8_t  FTMS_FEATURE[8] = {0x04, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // 0x00001004; targets 0

inline constexpr size_t CP_MAX_LEN = 20;  // Control Point write max length
inline constexpr size_t CP_RSP_LEN = 3;   // 80 <op> <result>

// FTMS Control Point op codes / result codes (FTMS v1.0 §4.16)
inline constexpr uint8_t CP_OP_REQUEST_CONTROL = 0x00;
inline constexpr uint8_t CP_OP_RESET           = 0x01;
inline constexpr uint8_t CP_OP_START_RESUME    = 0x07;
inline constexpr uint8_t CP_OP_STOP_PAUSE      = 0x08;
inline constexpr uint8_t CP_OP_RESPONSE        = 0x80;
inline constexpr uint8_t CP_RESULT_SUCCESS       = 0x01;
inline constexpr uint8_t CP_RESULT_NOT_SUPPORTED = 0x02;
inline constexpr uint8_t CP_RESULT_INVALID_PARAM = 0x03;
inline constexpr uint8_t CP_RESULT_NOT_PERMITTED = 0x05;

// FTMS Training Status (0x2AD3) codes and Machine Status (0x2ADA) op codes
inline constexpr uint8_t TS_IDLE         = 0x01;
inline constexpr uint8_t TS_MANUAL_MODE  = 0x0D;  // Manual Mode (Quick Start)
inline constexpr uint8_t TS_POST_WORKOUT = 0x0F;
inline constexpr uint8_t MS_STOPPED_OR_PAUSED_BY_USER = 0x02;  // param 0x01 = stop
inline constexpr uint8_t MS_STARTED_OR_RESUMED_BY_USER = 0x04;

// NimBLE host return codes the CP token logic distinguishes (ble_hs.h).
inline constexpr int HS_ENOTCONN = 7;
inline constexpr int HS_ETIMEOUT = 13;
inline constexpr int HS_EDONE    = 14;

// Connection handle "none" (BLE_HS_CONN_HANDLE_NONE).
inline constexpr uint16_t CONN_NONE = 0xFFFF;

// ---------------------------------------------------------------------------
// Encoders
// ---------------------------------------------------------------------------

// RSC Measurement (8 B): flags | speed u16 LE (1/256 m/s) | cadence 0 | total
// distance u32 LE (dm).
//   flags = 0x02 (Total Distance present) | 0x04 if mmps >= WALK_RUN_MMPS
//           (the running flag is decided on the UNROUNDED mm/s);
//   speed = (mmps*256 + 500)/1000 clamped to 0xFFFF (negative -> 0);
//   distance = floor(odometerPulses * 597400 / 10^5) dm clamped to 0xFFFFFFFF
//           (cumulative since boot: only Metrics' odometer, never per workout).
void encodeRscMeasurement(int32_t mmps, uint32_t odometerPulses, uint8_t out[RSC_MEAS_LEN]);

// FTMS Treadmill Data (9 B): flags 04 04 | speed u16 LE (0.01 km/h) |
// total distance u24 LE (m) | elapsed u16 LE (s).
//   speed = (mmps*36 + 50)/100 clamped to 0xFFFF (negative -> 0);
//   distance = floor(wkPulses * 597400 / 10^6) clamped to 16,777,215;
//   elapsed = floor(totalUs / 10^6) clamped to 0..65535.
void encodeTreadmillData(int32_t mmps, uint32_t wkPulses, int64_t totalUs, uint8_t out[TD_LEN]);

// Training Status (2 B, [0x00, code]): IDLE/WARMUP -> 0x01 Idle;
// RUNNING/PAUSED -> 0x0D Manual Mode; SUMMARY -> 0x0F Post-Workout.
uint8_t trainingStatusCode(WorkoutState s);
void    encodeTrainingStatus(WorkoutState s, uint8_t out[2]);

// Machine Status for one Workout event: Confirmed -> {04} (1 B, Started or
// Resumed by User); Stopped / AutoEnded -> {02 01} (2 B, Stopped by User,
// control info Stop). Every other event -> 0 (no notification). Returns the
// length written to out.
size_t machineStatusFor(WorkoutEventType e, uint8_t out[2]);

// FTMS status tracker: feed every Workout event in order; it reports
// at most one Machine Status and at most one Training Status per event. The
// Training Status is reported only when its CODE (the state class) changes,
// so RUNNING <-> PAUSED flapping never notifies.
class FtmsStatus {
 public:
  struct Out {
    size_t  msLen = 0;       // 0: no Machine Status notification
    uint8_t ms[2] = {0, 0};
    bool    tsChanged = false;
    uint8_t ts[2] = {0x00, TS_IDLE};
  };
  FtmsStatus() = default;
  void    reset() { code_ = TS_IDLE; }
  Out     onEvent(const WorkoutEvent& e);
  uint8_t trainingCode() const { return code_; }

 private:
  uint8_t code_ = TS_IDLE;  // boot state: IDLE (characteristic initial value 00 01)
};

// ---------------------------------------------------------------------------
// Control Point responder (lenient like qdomyos-zwift, spec-correct permission).
// Order of checks: op supported -> parameters -> permission.
//   empty write                      -> no response
//   00 Request Control (+ anything)  -> 80 00 01, grant
//   01 Reset / 07 Start-Resume (+ anything, trailing bytes ignored)
//                                    -> 80 op 01 if granted, else 80 op 05; no effect
//   08 Stop/Pause: param 01|02       -> 80 08 01 if granted, else 80 08 05; no effect
//              missing / other param -> 80 08 03
//   anything else (02-06, 09-14 targets, 15-FF incl. 80)
//                                    -> 80 op 02
// ---------------------------------------------------------------------------
struct CpResult {
  bool    respond = false;
  bool    grant = false;  // the request grants control to this link
  uint8_t rsp[CP_RSP_LEN] = {0, 0, 0};
};
CpResult cpRespond(const uint8_t* req, size_t len, bool granted);

// ---------------------------------------------------------------------------
// Advertising payloads (≤ 31 B each; the builder refuses anything longer)
// ---------------------------------------------------------------------------
enum class AdvLayout : uint8_t {
  B = 0,         // RSC only (FR955-proven)                               primary 22 B (9-char name), no scan response
  A = 1,         // dual-UUID primary + FTMS Service Data                  primary 27 B (9-char name), scan rsp appearance
  BPrime = 2,    // fallback: B primary + FTMS Service Data in the scan response (7 B)
  BDouble = 3,   // fallback 2: both packets use Incomplete UUID lists (0x02)
};
const char* advLayoutName(AdvLayout l);  // "B", "A", "Bp", "Bpp"

struct AdvPacket {
  static constexpr size_t MAX = 31;
  uint8_t len = 0;
  uint8_t b[MAX] = {};
  // Appends one AD structure (len byte, type, data). Returns false and leaves
  // the packet unchanged when it would exceed 31 B.
  bool add(uint8_t type, const uint8_t* data, size_t n);
};
struct AdvPair {
  AdvPacket primary;
  AdvPacket scanResponse;  // len 0: no scan response
};
// FTMS Service Data AD body: 26 18 | flags 0x01 (Available) | type u16 0x0001 (Treadmill).
inline constexpr uint8_t FTMS_SERVICE_DATA[5] = {0x26, 0x18, 0x01, 0x01, 0x00};

// Build the layout for a device name (BLE_NAME in the firmware; tests also
// pass longer names to prove the 31 B refusal). Returns false (and an empty
// pair) if any packet would exceed 31 B or the name is empty / > 29 chars.
bool buildAdvert(AdvLayout layout, const char* name, AdvPair& out);
// True if a layout advertises the FTMS service (UUID or Service Data).
constexpr bool advMentionsFtms(AdvLayout l) { return l != AdvLayout::B; }

// ---------------------------------------------------------------------------
// LinkTable: per-connection state + the global CP-indication token + the
// B-line event queue. Host-task callbacks call the on*() methods; loop()
// calls pollCp()/cpSendFailed()/popEvent()/log*(). The adapter serialises
// every call (portMUX); there is no internal locking.
// ---------------------------------------------------------------------------
enum class Chr : uint8_t { Rsc, Td, Ts, Ms, Cp, Other };
const char* chrName(Chr c);  // "rsc" "td" "ts" "ms" "cp" "other"

enum class BleEv : uint8_t {
  Connect,     // B,t,connect,<conn>,<addr>;links=<n>     (code = -1 if no slot was free)
  Disconnect,  // B,t,disconnect,<conn>,reason=<code>;pending_drop=<0|1>;token=<0|1>
  Sub,         // B,t,sub,<conn>,<chr>=<subValue>
  Unsub,       // B,t,unsub,<conn>,<chr>
  AdvRestart,  // B,t,adv_restart,-,<why>;ok=<0|1>         (loop side)
  CpWrite,     // B,t,cp_write,<conn>,<hex>
  CpInd,       // B,t,cp_ind,<conn>,<hex>                  (token taken)
  CpAck,       // B,t,cp_ack,<conn>,code=14
  CpDrop,      // B,t,cp_drop,<conn>,<why>[;code=<n>]
  CpTimeout,   // B,t,cp_timeout,<conn>,code=13 | fallback
  Ts,          // B,t,ts,-,<hex>                           (Training Status notified, loop side)
  Ms,          // B,t,ms,-,<hex>                           (Machine Status notified, loop side)
};
const char* bleEvName(BleEv e);

// Reasons carried by CpDrop / AdvRestart events (BleEvent::why).
enum class Why : uint8_t {
  None, NoCccd, Replaced, Disconnect, SendFail, Status, Stray, Empty, NoSlot, TooLong,
  Flag, Watchdog, Fallback,
};
const char* whyName(Why w);

struct BleEvent {
  int64_t  t = 0;
  BleEv    ev = BleEv::Connect;
  uint16_t conn = CONN_NONE;
  Chr      chr = Chr::Other;
  Why      why = Why::None;
  int32_t  code = 0;      // subValue, disconnect reason, status code, ok flag, link count
  uint8_t  len = 0;       // data bytes (cp request/response hex, address)
  uint8_t  data[CP_MAX_LEN] = {};
};

struct LinkSlot {
  bool     inUse = false;
  uint16_t conn = CONN_NONE;
  uint8_t  addr[6] = {};       // as NimBLE gives it (little endian)
  bool     rscSub = false;     // RSC Measurement notify
  bool     tdSub = false;      // Treadmill Data notify
  bool     tsSub = false;      // Training Status notify
  bool     msSub = false;      // Machine Status notify
  bool     cpIndicate = false; // Control Point indications enabled (CCCD bit 1)
  bool     controlGranted = false;
  bool     pending = false;    // one pending CP request
  uint8_t  pendingLen = 0;
  uint8_t  pendingCp[CP_MAX_LEN] = {};
  int64_t  pendingAtUs = 0;
};

struct CpToken {
  bool     inFlight = false;
  uint16_t conn = CONN_NONE;
  int64_t  sentAtUs = 0;
  uint32_t seq = 0;  // increments per indication; cpSendFailed() matches it
};

struct CpSend {
  uint16_t conn = CONN_NONE;
  uint8_t  rsp[CP_RSP_LEN] = {0, 0, 0};
  uint32_t seq = 0;
};

class LinkTable {
 public:
  static constexpr size_t SLOTS = MAX_LINKS;
  static constexpr size_t EVENT_QUEUE = 32;

  LinkTable() = default;
  void reset();

  // ---- host-task side (NimBLE callbacks) ----
  void onConnect(int64_t t, uint16_t conn, const uint8_t addr[6]);
  void onDisconnect(int64_t t, uint16_t conn, int32_t reason);
  // subValue as NimBLE reports it: bit 0 notify, bit 1 indicate.
  void onSubscribe(int64_t t, uint16_t conn, Chr c, uint16_t subValue);
  // A Control Point write (len may exceed CP_MAX_LEN only if the stack let it
  // through: then it is dropped). Empty writes are ignored (logged).
  void onCpWrite(int64_t t, uint16_t conn, const uint8_t* d, size_t n);
  // CP indication completion (onStatus; connInfo NOT used: zeroed in 2.5.0).
  // 14 -> ack, 13 -> timeout, anything else -> drop. Clears the token.
  void onCpStatus(int64_t t, int32_t code);

  // ---- loop side ----
  // One step of the CP scheduler at `now`:
  //   - token in flight for >= CP_IND_TIMEOUT_US -> released (cp_timeout fallback);
  //   - every pending request whose deferral elapsed on a link WITHOUT CP
  //     indications enabled is dropped with no effect (cp_drop no_cccd);
  //   - if no token is in flight: the oldest pending request with its
  //     deferral elapsed (now - pendingAt >= CP_IND_DEFER_US) on a link with
  //     indications enabled is answered: the response is computed (grant
  //     applied), the token is taken {conn, now, seq}, cp_ind is queued and
  //     the send is returned in `out` (return true). The caller indicates
  //     outside the lock, then calls cpSendFailed(seq) if that failed.
  bool pollCp(int64_t now, CpSend& out);
  void cpSendFailed(int64_t now, uint32_t seq, int32_t rc);
  // Loop-side log events.
  void logAdvRestart(int64_t t, Why why, bool ok);
  void logStatus(int64_t t, BleEv ev, const uint8_t* d, size_t n);  // Ts / Ms

  bool     popEvent(BleEvent& e);
  uint32_t eventDrops() const { return evDrops_; }

  // ---- queries ----
  uint8_t  links() const;
  // Peers with at least one data subscription (RSC or Treadmill Data): the
  // watch icon (>= 1) and H subscribed_peers.
  uint8_t  subscribedPeers() const;
  bool     anySub(Chr c) const;
  const CpToken& token() const { return token_; }
  const LinkSlot& slot(size_t i) const { return slots_[i]; }
  uint32_t cpIndications() const { return nInd_; }

 private:
  LinkSlot* find(uint16_t conn);
  const LinkSlot* find(uint16_t conn) const;
  void push(const BleEvent& e);
  void pushSimple(int64_t t, BleEv ev, uint16_t conn, Why why, int32_t code);

  LinkSlot slots_[SLOTS];
  CpToken  token_;
  uint32_t nInd_ = 0;
  BleEvent q_[EVENT_QUEUE];
  uint32_t qHead_ = 0, qTail_ = 0;  // pop at head, push at tail
  uint32_t evDrops_ = 0;
};

// The 1 Hz snapshot both services are encoded from (one per tick; the K line
// prints the same values).
struct BleSnapshot {
  int64_t      t = 0;
  int32_t      speedMmps = 0;      // Metrics.speedMmps(t), live in every state
  uint32_t     odoPulses = 0;      // Metrics odometer (RSC distance, cumulative)
  WorkoutState state = WorkoutState::Idle;
  uint32_t     wkPulses = 0;       // Workout snapshot (FTMS distance, per session)
  int64_t      totalUs = 0;        // Workout snapshot total (FTMS elapsed)
  uint32_t     workoutId = 0;
};
inline void encodeRsc(const BleSnapshot& s, uint8_t out[RSC_MEAS_LEN]) {
  encodeRscMeasurement(s.speedMmps, s.odoPulses, out);
}
inline void encodeTd(const BleSnapshot& s, uint8_t out[TD_LEN]) {
  encodeTreadmillData(s.speedMmps, s.wkPulses, s.totalUs, out);
}

// Lowercase hex of n bytes into out (2n+1 chars incl. NUL; truncated to fit).
void toHex(const uint8_t* d, size_t n, char* out, size_t outSize);

}  // namespace ble
}  // namespace ot
