// persist_policy.h - units persistence policy: the units setting is saved in
// NVS and survives power-off; imperial when NVS holds nothing valid.
//
// Load (validateUnits): the boot read of NVS namespace "opentread" gives the keys
// "schema" (u8) and "units" (u8: 0 imperial, 1 metric). Valid iff schema ==
// NVS_SCHEMA (1) and units is 0 or 1; anything else (a key missing, schema 0
// or 2, units 2 / 7 / 255) -> the default units (imperial) and an N,default
// line. The boot path NEVER writes.
//
// Commit (PersistPolicy::decide, called once per loop pass): a write is due
// when ALL of these hold at `now`:
//   1. current != stored (a net change: toggling back within the settle
//      window writes nothing);
//   2. now - lastToggle >= NVS_SETTLE_US (3 s; at exactly 3.000 s, not 2.999);
//   3. no earlier write, or now - lastWrite >= NVS_MIN_INTERVAL_US (10 s);
//   4. the belt is stationary: no accepted pulse yet, or now - lastPulse >
//      STOP_US (the same STOP as Metrics / Workout: a pulse AT last + STOP is
//      still movement, so the first commit instant is last + STOP + 1 us);
//   5. no touch hold is in progress (never inside a gesture's critical window).
// When 1-3 hold but 4 or 5 does not, decide() returns Defer ONCE per pending
// change (the app logs N,defer); later passes return None until the commit
// can happen (or a new toggle restarts the settle window and re-arms Defer).
// After the write attempt the app calls written(now, value, ok): the rate
// limit counts every attempt (a failing flash is retried at most once per
// 10 s), and only a successful write updates `stored`.
//
// Why "stationary": the pulse ISR is IRAM-safe (ESP_INTR_FLAG_IRAM) and
// keeps timestamping while the flash cache is disabled, but there is no
// reason to write flash mid-stride (defence in depth).
//
// Known limitation: a power cut within 3 s of a toggle, or while
// the belt keeps moving after one, loses that toggle.
//
// Pure C++17, integer only, no allocation, no clock reads.
#pragma once

#include <cstdint>

#include "config.h"
#include "format.h"

namespace ot {

constexpr int64_t NVS_SETTLE_US       = 3000000;   // no further toggle for 3 s
constexpr int64_t NVS_MIN_INTERVAL_US = 10000000;  // at most one write per 10 s
constexpr uint8_t NVS_SCHEMA          = 1;
constexpr uint8_t NVS_UNITS_MI        = 0;         // the "units" byte = static_cast<uint8_t>(Units)
constexpr uint8_t NVS_UNITS_KM        = 1;
constexpr Units   UNITS_DEFAULT       = Units::Mi; // imperial
static_assert(static_cast<uint8_t>(Units::Mi) == NVS_UNITS_MI && static_cast<uint8_t>(Units::Km) == NVS_UNITS_KM,
              "the NVS units byte is the Units enum value");
static_assert(NVS_SETTLE_US > 0 && NVS_MIN_INTERVAL_US >= NVS_SETTLE_US, "NVS timers");

enum class LoadStatus : uint8_t { Ok, Missing, BadSchema, BadValue };
const char* loadStatusName(LoadStatus s);  // "ok" / "missing" / "bad_schema" / "bad_value"

struct UnitsLoad {
  Units      units  = UNITS_DEFAULT;
  LoadStatus status = LoadStatus::Missing;
};

// hasX = the key exists (Preferences::isKey). A missing namespace = both
// missing. dflt = the units used when the stored value is not valid.
UnitsLoad validateUnits(bool hasSchema, uint8_t schema, bool hasUnits, uint8_t units, Units dflt = UNITS_DEFAULT);
uint8_t   unitsByte(Units u);

struct PersistConfig {
  int64_t settleUs      = NVS_SETTLE_US;
  int64_t minIntervalUs = NVS_MIN_INTERVAL_US;
  int64_t stopUs        = STOP_US;
};

enum class PersistAction : uint8_t { None, Commit, Defer };
const char* persistActionName(PersistAction a);  // "none" / "commit" / "defer"

class PersistPolicy {
 public:
  explicit PersistPolicy(const PersistConfig& cfg = PersistConfig{});

  // Boot: `stored` = the units the device booted with (the validated NVS
  // value, or the default). A default is never written back by itself: only
  // a toggle to a different value causes a write.
  void begin(Units stored);
  // A units toggle at t (the settle window restarts).
  void onToggle(int64_t t);
  // The loop's decision at `now` (see the header comment).
  PersistAction decide(int64_t now, Units current, bool hasPulse, int64_t lastPulseUs, bool holdActive);
  // After a write attempt of `value` at `now`.
  void written(int64_t now, Units value, bool ok);

  Units   stored() const { return stored_; }
  bool    pending(Units current) const { return current != stored_; }
  bool    hasWritten() const { return hasWrite_; }
  int64_t lastWriteUs() const { return lastWrite_; }
  bool    hasToggle() const { return hasToggle_; }
  int64_t lastToggleUs() const { return lastToggle_; }
  bool    deferReported() const { return deferReported_; }

 private:
  PersistConfig cfg_;
  Units   stored_        = UNITS_DEFAULT;
  bool    hasToggle_     = false;
  int64_t lastToggle_    = 0;
  bool    hasWrite_      = false;
  int64_t lastWrite_     = 0;
  bool    deferReported_ = false;
};

}  // namespace ot
