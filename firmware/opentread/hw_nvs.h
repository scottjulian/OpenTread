// hw_nvs.h - the units setting in NVS through Preferences (N lines; the
// policy is src/core/persist_policy.*).
//
// Namespace "opentread" (no collision with NimBLE's "nimble_bond" or the PHY's
// "phy"), keys "schema" (u8 = 1) and "units" (u8: 0 imperial, 1 metric).
// NVS sits at 0x9000 in every core-3.3.11 partition scheme, so a partition
// change keeps the setting.
//
// Rules:
//  - Loop task only (setup() for the boot read). Never from an ISR or a
//    callback.
//  - readUnits() opens the namespace READ-ONLY and never writes (no write at
//    boot). nvs_flash_init() already ran in the core's initArduino() before
//    setup(), so the read is never "before NVS init". A missing namespace or
//    key is a normal case (first boot, erased flash; NimBLE/the core erase NVS
//    on NO_FREE_PAGES / NEW_VERSION) and reads as "missing".
//  - writeUnits() opens read-write, puts the keys (Preferences commits each
//    put), and ALWAYS closes the handle again (Preferences never left open).
//    An entry write takes ~1-3 ms; a page erase/relocation tens of ms, during
//    which the flash cache is off on both cores. The GPIO 27 ISR is
//    installed with ESP_INTR_FLAG_IRAM and its code/data are in IRAM/DRAM
//    so edges are still timestamped; the ring absorbs the backlog.
//    The app commits only while the belt is stationary (PersistPolicy).
#pragma once

#include <stdint.h>

namespace nvshw {

constexpr const char* NAMESPACE  = "opentread";
constexpr const char* KEY_UNITS  = "units";
constexpr const char* KEY_SCHEMA = "schema";

struct Read {
  bool    opened    = false;  // the namespace exists (read-only open succeeded)
  bool    hasSchema = false;
  uint8_t schema    = 0;
  bool    hasUnits  = false;
  uint8_t units     = 0;
  uint32_t us       = 0;      // duration of the read
};

// Boot read (read-only).
Read readUnits();

struct Write {
  bool     ok        = false;  // every requested put succeeded
  bool     opened    = false;
  bool     schemaPut = false;  // the schema key was written in this call
  uint32_t us        = 0;      // duration of the whole open/put/close
};

// Write `units` (raw byte: the OT_DEV nX command writes an invalid 7), and
// the schema key when withSchema is true.
Write writeUnits(uint8_t units, bool withSchema);

// Entry statistics of the default NVS partition (nvs_get_stats; no
// allocation). Loop task only: the OT_DEV nw<k> bench prints them before and
// after the forced commits, so the log shows the pages being cycled.
// ok = false if the call failed (err = its esp_err_t).
struct Stats {
  bool     ok        = false;
  int32_t  err       = 0;
  uint32_t used      = 0;
  uint32_t free      = 0;  // includes the reserved (spare) page's entries
  uint32_t available = 0;
  uint32_t total     = 0;
  uint32_t namespaces = 0;
};
Stats stats();

}  // namespace nvshw
