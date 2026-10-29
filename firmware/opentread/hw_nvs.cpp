// hw_nvs.cpp - see hw_nvs.h.
#include "config_hw.h"

#if !OT_BRINGUP

#include "hw_nvs.h"

#include <Preferences.h>
#include <esp_timer.h>
#include <nvs.h>
#include <string.h>

#include "src/core/persist_policy.h"

namespace nvshw {

namespace {

Preferences gPrefs;  // one instance; every begin() is paired with end() in the same call

uint32_t since(int64_t t0) {
  const int64_t d = esp_timer_get_time() - t0;
  return d < 0 ? 0u : (d > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(d));
}

}  // namespace

Read readUnits() {
  Read r;
  const int64_t t0 = esp_timer_get_time();
  r.opened = gPrefs.begin(NAMESPACE, /*readOnly=*/true);  // false: namespace missing (first boot / erased)
  if (r.opened) {
    r.hasSchema = gPrefs.isKey(KEY_SCHEMA);
    if (r.hasSchema) r.schema = gPrefs.getUChar(KEY_SCHEMA, 0);
    r.hasUnits = gPrefs.isKey(KEY_UNITS);
    if (r.hasUnits) r.units = gPrefs.getUChar(KEY_UNITS, 0);
    gPrefs.end();
  }
  r.us = since(t0);
  return r;
}

Write writeUnits(uint8_t units, bool withSchema) {
  Write w;
  const int64_t t0 = esp_timer_get_time();
  w.opened = gPrefs.begin(NAMESPACE, /*readOnly=*/false);
  if (w.opened) {
    bool ok = gPrefs.putUChar(KEY_UNITS, units) == 1u;
    if (withSchema) {
      w.schemaPut = gPrefs.putUChar(KEY_SCHEMA, ot::NVS_SCHEMA) == 1u;
      ok = ok && w.schemaPut;
    }
    gPrefs.end();
    w.ok = ok;
  }
  w.us = since(t0);
  return w;
}

Stats stats() {
  Stats st;
  nvs_stats_t n;
  memset(&n, 0, sizeof(n));
  const esp_err_t e = nvs_get_stats(nullptr, &n);  // nullptr: the default "nvs" partition
  st.err = static_cast<int32_t>(e);
  st.ok = e == ESP_OK;
  if (st.ok) {
    st.used = static_cast<uint32_t>(n.used_entries);
    st.free = static_cast<uint32_t>(n.free_entries);
    st.available = static_cast<uint32_t>(n.available_entries);
    st.total = static_cast<uint32_t>(n.total_entries);
    st.namespaces = static_cast<uint32_t>(n.namespace_count);
  }
  return st;
}

}  // namespace nvshw

#endif  // !OT_BRINGUP
