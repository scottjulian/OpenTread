// emu_platform.h - the emulated ESP32/Arduino platform for the host
// emulation of the real app.cpp + hw_ble*.cpp (test/emu).
// Header-only (C++17 inline variables): one emulated device per process.
//
// Clock: esp_timer_get_time() returns the emulated time and advances it by
// 2 us per call (a loop pass takes time); vTaskDelay(n) advances n ms.
// Serial: everything written is appended to emu::serialOut; bytes queued
// with emu::serialIn() are read by the firmware's command reader.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <sys/types.h>

namespace emu {
inline int64_t     gNow = 1000000;          // emulated esp_timer time (us); boot at 1 s
inline std::string serialOut;               // captured TX
inline std::deque<uint8_t> serialRx;        // pending RX
inline int         txRoom = 4096;           // Serial.availableForWrite()
inline uint64_t    timerCalls = 0;
inline void serialIn(const std::string& s) {
  for (char c : s) serialRx.push_back(static_cast<uint8_t>(c));
}
}  // namespace emu

inline int64_t esp_timer_get_time() {
  ++emu::timerCalls;
  emu::gNow += 2;
  return emu::gNow;
}

// ---- Arduino ---------------------------------------------------------------
#define OUTPUT 0x03
#define HIGH 0x1
#define LOW 0x0
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}

class HardwareSerialStub {
 public:
  void setTxBufferSize(size_t) {}
  void begin(unsigned long) {}
  int availableForWrite() { return emu::txRoom; }
  size_t write(const uint8_t* b, size_t n) {
    emu::serialOut.append(reinterpret_cast<const char*>(b), n);
    return n;
  }
  int available() { return static_cast<int>(emu::serialRx.size()); }
  int read() {
    if (emu::serialRx.empty()) return -1;
    const int c = emu::serialRx.front();
    emu::serialRx.pop_front();
    return c;
  }
};
inline HardwareSerialStub Serial;

// ---- ESP-IDF -----------------------------------------------------------------
typedef int esp_err_t;
#define ESP_OK 0
#define MALLOC_CAP_8BIT (1u << 2)
#define MALLOC_CAP_DMA (1u << 3)
inline size_t   heap_caps_get_largest_free_block(uint32_t) { return 110580; }
inline uint32_t esp_get_free_heap_size() { return 150000; }
inline uint32_t esp_get_minimum_free_heap_size() { return 140000; }
typedef enum { ESP_MAC_WIFI_STA, ESP_MAC_WIFI_SOFTAP, ESP_MAC_BT, ESP_MAC_ETH } esp_mac_type_t;
inline esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t) {
  const uint8_t m[6] = {0xcc, 0xdb, 0xa7, 0x00, 0x00, 0x02};
  for (int i = 0; i < 6; ++i) mac[i] = m[i];
  return ESP_OK;
}
inline int esp_reset_reason() { return 1; }
inline esp_err_t esp_intr_dump(FILE*) { return ESP_OK; }
typedef enum { ESP_LOG_NONE } esp_log_level_t;
inline void esp_log_level_set(const char*, esp_log_level_t) {}

// newlib's fopencookie (debug_serial.cpp's interrupt-dump stream): not on
// macOS; returning nullptr makes 'd' print "intr;unavailable".
#if defined(__APPLE__)
typedef struct {
  ssize_t (*read)(void*, char*, size_t);
  ssize_t (*write)(void*, const char*, size_t);
  int (*seek)(void*, off_t*, int);
  int (*close)(void*);
} cookie_io_functions_t;
inline FILE* fopencookie(void*, const char*, cookie_io_functions_t) { return nullptr; }
#endif

// ---- FreeRTOS ------------------------------------------------------------------
typedef struct { int unused; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
namespace emu {
inline int lockDepth = 0;      // the spinlock is never taken twice (no nesting)
inline int lockViolations = 0;
}  // namespace emu
inline void portENTER_CRITICAL(portMUX_TYPE*) {
  if (emu::lockDepth != 0) ++emu::lockViolations;
  ++emu::lockDepth;
}
inline void portEXIT_CRITICAL(portMUX_TYPE*) { --emu::lockDepth; }
inline void vTaskDelay(uint32_t ticks) { emu::gNow += static_cast<int64_t>(ticks) * 1000; }
