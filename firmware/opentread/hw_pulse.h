// hw_pulse.h - GPIO 27 reed-switch edges (and the GPIO 0 BOOT marker) into a
// lock-free SPSC edge ring.
//
// Port of the original pulse logger's pushEdge/ring, installed IRAM-safe:
//   gpio_install_isr_service(ESP_INTR_FLAG_IRAM) is the FIRST GPIO-interrupt
//   call, then gpio_set_intr_type(ANYEDGE) + gpio_isr_handler_add() per pin.
//   No attachInterrupt anywhere (its service is installed non-IRAM in core
//   3.3.11 and would be masked during flash writes).
// Producer: the IDF GPIO dispatcher (gpio_intr_service, .iram1) on core 1,
// which calls the pin handlers one after another, never concurrently: one
// producer. Consumer: loop() only.
//
// This header has no Arduino/IDF includes.
#pragma once

#include <stdint.h>

namespace hwp {

enum : uint8_t { SRC_PULSE = 0, SRC_MARKER = 1 };

// One ring slot (16 bytes, DRAM). level = pin level read in the ISR right
// after the edge (0 = closed / LOW, 1 = open / HIGH).
struct EdgeEvent {
  int64_t t;
  uint8_t src;
  uint8_t level;
};

// What begin() did, for the banner and the 'd' dump.
struct InitInfo {
  int64_t  t0;              // sampled after pinMode, BEFORE any handler was added
  uint8_t  pulseLevel;      // GPIO 27 level at t0
  uint8_t  markerLevel;     // GPIO 0 level at t0
  uint8_t  pulseInstalled;  // 0 in a sim build (the two pulse sources are exclusive)
  uint8_t  isrCore;         // core that installed the service (= core of the ISR)
  int32_t  installErr;      // gpio_install_isr_service result (0 = ESP_OK)
  int32_t  pulseTypeErr;    // gpio_set_intr_type(27) result (0 when not installed)
  int32_t  pulseAddErr;     // gpio_isr_handler_add(27) result
  int32_t  markerTypeErr;
  int32_t  markerAddErr;
  uint32_t intrFlags;       // flags passed to gpio_install_isr_service
  uint8_t  pulseIsrInIram;  // esp_ptr_in_iram(&pulseIsr)
  uint8_t  markerIsrInIram;
  uint8_t  pushEdgeInIram;
  uint8_t  ringInDram;      // ring, head, tail and drop counter all in DRAM
  uint8_t  isrIram;         // banner isr_iram: all of the above OK
  uintptr_t pulseIsrAddr;
  uintptr_t markerIsrAddr;
  uintptr_t pushEdgeAddr;
  uintptr_t ringAddr;
};

// Configure GPIO 27/0 (INPUT_PULLUP), install the IRAM ISR service, sample
// t0 and the initial levels, then add the handlers. installPulse = false
// (sim build) skips the GPIO 27 handler; the marker is always installed.
// Call once from setup(), on the loop core.
void begin(bool installPulse, InitInfo& info);

// Consumer side (loop() only). peekEdge copies the oldest edge without
// removing it; popEdge removes it. Typical drain:
//   while (peekEdge(e) && e.t <= now) { popEdge(); handle(e); }
bool peekEdge(EdgeEvent& out);
void popEdge();

uint32_t droppedEdges();   // ISR ring overflows (never overwrites), pulse + marker
uint32_t occupancy();      // edges currently in the ring
uint8_t  readPulseLevel(); // GPIO 27 level now (REG_READ, no digitalRead)

}  // namespace hwp
