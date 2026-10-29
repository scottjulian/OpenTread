// hw_pulse.cpp - see hw_pulse.h. ISR/ring logic ported from the original
// pulse logger (pushEdge, drainEvents).
#include "hw_pulse.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_intr_alloc.h>
#include <esp_memory_utils.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>

#include "config_hw.h"
#include "src/core/config.h"

namespace hwp {

namespace {

constexpr uint32_t RING_SIZE = ot::EDGE_RING_SIZE;  // 1024, power of two
constexpr uint32_t RING_MASK = RING_SIZE - 1u;
static_assert((RING_SIZE & RING_MASK) == 0, "ring size must be a power of two");
static_assert(sizeof(EdgeEvent) == 16, "EdgeEvent is 16 bytes (int64 + 2 bytes, padded)");

// Plain zero-initialised globals: .bss, which is DRAM on the ESP32 (no
// PSRAM, no const tables in flash). Checked at runtime with esp_ptr_in_dram.
EdgeEvent         gRing[RING_SIZE];
volatile uint32_t gHead    = 0;  // written only by the ISR (release), read by loop (acquire)
volatile uint32_t gTail    = 0;  // written only by loop (release), read by the ISR (acquire)
volatile uint32_t gDropped = 0;  // written only by the ISR

template <typename F>
uintptr_t fnAddr(F* f) { return reinterpret_cast<uintptr_t>(f); }

bool inIram(uintptr_t a) { return esp_ptr_in_iram(reinterpret_cast<const void*>(a)); }
bool inDram(const volatile void* p) { return esp_ptr_in_dram(const_cast<const void*>(p)); }

}  // namespace

// The ISR functions use REG_READ, whose DPORT static-assert declares a
// block-scope extern; inside an unnamed namespace that would get internal
// linkage (-Wunused-function), so these are file-static in hwp instead.
//
// Producer. IRAM, no flash access, no float, no logging: esp_timer_get_time
// is in IRAM (CONFIG_ESP_TIMER_IN_IRAM=y), REG_READ is a register load, the
// __atomic ops on aligned 32-bit words compile to plain loads/stores + memw.
static void IRAM_ATTR pushEdge(uint8_t src, uint8_t pin) {
  const int64_t  t   = esp_timer_get_time();
  const uint8_t  lvl = static_cast<uint8_t>((REG_READ(GPIO_IN_REG) >> pin) & 1u);
  const uint32_t h   = gHead;                                         // we own head
  const uint32_t tl  = __atomic_load_n(&gTail, __ATOMIC_ACQUIRE);
  if (static_cast<uint32_t>(h - tl) >= RING_SIZE) {                   // full: count, never overwrite
    gDropped = gDropped + 1u;
    return;
  }
  EdgeEvent& slot = gRing[h & RING_MASK];
  slot.t     = t;
  slot.src   = src;
  slot.level = lvl;
  __atomic_store_n(&gHead, h + 1u, __ATOMIC_RELEASE);                 // publish after the slot
}

static void IRAM_ATTR pulseIsr(void* /*arg*/)  { pushEdge(SRC_PULSE,  hw::PULSE_PIN); }
static void IRAM_ATTR markerIsr(void* /*arg*/) { pushEdge(SRC_MARKER, hw::MARKER_PIN); }

static inline uint8_t readPin(uint8_t pin) {
  return static_cast<uint8_t>((REG_READ(GPIO_IN_REG) >> pin) & 1u);
}

void begin(bool installPulse, InitInfo& info) {
  info = InitInfo{};
  pinMode(hw::MARKER_PIN, INPUT_PULLUP);
  pinMode(hw::PULSE_PIN, INPUT_PULLUP);  // harmless with the console's pull-up

  // The FIRST GPIO-interrupt call of the firmware. If anything had
  // installed a (non-IRAM) service before us this returns
  // ESP_ERR_INVALID_STATE and the banner reports isr_iram=0.
  info.intrFlags  = static_cast<uint32_t>(ESP_INTR_FLAG_IRAM);
  info.installErr = gpio_install_isr_service(static_cast<int>(ESP_INTR_FLAG_IRAM));
  info.isrCore    = static_cast<uint8_t>(xPortGetCoreID());

  // t0 and the initial levels BEFORE any handler is added: every logged
  // edge is later than t0 (pulse-logger rule; the filter treats the boot state as a
  // pseudo-edge at t0).
  info.t0          = esp_timer_get_time();
  info.pulseLevel  = readPin(hw::PULSE_PIN);
  info.markerLevel = readPin(hw::MARKER_PIN);

  if (installPulse) {
    info.pulseTypeErr   = gpio_set_intr_type(static_cast<gpio_num_t>(hw::PULSE_PIN), GPIO_INTR_ANYEDGE);
    info.pulseAddErr    = gpio_isr_handler_add(static_cast<gpio_num_t>(hw::PULSE_PIN), pulseIsr, nullptr);
    info.pulseInstalled = (info.pulseTypeErr == ESP_OK && info.pulseAddErr == ESP_OK) ? 1 : 0;
  }
  info.markerTypeErr = gpio_set_intr_type(static_cast<gpio_num_t>(hw::MARKER_PIN), GPIO_INTR_ANYEDGE);
  info.markerAddErr  = gpio_isr_handler_add(static_cast<gpio_num_t>(hw::MARKER_PIN), markerIsr, nullptr);

  // isr_iram, computed from what actually happened (not assumed):
  //  - the service was installed by us, with ESP_INTR_FLAG_IRAM (ESP_OK);
  //  - every handler we added was added successfully;
  //  - the handlers and pushEdge are in IRAM; the ring and its indices in DRAM.
  // The IDF dispatcher gpio_intr_service itself is in .iram1 of
  // libesp_driver_gpio.a (checked with objdump); its allocation is shown
  // by the 'd' command (esp_intr_dump).
  info.pulseIsrAddr    = fnAddr(&pulseIsr);
  info.markerIsrAddr   = fnAddr(&markerIsr);
  info.pushEdgeAddr    = fnAddr(&pushEdge);
  info.ringAddr        = reinterpret_cast<uintptr_t>(&gRing[0]);
  info.pulseIsrInIram  = inIram(info.pulseIsrAddr) ? 1 : 0;
  info.markerIsrInIram = inIram(info.markerIsrAddr) ? 1 : 0;
  info.pushEdgeInIram  = inIram(info.pushEdgeAddr) ? 1 : 0;
  info.ringInDram      = (inDram(&gRing[0]) && inDram(&gRing[RING_SIZE - 1u]) && inDram(&gHead) &&
                          inDram(&gTail) && inDram(&gDropped)) ? 1 : 0;
  const bool pulseOk  = !installPulse || info.pulseInstalled;
  const bool markerOk = info.markerTypeErr == ESP_OK && info.markerAddErr == ESP_OK;
  info.isrIram = (info.installErr == ESP_OK && pulseOk && markerOk && info.pulseIsrInIram &&
                  info.markerIsrInIram && info.pushEdgeInIram && info.ringInDram) ? 1 : 0;
}

bool peekEdge(EdgeEvent& out) {
  const uint32_t head = __atomic_load_n(&gHead, __ATOMIC_ACQUIRE);  // slot writes visible after this
  const uint32_t tail = gTail;                                      // we own tail
  if (head == tail) return false;
  const EdgeEvent& s = gRing[tail & RING_MASK];
  out.t     = s.t;   // the ISR never writes this slot until popEdge releases it
  out.src   = s.src;
  out.level = s.level;
  return true;
}

void popEdge() {
  const uint32_t tail = gTail;
  if (__atomic_load_n(&gHead, __ATOMIC_ACQUIRE) == tail) return;  // empty: nothing to pop
  __atomic_store_n(&gTail, tail + 1u, __ATOMIC_RELEASE);           // free the slot (already copied)
}

uint32_t droppedEdges() { return gDropped; }

uint32_t occupancy() {
  return __atomic_load_n(&gHead, __ATOMIC_ACQUIRE) - gTail;
}

uint8_t readPulseLevel() { return readPin(hw::PULSE_PIN); }

}  // namespace hwp
