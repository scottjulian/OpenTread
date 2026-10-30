// The LovyanGFX platform functions (normally in its sdl/common.cpp), for a
// headless host build: nothing here touches real time or hardware.
#include <LovyanGFX.hpp>

namespace lgfx {
inline namespace v1 {
unsigned long millis(void) { return 0; }
unsigned long micros(void) { return 0; }
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}
void gpio_hi(uint32_t) {}
void gpio_lo(uint32_t) {}
bool gpio_in(uint32_t) { return false; }
void pinMode(int_fast16_t, pin_mode_t) {}
void lgfxPinMode(int_fast16_t, pin_mode_t) {}
}  // namespace v1
}  // namespace lgfx
