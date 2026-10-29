// config_hw.h - pins, hardware constants and EVERY build flag with its default.
//
// Build flags are set from the command line through
//   --build-property "compiler.cpp.extra_flags=-DOT_BUILD_ID=0x<id8> -DOT_BUILD_DIRTY=<0|1> ..."
// (tools/build.sh). Every flag is printed in the boot banner, so a log
// proves which variant ran.
//
// The pure core's constants (pulse filter, metrics, ring size) live in
// src/core/config.h. The hardware/loop constants (HEARTBEAT_US,
// STATS_PERIOD_US, LOOP_BUDGET_US, LOOPBACK_PIN) live here because they are
// not core logic.
#pragma once

#include <stddef.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// Build flags (defaults = the release flag set: sim=0 dev=0 short=0
// loopback=0 bringup=0, verb=2)
// ----------------------------------------------------------------------------
#ifndef OT_SIM_INPUT
#define OT_SIM_INPUT 0           // 1: serial s<kmh>/sj<kmh>/s0 pulse source; GPIO 27 ISR not installed
#endif
#ifndef OT_DEV
#define OT_DEV 0                 // 1: dev serial commands: hb/hc/ha hold begin/complete/abort
#endif
#ifndef OT_SHORT_TIMERS
#define OT_SHORT_TIMERS 0        // 1: bench short timers (AUTO_END/SUMMARY, DIM/OFF)
#endif
#ifndef OT_BRINGUP
#define OT_BRINGUP 0             // 1: display/touch bring-up build
#endif
#ifndef OT_LOOPBACK
#define OT_LOOPBACK 0            // 1: RMT loopback pulse generator on LOOPBACK_PIN, g<rate>,<count>
#endif
#ifndef OT_VERBOSITY_DEFAULT
#define OT_VERBOSITY_DEFAULT 2   // 2 in every build including release
#endif
// Display units when NVS holds no valid value (first boot, erased
// flash, corrupt value): a compile-time default. OT_UNITS_MI (default,
// matches the console's mph/miles) or
// OT_UNITS_KM, e.g. tools/build.sh -D OT_UNITS_DEFAULT=OT_UNITS_KM. The
// values are non-zero on purpose: an unknown token (e.g. "=km") evaluates to
// 0 in #if and is rejected below instead of silently meaning "mi".
#define OT_UNITS_MI 1
#define OT_UNITS_KM 2
#ifndef OT_UNITS_DEFAULT
#define OT_UNITS_DEFAULT OT_UNITS_MI
#endif
#if (OT_UNITS_DEFAULT) != OT_UNITS_MI && (OT_UNITS_DEFAULT) != OT_UNITS_KM
#error "OT_UNITS_DEFAULT must be OT_UNITS_MI or OT_UNITS_KM"
#endif
// ---- BLE -------------------------------------------------------------------
// OT_FTMS_ENABLE (default 1): 1 = FTMS service (Feature, Treadmill
// Data, Training Status, Control Point, Machine Status) next to the RSC
// footpod; 0 = an RSC-only build (no 0x1826 anywhere, banner ftms=0).
#ifndef OT_FTMS_ENABLE
#define OT_FTMS_ENABLE 1
#endif
#if OT_FTMS_ENABLE != 0 && OT_FTMS_ENABLE != 1
#error "OT_FTMS_ENABLE must be 0 or 1"
#endif
// Advertising layout (src/core/ble_payload.h AdvLayout):
//   0 = B   RSC only: the only layout with OT_FTMS_ENABLE=0
//   1 = A   dual-UUID primary + FTMS Service Data (the default with FTMS)
//   2 = B'  B primary + FTMS Service Data in the scan response (fallback 1)
//   3 = B'' both packets with Incomplete UUID lists (fallback 2)
// Banner adv=B|A|Bp|Bpp. Select a fallback with tools/build.sh -D OT_ADV_LAYOUT=2.
#define OT_ADV_B   0
#define OT_ADV_A   1
#define OT_ADV_BP  2
#define OT_ADV_BPP 3
#ifndef OT_ADV_LAYOUT
#if OT_FTMS_ENABLE
#define OT_ADV_LAYOUT OT_ADV_A
#else
#define OT_ADV_LAYOUT OT_ADV_B
#endif
#endif
#if (OT_ADV_LAYOUT) != OT_ADV_B && (OT_ADV_LAYOUT) != OT_ADV_A && (OT_ADV_LAYOUT) != OT_ADV_BP && \
    (OT_ADV_LAYOUT) != OT_ADV_BPP
#error "OT_ADV_LAYOUT must be 0 (B), 1 (A), 2 (B') or 3 (B'')"
#endif
#if !OT_FTMS_ENABLE && (OT_ADV_LAYOUT) != OT_ADV_B
#error "OT_FTMS_ENABLE=0 must use OT_ADV_LAYOUT=0 (B): never advertise 0x1826 without the FTMS service"
#endif
#if OT_FTMS_ENABLE && (OT_ADV_LAYOUT) == OT_ADV_B
#error "OT_FTMS_ENABLE=1 needs an FTMS layout (1 A, 2 B', 3 B''): apps may not find FTMS under B"
#endif

// Build identity. OT_BUILD_ID = first 8 hex digits of git HEAD,
// passed as a number (-DOT_BUILD_ID=0x<id8>). 0 = unknown: the boot
// banner prints build=00000000 followed by a WARNING line.
#ifndef OT_BUILD_ID
#define OT_BUILD_ID 0
#endif
// OT_BUILD_DIRTY: 1 = built from a dirty tree (dev build). When the build
// ID is unknown the provenance is unknown too, so the default is then 1.
#ifndef OT_BUILD_DIRTY
#if (OT_BUILD_ID) == 0
#define OT_BUILD_DIRTY 1
#else
#define OT_BUILD_DIRTY 0
#endif
#endif

#if (OT_SIM_INPUT != 0 && OT_SIM_INPUT != 1) || (OT_DEV != 0 && OT_DEV != 1) ||           \
    (OT_SHORT_TIMERS != 0 && OT_SHORT_TIMERS != 1) || (OT_BRINGUP != 0 && OT_BRINGUP != 1) || \
    (OT_LOOPBACK != 0 && OT_LOOPBACK != 1) || (OT_BUILD_DIRTY != 0 && OT_BUILD_DIRTY != 1)
#error "build flags must be 0 or 1"
#endif
#if OT_VERBOSITY_DEFAULT < 0 || OT_VERBOSITY_DEFAULT > 2
#error "OT_VERBOSITY_DEFAULT must be 0, 1 or 2"
#endif
#if OT_SIM_INPUT && OT_LOOPBACK
#error "OT_SIM_INPUT and OT_LOOPBACK are exclusive (sim does not install the GPIO 27 ISR the loopback tests)"
#endif

// CYD panel switch. 0 = ILI9341 (v1 micro-USB CYD,
// expected from the stock boot log), 1 = ST7789 (2-USB variant). Decided by
// the RDDID 0x04 read at bring-up (first byte 0x00 = ILI9341,
// 0x85 = ST7789); the bring-up build prints the inferred panel and warns on a
// mismatch. Set with tools/build.sh -D CYD_PANEL_ST7789=1.
#ifndef CYD_PANEL_ST7789
#define CYD_PANEL_ST7789 0
#endif
#if CYD_PANEL_ST7789 != 0 && CYD_PANEL_ST7789 != 1
#error "CYD_PANEL_ST7789 must be 0 or 1"
#endif

#if (OT_BUILD_ID) < 0 || (OT_BUILD_ID) > 0xFFFFFFFF
#error "OT_BUILD_ID must be 0x<8 hex digits>"
#endif

namespace hw {

constexpr uint32_t BUILD_ID   = static_cast<uint32_t>(OT_BUILD_ID);
constexpr uint8_t  BUILD_DIRTY = static_cast<uint8_t>(OT_BUILD_DIRTY);

// ----------------------------------------------------------------------------
// Pins (CYD ESP32-2432S028R)
// ----------------------------------------------------------------------------
constexpr uint8_t PULSE_PIN    = 27;  // reed SIG on CN1, INPUT_PULLUP, closure = LOW
constexpr uint8_t MARKER_PIN   = 0;   // BOOT button, active LOW (dev marker, kept from the original pulse logger)
constexpr uint8_t LOOPBACK_PIN = 22;  // CN1, next to 27; OT_LOOPBACK builds only (jumper 22 -> 27)
constexpr uint8_t LED_R_PIN    = 4;   // RGB LED, active LOW: driven HIGH (off) at boot
constexpr uint8_t LED_G_PIN    = 16;
constexpr uint8_t LED_B_PIN    = 17;

// TFT on HSPI (= SPI2_HOST). RST is tied to EN (no GPIO).
constexpr int8_t TFT_SCLK_PIN = 14;
constexpr int8_t TFT_MOSI_PIN = 13;
constexpr int8_t TFT_MISO_PIN = 12;
constexpr int8_t TFT_CS_PIN   = 15;
constexpr int8_t TFT_DC_PIN   = 2;
constexpr int8_t TFT_RST_PIN  = -1;
constexpr int8_t BL_PWM_PIN   = 21;   // backlight, active HIGH; driven ONLY by LovyanGFX Light_PWM

// XPT2046 touch on its own lines, software SPI. The IRQ line (GPIO 36)
// is documented here and NEVER used: touch is polled (GPIO 36/39 input-glitch
// erratum); LGFX_CYD passes pin_int = -1.
constexpr int8_t TOUCH_SCLK_PIN = 25;
constexpr int8_t TOUCH_MOSI_PIN = 32;
constexpr int8_t TOUCH_MISO_PIN = 39;
constexpr int8_t TOUCH_CS_PIN   = 33;
constexpr int8_t TOUCH_IRQ_PIN_UNUSED = 36;

// Display/touch pins must never collide with the reed line, the loopback
// generator, the BOOT marker, the RGB LED or each other.
namespace detail {
constexpr int8_t kDispPins[] = {TFT_SCLK_PIN, TFT_MOSI_PIN, TFT_MISO_PIN, TFT_CS_PIN, TFT_DC_PIN, BL_PWM_PIN,
                                TOUCH_SCLK_PIN, TOUCH_MOSI_PIN, TOUCH_MISO_PIN, TOUCH_CS_PIN, TOUCH_IRQ_PIN_UNUSED};
constexpr int8_t kOtherPins[] = {static_cast<int8_t>(PULSE_PIN), static_cast<int8_t>(MARKER_PIN),
                                 static_cast<int8_t>(LOOPBACK_PIN), static_cast<int8_t>(LED_R_PIN),
                                 static_cast<int8_t>(LED_G_PIN), static_cast<int8_t>(LED_B_PIN)};
constexpr size_t kNDisp  = sizeof(kDispPins) / sizeof(kDispPins[0]);
constexpr size_t kNOther = sizeof(kOtherPins) / sizeof(kOtherPins[0]);
constexpr bool dispPinsDistinct(size_t i = 0, size_t j = 1) {
  return i >= kNDisp ? true
         : j >= kNDisp ? dispPinsDistinct(i + 1, i + 2)
         : (kDispPins[i] == kDispPins[j] ? false : dispPinsDistinct(i, j + 1));
}
constexpr bool dispPinsAvoid(size_t i = 0, size_t j = 0) {
  return i >= kNDisp ? true
         : j >= kNOther ? dispPinsAvoid(i + 1, 0)
         : (kDispPins[i] == kOtherPins[j] ? false : dispPinsAvoid(i, j + 1));
}
}  // namespace detail
static_assert(detail::dispPinsDistinct(), "display/touch pins must be pairwise distinct");
static_assert(detail::dispPinsAvoid(), "display/touch pins must not use GPIO 27/0/22 or the RGB LED pins");

// The ISR reads GPIO_IN_REG, which only covers GPIO 0..31.
static_assert(PULSE_PIN < 32 && MARKER_PIN < 32, "ISR pins must be < 32 (GPIO_IN_REG)");
static_assert(LOOPBACK_PIN != PULSE_PIN && LOOPBACK_PIN != MARKER_PIN, "loopback pin must differ");

// ----------------------------------------------------------------------------
// Serial / debug protocol
// ----------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD     = 115200;  // the CYD's CH340 only works reliably at 115200
constexpr uint32_t DEBUG_TX_BUFFER = 4096;    // Serial TX ring buffer
constexpr uint32_t CMD_MAX_CHARS   = 16;      // longer inbound lines are discarded whole

// ----------------------------------------------------------------------------
// Loop timing
// ----------------------------------------------------------------------------
constexpr int64_t HEARTBEAT_US       = 5000000;   // H line
constexpr int64_t K_PERIOD_US        = 1000000;   // K line (1 Hz snapshot)
constexpr int64_t STATS_PERIOD_US    = 60000000;  // D and G lines
constexpr int64_t LOOP_BUDGET_US     = 40000;     // max single loop pass
constexpr int64_t MARKER_DEBOUNCE_US = 50000;     // BOOT marker: release and lockout
constexpr int64_t DROP_WARN_MIN_US   = 1000000;   // at most one edge-overflow S warning per second

// Short-timer set (OT_SHORT_TIMERS=1 bench builds only; banner short=1).
// The Workout timers are used by app.cpp workoutConfig(), the backlight
// timers by app.cpp uiConfig().
constexpr int64_t SHORT_AUTO_END_US  = 30000000;  // AUTO_END 30 s (production 900 s, src/core/config.h)
constexpr int64_t SHORT_SUMMARY_US   = 12000000;  // SUMMARY 12 s (production 120 s)
constexpr int64_t SHORT_DIM_US       = 6000000;   // DIM 6 s (production 60 s, src/core/ui_policy.h)
constexpr int64_t SHORT_OFF_US       = 15000000;  // OFF 15 s (production 300 s)

// ----------------------------------------------------------------------------
// Display, backlight and touch (used by LGFX_CYD in hw_display.h)
// ----------------------------------------------------------------------------
constexpr uint32_t TFT_SPI_WRITE_HZ = 40000000;  // stock boot log: 40 MHz ILI9341
constexpr uint32_t TFT_SPI_READ_HZ  = 16000000;
constexpr uint32_t TOUCH_SPI_HZ     = 1000000;   // LovyanGFX CYD config. NOTE: LovyanGFX 1.2.26's
                                                 // software-SPI XPT2046 path bit-bangs at CPU speed and
                                                 // ignores this value (it is still set for the record)
constexpr uint16_t TFT_PANEL_W = 240;            // native (portrait) panel size
constexpr uint16_t TFT_PANEL_H = 320;
// Rotation offsets (LovyanGFX CYD config): ILI9341 panel offset_rotation 2;
// ST7789 keeps the panel at 0 and moves the touch by 2 instead.
constexpr uint8_t TFT_PANEL_OFFSET_ROT = CYD_PANEL_ST7789 ? 0 : 2;
constexpr uint8_t TOUCH_OFFSET_ROT     = CYD_PANEL_ST7789 ? 2 : 0;
constexpr uint32_t BL_PWM_HZ = 12000;            // Light_PWM frequency
constexpr uint8_t  BL_FULL   = 255;              // backlight levels (LovyanGFX 0-255): full,
constexpr uint8_t  BL_DIM    = 30;               // dim ~12 % (confirmed on the device, bench 2026-10-22),
constexpr uint8_t  BL_OFF    = 0;                // off. Applied ONLY through lcd.setBrightness()
static_assert(BL_OFF < BL_DIM && BL_DIM < BL_FULL, "backlight levels: off < dim < full");
constexpr bool     UNITS_DEFAULT_KM = (OT_UNITS_DEFAULT) == OT_UNITS_KM;  // units when NVS holds none
constexpr uint8_t  BL_PWM_CH = 7;                // Light_PWM channel. NOTE: with core 3.x LovyanGFX
                                                 // 1.2.26 calls ledcAttach(pin, freq, bits), which
                                                 // picks the channel itself; this value is ignored

// ---- MEASURED bring-up values (bench, 2026-10-22/23) ----
// Landscape rotation chosen on the bench (2026-10-22): text, arrow
// and TOP label read correctly at rotation 1 (MADCTL reads back 0xB8).
constexpr uint8_t DISPLAY_ROTATION = 1;
// Touch calibration (LovyanGFX Touch_XPT2046 config): raw X at native x = 0 /
// x = 239 and raw Y at native y = 0 / y = 319 (portrait panel frame, before
// rotation). Measured: tools/touch_cal.py on a 2026-10-23 touch capture
// (kc fit, resid 6 px; firmware
// cal;suggest 301/3805/3722/184 agrees within 4 raw units). The 2026-10-22
// run (304/3750/3750/223) passed the 3x3 grid at max 7/9 px.
constexpr uint16_t TOUCH_X_MIN = 297;
constexpr uint16_t TOUCH_X_MAX = 3808;
constexpr uint16_t TOUCH_Y_MIN = 3722;
constexpr uint16_t TOUCH_Y_MAX = 184;
// Minimum pressure for a valid touch sample, in the units getTouchRaw()
// reports (touch_point_t::size = median raw z >> 8). LovyanGFX's own
// threshold is size >= 1 (getTouchRaw returns 0 touches when size is 0).
// Measured on the bench: the weakest deliberate press (light, damp) at the
// worst screen position projects to ~0, so there is no headroom above the
// library default: keep 1. Resistive touch already needs real pressure;
// accidental-touch protection is the gesture logic's job.
constexpr uint16_t TOUCH_Z_MIN = 1;
constexpr bool     CALIBRATION_PROVISIONAL = false;  // values above are measured

}  // namespace hw
