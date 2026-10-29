// hw_display.h - the CYD display device class LGFX_CYD, plus either the
// display/touch bring-up API (OT_BRINGUP=1 builds) or the main-screen
// renderer (every other build).
//
// LGFX_CYD is the class that SHIPS: it was verified with the bring-up build
// and is used unchanged by the renderer. Its configuration, all from
// config_hw.h:
//   Bus_SPI        SPI2_HOST (HSPI), mode 0, 40 MHz write / 16 MHz read,
//                  SPI_DMA_CH_AUTO, use_lock, SCLK 14 / MOSI 13 / MISO 12, DC 2
//   Panel_ILI9341  CS 15, RST -1 (tied to EN), 240x320, offset_rotation 2,
//                  readable. CYD_PANEL_ST7789=1 swaps in Panel_ST7789 (panel
//                  offset_rotation 0, touch offset_rotation 2).
//   Light_PWM      GPIO 21, 12 kHz, channel 7 (the only code that drives
//                  the backlight; nothing else touches LEDC on GPIO 21)
//   Touch_XPT2046  spi_host -1 (software SPI on its own lines), SCLK 25 /
//                  MOSI 32 / MISO 39 / CS 33, pin_int -1 (IRQ GPIO 36 is
//                  never attached: touch is polled), 1 MHz, calibrated ranges
// No pin of this class is GPIO 27 (reed), 22 (loopback), 0 (marker) or the
// RGB LED (static_asserts in config_hw.h). LGFX_AUTODETECT is never used: the
// detector drives GPIO 27 as an output while probing and writes NVS.
//
// Boot order: LED pins HIGH (app_setup, first) ->
// setBrightness(0) -> init() -> reserveDmaBuffers() -> create sprites -> first frame ->
// setBrightness(full) -> NimBLEDevice::init.
//
// Every display/touch call happens in the loop task (setup()/loop()), never
// from an ISR or a callback.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config_hw.h"
#include "src/core/format.h"
#include "src/core/touch_gesture.h"
#include "src/core/workout.h"

#if defined(LGFX_AUTODETECT)
#error "LGFX_AUTODETECT must never be defined: it drives GPIO 27 as an output while probing"
#endif

#include <LovyanGFX.hpp>

// The pinned library. The fallback 1.2.33 is a deliberate decision, not a
// silent switch: changing it means changing this guard.
static_assert(LGFX_VERSION_MAJOR == 1 && LGFX_VERSION_MINOR == 2 && LGFX_VERSION_PATCH == 26,
              "LovyanGFX must be 1.2.26");

#define OT_STR2_(x) #x
#define OT_STR_(x) OT_STR2_(x)
#define OT_LGFX_VERSION_STR \
  OT_STR_(LGFX_VERSION_MAJOR) "." OT_STR_(LGFX_VERSION_MINOR) "." OT_STR_(LGFX_VERSION_PATCH)

class LGFX_CYD : public lgfx::LGFX_Device {
 public:
#if CYD_PANEL_ST7789
  using Panel = lgfx::Panel_ST7789;
  static constexpr const char* PANEL_NAME = "ST7789";
#else
  using Panel = lgfx::Panel_ILI9341;
  static constexpr const char* PANEL_NAME = "ILI9341";
#endif

  LGFX_CYD() {
    {
      auto cfg = bus_.config();
      cfg.spi_host    = SPI2_HOST;  // HSPI
      cfg.spi_mode    = 0;
      cfg.freq_write  = hw::TFT_SPI_WRITE_HZ;
      cfg.freq_read   = hw::TFT_SPI_READ_HZ;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = hw::TFT_SCLK_PIN;
      cfg.pin_mosi    = hw::TFT_MOSI_PIN;
      cfg.pin_miso    = hw::TFT_MISO_PIN;
      cfg.pin_dc      = hw::TFT_DC_PIN;
      bus_.config(cfg);
      panel_.setBus(&bus_);
    }
    {
      auto cfg = panel_.config();  // panel-class defaults kept for the dummy-read and memory fields
      cfg.pin_cs          = hw::TFT_CS_PIN;
      cfg.pin_rst         = hw::TFT_RST_PIN;
      cfg.pin_busy        = -1;
      cfg.panel_width     = hw::TFT_PANEL_W;
      cfg.panel_height    = hw::TFT_PANEL_H;
      cfg.offset_x        = 0;
      cfg.offset_y        = 0;
      cfg.offset_rotation = hw::TFT_PANEL_OFFSET_ROT;
      cfg.readable        = true;
      cfg.invert          = false;
      cfg.rgb_order       = false;
      cfg.dlen_16bit      = false;
      cfg.bus_shared      = false;  // nothing else (no SD card) on the TFT bus
      panel_.config(cfg);
    }
    {
      auto cfg = light_.config();
      cfg.pin_bl      = hw::BL_PWM_PIN;
      cfg.invert      = false;
      cfg.freq        = hw::BL_PWM_HZ;
      cfg.pwm_channel = hw::BL_PWM_CH;
      light_.config(cfg);
      panel_.setLight(&light_);
    }
    {
      auto cfg = touch_.config();
      cfg.x_min           = hw::TOUCH_X_MIN;
      cfg.x_max           = hw::TOUCH_X_MAX;
      cfg.y_min           = hw::TOUCH_Y_MIN;
      cfg.y_max           = hw::TOUCH_Y_MAX;
      cfg.pin_int         = -1;     // IRQ (GPIO 36) never attached: polled
      cfg.pin_rst         = -1;
      cfg.bus_shared      = false;  // own lines, not the TFT bus
      cfg.offset_rotation = hw::TOUCH_OFFSET_ROT;
      cfg.spi_host        = -1;     // software SPI
      cfg.freq            = hw::TOUCH_SPI_HZ;
      cfg.pin_sclk        = hw::TOUCH_SCLK_PIN;
      cfg.pin_mosi        = hw::TOUCH_MOSI_PIN;
      cfg.pin_miso        = hw::TOUCH_MISO_PIN;
      cfg.pin_cs          = hw::TOUCH_CS_PIN;
      touch_.config(cfg);
      panel_.setTouch(&touch_);  // also derives the raw->screen affine from the ranges
    }
    setPanel(&panel_);
  }

  // Bus DMA flip-buffer size, per buffer (two are kept). LovyanGFX 1.2.26's
  // Bus_SPI::writePixels (every converting push, e.g. a 1/4-bpp sprite onto
  // the RGB565 panel) copies in chunks of 32, 64 .. 512 px and a last chunk
  // of up to 1023 px = 2046 B; writeBytes copies < 1024 B. Panel_Device::init
  // only reserves one line (640 B), so without this every hero push would
  // free and heap_caps_malloc the DMA buffers in the loop (no
  // allocation after setup()). 2048 B is the largest request any draw path
  // of this class makes, so FlipBuffer::getBuffer never grows or shrinks.
  static constexpr uint32_t DMA_FLIP_BYTES = 2048;

  // Call once, right after init() (boot order step 3). Returns false if the
  // DMA-capable heap could not supply both buffers.
  bool reserveDmaBuffers() { return bus_.reserveDMABuffer(DMA_FLIP_BYTES); }

#if OT_BRINGUP
  // Bring-up diagnostics only: the touch driver's live config, so a
  // calibration measured on the bench can be tried in RAM before it is
  // hardcoded in config_hw.h. Re-derives the affine (touchCalibrate()).
  void setTouchRanges(uint16_t xMin, uint16_t xMax, uint16_t yMin, uint16_t yMax) {
    auto cfg = touch_.config();
    cfg.x_min = xMin;
    cfg.x_max = xMax;
    cfg.y_min = yMin;
    cfg.y_max = yMax;
    touch_.config(cfg);
    panel_.touchCalibrate();
  }
#endif

 private:
  lgfx::Bus_SPI       bus_;
  Panel               panel_;
  lgfx::Light_PWM     light_;
  lgfx::Touch_XPT2046 touch_;
};

#if OT_BRINGUP
// ---------------------------------------------------------------------------
// Display/touch bring-up mode. Serial commands, all <= 16 chars and echoed
// as S,t,cmd;<text>:
//   m            home screen + command list (S help lines)
//   p            re-read RDDID 0x04 / RDDST 0x09 / RDDPM 0x0A / MADCTL 0x0B
//   r<0..7>      rotation (1 or 3 expected; 5/7 if mirrored) + rotation screen
//   b<0..255>    backlight level;  bc  cycle 0 -> 30 -> 255 (3 s each)
//   kc           touch calibration: 4 corners + centre, raw x/y/z per press
//   kg           3x3 grid check with the current constants (error in px)
//   ka / kr      apply the last suggested calibration in RAM / revert to config_hw.h
//   z<label>     z logging (e.g. zlight-dry, zfirm-damp); label 1..15 of [a-z0-9-]
//   f / f1 / f2  hero font comparison: both / Font7 x2 / Font8 x1.25 at the hero rect
//   x<hz>        stress: hero-sprite redraw at 1..30 Hz;  xf<hz> full-screen fills;  x0 stop
// ---------------------------------------------------------------------------
namespace disp {

// Boot order steps 2-6 (after the banner): backlight 0, init, D line, panel ID,
// planned sprites, D line, first frame, backlight full.
void bringupBegin();

// Handles a bring-up command. Returns false when the command is not a
// bring-up command or is malformed (the caller then prints cmd_err).
bool bringupCommand(int64_t now, const char* cmd);

// Called once per loop pass, after the edge drain. Does at most one bounded
// draw step (each < LOOP_BUDGET_US) plus a touch poll when one is due.
// Returns the number of draw steps done (0 = nothing drawn this pass).
uint32_t bringupService(int64_t now);

// G-line fields since the previous call (then reset): longest single render
// operation, number of render operations, worst touch-poll lateness.
void takeStats(uint32_t& renderMaxUs, uint32_t& renderCount, uint32_t& touchLateMaxUs);

// 'd' dump lines for the display/touch state.
void dump(int64_t now);

}  // namespace disp

#else  // !OT_BRINGUP
// ---------------------------------------------------------------------------
// Main-screen renderer + the touch UI feedback (hold ring, units toast). Loop
// task only; nothing here is ever called from an ISR or a callback.
//
// Screens (ot::screenForState): MAIN in WARMUP / RUNNING / PAUSED
// (status bar, hero pace, speed, distance / moving / total cells, the HOLD TO
// STOP strip); IDLE in IDLE ("Start moving to begin", the units label in the
// hero rect, which a tap toggles, small fw text at the bottom); SUMMARY in
// SUMMARY (title, distance / total / moving, avg pace / avg speed / top speed
// from the frozen record, a countdown bar for the SUMMARY time). While a hold is in progress the hero shows the 100x100 4 bpp
// ring filling clockwise + "release to cancel" (setHold, every pass). A units
// toggle shows a 1.5 s toast in the status bar and re-renders every field. Per frame (<= RENDER_MAX_HZ = 10 Hz) the app passes ONE Frame
// (one snapshot); the renderer runs the pace/speed display filters, formats
// every field and marks the ones whose string (or dimming palette) changed.
// service() then pushes dirty fields, a bounded amount per loop pass, inside
// startWrite()/endWrite(). No allocation after begin().
// ---------------------------------------------------------------------------
namespace disp {

// One snapshot per frame: everything the screens show.
struct Frame {
  int64_t           t         = 0;                      // the loop's `now` (Workout ticked to it)
  ot::WorkoutState  state     = ot::WorkoutState::Idle;
  int32_t           speedMmps = 0;                      // Metrics.speedMmps(t): live, unrounded
  uint32_t          wkPulses  = 0;                      // workout distance (0 in IDLE/WARMUP)
  int64_t           movingUs  = 0;
  int64_t           totalUs   = 0;
  // Summary screen: the FROZEN record (never live values) while the
  // Workout is in SUMMARY, and the Workout's SUMMARY duration for the
  // countdown (same clock and threshold as the clear: record.summaryStartUs).
  bool              hasSummary = false;
  ot::SummaryRecord summary;
  int64_t           summaryUs  = 0;
  ot::Units         units     = ot::Units::Mi;
  bool              linked    = false;                  // >= 1 peer subscribed to RSC or FTMS data (watch icon)
  // The Workout's re-arm block (a hold-stop while the
  // belt kept moving; cleared once it stops). In IDLE the idle screen then
  // says "Stop the belt to start again" instead of "Start moving to begin".
  bool              rearmBlocked = false;
};

// Boot order steps 2-6 (after the banner; LED pins are already HIGH):
// setBrightness(0), init(), reserveDmaBuffers(), sprites (D lines at boot,
// after init, after the sprites), the first frame drawn completely, then
// setBrightness(BL_FULL). Returns false if the panel or a sprite failed (the
// renderer then stays inactive; logged as ERROR).
bool begin(const Frame& first);

// True when the next frame is due (FRAME_US = 1 s / RENDER_MAX_HZ apart).
bool frameDue(int64_t now);
// Take one snapshot: filters, strings, dirty marks, screen switch.
void frame(const Frame& f);
// Push dirty fields (or one screen-switch step). Returns 1 if it drew.
uint32_t service(int64_t now);

// Bench builds (sim / loopback): every frame marks every field dirty
// ("full redraws"). Never used in a release build.
void setForceRedraw(bool on);

// Hold-to-stop feedback, called every loop pass before frameDue():
// active = the gesture recognizer is holding, permille = its progress
// (0..1000). A change of `active` makes the next frame due at once, so the
// hero switches between the pace and the ring within the same pass. The ring
// itself is pushed by service() (<= 25 Hz, only when its 2 % step changed).
void setHold(int64_t now, bool active, int32_t permille);

// Units toggle: every field is re-rendered (force-dirty), the status bar
// shows the UNITS_TOAST_MS toast, and the next frame is due at once.
void unitsChanged(int64_t now);

// ---- Backlight (the ONLY brightness changes after boot) -------------------
// setBacklight(now, level) with level BL_FULL / BL_DIM / BL_OFF, called every
// loop pass with the UiPolicy level, before frameDue(). Every applied change
// goes through lcd.setBrightness() (LovyanGFX Light_PWM, nothing else drives
// LEDC on GPIO 21) and emits L,<t>,<level> (t = the instant it was applied):
//  - to OFF: brightness 0 at once; from then on service() skips every push
//    ("skip rendering, keep computing": frame() still runs the filters and
//    builds the content);
//  - FULL <-> DIM: at once (the panel already shows the current frame);
//  - from OFF to FULL/DIM (wake): every field is invalidated and the next
//    frame is due at once; the brightness is raised only when that frame
//    (and any pending screen switch) is completely on the panel, so no stale
//    frame ever flashes.
void setBacklight(int64_t now, uint8_t level);
// The level on the panel now (0..255) and whether a wake redraw is pending.
uint8_t backlight();
bool    backlightWakePending();

// Touch read for hw_touch (one getTouchRaw(): LovyanGFX's median of 7
// with its pressure check). valid = a point with size >= TOUCH_Z_MIN; x/y =
// the calibrated screen point (convertRawXY of a copy, clamped to the panel).
struct TouchRaw {
  bool     valid = false;
  uint16_t xRaw = 0, yRaw = 0, z = 0;
  int16_t  x = 0, y = 0;
};
bool touchRead(TouchRaw& out);

// G-line fields since the previous call (then reset): longest single
// service() pass that drew, number of such passes. touchLateMaxUs is always 0
// here: the touch poll lateness comes from hw_touch (touchhw::takeLateMaxUs).
void takeStats(uint32_t& renderMaxUs, uint32_t& renderCount, uint32_t& touchLateMaxUs);

// 'd' dump lines.
void dump(int64_t now);

}  // namespace disp
#endif  // OT_BRINGUP
