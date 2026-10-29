// hw_display.cpp - two halves, selected by OT_BRINGUP:
//  - OT_BRINGUP=1: the display/touch bring-up;
//  - otherwise: the main-screen renderer, at the end of this file.
//
// Bring-up half:
// Everything here runs in the loop task (setup()/loop()). Drawing is split
// into bounded steps: bringupService() does at most ONE step per loop pass,
// and no step is longer than a full-screen fill (~31 ms at 40 MHz, below
// LOOP_BUDGET_US = 40 ms). Every step is timed into the G-line render stats.
// Touch is polled every BRINGUP_TOUCH_POLL_US; the IRQ pin is never used.
// No allocation after setup(): the sprites are created once in
// bringupBegin() and never freed; every buffer here is static; the bus DMA
// flip buffers are pre-sized once (LGFX_CYD::reserveDmaBuffers) so no push
// resizes them; no float reaches a printf format (newlib's dtoa allocates).
#include "config_hw.h"

#if OT_BRINGUP

#include "hw_display.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "debug_serial.h"
#include "version.h"

namespace disp {

namespace {

// ---------------------------------------------------------------------------
// Bring-up constants (local: the shipping equivalents, e.g. TOUCH_POLL_US /
// TOUCH_RELEASE_MS and BL_FULL, live with the renderer and touch code)
// ---------------------------------------------------------------------------
constexpr uint8_t  BRINGUP_BL_FULL        = 255;
constexpr int64_t  BRINGUP_TOUCH_POLL_US  = 20000;    // 50 Hz, the shipping touch poll rate
constexpr int64_t  BRINGUP_RELEASE_US     = 120000;   // a press ends after 120 ms without a valid sample
constexpr uint16_t PRESS_MIN_SAMPLES      = 5;        // shorter presses are reported and ignored
// Bring-up's own z filter: 0 = log every point getTouchRaw() returns (the
// library already requires size > 0), so light presses are never censored
// before tools/touch_cal.py derives TOUCH_Z_MIN. Reported as touchcfg zth=.
constexpr uint16_t BRINGUP_ZTH            = 0;
constexpr uint16_t PRESS_MAX_SAMPLES      = 256;      // 5.1 s at 50 Hz; later samples only extend the stats
constexpr int64_t  BL_CYCLE_HOLD_US       = 3000000;  // 'bc': each level held 3 s
constexpr uint8_t  BL_CYCLE[]             = {0, 30, 255};
constexpr uint32_t STRESS_HZ_MAX          = 30;
constexpr int16_t  CAL_INSET              = 20;       // calibration targets: px from the edges
constexpr int16_t  GRID_INSET             = 30;       // 3x3 grid targets: px from the edges
constexpr int32_t  GRID_TOL_PX            = 15;       // acceptance: within +-15 px
constexpr uint32_t HEAP_FLOOR_FREE        = 60u * 1024u;  // heap floors
constexpr uint32_t HEAP_FLOOR_LARGEST     = 30u * 1024u;
constexpr int16_t  HERO_Y                 = 24;       // renderer layout: hero rect y 24-119

constexpr uint16_t COL_BG    = TFT_BLACK;
constexpr uint16_t COL_FG    = TFT_WHITE;
constexpr uint16_t COL_DIM   = TFT_DARKGREY;
constexpr uint16_t COL_HI    = TFT_YELLOW;
constexpr uint16_t COL_OK    = TFT_GREEN;
constexpr uint16_t COL_BAD   = TFT_RED;

LGFX_CYD gLcd;

// ---------------------------------------------------------------------------
// The renderer's sprites (its sprite budget), created here once so the
// D lines measure the real heap cost before any UI or BLE exists.
// ---------------------------------------------------------------------------
struct SpriteSpec {
  const char* name;
  int16_t w;
  int16_t h;
  uint8_t bpp;
};
constexpr SpriteSpec SPRITES[] = {
    {"hero", 320, 96, 1},  {"speed", 320, 30, 1},  {"cell0", 106, 46, 1}, {"cell1", 106, 46, 1},
    {"cell2", 106, 46, 1}, {"status", 320, 22, 4}, {"ring", 100, 100, 4},
};
constexpr size_t N_SPRITES = sizeof(SPRITES) / sizeof(SPRITES[0]);
constexpr size_t HERO = 0;
lgfx::LGFX_Sprite gSpr[N_SPRITES];
bool     gSpritesOk    = false;
uint32_t gSpriteBytes  = 0;
const char* gSpriteFail = "";

// ---------------------------------------------------------------------------
// Render / touch statistics (G line, reset by takeStats)
// ---------------------------------------------------------------------------
uint32_t gRenderMax   = 0;
uint32_t gRenderCount = 0;
uint32_t gTouchLate   = 0;

uint32_t clampUs(int64_t d) { return d < 0 ? 0u : (d > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(d)); }

void recordRender(int64_t t0) {
  const uint32_t us = clampUs(esp_timer_get_time() - t0);
  if (us > gRenderMax) gRenderMax = us;
  ++gRenderCount;
}

// ---------------------------------------------------------------------------
// Panel identification
// ---------------------------------------------------------------------------
struct PanelId {
  uint32_t rddid;   // 0x04, 3 bytes in receive order (byte i at bits 8i)
  uint32_t rddst;   // 0x09, 4 bytes
  uint32_t rddpm;   // 0x0A, 1 byte
  uint32_t madctl;  // 0x0B, 1 byte
};
PanelId gId = {0, 0, 0, 0};

uint8_t byteAt(uint32_t v, unsigned i) { return static_cast<uint8_t>((v >> (8u * i)) & 0xFFu); }

// 8-bit register read (RDDPM 0x0A, MADCTL 0x0B, ...). In the 4-wire serial
// protocol (ILI9341 datasheet p. 38 "8-bit read", ST7789 the same) these
// have NO dummy clock; only the 24-bit RDDID 0x04 and 32-bit RDDST 0x09 reads
// do (pp. 39-40). Panel_LCD::readCommand always clocks dummy_read_bits (= 1,
// the panel default kept by LGFX_CYD) first, which would drop D7, so the
// config is set to 0 dummy bits for this one read and restored.
uint32_t readReg8(uint8_t cmd) {
  auto* p = gLcd.getPanel();
  auto cfg = p->config();
  const auto saved = cfg.dummy_read_bits;
  cfg.dummy_read_bits = 0;
  p->config(cfg);
  const uint32_t v = p->readCommand(cmd, 0, 1);
  cfg.dummy_read_bits = saved;
  p->config(cfg);
  return v;
}

PanelId readPanelId() {
  PanelId id;
  auto* p = gLcd.getPanel();
  id.rddid  = p->readCommand(0x04, 0, 3);  // 24-bit read, dummy clock first
  id.rddst  = p->readCommand(0x09, 0, 4);  // 32-bit read, dummy clock first
  id.rddpm  = readReg8(0x0A);
  id.madctl = readReg8(0x0B);
  return id;
}

// RDDID first byte 0x00 = ILI9341, 0x85 = ST7789. An all-zero ID
// with an all-zero status is what a dead MISO line reads, so 0x00 also needs
// a non-zero RDDST (LovyanGFX's own detector applies the same rule).
const char* inferPanel(const PanelId& id) {
  const uint8_t b0 = byteAt(id.rddid, 0);
  if (b0 == 0x85) return "ST7789";
  if (b0 == 0x00 && (id.rddst & 0xFFFFFFu) != 0) return "ILI9341";
  if (id.rddid == 0 && id.rddst == 0) return "unknown(all-zero: MISO not read?)";
  if (id.rddid == 0xFFFFFFu) return "unknown(all-0xFF: MISO floating?)";
  return "unknown";
}

void reportPanelId(int64_t t) {
  const char* inferred = inferPanel(gId);
  const bool match = strcmp(inferred, LGFX_CYD::PANEL_NAME) == 0;
  dbg::status(t,
              "panel;rddid_04=%02x %02x %02x rddst_09=%02x %02x %02x %02x rddpm_0a=%02x madctl_0b=%02x"
              " inferred=%s built=%s match=%u",
              byteAt(gId.rddid, 0), byteAt(gId.rddid, 1), byteAt(gId.rddid, 2),
              byteAt(gId.rddst, 0), byteAt(gId.rddst, 1), byteAt(gId.rddst, 2), byteAt(gId.rddst, 3),
              byteAt(gId.rddpm, 0), byteAt(gId.madctl, 0), inferred, LGFX_CYD::PANEL_NAME, match ? 1u : 0u);
  if (!match)
    dbg::status(t, "WARNING panel mismatch: built for %s but RDDID says %s; rebuild with -D CYD_PANEL_ST7789=%u"
                   " if the inference is right",
                LGFX_CYD::PANEL_NAME, inferred, strcmp(inferred, "ST7789") == 0 ? 1u : 0u);
}

// ---------------------------------------------------------------------------
// Touch constants in use (compiled, or applied in RAM with 'ka')
// ---------------------------------------------------------------------------
struct TouchConsts {
  uint16_t xMin, xMax, yMin, yMax;
};
TouchConsts gTc = {hw::TOUCH_X_MIN, hw::TOUCH_X_MAX, hw::TOUCH_Y_MIN, hw::TOUCH_Y_MAX};
bool        gTcApplied = false;
TouchConsts gSuggest = {0, 0, 0, 0};
bool        gHaveSuggest = false;

// Forward: the LGFX_CYD instance and rotation helpers are defined below.
void reportTouchCfg(int64_t t);

void reportTouchConsts(int64_t t) {
  dbg::status(t, "touch_consts;source=%s x_min=%u x_max=%u y_min=%u y_max=%u z_min=%u provisional=%u",
              gTcApplied ? "applied(RAM)" : "config_hw.h", gTc.xMin, gTc.xMax, gTc.yMin, gTc.yMax, hw::TOUCH_Z_MIN,
              (hw::CALIBRATION_PROVISIONAL && !gTcApplied) ? 1u : 0u);
  reportTouchCfg(t);
}

// ---------------------------------------------------------------------------
// Rotation helpers: screen <-> touch-native frame (inverse of LovyanGFX
// Panel_Device::convertRawXY for 1.2.26)
// ---------------------------------------------------------------------------
uint8_t addRot(uint8_t r, uint8_t off) {
  return static_cast<uint8_t>(((r + off) & 3u) | ((r & 4u) ^ (off & 4u)));
}
uint8_t touchRot() {  // the r convertRawXY uses: rotation + panel offset + touch offset
  return addRot(addRot(static_cast<uint8_t>(gLcd.getRotation() & 7u), hw::TFT_PANEL_OFFSET_ROT), hw::TOUCH_OFFSET_ROT);
}
void screenToNative(int32_t sx, int32_t sy, int32_t& nx, int32_t& ny) {
  const uint8_t r = touchRot();
  const int32_t w = gLcd.width();
  const int32_t h = gLcd.height();
  int32_t a = sx, b = sy;
  if (r != 0) {
    const bool vflip = ((1u << r) & 0b10010110u) != 0;  // r 1,2,4,7
    if (vflip) b = (h - 1) - b;
    if (r & 2u) a = (w - 1) - a;
    if (r & 1u) {
      const int32_t tmp = a;
      a = b;
      b = tmp;
    }
  }
  nx = a;
  ny = b;
}

// touchcfg; line for tools/touch_cal.py (its input contract): printed when a
// touch mode starts and whenever the rotation or the constants change.
void reportTouchCfg(int64_t t) {
  dbg::status(t, "touchcfg;rot=%u poff=%u toff=%u w=%ld h=%ld pw=%u ph=%u x_min=%u x_max=%u y_min=%u y_max=%u zth=%u"
                 " inset=%d reff=%u",
              static_cast<unsigned>(gLcd.getRotation()), static_cast<unsigned>(hw::TFT_PANEL_OFFSET_ROT),
              static_cast<unsigned>(hw::TOUCH_OFFSET_ROT), static_cast<long>(gLcd.width()),
              static_cast<long>(gLcd.height()), static_cast<unsigned>(hw::TFT_PANEL_W),
              static_cast<unsigned>(hw::TFT_PANEL_H), gTc.xMin, gTc.xMax, gTc.yMin, gTc.yMax, BRINGUP_ZTH,
              static_cast<int>(CAL_INSET), static_cast<unsigned>(touchRot()));
}

// ---------------------------------------------------------------------------
// Press tracking
// ---------------------------------------------------------------------------
struct Sample {
  uint16_t x, y, z;
};
struct PressResult {
  int64_t  tDown;
  uint32_t durMs;
  uint16_t n;          // valid samples
  uint16_t drops;      // invalid samples bridged inside the press (< 120 ms gaps)
  uint8_t  trunc;      // more than PRESS_MAX_SAMPLES samples (stats from the first ones)
  int32_t  x, y;       // raw x/y, trimmed median
  uint16_t zMed, zMin, zMax, zAvg;
};

struct Press {
  bool     active = false;
  int64_t  tDown = 0;
  int64_t  tLastValid = 0;
  uint16_t n = 0;
  uint16_t drops = 0;
  uint16_t pendingDrops = 0;
  bool     trunc = false;
  Sample   s[PRESS_MAX_SAMPLES];
};
Press    gPress;
int64_t  gNextPoll = 0;
bool     gPollArmed = false;  // a touch screen is active and gNextPoll is a scheduled due time
uint16_t gSort[PRESS_MAX_SAMPLES];

// Median of one field over samples [lo, hi) (insertion sort into gSort).
uint16_t medianOf(const Sample* smp, uint16_t lo, uint16_t hi, uint16_t Sample::*field) {
  uint16_t n = 0;
  for (uint16_t i = lo; i < hi; ++i, ++n) {
    const uint16_t v = smp[i].*field;
    uint16_t j = n;
    while (j > 0 && gSort[j - 1] > v) {
      gSort[j] = gSort[j - 1];
      --j;
    }
    gSort[j] = v;
  }
  return n ? gSort[n / 2u] : 0;
}

// Press statistics. The first and last 2 samples (touch-down / lift-off
// settling) are dropped when the press has >= 7 stored samples; x, y and z
// are then medians, z_min/z_max/z_avg over the same samples. (The same rule
// as the tools/touch_cal.py input contract.)
void finishPress(PressResult& r) {
  const Press& p = gPress;
  r.tDown = p.tDown;
  r.durMs = static_cast<uint32_t>((p.tLastValid - p.tDown) / 1000);
  r.n     = p.n;
  r.drops = p.drops;
  r.trunc = p.trunc ? 1 : 0;
  const uint16_t stored = p.n < PRESS_MAX_SAMPLES ? p.n : PRESS_MAX_SAMPLES;
  const uint16_t lo = stored >= 7 ? 2 : 0;
  const uint16_t hi = stored >= 7 ? static_cast<uint16_t>(stored - 2) : stored;
  r.x    = medianOf(p.s, lo, hi, &Sample::x);
  r.y    = medianOf(p.s, lo, hi, &Sample::y);
  r.zMed = medianOf(p.s, lo, hi, &Sample::z);
  uint32_t zs = 0;
  r.zMin = hi > lo ? UINT16_MAX : 0;
  r.zMax = 0;
  for (uint16_t i = lo; i < hi; ++i) {
    const uint16_t z = p.s[i].z;
    zs += z;
    if (z < r.zMin) r.zMin = z;
    if (z > r.zMax) r.zMax = z;
  }
  const uint32_t m = hi > lo ? static_cast<uint32_t>(hi - lo) : 1u;
  r.zAvg = static_cast<uint16_t>((zs + m / 2u) / m);
}

// ---------------------------------------------------------------------------
// Screens and the bounded draw-step machine
// ---------------------------------------------------------------------------
enum class Screen : uint8_t { Home, Rotation, Cal, Grid, Zlog, FontBoth, FontA, FontB, StressHero, StressFull };

const char* screenName(Screen s) {
  switch (s) {
    case Screen::Home: return "home";
    case Screen::Rotation: return "rotation";
    case Screen::Cal: return "cal";
    case Screen::Grid: return "grid";
    case Screen::Zlog: return "zlog";
    case Screen::FontBoth: return "font";
    case Screen::FontA: return "font_a";
    case Screen::FontB: return "font_b";
    case Screen::StressHero: return "stress_hero";
    case Screen::StressFull: return "stress_full";
  }
  return "?";
}

Screen  gScreen = Screen::Home;
uint8_t gStage = 0;        // next draw step of gScreen
bool    gDrawPending = false;
bool    gLogSteps = true;  // S draw;... per step (one-shot screens; off during stress frames)

// Calibration / grid state
constexpr uint8_t CAL_N = 5;
constexpr uint8_t GRID_N = 9;
const char* const CAL_NAMES[CAL_N] = {"TL", "TR", "BR", "BL", "C"};
uint8_t gTarget = 0;         // current target index
int16_t gPrevTx = -1, gPrevTy = -1;
int16_t gDotX = -1, gDotY = -1;  // grid: where the last press landed
uint16_t gDotCol = 0;
struct CalPoint {
  int16_t sx, sy;
  int32_t rx, ry;
  uint16_t z;
};
CalPoint gCal[CAL_N];
int32_t  gGridMaxDx = 0, gGridMaxDy = 0;
uint8_t  gGridFails = 0;

// z logging
char     gZLabel[hw::CMD_MAX_CHARS + 1] = "";
uint16_t gZCount = 0;
uint16_t gZLabelMin = UINT16_MAX;

// Stress
uint8_t  gStressMode = 0;  // 0 off, 1 hero sprite, 2 full screen
uint32_t gStressHz = 0;
int64_t  gStressPeriod = 0;
int64_t  gStressNext = 0;
uint32_t gStressFrames = 0;
uint32_t gStressMaxUs = 0;
uint32_t gStressSkipped = 0;  // frame slots missed (the loop was late by more than a period)

// Backlight
uint8_t gBrightness = 0;
bool    gBlCycle = false;
uint8_t gBlStep = 0;
int64_t gBlNext = 0;

void setBacklight(int64_t t, uint8_t level) {
  gLcd.setBrightness(level);
  gBrightness = level;
  dbg::line(dbg::LV_K, "L,%" PRId64 ",%u", t, static_cast<unsigned>(level));
}

void cellRect(int32_t i, int32_t n, int32_t inset, int32_t extent, int16_t& out) {
  // i of n evenly spread targets from inset to extent-1-inset
  out = static_cast<int16_t>(n == 1 ? extent / 2 : inset + i * (extent - 1 - 2 * inset) / (n - 1));
}

void calTarget(uint8_t i, int16_t& x, int16_t& y) {
  const int16_t w = static_cast<int16_t>(gLcd.width());
  const int16_t h = static_cast<int16_t>(gLcd.height());
  switch (i) {
    case 0: x = CAL_INSET; y = CAL_INSET; break;
    case 1: x = static_cast<int16_t>(w - 1 - CAL_INSET); y = CAL_INSET; break;
    case 2: x = static_cast<int16_t>(w - 1 - CAL_INSET); y = static_cast<int16_t>(h - 1 - CAL_INSET); break;
    case 3: x = CAL_INSET; y = static_cast<int16_t>(h - 1 - CAL_INSET); break;
    default: x = static_cast<int16_t>(w / 2); y = static_cast<int16_t>(h / 2); break;
  }
}

void gridTarget(uint8_t i, int16_t& x, int16_t& y) {
  cellRect(i % 3, 3, GRID_INSET, gLcd.width(), x);
  cellRect(i / 3, 3, GRID_INSET, gLcd.height(), y);
}

void drawCross(int16_t x, int16_t y, uint16_t col) {
  gLcd.drawFastHLine(x - 12, y, 25, col);
  gLcd.drawFastVLine(x, y - 12, 25, col);
  gLcd.drawCircle(x, y, 8, col);
}
void eraseCross(int16_t x, int16_t y) { gLcd.fillRect(x - 12, y - 12, 25, 25, COL_BG); }

void text(const lgfx::IFont* f, float size, uint16_t fg, lgfx::textdatum_t datum, int32_t x, int32_t y, const char* s) {
  gLcd.setFont(f);
  gLcd.setTextSize(size);
  gLcd.setTextColor(fg, COL_BG);
  gLcd.setTextDatum(datum);
  gLcd.drawString(s, x, y);
}

// Hero sprite (1 bpp, palette 0 = black, 1 = white): one candidate font.
struct HeroFont {
  const char* tag;
  const lgfx::IFont* font;
  float size;
  const char* desc;
};
const HeroFont HERO_A = {"A", &lgfx::fonts::Font7, 2.0f, "Font7 x2"};
const HeroFont HERO_B = {"B", &lgfx::fonts::Font8, 1.25f, "Font8 x1.25"};
constexpr const char* HERO_SAMPLE = "12:34";

// Renders txt into the hero sprite; returns the text width. unit: small
// "/mi" bottom-right if it fits beside the digits.
int32_t renderHero(const HeroFont& hf, const char* txt, bool unit) {
  lgfx::LGFX_Sprite& s = gSpr[HERO];
  s.fillSprite(0u);
  s.setFont(hf.font);
  s.setTextSize(hf.size);
  s.setTextColor(1u, 0u);
  s.setTextDatum(lgfx::textdatum_t::middle_left);
  const int32_t w = s.textWidth(txt);
  const int32_t unitW = 28;
  int32_t x = (s.width() - w) / 2;
  if (unit && w + unitW + 4 <= s.width()) x = (s.width() - unitW - 4 - w) / 2;
  s.drawString(txt, x, s.height() / 2);
  if (unit && w + unitW + 4 <= s.width()) {
    s.setFont(&lgfx::fonts::Font2);
    s.setTextSize(1.0f);
    s.setTextDatum(lgfx::textdatum_t::bottom_right);
    s.drawString("/mi", s.width() - 1, s.height() - 4);
  }
  return w;
}

void reportHero(int64_t t, const HeroFont& hf, int32_t w, bool unit) {
  gSpr[HERO].setFont(hf.font);
  gSpr[HERO].setTextSize(hf.size);
  const int32_t h = gSpr[HERO].fontHeight();
  dbg::status(t, "font;%s font=%s text=%s w=%" PRId32 " h=%" PRId32 " fits_320x96=%u room_for_unit=%u", hf.tag, hf.desc,
              HERO_SAMPLE, w, h, (w <= 320 && h <= 96) ? 1u : 0u, (unit && w + 32 <= 320) ? 1u : 0u);
}

// Draws one step of the current screen. Returns true while more steps remain.
bool drawStep(int64_t t, Screen sc, uint8_t step) {
  const int32_t w = gLcd.width();
  const int32_t h = gLcd.height();
  char buf[96];
  if (step == 0 && sc != Screen::StressHero && sc != Screen::StressFull) {
    gLcd.fillScreen(COL_BG);  // the longest single step (~31 ms at 40 MHz)
    return true;
  }
  switch (sc) {
    case Screen::Home:
      if (step == 1) {
        text(&lgfx::fonts::Font4, 1.0f, COL_HI, lgfx::textdatum_t::top_center, w / 2, 4, "Display bring-up");
        snprintf(buf, sizeof(buf), "fw %s build %08" PRIx32 "%s", OT_FW_VERSION, hw::BUILD_ID,
                 hw::BUILD_DIRTY ? " DIRTY" : "");
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::top_left, 6, 34, buf);
        snprintf(buf, sizeof(buf), "panel %s (built %s) rot %u  %ldx%ld", inferPanel(gId), LGFX_CYD::PANEL_NAME,
                 static_cast<unsigned>(gLcd.getRotation()), static_cast<long>(w), static_cast<long>(h));
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::top_left, 6, 52, buf);
        snprintf(buf, sizeof(buf), "touch x %u..%u y %u..%u z>=%u %s", gTc.xMin, gTc.xMax, gTc.yMin, gTc.yMax,
                 hw::TOUCH_Z_MIN, gTcApplied ? "(RAM)" : (hw::CALIBRATION_PROVISIONAL ? "PROVISIONAL" : ""));
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::top_left, 6, 70, buf);
        return true;
      }
      if (step == 2) {
        static const char* const LINES[] = {
            "serial commands (Enter after each):",
            "m home    p panel id    r1 / r3 rotation",
            "b0..b255 backlight    bc cycle 0/30/255",
            "kc calibrate  kg 3x3 grid  ka apply  kr revert",
            "z<label> pressure log, e.g. zlight-dry",
            "f fonts  f1 Font7x2  f2 Font8x1.25",
            "x10 hero stress  xf5 full-screen  x0 stop",
        };
        int32_t y = 96;
        for (const char* l : LINES) {
          text(&lgfx::fonts::Font2, 1.0f, l == LINES[0] ? COL_HI : COL_FG, lgfx::textdatum_t::top_left, 6, y, l);
          y += 19;
        }
        return false;
      }
      return false;

    case Screen::Rotation:
      if (step == 1) {
        gLcd.drawRect(0, 0, w, h, COL_FG);  // 1-px border: every edge pixel must be visible
        gLcd.drawRect(1, 1, w - 2, h - 2, COL_DIM);
        gLcd.fillTriangle(w / 2, 4, w / 2 - 16, 26, w / 2 + 16, 26, COL_HI);
        text(&lgfx::fonts::Font4, 1.0f, COL_HI, lgfx::textdatum_t::top_center, w / 2, 30, "TOP");
        gLcd.fillRect(3, 3, 10, 10, COL_BAD);  // origin (0,0) marker
        text(&lgfx::fonts::Font2, 1.0f, COL_BAD, lgfx::textdatum_t::top_left, 16, 4, "0,0");
        return true;
      }
      if (step == 2) {
        gLcd.fillTriangle(w / 2, h - 5, w / 2 - 16, h - 27, w / 2 + 16, h - 27, COL_FG);
        text(&lgfx::fonts::Font4, 1.0f, COL_FG, lgfx::textdatum_t::bottom_center, w / 2, h - 30, "BOTTOM");
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_left, 6, h / 2, "< LEFT");
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_right, w - 7, h / 2, "RIGHT >");
        return true;
      }
      if (step == 3) {
        snprintf(buf, sizeof(buf), "rotation %u", static_cast<unsigned>(gLcd.getRotation()));
        text(&lgfx::fonts::Font4, 1.0f, COL_HI, lgfx::textdatum_t::middle_center, w / 2, h / 2 - 34, buf);
        snprintf(buf, sizeof(buf), "%ldx%ld  MADCTL 0x%02x", static_cast<long>(w), static_cast<long>(h),
                 byteAt(gId.madctl, 0));
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_center, w / 2, h / 2 - 8, buf);
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_center, w / 2, h / 2 + 10,
             "TOP must be the enclosure top");
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_center, w / 2, h / 2 + 28,
             "send r1 / r3; mirrored text: r5 / r7");
        return false;
      }
      return false;

    case Screen::Cal:
    case Screen::Grid: {
      const bool cal = sc == Screen::Cal;
      const uint8_t n = cal ? CAL_N : GRID_N;
      if (step == 1) {
        text(&lgfx::fonts::Font2, 1.0f, COL_HI, lgfx::textdatum_t::middle_center, w / 2, h / 2 - 40,
             cal ? "CALIBRATION: press the + firmly" : "GRID CHECK: press each + firmly");
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_center, w / 2, h / 2 - 22,
             "hold ~1 s, lift, wait for the next +");
        return true;
      }
      // step 2: target update (also used after each press)
      if (gPrevTx >= 0) eraseCross(gPrevTx, gPrevTy);
      gPrevTx = -1;
      if (!cal && gDotX >= 0) gLcd.fillCircle(gDotX, gDotY, 3, gDotCol);  // keep the last landing dot
      if (gTarget < n) {
        int16_t x = 0, y = 0;
        if (cal) calTarget(gTarget, x, y);
        else gridTarget(gTarget, x, y);
        drawCross(x, y, COL_HI);
        gPrevTx = x;
        gPrevTy = y;
        snprintf(buf, sizeof(buf), " %s %u/%u ", cal ? CAL_NAMES[gTarget] : "grid", gTarget + 1u, static_cast<unsigned>(n));
      } else {
        snprintf(buf, sizeof(buf), " done - see serial ");
      }
      text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::middle_center, w / 2, h / 2 + 30, buf);
      return false;
    }

    case Screen::Zlog:
      if (step == 1) {
        snprintf(buf, sizeof(buf), "PRESSURE LOG: %s", gZLabel);
        text(&lgfx::fonts::Font2, 1.0f, COL_HI, lgfx::textdatum_t::top_center, w / 2, 8, buf);
        text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::top_center, w / 2, 28,
             "press inside the box, ~1 s each, 5+ times");
        gLcd.drawRect(60, 60, w - 120, h - 100, COL_FG);
        return true;
      }
      snprintf(buf, sizeof(buf), " presses: %u ", static_cast<unsigned>(gZCount));
      text(&lgfx::fonts::Font2, 1.0f, COL_FG, lgfx::textdatum_t::bottom_center, w / 2, h - 8, buf);
      return false;

    case Screen::FontBoth:
      // labels y 0-15 / 116-131, heroes y 16-111 / 134-229 (both sprites are the 320x96 hero)
      if (step == 1) {
        text(&lgfx::fonts::Font2, 1.0f, COL_HI, lgfx::textdatum_t::top_left, 2, 0, "A  Font7 x2");
        const int32_t tw = renderHero(HERO_A, HERO_SAMPLE, false);
        gSpr[HERO].pushSprite(&gLcd, 0, 16);
        reportHero(t, HERO_A, tw, true);
        return true;
      }
      if (step == 2) {
        text(&lgfx::fonts::Font2, 1.0f, COL_HI, lgfx::textdatum_t::top_left, 2, 116, "B  Font8 x1.25");
        const int32_t tw = renderHero(HERO_B, HERO_SAMPLE, false);
        gSpr[HERO].pushSprite(&gLcd, 0, 134);
        reportHero(t, HERO_B, tw, true);
        return false;
      }
      return false;

    case Screen::FontA:
    case Screen::FontB: {
      const HeroFont& hf = sc == Screen::FontA ? HERO_A : HERO_B;
      if (step == 1) {
        const int32_t tw = renderHero(hf, HERO_SAMPLE, true);
        gSpr[HERO].pushSprite(&gLcd, 0, HERO_Y);  // the renderer's hero rect (y 24-119), exactly as it pushes it
        reportHero(t, hf, tw, true);
        return true;
      }
      snprintf(buf, sizeof(buf), "%s  %s  (hero rect y 24-119, photo from 1.5 m)", hf.tag, hf.desc);
      text(&lgfx::fonts::Font2, 1.0f, COL_DIM, lgfx::textdatum_t::bottom_center, w / 2, h - 4, buf);
      return false;
    }

    case Screen::StressHero:
    case Screen::StressFull:
      if (step == 0) {
        gLcd.fillScreen(COL_BG);
        return true;
      }
      snprintf(buf, sizeof(buf), "STRESS %s %lu Hz   x0 stops", sc == Screen::StressHero ? "hero" : "full",
               static_cast<unsigned long>(gStressHz));
      text(&lgfx::fonts::Font2, 1.0f, COL_HI, lgfx::textdatum_t::bottom_center, w / 2, h - 8, buf);
      return false;
  }
  return false;
}

// Any screen change ends a running stress mode (with its summary line).
void stopStress(int64_t t) {
  if (gStressMode == 0) return;
  dbg::status(t, "stress;stop mode=%s hz=%" PRIu32 " frames=%" PRIu32 " skipped=%" PRIu32 " max_us=%" PRIu32,
              gStressMode == 1 ? "hero" : "full", gStressHz, gStressFrames, gStressSkipped, gStressMaxUs);
  gStressMode = 0;
}

void startScreen(int64_t t, Screen sc) {
  stopStress(t);
  gScreen = sc;
  gStage = 0;
  gDrawPending = true;
  gPrevTx = -1;
  gDotX = -1;
  gPress.active = false;  // a press in progress belongs to the old screen
  dbg::status(t, "screen;%s", screenName(sc));
}

// Runs one draw step (timed). Returns 1 if a step ran.
uint32_t runDrawStep(int64_t now) {
  if (!gDrawPending) return 0;
  const int64_t t0 = esp_timer_get_time();
  const uint8_t step = gStage;
  const bool more = drawStep(now, gScreen, step);
  recordRender(t0);
  const uint32_t us = clampUs(esp_timer_get_time() - t0);
  if (gLogSteps) dbg::status(now, "draw;screen=%s step=%u us=%" PRIu32, screenName(gScreen), static_cast<unsigned>(step), us);
  if (more) ++gStage;
  else gDrawPending = false;
  return 1;
}

// Redraw only the target/progress part of a touch screen (its step 2). If
// the screen's own first draw is still running, it ends with step 2 anyway.
void requestTargetRedraw() {
  if (gDrawPending && gStage < 2) return;
  gStage = 2;
  gDrawPending = true;
}

// ---------------------------------------------------------------------------
// Calibration fit
// ---------------------------------------------------------------------------
// Least squares raw = a + b * native over the 5 points, extrapolated to the
// native panel edges (x 0..239, y 0..319), which is what LovyanGFX's
// x_min/x_max/y_min/y_max mean (Panel_Device::touchCalibrate()).
void fitAxis(const double* nat, const double* raw, uint8_t n, double& a, double& b, double& corr) {
  double sn = 0, sr = 0, snn = 0, srr = 0, snr = 0;
  for (uint8_t i = 0; i < n; ++i) {
    sn += nat[i];
    sr += raw[i];
    snn += nat[i] * nat[i];
    srr += raw[i] * raw[i];
    snr += nat[i] * raw[i];
  }
  const double cov = snr - sn * sr / n;
  const double vn = snn - sn * sn / n;
  const double vr = srr - sr * sr / n;
  b = vn != 0 ? cov / vn : 0;
  a = (sr - b * sn) / n;
  corr = (vn > 0 && vr > 0) ? cov / sqrt(vn * vr) : 0;
}

uint16_t clamp12(double v) {
  if (v < 0) return 0;
  if (v > 4095) return 4095;
  return static_cast<uint16_t>(v + 0.5);
}

void finishCalibration(int64_t t) {
  double nx[CAL_N], ny[CAL_N], rx[CAL_N], ry[CAL_N];
  int32_t invErr = 0;
  // Native-frame span, as in Panel_Device::setCalibrate (swapped when the touch offset is odd).
  const int32_t pw = (hw::TOUCH_OFFSET_ROT & 1u) ? hw::TFT_PANEL_H : hw::TFT_PANEL_W;
  const int32_t ph = (hw::TOUCH_OFFSET_ROT & 1u) ? hw::TFT_PANEL_W : hw::TFT_PANEL_H;
  for (uint8_t i = 0; i < CAL_N; ++i) {
    int32_t a = 0, b = 0;
    screenToNative(gCal[i].sx, gCal[i].sy, a, b);
    nx[i] = a;
    ny[i] = b;
    rx[i] = gCal[i].rx;
    ry[i] = gCal[i].ry;
    // Self-check of screenToNative against LovyanGFX's own forward mapping:
    // map the measured raw point with the constants in use, invert, and
    // compare with the same constants' linear model in the native frame.
    lgfx::touch_point_t tp;
    tp.x = static_cast<int16_t>(gCal[i].rx);
    tp.y = static_cast<int16_t>(gCal[i].ry);
    gLcd.convertRawXY(&tp, 1);
    int32_t ia = 0, ib = 0;
    screenToNative(tp.x, tp.y, ia, ib);
    const double ex = (gTc.xMax != gTc.xMin) ? (rx[i] - gTc.xMin) * (pw - 1) / static_cast<double>(gTc.xMax - gTc.xMin) : 0;
    const double ey = (gTc.yMax != gTc.yMin) ? (ry[i] - gTc.yMin) * (ph - 1) / static_cast<double>(gTc.yMax - gTc.yMin) : 0;
    const int32_t dx = static_cast<int32_t>(ia - ex + (ia >= ex ? 0.5 : -0.5));
    const int32_t dy = static_cast<int32_t>(ib - ey + (ib >= ey ? 0.5 : -0.5));
    const int32_t e = (dx < 0 ? -dx : dx) > (dy < 0 ? -dy : dy) ? (dx < 0 ? -dx : dx) : (dy < 0 ? -dy : dy);
    if (e > invErr) invErr = e;
  }
  double ax = 0, bx = 0, cxx = 0, ay = 0, by = 0, cyy = 0, cxy = 0, cyx = 0, d0 = 0, d1 = 0;
  fitAxis(nx, rx, CAL_N, ax, bx, cxx);
  fitAxis(ny, ry, CAL_N, ay, by, cyy);
  fitAxis(ny, rx, CAL_N, d0, d1, cxy);  // raw X against native y: must be weak
  fitAxis(nx, ry, CAL_N, d0, d1, cyx);
  gSuggest.xMin = clamp12(ax);
  gSuggest.xMax = clamp12(ax + bx * (pw - 1));
  gSuggest.yMin = clamp12(ay);
  gSuggest.yMax = clamp12(ay + by * (ph - 1));
  gHaveSuggest = true;
  // Residual of the fit, in native px.
  double resMax = 0;
  for (uint8_t i = 0; i < CAL_N; ++i) {
    const double px = bx != 0 ? (rx[i] - ax) / bx : 0;
    const double py = by != 0 ? (ry[i] - ay) / by : 0;
    const double ex = px > nx[i] ? px - nx[i] : nx[i] - px;
    const double ey = py > ny[i] ? py - ny[i] : ny[i] - py;
    if (ex > resMax) resMax = ex;
    if (ey > resMax) resMax = ey;
  }
  const bool swapped = (cxy * cxy > cxx * cxx) || (cyx * cyx > cyy * cyy);
  // Correlations as integer thousandths: no floating-point conversion reaches
  // vsnprintf after setup() (full newlib _dtoa_r heap-allocates its Bigints
  // on first use in a task).
  const long corrXMilli = lround(cxx * 1000.0);
  const long corrYMilli = lround(cyy * 1000.0);
  dbg::status(t, "cal;suggest x_min=%u x_max=%u y_min=%u y_max=%u fit_resid_max_px=%d corr_x_milli=%ld"
                 " corr_y_milli=%ld axes_swapped=%u inverse_check_max_px=%" PRId32 " rotation=%u",
              gSuggest.xMin, gSuggest.xMax, gSuggest.yMin, gSuggest.yMax, static_cast<int>(resMax + 0.5), corrXMilli,
              corrYMilli,
              swapped ? 1u : 0u, invErr, static_cast<unsigned>(gLcd.getRotation()));
  if (swapped)
    dbg::status(t, "WARNING cal: raw X follows the native y axis (or Y follows x); x/y ranges cannot express that."
                   " Check TOUCH_OFFSET_ROT / the panel switch before hardcoding");
  dbg::status(t, "cal;next: send ka (apply in RAM) then kg (3x3 grid). If the grid passes, copy the four values"
                 " into config_hw.h TOUCH_X_MIN/X_MAX/Y_MIN/Y_MAX and rebuild");
}

void onPressDone(int64_t t, const PressResult& r) {
  if (gScreen == Screen::Zlog) {
    if (r.n < PRESS_MIN_SAMPLES) {
      dbg::status(t, "zlog;ignored short press n=%u (need %u)", r.n, PRESS_MIN_SAMPLES);
      return;
    }
    ++gZCount;
    if (r.zMin < gZLabelMin) gZLabelMin = r.zMin;
    {
      // touchcal; line: press=/finger= when the label is <light|normal|firm>-<dry|damp>
      char press[8] = "", finger[8] = "";
      const char* dash = strchr(gZLabel, '-');
      if (dash != nullptr && static_cast<size_t>(dash - gZLabel) < sizeof(press) && strlen(dash + 1) < sizeof(finger)) {
        memcpy(press, gZLabel, static_cast<size_t>(dash - gZLabel));
        press[dash - gZLabel] = '\0';
        strcpy(finger, dash + 1);
      }
      const bool known = (strcmp(press, "light") == 0 || strcmp(press, "normal") == 0 || strcmp(press, "firm") == 0) &&
                         (strcmp(finger, "dry") == 0 || strcmp(finger, "damp") == 0);
      lgfx::touch_point_t cp;  // sx/sy: convertRawXY of a copy of the same raw point
      cp.x = static_cast<int16_t>(r.x);
      cp.y = static_cast<int16_t>(r.y);
      gLcd.convertRawXY(&cp, 1);
      if (known)
        dbg::status(t, "touchcal;tgt=z press=%s finger=%s x=%" PRId32 " y=%" PRId32 " z=%u zmin=%u n=%u sx=%d sy=%d",
                    press, finger, r.x, r.y, r.zMed, r.zMin, r.n, cp.x, cp.y);
      else
        dbg::status(t, "touchcal;tgt=z_%s x=%" PRId32 " y=%" PRId32 " z=%u zmin=%u n=%u sx=%d sy=%d", gZLabel, r.x, r.y,
                    r.zMed, r.zMin, r.n, cp.x, cp.y);
    }
    dbg::status(t, "zlog;label=%s k=%u n=%u dur_ms=%" PRIu32 " drops=%u z_med=%u z_min=%u z_max=%u z_avg=%u"
                   " label_min_z=%u x=%" PRId32 " y=%" PRId32 " trunc=%u",
                gZLabel, gZCount, r.n, r.durMs, r.drops, r.zMed, r.zMin, r.zMax, r.zAvg, gZLabelMin, r.x, r.y, r.trunc);
    requestTargetRedraw();
    return;
  }
  const bool cal = gScreen == Screen::Cal;
  const uint8_t n = cal ? CAL_N : GRID_N;
  if (gTarget >= n) return;
  if (r.n < PRESS_MIN_SAMPLES) {
    dbg::status(t, "%s;ignored short press n=%u (need %u): press again", cal ? "cal" : "grid", r.n, PRESS_MIN_SAMPLES);
    return;
  }
  int16_t tx = 0, ty = 0;
  if (cal) calTarget(gTarget, tx, ty);
  else gridTarget(gTarget, tx, ty);
  {
    // touchcal; line (tools/touch_cal.py contract). sx/sy = convertRawXY of
    // a COPY of the same raw point, with the constants in use.
    static const char* const CAL_TGT[CAL_N] = {"tl", "tr", "br", "bl", "c"};
    lgfx::touch_point_t cp;
    cp.x = static_cast<int16_t>(r.x);
    cp.y = static_cast<int16_t>(r.y);
    gLcd.convertRawXY(&cp, 1);
    char tgt[12];
    if (cal) snprintf(tgt, sizeof(tgt), "%s", CAL_TGT[gTarget]);
    else snprintf(tgt, sizeof(tgt), "g%u", static_cast<unsigned>(gTarget));
    dbg::status(t, "touchcal;tgt=%s tx=%d ty=%d x=%" PRId32 " y=%" PRId32 " z=%u zmin=%u n=%u sx=%d sy=%d", tgt, tx, ty,
                r.x, r.y, r.zMed, r.zMin, r.n, cp.x, cp.y);
  }
  if (cal) {
    gCal[gTarget] = {tx, ty, r.x, r.y, r.zMed};
    dbg::status(t, "cal;i=%u name=%s sx=%d sy=%d raw_x=%" PRId32 " raw_y=%" PRId32 " z_med=%u z_min=%u n=%u dur_ms=%" PRIu32
                   " drops=%u",
                gTarget, CAL_NAMES[gTarget], tx, ty, r.x, r.y, r.zMed, r.zMin, r.n, r.durMs, r.drops);
    ++gTarget;
    if (gTarget == CAL_N) finishCalibration(t);
  } else {
    lgfx::touch_point_t tp;
    tp.x = static_cast<int16_t>(r.x);
    tp.y = static_cast<int16_t>(r.y);
    gLcd.convertRawXY(&tp, 1);  // the shipping conversion with the constants in use
    const int32_t dx = tp.x - tx;
    const int32_t dy = tp.y - ty;
    const int32_t adx = dx < 0 ? -dx : dx;
    const int32_t ady = dy < 0 ? -dy : dy;
    const bool ok = adx <= GRID_TOL_PX && ady <= GRID_TOL_PX;
    if (adx > gGridMaxDx) gGridMaxDx = adx;
    if (ady > gGridMaxDy) gGridMaxDy = ady;
    if (!ok) ++gGridFails;
    {
      const int64_t t0 = esp_timer_get_time();
      gDotX = tp.x;
      gDotY = tp.y;
      gDotCol = ok ? COL_OK : COL_BAD;
      gLcd.fillCircle(gDotX, gDotY, 3, gDotCol);  // where the press landed
      recordRender(t0);
    }
    dbg::status(t, "grid;i=%u tx=%d ty=%d raw_x=%" PRId32 " raw_y=%" PRId32 " got_x=%d got_y=%d dx=%" PRId32
                   " dy=%" PRId32 " ok=%u z_med=%u n=%u",
                gTarget, tx, ty, r.x, r.y, tp.x, tp.y, dx, dy, ok ? 1u : 0u, r.zMed, r.n);
    ++gTarget;
    if (gTarget == GRID_N)
      dbg::status(t, "grid;done max_abs_dx=%" PRId32 " max_abs_dy=%" PRId32 " fails=%u tol_px=%" PRId32
                     " pass=%u consts=%s x_min=%u x_max=%u y_min=%u y_max=%u rotation=%u",
                  gGridMaxDx, gGridMaxDy, gGridFails, GRID_TOL_PX, gGridFails == 0 ? 1u : 0u,
                  gTcApplied ? "applied(RAM)" : "config_hw.h", gTc.xMin, gTc.xMax, gTc.yMin, gTc.yMax,
                  static_cast<unsigned>(gLcd.getRotation()));
  }
  requestTargetRedraw();
}

bool touchScreen() { return gScreen == Screen::Cal || gScreen == Screen::Grid || gScreen == Screen::Zlog; }

void pollTouch(int64_t now) {
  if (!touchScreen()) {
    gPollArmed = false;
    return;
  }
  if (!gPollArmed) {
    // First poll of a touch screen: poll now, but no due time was scheduled
    // before it, so there is no lateness to record.
    gPollArmed = true;
    gNextPoll = now + BRINGUP_TOUCH_POLL_US;
  } else {
    if (now < gNextPoll) return;
    const uint32_t late = clampUs(now - gNextPoll);
    if (late > gTouchLate) gTouchLate = late;
    gNextPoll += BRINGUP_TOUCH_POLL_US;
    if (gNextPoll <= now) gNextPoll = now + BRINGUP_TOUCH_POLL_US;
  }

  lgfx::touch_point_t tp;
  const bool valid = gLcd.getTouchRaw(&tp, 1) > 0 && tp.size > BRINGUP_ZTH;
  Press& p = gPress;
  if (valid) {
    const Sample s = {static_cast<uint16_t>(tp.x), static_cast<uint16_t>(tp.y), tp.size};
    if (!p.active) {
      p.active = true;
      p.tDown = now;
      p.n = 0;
      p.drops = 0;
      p.pendingDrops = 0;
      p.trunc = false;
    }
    p.drops = static_cast<uint16_t>(p.drops + p.pendingDrops);
    p.pendingDrops = 0;
    if (p.n < PRESS_MAX_SAMPLES) p.s[p.n] = s;
    else p.trunc = true;
    if (p.n < UINT16_MAX) ++p.n;
    p.tLastValid = now;
    dbg::line(dbg::LV_EDGE, "T,%" PRId64 ",%u,%u,%u,%s", now, s.x, s.y, s.z, p.n == 1 ? "down" : "sample");
    return;
  }
  if (!p.active) return;
  if (p.pendingDrops < UINT16_MAX) ++p.pendingDrops;
  if (now - p.tLastValid < BRINGUP_RELEASE_US) return;
  p.active = false;
  PressResult r;
  finishPress(r);
  dbg::line(dbg::LV_EDGE, "T,%" PRId64 ",%" PRId32 ",%" PRId32 ",%u,up", now, r.x, r.y, r.zMed);
  onPressDone(now, r);
}

// ---------------------------------------------------------------------------
// Stress frames
// ---------------------------------------------------------------------------
uint32_t stressService(int64_t now) {
  if (gStressMode == 0 || gDrawPending || now < gStressNext) return 0;
  // Schedule: one frame per period; a late loop skips slots, never bursts.
  gStressNext += gStressPeriod;
  while (gStressNext <= now) {
    gStressNext += gStressPeriod;
    ++gStressSkipped;
  }
  const int64_t t0 = esp_timer_get_time();
  if (gStressMode == 1) {
    char txt[8];
    const uint32_t f = gStressFrames;
    snprintf(txt, sizeof(txt), "%02lu:%02lu", static_cast<unsigned long>((f / 60u) % 100u),
             static_cast<unsigned long>(f % 60u));
    renderHero(HERO_A, txt, true);
    gSpr[HERO].pushSprite(&gLcd, 0, HERO_Y);
  } else {
    gLcd.fillScreen((gStressFrames & 1u) ? TFT_NAVY : TFT_MAROON);
  }
  recordRender(t0);
  const uint32_t us = clampUs(esp_timer_get_time() - t0);
  if (us > gStressMaxUs) gStressMaxUs = us;
  ++gStressFrames;
  return 1;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
bool parseU32Exact(const char* s, uint32_t& v) {
  const char* end = nullptr;
  return dbg::parse::u32(s, v, &end) && *end == '\0';
}

void cmdOk(int64_t t, const char* c) { dbg::status(t, "cmd;%s", c); }

void startTouchMode(int64_t t, Screen sc) {
  gTarget = 0;
  gGridMaxDx = gGridMaxDy = 0;
  gGridFails = 0;
  startScreen(t, sc);
  reportTouchConsts(t);
  dbg::status(t, "%s;start rotation=%u targets=%u (press each + ~1 s; T lines at verbosity 2)",
              sc == Screen::Cal ? "cal" : "grid", static_cast<unsigned>(gLcd.getRotation()),
              sc == Screen::Cal ? static_cast<unsigned>(CAL_N) : static_cast<unsigned>(GRID_N));
}

void help(int64_t t) {
  dbg::status(t, "help;m home  p panel-id  r<0-7> rotation  b<0-255> backlight  bc cycle 0/30/255");
  dbg::status(t, "help;kc calibrate(4 corners+centre)  kg 3x3 grid check  ka apply suggestion(RAM)  kr revert");
  dbg::status(t, "help;z<label> pressure log (e.g. zlight-dry)  f / f1 / f2 hero fonts  x<hz> / xf<hz> stress  x0 stop");
}

}  // namespace

// ---------------------------------------------------------------------------
void bringupBegin() {
  int64_t t = esp_timer_get_time();
  dbg::status(t, "bringup;display/touch bring-up: panel_build=%s rotation=%u spi_write_hz=%" PRIu32
                 " spi_read_hz=%" PRIu32 " bl_pin=%d bl_hz=%" PRIu32 " touch_sw_spi=1 touch_irq=unused calibration=%s",
              LGFX_CYD::PANEL_NAME, static_cast<unsigned>(hw::DISPLAY_ROTATION), hw::TFT_SPI_WRITE_HZ,
              hw::TFT_SPI_READ_HZ, static_cast<int>(hw::BL_PWM_PIN), hw::BL_PWM_HZ,
              hw::CALIBRATION_PROVISIONAL ? "PROVISIONAL" : "measured");
  dbg::status(t, "heap;point=before_lcd_init");
  dbg::heap(t);

  // Boot order: LED pins are already HIGH (app_setup, first).
  gLcd.setBrightness(0);  // 2. init() then keeps the backlight at 0
  gBrightness = 0;
  const int64_t ti = esp_timer_get_time();
  const bool ok = gLcd.init();  // 3.
  const bool dmaOk = gLcd.reserveDmaBuffers();  // right after init: no draw may resize them later
  gLcd.setRotation(hw::DISPLAY_ROTATION);
  t = esp_timer_get_time();
  dbg::status(t, "display;init ok=%u us=%" PRIu32 " w=%ld h=%ld rotation=%u dma_flip_bytes=%" PRIu32 "x2 dma_ok=%u",
              ok ? 1u : 0u, clampUs(t - ti), static_cast<long>(gLcd.width()), static_cast<long>(gLcd.height()),
              static_cast<unsigned>(gLcd.getRotation()), LGFX_CYD::DMA_FLIP_BYTES, dmaOk ? 1u : 0u);
  dbg::status(t, "heap;point=after_lcd_init");
  dbg::heap(t);

  gId = readPanelId();
  reportPanelId(esp_timer_get_time());
  reportTouchConsts(esp_timer_get_time());

  // 4. the renderer's sprites, once, never freed
  gSpritesOk = true;
  gSpriteBytes = 0;
  for (size_t i = 0; i < N_SPRITES; ++i) {
    gSpr[i].setPsram(false);
    gSpr[i].setColorDepth(SPRITES[i].bpp);
    if (gSpr[i].createSprite(SPRITES[i].w, SPRITES[i].h) == nullptr) {
      gSpritesOk = false;
      gSpriteFail = SPRITES[i].name;
      break;
    }
    gSpriteBytes += gSpr[i].bufferLength();
  }
  t = esp_timer_get_time();
  dbg::status(t, "sprites;ok=%u n=%u bytes=%" PRIu32 " failed=%s set=hero320x96x1 speed320x30x1 cell106x46x1*3"
                 " status320x22x4 ring100x100x4",
              gSpritesOk ? 1u : 0u, static_cast<unsigned>(N_SPRITES), gSpriteBytes, gSpritesOk ? "-" : gSpriteFail);
  dbg::status(t, "heap;point=after_sprites");
  dbg::heap(t);
  {
    const uint32_t fr = static_cast<uint32_t>(esp_get_free_heap_size());
    const uint32_t lg = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    dbg::status(t, "heap;floors free=%" PRIu32 ">=%" PRIu32 " largest=%" PRIu32 ">=%" PRIu32
                   " ok=%u (before BLE; BLE takes its share later)",
                fr, HEAP_FLOOR_FREE, lg, HEAP_FLOOR_LARGEST, (fr >= HEAP_FLOOR_FREE && lg >= HEAP_FLOOR_LARGEST) ? 1u : 0u);
  }

  // 5. first frame (setup: drawn completely, step by step)
  gScreen = Screen::Home;
  gStage = 0;
  gDrawPending = true;
  while (gDrawPending) runDrawStep(esp_timer_get_time());

  // 6. backlight full
  setBacklight(esp_timer_get_time(), BRINGUP_BL_FULL);
  help(esp_timer_get_time());
}

bool bringupCommand(int64_t now, const char* c) {
  const size_t len = strlen(c);
  switch (c[0]) {
    case 'm':
      if (len != 1) return false;
      cmdOk(now, c);
      startScreen(now, Screen::Home);
      help(now);
      return true;

    case 'p':
      if (len != 1) return false;
      cmdOk(now, c);
      gId = readPanelId();
      reportPanelId(now);
      return true;

    case 'r': {
      if (len != 2 || c[1] < '0' || c[1] > '7') return false;
      cmdOk(now, c);
      gLcd.setRotation(static_cast<uint8_t>(c[1] - '0'));
      gId.madctl = readReg8(0x0B);
      dbg::status(now, "rotation;n=%u w=%ld h=%ld madctl_0b=%02x compiled=%u (set DISPLAY_ROTATION in config_hw.h)",
                  static_cast<unsigned>(gLcd.getRotation()), static_cast<long>(gLcd.width()),
                  static_cast<long>(gLcd.height()), byteAt(gId.madctl, 0), static_cast<unsigned>(hw::DISPLAY_ROTATION));
      reportTouchCfg(now);
      startScreen(now, Screen::Rotation);
      return true;
    }

    case 'b': {
      if (strcmp(c, "bc") == 0) {
        cmdOk(now, c);
        gBlCycle = true;
        gBlStep = 0;
        gBlNext = now + BL_CYCLE_HOLD_US;
        dbg::status(now, "backlight;cycle start levels=0/30/255 hold_ms=%" PRId64, BL_CYCLE_HOLD_US / 1000);
        setBacklight(now, BL_CYCLE[0]);
        return true;
      }
      uint32_t v = 0;
      if (!parseU32Exact(c + 1, v) || v > 255) return false;
      cmdOk(now, c);
      if (gBlCycle) dbg::status(now, "backlight;cycle cancelled");
      gBlCycle = false;
      setBacklight(now, static_cast<uint8_t>(v));
      return true;
    }

    case 'k':
      if (len != 2) return false;
      if (c[1] == 'c') {
        cmdOk(now, c);
        startTouchMode(now, Screen::Cal);
        return true;
      }
      if (c[1] == 'g') {
        cmdOk(now, c);
        startTouchMode(now, Screen::Grid);
        return true;
      }
      if (c[1] == 'a') {
        if (!gHaveSuggest) {
          dbg::status(now, "cmd_err;%s (no calibration yet: run kc first)", c);
          return true;
        }
        cmdOk(now, c);
        gTc = gSuggest;
        gTcApplied = true;
        gLcd.setTouchRanges(gTc.xMin, gTc.xMax, gTc.yMin, gTc.yMax);
        reportTouchConsts(now);
        return true;
      }
      if (c[1] == 'r') {
        cmdOk(now, c);
        gTc = {hw::TOUCH_X_MIN, hw::TOUCH_X_MAX, hw::TOUCH_Y_MIN, hw::TOUCH_Y_MAX};
        gTcApplied = false;
        gLcd.setTouchRanges(gTc.xMin, gTc.xMax, gTc.yMin, gTc.yMax);
        reportTouchConsts(now);
        return true;
      }
      return false;

    case 'z': {
      if (len < 2) return false;
      for (size_t i = 1; i < len; ++i) {
        const char ch = c[i];
        if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-')) return false;
      }
      cmdOk(now, c);
      memcpy(gZLabel, c + 1, len);  // includes the NUL
      gZCount = 0;
      gZLabelMin = UINT16_MAX;
      startScreen(now, Screen::Zlog);
      reportTouchCfg(now);
      dbg::status(now, "zlog;start label=%s zth=%u touch_z_min=%u (z = getTouchRaw size = median raw z >> 8)", gZLabel,
                  BRINGUP_ZTH, hw::TOUCH_Z_MIN);
      return true;
    }

    case 'f':
      if ((strcmp(c, "f") == 0 || strcmp(c, "f1") == 0 || strcmp(c, "f2") == 0) && !gSpritesOk) {
        dbg::status(now, "cmd_err;%s (sprites not created)", c);
        return true;
      }
      if (strcmp(c, "f") == 0) {
        cmdOk(now, c);
        startScreen(now, Screen::FontBoth);
        return true;
      }
      if (strcmp(c, "f1") == 0 || strcmp(c, "f2") == 0) {
        cmdOk(now, c);
        startScreen(now, c[1] == '1' ? Screen::FontA : Screen::FontB);
        return true;
      }
      return false;

    case 'x': {
      const bool full = c[1] == 'f';
      uint32_t hz = 0;
      if (!parseU32Exact(c + (full ? 2 : 1), hz) || hz > STRESS_HZ_MAX) return false;
      if (hz != 0 && !full && !gSpritesOk) {
        dbg::status(now, "cmd_err;%s (sprites not created)", c);
        return true;
      }
      cmdOk(now, c);
      if (hz == 0) {
        if (gStressMode != 0) startScreen(now, Screen::Home);  // prints stress;stop
        return true;
      }
      startScreen(now, full ? Screen::StressFull : Screen::StressHero);  // prints stress;stop for a running one
      gStressMode = full ? 2 : 1;
      gStressHz = hz;
      gStressPeriod = 1000000 / static_cast<int64_t>(hz);
      gStressNext = now;
      gStressFrames = 0;
      gStressMaxUs = 0;
      gStressSkipped = 0;
      dbg::status(now, "stress;start mode=%s hz=%" PRIu32 " period_us=%" PRId64, full ? "full" : "hero", hz,
                  gStressPeriod);
      return true;
    }

    default:
      return false;
  }
}

uint32_t bringupService(int64_t now) {
  // Backlight cycle
  if (gBlCycle && now >= gBlNext) {
    ++gBlStep;
    if (gBlStep >= sizeof(BL_CYCLE)) {
      gBlCycle = false;
      dbg::status(now, "backlight;cycle done level=%u", static_cast<unsigned>(gBrightness));
    } else {
      setBacklight(now, BL_CYCLE[gBlStep]);
      gBlNext = now + BL_CYCLE_HOLD_US;
    }
  }
  pollTouch(now);
  if (gDrawPending) return runDrawStep(now);
  return stressService(now);
}

void takeStats(uint32_t& renderMaxUs, uint32_t& renderCount, uint32_t& touchLateMaxUs) {
  renderMaxUs = gRenderMax;
  renderCount = gRenderCount;
  touchLateMaxUs = gTouchLate;
  gRenderMax = 0;
  gRenderCount = 0;
  gTouchLate = 0;
}

void dump(int64_t now) {
  dbg::status(now,
              "dump;display panel_build=%s inferred=%s rotation=%u w=%ld h=%ld brightness=%u screen=%s draw_pending=%u"
              " sprites_ok=%u sprite_bytes=%" PRIu32 " render_max_us=%" PRIu32 " render_count=%" PRIu32
              " touch_late_max_us=%" PRIu32 " (since the last G line)",
              LGFX_CYD::PANEL_NAME, inferPanel(gId), static_cast<unsigned>(gLcd.getRotation()),
              static_cast<long>(gLcd.width()), static_cast<long>(gLcd.height()), static_cast<unsigned>(gBrightness),
              screenName(gScreen), gDrawPending ? 1u : 0u, gSpritesOk ? 1u : 0u, gSpriteBytes, gRenderMax, gRenderCount,
              gTouchLate);
  dbg::status(now, "dump;stress mode=%u hz=%" PRIu32 " frames=%" PRIu32 " skipped=%" PRIu32 " max_us=%" PRIu32,
              static_cast<unsigned>(gStressMode), gStressHz, gStressFrames, gStressSkipped, gStressMaxUs);
  reportTouchConsts(now);
}

}  // namespace disp

#else  // !OT_BRINGUP: the renderer

// ===========================================================================
// Main-screen renderer.
//
// Rules:
//  - frame() takes ONE Frame (snapshot) at most RENDER_MAX_HZ times per
//    second, runs the pace/speed display filters and formats every field into
//    its `want` content. A field is dirty when its content (strings) or its
//    dimming differs from what was last pushed.
//  - service() pushes dirty fields inside startWrite()/endWrite(), at most
//    ~PASS_BUDGET_US of pushes per loop pass (the hero, ~14 ms, always goes
//    alone), so the edge ring is drained between pushes.
//  - Dimming (PAUSED: the frozen fields; WARMUP: the cells) changes palette
//    index 1 of the 1 bpp sprites; a field whose only change is the palette
//    is pushed WITHOUT re-rendering its glyphs.
//  - A screen switch (MAIN / IDLE / SUMMARY) clears the panel in 4
//    bands (one per pass, ~7.7 ms each), draws the static rules, then
//    invalidates every field so all of them are pushed again: no stale pixels.
//  - Backlight (setBacklight): OFF skips every push (frames are still
//    built); a wake from OFF invalidates every field and raises the
//    brightness only once the fresh frame is completely on the panel.
//  - Every field's sprite covers its whole rectangle and is filled before
//    drawing, so a shorter string never leaves old pixels.
//  - The HOLD TO STOP strip is static (drawn once by the main screen's
//    last switch step). While a hold is in progress the hero sprite shows
//    the hold text (its ring area blank) and the 4 bpp ring sprite is pushed
//    over the ring area, clipped to the hero rect, at <= 25 Hz when its 2 %
//    step changed (drawn incrementally: only the new arc segment). A units
//    toggle invalidates every field and shows a toast in the status bar.
//  - No allocation after begin(): the sprites are created once (before BLE),
//    the bus DMA flip buffers are pre-sized, strings live in static
//    buffers, no float reaches a printf format.
// ===========================================================================
#include "hw_display.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug_serial.h"
#include "src/core/ui_policy.h"
#include "version.h"

namespace disp {

namespace {

// ---------------------------------------------------------------------------
// Layout (landscape 320x240)
//   y   0- 23  status bar   (4 bpp 320x24): watch icon + link badge + "watch"
//                           (left); state glyph + state text in Font4 (right):
//                           |> "moving" green, || "PAUSED" orange, hourglass
//                           "WARM-UP" yellow
//   y  24-119  hero pace    (1 bpp 320x96): Font7 x2 "12:34" / "--:--" right-
//                           aligned to HERO_RIGHT (x 292, ink ends at 285); unit "/mi" / "/km"
//                           Font2 x(1,2) left-aligned at UNIT_X (x 290),
//                           bottom-aligned with the digits
//   y 122-151  speed        (1 bpp 320x30): value FreeSansBold18pt right-
//                           aligned to VALUE_RIGHT, unit Font2 at UNIT_X (the
//                           same unit column as the hero: digits don't shift
//                           at 9.9 -> 10.0)
//   y 152      rule
//   y 154-199  3 cells      (1 bpp 106x46 each, x 0 / 107 / 214, rules at x
//                           106 / 213): value FreeSansBold18pt where it fits
//                           (<= 102 px incl. unit), else Font4 / Font2 / Font0;
//                           label Font2 below
//   y 202-239  HOLD TO STOP strip: a rounded box x 12-307, y 202-227
//                           with the label; the box IS the active hold-start
//                           rect (ot::STOP_RECT: a hold starts only there);
//                           y 228-239 and x 0-11 / 308-319 = 12 px bezel
//                           margin, left blank
//   hold:       the hero shows the ring (100x100 4 bpp, circle r 47 around
//               (49,49): its outer 2 rows/cols stay blank) at x 16,
//               pushed at y 22 clipped to the hero rows 24-119, + "Stopping /
//               release / to cancel" centred in x 116-319
// ---------------------------------------------------------------------------
constexpr int16_t SCREEN_W = 320;
constexpr int16_t SCREEN_H = 240;
constexpr int16_t STATUS_Y = 0, STATUS_H = 24;
constexpr int16_t HERO_Y = 24, HERO_H = 96;
constexpr int16_t SPEED_Y = 122, SPEED_H = 30;
constexpr int16_t RULE_Y = 152;
constexpr int16_t CELL_Y = 154, CELL_H = 46, CELL_W = 106;
constexpr int16_t CELL_X0 = 0, CELL_X1 = 107, CELL_X2 = 214;
constexpr int16_t HOLD_Y = 202;  // HOLD TO STOP strip area y 202-239
// Hold ring sprite. Its outer 2 rows/columns stay blank (the ring never touches the sprite edge, so a push never leaves a hard edge).
constexpr int16_t RING_W = 100;
constexpr int16_t RING_X = 16;               // left part of the hero; text to its right
constexpr int16_t RING_Y = 24 - 2;           // HERO_Y - 2: the circle's rows 2..96 land on hero rows 24..118
constexpr int16_t RING_C = 49;               // centre (sprite px)
constexpr int16_t RING_R_OUT = 47;           // 95 px circle: cols/rows 2..96
constexpr int16_t RING_R_IN = 35;            // 13 px thick
constexpr int32_t RING_STEPS = 50;           // 2 % steps (40 ms of a 2 s hold)
constexpr int64_t RING_MIN_US = 40000;       // ring pushes <= 25 Hz
static_assert(RING_C - RING_R_OUT >= 2 && RING_C + RING_R_OUT <= RING_W - 3, "outer 2 rows/cols of the ring blank");
constexpr int16_t HOLD_TEXT_L = RING_X + RING_W;  // 116: hold text area x 116..319
// The HOLD TO STOP strip box = the active hold-start rect.
constexpr int16_t STRIP_X = ot::STOP_RECT.x, STRIP_Y = ot::STOP_RECT.y, STRIP_W = ot::STOP_RECT.w,
                  STRIP_H = ot::STOP_RECT.h, STRIP_R = 6;
static_assert(STATUS_Y + STATUS_H <= HERO_Y && HERO_Y + HERO_H <= SPEED_Y && SPEED_Y + SPEED_H <= RULE_Y &&
                  RULE_Y < CELL_Y && CELL_Y + CELL_H <= HOLD_Y && HOLD_Y < SCREEN_H,
              "layout bands must not overlap");
static_assert(CELL_X0 + CELL_W < CELL_X1 && CELL_X1 + CELL_W < CELL_X2 && CELL_X2 + CELL_W == SCREEN_W,
              "three cells with 1 px rules fill the width");
// The touch targets (src/core/touch_gesture.h) must match this layout.
static_assert(ot::PACE_RECT.x == 0 && ot::PACE_RECT.y == HERO_Y && ot::PACE_RECT.w == SCREEN_W &&
                  ot::PACE_RECT.h == HERO_H,
              "the units tap rect is the hero rect");
static_assert(ot::STOP_RECT.y == HOLD_Y && ot::STOP_RECT.y > CELL_Y + CELL_H && ot::STOP_RECT.x == ot::HOLD_EDGE_MARGIN_PX &&
                  ot::STOP_RECT.x + ot::STOP_RECT.w == SCREEN_W - ot::HOLD_EDGE_MARGIN_PX &&
                  ot::STOP_RECT.y + ot::STOP_RECT.h == SCREEN_H - ot::HOLD_EDGE_MARGIN_PX &&
                  ot::HOLD_EDGE_MARGIN_PX >= 12,
              "a hold starts only on the strip (y >= 202, below the cells), 12 px bezel margins");
static_assert(RING_Y + 2 >= HERO_Y && RING_Y + RING_W - 3 <= HERO_Y + HERO_H - 1, "ring circle inside the hero rows");
static_assert(STRIP_X == ot::STOP_RECT.x && STRIP_X + STRIP_W == ot::STOP_RECT.x + ot::STOP_RECT.w &&
                  STRIP_Y == ot::STOP_RECT.y && STRIP_Y + STRIP_H == ot::STOP_RECT.y + ot::STOP_RECT.h,
              "the drawn strip box is exactly the active hold-start rect");

// Horizontal anchors shared by the hero and the speed row: values end at
// VALUE_RIGHT, units start at UNIT_X (a fixed unit column as wide as the
// widest unit, "km/h" in Font2 = 28 px; checked at boot by layoutCheck).
constexpr int16_t MARGIN      = 2;   // px kept free at the sprite edges
constexpr int16_t UNIT_GAP    = 4;   // px between a value and its unit (hero, speed)
constexpr int16_t CELL_GAP    = 3;   // same in the cells ("99.99 km" fits FreeSansBold18 in 102 px)
constexpr int16_t UNIT_COL_W  = 28;
constexpr int16_t UNIT_X      = SCREEN_W - MARGIN - UNIT_COL_W;  // 290
constexpr int16_t VALUE_RIGHT = UNIT_X - UNIT_GAP;               // 286: speed value's right edge
// Font7x2 digits carry ~7 px of blank advance on their right (FreeSansBold18:
// ~2 px), so the hero is drawn 6 px further right: both rows' last digits
// then end on ink column ~285 and the hero's blank advance is the unit gap.
constexpr int16_t HERO_BEARING = 6;
constexpr int16_t HERO_RIGHT   = VALUE_RIGHT + HERO_BEARING;      // 292: "99:59" (280 px) starts at x 12
// Vertical anchors (measured with the host mock-up harness, LovyanGFX 1.2.26):
//  - hero digits (Font7x2, middle datum at HERO_H/2) have ink on rows 2..93;
//    the Font2 x(1,2) unit's letters have ink on rows top+8..top+25, so
//    HERO_UNIT_TOP 68 puts their bottom on row 93 ('/' reaches row 95);
//  - speed / cell values use a baseline datum (FreeSansBold18 digits: ink
//    on the 25 rows ending at the baseline row);
//  - STATUS_MID: Font4 middle datum; ink rows mid-12..mid+11: caps 0..17,
//    "moving"'s g descender to row 23 (nothing clipped in 24 rows).
constexpr int16_t HERO_UNIT_TOP = 68;
constexpr int16_t SPEED_BASE    = 27;
constexpr int16_t CELL_BASE     = 28;
constexpr int16_t CELL_LABEL_Y  = 30;  // label (Font2, top datum): ink down to row 45
constexpr int16_t STATUS_MID    = 12;
static_assert(HERO_RIGHT - 280 >= MARGIN, "Font7x2 99:59 (280 px) fits left of the unit column");

// ---------------------------------------------------------------------------
// Idle and summary screens (same sprites, same field slots as MAIN):
//   IDLE     status   watch icon + "idle" (grey)
//            hero     "Start moving to begin" (Font4, y 22; while re-arm
//                     blocked, i.e. the belt kept moving after a hold-stop:
//                     "Stop the belt to start again"), the units line
//                     (Font4, y 56) and "(tap here to change units)" (Font2,
//                     y 84): the hero rect is the units tap target (PACE_RECT)
//            speed    the small fw/build text, pushed at y 210 (bottom rows
//                     210-239) instead of the speed row
//            cells    unused (blank)
//   SUMMARY  status   watch icon + "SUMMARY" (white)
//            hero     title (Font4, middle at y 13: "Workout complete", or
//                     "Ended: no movement" after an auto-end), then row 1 at
//                     sprite rows 40-85 laid out like the cells: distance |
//                     total | moving (value + unit, label below)
//            speed    countdown: "back to start in m:ss" (Font2) and a thin
//                     bar x 12-307, rows 19-26, emptying over SUMMARY
//            rule     y 152 and the cell rules, as on MAIN
//            cells    row 2: avg pace (+ /mi|/km) | avg speed | top speed
//            strip    none (y 202-239 blank: no hold on the summary)
// ---------------------------------------------------------------------------
constexpr int16_t IDLE_MSG_Y    = 22;
constexpr int16_t IDLE_UNITS_Y  = 56;
constexpr int16_t IDLE_HINT_Y   = 84;
constexpr int16_t IDLE_FW_Y     = SCREEN_H - SPEED_H;  // 210
constexpr int16_t SUM_TITLE_Y   = 13;
constexpr int16_t SUM_ROW_TOP   = 40;                  // hero sprite rows 40..85 (a 46-row cell block)
constexpr int16_t SUM_TEXT_Y    = 8;                   // countdown text (Font2, middle) in the speed sprite
constexpr int16_t SUM_BAR_X     = 12, SUM_BAR_W = SCREEN_W - 2 * 12, SUM_BAR_Y = 19, SUM_BAR_H = 8;
static_assert(SUM_ROW_TOP + CELL_H <= HERO_H, "summary row 1 inside the hero sprite");
static_assert(SUM_BAR_Y + SUM_BAR_H <= SPEED_H, "summary bar inside the speed sprite");
static_assert(IDLE_FW_Y >= CELL_Y + CELL_H, "idle fw text below the (unused) cells");

constexpr int64_t FRAME_US       = 1000000 / static_cast<int64_t>(ot::RENDER_MAX_HZ);  // 100 ms
constexpr int64_t PASS_BUDGET_US = 8000;   // no further push is started once a pass has used this
constexpr int16_t CLEAR_BANDS    = 4;      // screen switch: 4 x 60 rows (~7.7 ms each at 40 MHz)
constexpr int16_t CLEAR_BAND_H   = SCREEN_H / CLEAR_BANDS;
static_assert(CLEAR_BAND_H * CLEAR_BANDS == SCREEN_H, "bands cover the screen");
constexpr uint32_t HEAP_FLOOR_FREE    = 60u * 1024u;  // heap floors
constexpr uint32_t HEAP_FLOOR_LARGEST = 30u * 1024u;

// Colours. Panel (RGB565) for the direct clears/rules; palette entries as RGB.
constexpr uint16_t PANEL_BG   = TFT_BLACK;
constexpr uint16_t PANEL_RULE = 0x4208;  // dark grey (64,64,64)
constexpr uint16_t PANEL_STRIP = lgfx::color565(176, 40, 32);   // HOLD TO STOP border / icon
constexpr uint16_t PANEL_STRIP_TEXT = TFT_WHITE;
struct Rgb { uint8_t r, g, b; };
constexpr Rgb RGB_BG     = {0, 0, 0};
constexpr Rgb RGB_FG     = {255, 255, 255};
constexpr Rgb RGB_DIM    = {128, 128, 128}; // frozen/not-yet-counting numbers (palette index 1)
constexpr Rgb RGB_GREY   = {128, 128, 128};
constexpr Rgb RGB_YELLOW = {255, 208, 0};
constexpr Rgb RGB_GREEN  = {64, 224, 64};
constexpr Rgb RGB_ORANGE = {255, 128, 0};
// Status sprite (4 bpp) palette indices. uint32_t on purpose: for palette
// sprites LovyanGFX takes the raw integer as the index (as the 1 bpp sprites'
// 0u / 1u), whatever the drawing call.
constexpr uint32_t PI_BG = 0, PI_FG = 1, PI_GREY = 2, PI_YELLOW = 3, PI_GREEN = 4, PI_ORANGE = 5;
// Ring sprite (4 bpp) palette indices.
constexpr uint32_t RI_BG = 0, RI_TRACK = 1, RI_FILL = 2, RI_ICON = 3;
constexpr Rgb RGB_RING_TRACK = {56, 56, 56};
constexpr Rgb RGB_RING_FILL  = {255, 72, 48};
constexpr Rgb RGB_RING_ICON  = {160, 160, 160};

LGFX_CYD gLcd;

// ---------------------------------------------------------------------------
// Sprites (created once in begin(), never freed)
// ---------------------------------------------------------------------------
enum Spr : uint8_t { SP_STATUS, SP_HERO, SP_SPEED, SP_CELL0, SP_CELL1, SP_CELL2, SP_RING, SP_N };
struct SpriteSpec {
  const char* name;
  int16_t w, h;
  uint8_t bpp;
};
constexpr SpriteSpec SPRITES[SP_N] = {
    {"status", SCREEN_W, STATUS_H, 4}, {"hero", SCREEN_W, HERO_H, 1}, {"speed", SCREEN_W, SPEED_H, 1},
    {"cell0", CELL_W, CELL_H, 1},      {"cell1", CELL_W, CELL_H, 1},  {"cell2", CELL_W, CELL_H, 1},
    {"ring", RING_W, RING_W, 4},
};
lgfx::LGFX_Sprite gSpr[SP_N];
uint32_t gSpriteBytes = 0;
bool     gReady = false;  // panel + sprites OK: the renderer runs

// ---------------------------------------------------------------------------
// Fonts and fallback chains ("use the next smaller font rather than truncate")
// ---------------------------------------------------------------------------
struct FontSpec {
  const lgfx::IFont* font;
  float sx, sy;  // text size (x, y)
  const char* name;
};
const FontSpec HERO_FONTS[]  = {{&lgfx::fonts::Font7, 2.0f, 2.0f, "Font7x2"}, {&lgfx::fonts::Font4, 1.0f, 1.0f, "Font4"}};
// speed and cell values: FreeSansBold18pt where it fits, then smaller fonts
// (the last one fits every possible string, checked at boot)
const FontSpec VALUE_FONTS[] = {{&lgfx::fonts::FreeSansBold18pt7b, 1.0f, 1.0f, "FreeSansBold18pt"},
                                {&lgfx::fonts::Font4, 1.0f, 1.0f, "Font4"},
                                {&lgfx::fonts::Font2, 1.0f, 1.0f, "Font2"},
                                {&lgfx::fonts::Font0, 1.0f, 1.0f, "Font0"}};
const FontSpec HERO_UNIT_FONT = {&lgfx::fonts::Font2, 1.0f, 2.0f, "Font2x(1,2)"};  // "/mi"
const FontSpec SMALL_FONT     = {&lgfx::fonts::Font2, 1.0f, 1.0f, "Font2"};       // speed/cell units, labels, "watch"
const FontSpec STATUS_FONT    = {&lgfx::fonts::Font4, 1.0f, 1.0f, "Font4"};       // state text
const FontSpec MSG_FONT       = {&lgfx::fonts::Font4, 1.0f, 1.0f, "Font4"};       // idle placeholder message
const FontSpec HOLD_HEAD_FONT = {&lgfx::fonts::FreeSansBold18pt7b, 1.0f, 1.0f, "FreeSansBold18pt"};  // "Stopping"
const FontSpec HOLD_FONT      = {&lgfx::fonts::Font4, 1.0f, 1.0f, "Font4"};       // "release" / "to cancel", toast, strip
template <size_t N> constexpr size_t countOf(const FontSpec (&)[N]) { return N; }

void useFont(lgfx::LGFX_Sprite& s, const FontSpec& f) {
  s.setFont(f.font);
  s.setTextSize(f.sx, f.sy);
}
int32_t widthOf(lgfx::LGFX_Sprite& s, const FontSpec& f, const char* txt) {
  useFont(s, f);
  return s.textWidth(txt);
}
// The first font of the chain whose value width + extra fits `avail` (the
// last one otherwise; every chain's last font fits every possible string,
// checked at boot by the layout self-check).
const FontSpec& pick(lgfx::LGFX_Sprite& s, const FontSpec* chain, size_t n, const char* txt, int32_t extra,
                     int32_t avail) {
  for (size_t i = 0; i + 1 < n; ++i)
    if (widthOf(s, chain[i], txt) + extra <= avail) return chain[i];
  return chain[n - 1];
}

// ---------------------------------------------------------------------------
// Fields
// ---------------------------------------------------------------------------
enum Field : uint8_t { F_HERO, F_STATUS, F_SPEED, F_TOTAL, F_MOVING, F_DIST, F_N };  // = push order
const char* const FIELD_NAMES[F_N] = {"hero", "status", "speed", "total", "moving", "dist"};
constexpr uint8_t FIELD_SPRITE[F_N] = {SP_HERO, SP_STATUS, SP_SPEED, SP_CELL2, SP_CELL1, SP_CELL0};
constexpr int16_t FIELD_X[F_N] = {0, 0, 0, CELL_X2, CELL_X1, CELL_X0};
constexpr int16_t FIELD_Y[F_N] = {HERO_Y, STATUS_Y, SPEED_Y, CELL_Y, CELL_Y, CELL_Y};

enum class Screen : uint8_t { None, Idle, Main, Summary };
const char* screenName(Screen s) {
  return s == Screen::Main ? "main" : (s == Screen::Idle ? "idle" : (s == Screen::Summary ? "summary" : "none"));
}

constexpr size_t STR_N = 32;
// Everything that decides a field's pixels except the palette. Compared with
// memcmp, so it is always fully written (zero-padded strings).
struct Content {
  uint8_t used;   // 0: the field is not shown on this screen (area stays clear)
  uint8_t style;  // screen / colour selector
  uint8_t link;   // status bar only: 1 = a watch/app is subscribed (filled icon badge)
  char    a[STR_N];
  char    b[STR_N];
};
struct FieldState {
  Content want;
  Content shown;
  bool    wantDim;
  bool    shownDim;
  bool    valid;    // pushed since the last screen switch
  uint32_t pushes;  // since boot
};
FieldState gF[F_N];

void setStr(char (&dst)[STR_N], const char* src) {
  memset(dst, 0, STR_N);
  strncpy(dst, src, STR_N - 1);
}

bool dirty(const FieldState& f) {
  if (!f.want.used) return false;
  return !f.valid || memcmp(&f.want, &f.shown, sizeof(Content)) != 0 || f.wantDim != f.shownDim;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
Screen   gScreen = Screen::None;  // what the panel shows (after a completed switch)
Screen   gTarget = Screen::None;  // what the latest frame wants
bool     gSwitching = false;
int16_t  gSwitchStep = 0;         // 0..CLEAR_BANDS-1 clear bands, CLEAR_BANDS = static rules
int64_t  gNextFrame = 0;
bool     gForce = false;
uint32_t gFrames = 0, gSwitches = 0;
ot::PaceDisplayFilter  gPace;
ot::SpeedDisplayFilter gSpeed;
Frame    gLast;

uint32_t gRenderMax = 0, gRenderCount = 0;  // G line, reset by takeStats
uint32_t gRenderMaxBoot = 0;                // longest pass since boot (dump)

uint32_t clampUs(int64_t d) {
  return d < 0 ? 0u : (d > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(d));
}

// ---------------------------------------------------------------------------
// Content per screen
// ---------------------------------------------------------------------------
enum : uint8_t { ST_WARMUP = 1, ST_MOVING, ST_PAUSED, ST_IDLE, ST_SUMMARY, ST_FW, ST_HOLD };

// Hold / toast state (loop task only)
bool     gHold = false;          // the recognizer is holding (setHold, every pass)
int32_t  gHoldPermille = 0;
uint32_t gHolds = 0;             // hold starts seen (dump)
int32_t  gRingDrawn = -1;        // step the ring SPRITE holds (-1: full redraw needed)
int32_t  gRingShown = -1;        // step on the PANEL (-1: not on the panel)
int64_t  gRingLastPush = 0;
uint32_t gRingPushes = 0;
uint32_t gRingMaxUs = 0;         // longest ring draw+push since boot (dump)
int64_t  gToastUntil = 0;        // units toast shown while frame t < this
uint32_t gToasts = 0;
const char* const TOAST_PREFIX = "units: ";

int32_t ringStep(int32_t permille) {
  if (permille <= 0) return 0;
  if (permille >= 1000) return RING_STEPS;
  return permille * RING_STEPS / 1000;
}
void setToast(Content& c, const Frame& f) {
  if (f.t < gToastUntil) {
    char buf[STR_N];
    snprintf(buf, sizeof(buf), "%s%s", TOAST_PREFIX, ot::unitsName(f.units));
    setStr(c.b, buf);
  }
}

void buildMain(const Frame& f) {
  const bool paused = f.state == ot::WorkoutState::Paused;
  Content c;
  // status: watch icon (link state) + state text
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.link = f.linked ? 1 : 0;
  c.style = f.state == ot::WorkoutState::Warmup ? ST_WARMUP : (paused ? ST_PAUSED : ST_MOVING);
  setStr(c.a, f.state == ot::WorkoutState::Warmup ? "WARM-UP" : (paused ? "PAUSED" : "moving"));
  setToast(c, f);
  gF[F_STATUS].want = c;
  gF[F_STATUS].wantDim = false;
  // hero: pace from the filter (unrounded speed in, hysteresis, dead-band),
  // or the hold text while a hold is in progress (the ring is separate)
  char buf[STR_N];
  memset(&c, 0, sizeof(c));
  c.used = 1;
  if (gHold) {
    c.style = ST_HOLD;
  } else {
    gPace.format(buf, sizeof(buf));
    setStr(c.a, buf);
    setStr(c.b, ot::paceUnitName(f.units));
  }
  gF[F_HERO].want = c;
  // speed
  memset(&c, 0, sizeof(c));
  c.used = 1;
  gSpeed.format(buf, sizeof(buf));
  setStr(c.a, buf);
  setStr(c.b, ot::speedUnitName(f.units));
  gF[F_SPEED].want = c;
  // cells: a = value, b = unit (distance only); the label is fixed per field
  memset(&c, 0, sizeof(c));
  c.used = 1;
  ot::fmtDistance(buf, sizeof(buf), f.wkPulses, f.units);
  setStr(c.a, buf);
  setStr(c.b, ot::unitsName(f.units));
  gF[F_DIST].want = c;
  memset(&c, 0, sizeof(c));
  c.used = 1;
  ot::fmtDuration(buf, sizeof(buf), f.movingUs);
  setStr(c.a, buf);
  gF[F_MOVING].want = c;
  memset(&c, 0, sizeof(c));
  c.used = 1;
  ot::fmtDuration(buf, sizeof(buf), f.totalUs);
  setStr(c.a, buf);
  gF[F_TOTAL].want = c;
  // Dimming (palette index 1, no glyph re-render):
  //  - PAUSED: the frozen fields (hero "--:--", speed, distance, moving);
  //    total keeps ticking and stays bright;
  //  - WARMUP: the three cells (0, "not counting yet"); pace/speed are live.
  const bool warmup = f.state == ot::WorkoutState::Warmup;
  gF[F_HERO].wantDim = paused && !gHold;
  gF[F_SPEED].wantDim = paused;
  gF[F_DIST].wantDim = paused || warmup;
  gF[F_MOVING].wantDim = paused || warmup;
  gF[F_TOTAL].wantDim = warmup;
}

// Summary / backlight state (loop task only): the summary values of the current frame and
// the backlight.
ot::SummaryView gSum;              // makeSummaryView(frame.summary, frame.units), refreshed every summary frame
uint8_t  gBlTarget  = 0;           // the level asked for (setBacklight)
uint8_t  gBlApplied = 0;           // the level on the panel
bool     gBlWake    = false;       // OFF -> on: full redraw in progress, brightness raised when complete
int64_t  gBlWakeT0  = 0;           // esp_timer time of the wake request
uint32_t gBlChanges = 0, gWakes = 0, gWakeMaxUs = 0, gDarkFrames = 0;

// Idle hero message: after a hold-stop the belt may
// keep moving; the Workout then stays re-arm blocked (no new bout) until it
// stops, even after the summary cleared to IDLE. "Start moving to begin"
// would be wrong then, so the idle screen asks for the stop instead.
// Both fit Font4 in SCREEN_W - 2 * MARGIN (layout;screens msg= / rearm=).
constexpr const char* IDLE_MSG       = "Start moving to begin";
constexpr const char* IDLE_MSG_REARM = "Stop the belt to start again";

void idleUnitsLine(char* buf, size_t n, ot::Units u) {
  snprintf(buf, n, "%s, %s, min%s", ot::unitsName(u), ot::speedUnitName(u), ot::paceUnitName(u));
}

void buildIdle(const Frame& f) {
  Content c;
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.link = f.linked ? 1 : 0;  // watch icon (the idle screen shows the watch status too)
  c.style = ST_IDLE;
  setStr(c.a, "idle");
  setToast(c, f);
  gF[F_STATUS].want = c;
  // hero sprite: the message, the units line and the tap hint (the hero rect
  // is the units tap target, ot::PACE_RECT)
  char buf[STR_N];
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.style = ST_IDLE;
  setStr(c.a, f.rearmBlocked ? IDLE_MSG_REARM : IDLE_MSG);
  idleUnitsLine(buf, sizeof(buf), f.units);
  setStr(c.b, buf);
  gF[F_HERO].want = c;
  // speed sprite: firmware identity (version + build id on the idle screen)
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.style = ST_FW;
  snprintf(buf, sizeof(buf), "fw %s build %08" PRIx32 "%s", OT_FW_VERSION, hw::BUILD_ID,
           hw::BUILD_DIRTY ? " DIRTY" : "");
  setStr(c.a, buf);
  gF[F_SPEED].want = c;
  for (uint8_t i : {F_DIST, F_MOVING, F_TOTAL}) memset(&gF[i].want, 0, sizeof(Content));  // unused
  for (uint8_t i = 0; i < F_N; ++i) gF[i].wantDim = false;
}

// Summary screen. Every value comes from the FROZEN record in the frame
// (Frame.summary, = Workout::summary()) in the current units, never from live
// values; the countdown from the record's own summaryStartUs and the
// Workout's SUMMARY duration at the frame's t (the loop's clock).
const char* summaryTitle(const ot::SummaryRecord& r) {
  return r.endReason == ot::EndReason::AutoEnd ? "Ended: no movement" : "Workout complete";
}

void buildSummary(const Frame& f) {
  ot::makeSummaryView(f.summary, f.units, gSum);
  Content c;
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.link = f.linked ? 1 : 0;  // watch icon on the summary status bar too
  c.style = ST_SUMMARY;
  setStr(c.a, "SUMMARY");
  setToast(c, f);
  gF[F_STATUS].want = c;
  // hero: title + row 1 (distance | total | moving), drawn from gSum. Its
  // content is a key of everything those strings depend on (units, record
  // identity), so it is re-rendered exactly when they can change.
  char buf[STR_N];
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.style = ST_SUMMARY;
  snprintf(buf, sizeof(buf), "%s;%" PRIu32 ";%" PRId64, ot::unitsName(f.units), f.summary.workoutId,
           f.summary.summaryStartUs);
  setStr(c.a, buf);
  setStr(c.b, summaryTitle(f.summary));
  gF[F_HERO].want = c;
  // speed sprite: the countdown text (whole seconds, rounded UP, so it reads
  // 0:01 until the clear) and the bar's fill width in px
  const int64_t rem = ot::summaryRemainingUs(f.summary, f.summaryUs, f.t);
  const int64_t remS = (rem + 999999) / 1000000;
  const int64_t inner = SUM_BAR_W - 4;
  const int64_t fill = f.summaryUs > 0 ? rem * inner / f.summaryUs : 0;
  char dur[ot::DUR_STR_SIZE];
  ot::fmtDuration(dur, sizeof(dur), remS * 1000000);
  memset(&c, 0, sizeof(c));
  c.used = 1;
  c.style = ST_SUMMARY;
  char line[24 + ot::DUR_STR_SIZE];  // any duration string fits (setStr then keeps STR_N - 1 chars)
  snprintf(line, sizeof(line), "back to start in %s", dur);
  setStr(c.a, line);
  snprintf(buf, sizeof(buf), "%" PRId64, fill);
  setStr(c.b, buf);
  gF[F_SPEED].want = c;
  // cells (row 2): a = value, b = unit; labels by style in renderCell
  const struct {
    Field f;
    const char* v;
    const char* u;
  } cells[] = {{F_DIST, gSum.avgPace, ot::paceUnitName(f.units)},
               {F_MOVING, gSum.avgSpeed, ot::speedUnitName(f.units)},
               {F_TOTAL, gSum.topSpeed, ot::speedUnitName(f.units)}};
  for (const auto& x : cells) {
    memset(&c, 0, sizeof(c));
    c.used = 1;
    c.style = ST_SUMMARY;
    setStr(c.a, x.v);
    setStr(c.b, x.u);
    gF[x.f].want = c;
  }
  for (uint8_t i = 0; i < F_N; ++i) gF[i].wantDim = false;
}

// ---------------------------------------------------------------------------
// Drawing (into the field's sprite; the push is separate)
// ---------------------------------------------------------------------------
void setPal(lgfx::LGFX_Sprite& s, uint32_t idx, const Rgb& c) { s.setPaletteColor(idx, c.r, c.g, c.b); }

void drawWatchIcon(lgfx::LGFX_Sprite& s, int32_t x, int32_t y, uint32_t col, bool linked) {
  // A small watch (strap stubs, case, hands) with a "link" badge on the
  // case's top-right corner: hollow = no subscriber, filled = >= 1 peer
  // subscribed to RSC or FTMS data (shape, not colour
  // alone). The badge sits above the text's x-height with an 8 px gap, so it
  // never reads as a letter ("owatch").
  s.fillRect(x + 3, y, 6, 3, col);
  s.fillRect(x + 3, y + 15, 6, 3, col);
  s.drawRoundRect(x, y + 3, 12, 12, 3, col);
  s.drawFastVLine(x + 6, y + 6, 4, col);
  s.drawFastHLine(x + 6, y + 9, 3, col);
  s.fillCircle(x + 13, y + 3, 4, PI_BG);  // cut the case outline under the badge
  if (linked) s.fillCircle(x + 13, y + 3, 3, PI_GREEN);
  else s.drawCircle(x + 13, y + 3, 3, col);
}
constexpr int32_t WATCH_TEXT_X = 4 + 24;  // icon at x 4; badge ends at x 20

// State glyphs, 16 px tall, centred on the caps of the Font4 state text
// (caps rows 0..17, glyph rows 1..16): |> running, || paused, hourglass
// warm-up (shape + colour, not colour alone).
constexpr int32_t GLYPH_W = 14, GLYPH_TOP = 1, GLYPH_H = 16, GLYPH_GAP = 6;
void drawStateGlyph(lgfx::LGFX_Sprite& s, uint8_t style, int32_t x, uint32_t col) {
  const int32_t t = GLYPH_TOP, b = GLYPH_TOP + GLYPH_H - 1, m = GLYPH_TOP + GLYPH_H / 2;
  switch (style) {
    case ST_MOVING: s.fillTriangle(x + 1, t, x + 1, b, x + GLYPH_W - 1, m, col); break;
    case ST_PAUSED:
      s.fillRect(x + 1, t, 4, GLYPH_H, col);
      s.fillRect(x + GLYPH_W - 5, t, 4, GLYPH_H, col);
      break;
    case ST_WARMUP:
      s.fillRect(x, t, GLYPH_W, 2, col);
      s.fillRect(x, b - 1, GLYPH_W, 2, col);
      s.fillTriangle(x + 1, t + 2, x + GLYPH_W - 2, t + 2, x + GLYPH_W / 2 - 1, m, col);
      s.fillTriangle(x + GLYPH_W / 2 - 1, m, x + 1, b - 2, x + GLYPH_W - 2, b - 2, col);
      break;
    default: break;
  }
}
bool hasGlyph(uint8_t style) { return style == ST_MOVING || style == ST_PAUSED || style == ST_WARMUP; }

void renderStatus(const Content& c) {
  lgfx::LGFX_Sprite& s = gSpr[SP_STATUS];
  s.fillSprite(PI_BG);
  if (c.b[0] != '\0') {
    // units toast (UNITS_TOAST_MS): replaces the watch icon + label
    useFont(s, HOLD_FONT);
    s.setTextColor(PI_YELLOW);
    s.setTextDatum(lgfx::textdatum_t::middle_left);
    s.drawString(c.b, 4, STATUS_MID);
  } else {
    // linked = white icon + green filled badge + white label; else grey + hollow
    drawWatchIcon(s, 4, 3, c.link ? PI_FG : PI_GREY, c.link != 0);
    useFont(s, SMALL_FONT);
    s.setTextColor(c.link ? PI_FG : PI_GREY);
    s.setTextDatum(lgfx::textdatum_t::middle_left);
    s.drawString("watch", WATCH_TEXT_X, STATUS_MID);
  }
  uint32_t col = PI_FG;
  switch (c.style) {
    case ST_WARMUP: col = PI_YELLOW; break;
    case ST_MOVING: col = PI_GREEN; break;
    case ST_PAUSED: col = PI_ORANGE; break;
    case ST_IDLE: col = PI_GREY; break;
    default: col = PI_FG; break;
  }
  useFont(s, STATUS_FONT);
  const int32_t right = SCREEN_W - 4;
  const int32_t w = s.textWidth(c.a);
  s.setTextColor(col);
  s.setTextDatum(lgfx::textdatum_t::middle_right);
  s.drawString(c.a, right, STATUS_MID);
  if (hasGlyph(c.style)) drawStateGlyph(s, c.style, right - w - GLYPH_GAP - GLYPH_W, col);
}

void renderHeroMain(const Content& c) {
  lgfx::LGFX_Sprite& s = gSpr[SP_HERO];
  s.fillSprite(0u);
  s.setTextColor(1u);  // transparent: a glyph's box never erases a neighbour
  // The number right-aligned to HERO_RIGHT, then the unit in the fixed unit
  // column, its letters' bottom on the digits' bottom row: the seconds digits
  // and the unit never move (only the minutes grow to the left). The digits'
  // blank advance reaches x 291, past UNIT_X, which is why text is drawn
  // transparent.
  const FontSpec& f = pick(s, HERO_FONTS, countOf(HERO_FONTS), c.a, 0, HERO_RIGHT - MARGIN);
  useFont(s, f);
  s.setTextDatum(lgfx::textdatum_t::middle_right);
  s.drawString(c.a, HERO_RIGHT, HERO_H / 2);
  useFont(s, HERO_UNIT_FONT);
  s.setTextDatum(lgfx::textdatum_t::top_left);
  s.drawString(c.b, UNIT_X, HERO_UNIT_TOP);
}

void renderHeroIdle(const Content& c) {
  lgfx::LGFX_Sprite& s = gSpr[SP_HERO];
  s.fillSprite(0u);
  s.setTextColor(1u);  // transparent: a glyph's box never erases a neighbour
  s.setTextDatum(lgfx::textdatum_t::middle_center);
  useFont(s, MSG_FONT);
  s.drawString(c.a, SCREEN_W / 2, IDLE_MSG_Y);
  s.drawString(c.b, SCREEN_W / 2, IDLE_UNITS_Y);  // the units line, Font4
  useFont(s, SMALL_FONT);
  s.drawString("(tap here to change units)", SCREEN_W / 2, IDLE_HINT_Y);
}

// A value (+ optional small unit) centred in a w-px column of sprite `s` at
// x0, its baseline at top + CELL_BASE, in the largest VALUE_FONTS font whose
// value + gap + unit fits w - 2*MARGIN; the label (Font2) below at top +
// CELL_LABEL_Y. The cells (renderCell) and the summary's row 1 use it.
void drawValueLabel(lgfx::LGFX_Sprite& s, int32_t x0, int32_t w, int32_t top, const char* value, const char* unit,
                    const char* label) {
  const bool hasUnit = unit[0] != '\0';
  const int32_t unitW = hasUnit ? widthOf(s, SMALL_FONT, unit) + CELL_GAP : 0;
  const FontSpec& f = pick(s, VALUE_FONTS, countOf(VALUE_FONTS), value, unitW, w - 2 * MARGIN);
  const int32_t valueW = widthOf(s, f, value);
  int32_t x = (w - valueW - unitW) / 2;
  if (x < MARGIN) x = MARGIN;
  s.setTextDatum(lgfx::textdatum_t::baseline_left);
  s.drawString(value, x0 + x, top + CELL_BASE);
  useFont(s, SMALL_FONT);
  if (hasUnit) s.drawString(unit, x0 + x + valueW + CELL_GAP, top + CELL_BASE);
  s.setTextDatum(lgfx::textdatum_t::top_center);
  s.drawString(label, x0 + w / 2, top + CELL_LABEL_Y);
}

// Summary hero: title, then distance | total | moving from gSum (the
// frozen record in the frame's units; c.a is only the dirty key).
void renderHeroSummary(const Content& c) {
  lgfx::LGFX_Sprite& s = gSpr[SP_HERO];
  s.fillSprite(0u);
  s.setTextColor(1u);
  useFont(s, MSG_FONT);
  s.setTextDatum(lgfx::textdatum_t::middle_center);
  s.drawString(c.b, SCREEN_W / 2, SUM_TITLE_Y);
  drawValueLabel(s, CELL_X0, CELL_W, SUM_ROW_TOP, gSum.distance, ot::unitsName(gLast.units), "distance");
  drawValueLabel(s, CELL_X1, CELL_W, SUM_ROW_TOP, gSum.total, "", "total");
  drawValueLabel(s, CELL_X2, CELL_W, SUM_ROW_TOP, gSum.moving, "", "moving");
}

// The hero while a hold is in progress. The ring area (x 16..115) stays
// blank: the ring sprite is pushed over it after this sprite (service()).
void renderHeroHold() {
  lgfx::LGFX_Sprite& s = gSpr[SP_HERO];
  s.fillSprite(0u);
  s.setTextColor(1u);
  const int32_t cx = (HOLD_TEXT_L + SCREEN_W) / 2;
  useFont(s, HOLD_HEAD_FONT);
  s.setTextDatum(lgfx::textdatum_t::baseline_center);
  s.drawString("Stopping", cx, 30);
  useFont(s, HOLD_FONT);
  s.setTextDatum(lgfx::textdatum_t::middle_center);
  s.drawString("release", cx, 52);
  s.drawString("to cancel", cx, 78);
}

// The ring sprite. Full redraw (track + arc 0..step + stop icon) when
// gRingDrawn < 0, else only the new arc segment gRingDrawn..step. 0 deg =
// 3 o'clock, clockwise; the fill starts at 12 o'clock (-90 deg).
void drawRing(int32_t step) {
  lgfx::LGFX_Sprite& s = gSpr[SP_RING];
  const float deg = 360.0f / static_cast<float>(RING_STEPS);
  if (gRingDrawn < 0) {
    s.fillSprite(RI_BG);
    s.fillArc(RING_C, RING_C, RING_R_IN, RING_R_OUT, 0.0f, 360.0f, RI_TRACK);
    s.fillRect(RING_C - 11, RING_C - 11, 23, 23, RI_ICON);  // a "stop" square
    gRingDrawn = 0;
  }
  if (step > gRingDrawn) {
    s.fillArc(RING_C, RING_C, RING_R_IN, RING_R_OUT, -90.0f + deg * static_cast<float>(gRingDrawn),
              -90.0f + deg * static_cast<float>(step), RI_FILL);
    gRingDrawn = step;
  }
}

void pushRing(int64_t now) {
  const int64_t t0 = esp_timer_get_time();
  const int32_t step = ringStep(gHoldPermille);
  drawRing(step);
  gLcd.setClipRect(0, HERO_Y, SCREEN_W, HERO_H);  // only the hero rows: never touches the status bar
  gSpr[SP_RING].pushSprite(&gLcd, RING_X, RING_Y);
  gLcd.clearClipRect();
  gRingShown = step;
  gRingLastPush = now;
  ++gRingPushes;
  const uint32_t us = clampUs(esp_timer_get_time() - t0);
  if (us > gRingMaxUs) gRingMaxUs = us;
}

// The ring is due when a hold is in progress, the hold hero is on the panel,
// and the ring is missing there or its step changed (<= 25 Hz).
bool ringDue(int64_t now) {
  if (!gHold || gSwitching || gScreen != Screen::Main) return false;
  const FieldState& h = gF[F_HERO];
  if (!h.valid || h.shown.style != ST_HOLD || h.want.style != ST_HOLD) return false;
  if (gRingShown < 0) return true;
  return ringStep(gHoldPermille) != gRingShown && now - gRingLastPush >= RING_MIN_US;
}

void renderSpeed(const Content& c) {
  lgfx::LGFX_Sprite& s = gSpr[SP_SPEED];
  s.fillSprite(0u);
  s.setTextColor(1u);  // transparent: a glyph's box never erases a neighbour
  if (c.style == ST_FW) {
    useFont(s, SMALL_FONT);
    s.setTextDatum(lgfx::textdatum_t::middle_center);
    s.drawString(c.a, SCREEN_W / 2, SPEED_H / 2);
    return;
  }
  if (c.style == ST_SUMMARY) {  // countdown: text + a bar emptying over SUMMARY (c.b = fill px)
    useFont(s, SMALL_FONT);
    s.setTextDatum(lgfx::textdatum_t::middle_center);
    s.drawString(c.a, SCREEN_W / 2, SUM_TEXT_Y);
    s.drawRect(SUM_BAR_X, SUM_BAR_Y, SUM_BAR_W, SUM_BAR_H, 1u);
    const int32_t fill = static_cast<int32_t>(strtol(c.b, nullptr, 10));
    if (fill > 0) s.fillRect(SUM_BAR_X + 2, SUM_BAR_Y + 2, fill, SUM_BAR_H - 4, 1u);
    return;
  }
  // Anchored like the hero: value right-aligned to VALUE_RIGHT,
  // unit at UNIT_X, both on one baseline.
  const FontSpec& f = pick(s, VALUE_FONTS, countOf(VALUE_FONTS), c.a, 0, VALUE_RIGHT - MARGIN);
  useFont(s, f);
  s.setTextDatum(lgfx::textdatum_t::baseline_right);
  s.drawString(c.a, VALUE_RIGHT, SPEED_BASE);
  useFont(s, SMALL_FONT);
  s.setTextDatum(lgfx::textdatum_t::baseline_left);
  s.drawString(c.b, UNIT_X, SPEED_BASE);
}

// A cell value (+ optional small unit) centred in the cell on CELL_BASE, in
// the largest VALUE_FONTS font whose value + gap + unit fits CELL_W - 2*MARGIN.
void renderCell(Field fld, const Content& c) {
  lgfx::LGFX_Sprite& s = gSpr[FIELD_SPRITE[fld]];
  s.fillSprite(0u);
  s.setTextColor(1u);  // transparent: a glyph's box never erases a neighbour
  const char* label = c.style == ST_SUMMARY  // summary row 2
                          ? (fld == F_DIST ? "avg pace" : (fld == F_MOVING ? "avg speed" : "top speed"))
                          : (fld == F_DIST ? "distance" : (fld == F_MOVING ? "moving" : "total"));
  drawValueLabel(s, 0, CELL_W, 0, c.a, c.b, label);
}

void renderField(Field fld) {
  const Content& c = gF[fld].want;
  switch (fld) {
    case F_STATUS: renderStatus(c); break;
    case F_HERO:
      if (c.style == ST_IDLE) renderHeroIdle(c);
      else if (c.style == ST_SUMMARY) renderHeroSummary(c);
      else if (c.style == ST_HOLD) renderHeroHold();
      else renderHeroMain(c);
      break;
    case F_SPEED: renderSpeed(c); break;
    default: renderCell(fld, c); break;
  }
}

// Render (only if the content changed), apply the palette, push.
void pushField(Field fld) {
  FieldState& st = gF[fld];
  lgfx::LGFX_Sprite& s = gSpr[FIELD_SPRITE[fld]];
  const bool contentChanged = !st.valid || memcmp(&st.want, &st.shown, sizeof(Content)) != 0;
  if (contentChanged) renderField(fld);
  if (fld != F_STATUS) setPal(s, 1, st.wantDim ? RGB_DIM : RGB_FG);  // 1 bpp: PAUSED = palette only
  // On the idle screen the speed sprite (the fw text) sits at the bottom.
  const int16_t y = (fld == F_SPEED && gScreen == Screen::Idle) ? IDLE_FW_Y : FIELD_Y[fld];
  s.pushSprite(&gLcd, FIELD_X[fld], y);
  if (fld == F_HERO) gRingShown = -1;  // the hero push covers the ring area
  st.shown = st.want;
  st.shownDim = st.wantDim;
  st.valid = true;
  ++st.pushes;
}

// The static HOLD TO STOP strip (rounded 2 px box, a 3/4-ring icon and
// the label), drawn straight to the panel once per switch to the main
// screen. The box is exactly the active hold-start rect (ot::STOP_RECT).
void drawHoldStrip() {
  gLcd.drawRoundRect(STRIP_X, STRIP_Y, STRIP_W, STRIP_H, STRIP_R, PANEL_STRIP);
  gLcd.drawRoundRect(STRIP_X + 1, STRIP_Y + 1, STRIP_W - 2, STRIP_H - 2, STRIP_R - 1, PANEL_STRIP);
  gLcd.setFont(HOLD_FONT.font);
  gLcd.setTextSize(HOLD_FONT.sx, HOLD_FONT.sy);
  const int32_t w = gLcd.textWidth("HOLD TO STOP");
  // Font4 caps ink (18 rows) sits 3 rows above the datum's middle: +3 centres
  // it in the 22 rows inside the 2 px border (mock: text ink y 206-223, icon
  // 207-223, 2 px clear of the border; the box is only 26 px tall).
  const int32_t cy = STRIP_Y + STRIP_H / 2;
  const int32_t iconR = 8, iconGap = 8;
  const int32_t left = (SCREEN_W - (2 * iconR + iconGap + w)) / 2;
  gLcd.fillArc(left + iconR, cy, iconR - 3, iconR, -90.0f, 180.0f, PANEL_STRIP);
  gLcd.setTextColor(PANEL_STRIP_TEXT);
  gLcd.setTextDatum(lgfx::textdatum_t::middle_left);
  gLcd.drawString("HOLD TO STOP", left + 2 * iconR + iconGap, cy + 3);
}

// One screen-switch step. Returns true when the switch is complete.
bool switchStep() {
  if (gSwitchStep < CLEAR_BANDS) {
    gLcd.fillRect(0, gSwitchStep * CLEAR_BAND_H, SCREEN_W, CLEAR_BAND_H, PANEL_BG);
    ++gSwitchStep;
    return false;
  }
  if (gTarget == Screen::Main || gTarget == Screen::Summary) {  // the summary's row 2 uses the cells
    gLcd.drawFastHLine(0, RULE_Y, SCREEN_W, PANEL_RULE);
    gLcd.drawFastVLine(CELL_X0 + CELL_W, CELL_Y, CELL_H, PANEL_RULE);
    gLcd.drawFastVLine(CELL_X1 + CELL_W, CELL_Y, CELL_H, PANEL_RULE);
  }
  if (gTarget == Screen::Main) drawHoldStrip();
  gRingShown = -1;
  gScreen = gTarget;
  gSwitching = false;
  for (FieldState& f : gF) f.valid = false;  // every field is pushed again on the new screen
  return true;
}

void recordRender(int64_t t0) {
  const uint32_t us = clampUs(esp_timer_get_time() - t0);
  if (us > gRenderMax) gRenderMax = us;
  if (us > gRenderMaxBoot) gRenderMaxBoot = us;
  ++gRenderCount;
}

void initPalettes() {
  lgfx::LGFX_Sprite& st = gSpr[SP_STATUS];
  setPal(st, PI_BG, RGB_BG);
  setPal(st, PI_FG, RGB_FG);
  setPal(st, PI_GREY, RGB_GREY);
  setPal(st, PI_YELLOW, RGB_YELLOW);
  setPal(st, PI_GREEN, RGB_GREEN);
  setPal(st, PI_ORANGE, RGB_ORANGE);
  for (uint8_t i : {SP_HERO, SP_SPEED, SP_CELL0, SP_CELL1, SP_CELL2}) {
    setPal(gSpr[i], 0, RGB_BG);
    setPal(gSpr[i], 1, RGB_FG);
  }
  lgfx::LGFX_Sprite& rs = gSpr[SP_RING];
  setPal(rs, RI_BG, RGB_BG);
  setPal(rs, RI_TRACK, RGB_RING_TRACK);
  setPal(rs, RI_FILL, RGB_RING_FILL);
  setPal(rs, RI_ICON, RGB_RING_ICON);
}

// Boot self-check of the worst-case strings (they must fit their cells,
// using the next smaller font rather than truncating): the font
// each one gets and its width (value + gap + unit). fits=1 when every width is
// within the field. Absolute extremes (INT64_MAX timers, UINT32_MAX pulses,
// INT32_MAX speed) must fit too (they get the chain's last font).
struct Fit {
  const char* font;
  int32_t w;
};
Fit fitValue(lgfx::LGFX_Sprite& s, const char* v, const char* unit, int32_t gap, int32_t avail) {
  const int32_t uw = unit[0] != '\0' ? widthOf(s, SMALL_FONT, unit) + gap : 0;
  const FontSpec& f = pick(s, VALUE_FONTS, countOf(VALUE_FONTS), v, uw, avail);
  return {f.name, widthOf(s, f, v) + uw};
}

void layoutCheck(int64_t t) {
  bool ok = true;
  // hero: "--:--" / "99:59" left of VALUE_RIGHT; the unit column holds every unit
  lgfx::LGFX_Sprite& hs = gSpr[SP_HERO];
  const int32_t heroAvail = HERO_RIGHT - MARGIN;
  const int32_t wDash = widthOf(hs, HERO_FONTS[0], "--:--");
  const int32_t w9959 = widthOf(hs, HERO_FONTS[0], "99:59");
  useFont(hs, HERO_FONTS[0]);
  const int32_t heroH = hs.fontHeight();
  const bool heroFits = wDash <= heroAvail && w9959 <= heroAvail && heroH <= HERO_H;
  ok = ok && heroFits;
  dbg::status(t, "layout;hero font=%s w_dashes=%" PRId32 " w_99_59=%" PRId32 " h=%" PRId32 " right=%d avail=%" PRId32
                 "x%d fits=%u",
              HERO_FONTS[0].name, wDash, w9959, heroH, static_cast<int>(HERO_RIGHT), heroAvail,
              static_cast<int>(HERO_H), heroFits ? 1u : 0u);
  const int32_t uMi = widthOf(hs, HERO_UNIT_FONT, "/mi"), uKm = widthOf(hs, HERO_UNIT_FONT, "/km");
  const int32_t uMph = widthOf(hs, SMALL_FONT, "mph"), uKmh = widthOf(hs, SMALL_FONT, "km/h");
  const bool unitsFit = uMi <= UNIT_COL_W && uKm <= UNIT_COL_W && uMph <= UNIT_COL_W && uKmh <= UNIT_COL_W;
  ok = ok && unitsFit;
  dbg::status(t, "layout;units x=%d col_w=%d /mi=%" PRId32 " /km=%" PRId32 " (%s) mph=%" PRId32 " km/h=%" PRId32
                 " (%s) fits=%u",
              static_cast<int>(UNIT_X), static_cast<int>(UNIT_COL_W), uMi, uKm, HERO_UNIT_FONT.name, uMph, uKmh,
              SMALL_FONT.name, unitsFit ? 1u : 0u);
  // speed: value left of VALUE_RIGHT (the unit is in the unit column)
  lgfx::LGFX_Sprite& ss = gSpr[SP_SPEED];
  const int32_t spAvail = VALUE_RIGHT - MARGIN;
  const Fit s250 = fitValue(ss, "25.0", "", 0, spAvail);
  char ext[ot::SPEED_STR_SIZE];
  ot::fmtSpeed(ext, sizeof(ext), INT32_MAX, ot::Units::Km);
  const Fit sExt = fitValue(ss, ext, "", 0, spAvail);
  const bool speedFits = s250.w <= spAvail && sExt.w <= spAvail;
  ok = ok && speedFits;
  dbg::status(t, "layout;speed 25.0=%" PRId32 "(%s) extreme %s=%" PRId32 "(%s) avail=%" PRId32 " fits=%u", s250.w,
              s250.font, ext, sExt.w, sExt.font, spAvail, speedFits ? 1u : 0u);
  // cells: value + gap + unit within CELL_W - 2*MARGIN, font as rendered
  lgfx::LGFX_Sprite& cs = gSpr[SP_CELL0];
  const int32_t cAvail = CELL_W - 2 * MARGIN;
  const Fit dMi = fitValue(cs, "99.99", "mi", CELL_GAP, cAvail);
  const Fit dKm = fitValue(cs, "99.99", "km", CELL_GAP, cAvail);
  const Fit d100 = fitValue(cs, "100.00", "km", CELL_GAP, cAvail);
  const Fit t59 = fitValue(cs, "59:59", "", 0, cAvail);
  const Fit t9 = fitValue(cs, "9:59:59", "", 0, cAvail);
  const Fit t10 = fitValue(cs, "10:00:00", "", 0, cAvail);
  char dx[ot::DIST_STR_SIZE], tx[ot::DUR_STR_SIZE];
  ot::fmtDistance(dx, sizeof(dx), UINT32_MAX, ot::Units::Km);
  ot::fmtDuration(tx, sizeof(tx), INT64_MAX);
  const Fit dX = fitValue(cs, dx, "km", CELL_GAP, cAvail);
  const Fit tX = fitValue(cs, tx, "", 0, cAvail);
  const bool cellFits = dMi.w <= cAvail && dKm.w <= cAvail && d100.w <= cAvail && t59.w <= cAvail &&
                        t9.w <= cAvail && t10.w <= cAvail && dX.w <= cAvail && tX.w <= cAvail;
  ok = ok && cellFits;
  dbg::status(t, "layout;cells avail=%" PRId32 " 99.99mi=%" PRId32 "(%s) 99.99km=%" PRId32 "(%s) 100.00km=%" PRId32
                 "(%s) 59:59=%" PRId32 "(%s) 9:59:59=%" PRId32 "(%s) 10:00:00=%" PRId32 "(%s) extremes dist=%" PRId32
                 "(%s) time=%" PRId32 "(%s) fits=%u",
              cAvail, dMi.w, dMi.font, dKm.w, dKm.font, d100.w, d100.font, t59.w, t59.font, t9.w, t9.font, t10.w,
              t10.font, dX.w, dX.font, tX.w, tX.font, cellFits ? 1u : 0u);
  // Hold hero text (x 116..319), units toast (left of the widest state
  // text + glyph), strip label (inside the strip box), idle hint
  const int32_t holdAvail = SCREEN_W - HOLD_TEXT_L - 2 * MARGIN;
  const int32_t wStop = widthOf(hs, HOLD_HEAD_FONT, "Stopping");
  const int32_t wRel = widthOf(hs, HOLD_FONT, "release"), wCan = widthOf(hs, HOLD_FONT, "to cancel");
  lgfx::LGFX_Sprite& st = gSpr[SP_STATUS];
  const int32_t wToastKm = widthOf(st, HOLD_FONT, "units: km"), wToastMi = widthOf(st, HOLD_FONT, "units: mi");
  const int32_t wWarm = widthOf(st, STATUS_FONT, "WARM-UP");
  const int32_t toastAvail = SCREEN_W - 4 - wWarm - GLYPH_GAP - GLYPH_W - 4 - 8;  // 8 px clear of the glyph
  const int32_t wStrip = widthOf(hs, HOLD_FONT, "HOLD TO STOP") + 2 * 8 + 8;
  const int32_t wHint = widthOf(hs, SMALL_FONT, "(tap here to change units)");
  const bool holdFits = wStop <= holdAvail && wRel <= holdAvail && wCan <= holdAvail && wToastKm <= toastAvail &&
                       wToastMi <= toastAvail && wStrip <= STRIP_W - 2 * STRIP_R && wHint <= SCREEN_W - 2 * MARGIN;
  ok = ok && holdFits;
  dbg::status(t, "layout;hold hold_text avail=%" PRId32 " Stopping=%" PRId32 " release=%" PRId32 " to_cancel=%" PRId32
                 " toast avail=%" PRId32 " km=%" PRId32 " mi=%" PRId32 " strip=%" PRId32 "/%d hint=%" PRId32 " fits=%u",
              holdAvail, wStop, wRel, wCan, toastAvail, wToastKm, wToastMi, wStrip,
              static_cast<int>(STRIP_W - 2 * STRIP_R), wHint, holdFits ? 1u : 0u);
  // Idle units lines (Font4), summary titles (Font4), countdown text
  // (Font2) and the summary's row-2 worst cases (value + unit in a cell; the
  // row-1 strings are the cells' distance/duration strings checked above).
  char ul[STR_N];
  idleUnitsLine(ul, sizeof(ul), ot::Units::Km);
  const int32_t wUnitsKm = widthOf(hs, MSG_FONT, ul);
  idleUnitsLine(ul, sizeof(ul), ot::Units::Mi);
  const int32_t wUnitsMi = widthOf(hs, MSG_FONT, ul);
  const int32_t wMsg = widthOf(hs, MSG_FONT, IDLE_MSG);
  const int32_t wRearm = widthOf(hs, MSG_FONT, IDLE_MSG_REARM);
  const int32_t wT1 = widthOf(hs, MSG_FONT, "Workout complete"), wT2 = widthOf(hs, MSG_FONT, "Ended: no movement");
  const int32_t wCd = widthOf(ss, SMALL_FONT, "back to start in 59:59");
  char fw[STR_N];
  snprintf(fw, sizeof(fw), "fw %s build %08" PRIx32 " DIRTY", OT_FW_VERSION, hw::BUILD_ID);
  const int32_t wFw = widthOf(ss, SMALL_FONT, fw);
  const Fit pMi = fitValue(cs, "99:59", "/mi", CELL_GAP, cAvail);
  const Fit pKm = fitValue(cs, "--:--", "/km", CELL_GAP, cAvail);
  const Fit vKm = fitValue(cs, "99.9", "km/h", CELL_GAP, cAvail);
  const Fit vX = fitValue(cs, ext, "km/h", CELL_GAP, cAvail);
  const int32_t full = SCREEN_W - 2 * MARGIN;
  const bool screensFits = wUnitsKm <= full && wUnitsMi <= full && wMsg <= full && wRearm <= full && wT1 <= full && wT2 <= full &&
                       wCd <= full && wFw <= full && pMi.w <= cAvail && pKm.w <= cAvail && vKm.w <= cAvail &&
                       vX.w <= cAvail;
  ok = ok && screensFits;
  dbg::status(t, "layout;screens avail=%" PRId32 " units_km=%" PRId32 " units_mi=%" PRId32 " msg=%" PRId32 " rearm=%" PRId32
                 " titles=%" PRId32
                 "/%" PRId32 " countdown=%" PRId32 " fw=%" PRId32 " cell avail=%" PRId32 " 99:59/mi=%" PRId32
                 "(%s) --:--/km=%" PRId32 "(%s) 99.9km/h=%" PRId32 "(%s) extreme=%" PRId32 "(%s) fits=%u",
              full, wUnitsKm, wUnitsMi, wMsg, wRearm, wT1, wT2, wCd, wFw, cAvail, pMi.w, pMi.font, pKm.w, pKm.font, vKm.w,
              vKm.font, vX.w, vX.font, screensFits ? 1u : 0u);
  if (!ok) dbg::status(t, "WARNING layout: a worst-case string does not fit its field (see layout; lines)");
}

void startSwitch(Screen to) {
  gTarget = to;
  gSwitching = true;
  gSwitchStep = 0;
  ++gSwitches;
}

}  // namespace

// ---------------------------------------------------------------------------
bool begin(const Frame& first) {
  int64_t t = esp_timer_get_time();
  dbg::status(t, "display;main renderer panel=%s rotation=%u spi_write_hz=%" PRIu32 " frame_us=%" PRId64
                 " pass_budget_us=%" PRId64 " units=%s",
              LGFX_CYD::PANEL_NAME, static_cast<unsigned>(hw::DISPLAY_ROTATION), hw::TFT_SPI_WRITE_HZ, FRAME_US,
              PASS_BUDGET_US, ot::unitsName(first.units));
  dbg::status(t, "heap;point=boot");
  dbg::heap(t);

  // Boot order: 1. LED pins HIGH (app_setup, first)
  gLcd.setBrightness(0);                        // 2.
  gBlApplied = 0;
  gBlTarget = hw::BL_FULL;                      // the first frame is drawn dark, then raised (6.)
  const int64_t ti = esp_timer_get_time();
  const bool initOk = gLcd.init();              // 3.
  const bool dmaOk = gLcd.reserveDmaBuffers();  //    right after init: no push resizes them later
  gLcd.setRotation(hw::DISPLAY_ROTATION);
  t = esp_timer_get_time();
  dbg::status(t, "display;init ok=%u us=%" PRIu32 " w=%ld h=%ld rotation=%u dma_flip_bytes=%" PRIu32 "x2 dma_ok=%u",
              initOk ? 1u : 0u, clampUs(t - ti), static_cast<long>(gLcd.width()), static_cast<long>(gLcd.height()),
              static_cast<unsigned>(gLcd.getRotation()), LGFX_CYD::DMA_FLIP_BYTES, dmaOk ? 1u : 0u);
  dbg::status(t, "heap;point=after_lcd_init");
  dbg::heap(t);

  // 4. sprites, once, never freed (before BLE: NimBLE is initialised after this)
  bool sprOk = true;
  const char* failed = "-";
  gSpriteBytes = 0;
  for (uint8_t i = 0; i < SP_N; ++i) {
    gSpr[i].setPsram(false);
    gSpr[i].setColorDepth(SPRITES[i].bpp);
    if (gSpr[i].createSprite(SPRITES[i].w, SPRITES[i].h) == nullptr) {
      sprOk = false;
      failed = SPRITES[i].name;
      break;
    }
    gSpriteBytes += gSpr[i].bufferLength();
  }
  t = esp_timer_get_time();
  dbg::status(t, "sprites;ok=%u n=%u bytes=%" PRIu32 " failed=%s set=status320x24x4 hero320x96x1 speed320x30x1"
                 " cell106x46x1*3 ring100x100x4",
              sprOk ? 1u : 0u, static_cast<unsigned>(SP_N), gSpriteBytes, failed);
  dbg::status(t, "heap;point=after_sprites");
  dbg::heap(t);
  {
    const uint32_t fr = static_cast<uint32_t>(esp_get_free_heap_size());
    const uint32_t lg = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    dbg::status(t, "heap;floors free=%" PRIu32 ">=%" PRIu32 " largest=%" PRIu32 ">=%" PRIu32 " ok=%u (before BLE)", fr,
                HEAP_FLOOR_FREE, lg, HEAP_FLOOR_LARGEST, (fr >= HEAP_FLOOR_FREE && lg >= HEAP_FLOOR_LARGEST) ? 1u : 0u);
  }
  gReady = initOk && dmaOk && sprOk;
  if (!gReady) {
    dbg::status(t, "ERROR display not ready (init_ok=%u dma_ok=%u sprites_ok=%u failed=%s): renderer disabled",
                initOk ? 1u : 0u, dmaOk ? 1u : 0u, sprOk ? 1u : 0u, failed);
    gLcd.setBrightness(hw::BL_FULL);
    gBlApplied = hw::BL_FULL;
    return false;
  }
  initPalettes();
  layoutCheck(t);

  // 5. the first frame, drawn completely (bounded steps, as in loop())
  const int64_t tf = esp_timer_get_time();
  frame(first);
  uint32_t passes = 0;
  while (service(esp_timer_get_time()) != 0) ++passes;
  t = esp_timer_get_time();
  dbg::status(t, "display;first_frame screen=%s passes=%" PRIu32 " us=%" PRIu32 " boot_render_max_us=%" PRIu32,
              screenName(gScreen), passes, clampUs(t - tf), gRenderMax);
  // The boot passes are logged above (and stay in gRenderMaxBoot for 'd');
  // the first G line covers loop() passes only.
  gRenderMax = 0;
  gRenderCount = 0;

  // 6. backlight full (7. NimBLEDevice::init comes after this)
  gLcd.setBrightness(hw::BL_FULL);
  gBlApplied = hw::BL_FULL;
  dbg::line(dbg::LV_K, "L,%" PRId64 ",%u", esp_timer_get_time(), static_cast<unsigned>(hw::BL_FULL));
  gNextFrame = esp_timer_get_time() + FRAME_US;
  return true;
}

bool frameDue(int64_t now) { return gReady && now >= gNextFrame; }

void frame(const Frame& f) {
  if (!gReady) return;
  gNextFrame += FRAME_US;
  if (gNextFrame <= f.t) gNextFrame = f.t + FRAME_US;  // after a long block: no burst of frames
  ++gFrames;
  gLast = f;
  // Filters see every frame, whatever the screen (their state stays current).
  gPace.update(f.t, f.speedMmps, f.units, ot::paceShownInState(f.state));
  gSpeed.update(f.t, f.speedMmps, f.units);
  // The screen from the state (ot::screenForState); SUMMARY needs the
  // frozen record in the frame (the app always passes it in SUMMARY).
  const ot::UiScreen ui = ot::screenForState(f.state);
  const Screen want = ui == ot::UiScreen::Main ? Screen::Main
                      : (ui == ot::UiScreen::Summary && f.hasSummary ? Screen::Summary : Screen::Idle);
  if (want != (gSwitching ? gTarget : gScreen)) startSwitch(want);
  if (want == Screen::Main) buildMain(f);
  else if (want == Screen::Summary) buildSummary(f);
  else buildIdle(f);
  if (gBlTarget == hw::BL_OFF) ++gDarkFrames;  // computed, never pushed while OFF
  if (gForce)
    for (FieldState& st : gF) st.valid = false;
}

// One bounded drawing pass: a screen-switch step, or the dirty
// fields (+ the hold ring). service() wraps it with the backlight rules.
static uint32_t drawPass(int64_t now) {
  if (gSwitching) {
    const int64_t t0 = esp_timer_get_time();
    gLcd.startWrite();
    switchStep();
    gLcd.endWrite();
    recordRender(t0);
    return 1;
  }
  if (gScreen == Screen::None) return 0;
  bool any = false;
  for (const FieldState& f : gF) any = any || dirty(f);
  if (!any && !ringDue(now)) return 0;
  const int64_t t0 = esp_timer_get_time();
  gLcd.startWrite();
  bool pushed = false, heroPushed = false;
  for (uint8_t i = 0; i < F_N; ++i) {
    if (!dirty(gF[i])) continue;
    if (pushed && esp_timer_get_time() - t0 >= PASS_BUDGET_US) break;
    pushField(static_cast<Field>(i));
    pushed = true;
    if (i == F_HERO) {
      heroPushed = true;
      break;  // the hero (~14 ms) always goes alone
    }
  }
  // Ring (~5 ms): after the fields, never in the hero's pass, and only
  // while the pass has used < PASS_BUDGET_US.
  if (!heroPushed && ringDue(now) && (!pushed || esp_timer_get_time() - t0 < PASS_BUDGET_US)) pushRing(now);
  gLcd.endWrite();
  recordRender(t0);
  return 1;
}

// ---------------------------------------------------------------------------
// Backlight
// ---------------------------------------------------------------------------
static void applyBacklight(uint8_t level) {
  gLcd.setBrightness(level);  // LovyanGFX Light_PWM: the only driver of GPIO 21
  gBlApplied = level;
  ++gBlChanges;
  dbg::line(dbg::LV_K, "L,%" PRId64 ",%u", esp_timer_get_time(), static_cast<unsigned>(level));
}

// The wake frame is completely on the panel: no screen switch in progress and
// every field the screen uses pushed since the wake invalidated them.
static bool wakeFrameDone() {
  if (gSwitching || gScreen == Screen::None) return false;
  for (const FieldState& f : gF)
    if (f.want.used && !f.valid) return false;
  return true;
}

uint32_t service(int64_t now) {
  if (!gReady) return 0;
  if (gBlTarget == hw::BL_OFF) return 0;  // OFF: skip rendering (frame() keeps computing)
  const uint32_t drew = drawPass(now);
  if (gBlWake && wakeFrameDone()) {  // the fresh frame is on the panel: now raise the brightness
    gBlWake = false;
    applyBacklight(gBlTarget);
    const int64_t d = esp_timer_get_time() - gBlWakeT0;
    const uint32_t us = d < 0 ? 0u : (d > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(d));
    if (us > gWakeMaxUs) gWakeMaxUs = us;
  }
  return drew;
}

void setBacklight(int64_t now, uint8_t level) {
  if (level == gBlTarget) return;
  gBlTarget = level;
  if (!gReady) {  // no renderer (panel/sprites failed): just the brightness
    applyBacklight(level);
    return;
  }
  if (level == hw::BL_OFF) {
    gBlWake = false;
    applyBacklight(level);
    return;
  }
  if (gBlApplied == hw::BL_OFF) {
    if (gBlWake) return;  // a wake redraw is already in progress: it applies the new target
    // Wake from OFF: the panel holds a frame from before OFF. Redraw every
    // field from a frame taken now, then raise the brightness (service()).
    gBlWakeT0 = esp_timer_get_time();
    gBlWake = true;
    ++gWakes;
    for (FieldState& f : gF) f.valid = false;
    gRingShown = -1;
    gNextFrame = now;  // frameDue() in this same pass
    return;
  }
  applyBacklight(level);  // FULL <-> DIM: the panel is current
}

uint8_t backlight() { return gBlApplied; }
bool backlightWakePending() { return gBlWake; }

void setForceRedraw(bool on) { gForce = on; }

void setHold(int64_t now, bool active, int32_t permille) {
  if (active != gHold) {
    gHold = active;
    if (active) {
      ++gHolds;
      gRingDrawn = -1;  // a new hold: the ring sprite starts empty
      gRingShown = -1;
    }
    gNextFrame = now;  // the hero switches (pace <-> hold text) in this pass
  }
  gHoldPermille = active ? permille : 0;
}

void unitsChanged(int64_t now) {
  gToastUntil = now + ot::UNITS_TOAST_MS * 1000;
  ++gToasts;
  for (FieldState& st : gF) st.valid = false;  // force-dirty: every field re-rendered in the new units
  gRingShown = -1;
  gNextFrame = now;
}

bool touchRead(TouchRaw& out) {
  out = TouchRaw{};
  if (!gReady) return false;  // the panel (and its touch driver) failed to init: no touch
  lgfx::touch_point_t tp;
  const uint_fast8_t n = gLcd.getTouchRaw(&tp, 1);
  out.xRaw = static_cast<uint16_t>(tp.x < 0 ? 0 : tp.x);
  out.yRaw = static_cast<uint16_t>(tp.y < 0 ? 0 : tp.y);
  out.z = tp.size;
  out.valid = n > 0 && tp.size >= hw::TOUCH_Z_MIN;
  if (!out.valid) return false;
  lgfx::touch_point_t c = tp;  // convert a copy: the raw values stay for the T line
  gLcd.convertRawXY(&c, 1);
  out.x = static_cast<int16_t>(c.x < 0 ? 0 : (c.x >= SCREEN_W ? SCREEN_W - 1 : c.x));
  out.y = static_cast<int16_t>(c.y < 0 ? 0 : (c.y >= SCREEN_H ? SCREEN_H - 1 : c.y));
  return true;
}

void takeStats(uint32_t& renderMaxUs, uint32_t& renderCount, uint32_t& touchLateMaxUs) {
  renderMaxUs = gRenderMax;
  renderCount = gRenderCount;
  touchLateMaxUs = 0;  // touch poll lateness comes from hw_touch
  gRenderMax = 0;
  gRenderCount = 0;
}

void dump(int64_t now) {
  dbg::status(now,
              "dump;display ready=%u panel=%s screen=%s switching=%u frames=%" PRIu32 " switches=%" PRIu32
              " force=%u sprites_bytes=%" PRIu32 " render_max_us=%" PRIu32 " render_count=%" PRIu32
              " render_max_boot_us=%" PRIu32,
              gReady ? 1u : 0u, LGFX_CYD::PANEL_NAME, screenName(gScreen), gSwitching ? 1u : 0u, gFrames, gSwitches,
              gForce ? 1u : 0u, gSpriteBytes, gRenderMax, gRenderCount, gRenderMaxBoot);
  char pace[ot::PACE_STR_SIZE], spd[ot::SPEED_STR_SIZE];
  gPace.format(pace, sizeof(pace));
  gSpeed.format(spd, sizeof(spd));
  dbg::status(now,
              "dump;display frame t=%" PRId64 " state=%s speed_mmps=%" PRId32 " units=%s pace=%s pace_hyst=%u"
              " speed=%s",
              gLast.t, ot::workoutStateName(gLast.state), gLast.speedMmps, ot::unitsName(gLast.units), pace,
              gPace.hysteresisVisible() ? 1u : 0u, spd);
  dbg::status(now,
              "dump;display hold hold=%u permille=%" PRId32 " holds=%" PRIu32 " ring_drawn=%" PRId32 " ring_shown=%" PRId32
              " ring_pushes=%" PRIu32 " ring_max_us=%" PRIu32 " toast_until=%" PRId64 " toasts=%" PRIu32,
              gHold ? 1u : 0u, gHoldPermille, gHolds, gRingDrawn, gRingShown, gRingPushes, gRingMaxUs, gToastUntil,
              gToasts);
  dbg::status(now,
              "dump;display backlight bl_target=%u bl_applied=%u wake_pending=%u bl_changes=%" PRIu32 " wakes=%" PRIu32
              " wake_redraw_max_us=%" PRIu32 " dark_frames=%" PRIu32 " summary=%s|%s|%s|%s|%s|%s",
              static_cast<unsigned>(gBlTarget), static_cast<unsigned>(gBlApplied), gBlWake ? 1u : 0u, gBlChanges,
              gWakes, gWakeMaxUs, gDarkFrames, gSum.distance, gSum.total, gSum.moving, gSum.avgPace, gSum.avgSpeed,
              gSum.topSpeed);
  for (uint8_t i = 0; i < F_N; ++i)
    dbg::status(now, "dump;field %s used=%u valid=%u dirty=%u dim=%u pushes=%" PRIu32 " shown=%s|%s",
                FIELD_NAMES[i], static_cast<unsigned>(gF[i].want.used), gF[i].valid ? 1u : 0u,
                dirty(gF[i]) ? 1u : 0u, gF[i].shownDim ? 1u : 0u, gF[i].pushes, gF[i].shown.a, gF[i].shown.b);
}

}  // namespace disp

#endif  // OT_BRINGUP
