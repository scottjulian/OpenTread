/*
 * pulse_gen: independent bench pulse generator for the OpenTread head unit
 * (loopback / stress testing without a treadmill).
 * ------------------------------------------------------------------------
 * Runs on a SECOND plain ESP32 dev board (ESP32-WROOM-32, CP2102, FQBN
 * esp32:esp32:esp32), never on the CYD. It emulates the treadmill's reed
 * switch: each "closure" pulls GEN_PIN to GND (open-drain, the line is never
 * driven HIGH), with a bounce burst on closure and a short chatter on
 * release, at a programmed rate. The CYD's GPIO 27 pull-up provides the HIGH
 * level, exactly as with the real reed. It reports its OWN count of
 * generated closures over its own serial port, so the pass rule
 * "CYD P count == generated count" compares two independent devices.
 *
 * Wiring (see README.md): GEN_PIN (GPIO 25) -> CYD GPIO 27 (CN1 signal),
 * GND -> CYD GND (common ground is required). Treadmill harness unplugged.
 *
 * Serial (115200 8N1, one line per event, '\n' terminated; same letters as
 * the head unit's protocol so tools/capture.py / replay.py can parse it):
 *   S,<t_us>,pulse_gen v1 pin=25 mode=open_drain ...      boot banner
 *   S,<t_us>,cmd;<text>          accepted command (echo; ',' shown as ';', e.g. cmd;g10;600)
 *   S,<t_us>,cmd_err;<text>      rejected command, no effect
 *   S,<t_us>,gen_start;rate_mhz=<r>;count=<n>;period_us=<p>;dwell_us=<d>;bounce=<0|1>
 *   C,<t_us>,<k>                 closure k (1..n) started; t_us = its SCHEDULED first
 *                                falling edge (the real edge follows within the loop latency)
 *   S,<t_us>,gen_done;<generated>;edges=<e>;aborted=<0|1>;print_drop=<d>;max_late_us=<l>
 * t_us is THIS board's esp_timer clock (not comparable with the CYD's).
 * <generated> = closures whose first falling edge was emitted (an aborted
 * train counts the closure in progress: the head unit counts a closure at its
 * first falling edge). print_drop = C lines dropped because the TX buffer
 * was full (the count in gen_done is never affected). max_late_us = the
 * worst lateness of any edge versus its schedule; edges within one closure
 * are 250..900 us apart and the open time before a closure is >= 14 ms at
 * 40/s, so a value of a few hundred us leaves the waveform valid; a value
 * near the 5 ms re-arm time means the run is not trustworthy.
 *
 * Commands (one per line, max 24 chars, '\r' ignored):
 *   g<rate>,<count>   start a train: <rate> closures/s (0.1..40, up to 3
 *                     decimals, e.g. g10,600 or g2.5,100), <count> 1..10000000
 *   g0                stop now (releases the line)
 *   b0 / b1           bounce bursts off / on (default on); not while running
 *   ?                 status
 *
 * Waveform of one closure k (offsets from its scheduled start
 * T_k = T_0 + k * 10^9 / rate_mHz, integer us, no drift):
 *   closure burst (75%): L@0, H@400+j, L@900+j            (3 edges)
 *                 (25%): L@0, H@300+j, L@800+j, H@1700+j, L@2600+j  (5 edges)
 *                 -> mean 3.5 edges within 3 ms; j = seeded jitter 0..150 us
 *   dwell: closed until max(40% of the period, 4 ms)
 *   release (50%): H                                       (1 edge)
 *           (50%): H, L@+250, H@+600                       (chatter, 3 edges)
 * Every open blip inside a burst or chatter is < 5 ms, so the head unit's filter
 * (5 ms release re-arm, 20 ms lockout) must count exactly 1 pulse per
 * closure. The open time before the next closure is >= 8 ms at 40/s, and
 * closure-to-closure >= 25 ms > the 20 ms lockout, so every closure is
 * countable. The PRNG is re-seeded at each start: every train with the same
 * rate and count is the identical waveform.
 */

#include <Arduino.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ----------------------------- Config -----------------------------------
static const uint8_t  GEN_PIN          = 25;          // -> CYD GPIO 27 (CN1)
static const uint32_t SERIAL_BAUD      = 115200;
static const uint32_t RATE_MIN_MHZ     = 100;         // 0.1 closures/s
static const uint32_t RATE_MAX_MHZ     = 40000;       // 40 closures/s (25 ms period)
static const uint32_t COUNT_MAX        = 10000000;
static const uint32_t DWELL_PERMILLE   = 400;         // closed for 40% of the period...
static const uint32_t DWELL_MIN_US     = 4000;        // ...but at least 4 ms (burst ends <= 2.75 ms)
static const uint32_t START_DELAY_US   = 50000;       // line idle-open before the first closure
static const uint32_t JITTER_MAX_US    = 150;
static const uint32_t SEED             = 0x2545F491u;
static const int64_t  YIELD_IF_US      = 2500;        // vTaskDelay(1) when the next edge is further away
static const size_t   CMD_MAX          = 24;
static const size_t   OUT_LINE_MAX     = 160;
static const int      MAX_STEPS        = 8;           // 5 closure + 3 release edges

// ----------------------------- State ------------------------------------
struct Step { uint32_t offsetUs; uint8_t level; };

static bool     gBounce     = true;
static bool     gRunning    = false;
static uint32_t gRateMhz    = 0;
static uint32_t gCount      = 0;
static uint32_t gNext       = 0;      // index of the closure being generated (0-based)
static uint32_t gGenerated  = 0;      // closures whose first falling edge was emitted
static uint32_t gEdges      = 0;      // level changes emitted this train
static uint32_t gPrintDrop  = 0;
static int64_t  gMaxLateUs  = 0;      // worst (actual - scheduled) edge time this train
static int64_t  gT0         = 0;
static int64_t  gClosureT   = 0;      // scheduled start of the current closure
static Step     gSteps[MAX_STEPS];
static int      gNumSteps   = 0;
static int      gStepIdx    = 0;
static uint8_t  gLevel      = 1;      // current output: 1 = released (open), 0 = pulled low
static uint32_t gRng        = SEED;
static char     gCmd[CMD_MAX + 1];
static size_t   gCmdLen     = 0;
static bool     gCmdOverflow = false;

// ----------------------------- Output -----------------------------------
// One formatted line per Serial.write. Event lines are dropped (and counted)
// rather than blocking when the TX buffer is short, so printing can never
// stretch the waveform; status lines (banner, start, done, cmd echo) wait.
static bool emit(bool mayDrop, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static bool emit(bool mayDrop, const char* fmt, ...) {
  char buf[OUT_LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return false;
  size_t len = static_cast<size_t>(n);
  if (len > sizeof(buf) - 2) len = sizeof(buf) - 2;
  buf[len++] = '\n';
  if (mayDrop && static_cast<size_t>(Serial.availableForWrite()) < len) {
    gPrintDrop++;
    return false;
  }
  Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
  return true;
}

static void setLine(uint8_t level) {
  if (level == gLevel) return;
  gpio_set_level(static_cast<gpio_num_t>(GEN_PIN), level);
  gLevel = level;
  gEdges++;
}

static uint32_t rng() {               // xorshift32
  uint32_t x = gRng;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  gRng = x;
  return x;
}

static uint32_t jitter() { return rng() % (JITTER_MAX_US + 1); }

static int64_t closureStart(uint32_t k) {
  return gT0 + static_cast<int64_t>((static_cast<uint64_t>(k) * 1000000000ull) / gRateMhz);
}

static uint32_t periodUs() { return static_cast<uint32_t>(1000000000ull / gRateMhz); }

static uint32_t dwellUs() {
  uint32_t d = static_cast<uint32_t>((static_cast<uint64_t>(periodUs()) * DWELL_PERMILLE) / 1000u);
  return d < DWELL_MIN_US ? DWELL_MIN_US : d;
}

// Build the edge list of closure gNext (offsets from gClosureT, increasing).
static void planClosure() {
  int n = 0;
  gSteps[n++] = {0, 0};
  if (gBounce) {
    if ((rng() & 3u) != 0u) {         // 75%: 3-edge burst
      gSteps[n++] = {400 + jitter(), 1};
      gSteps[n++] = {900 + jitter(), 0};
    } else {                          // 25%: 5-edge burst
      gSteps[n++] = {300 + jitter(), 1};
      gSteps[n++] = {800 + jitter(), 0};
      gSteps[n++] = {1700 + jitter(), 1};
      gSteps[n++] = {2600 + jitter(), 0};
    }
  }
  const uint32_t rel = dwellUs();
  gSteps[n++] = {rel, 1};
  if (gBounce && (rng() & 1u) != 0u) {  // 50%: release chatter
    gSteps[n++] = {rel + 250, 0};
    gSteps[n++] = {rel + 600, 1};
  }
  gNumSteps = n;
  gStepIdx = 0;
}

static void finishTrain(int64_t now, bool aborted) {
  setLine(1);
  gRunning = false;
  emit(false, "S,%" PRId64 ",gen_done;%" PRIu32 ";edges=%" PRIu32 ";aborted=%u;print_drop=%" PRIu32
       ";max_late_us=%" PRId64, now, gGenerated, gEdges, aborted ? 1u : 0u, gPrintDrop, gMaxLateUs);
}

static void startTrain(int64_t now, uint32_t rateMhz, uint32_t count) {
  gRateMhz = rateMhz;
  gCount = count;
  gNext = 0;
  gGenerated = 0;
  gEdges = 0;
  gPrintDrop = 0;
  gMaxLateUs = 0;
  gRng = SEED;
  setLine(1);
  gEdges = 0;
  gT0 = now + START_DELAY_US;
  gClosureT = closureStart(0);
  planClosure();
  gRunning = true;
  emit(false, "S,%" PRId64 ",gen_start;rate_mhz=%" PRIu32 ";count=%" PRIu32 ";period_us=%" PRIu32
       ";dwell_us=%" PRIu32 ";bounce=%u", now, gRateMhz, gCount, periodUs(), dwellUs(), gBounce ? 1u : 0u);
}

// Emit every edge that is due. Returns the time of the next edge.
static int64_t runTrain() {
  int64_t now = esp_timer_get_time();
  while (gRunning) {
    const int64_t due = gClosureT + static_cast<int64_t>(gSteps[gStepIdx].offsetUs);
    now = esp_timer_get_time();
    if (now < due) return due;
    setLine(gSteps[gStepIdx].level);
    if (now - due > gMaxLateUs) gMaxLateUs = now - due;
    if (gStepIdx == 0) {
      gGenerated++;
      emit(true, "C,%" PRId64 ",%" PRIu32, due, gGenerated);
    }
    if (++gStepIdx < gNumSteps) continue;
    // closure complete
    if (++gNext >= gCount) {
      finishTrain(now, false);
      break;
    }
    gClosureT = closureStart(gNext);
    planClosure();
  }
  return now;
}

// ----------------------------- Commands ---------------------------------
// Parses "<int>[.<1..3 digits>]" into milli-units. Returns false on junk.
static bool parseMilli(const char* s, const char* end, uint32_t& out) {
  uint64_t v = 0;
  int frac = -1;
  if (s == end) return false;
  for (const char* p = s; p < end; ++p) {
    if (*p == '.') {
      if (frac >= 0 || p == s || p + 1 == end) return false;
      frac = 0;
      continue;
    }
    if (*p < '0' || *p > '9') return false;
    if (frac >= 3) return false;
    v = v * 10u + static_cast<uint64_t>(*p - '0');
    if (frac >= 0) frac++;
    if (v > 100000000ull) return false;
  }
  for (int f = (frac < 0 ? 0 : frac); f < 3; ++f) v *= 10u;
  out = static_cast<uint32_t>(v);
  return true;
}

static bool parseUint(const char* s, const char* end, uint32_t& out) {
  uint64_t v = 0;
  if (s == end) return false;
  for (const char* p = s; p < end; ++p) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10u + static_cast<uint64_t>(*p - '0');
    if (v > 0xFFFFFFFFull) return false;
  }
  out = static_cast<uint32_t>(v);
  return true;
}

// Echo text: S lines never carry a ',' inside their text (log-format convention).
static const char* echoText(const char* c) {
  static char e[CMD_MAX + 1];
  size_t i = 0;
  for (; c[i] != '\0' && i < CMD_MAX; ++i) e[i] = (c[i] == ',') ? ';' : c[i];
  e[i] = '\0';
  return e;
}

static void handleCommand(int64_t now, const char* c) {
  const size_t len = strlen(c);
  if (strcmp(c, "g0") == 0) {
    emit(false, "S,%" PRId64 ",cmd;%s", now, echoText(c));
    if (gRunning) finishTrain(now, true);
    return;
  }
  if (strcmp(c, "?") == 0) {
    emit(false, "S,%" PRId64 ",cmd;%s", now, echoText(c));
    emit(false, "S,%" PRId64 ",status;running=%u;generated=%" PRIu32 ";count=%" PRIu32 ";rate_mhz=%" PRIu32
         ";bounce=%u;line=%u", now, gRunning ? 1u : 0u, gGenerated, gCount, gRateMhz, gBounce ? 1u : 0u,
         static_cast<unsigned>(gLevel));
    return;
  }
  if ((strcmp(c, "b0") == 0 || strcmp(c, "b1") == 0) && !gRunning) {
    gBounce = (c[1] == '1');
    emit(false, "S,%" PRId64 ",cmd;%s", now, echoText(c));
    return;
  }
  if (len >= 4 && c[0] == 'g' && !gRunning) {
    const char* comma = strchr(c, ',');
    uint32_t rate = 0, count = 0;
    if (comma != nullptr && parseMilli(c + 1, comma, rate) && parseUint(comma + 1, c + len, count) &&
        rate >= RATE_MIN_MHZ && rate <= RATE_MAX_MHZ && count >= 1 && count <= COUNT_MAX) {
      emit(false, "S,%" PRId64 ",cmd;%s", now, echoText(c));
      startTrain(now, rate, count);
      return;
    }
  }
  emit(false, "S,%" PRId64 ",cmd_err;%s", now, echoText(c));
}

static void pollSerial(int64_t now) {
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      gCmd[gCmdLen] = '\0';
      if (gCmdOverflow) emit(false, "S,%" PRId64 ",cmd_err;too_long", now);
      else if (gCmdLen > 0) handleCommand(now, gCmd);
      gCmdLen = 0;
      gCmdOverflow = false;
      continue;
    }
    if (gCmdLen < CMD_MAX && ch >= 0x20 && ch < 0x7f) gCmd[gCmdLen++] = static_cast<char>(ch);
    else gCmdOverflow = true;          // too long or non-printable: discard the whole line
  }
}

// ----------------------------- Arduino ----------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  // Open-drain: level 1 = released (high-Z, the CYD pull-up makes it HIGH),
  // level 0 = pulled to GND. No internal pull-up on this side.
  gpio_set_level(static_cast<gpio_num_t>(GEN_PIN), 1);
  pinMode(GEN_PIN, OUTPUT_OPEN_DRAIN);
  gpio_set_level(static_cast<gpio_num_t>(GEN_PIN), 1);
  gLevel = 1;
  delay(200);   // setup() only: let the USB-serial host attach
  const int64_t now = esp_timer_get_time();
  emit(false, "S,%" PRId64 ",pulse_gen v1 pin=%u mode=open_drain baud=%" PRIu32 " bounce=%u rate_min_mhz=%" PRIu32
       " rate_max_mhz=%" PRIu32 " dwell_permille=%" PRIu32 " dwell_min_us=%" PRIu32, now,
       static_cast<unsigned>(GEN_PIN), SERIAL_BAUD, gBounce ? 1u : 0u, RATE_MIN_MHZ, RATE_MAX_MHZ, DWELL_PERMILLE,
       DWELL_MIN_US);
  emit(false, "S,%" PRId64 ",help (';' = ','): g<rate>;<count> start (e.g. g10;600) | g0 stop | b0/b1 bounce | ? status", now);
}

void loop() {
  const int64_t now = esp_timer_get_time();
  pollSerial(now);
  int64_t next = now + YIELD_IF_US + 1;
  if (gRunning) next = runTrain();
  if (!gRunning || next - esp_timer_get_time() > YIELD_IF_US) vTaskDelay(1);
}
