// debug_serial.cpp - see debug_serial.h.
#include "debug_serial.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <esp_system.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include "config_hw.h"

namespace dbg {

namespace {

// Line buffers (static: no allocation, single writer).
constexpr size_t LINE_CAP = 480;   // longest line incl. the newline (the banner is ~340); longer output is truncated
constexpr size_t TEXT_CAP = 448;   // S text before sanitising
char gLine[LINE_CAP];
char gText[TEXT_CAP];

// TX room rule: dbg::tx::roomFor() in debug_serial.h (the reasoning is there;
// in short, the reported space is the sum of two ring regions, the data must
// fit in the larger one, and the wrap waste can be up to one line long).

uint8_t  gVerb = 2;
uint32_t gDrop = 0;

// Inbound command line.
char     gCmd[hw::CMD_MAX_CHARS + 1];
uint32_t gCmdLen  = 0;
bool     gCmdLong = false;

// esp_intr_dump() target: a cookie stream that appends into gDump. The FILE
// and its lock are created once in begin() (setup); the stream is fully
// buffered with a static buffer, so printing allocates nothing later.
constexpr size_t DUMP_MAX = 3584;
char   gDump[DUMP_MAX];
size_t gDumpLen   = 0;
bool   gDumpTrunc = false;
char   gDumpStreamBuf[256];
FILE*  gDumpStream = nullptr;

ssize_t dumpWrite(void* /*cookie*/, const char* buf, size_t n) {
  const size_t room = DUMP_MAX - gDumpLen;
  const size_t k = n < room ? n : room;
  if (k < n) gDumpTrunc = true;
  memcpy(gDump + gDumpLen, buf, k);
  gDumpLen += k;
  return static_cast<ssize_t>(n);  // claim everything, so stdio never retries
}

void writeOrDrop(const char* s, size_t n) {
  if (!tx::roomFor(Serial.availableForWrite(), n)) {
    ++gDrop;
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t*>(s), n);
}

void vline(uint8_t level, const char* fmt, va_list ap) {
  if (level > gVerb) return;
  const int r = vsnprintf(gLine, LINE_CAP - 1, fmt, ap);  // leave room for '\n'
  if (r < 0) return;
  size_t n = static_cast<size_t>(r);
  if (n > LINE_CAP - 2) n = LINE_CAP - 2;                  // truncated: keep the line terminated
  gLine[n++] = '\n';
  writeOrDrop(gLine, n);
}

void sanitize(char* s) {
  for (; *s != '\0'; ++s) {
    const unsigned char c = static_cast<unsigned char>(*s);
    if (c == ',') *s = ';';
    else if (c == '\n' || c == '\r') *s = ' ';
    else if (c < 0x20 || c >= 0x7f) *s = '?';
  }
}

bool contains(const char* hay, size_t n, const char* needle) {
  const size_t m = strlen(needle);
  if (m == 0 || m > n) return false;
  for (size_t i = 0; i + m <= n; ++i)
    if (memcmp(hay + i, needle, m) == 0) return true;
  return false;
}

}  // namespace

void begin(uint8_t verbosity) {
  setVerbosity(verbosity);
  Serial.setTxBufferSize(hw::DEBUG_TX_BUFFER);  // must precede begin()
  Serial.begin(hw::SERIAL_BAUD);
  esp_log_level_set("*", ESP_LOG_NONE);         // no IDF log output tearing our lines
  Serial.write(reinterpret_cast<const uint8_t*>("\n"), 1);  // end any boot-ROM garbage line

  cookie_io_functions_t io = {nullptr, dumpWrite, nullptr, nullptr};
  gDumpStream = fopencookie(nullptr, "w", io);
  if (gDumpStream != nullptr) {
    setvbuf(gDumpStream, gDumpStreamBuf, _IOFBF, sizeof(gDumpStreamBuf));
    // Warm-up: newlib creates the stream's lock and any per-task stdio state
    // lazily on first use; do it here, in setup().
    fputs(" ", gDumpStream);
    fflush(gDumpStream);
  }
  gDumpLen = 0;
  gDumpTrunc = false;
}

uint8_t verbosity() { return gVerb; }
void setVerbosity(uint8_t v) { gVerb = v > LV_MAX ? static_cast<uint8_t>(LV_MAX) : v; }
uint32_t droppedLines() { return gDrop; }

void line(uint8_t level, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vline(level, fmt, ap);
  va_end(ap);
}

void status(int64_t t, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int r = vsnprintf(gText, TEXT_CAP, fmt, ap);
  va_end(ap);
  if (r < 0) return;
  sanitize(gText);
  line(LV_ALWAYS, "S,%" PRId64 ",%s", t, gText);
}

void workoutTransition(int64_t t, const char* from, const char* to, const char* reason) {
  line(LV_ALWAYS, "W,%" PRId64 ",%s,%s,%s", t, from, to, reason);
}

void input(int64_t t, const char* event, const char* arg) {
  line(LV_ALWAYS, "I,%" PRId64 ",%s,%s", t, event, arg);
}

void summary(int64_t t, const char* reason, uint32_t wkPulses, int64_t movingUs, int64_t totalUs, int32_t topMmps,
             int32_t avgMmps, uint32_t workoutId) {
  line(LV_ALWAYS, "Z,%" PRId64 ",%s,%" PRIu32 ",%" PRId64 ",%" PRId64 ",%" PRId32 ",%" PRId32 ",%" PRIu32, t, reason,
       wkPulses, movingUs, totalUs, topMmps, avgMmps, workoutId);
}

void heap(int64_t t) {
  line(LV_ALWAYS, "D,%" PRId64 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32, t,
       static_cast<uint32_t>(esp_get_free_heap_size()), static_cast<uint32_t>(esp_get_minimum_free_heap_size()),
       static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
       static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)));
}

void dumpInterrupts(int64_t t) {
  if (gDumpStream == nullptr) {
    status(t, "intr;unavailable (stream not created)");
    return;
  }
  gDumpLen = 0;
  gDumpTrunc = false;
  const esp_err_t err = esp_intr_dump(gDumpStream);
  fflush(gDumpStream);
  uint32_t skipped = 0;
  size_t start = 0;
  for (size_t i = 0; i <= gDumpLen; ++i) {
    if (i < gDumpLen && gDump[i] != '\n') continue;
    const size_t n = i - start;
    const char* row = gDump + start;
    start = i + 1;
    if (n == 0) continue;
    if (contains(row, n, "Free") || contains(row, n, "Reserved") || contains(row, n, "CPU-internal")) {
      ++skipped;
      continue;
    }
    status(t, "intr;%.*s", static_cast<int>(n), row);
  }
  status(t, "intr;end err=%d skipped_free_reserved_rows=%" PRIu32 " truncated=%u",
         static_cast<int>(err), skipped, gDumpTrunc ? 1u : 0u);
}

ReadResult readCommand(char* out, size_t outSize, uint32_t maxBytes, uint32_t& bytesRead) {
  bytesRead = 0;
  while (bytesRead < maxBytes && Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    ++bytesRead;
    if (c == '\r') continue;
    if (c == '\n') {
      if (gCmdLong) {
        gCmdLen = 0;
        gCmdLong = false;
        return ReadResult::TooLong;
      }
      if (gCmdLen == 0) continue;  // empty line: ignored
      const size_t n = gCmdLen < outSize - 1 ? gCmdLen : outSize - 1;
      memcpy(out, gCmd, n);
      out[n] = '\0';
      gCmdLen = 0;
      return ReadResult::Line;
    }
    if (gCmdLong) continue;
    if (gCmdLen < hw::CMD_MAX_CHARS) {
      gCmd[gCmdLen++] = static_cast<char>(c);
    } else {
      gCmdLong = true;  // the whole line will be discarded at its '\n'
    }
  }
  return ReadResult::None;
}

}  // namespace dbg
