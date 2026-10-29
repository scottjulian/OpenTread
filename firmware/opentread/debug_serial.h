// debug_serial.h - the single serial writer and the inbound command reader.
//
// Output rules:
//  - Only loop() (and setup(), same task) calls these. ISRs and callbacks
//    never touch Serial.
//  - One line = one snprintf into a static buffer + one Serial.write, so
//    lines never tear or interleave.
//  - Non-blocking: before writing, Serial.availableForWrite() must pass
//    dbg::tx::roomFor() (below: about twice the line length, because the
//    reported space may be split in two ring regions). If it doesn't, the
//    line is DROPPED and counted (H field 6, debug_drop).
//  - Verbosity: a line of level L is emitted only when L <= verbosity.
//    Level 2 = E/P, 1 = K, 0 = everything else (S, M, H, D, G, W, I, Z).
//
// Inbound commands: ASCII, one per line, '\n' terminated, '\r' ignored,
// at most CMD_MAX_CHARS (16) characters; a longer line is discarded whole.
//
// The parse helpers in dbg::parse are pure (no Arduino) so they can be
// unit-tested on the host; this header has no Arduino/IDF includes.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dbg {

enum : uint8_t { LV_ALWAYS = 0, LV_K = 1, LV_EDGE = 2, LV_MAX = 2 };

// Serial.setTxBufferSize(4096), Serial.begin(115200), silence ESP-IDF
// logging, terminate any boot-ROM garbage line, prepare the interrupt-dump
// stream (all allocation happens here, in setup()).
void begin(uint8_t verbosity);

uint8_t  verbosity();
void     setVerbosity(uint8_t v);            // clamped to 0..2
uint32_t droppedLines();                     // debug_drop
inline bool enabled(uint8_t level) { return level <= verbosity(); }

// Emit one complete line (fmt must NOT end with '\n'; it is appended).
void line(uint8_t level, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// S,<t>,<text>: commas in text become ';', CR/LF become ' ', other control
// characters '?'. Always level 0.
void status(int64_t t, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// D,<t>,<free>,<min_free>,<largest_8bit>,<largest_dma>: heap snapshot,
// level 0. Used by the 60 s stats and by the bring-up boot points.
void heap(int64_t t);

// Workout lines, level 0:
//   W,<t>,<from>,<to>,<reason>          every Workout transition (states uppercase)
//   I,<t>,<event>,<arg>                 every input to Workout / the units state, with
//                                       the exact timestamp passed to the core; arg is
//                                       '-' for hold_begin / hold_abort, ok|void|ignored
//                                       for hold_complete
//   Z,<t>,<reason>,<wk_pulses>,<moving_us>,<total_us>,<top_mmps>,<avg_mmps>,<workout_id>
//                                       once on entry to SUMMARY (the frozen record)
void workoutTransition(int64_t t, const char* from, const char* to, const char* reason);
void input(int64_t t, const char* event, const char* arg);
void summary(int64_t t, const char* reason, uint32_t wkPulses, int64_t movingUs, int64_t totalUs, int32_t topMmps,
             int32_t avgMmps, uint32_t workoutId);

// Run esp_intr_dump() into a static memory stream and emit the interesting
// rows as S,<t>,intr;<row> lines (Free/Reserved/CPU-internal rows are
// summarised as a count, so the dump fits the TX buffer).
void dumpInterrupts(int64_t t);

// ---- inbound -------------------------------------------------------------
enum class ReadResult : uint8_t { None, Line, TooLong };

// Read up to maxBytes pending RX bytes. Returns Line with a NUL-terminated
// command in out (1..16 chars), TooLong when a > 16-char line ended (it is
// discarded whole), or None. Empty lines are ignored.
ReadResult readCommand(char* out, size_t outSize, uint32_t maxBytes, uint32_t& bytesRead);

// ---- TX room rule (pure, host-testable: test/test_debug_serial.cpp) --------
//
// A line of n bytes is written only if Serial.availableForWrite() >=
// txNeed(n); otherwise it is dropped and counted. Why 2n + 64 and not n plus
// a fixed margin (IDF 5.5 UART driver, core 3.3.11, checked on the ELF):
//  - The TX ring is a NOSPLIT ring buffer. When it has wrapped, its free
//    space is two regions: the tail (write pointer .. end) and the head
//    (start .. oldest unsent item). availableForWrite() reports roughly
//    their SUM (each region minus an 8-byte item header, minus 20).
//  - uart_write_bytes() first queues a 12-byte header item (20 B of ring),
//    then data chunks of min(remaining, xRingbufferGetCurFreeSize()), and the
//    latter is the LARGER region minus 8. A chunk that goes to the head
//    wastes whatever is left of the tail, so the wrap waste can be almost a
//    whole line. When the free size reaches 0 with data left, the write
//    busy-waits inside loop() until the UART has sent the oldest item.
//  - The write never waits when the larger region holds n + 11 bytes (8-byte
//    header + 4-byte alignment), and a reported sum of about 2n guarantees
//    that. The host model (test_debug_serial.cpp, every ring geometry) finds
//    no waiting write for 2n + s with s >= 0 and some for s = -16, so
//    TX_SLACK = 64 is margin on top.
//  - TX_FLOOR: when the ring is completely full, core 3.3.11 reports the
//    free space of the 128-byte hardware FIFO instead, so any reading below
//    the floor (more than the FIFO length) is treated as "full".
namespace tx {
constexpr int TX_SLACK = 64;
constexpr int TX_FLOOR = 256;
constexpr int need(size_t n) {
  return 2 * static_cast<int>(n) + TX_SLACK > TX_FLOOR ? 2 * static_cast<int>(n) + TX_SLACK : TX_FLOOR;
}
constexpr bool roomFor(int availableForWrite, size_t n) { return availableForWrite >= need(n); }
}  // namespace tx

// ---- pure parse helpers (host-testable) ------------------------------------
namespace parse {

// Unsigned decimal, digits only, no sign, 1..9 digits. Returns false
// otherwise. *end receives the first unparsed character.
inline bool u32(const char* s, uint32_t& out, const char** end) {
  uint32_t v = 0;
  uint32_t n = 0;
  while (s[n] >= '0' && s[n] <= '9') {
    if (n == 9) return false;
    v = v * 10u + static_cast<uint32_t>(s[n] - '0');
    ++n;
  }
  if (n == 0) return false;
  out = v;
  *end = s + n;
  return true;
}

// km/h as a whole string: "<1-3 digits>[.<1-3 digits>]" -> milli-km/h.
// No float. "10" -> 10000, "1.5" -> 1500, "0.25" -> 250.
inline bool kmhMilli(const char* s, uint32_t& out) {
  uint32_t ip = 0;
  uint32_t n = 0;
  while (s[n] >= '0' && s[n] <= '9') {
    if (n == 3) return false;
    ip = ip * 10u + static_cast<uint32_t>(s[n] - '0');
    ++n;
  }
  if (n == 0) return false;
  uint32_t frac = 0;
  uint32_t scale = 1000;
  if (s[n] == '.') {
    ++n;
    uint32_t fd = 0;
    while (s[n] >= '0' && s[n] <= '9') {
      if (fd == 3) return false;
      scale /= 10u;
      frac += static_cast<uint32_t>(s[n] - '0') * scale;
      ++fd;
      ++n;
    }
    if (fd == 0) return false;
  }
  if (s[n] != '\0') return false;
  out = ip * 1000u + frac;
  return true;
}

}  // namespace parse
}  // namespace dbg
