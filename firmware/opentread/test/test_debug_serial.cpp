// test_debug_serial.cpp - host tests for the pure parts of debug_serial.h:
// the TX room rule (dbg::tx) and the command
// parse helpers (dbg::parse).
//
// The TX test runs a model of the ESP-IDF 5.5 UART TX path as compiled into
// core 3.3.11 (read from a release ELF disassembly):
//   - the TX ring is a NOSPLIT ring buffer of DEBUG_TX_BUFFER bytes; an item
//     takes 8 header bytes + its length rounded up to 4, never wraps, and a
//     tail too short for it is wasted (the item goes to the head instead);
//     a tail left shorter than a header after an item is wasted at once;
//   - uart_get_tx_buffer_free_size() (what Serial.availableForWrite()
//     returns while the ring isn't full) = for each free region
//     ((len - 8) & ~3) capped at the max item size, summed, minus 20;
//   - uart_tx_all() sends a 12-byte header item (portMAX_DELAY), then loops
//     chunk = min(remaining, xRingbufferGetCurFreeSize()) where the latter is
//     the LARGER free region minus 8; a chunk of 0 means it busy-waits.
// The model never drains the ring during the write (worst case: draining
// only ever adds space at the head). It over-reports availableForWrite by up
// to 8 bytes against the real function (which reconstructs the end of the
// buffer from the max item size), so it tests MORE states than reality.
//
// Property: whenever dbg::tx::roomFor(availableForWrite, n) passes, the write
// of an n-byte line completes without waiting. The old rule (n + 128, floor
// 256) is shown to fail it (the known failing scenario included).
#include <cstdint>
#include <cstdio>

#include "../debug_serial.h"
#include "check.h"

namespace {

constexpr int RING = 4096;                  // hw::DEBUG_TX_BUFFER
constexpr int HDR = 8;                      // ring item header
constexpr int MAX_ITEM = RING / 2 - HDR;    // NOSPLIT max item size
constexpr int UART_HDR_ITEM = 12;           // uart_tx_data_t
constexpr int LINE_LEN_MAX = 479;               // debug_serial.cpp LINE_CAP - 1 (longest line incl. '\n')

int align4(int x) { return (x + 3) & ~3; }
int minI(int a, int b) { return a < b ? a : b; }
int maxI(int a, int b) { return a > b ? a : b; }

// Free space of the ring. split = true: write pointer at or after the oldest
// unsent item; free = tail A (write .. end) + head B (start .. oldest).
// split = false: one region R between the write pointer and the oldest item.
struct Ring {
  bool split;
  int a;  // split: tail A; else R
  int b;  // split: head B; else unused
  bool empty;
};

int regionTerm(int len) { return len <= HDR ? 0 : minI((len - HDR) & ~3, MAX_ITEM); }

int availableForWrite(const Ring& r) {
  int sum;
  if (r.empty) sum = RING - HDR;                            // real: (size - 8) - 20
  else if (r.split) sum = regionTerm(r.a) + regionTerm(r.b);
  else sum = regionTerm(r.a);
  return maxI(sum, 20) - 20;
}

int curFreeSize(const Ring& r) {
  const int m = r.split ? maxI(r.a, r.b) : r.a;
  const int v = m - HDR;
  return v < 0 ? 0 : minI(v, MAX_ITEM);
}

// xRingbufferSend (NOSPLIT). Returns false if the item doesn't fit (the real
// call would block).
bool send(Ring& r, int len) {
  const int total = HDR + align4(len);
  r.empty = false;
  if (r.split) {
    if (r.a >= total) {
      r.a -= total;
      if (r.a < HDR) {           // short tail is wasted: the write pointer wraps
        r.split = false;
        r.a = r.b;
        r.b = 0;
      }
      return true;
    }
    if (r.b >= total) {          // tail wasted, item at the head
      r.split = false;
      r.a = r.b - total;
      r.b = 0;
      return true;
    }
    return false;
  }
  if (r.a >= total) {
    r.a -= total;
    return true;
  }
  return false;
}

// uart_tx_all() for an n-byte line. Returns true if it would wait.
bool writeWaits(Ring r, int n) {
  if (!send(r, UART_HDR_ITEM)) return true;
  int rem = n;
  while (rem > 0) {
    const int c = minI(curFreeSize(r), rem);
    if (c == 0) return true;     // busy-wait in uart_tx_all
    if (!send(r, c)) return true;
    rem -= c;
  }
  return false;
}

bool oldRule(int avail, int n) { return avail >= n + 128 && avail >= 256; }

struct Tally {
  long states = 0;
  long passedNew = 0;
  long waitsNew = 0;
  long waitsOld = 0;
};

void checkState(const Ring& r, Tally& t) {
  ++t.states;
  const int avail = availableForWrite(r);
  // The largest n the new rule admits, and its neighbours (alignment), plus
  // short lines.
  const int nMax = minI(LINE_LEN_MAX, (avail - dbg::tx::TX_SLACK) / 2);
  const int cand[] = {1, 14, 20, nMax - 3, nMax - 2, nMax - 1, nMax};
  for (int n : cand) {
    if (n < 1 || n > LINE_LEN_MAX) continue;
    if (dbg::tx::roomFor(avail, static_cast<size_t>(n))) {
      ++t.passedNew;
      if (writeWaits(r, n)) {
        ++t.waitsNew;
        if (t.waitsNew <= 5)
          std::fprintf(stderr, "new rule admits a waiting write: split=%d a=%d b=%d avail=%d n=%d\n",
                       r.split ? 1 : 0, r.a, r.b, avail, n);
      }
    }
  }
  // Old rule, worst n it admits.
  const int nOld = minI(LINE_LEN_MAX, avail - 128);
  if (nOld >= 1 && oldRule(avail, nOld) && writeWaits(r, nOld)) ++t.waitsOld;
}

void testTxRule() {
  // Constants and the FIFO case.
  CHECK_EQ(dbg::tx::need(1), 256);
  CHECK_EQ(dbg::tx::need(96), 256);
  CHECK_EQ(dbg::tx::need(97), 258);
  CHECK_EQ(dbg::tx::need(479), 1022);
  CHECK(!dbg::tx::roomFor(128, 1));   // ring full: core reports the 128 B FIFO
  CHECK(!dbg::tx::roomFor(255, 1));
  CHECK(dbg::tx::roomFor(256, 14));

  // The known failing scenario: tail 172 B, head 176 B, a 175-byte dump line.
  {
    const Ring r{true, 172, 176, false};
    const int avail = availableForWrite(r);
    CHECK_EQ(avail, 312);
    CHECK(oldRule(avail, 175));              // the old rule wrote it ...
    CHECK(writeWaits(r, 175));               // ... and uart_tx_all waited
    CHECK(!dbg::tx::roomFor(avail, 175));    // the new rule drops it
  }

  // Exhaustive over every reachable free-space geometry (4-byte steps).
  Tally t;
  for (int a = HDR; a <= RING; a += 4) {        // split: tail A >= 8 (shorter tails wrap)
    for (int b = 0; a + b <= RING; b += 4) {
      checkState(Ring{true, a, b, a + b == RING}, t);
    }
  }
  for (int r = 0; r < RING; r += 4) checkState(Ring{false, r, 0, false}, t);
  std::printf("tx rule: %ld geometries, %ld admitted writes, %ld waited (new rule), "
              "%ld geometries where the old rule admitted a waiting write\n",
              t.states, t.passedNew, t.waitsNew, t.waitsOld);
  CHECK(t.states > 500000);
  CHECK(t.passedNew > 1000000);
  CHECK_EQ(t.waitsNew, 0L);
  CHECK(t.waitsOld > 0);
}

void testParse() {
  uint32_t v = 0;
  const char* end = nullptr;
  CHECK(dbg::parse::u32("600", v, &end));
  CHECK_EQ(v, 600u);
  CHECK_EQ(*end, '\0');
  CHECK(dbg::parse::u32("999999999", v, &end));
  CHECK(!dbg::parse::u32("1000000000", v, &end));
  CHECK(!dbg::parse::u32("", v, &end));
  CHECK(!dbg::parse::u32("-1", v, &end));
  CHECK(dbg::parse::kmhMilli("10", v));
  CHECK_EQ(v, 10000u);
  CHECK(dbg::parse::kmhMilli("1.5", v));
  CHECK_EQ(v, 1500u);
  CHECK(dbg::parse::kmhMilli("0.25", v));
  CHECK_EQ(v, 250u);
  CHECK(!dbg::parse::kmhMilli("1.", v));
  CHECK(!dbg::parse::kmhMilli("1.2345", v));
  CHECK(!dbg::parse::kmhMilli("1000", v));
  CHECK(!dbg::parse::kmhMilli("1x", v));
}

}  // namespace

int main(int /*argc*/, char** /*argv*/) {
  testTxRule();
  testParse();
  return check::summary("test_debug_serial");
}
