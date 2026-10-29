// format.cpp - see format.h.
#include "format.h"

#include "fixed.h"

namespace ot {

namespace {

constexpr uint64_t UM_PER_KM_HUNDREDTH = 10000000u;   // 10 m
constexpr uint64_t UM_PER_MI_HUNDREDTH = 16093440u;   // 16.09344 m (1 mi = 1609.344 m exactly)
constexpr uint64_t PACE_NUM_KM = 1000000u;            // mm per km
constexpr uint64_t PACE_NUM_MI = 1609344u;            // mm per mi
static_assert(UM_PER_MI_HUNDREDTH * 100u == PACE_NUM_MI * 1000u, "1 mi = 1609.344 m in both forms");

// A small bounded string builder: appends into a local buffer that is large
// enough for every string of this file, then copyOut() applies the caller's
// size with snprintf-like truncation.
class Builder {
 public:
  void ch(char c) {
    if (len_ < CAP) buf_[len_++] = c;
  }
  void str(const char* s) {
    while (*s != '\0') ch(*s++);
  }
  // Unsigned decimal, at least minDigits digits (zero-padded).
  void u64(uint64_t v, uint32_t minDigits = 1) {
    char tmp[20];  // UINT64_MAX has 20 digits
    uint32_t n = 0;
    do {
      tmp[n++] = static_cast<char>('0' + static_cast<char>(v % 10u));
      v /= 10u;
    } while (v != 0 && n < sizeof(tmp));
    while (n < minDigits && n < sizeof(tmp)) tmp[n++] = '0';
    while (n > 0) ch(tmp[--n]);
  }
  size_t copyOut(char* out, size_t n) const {
    if (n != 0 && out != nullptr) {
      const size_t k = len_ < n - 1u ? len_ : n - 1u;
      for (size_t i = 0; i < k; ++i) out[i] = buf_[i];
      out[k] = '\0';
    }
    return len_;
  }

 private:
  static constexpr size_t CAP = 32;  // > every full string here (longest: 16 chars)
  char   buf_[CAP] = {};
  size_t len_ = 0;
};

constexpr char NO_PACE[] = "--:--";

size_t writeNoPace(char* buf, size_t n) {
  Builder b;
  b.str(NO_PACE);
  return b.copyOut(buf, n);
}

uint32_t absDiff(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }

}  // namespace

const char* unitsName(Units u) { return u == Units::Km ? "km" : "mi"; }
const char* speedUnitName(Units u) { return u == Units::Km ? "km/h" : "mph"; }
const char* paceUnitName(Units u) { return u == Units::Km ? "/km" : "/mi"; }

uint32_t paceSeconds(int32_t mmps, Units u) {
  if (mmps <= 0) return 0;
  const uint64_t v = static_cast<uint64_t>(mmps);
  const uint64_t num = u == Units::Km ? PACE_NUM_KM : PACE_NUM_MI;
  return static_cast<uint32_t>((num + v / 2u) / v);  // v >= 1 -> <= 1609344
}

uint32_t paceExactMs(int32_t mmps, Units u) {
  if (mmps <= 0) return 0;
  const uint64_t v = static_cast<uint64_t>(mmps);
  const uint64_t num = (u == Units::Km ? PACE_NUM_KM : PACE_NUM_MI) * 1000u;
  return static_cast<uint32_t>((num + v / 2u) / v);  // v >= 1 -> <= 1609344000 < 2^32
}

uint32_t speedTenths(int32_t mmps, Units u) {
  if (mmps <= 0) return 0;
  const uint64_t v = static_cast<uint64_t>(mmps);
  // km/h tenths = v * 0.036; mph tenths = v * 36000 / 1609344. v <= 2^31 -> < 2^47.
  if (u == Units::Km) return static_cast<uint32_t>((v * 36u + 500u) / 1000u);
  return static_cast<uint32_t>((v * 36000u + 804672u) / 1609344u);
}

uint32_t speedCenti(int32_t mmps, Units u) {
  if (mmps <= 0) return 0;
  const uint64_t v = static_cast<uint64_t>(mmps);
  if (u == Units::Km) return static_cast<uint32_t>((v * 36u + 50u) / 100u);  // = ftmsSpeedCentiKmh
  return static_cast<uint32_t>((v * 360000u + 804672u) / 1609344u);          // v <= 2^31 -> < 2^50
}

uint64_t distanceHundredths(uint32_t pulses, Units u) {
  return pulsesToUm(pulses) / (u == Units::Km ? UM_PER_KM_HUNDREDTH : UM_PER_MI_HUNDREDTH);
}

bool paceShownInState(WorkoutState s) { return s == WorkoutState::Warmup || s == WorkoutState::Running; }

size_t fmtPaceSeconds(char* buf, size_t n, uint32_t seconds) {
  if (seconds > PACE_MAX_S) return writeNoPace(buf, n);
  Builder b;
  b.u64(seconds / 60u);
  b.ch(':');
  b.u64(seconds % 60u, 2);
  return b.copyOut(buf, n);
}

size_t fmtPace(char* buf, size_t n, int32_t mmps, Units u, bool visible) {
  if (!visible || mmps <= 0) return writeNoPace(buf, n);
  return fmtPaceSeconds(buf, n, paceSeconds(mmps, u));
}

size_t fmtTenths(char* buf, size_t n, uint32_t tenths) {
  Builder b;
  b.u64(tenths / 10u);
  b.ch('.');
  b.u64(tenths % 10u);
  return b.copyOut(buf, n);
}

size_t fmtSpeed(char* buf, size_t n, int32_t mmps, Units u) { return fmtTenths(buf, n, speedTenths(mmps, u)); }

size_t fmtDistance(char* buf, size_t n, uint32_t pulses, Units u) {
  const uint64_t h = distanceHundredths(pulses, u);
  Builder b;
  b.u64(h / 100u);
  b.ch('.');
  b.u64(h % 100u, 2);
  return b.copyOut(buf, n);
}

size_t fmtDuration(char* buf, size_t n, int64_t us) {
  const uint64_t s = us > 0 ? static_cast<uint64_t>(us) / 1000000u : 0u;  // floored
  Builder b;
  if (s < 3600u) {
    b.u64(s / 60u);
    b.ch(':');
    b.u64(s % 60u, 2);
  } else {
    b.u64(s / 3600u);
    b.ch(':');
    b.u64((s / 60u) % 60u, 2);
    b.ch(':');
    b.u64(s % 60u, 2);
  }
  return b.copyOut(buf, n);
}

// ---------------------------------------------------------------------------
// PaceDisplayFilter
// ---------------------------------------------------------------------------
void PaceDisplayFilter::reset() { *this = PaceDisplayFilter(); }

bool PaceDisplayFilter::update(int64_t now, int32_t mmps, Units u, bool stateShowsPace) {
  // 1. Visibility hysteresis (always tracked).
  if (mmps >= PACE_SHOW_MMPS) hyst_ = true;
  else if (mmps < PACE_HIDE_MMPS) hyst_ = false;

  // 2. Target.
  const uint32_t s = paceSeconds(mmps, u);
  const bool want = stateShowsPace && hyst_ && mmps > 0 && s <= PACE_MAX_S;

  // 3. Immediate changes.
  const bool first = !started_;
  const bool unitsChanged = started_ && u != units_;
  if (first || unitsChanged || (!stateShowsPace && shown_)) {
    // (a state-forced hide always changes the value: shown_ was true, want is false)
    started_ = true;
    units_ = u;
    shown_ = want;
    shownS_ = want ? s : 0;
    lastChangeUs_ = now;
    return true;
  }

  // 4. Rate-limited changes.
  if (now - lastChangeUs_ < PACE_REFRESH_US) return false;
  if (want != shown_) {
    shown_ = want;
    shownS_ = want ? s : 0;
    lastChangeUs_ = now;
    return true;
  }
  if (!shown_) return false;
  const int64_t exactMs = static_cast<int64_t>(paceExactMs(mmps, u));
  const int64_t shownMs = static_cast<int64_t>(shownS_) * 1000;
  const int64_t d = exactMs > shownMs ? exactMs - shownMs : shownMs - exactMs;
  if (d < PACE_DEADBAND_MS || s == shownS_) return false;
  shownS_ = s;
  lastChangeUs_ = now;
  return true;
}

size_t PaceDisplayFilter::format(char* buf, size_t n) const {
  if (!shown_) return writeNoPace(buf, n);
  return fmtPaceSeconds(buf, n, shownS_);
}

// ---------------------------------------------------------------------------
// SpeedDisplayFilter
// ---------------------------------------------------------------------------
void SpeedDisplayFilter::reset() { *this = SpeedDisplayFilter(); }

bool SpeedDisplayFilter::update(int64_t now, int32_t mmps, Units u) {
  const uint32_t c = speedCenti(mmps, u);
  // New value: the exact speed rounded ONCE to tenths (= fmtSpeed), not the
  // (c+5)/10, which rounds twice (8.046 km/h -> c 805 -> "8.1").
  const uint32_t target = speedTenths(mmps, u);
  if (!started_ || u != units_) {  // first value, or re-formatted in the new units: at once
    started_ = true;
    units_ = u;
    s_ = target;
    lastChangeUs_ = now;
    return true;
  }
  if (now - lastChangeUs_ < SPEED_REFRESH_US) return false;
  if (absDiff(c, 10u * s_) < SPEED_HYST_CENTI) return false;
  s_ = target;  // |c - 10 s| >= 8 -> |exact - s| >= 0.75 tenths -> target != s
  lastChangeUs_ = now;
  return true;
}

size_t SpeedDisplayFilter::format(char* buf, size_t n) const { return fmtTenths(buf, n, s_); }

}  // namespace ot
