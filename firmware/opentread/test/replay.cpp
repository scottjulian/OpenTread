// replay.cpp - see replay.h.
#include "replay.h"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "metrics.h"
#include "pulse_filter.h"
#include "workout.h"

namespace replay {

const NamedLog kLogs[4] = {
    {"S2_walk", "firmware/opentread/test/data/2026-10-18_1329_S2_walk.csv"},
    {"S1b_10rev", "firmware/opentread/test/data/2026-10-18_1416_S1b_10rev.csv"},
    {"S1_handturn", "firmware/opentread/test/data/2026-10-18_1323_S1_handturn.csv"},
    {"bench_jumper", "firmware/opentread/test/data/2026-10-18_1315_bench_jumper.csv"},
};

bool parseI64(const std::string& s, int64_t& v) {
  if (s.empty()) return false;
  const char* b = s.data();
  const char* e = b + s.size();
  const auto r = std::from_chars(b, e, v);
  return r.ec == std::errc() && r.ptr == e;
}

bool parseU32(const std::string& s, uint32_t& v) {
  if (s.empty() || s[0] == '-') return false;
  const char* b = s.data();
  const char* e = b + s.size();
  const auto r = std::from_chars(b, e, v);
  return r.ec == std::errc() && r.ptr == e;
}

bool parseDouble(const std::string& s, double& v) {
  if (s.empty()) return false;
  char* end = nullptr;
  v = std::strtod(s.c_str(), &end);
  return end == s.c_str() + s.size();
}

namespace {

std::vector<std::string> split(const std::string& s, char sep, size_t maxSplits) {
  std::vector<std::string> out;
  size_t start = 0;
  while (out.size() < maxSplits) {
    const size_t p = s.find(sep, start);
    if (p == std::string::npos) break;
    out.push_back(s.substr(start, p - start));
    start = p + 1;
  }
  out.push_back(s.substr(start));
  return out;
}

// Log-field grammar (same as the oracle's ASCII regexes): an unsigned field is
// one or more ASCII digits - no sign, so "-0" / "+1" are malformed; leading
// zeros are allowed (the oracle's \d+ allows them too). Hex is lowercase only,
// as the pulse logger emits it.
bool isDigits(const std::string& s) {
  if (s.empty()) return false;
  for (const char c : s) {
    if (c < '0' || c > '9') return false;
  }
  return true;
}

bool fieldI64(const std::string& s, int64_t& v) { return isDigits(s) && parseI64(s, v); }
bool fieldU32(const std::string& s, uint32_t& v) { return isDigits(s) && parseU32(s, v); }

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool parseHex(const std::string& s, std::vector<uint8_t>& out) {
  if (s.empty() || (s.size() % 2) != 0) return false;
  out.clear();
  for (size_t i = 0; i < s.size(); i += 2) {
    const int hi = hexVal(s[i]);
    const int lo = hexVal(s[i + 1]);
    if (hi < 0 || lo < 0) return false;
    out.push_back(static_cast<uint8_t>((hi << 4) | lo));
  }
  return true;
}

bool startsWith(const std::string& s, const char* prefix) {
  return s.rfind(prefix, 0) == 0;
}

// "line_level=<0|1>" token inside a banner text.
bool bannerLevel(const std::string& text, uint8_t& level) {
  const std::string key = " line_level=";
  const size_t p = text.find(key);
  if (p == std::string::npos) return false;
  const size_t b = p + key.size();
  size_t e = text.find(' ', b);
  if (e == std::string::npos) e = text.size();
  const std::string v = text.substr(b, e - b);
  if (v == "0") { level = 0; return true; }
  if (v == "1") { level = 1; return true; }
  return false;
}

bool isLowerWord(const std::string& s) {
  if (s.empty()) return false;
  for (const char c : s) {
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  return true;
}

// I-line event/arg pairs (replay.h lists the grammar).
bool validInput(const std::string& ev, const std::string& arg) {
  if (ev == "hold_begin" || ev == "hold_abort") return arg == "-";
  if (ev == "hold_complete") return arg == "ok" || arg == "void" || arg == "ignored";
  if (ev == "units_toggle") return isLowerWord(arg);
  if (ev == "touch_down" || ev == "touch_up") {
    const size_t semi = arg.find(';');
    return semi != std::string::npos && isDigits(arg.substr(0, semi)) && isDigits(arg.substr(semi + 1));
  }
  return false;
}

template <class T>
void sortByT(std::vector<T>& v) {
  std::stable_sort(v.begin(), v.end(), [](const T& a, const T& b) { return a.t < b.t; });
}

}  // namespace

void parseText(const std::string& text, Log& out) {
  Stats& st = out.stats;
  bool haveMax = false;
  int64_t maxT = 0;
  size_t pos = 0;
  while (pos < text.size()) {
    size_t nl = text.find('\n', pos);
    if (nl == std::string::npos) nl = text.size();
    std::string line = text.substr(pos, nl - pos);
    pos = nl + 1;
    while (!line.empty() && (line.back() == '\r')) line.pop_back();
    ++st.lines;
    if (line.empty()) { ++st.blank; continue; }
    if (line[0] == '#') { ++st.header; continue; }
    if (line.size() < 2 || line[0] < 'A' || line[0] > 'Z' || line[1] != ',') { ++st.garbage; continue; }

    const char letter = line[0];
    const std::vector<std::string> f = split(line, ',', letter == 'S' ? 2 : 64);
    int64_t t = 0;
    bool ok = f.size() >= 2 && fieldI64(f[1], t);
    if (ok) {
      switch (letter) {
        case 'E': {
          ok = f.size() == 3 && (f[2] == "0" || f[2] == "1");  // exactly one char, like [01]
          if (ok) { out.edges.push_back({t, static_cast<uint8_t>(f[2] == "1" ? 1 : 0)}); ++st.e; }
          break;
        }
        case 'P': {
          uint32_t n = 0;
          int64_t dt = 0;
          ok = f.size() == 4 && fieldU32(f[2], n) && fieldI64(f[3], dt);
          if (ok) { out.pulses.push_back({t, n, dt, st.lines}); ++st.p; }
          break;
        }
        case 'M': {
          uint32_t n = 0;
          ok = f.size() == 3 && fieldU32(f[2], n);
          if (ok) { out.markers.push_back(t); ++st.m; }
          break;
        }
        case 'F': {
          std::vector<uint8_t> bytes;
          ok = f.size() == 3 && parseHex(f[2], bytes);
          if (ok) { out.ftms.push_back({t, bytes}); ++st.f; }
          break;
        }
        case 'S': {
          ok = f.size() == 3;
          if (ok) {
            const std::string& txt = f[2];
            if (startsWith(txt, "M4 pulse logger") || startsWith(txt, "OpenTread fw=")) {
              uint8_t lv = 0;
              ok = bannerLevel(txt, lv);
              if (ok) out.banners.push_back({t, lv, txt});
            }
            if (ok) { out.status.push_back({t, txt}); ++st.s; }
          }
          break;
        }
        case 'H': {
          ok = f.size() == 9;
          for (size_t i = 2; ok && i < f.size(); ++i) {
            int64_t x = 0;
            ok = fieldI64(f[i], x);
          }
          if (ok) ++st.h;
          break;
        }
        case 'I': {
          ok = f.size() == 4 && validInput(f[2], f[3]);
          if (ok) { out.inputs.push_back({t, f[2], f[3], st.lines}); ++st.i; }
          break;
        }
        default:
          ++st.other;  // well-formed "<Letter>,<t>,..." with a letter the parser does not consume
          break;
      }
    }
    if (!ok) {
      ++st.malformed;
      st.malformedLines.push_back(line);
      continue;
    }
    if (haveMax && t < maxT) ++st.outOfOrder;
    if (!haveMax || t > maxT) { maxT = t; haveMax = true; }
  }
  sortByT(out.edges);
  sortByT(out.pulses);
  sortByT(out.inputs);
  std::stable_sort(out.markers.begin(), out.markers.end());
  sortByT(out.ftms);
  sortByT(out.status);
  sortByT(out.banners);
}

bool loadFile(const std::string& path, Log& out, std::string& err) {
  std::ifstream in(path, std::ios::binary);
  if (!in) { err = "cannot open " + path; return false; }
  std::ostringstream ss;
  ss << in.rdbuf();
  out = Log{};
  out.path = path;
  parseText(ss.str(), out);
  return true;
}

bool refilter(const Log& log, const ot::Config& cfg, std::vector<int64_t>& pulses, std::string& err) {
  pulses.clear();
  if (log.banners.size() != 1) {
    err = "expected exactly 1 banner, found " + std::to_string(log.banners.size());
    return false;
  }
  ot::PulseFilter pf(cfg);
  pf.reset(log.banners[0].t, log.banners[0].lineLevel);
  for (const Edge& e : log.edges) {
    if (pf.onEdge(e.t, e.level)) pulses.push_back(e.t);
  }
  return true;
}

std::vector<Sample> sampleMetrics(const std::vector<int64_t>& pulses, int64_t start, int64_t end,
                                  int64_t step, const ot::Config& cfg) {
  std::vector<Sample> out;
  if (step <= 0) return out;
  ot::Metrics m(cfg);
  size_t i = 0;
  for (int64_t t = start; t <= end; t += step) {
    while (i < pulses.size() && pulses[i] <= t) m.onPulse(pulses[i++]);
    out.push_back({t, m.speedMmps(t), m.odometerPulses()});
  }
  return out;
}

// ---------------------------------------------------------------------------
// Workout replay
// ---------------------------------------------------------------------------
std::vector<WorkoutInput> workoutInputs(const Log& log) {
  struct Item { int64_t t; uint32_t seq; WorkoutInput in; };
  std::vector<Item> items;
  for (const PulseLine& p : log.pulses) items.push_back({p.t, p.seq, {p.t, InputKind::Pulse, std::string()}});
  for (const InputLine& i : log.inputs) {
    if (i.event == "hold_begin") items.push_back({i.t, i.seq, {i.t, InputKind::HoldBegin, std::string()}});
    else if (i.event == "hold_abort") items.push_back({i.t, i.seq, {i.t, InputKind::HoldAbort, std::string()}});
    else if (i.event == "hold_complete") items.push_back({i.t, i.seq, {i.t, InputKind::HoldComplete, i.arg}});
  }
  std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
    return a.t != b.t ? a.t < b.t : a.seq < b.seq;
  });
  std::vector<WorkoutInput> out;
  out.reserve(items.size());
  for (const Item& it : items) out.push_back(it.in);
  return out;
}

std::vector<WorkoutInput> pulseInputs(const std::vector<int64_t>& pulses) {
  std::vector<WorkoutInput> out;
  out.reserve(pulses.size());
  for (int64_t t : pulses) out.push_back({t, InputKind::Pulse, std::string()});
  return out;
}

void mergeInputs(std::vector<WorkoutInput>& into, const std::vector<WorkoutInput>& more) {
  into.insert(into.end(), more.begin(), more.end());
  std::stable_sort(into.begin(), into.end(), [](const WorkoutInput& a, const WorkoutInput& b) { return a.t < b.t; });
}

WorkoutReplay::WorkoutReplay(const ot::Config& cfg) : metrics(cfg), workout(cfg) {}

void WorkoutReplay::drain() {
  ot::WorkoutEvent e;
  while (workout.pollEvent(e)) {
    trace.events.push_back(e);
    switch (e.type) {
      case ot::WorkoutEventType::Confirmed: ++trace.workouts; break;
      case ot::WorkoutEventType::WarmupFailed: ++trace.warmupFails; break;
      case ot::WorkoutEventType::Paused: ++trace.pauses; break;
      case ot::WorkoutEventType::Resumed: ++trace.resumes; break;
      default: break;
    }
  }
  if (workout.eventOverflow()) trace.overflow = true;
}

void WorkoutReplay::feed(const WorkoutInput& in) {
  switch (in.kind) {
    case InputKind::Pulse:
      ++trace.inputsP;
      metrics.onPulse(in.t);
      workout.onPulse(in.t, metrics.speedAtLastPulseMmps());
      break;
    case InputKind::HoldBegin:
      ++trace.inputsI;
      workout.holdBegan(in.t);
      break;
    case InputKind::HoldAbort:
      ++trace.inputsI;
      workout.holdAborted(in.t);
      break;
    case InputKind::HoldComplete: {
      ++trace.inputsI;
      lastResult = workout.holdCompleted(in.t);
      lastResultT = in.t;
      if (!in.expect.empty() && in.expect != ot::holdResultName(lastResult)) ++trace.holdMismatch;
      break;
    }
  }
  drain();
}

void WorkoutReplay::tickTo(int64_t now) {
  workout.tick(now);
  drain();
}

void WorkoutReplay::runUntil(const std::vector<WorkoutInput>& in, size_t& next, int64_t until) {
  while (next < in.size() && in[next].t <= until) feed(in[next++]);
  tickTo(until);
}

void WorkoutReplay::finish() {
  int64_t d = 0;
  for (int guard = 0; guard < 64 && workout.nextDeadline(d); ++guard) tickTo(d);
}

WorkoutTrace replayWorkout(const std::vector<WorkoutInput>& in, const ot::Config& cfg) {
  WorkoutReplay r(cfg);
  for (const WorkoutInput& x : in) r.feed(x);
  r.finish();
  return r.trace;
}

bool isZEvent(const ot::WorkoutEvent& e) {
  return e.type == ot::WorkoutEventType::Stopped || e.type == ot::WorkoutEventType::AutoEnded;
}

std::string formatW(const ot::WorkoutEvent& e) {
  return std::to_string(e.t) + " " + ot::workoutStateName(e.from) + " " + ot::workoutStateName(e.to) + " " +
         ot::workoutEventReason(e.type);
}

std::string formatZ(const ot::WorkoutEvent& e) {
  const ot::SummaryRecord& s = e.summary;
  return std::to_string(e.t) + " " + ot::endReasonName(s.endReason) + " " + std::to_string(s.pulses) + " " +
         std::to_string(s.movingUs) + " " + std::to_string(s.totalUs) + " " + std::to_string(s.topMmps) + " " +
         std::to_string(s.avgMmps) + " " + std::to_string(s.workoutId);
}

bool Golden::has(const std::string& key) const {
  for (const auto& p : kv) if (p.first == key) return true;
  return false;
}

std::string Golden::get(const std::string& key) const {
  for (const auto& p : kv) if (p.first == key) return p.second;
  return std::string();
}

std::vector<std::string> Golden::all(const std::string& key) const {
  std::vector<std::string> v;
  for (const auto& p : kv) if (p.first == key) v.push_back(p.second);
  return v;
}

bool loadGolden(const std::string& path, Golden& g, std::string& err) {
  std::ifstream in(path);
  if (!in) { err = "missing golden " + path; return false; }
  g.kv.clear();
  std::string line;
  while (std::getline(in, line)) {
    while (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos) { err = "bad golden line in " + path + ": " + line; return false; }
    g.kv.emplace_back(line.substr(0, eq), line.substr(eq + 1));
  }
  return true;
}

std::string repoRootFromArgs(int argc, char** argv) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--root") return argv[i + 1];
  }
  if (const char* env = std::getenv("OT_REPO_ROOT")) return env;
  return ".";
}

}  // namespace replay
