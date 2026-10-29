// replay.h - serial-log parser and replay drivers. Host only.
//
// Parser rules:
//   - '#' lines (tools/capture.py header) are counted as `header` and skipped;
//   - empty lines are counted as `blank`;
//   - a line that does not start with "<A-Z>," is boot-ROM noise: `garbage`;
//   - a "<A-Z>," line that fails its letter's grammar is `malformed`: counted,
//     kept verbatim in Stats::malformedLines, never accepted;
//   - S lines are split at most twice (everything after the 2nd comma is text);
//   - the banner is an S line whose text starts with "M4 pulse logger" or
//     "OpenTread fw=" and carries line_level=<0|1>;
//   - every record list is stable-sorted by t_us after parsing (file order is
//     kept for equal timestamps); records whose t is below the running maximum
//     in file order are counted in `outOfOrder`.
// Grammar: E,t,level(0|1)  P,t,n,dt  M,t,n  F,t,<even-length lowercase hex>
//          S,t,text  H,t,+7 unsigned integers  I,t,event,arg (below)
//          any other <A-Z>,t,... is `other`.
//          I (inputs to Workout / the units state), exactly 4 fields:
//            hold_begin,-   hold_abort,-   hold_complete,<ok|void|ignored>
//            units_toggle,<[a-z0-9_]+>   touch_down,<x>;<y>   touch_up,<x>;<y>
//          (x, y unsigned digits). Anything else after "I," is malformed.
//          t, n, dt and the H fields are ASCII digits only (no sign: "-0" is
//          malformed; leading zeros allowed); the E level is exactly "0" or "1".
//          This is the oracle's regex grammar (test/oracle/oracle.py _RE,
//          re.ASCII); lines end only at '\n' (a trailing '\r' is stripped)
//          on both sides.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "config.h"
#include "metrics.h"
#include "workout.h"

namespace replay {

struct Edge { int64_t t; uint8_t level; };
struct PulseLine { int64_t t; uint32_t n; int64_t dt; uint32_t seq; };  // seq = line number (file order)
struct InputLine { int64_t t; std::string event; std::string arg; uint32_t seq; };
struct FtmsLine { int64_t t; std::vector<uint8_t> bytes; };
struct StatusLine { int64_t t; std::string text; };
struct Banner { int64_t t; uint8_t lineLevel; std::string text; };

struct Stats {
  uint32_t lines = 0, header = 0, blank = 0, garbage = 0, malformed = 0, other = 0, outOfOrder = 0;
  uint32_t e = 0, p = 0, f = 0, m = 0, s = 0, h = 0, i = 0;
  std::vector<std::string> malformedLines;
};

struct Log {
  std::string path;
  std::vector<Edge> edges;
  std::vector<PulseLine> pulses;
  std::vector<InputLine> inputs;   // I lines
  std::vector<int64_t> markers;
  std::vector<FtmsLine> ftms;
  std::vector<StatusLine> status;
  std::vector<Banner> banners;
  Stats stats;
};

// Parse a whole log text (never throws; problems are counted in out.stats).
void parseText(const std::string& text, Log& out);
bool loadFile(const std::string& path, Log& out, std::string& err);

// Re-filter the E lines through PulseFilter initialised from the banner's
// line_level and t_us. Requires exactly one banner (a reboot inside a log
// would mix two clocks) and at least one edge list (possibly empty).
bool refilter(const Log& log, const ot::Config& cfg, std::vector<int64_t>& pulses, std::string& err);

// Feed `pulses` (sorted) to Metrics and sample at start, start+step, ... <= end.
// A pulse AT a sample time counts as already received at that time.
struct Sample { int64_t t; int32_t speedMmps; uint32_t odometer; };
std::vector<Sample> sampleMetrics(const std::vector<int64_t>& pulses, int64_t start, int64_t end,
                                  int64_t step, const ot::Config& cfg = ot::Config{});

// ---- Workout replay -------------------------------------------------------
// P lines drive Metrics -> Workout (speedAtLastPulse at the pulse instant);
// I lines hold_begin / hold_abort / hold_complete drive the hold API with
// their exact t. Inputs are merged in t order, ties in file order (the
// device prints lines in the order it processed them).
enum class InputKind : uint8_t { Pulse, HoldBegin, HoldAbort, HoldComplete };
struct WorkoutInput {
  int64_t     t;
  InputKind   kind;
  std::string expect;  // hold_complete: the result the log recorded ("" = none)
};

// P lines + the Workout-relevant I lines of `log` (units/touch I lines are not
// Workout inputs and are skipped).
std::vector<WorkoutInput> workoutInputs(const Log& log);
// Synthetic helpers.
std::vector<WorkoutInput> pulseInputs(const std::vector<int64_t>& pulses);
void mergeInputs(std::vector<WorkoutInput>& into, const std::vector<WorkoutInput>& more);  // stable by t

struct WorkoutTrace {
  std::vector<ot::WorkoutEvent> events;   // every W, in emission order (Z = Stopped/AutoEnded events)
  uint32_t workouts = 0, warmupFails = 0, pauses = 0, resumes = 0;
  uint32_t holdMismatch = 0;              // hold_complete results differing from the logged arg
  uint32_t inputsP = 0, inputsI = 0;
  bool overflow = false;
};

class WorkoutReplay {
 public:
  explicit WorkoutReplay(const ot::Config& cfg = ot::Config{});
  void feed(const WorkoutInput& in);       // one input, then drain the events
  void tickTo(int64_t now);                // Workout.tick(now), then drain
  // Feed every input with t <= until (from index `next` on), then tick(until).
  void runUntil(const std::vector<WorkoutInput>& in, size_t& next, int64_t until);
  // Tick at each next deadline until no time-based transition is left
  // (IDLE, no bout, no re-arm block: the log's tail fully played out).
  void finish();
  void drain();

  ot::Metrics  metrics;
  ot::Workout  workout;
  WorkoutTrace trace;
  int64_t      lastResultT = 0;
  ot::HoldResult lastResult = ot::HoldResult::Ignored;
};

// Replay all inputs then finish().
WorkoutTrace replayWorkout(const std::vector<WorkoutInput>& in, const ot::Config& cfg);

// Golden-contract line bodies (test/golden/workout_*.txt):
//   w=<t_us> <FROM> <TO> <reason>
//   z=<t_us> <reason> <wk_pulses> <moving_us> <total_us> <top_mmps> <avg_mmps> <workout_id>
std::string formatW(const ot::WorkoutEvent& e);
std::string formatZ(const ot::WorkoutEvent& e);
bool isZEvent(const ot::WorkoutEvent& e);

// Golden files: key=value lines (repeated keys allowed, order kept).
struct Golden {
  std::vector<std::pair<std::string, std::string>> kv;
  bool has(const std::string& key) const;
  std::string get(const std::string& key) const;              // first value or ""
  std::vector<std::string> all(const std::string& key) const;  // every value, in order
};
bool loadGolden(const std::string& path, Golden& g, std::string& err);

// Strict integer / decimal parsing helpers (whole string must be consumed).
bool parseI64(const std::string& s, int64_t& v);
bool parseU32(const std::string& s, uint32_t& v);
bool parseDouble(const std::string& s, double& v);

// Repo root from "--root <dir>", else $OT_REPO_ROOT, else ".".
std::string repoRootFromArgs(int argc, char** argv);

// Logs used by the tests (repo-relative).
struct NamedLog { const char* name; const char* path; };
extern const NamedLog kLogs[4];  // S2_walk, S1b_10rev, S1_handturn, bench_jumper

}  // namespace replay
