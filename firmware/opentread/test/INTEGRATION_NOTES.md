# Oracle integration notes

Two parts were written independently: (A) the C++ core, tests and
`run_host_tests.sh`; (B) the Python oracle and the goldens. This file records
every place where they disagreed (or where either disagreed with the written
spec), the root cause, and how it was resolved. Each resolution was checked
with a third computation written from the spec text (a scratch script, not
committed), never by copying one side into the other.

## Disagreements and resolutions

| # | What | A (C++) | B (oracle) | Root cause | Resolution |
|---|---|---|---|---|---|
| D1 | Rounding of the decay cap in `speedMmps(now)` | floor: `597400000 / d` | round half up: `(597400000 + d/2) / d` | The spec writes the cap as the plain quotient `M·10⁹/(now−t_last)` with no rounding term, and states the bound `v ≤ M/(now−t_last)`, which only flooring guarantees. Test 1.9 says the speed "equals M/(now−t_last)". B reused the *span* conversion (which does round) for the cap. | **Oracle fixed to floor.** The goldens `metrics_*.txt`, `bias_*.txt` and `README.md` were regenerated. See "D1 evidence" below. |
| D2 | Scope of the golden drift check | `diff -r -x README.md` (assumed the README was hand-written) | the oracle writes `golden/README.md` itself | A could not see B's output. | `run_host_tests.sh` runs a plain `diff -r $TMP test/golden`, so the format contract can't drift from the oracle either. |
| D3 | B's README claim about the continuous 1.11 integral | n/a | "integrating the integer cap instead changes the result by < 0.0001 m" | That held for the rounded cap. With the (correct) floored cap, a µs-exact integration of the integer `speedMmps` moves the result by −0.0107 / −0.0018 / −0.0046 m (S2 / S1b / S1_handturn). | Text corrected. Both sides integrate the real-valued tail `M/(t−t_k)`, which is what the written definition says, and they agree to 1e-6 m. |

### D1 evidence (independent re-derivation)

The independent script re-implemented the span estimator, `speedMmps` and the
1.11 definition from the spec text, with both cap roundings:

| Log | Spec (quoted) | floor (min / max / mean) | nearest (min / max / mean) |
|---|---|---|---|
| S2_walk | −2.26 / −0.41 / −1.33 | **−2.2602 / −0.4142 / −1.3265** | −2.2522 / −0.4022 / −1.3163 |
| S1b_10rev | −0.28 / +0.20 / −0.02 | **−0.2804 / +0.2046 / −0.0248** | −0.2784 / +0.2056 / −0.0229 |
| S1_handturn | −5.42 / −4.60 / −5.01 | **−5.4196 / −4.6016 / −5.0055** | −5.4156 / −4.5956 / −5.0011 |

Floor reproduces all 9 quoted phase figures to the quoted 2 dp. Nearest misses
5 of them (S2 −2.25, −0.40, −1.32; S1b +0.21; S1_handturn −5.00). The
continuous figures (−1.171324 / −0.012603 / −5.060662) and truths
(255.6872 / 36.4414 / 97.9736 m) don't depend on the cap and match on all
three sides. Tests 1.6 (446 / 3514 / 1195 mm/s, which do round) and 1.5 were
also re-derived by hand and match.

After the fix, 1.10 shows a worst |C++ − oracle| of **0 mm/s** on all 20 Hz
samples (it was 1 mm/s before), the odometers match exactly, and every
`bias_*` value agrees to 1e-6 m.

## Points where both sides agreed but the spec text differs

| # | Spec text | Implemented | Why |
|---|---|---|---|
| P1 | PulseFilter pseudo-code: `reset: armed = (initialLevel == 1)` | The original pulse logger's semantics: the boot state is an open pseudo-edge at `t0`, so `armed=false` at reset and the first closure needs ≥ 5 ms after `t0` (4999 µs rejected, 5000 µs accepted; test 1.2) | The spec makes the pulse logger's `acceptClosure` authoritative, so the pseudo-code loses to the test. Test 1.1 reproduces every logged P exactly with those semantics. |
| P2 | 1.15: median console/ours ratio "1.143 ± 0.02" | 1.148412 over 124 samples (A and B used the same sampling, written independently: F lines after marker 1, flags bit 0 clear, both speeds > 0, ratio = console/ours in mm/s) | The spec doesn't define the sampling. The result is within the stated tolerance. The 1.143 figure probably came from a different research sampling. It's a sanity check only. |
| P3 | 1.12 windows not defined; research figures 0.8% / 2.2% | Sprint plateau [1139, 1143] s → 0.72%; walk [1217, 1223] s → 1.97% (seconds after the first S2 pulse; RMS of (v−m)/m against the ±2 s centred mean) | The limits (≤ 1.0% / ≤ 3.0%) pass with margin. The test also asserts each window's mean speed (≈12.3 / ≈4.64 km/h) so it can't measure the wrong segment. |
| P4 | 1.11: "n = all P lines of the log" | Both use the re-filtered pulses | Identical by test 1.1 (the C++ asserts timestamp equality with the logged P lines). |

## Minor, non-material differences (left as is)

- Banner detection: the C++ matches an S text **starting with** `M4 pulse logger`
  or `OpenTread fw=`; the oracle matches one **containing** `pulse logger` or
  `OpenTread fw=`. Both find exactly one banner in each log and both reject a
  banner without `line_level`.
- Marker for 1.15: the C++ takes the first `M` line; the oracle takes the
  `M` line with n = 1. S2_walk has exactly one marker (n = 1).
- Both stable-sort events by `t_us`. S1_handturn has one out-of-order `S`
  line (`scan match`, 1 µs early), which affects neither the filter nor the metrics.

## Parser strictness review

| Finding | Verdict | Reason | Fix |
|---|---|---|---|
| replay.cpp parser looser than its grammar and the oracle | **CONFIRMED** | A scratch probe linked against `test/replay.cpp` accepted `E,1000,01`, `E,-0,1`, `P,100,1,-0`, `H,1,-1,…,-7`, `F,1,A6`, plus `M,-0,1` and `S,-0,hello`, all with `malformed=0`. The oracle raised `LogError` on every one. | `replay.cpp`: t, n, dt and the H fields must be ASCII digits only, so a sign is malformed and leading zeros are still allowed, as with the oracle's `\d+`. The E level must be exactly `"0"` or `"1"`. F hex is lowercase only. `testParser` now counts all 9 probe lines as malformed (18 total) and checks that `E,0001250,1` is still accepted. |

The probe also showed that the oracle was looser than the C++ in two ways (the same
defect class, in the other direction). These are fixed in the oracle so both parsers
follow one grammar:
- A `str` regex `\d` also matches non-ASCII digits, so `E,١٠٠,1` was accepted as t=100. Fix: `re.ASCII` on every `_RE` pattern.
- `open(..., newline="")` also ends a line at a lone `\r`, so `E,5,1\rE,6,1` became two valid edges. The C++ splits only at `\n`. Fix: `newline="\n"`.

The committed logs contain no lone CR and no non-ASCII bytes. **The goldens are
unchanged.** `run_host_tests.sh` regenerated them and `diff -r` was empty.

## Workout integration (2026-10-23)

The C++ workout core and the oracle's workout model were written in parallel by
two agents with no access to each other's code. Result: **all 11 workout goldens
match the C++ exactly** (W timestamps to the µs, Z lines byte-identical) on the
first integration run; `test_workout` 476/476; earlier tests unchanged.

Both sides independently chose the same resolutions for the spec's ambiguities:
- **Pause counting.** The spec's "N pauses" (S2 6, S1b 0, handturn 11/7/6) counts
  pauses that were followed by a resume. The goldens' `pauses=` footer counts every
  W `pause` line, including the final pause before auto-end (S2: 7). Transitions agree;
  only the summary statistic's definition differs.
- **One STOP value** drives both Metrics (speed) and Workout (2.4 sensitivity cases
  change both); only that reproduces the spec's 2.4 moving times.
- **Top speed** excludes the confirm (baseline) pulse; resume pulses have speed 0.
- **Last-digit differences vs the spec** (217.726 vs 217.7265 s, top 4.317 vs 4.316 km/h):
  the spec's research used real-valued speeds; we use the pinned integer arithmetic.
  All within the spec's tolerances (±1 ms, ±0.01 km/h).

C++-only choices (not exercised by goldens):
- Event queue has 10 types (adds BoutStarted, Rearmed so the app can print those W lines).
- Same-instant ordering: `summary_timeout` (≥ guard) before the strict (> guard) transitions.
- Hold result order: no active hold → Ignored; open workout id changed → Void; began or
  completes in IDLE/SUMMARY → Ignored; WARMUP → cancel; RUNNING/PAUSED → stop.
- During a hold, top speed freezes with the other workout fields.
- `OT_SHORT_TIMERS=1` shortens AUTO_END to 30 s and SUMMARY to 12 s (and DIM/OFF),
  matching how replay.py models `short=1` logs.

## Workout review (2026-10-23)

- **Hold began in IDLE/SUMMARY.** The C++ already returned `Ignored`
  for a hold that *began* with no stop button shown, whatever the state at completion;
  the oracle only looked at the completion state and cancelled a warm-up that started
  during such a hold. The spec (as clarified) sides with the C++, so the
  **oracle was changed** (it now records the hold-begin state) and two synthetic goldens
  pin the rule: `workout_synth_hold_idle_to_warmup.txt`,
  `workout_synth_hold_summary_to_warmup.txt` (the pre-fix oracle produces a
  `hold_cancel_warmup` in both). The 11 existing workout goldens are byte-identical;
  `golden/README.md` changed (new cases and the clarified hold rule).
- **Input time clamp.** Both models keep `latestUs` (max t of
  every input and tick) and process every input at `max(t, latestUs)`; snapshots are
  evaluated at `max(now, latestUs)`; the hold calls return that t_eff (the app logs it in
  the I line). The goldens' inputs are in time order, so none of them changed. A 2000-case
  differential fuzz with out-of-order inputs (C++ vs oracle, scratch only) found no
  difference.
