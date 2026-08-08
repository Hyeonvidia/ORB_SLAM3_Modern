# Tracking state-transition traces (P7-1a)

Reference captures of the `Tracking` state machine, produced by the opt-in
instrumentation added in P7-1a. Golden site census and transition table:
`docs/P7_RECON.md` §A.

## How to regenerate

```bash
docker compose run --rm \
  -e HEADLESS=1 \
  -e ORB_TRACE_STATE=1 \
  -e ORB_TRACE_STATE_FILE=/workspace/benchmark/golden/state_traces/<mode>.trace \
  dev docker/scripts/run_slam.sh <mode> <sequence>
```

`ORB_TRACE_STATE=1` turns tracing on (anything other than unset/empty/`0`
works). `ORB_TRACE_STATE_FILE` picks the destination; without it the trace
goes to stderr. Tracing is **off by default** and costs one bool test per
state write when off, so the instrumented binary is the shipping binary.

Captured here (counts exclude the leading construction line):

| file | mode | sequence | transitions | ATE RMSE of that run |
|---|---|---|---|---|
| `euroc_mono.trace` | `euroc_mono` | EuRoC MH01 | 2 | 0.017698 |
| `euroc_stereo.trace` | `euroc_stereo` | EuRoC MH01 | 2 | 0.049775 |
| `euroc_mono_inertial.trace` | `euroc_mono_inertial` | EuRoC MH01 | 62 | 0.089406 |
| `kitti_stereo.trace` | `kitti_stereo` | KITTI 07 | 2 | 0.536298 |

The three visual modes are the minimal healthy shape: construct, promote to
`NOT_INITIALIZED` on the first image, initialize to `OK`, never lose tracking.

`euroc_mono_inertial` is the outlier, and the trace makes the reason legible:
15 initializations, 14 `ResetActiveMap`s and 11 `OK -> RECENTLY_LOST`
(`TrackLocalMap failed (inertial)`) — the tracker repeatedly initializes, fails
IMU-aided local-map tracking within a few dozen frames, and resets. This is the
**pre-existing** monocular-inertial behavior on this build, not something P7-1a
introduced; the run's ATE (0.089) sits inside the recorded baseline spread for
this mode (0.068-0.124, `../baseline_ate_distribution.csv`). The trace is a
useful handle on that open issue.

## Format

One line per **actual** state transition:

```
<frameId> <fromState> -> <toState> <reason>
```

- `frameId` is `mCurrentFrame.mnId` at the moment of the write. On the
  construction line it is `-`, because no frame exists yet. On reset and
  map-fork paths `mCurrentFrame` can be default-constructed or stale; the
  value is recorded as-is rather than guessed.
- States are printed as enumerator names (`NO_IMAGES_YET`, `NOT_INITIALIZED`,
  `OK`, `RECENTLY_LOST`, `LOST`), never as integers.
- `reason` names the call site.

**No-op self-assignments are not logged.** The `mState = OK` at the top of the
normal tracking path re-assigns `OK` on essentially every tracked frame; those
writes have no observable effect and would bury the real transitions. Verified
against the P7_RECON census: no site depends on re-assigning the same value.

**One deliberate exception:** `LocalMapping::InitializeIMU (cross-thread)` is
logged even when it is a no-op (`OK -> OK`). That write comes from the
local-mapping thread (`docs/P7_RECON.md` transition T22, and the warning block
at the top of that document) and usually lands while the tracker is already
`OK`. It is the one transition whose *occurrence* is the interesting fact, so
it is never dropped. Its presence in `euroc_mono_inertial.trace` is the direct
evidence that the cross-thread writer survived the P7-1a encapsulation.

## These are a diagnostic reference, NOT a bit-exact gate

Do not diff them in CI. ORB-SLAM3 runs local mapping, loop closing and the
viewer on their own threads, so the exact frame ids at which tracking is lost,
recovered, or reset vary run to run — as does the frame id attached to the
cross-thread IMU-initialization line, which depends purely on when the
local-mapping thread happens to finish `InitializeIMU()`. Sequences with no
tracking loss are stable in *shape* (init then nothing), but nothing here is
guaranteed reproducible at the line level.

Use them to answer "which transitions does this mode actually exercise, and
roughly when", and to spot **structural** regressions: a transition kind that
appears or disappears entirely, an initialization that stops happening, a mode
that starts thrashing between `OK` and `RECENTLY_LOST`. Precision regressions
are the ATE gates' job (`benchmark/scripts/smoke_gate.sh`,
`benchmark/scripts/full_gate_paired.sh`).
