# Kuruma localization algorithm selection and repair plan

## 1. Outcome

Select a production-portable localization stack using repeatable evidence from synthetic fixtures, deterministic simulation routes, recorded MID-360 bags, and a Pi 5 runtime gate.

The selected stack must prevent the observed failure chain:

1. fixed-map matching becomes invalid;
2. an old correction remains operational;
3. map-frame error grows to approximately 0.38 m;
4. Nav2 places the estimated footprint in lethal space;
5. planning, Spin, and BackUp all become impossible.

A lower navigation speed is not considered a localization fix.

## 2. Binding invariants

- FAST-LIO remains the high-rate local odometry source.
- The global localizer publishes correction measurements; it does not overwrite FAST-LIO state.
- `transform_fusion` remains the only operational map/odom TF owner.
- Ground truth is scoring-only and is never consumed by localization, health, navigation, command gating, or recovery.
- Normal tracking never performs an unbounded/global search.
- Equality at a safety threshold is rejection.
- No correction is committed from fitness/RMSE alone.
- Relocalization occurs only while the base is stationary.
- Invalid or stale localization stops map-TF publication and motion authorization.
- Generic behavior remains unchanged unless the Kuruma profile opts in.

## 3. Module seam

All algorithms are compared behind one deep module interface:

```text
FixedMapLocalizer
  load_map(structural_map) -> MapStatus
  estimate(scan, prior, mode) -> LocalizationMeasurement
```

`mode` is either bounded `TRACKING` or stationary `RELOCALIZATION`.

`LocalizationMeasurement` contains:

- `T_map_cameraInit` candidate;
- state and rejection reason;
- fitness, RMSE, inlier support;
- SE(2) observability spectrum/ratio;
- best-versus-second hypothesis separation;
- innovation from the prior;
- source/map timestamps and map identifier;
- execution time and memory statistics.

Adapters:

- deterministic bounded SE(2) + small_gicp refinement;
- fixed-map NDT;
- Scan Context candidate retrieval feeding one of the geometric adapters.

This keeps TF, health, Nav2, and test logic independent of the chosen matcher.

## 4. Candidate algorithms

### A. Bounded SE(2) + small_gicp

Use the existing immutable full-map model, deterministic correlative search, bounded planar refinement, observability, and ambiguity outputs.

Role: primary normal tracker and geometric verifier.

Expected advantages: deterministic, native C++, strict trust region, existing tests and installed-package seam.

Known risks: repeated corridors remain unobservable; current live search is approximately 0.25 s and needs profiling/optimization.

### B. Fixed-map NDT

Implement an adapter with the same inputs/results and Autoware-style validation/diagnostics.

Role: direct A/B comparison for normal tracking and bounded stationary recovery.

Expected advantages: smoother voxel-distribution objective and established fixed-map localization precedent.

Known risks: resolution-sensitive, weaker behavior in sparse geometry, more configuration, and no guarantee against corridor ambiguity.

### C. Scan Context + geometric verification

Use descriptors only to retrieve a small set of spatially distinct map hypotheses. Every hypothesis must then pass A or B, observability, ambiguity, and multi-scan confirmation.

Role: stopped relocalization and kidnapped-robot recovery only.

Known risks: perceptual aliasing in repeated rooms/corridors; never sufficient as a standalone acceptance signal.

### D. Pose graph / loop closure

FAST-LIO-SAM/SC-PGO-style backend.

Role: optional future solution for long repeated routes and mapping sessions.

It is excluded from the first selection round because it changes lifecycle, memory growth, and map semantics. Adopt only if A/B/C pass local tests but long-loop drift remains unacceptable.

## 5. Test corpus

### Tier 1 — deterministic synthetic fixtures

Each adapter must process identical structural cells and priors:

1. corner/two nonparallel walls — observable positive case;
2. single long wall — reject along-wall degeneracy;
3. repeated parallel corridor — reject equal hypotheses;
4. floor-dominated scan — reject after structural filtering;
5. fewer than 50 structural cells — reject without fallback;
6. NaN/Inf/extreme coordinates — fail closed;
7. moving-object outliers — robust result or explicit rejection;
8. exact tracking boundaries: `0.249 m/0.149 rad` eligible, `0.250 m/0.150 rad` rejected;
9. alternating hypotheses over three scans — never commit;
10. three compatible independent scans — commit exactly once.

Run each fixture 100 times and require identical state, candidate ordering, and rejection reason.

### Tier 2 — captured simulation incidents

Convert the two recorded failures into replayable scenarios:

- Goal A: `(3.78034, -5.14132, 3.08703)`.
- Goal B: `(6.09, -2.95)` from approximately `(3.64, -5.14)`.
- Failure-B scoring point: localization `(5.018, -4.491)`, truth `(5.367, -4.337)`, ICP fitness `0.323`.

Scenarios:

1. fresh start to each goal;
2. return route followed by each goal;
3. injected planar drift: `0.15`, `0.25`, `0.40`, `0.70 m` and `0.10`, `0.20`, `0.35 rad`;
4. moving drift injection — correction must not commit;
5. stopped recovery from the same drift;
6. LiDAR dropout, odometry dropout, localizer crash, stale map;
7. repeated corridor with a deliberately wrong alternative prior;
8. map obstacles changed without changing runtime truth inputs.

Use at least 20 deterministic runs per route/profile. Ground truth only scores output.

### Tier 3 — physical MID-360 bags

Record before selecting production thresholds:

- stationary 60 s in an observable room;
- straight corridor in both directions;
- 90° doorway entry and exit;
- long-wall traversal;
- closed loop returning to start;
- stop-and-relocalize at corners and in an ambiguous corridor;
- people/moving obstacles;
- cold start and deliberately displaced initial pose;
- vibration and normal chassis acceleration.

Bags must include raw Livox/IMU, FAST-LIO odometry, TF/static extrinsics, commands, map ID, and timestamps. No ground truth is required for runtime; surveyed checkpoints or an independent measurement system may be used offline for scoring.

## 6. Hard gates

A candidate is eliminated immediately if any safety gate fails.

### Safety and correctness

- Zero false correction commits in all negative/ambiguous fixtures.
- Localization becomes non-healthy within `0.5 s` of stale input or matcher death.
- No stale `map→odom` TF or `/localization` re-stamping while unhealthy.
- Hardware command reaches zero and the active Nav2 action is canceled within `0.5 s` of unhealthy localization.
- No automatic resume after recovery; a new goal is required.
- Runtime graph proves no `/ground_truth/*` subscription by estimator/control modules.

### Accuracy

Initial targets, subject to real bag confirmation:

- observable tracking p95 XY error `<= 0.08 m`;
- p95 yaw error `<= 0.05 rad`;
- maximum accepted XY error `<= 0.15 m`;
- maximum accepted yaw error `<= 0.10 rad`;
- no accepted correction outside normal `0.25 m/0.15 rad` innovation bounds.

The narrow 0.781 m passage with a 0.62 m footprint requires a stricter route-level clearance check; meeting a generic 0.20 m localization threshold is insufficient.

### Recovery

- `0.40 m` planar drift: tracking rejects, robot stops, bounded stationary recovery succeeds within 30 s in observable scenes.
- Recovery requires three compatible scans within `0.125 m/0.075 rad`.
- Recovery searches no farther than `0.75 m/0.35 rad` automatically.
- Ambiguous/degenerate recovery ends in `LOST`, never in a guessed correction.
- Cold/kidnapped recovery success `>=95%` in observable test locations and zero false positives in the test corpus.

### Navigation consumer seam

- 20/20 runs for both recorded goals succeed without `Starting point in lethal space` caused by localization drift.
- Zero footprint collisions.
- No Spin/BackUp command bypasses localization or collision safety.
- Rotation Shim behavior remains available independently of localizer selection.

### Pi 5 performance

At a provisional 1 Hz global-correction rate while FAST-LIO and Nav2 run:

- callback p95 `<=250 ms`, p99 `<=500 ms`;
- average localization CPU below `0.5` core;
- incremental RSS `<=128 MiB`;
- fixed-field payload `<=64 MiB`;
- no map rebuild per cloud and no callback backlog during a 30-minute replay.

## 7. Experiment funnel

### Phase 0 — make the bug replayable

- Save sensor-only data for both blocked goals.
- Add an assertion for the exact failure: after consecutive matcher rejection, an old correction must not remain operational until Nav2 reports lethal start.
- Record map, config, software commit, random seeds, and map hash.

Exit: one unattended replay reliably turns red on stale correction/lethal start.

### Phase 1 — algorithm-independent health fix

Before comparing matchers:

- add explicit `INITIALIZING/HEALTHY/DEGRADED/RELOCALIZING/LOST` health;
- stop stale map-frame output;
- gate final actuator commands;
- cancel active goals on loss;
- preserve local odometry while stopped.

Exit: all dropout/crash/stale tests pass with the existing matcher.

### Phase 2 — offline A/B comparison

Run bounded small_gicp and NDT on exactly the same Tier-1 fixtures and bag frames. Do not tune against the final route; use a training subset and freeze parameters before evaluation.

Exit: eliminate any adapter with false commits, nondeterminism, missed latency, or inferior accuracy/observability separation.

### Phase 3 — normal tracking shadow test

Run the surviving adapter(s) in shadow mode through both simulation routes and hardware bags. Compare candidate pose, health, ambiguity, and latency without publishing operational TF.

Exit: one adapter passes accuracy, safety, and Pi timing gates.

### Phase 4 — stopped relocalization

Add bounded multi-start recovery to the winning geometric matcher. If cold/kidnapped cases cannot meet the success gate, add Scan Context retrieval, but retain geometric verification and three-scan confirmation.

Exit: recovery passes positive cases and rejects repeated-corridor aliases.

### Phase 5 — operational simulation

Promote behind an opt-in profile. Run fault injection and 20 repeated executions of both recorded goals. Compare against the frozen baseline.

Exit: no lethal-start failure, no false correction, no collision, and all consumer-seam gates pass.

### Phase 6 — Pi and physical deployment

Replay bags on target Pi, then perform supervised low-speed field trials. Enable normal tracking first; enable automatic relocalization only after its independent gates pass.

## 8. Decision rule

Hard gates are pass/fail. Among candidates that pass all hard gates, score:

| Dimension | Weight |
|---|---:|
| Safety / false-correction resistance | 35% |
| Tracking and recovery accuracy | 25% |
| Pi latency, CPU, and memory | 20% |
| Recovery coverage | 15% |
| Integration/maintenance cost | 5% |

Use held-out routes/bags for the final score. A candidate failing any hard gate cannot win through a higher weighted score.

## 9. Expected winner

The leading expected stack is:

1. FAST-LIO for high-rate local odometry;
2. persistent full-map deterministic bounded SE(2) search;
3. small_gicp-style bounded geometric refinement for normal tracking;
4. explicit temporal health and fail-closed fusion/command gating;
5. Scan Context candidate retrieval only for stationary `RELOCALIZING`, followed by the same geometric validation.

NDT replaces the bounded small_gicp adapter only if held-out physical bags show materially better accuracy/robustness while meeting the same ambiguity and Pi gates. Pose graph becomes justified only if long-loop drift remains after the fixed-map stack passes local and recovery tests.

## 10. Irreducible limitations

- A single scan in a featureless parallel corridor cannot observe translation along the corridor. Correct behavior is `DEGRADED/LOST`, not a guessed pose.
- Scan Context can confuse repeated rooms; geometric and temporal confirmation remains mandatory.
- A changed or structurally inconsistent map may make safe localization impossible.
- Poor LiDAR/IMU timestamps or extrinsics cannot be repaired by registration thresholds.
- The 0.781 m bottleneck leaves approximately 0.080 m nominal clearance per side; real chassis tolerance, map error, and localization error may make it physically unsafe even with a better algorithm.
- Simulator truth, perfect timestamps, and clean geometry cannot establish real-world thresholds; physical bags and supervised field tests are mandatory.
