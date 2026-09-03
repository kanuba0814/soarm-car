# Kuruma blocked-goal diagnosis

## Recorded goal

- Frame: `map`
- Position: `x=3.78034 m`, `y=-5.14132 m`
- Yaw: `3.08703 rad`
- Quaternion: `z=0.999628`, `w=0.0272794`
- Initial pose: approximately `(-0.005, -0.020)`

## Reproduced blocker

The goal is globally reachable: a valid global path was produced and retries can arrive. The intermittent failure occurs at the local controller/progress seam near the constrained route. Observed errors include:

- `Failed to make progress`
- `Optimizer fail to compute path`
- missed controller loop deadlines

The original minimal behavior tree aborted instead of replanning or recovering. A Humble-compatible rotation shim plus the standard replanning/recovery structure changes this from a terminal failure into eventual arrival, but does not remove the underlying instability.

## Controlled comparisons

All rows used the same goal and a fresh simulation/initial pose. Ground truth was used only for scoring, never by localization or navigation.

| Profile | Result | Time | No-progress | Optimizer failure | Control deadline miss | Localization-vs-truth XY |
|---|---:|---:|---:|---:|---:|---:|
| Rotation shim + recovery + shadow matcher | arrived | 64.4 s | observed | observed | observed | 0.285 m |
| Same, shadow disabled | arrived | 54.9 s | 1 | 0 | 8 | 0.277 m |
| Close-range obstacle input `0.60 -> 0.20 m` | arrived | 70.9 s | 2 | 1 | 11 | 0.329 m |
| MPPI batch `2000 -> 1000` | arrived | 85.2 s | 3 | 0 | 7 | 0.336 m |
| Controller/model step `20 Hz/0.05 s -> 10 Hz/0.1 s` | arrived | 98.7 s | 1 | 1 | 0 | 0.260 m |
| Persistent full-map matcher in non-authoritative shadow mode | arrived | 132.6 s | 4 | 2 | 7 | old 0.268 m / shadow 0.261 m |
| Translation limit `0.4 -> 0.2 m/s`, fresh run | arrived | 42.7 s | 0 | 0 | 0 | 0.385 m |
| Translation limit `0.4 -> 0.2 m/s`, return + repeat | both arrived | 101.2 + 53.2 s | 4 total | 0 | 3 total | return 0.055 m / repeat 0.335 m |

The earlier parameter experiments were rejected and reverted. The low-speed value remains an uncommitted visual experiment. Shadow matching remains disabled by default and non-authoritative.

## Current conclusion

1. Shadow localization CPU load amplifies the failure but is not the root cause.
2. Lowering the LiDAR obstacle minimum range adds work/cost and makes this route worse; it is not a valid fix.
3. Reducing MPPI samples or frequency reduces some deadline misses but materially worsens route completion.
4. The recovery tree is a portable safety/resilience improvement, not a complete fix.
5. During the route, ICP fitness fell from about `0.99` to `0.50`; RMSE rose as high as about `0.25 m`, and map-frame localization drift remained about `0.26–0.34 m`. This consumes a large fraction of narrow-passage clearance and is the leading remaining cause of unstable local planning.
6. The full-map shadow matcher correctly fails closed in the corridor: a representative candidate used 152 source cells and 1629 map cells, but had observability rank `0` and a best/second score gap of only `0.000877`. Search took `0.249 s`, refinement `0.025 s`, total `0.274 s`. The scene is locally ambiguous/degenerate, so accepting this candidate would be unsafe.
7. Halving translation speed can remove recoveries in one fresh run, but the effect is not repeatable and localization truth error remains worse (`0.335–0.385 m`). The apparent localization lag is therefore not primarily caused by vehicle speed.

## Second blocked goal captured during low-speed validation

- Goal: `map (6.09, -2.95)`
- Reported start: approximately `(3.64, -5.14)`
- Failure status: `ABORTED (6)`
- Final localization: approximately `(5.018, -4.491)`
- Scoring-only truth: approximately `(5.367, -4.337)`
- Localization-vs-truth XY error: `0.381 m`
- Remaining localization-to-goal distance: `1.878 m`

Failure chain:

1. Controller reported `Failed to make progress`.
2. ICP fitness fell to `0.323` and corrections were repeatedly rejected.
3. SmacPlanner2D rejected replanning with `Starting point in lethal space`.
4. Costmap clearing did not change the static collision state.
5. Both Spin and BackUp failed their collision-ahead checks.
6. Recovery retries were exhausted and Nav2 aborted the goal.

This directly demonstrates that the apparent navigation blockage is downstream of map-frame localization error: once the estimated footprint is inside lethal cost, neither planning nor collision-safe recovery is legally allowed to move it.

## Next solution seam

Keep FAST-LIO as local odometry and the rotation/recovery path as a fallback. The fixed-map matcher needs temporal/multi-scan structural evidence and a separate bounded relocalization/place-recognition path for degenerate corridors; it must continue rejecting rank-zero or ambiguous candidates. Before promotion, optimize deterministic search below the Pi callback budget and prove repeatable route arrival, bounded localization error, no ground-truth dependency, and acceptable CPU latency. Do not relax ICP gates, alter the simulated plant, or tune around the error with smaller footprints/inflation.

Raw run evidence:

- `/tmp/kuruma-blocked-goal-repro.json`
- `/tmp/kuruma-blocked-goal-repeat.json`
- `/tmp/kuruma-recovery-goal-repro.json`
- `/tmp/kuruma-no-shadow-goal.json`
- `/tmp/kuruma-close-range-goal.json`
- `/tmp/kuruma-mppi-1000-goal.json`
- `/tmp/kuruma-10hz-goal.json`
- `/tmp/kuruma-shadow-compare-goal.json`
- `/tmp/kuruma-shadow-compare-events.log`
- `/tmp/kuruma-low-speed-goal.json`
- `/tmp/kuruma-low-speed-repeat.json`
- `/tmp/kuruma-low-speed-repeat-events.log`
- Live container log for the second goal: `kuruma-low-speed` (captured before restart)
