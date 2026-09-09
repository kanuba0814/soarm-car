# DEV Grid Configuration-Space Correction v3

This is a CPU-only correction of the offline VLM/grid development evidence. It replays the same four public DEV RGB-D observations and does not run a VLM, simulator, ROS, Nav2, or heldout evaluation. v1 and v2 artifacts remain unchanged.

## Corrected Invariant

The earlier v1 implementation had two geometry defects:

1. A non-obstacle ray endpoint wrote `FREE` unconditionally, so a later free observation could erase a previously occupied endpoint.
2. Obstacle-only padding did not reject robot-center paths whose 0.50 m footprint crossed `UNKNOWN` cells or the map boundary.

v3 preserves occupied endpoint precedence regardless of ray order. It produces a configuration-space grid in which a robot center is `FREE` only if **every** cell in its radius disk is in bounds and observed `FREE`. Any `UNKNOWN`, `OCCUPIED`, or out-of-bounds disk member blocks the center. The path routine is honestly named `bfs_shortest_path`: it is BFS, which returns a shortest 4-connected path on the configuration-space lattice; it is not A*.

The corrected output is [results.json](/home/gxxl/soarm-car-vlm-grid/artifacts/companion-home-validation/evidence/vlm-grid-dev-v3/geometry-results/results.json:1), with configuration-space overlays alongside it.

## Measured Change

| Measure | v1 legacy lattice | v3 full-footprint lattice |
| --- | --- | --- |
| Observations | 4 | 4, identical public DEV RGB-D inputs |
| Robot radius | 0.50 m obstacle padding | 0.50 m entire disk must be observed free |
| Geometry-feasible choice | 1/4 (`obs_002:1`) | 0/4 |
| Unknown interpretation | Center may be free beside unknown | Unknown/boundary anywhere in disk blocks center |
| Search name | Incorrectly called A* | BFS shortest 4-connected path |

The prior `1/4` geometry-only feasibility result is **superseded**. Under the corrected gate, all candidate approach rings are rejected as `all_approach_footprints_occupied_or_unknown`. This is an expected conservative result, not a reason to infer obstacle-free space from absent RGB-D returns.

Even a free v3 configuration cell is conditional single-view RGB-D height-classification evidence, not a collision proof. A reconciled persistent map and the existing navigation safety stack remain necessary before action execution.

## Reuse Boundary

This change does not implement or modify navigation. Per `/home/gxxl/soarm-car/WORKSPACE_INDEX.md` at root commit `3ac2d5c`, the authoritative reusable navigation worktree is `/home/gxxl/soarm-car-nav2-clean`, baseline `experiment/nav2-clean-baseline` at `41463f9`, plus its current dirty paired configuration and scripts. Its saved `artifacts/nav2-clean-direct-mppi-20260905T173147Z/summary.json` records `cases.direct_mppi.complete=true` and `passed=true`. Do not replace that baseline, do not checkout its HEAD as a complete snapshot, and do not claim this offline geometry correction means overall navigation is broken.

## Residual Risks

- Four sparse DEV observations cannot demonstrate safe action execution or persistent-map reachability.
- Conservative unknown blocking may produce no route even when a later multi-view or mapped system could validate one.
- VLM format/semantic results in v1/v2 remain development-only and independent from this CPU correction.
- Qwen3-VL-2B remains unmeasured because its earlier pinned download timed out; no retry was performed here.
