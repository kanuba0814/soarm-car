# Prior Map Assisted Exploration Core

`scripts/exploration_core.py` is an importable, deterministic CPU core. It is not a route catalog, Nav2 replacement, ROS node, simulator, or freshness gate. The pinned read-only map input is `config/prior_map_assisted_exploration_input_lock.json`; loader validation checks PGM digest, dimensions, resolution, origin, and map identity before construction.

## Integration Contract

The JSON CLI input is validated by `schemas/exploration-observation.schema.json`; output is described by `schemas/exploration-candidate.schema.json`. Run:

```bash
python3 scripts/exploration_core.py request.json shortlist.json
```

The request provides `prior_manifest`, `robot_pose`, optional bounded `config`, and ordered `observations`. Every observation has a unique `id`, strictly increasing positive `revision`, `map_identity`, `frame: "map"`, `sensor_origin` XYZ, `robot_pose` XYZ, optional opaque `source_stamp`, and finite measured returns. A duplicate ID is idempotent; stale revisions, map mismatch, wrong frame, invalid pose, nonfinite returns, and nonzero map origin yaw are rejected. The core deliberately never reads a wall clock.

Each return is `{ "xyz": [x,y,z], "kind": "obstacle" | "floor" | "hit" | "no_return" }`. `no_return` conveys no observation unless the integration adapter has separately calibrated an explicit finite return semantics. Rays start at the sensor origin and stop at the first prior obstacle or finite endpoint. Floor and high returns add visibility only: they do not promote prior unknown cells or certify a body volume. Live obstacle evidence is monotonic and wins independent of observation order.

`shortlist.candidates` is an ordered Nav2 adapter input, each with a map-frame `goal`, stable `cluster_id`, predicted gain, and distance. Before submitting one, the adapter must call Nav2 `ComputePathToPose`; this core verifies only observed/prior configuration-space connectivity. Report execution with `report_result(cluster_id, gained_observed_cells, succeeded)` so failed/no-gain clusters are retried no more than `max_attempts`.

Coverage is `observed_area_cells`; `reference_prior_connected_area_cells` is a declared-prior reference only, never a universal completion claim. Status is `ok`, `no_frontier`, `blocked`, or `exhausted`.

## Safety and Limits

Initial observation is all unknown even when the prior says free. Candidate approaches are kept on the observed side of the frontier and must clear the enclosing robot radius `sqrt(.3^2 + .3^2) + .05` using circle/cell intersection; frontier cells themselves are not required to hold the entire footprint. Out-of-bounds, prior unknown, live obstacles, and unobserved footprint cells fail closed. Geometry is never shrunk. `max_rays` defaults to 2048 and `max_candidates` to 16; no all-pairs ranking is used. At the locked 320x290 map size, the three boolean-like grid states are approximately 278400 bytes before Python container overhead.

The core owns neither clock freshness nor sensor calibration. A PCD prior can be declared by the localization layer, but it cannot seed `observed` coverage.
