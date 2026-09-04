# Kuruma east-doorway navigation failure

## Reproduction

Goal recorded from RViz:

- position: `(4.08818, 0.561433)`
- yaw: `-1.57291 rad`

The robot repeatedly stops at the east doorway near ground-truth position
`(3.0–3.3, 1.8–2.2)`. Nav2 then reports `Failed to make progress` and/or
`Starting point in lethal space` and cannot arrive.

## Verified diagnosis

The doorway is approximately `0.914 m` wide. The STEP/SDF collision perimeter
is `0.600 m` square, leaving little room for localization error. During a
recorded failure, `/localization` ended near `(3.112, 1.989)` while scoring-only
`/ground_truth/odom` was `(3.275, 2.141)`: approximately `0.16 m` error on both
axes. That is enough for Nav2's footprint to overlap the doorway endpoint.

The reference PCD is incomplete. Overlaying
`artifacts/fastlio_house_floorplan.pcd` on the occupancy grid shows good
agreement where points exist, but almost no mapped wall geometry at the east
doorway or destination room. The generator in
`scripts/dev-generate-house-pcd.sh` deliberately records only a short arc in
the initial open area. As the robot reaches the unmapped doorway, ICP fitness
falls from about `0.99` to `0.25–0.35` while RMSE remains about `0.19–0.25 m`.
No registration algorithm can recover a unique pose from fixed-map geometry
that was never recorded.

A fixed-map C++ shadow run confirmed the limitation rather than providing a
safe correction: the estimate was observable (`rank=3`) but the best/second
pre-search score gap was only `0.00010–0.00030`, indicating ambiguous repeated
wall hypotheses.

## Experiments rejected

- Lowering ICP fitness acceptance from `0.4` to `0.2` kept health green but did
  not remove the roughly `0.2 m` pose bias or allow arrival.
- Open3D point-to-plane refinement was slower and did not restore overlap.
- Humble Smac State Lattice with bundled omni primitives did not resolve the
  failure with the current incomplete localization map and controller stack.
- Adding the static map to the rolling local costmap couples its map-frame pose
  error into local collision avoidance and can stop the robot earlier.
- Blindly shrinking the physical footprint or widening correction jumps would
  hide the missing-map defect and is unsafe for hardware.

## Retained improvements

- Preserve MID-360 returns from `0.20 m` instead of discarding everything below
  `0.60 m`, so nearby walls remain visible to local collision avoidance.
- Use the measured `0.600 m` square collision perimeter in both costmaps.
- Keep local collision avoidance in `odom_combined` using live lidar; keep the
  static occupancy map as global route authority.
- Use bounded low-overlap dead reckoning rather than declaring the last map
  alignment invalid after five low-fitness scans. This never accepts a failed
  ICP correction and still stops at strict time/motion bounds.
- Limit translation speed to `0.25 m/s` while localization coverage is being
  validated.

## Required closure

Create a complete fixed-map PCD by driving a supervised mapping route through
the east doorway and destination room (in simulation and later on hardware),
or replay a representative sensor-only mapping bag. Before enabling navigation,
validate actual structural point support in every traversable region—not only
the PCD AABB. Then calibrate the fixed-map shadow uniqueness/observability
thresholds on positive and repeated-wall negative samples, and promote it only
after this exact goal succeeds repeatedly without ground-truth input.
