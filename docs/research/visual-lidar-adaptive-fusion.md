# Uncertainty-adaptive stereo/active-lidar mapping prototype

## Direction

The first research implementation is **UAF-TM (Uncertainty-Adaptive Fusion with
Temporal Memory)**. It targets the configured moving mecanum base and a DABAI DC1
stereo camera with active laser: stereo supplies dense structure but depth variance
increases quadratically with range; active lidar supplies sparse, high-confidence
geometry. FAST-LIO remains the high-rate lidar-inertial odometry/localization layer,
while UAF-TM is a mapping consumer and does not replace it.

## Algorithm

For each voxel/cell, compute inverse-variance weights. Stereo variance is parameterized
as `sigma_v = a + b*z^2`; lidar variance is sensor-calibrated. The lidar weight is
attenuated by local return density (0.45--1.0), preventing a single return from
creating a hard surface in an otherwise sparse region. The fused estimate is then
passed through a temporal innovation gate: observations farther than `2.5 sigma`
from the last stable map value are held out. This rejects transient visual
occlusions/moving objects and makes the map conservative rather than blurry.

## Reproducible validation

```bash
python3 scripts/visual_lidar_fusion_smoke.py
```

The script creates a synthetic curved wall, sparse active-lidar returns, range
-dependent stereo noise, and a transient visual outlier. It emits
`artifacts/visual_lidar_fusion_smoke.json`; acceptance is fused RMSE lower than
stereo RMSE and a nonzero temporal hold rate. The same interface can later consume
`/stereo/*`, `/camera/depth/image_raw`, and `/livox/lidar` in a ROS node.

## Gazebo integration plan

Use `ros2_ws/src/soarm_simulation/worlds/house_floorplan.sdf` (already present) and
`kuruma_ros2_control.sdf`. Add DC1-equivalent calibrated intrinsics and a co-located
active-laser/ray sensor to `soarm_description`; remap their topics into the fusion
consumer. Record `/tf`, `/odom`, stereo depth and lidar, then compare map RMSE/ATE
against Gazebo ground truth and ablate: stereo-only, lidar-only, static weighting,
and UAF-TM. Current repository already contains FAST-LIO and small_gicp assets, so
those are baselines rather than new dependencies.
