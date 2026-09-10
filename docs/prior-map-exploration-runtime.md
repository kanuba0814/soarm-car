# Prior Map Exploration Runtime

Run only after the NVIDIA runtime owner has released the validated restored Nav2 runtime. Execute this inside that same container and ROS overlay; it does not launch Gazebo or a second navigation stack:

```bash
EXPLORATION_EVIDENCE_DIR=/output/prior-map-run-001 \
NAV2_ASSET_ROOT=/workspaces/soarm-car \
./scripts/run-prior-map-exploration-task.sh \
  --ros-args
```

`prior_asset_root` relocates only the locked prior manifest's YAML and PGM paths
from the original read-only worktree to the validated recovered asset root. The
map identity, PGM SHA-256, and geometry remain unchanged. The relocated manifest
is retained as `relocated-prior-manifest.json` in the evidence directory.

The node requires generated `soarm_interfaces/msg/LocalizationHealth` in the
runtime image. Its CPU-only no-network smoke command is:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/soarm-car/install/setup.bash
PYTHONPATH=/work/scripts:$PYTHONPATH \
timeout 3 python3 /work/scripts/prior_map_assisted_exploration_ros.py --ros-args \
  -p evidence_dir:=/output/evidence \
  -p prior_asset_root:=/workspaces/soarm-car \
  -p asset_root:=/workspaces/soarm-car
```

This starts no Gazebo, renderer, devices, publishers, or goals. A timeout exit
only proves the node consumer remains alive during the startup data grace; it is
not live E2E evidence.

## Native Selector Compatibility

The named-place lock accepts only the original audited launch SHA-256
`c88daf45e9627d79fc13812b27795f57dd9936ebb39b40ba475aba3bbb7a3273` and
the audited selector-only overlay SHA-256
`a520fa54f04feb26b2ceb8da8723046b89c2c8ab1d4cb1444cba63df17ec94d5`.
This file-identity allowance does not establish which runtime backend is active;
that requires runtime evidence.

## Native Cloud Interpretation

The native lidar has 720 horizontal samples and 16 vertical samples across
`[-0.4, 0.4]` radians, not degrees. Observations use source-stamped
`base_link <- livox_frame` and `map <- livox_frame` transforms. The frozen
robot SDF hash is checked before use; only its documented body boxes and wheel
spheres are self-filtered. A measured 0.1 mm numerical surface tolerance is
recorded with each observation after a raw float32 residual measurement.

Map points remain in `map`. Floor and navigation-return height are evaluated
relative to the source-stamped `map` Z of `base_footprint`: floor tolerance is
0.03 m and navigation-return band is 0.03 to 1.50 m. The adapter retains the
nearest non-self navigation return in every one of 720 azimuth bins before
adding bounded floor/high visibility extras. It records transform matrices,
selected raw indices, ranges, model hash, ground reference, and policy counts
for offline replay.
