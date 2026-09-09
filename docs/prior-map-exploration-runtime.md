# Prior Map Exploration Runtime

Run only after the Intel owner has started the validated restored Nav2 runtime:

```bash
EXPLORATION_EVIDENCE_DIR=/tmp/prior-map-run-001 \
./scripts/run-prior-map-exploration-task.sh \
  --ros-args -p prior_asset_root:=/tmp/soarm-intel-nav-restore-20260909
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
  -p prior_asset_root:=/asset \
  -p asset_root:=/asset
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
