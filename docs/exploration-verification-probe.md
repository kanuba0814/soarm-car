# Exploration Verification Probe

`scripts/exploration_verification_probe.py` is a bounded verifier for an already active real exploration goal. It never creates Nav2 goals, starts or stops simulation, changes external lidar inputs, or forces a source-drop state. Use it only after the exploration owner has released live control.

The helper observes `/navigate_to_pose/_action/status`, `/odom`, `/safe_cmd_vel`, `/localization/health`, and `/clock`. It accepts a control trigger only after a newly observed real action UUID has an active status, fresh advancing source clock and healthy localization, and stamped odometry finite-difference motion above `0.02 m/s` linear or `0.03 rad/s` yaw. It records the actual UUID and goal stamp; old pre-observation goals and concurrent/late goals are not control targets.

The selected Trigger service is called once: `/prior_map_assisted_exploration/pause` or `/prior_map_assisted_exploration/inject_source_drop`. The latter existing runtime service is a toggle, so the helper fails if its response does not prove `injection=True`; it cannot make a safe assertion when a prior caller has already toggled it on. A pass requires its RPC acknowledgement, actual Nav2 `CANCELED` status (5), and fresh source-stamped odometry plus `/safe_cmd_vel` zero evidence sustained for at least 0.5 wall seconds over three distinct odometry stamps. A `SUCCEEDED` status (4) before cancellation is `MISSED_WINDOW`, not a pass.

The command must run in the existing simulation container after copying the helper there and sourcing the workspace that provides `soarm_interfaces` (including `LocalizationHealth`). It does not ship message stubs. Choose a new output directory; existing directories are refused.

```bash
python3 scripts/exploration_verification_probe.py --operation pause --output-dir /tmp/exploration-probe-pause
```

`--observe-only` records actual UUID trajectory without controls. `--budget` is wall-clock bounded to 120 seconds. `--resume-after-verified-stop` is available only in active pause mode: after cancellation and stop proof it requires fresh health and records the existing resume RPC acknowledgement. CPU policy tests are synthetic coverage, not live control evidence.
