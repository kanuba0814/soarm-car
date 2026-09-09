# Findings

## Blocker

The sole `direct_mppi` execution did not reach Nav2 readiness or submit an action. Its assigned `ROS_DOMAIN_ID=246` is outside the Fast DDS range supported by this runtime: every launch participant reported `Calculated port number is too high. Probably the domainId is over 232 or portBase is too high.` The runner exited `1` after its readiness wait. See `nav2-container.log` and `run-timing.txt`.

## Compatibility Finding

The pre-run named request was prepared and revalidated against the recovered asset root. It resolved `room-1` to `[-1.2240907573593973, 5.400267829207922, 0.0]`, the same numeric catalogue pose used by the unchanged route consumer. The consumer maps the selected `room-1` route to `room-1` followed by `hub`, whose catalogue entry is `mian` at `[0, 0, 0]`. See `named-request-room-1-prepared.json`, `named-request-room-1-verified.json`, and `runtime-snapshot.json`.
