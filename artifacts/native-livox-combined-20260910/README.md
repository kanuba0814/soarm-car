# Combined Native And Freshness Run

This distinct profile used the native converter, `throttle_odom_path:=true`, approved health commits `f44472f` and `abee380`, and an opt-in trace-only `health_diagnostics_trace: true` runtime overlay. The 0.5-second freshness threshold, Nav2, command gate, ICP configuration, LiDAR geometry, source timestamps, and original route consumer were unchanged.

The unchanged recovered `nav2-clean-route-consumer.py` was invoked with `NAV2_CLEAN_ROUTE=room-1`. It failed at its bounded sensor/FAST-LIO readiness gate before publishing initialpose: the simulator advertised `/clock`, source PointCloud2, and native Livox endpoints but emitted no clock sample, scan, or odometry. `health-trace.jsonl` records 2,590 steady-time heartbeats with maximum publication gap 0.101565196 seconds. Every trace record has `node_clock_ns=0`, null scan/odom accepted source stamps and receipt ages, `state=2`, and stale-input reason mask 8.

This proves the combined repair does not mask an upstream clock/readiness absence. `publisher-owners.txt` retains native publisher ownership for the cloud topics; no runtime OGRE log existed because the simulator never reached clock/render initialization, recorded in `ogre2-status.txt`. No NVIDIA path was used.
