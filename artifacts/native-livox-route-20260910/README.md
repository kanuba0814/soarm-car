# Native Livox Original Consumer Run

This is the original recovered `scripts/nav2-clean-route-consumer.py`, invoked with `NAV2_CLEAN_ROUTE=room-1` in the same Intel-only, headless runtime image and recovered `41463f9` source copy used by the pipeline comparison. It is not a replacement action client and no manual ground-truth pose was published.

The consumer passed its action-server, clock, ground-truth odometry, and FAST-LIO readiness gate. It published `/initialpose` itself; `initialpose.yaml` captures that map-frame event. Its metrics record initial healthy localization at simulation time 28.341, a 108-point plan to room-1, an accepted goal, and cancellation after 0.182 wall seconds with zero motion. The runtime log attributes cancellation to `localization_cmd_gate` losing health. This is an actual failed route result, not a navigation success claim.

`publisher-owners.txt` proves `/mid360/points` and `/livox/lidar` each had exactly one publisher, `native_livox_converter`; `/livox/imu` had exactly one publisher, `kuruma_sim_adapter`, with FAST-LIO subscribed to each. The source input and timestamp behavior were unchanged. OGRE reports Intel Mesa ARL.

The parent house launch is a named-place locked asset. This run used the explicit authorized overlay recorded in `authorized-house-launch-overlay.diff`: two false-default argument declarations plus forwarding of those child launch arguments only. `locked-house-launch.sha256` records the resulting hash. No named-request lock was changed.
