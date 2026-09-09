# Findings

## Blocker

The corrected `ROS_DOMAIN_ID=211` execution reached the unchanged runner's lifecycle-node gate and invoked the existing route consumer, but it did not reach a route action. Gazebo's Ogre2 renderer failed to initialize EGL on `/dev/dri/card0`, reported missing OpenGL 3.3 support, and then exited with signal 139. Consequently, the consumer reported `RuntimeError: sensor/FAST-LIO readiness unavailable` and sent no goal. See `runtime-gates.txt`, `nav2-container.log`, `consumer.log`, and `metrics.json`.

## Not A Navigation Result

The prior Fast DDS port-range failure was absent at domain 211. The runner had observed `/global_localization`, `/planner_server`, and `/bt_navigator` before starting the consumer. The consumer's readiness gate did not obtain the required clock, truth odometry, and FAST-LIO evidence, so no target status may be interpreted as a Nav2 algorithm failure. See `runtime-gates.txt` and `runtime-summary.json`.
