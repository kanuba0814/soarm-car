# Corrected Isolated Combined Run

This run supersedes the earlier combined startup result only for startup diagnosis. The earlier launch hash `37c3e3b7ac44701420c0cb001fa04cfb03d7dc160183094326a239b9ffaa7c06` is invalid as a matched navigation baseline because it removed the original `nav2_params_file` and `nav_to_pose_bt_xml` arguments and hardcoded a different behavior tree. It was not used to update any named-request lock.

The corrected house launch is based on recovered SHA-256 `c88daf45e9627d79fc13812b27795f57dd9936ebb39b40ba475aba3bbb7a3273`. `authorized-house-launch-overlay.diff` proves its only changes are false-default declarations and forwarding of `use_native_livox_converter` and `throttle_odom_path`. Corrected overlay SHA-256 is `a520fa54f04feb26b2ceb8da8723046b89c2c8ab1d4cb1444cba63df17ec94d5`. Generated launch arguments retain `nav2_params_file` and the default `navigate_position_only_holonomic.xml` behavior tree.

The environment-only transport delta is exactly `run-corrected.sh`: `ROS_DOMAIN_ID=97`, `ROS_LOCALHOST_ONLY=1`, `IGN_PARTITION=soarm-combined-97`, `IGN_IP=127.0.0.1`, `--network none`, private IPC, `--shm-size 256m`, and only `/dev/dri/renderD128`. All ROS and Gazebo processes run in the container. `assets-audit.txt` records the local-resource audit. OGRE reports `GL_VENDOR = Intel` and `GL_RENDERER = Mesa Intel(R) Graphics (ARL)`.

The isolated environment started successfully: `ros-live-corrected.txt` proves `/clock`, `robot_state_publisher/get_parameters`, controller manager, raw PointCloud2, native Livox, and IMU endpoints. `gazebo-live-corrected.txt` proves Gazebo world clock, source topics, and services. The explicit service call returned the generated `robot_description`.

`health-trace.jsonl` contains 4,132 opt-in diagnostics heartbeats. All 8,220 recorded scan/odom attempts were accepted; no future, duplicate, or rollback rejection occurred. Maximum scan and odom receipt ages were 1.294705274 s and 1.287747659 s, respectively, and input freshness became unhealthy during real receipt gaps above the unchanged 0.5 s policy.

The unchanged original `nav2-clean-route-consumer.py` ran with `NAV2_CLEAN_ROUTE=room-1`. It made a 108-point, 5.799932386 m plan, then its accepted goal was canceled after 0.021 sim seconds / 0.073369621 wall seconds with 0.0 m truth progress. The command gate logged health loss during an actual scan/odom age of approximately 0.51 s; trace attempts remained accepted and source timestamps were behind node clock. This does not justify relaxing the strict future guard or any navigation policy.
