# Fortress simulation seam

`ros2 launch soarm_simulation simulation.launch.py` starts the headless Fortress scene and ROS compatibility adapter. Use `gz_args:="-r" rviz:=true` to open both the Gazebo physical view and the RViz algorithm view. RViz uses `odom_combined` as its fixed frame and displays the live robot model, wheel/steering/arm joints, TF tree, odometry, bounded `/odom_path`, MID-360 approximation, and optional RGB image. `simulation:=false` starts only the ROS seam for hardware-facing graph checks; it does **not** start hardware drivers.

* `/cmd_vel` is Ackermann/non-holonomic: `linear.y` is explicitly discarded. The adapter emits `/odom`, `/imu/data_raw`, `/PowerVoltage`, `odom_combined -> base_footprint`, and the URDF static chain to `base_link` / `gyro_link`. `/PowerVoltage` is a fixed 24.0 V compatibility value, not a battery/electrical simulation.
* `/arm_controller/follow_joint_trajectory` drives the five native Fortress arm joints and verifies convergence through the bridged `/joint_states`; no arm state is fabricated. The coarse arm links use position-controlled, gravity-disabled dynamics to emulate powered servos; this is a pose/API simulation, not evidence for torque, payload, or contact performance. `/gripper_controller/follow_joint_trajectory` preserves the real controller-facing API, but the coarse single-servo gripper is explicitly an interface-level kinematic approximation: contact and physical position convergence are not acceptance claims.
* `/go_to_place_and_grasp` accepts only `demo_table` / `demo_cube`; it moves, stops, commands the approximate gripper seam, physically lifts the shoulder joint, and supports rejection/cancel. There is no MoveIt or Nav2 and no Gazebo object-attach plugin, so object grasp success remains an approximation.
* Fortress attaches RGB-D, chassis IMU, GPU LiDAR, and a separate LiDAR-collocated IMU to `camera_link`, `gyro_link`, and `livox_frame`. `/mid360/points` preserves the native three-dimensional `PointCloudPacked` geometry. The adapter also publishes `/livox/lidar` as the official `livox_ros_driver2/msg/CustomMsg` ABI and `/livox/imu` for FAST-LIO integration.
* The Livox output is an **algorithm-compatibility simulation**, not a hardware-accurate MID-360 model. XYZ returns come from Fortress GPU ray casting, but the scan pattern is the configured raster, per-point `offset_time` is linearly distributed across 100 ms, `line` is `index % 4`, `tag` is `0x10`, and reflectivity is a simplified sensor value. It does not reproduce Livox's proprietary/non-repetitive firing pattern, calibrated timing, material response, packet loss, or device clock behaviour.

## Navigation/localisation room

Use `ros2 launch soarm_simulation simulation.launch.py world:=navigation_room` to start the 8 m x 6 m localisation room. It keeps the existing native `senior_akm` model (spawn pose `0 0 0`) and adds 1.8 m-high, 0.12 m-thick walls, a south-wall door aperture (`x=2..3`), an L wall, cylinder, and distinct small/large boxes. The spawn has at least 1 m clearance to every static collision.

`maps/navigation_room.yaml` is a standard `nav_msgs/OccupancyGrid` map-server YAML: resolution `0.05` m/cell and origin `[-4.0, -3.0, 0.0]`. Its 160 x 120 PGM covers `x=[-4,4]`, `y=[-3,3]`, matching the world coordinates (+X forward, +Y left). Black cells represent the same walls and static obstacles; the south door aperture remains free.

For example, when `nav2_map_server` is installed: `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$(ros2 pkg prefix soarm_simulation)/share/soarm_simulation/maps/navigation_room.yaml`.

## FAST-LIO mapping and global localisation

The container builds the pinned `vendor/localization` snapshot that matches `temple/localization_ws_portale.zip`; the archive and its nested Git objects are not copied into the image. `livox_ros_driver2` is built with `LIVOX_ROS_DRIVER2_MESSAGES_ONLY=ON`, which generates the official messages without linking the physical Livox SDK. Hardware builds retain the driver's default SDK-dependent path.

Run the two consumer smokes in order:

```bash
./scripts/dev-fastlio-navigation-room-smoke.sh
./scripts/dev-fastlio-localization-smoke.sh
```

The first starts Fortress and FAST-LIO mapping, drives a short safe arc, verifies `CustomMsg`/IMU/registered-cloud/odometry consumers, and saves `artifacts/fastlio_navigation_room.pcd`. The second restarts the same room with ground-truth map TF disabled, loads that PCD, seeds ICP near the known simulation origin, and verifies `/map_to_odom`, `/localization`, the two Autoware covariance topics, and `map -> camera_init`. Both smokes isolate ROS DDS and Ignition Transport (`ROS_DOMAIN_ID` and `IGN_PARTITION`) so another running Gazebo instance cannot mix simulation clocks into FAST-LIO.

Passing these tests proves the simulated ROS input and localisation software seams work together in this room. It does not validate a physical MID-360, unknown initial-pose recovery, long-run drift, relocalisation after kidnapping, real-time performance on the deployment computer, or vehicle-specific sensor calibration.
