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

## Sweet Home 3D house floor plan

Use `ros2 launch soarm_simulation simulation.launch.py world:=house_floorplan` to start the navigation house. Its ground-floor structure is generated directly from `house.sh3d` in <https://github.com/xtrinch/sweethome3d-house-plans.git> at commit `f0af4c6258ab5fcdabac1654e0796f9b18b44e94`, not traced from screenshot pixels. The source supplies 21 metric wall segments over an approximately 14.9 m x 13.3 m envelope, including the diagonal entrance wall, 2.4384 m wall height, and 0.12/0.23 m wall thicknesses.

All 10 source objects whose names contain `door` become permanently open frames: the door leaves and swing arcs are absent, the original clear widths remain available, and each opening has jamb and lintel geometry. Roof and ceiling are omitted. Three source-positioned, low-poly table obstacles are included for navigation testing: a 1.17 m x 0.49 m coffee table, a 1.20 m square table, and a 1.60 m x 0.80 m desk. Each uses a tabletop plus four physical legs; other loose furniture is omitted. Windows stay collision-solid. The robot spawns at the plan centre in the open kitchen/dining area. `maps/house_floorplan.yaml` is generated from the same wall, door, and furniture geometry at 0.05 m/cell, and marks the area outside the building shell unknown so an open exterior door does not invite planning outdoors. Source provenance and regeneration instructions are in `models/house_floorplan/SOURCE.md` and `models/house_furniture/SOURCE.md`.

The launch file serves the map matching the selected localization world. Keep `ground_truth_map_tf:=true` for scene preview, or disable it when AMCL/localization owns `map -> odom_combined`:

```bash
ros2 launch soarm_simulation simulation.launch.py \
  world:=house_floorplan gz_args:="-r" rviz:=true
```

## FAST-LIO mapping and global localisation

The container builds the pinned `vendor/localization` snapshot that matches `temple/localization_ws_portale.zip`; the archive and its nested Git objects are not copied into the image. `livox_ros_driver2` is built with `LIVOX_ROS_DRIVER2_MESSAGES_ONLY=ON`, which generates the official messages without linking the physical Livox SDK. Hardware builds retain the driver's default SDK-dependent path.

Run the mapping and localisation consumer smokes in order, then run the independent multi-direction odometry accuracy check:

```bash
./scripts/dev-fastlio-navigation-room-smoke.sh
./scripts/dev-fastlio-localization-smoke.sh
./scripts/dev-fastlio-odometry-accuracy-smoke.sh
```

The first starts Fortress and FAST-LIO mapping, drives a short safe arc, verifies `CustomMsg`/IMU/registered-cloud/odometry consumers, and saves `artifacts/fastlio_navigation_room.pcd`. The second restarts the same room with ground-truth map TF disabled, loads that PCD, seeds ICP near the known simulation origin, and verifies `/map_to_odom`, `/localization`, the two Autoware covariance topics, and `map -> camera_init`. The accuracy smoke runs forward, left, right, reverse, and second-left segments, aligns the initial FAST-LIO and Gazebo frames, checks path length and checkpoint position/yaw errors, and writes its runtime report to `artifacts/fastlio_odometry_accuracy.json`. Set `FASTLIO_GUI=true` to show Gazebo and RViz during that check. All smokes isolate ROS DDS and Ignition Transport (`ROS_DOMAIN_ID` and `IGN_PARTITION`) so another running Gazebo instance cannot mix simulation clocks into FAST-LIO.

Passing these tests proves the simulated ROS input and localisation software seams work together in this room. It does not validate a physical MID-360, unknown initial-pose recovery, long-run drift, relocalisation after kidnapping, real-time performance on the deployment computer, or vehicle-specific sensor calibration.
