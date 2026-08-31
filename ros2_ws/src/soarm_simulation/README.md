# Kuruma Fortress simulation

`ros2 launch soarm_simulation simulation.launch.py` starts the native Fortress `kuruma` model and ROS compatibility adapter. Kuruma is a four-wheel holonomic mecanum chassis: `/cmd_vel` forwards `linear.x`, `linear.y`, and `angular.z` to Fortress `ignition::gazebo::systems::MecanumDrive`. The configured roller-envelope radius is `0.0763 m`, wheelbase `0.3472 m`, track `0.4456 m`, and the collision-relevant frame perimeter is `0.600 m` square.

Fortress 6 `MecanumDrive` accepts `/cmd_vel` but its bundled official example does not emit odometry. Kuruma therefore adds Fortress's native `OdometryPublisher` companion system, publishing physical model-state `/gazebo/odom` at 50 Hz; the adapter republishes it as `/odom`. Camera, chassis IMU, MID-360 point cloud, and LiDAR IMU topic/TF seams remain available through provisional mounts. They are compatibility geometry, not STEP-derived extrinsics or hardware sensor validation. The simulated Livox output remains an algorithm-compatibility approximation, not a physical MID-360 timing or optical model.

This is a chassis-first stage. The default URDF and SDF contain no arm geometry or active arm controller. `/arm_controller/follow_joint_trajectory`, `/gripper_controller/follow_joint_trajectory`, and `/go_to_place_and_grasp` remain discoverable only to fail closed: with `arm_enabled=false`, every goal is rejected and no grasp or mission success is fabricated. Arm mounting dimensions and position are the next mechanical study.

`world:=navigation_room` and `world:=house_floorplan` include the same native `kuruma` model. The prior WHEELTEC chassis state is retained at `backup/wheeltec-chassis`.

## Isolated ros2_control plant

`ros2 launch soarm_simulation kuruma_ros2_control.launch.py` starts the separate real-time algorithm plant. It attaches Humble `gz_ros2_control` to the native SDF model through the matching published URDF and runs `joint_state_broadcaster` plus `mecanum_drive_controller`. In this algorithm profile, all four wheel visuals and collision bodies are radius-`0.0763 m` spheres. Their driven axes are fixed at alternating `45°` diagonals in `base_link`; each contact has traction along its normalized diagonal and free slip across it. This deliberately simplified omnidirectional contact proxy is not a physical mecanum roller model. The controller accepts the hardware-compatible `/cmd_vel` seam, stops after a 0.5 s stale-command timeout, and publishes wheel-derived `/wheel/odom_raw` with odometry TF disabled. Physical model-state truth is independently available as `/ground_truth/odom`; this plant-only profile intentionally has no operational `/odom` publisher.

The default native Fortress model remains unchanged and is the rollback-safe preview. Validate the isolated plant with `./scripts/dev-kuruma-ros2-control-smoke.sh`.
