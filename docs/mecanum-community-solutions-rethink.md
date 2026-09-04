# Kuruma mecanum simulation: community solution review

Date: 2026-08-31

## Scope

This note reviews community and official approaches for a ROS 2 Humble + Gazebo Fortress four-mecanum simulation with these constraints:

- the default simulation must remain stable and real-time enough for Nav2;
- raw wheel feedback and physical simulator truth must remain independently observable;
- simulator truth must not masquerade as operational `/odom`;
- wheel/contact slip must not be hidden by a truth-derived correction;
- the explicit 64-roller model remains experimental while its RTF is below `0.8`.

No implementation change was made during this review.

## Locally verified runtime facts

Inside the current `ros2-dev` image:

- `mecanum_drive_controller` is installed at `/opt/ros/humble`;
- `libmecanum_drive_controller.so` is present;
- `controller_manager` is installed;
- neither `gz_ros2_control` nor `ign_ros2_control` is installed or discoverable through `ros2 pkg prefix`;
- the current apt metadata has no candidate for either bridge package.

Therefore the standard bridge is an image/dependency gap, not an already working project capability.

All previously running `soarm-car`, Gazebo, RViz, Nav2, simulation adapter, and Astra containers were stopped before this review.

## Official guidance

### Humble + Fortress has an official ros2_control bridge path

The maintained `gz_ros2_control` compatibility matrix lists ROS 2 Humble with Gazebo Fortress on its `humble` branch. Current Humble documentation names the binary package `ros-humble-gz-ros2-control` and says Fortress is the default source-build target.

The project went through an `ign_ros2_control` to `gz_ros2_control` naming transition. Historical issues refer to `ros-humble-ign-ros2-control`; current documentation and the current Humble branch use the `gz` name. The exact binary available to this image must therefore be validated during an isolated dependency spike rather than assumed.

Sources:

- [gz_ros2_control Humble branch and compatibility matrix](https://github.com/ros-controls/gz_ros2_control/tree/humble)
- [gz_ros2_control Humble documentation](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)
- [Humble package-name transition discussion](https://github.com/ros-controls/gz_ros2_control/issues/548)

### Humble has a standard mecanum controller

The Humble `mecanum_drive_controller`:

- accepts planar Twist references (`linear.x`, `linear.y`, `angular.z`);
- owns four wheel command interfaces and four wheel state interfaces;
- implements mecanum wheel odometry;
- publishes `nav_msgs/Odometry` and an optional odometry TF;
- includes command timeout and velocity/acceleration/jerk limits.

This is a better hardware-transfer seam than a simulation-only body-velocity plugin, but it still integrates wheel state. It does not eliminate physical slip and must not be treated as ground truth.

Source:

- [mecanum_drive_controller Humble documentation](https://control.ros.org/humble/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)

### Nav2 requires coherent odometry ownership

Nav2 documents these responsibilities:

- localization or SLAM owns `map -> odom`;
- the odometry system owns `odom -> base_link`;
- Nav2 also consumes a coherent `nav_msgs/Odometry` velocity signal;
- wheel encoders, IMU, and other continuous odometry sources are commonly fused with `robot_localization`;
- when an EKF owns `odom -> base_link`, a competing Gazebo TF bridge must be removed.

This supports one authoritative local state estimator, not a hybrid of wheel-local pose, truth-global correction, and truth velocity.

Sources:

- [Nav2 state estimation](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)
- [Nav2 transformation setup](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)
- [Nav2 robot_localization guide](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/)

## Community patterns

### linorobot2

`linorobot2` supports mecanum hardware and Gazebo simulation while keeping high-level Nav2 and SLAM topics aligned between hardware and simulation. Its main reusable lesson is a common controller/sensor contract rather than making simulator pose the hardware odometry interface.

- [linorobot2](https://github.com/linorobot/linorobot2)

### Karma Garden simulations

This ROS 2 mecanum example uses `gz_ros2_control/GazeboSimSystem`, `mecanum_drive_controller`, wheel/IMU odometry fusion, and Nav2. It targets Gazebo Garden, so it is evidence for architecture and ownership, not direct Fortress compatibility.

- [karma_fortress_simulations](https://github.com/SanjayS66/karma_fortress_simulations)

Other maintained examples commonly follow the same pattern but target Gazebo Classic or Jazzy/Harmonic. They should not be copied as Humble/Fortress compatibility proof.

## Options

### A. Explicit truth-preview plus separate localization profile

Use simulator physical pose and twist only in a launch profile clearly named `truth_preview`. Keep `/ground_truth/odom` and truth-specific frames; do not alias them to `/odom`.

Advantages:

- lowest implementation risk;
- preserves the current stable, real-time reduced plant;
- directly verifies Nav2 planning, costmaps, command generation, collision geometry, and sensors.

Limitations:

- does not validate encoder odometry or localization robustness;
- must never become the operational localization profile.

### B. gz_ros2_control + mecanum_drive_controller + estimator

Replace native `MecanumDrive` actuation with four ros2_control wheel interfaces through `gz_ros2_control`. Configure `mecanum_drive_controller` to publish wheel odometry without owning the final operational TF. Fuse wheel odometry and IMU in one estimator; localization owns `map -> odom`. Keep simulator model-state odometry on `/ground_truth/odom` for scoring only.

Suggested ownership:

```text
Nav2 / teleop
      |
      v
command mux + safety
      |
      v
mecanum_drive_controller
      |
      v
four wheel velocity command interfaces

wheel state interfaces -> /wheel/odom_raw --+
IMU ----------------------------------------+-> local estimator -> /odom + odom->base_footprint
LiDAR localization / SLAM ---------------------------------------> map->odom
Gazebo model pose -----------------------------------------------> /ground_truth/odom only
```

Advantages:

- standard ROS 2 control architecture;
- closest match to future hardware controller interfaces;
- controller timeouts, limits, state interfaces, and wheel ordering are explicit;
- avoids maintaining custom inverse kinematics and odometry code.

Limitations:

- bridge is absent from the current image and must first pass an isolated ABI/plugin spike;
- wheel odometry will still exhibit slip and cannot make Nav2 reliable without a coherent estimator/localization chain;
- replacing the actuator may alter the stable plant and therefore needs direction, stability, RTF, and Nav2 regression gates.

### C. Custom Fortress actuator/encoder system

Implement four wheel controllers or a custom Gazebo system that exposes wheel command/state interfaces and encoder effects such as quantization, latency, bias, and dropout.

Advantages:

- exact control over simulated actuator and encoder semantics;
- no dependency on the missing bridge binary.

Limitations:

- highest maintenance and validation cost;
- easy to accidentally leak simulator truth or implement a nonstandard mecanum convention;
- unjustified until the official bridge is proven unusable.

### D. Keep native MecanumDrive and add wheel/IMU/LiDAR fusion

Retain native `MecanumDrive` as the actuator, publish joint-state-derived wheel odometry only as `/wheel/odom_raw`, and fuse it with operational IMU/LiDAR sources.

Advantages:

- smallest plant change;
- preserves the known real-time reduced model.

Limitations:

- retains a simulation-only actuation seam unlike future hardware;
- the observed large wheel/physical disagreement still requires diagnosis;
- feeding simulator pose into the estimator would invalidate this option.

## Recommended decision

Use a two-step decision rather than immediately replacing the stable model:

1. **Run a bounded `gz_ros2_control` feasibility spike** in an isolated branch/container. Prove package installation or source build, plugin loading under Fortress 6.18, four command/state interfaces, controller timeout, controller odometry with TF disabled, and RTF `>= 0.8`.
2. If the spike passes, select **Option B** as the operational architecture and retain **Option A** only as an explicitly named planning/collision preview. If the spike fails because of ABI, controller-manager, or performance problems, keep the stable native actuator and select **Option D** temporarily; use **Option C** only when physical motor/encoder measurements justify custom actuator work.

This sequence follows the maintained community path without sacrificing the known stable checkpoint before the bridge has proven it works in this exact runtime.

## Acceptance boundary for the spike

The feasibility spike passes only if all are true:

- Fortress consumes the generated robot/controller configuration without plugin or ABI errors;
- exactly four wheel velocity command interfaces and four wheel state interfaces are claimed;
- `linear.x`, `linear.y`, and `angular.z` produce correct wheel signs and physical directions;
- a stale command actively sends zero and stops all wheel commands;
- controller wheel odometry is published under `/wheel/odom_raw` with odometry TF disabled;
- simulator truth is independently published under `/ground_truth/odom`;
- no `/odom`, `odom -> base_footprint`, or `map -> odom` owner exists in plant-only mode;
- idle drift is below `0.01 m` over ten seconds;
- mean RTF is at least `0.8`;
- the stable default is not replaced until these checks pass.
