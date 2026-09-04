# Kuruma holonomic Nav2 simulation

`ros2 launch soarm_navigation house_nav2.launch.py` starts Fortress with the Kuruma mecanum chassis, the selected map server, and Nav2. The planner is `SmacPlanner2D`; the installed DWB local controller permits `x`, `y`, and rotate-in-place commands. The velocity smoother retains the `0.4 m/s` planar speed limit and accepts lateral velocity. Both costmaps use the `+/-0.31 m` square footprint required to cover the measured 3030 frame perimeter.

For algorithm development with the isolated `gz_ros2_control` plant and FAST-LIO localization, run:

```bash
ros2 launch soarm_navigation house_fastlio_nav2.launch.py
```

This profile uses the Humble MPPI controller in `Omni` mode. `mecanum_drive_controller` publishes diagnostic `/wheel/odom_raw` without TF, Fortress publishes independent `/ground_truth/odom` for scoring only, and FAST-LIO's `transform_fusion` is the sole owner of operational `/odom` and `odom_combined -> base_footprint`. Ground truth is not an input to Nav2 or localization. Add `gz_args:=-r rviz:=true` for Gazebo and RViz GUIs. Run `./scripts/dev-fastlio-house-nav2-smoke.sh` for the four-goal localization/navigation acceptance test.

Navigation sensor topics and TF frames remain provisional mounts pending physical installation and calibration. The chassis-only stage does not make grasp missions available.
