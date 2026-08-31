# Kuruma holonomic Nav2 simulation

`ros2 launch soarm_navigation house_nav2.launch.py` starts Fortress with the Kuruma mecanum chassis, the selected map server, and Nav2. The planner is `SmacPlanner2D`; the installed DWB local controller permits `x`, `y`, and rotate-in-place commands. The velocity smoother retains the `0.4 m/s` planar speed limit and accepts lateral velocity. Both costmaps use the `+/-0.31 m` square footprint required to cover the measured 3030 frame perimeter.

Navigation sensor topics and TF frames remain provisional mounts pending physical installation and calibration. The chassis-only stage does not make grasp missions available.
