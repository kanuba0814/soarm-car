# UAF-TM camera fusion

`uaf_tm_node` consumes Gazebo RGB-D (`/camera/depth_image` and `/camera/camera_info`) plus non-GT wheel odometry (`/wheel/odom_raw`). It publishes `/uaf_tm/points`, `/uaf_tm/map` and `/uaf_tm/odom`. The map is a confidence-weighted temporal occupancy projection; invalid depth is rejected and no ground-truth topic is subscribed. `camera_right` is an optional synchronized equivalent stereo stream for ablation.

Example: `ros2 run soarm_uaf_tm uaf_tm_node --ros-args -p depth_topic:=/camera/depth_image -p odom_topic:=/wheel/odom_raw`
