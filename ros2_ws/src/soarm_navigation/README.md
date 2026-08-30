# House Nav2 Simulation

Run the complete ground-truth Nav2 simulation from a built workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch soarm_navigation house_nav2.launch.py
```

The launch starts Fortress with `house_floorplan`, the simulator map server and ground-truth `map -> odom_combined`, plus Nav2 planner, controller, behavior server, costmaps, velocity smoother, and lifecycle manager. It does not start AMCL. `NavigateToPose` commands pass through `velocity_smoother` to the native `/cmd_vel` Ackermann bridge.

For the Gazebo GUI, use:

```bash
ros2 launch soarm_navigation house_nav2.launch.py gz_args:="-r"
```

The global costmap consumes the installed house OccupancyGrid. The rolling local costmap consumes only the current `/mid360/points` PointCloud2 with bounded range, height, and persistence; it does not consume FAST-LIO `Laser_map`.

Run `scripts/dev-nav2-house-smoke.sh` after building `ros2-dev` to exercise the installed stack with a real `NavigateToPose` action.
