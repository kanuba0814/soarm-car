#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
workspace=/workspaces/soarm-car/ros2_ws
local_build=/tmp/soarm_ros2
# Keep generated root-owned files out of the host bind mount.
if [ -d "$workspace/src/soarm_interfaces" ]; then
    mkdir -p "$local_build/build" "$local_build/install" "$local_build/log"
    # Simulation builds the official message ABI without requiring the hardware SDK.
    colcon --log-base "$local_build/log" build \
        --base-paths "$workspace/src" /workspaces/soarm-car/vendor/localization/src \
        --build-base "$local_build/build" \
        --install-base "$local_build/install" \
        --symlink-install \
        --cmake-args -DLIVOX_ROS_DRIVER2_MESSAGES_ONLY=ON
fi
if [ -f "$local_build/install/setup.bash" ]; then
    source "$local_build/install/setup.bash"
fi
exec "$@"
