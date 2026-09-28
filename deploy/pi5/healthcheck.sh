#!/usr/bin/env bash
set -euo pipefail

# Confirm all installed hardware packages are discoverable without opening devices.
source /opt/ros/humble/setup.bash
source /opt/wheeltec_ws/install/setup.bash
for package_name in \
    turn_on_wheeltec_robot \
    astra_camera \
    livox_ros_driver2 \
    fast_lio_localization; do
    ros2 pkg prefix "${package_name}" >/dev/null
done
