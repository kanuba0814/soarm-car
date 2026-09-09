#!/usr/bin/env bash
set -euo pipefail
docker run -d --name soarm-native-combined-isolated \
  --network none \
  --ipc private \
  --shm-size 256m \
  --device /dev/dri/renderD128:/dev/dri/renderD128 \
  --group-add 105 \
  -v /tmp/soarm-nav2-native-runtime-41463f9:/workspaces/soarm-car \
  -w /workspaces/soarm-car \
  -e ROS_DOMAIN_ID=97 \
  -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION=soarm-combined-97 \
  -e IGN_IP=127.0.0.1 \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  soarm-car-ros2-dev:latest \
  ros2 launch soarm_navigation house_fastlio_nav2.launch.py \
    rviz:=false use_native_livox_converter:=true throttle_odom_path:=true
