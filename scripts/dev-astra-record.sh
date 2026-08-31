#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ! $1 =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
    echo "Usage: $0 BAG_NAME" >&2
    exit 2
fi
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
mkdir -p bags
output="/workspaces/soarm-car/bags/$1"
if [[ -e "bags/$1" ]]; then
    echo "Bag already exists: bags/$1" >&2
    exit 1
fi
exec docker compose run --rm -e ROS_LOCALHOST_ONLY=1 ros2-dev \
    ros2 bag record -o "$output" \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /camera/depth/image_raw \
    /camera/depth/camera_info \
    /camera/depth/points \
    /tf \
    /tf_static
