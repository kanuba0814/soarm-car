#!/usr/bin/env bash
# CPU-only research diagnostic: no Gazebo, renderer, device, or host network.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
image="${SOARM_TEST_IMAGE:-soarm-car-nav2-ros2-dev:latest}"
name="${SOARM_TEST_CONTAINER:-aws-knownselfpose-node-$RANDOM-$RANDOM}"
docker container inspect "$name" >/dev/null 2>&1 && { echo "refusing existing container $name" >&2; exit 2; }
docker run --rm --name "$name" --network none \
    -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-182}" \
    -v "$root:/feature:ro" -w /feature --entrypoint bash "$image" -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 /feature/tests/test_aws_knownselfpose_node_diagnostic.py
'
