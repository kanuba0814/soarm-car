#!/usr/bin/env bash
set -euo pipefail
state="${1:-IDLE}"
source "$(dirname "$0")/dev-env.sh"
exec docker compose run --rm --no-deps ros2-dev ros2 topic pub --once /robot_state std_msgs/msg/String "{data: '$state'}"
