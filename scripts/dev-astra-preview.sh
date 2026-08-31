#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
source scripts/dev-env.sh
exec docker compose run --rm -e ROS_LOCALHOST_ONLY=1 ros2-dev \
    rviz2 -d /workspaces/soarm-car/config/astra.rviz
