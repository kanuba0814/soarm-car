#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/dev-env.sh"
exec docker compose run --rm ros2-dev ros2 launch soarm_fake simulation.launch.py \
    rviz:="${RVIZ:-true}" foxglove:="${FOXGLOVE:-true}"
