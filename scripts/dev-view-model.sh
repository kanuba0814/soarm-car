#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/dev-env.sh"
exec docker compose run --rm ros2-dev ros2 launch soarm_description view_model.launch.py \
    rviz:="${RVIZ:-true}" foxglove:="${FOXGLOVE:-true}" gui:="${GUI:-true}" \
    pose:="${POSE:-new_calib_mid}"
