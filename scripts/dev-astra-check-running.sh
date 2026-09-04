#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
exec docker compose run --rm -e ROS_LOCALHOST_ONLY=1 ros2-dev bash -lc '
    nodes=$(ros2 node list)
    [[ $nodes == *"/camera/camera"* ]]
    python3 /workspaces/soarm-car/scripts/astra-contract-check.py --timeout 25
'
