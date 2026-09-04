#!/usr/bin/env bash
set -euo pipefail
PYTHONPATH="${PYTHONPATH:-}:$(dirname "$0")/../ros2_ws/src/soarm_room_search" python3 -m soarm_room_search.http_gui "$@"
