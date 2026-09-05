#!/usr/bin/env bash
set -euo pipefail
export PYTHONPATH="${PYTHONPATH:-}:$(dirname "$0")/../ros2_ws/src/soarm_room_search"
exec python3 -m soarm_room_search.http_gui "$@"
