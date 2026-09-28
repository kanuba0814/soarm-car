#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /opt/wheeltec_ws/install/setup.bash
exec "$@"
