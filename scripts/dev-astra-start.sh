#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
source scripts/astra-usb-device.sh
export ASTRA_USB_DEVICE="$(astra_usb_device)"
exec docker compose -f compose.yaml -f compose.astra.yaml run --rm \
    -e ROS_LOCALHOST_ONLY=1 ros2-dev ros2 launch soarm_astra hardware.launch.py
