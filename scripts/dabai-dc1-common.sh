#!/usr/bin/env bash
# Sourced by the DC1 recording tools.
ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DC1_IMAGE=${DC1_IMAGE:-soarm-car-astra-s-ros2-dev:latest}
DC1_CONTAINER=${DC1_CONTAINER:-soarm-dabai-dc1-camera}
DC1_ROS_DOMAIN_ID=${DC1_ROS_DOMAIN_ID:-44}
export DC1_IMAGE DC1_CONTAINER DC1_ROS_DOMAIN_ID
mkdir -p "$ROOT_DIR/recordings" "$ROOT_DIR/artifacts/perception-dev/dabai-dc1"
