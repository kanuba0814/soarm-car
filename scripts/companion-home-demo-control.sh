#!/usr/bin/env bash
set -euo pipefail

# Sends bounded commands only to the named isolated companion-demo container.
container_name="${SOARM_DEMO_CONTAINER:-soarm-companion-home-demo}"
action="${1:-}"

case "$action" in
    forward) command='{linear: {x: 0.12, y: 0.0}, angular: {z: 0.0}}' ;;
    back)    command='{linear: {x: -0.12, y: 0.0}, angular: {z: 0.0}}' ;;
    left)    command='{linear: {x: 0.0, y: 0.12}, angular: {z: 0.0}}' ;;
    right)   command='{linear: {x: 0.0, y: -0.12}, angular: {z: 0.0}}' ;;
    turn-left)  command='{linear: {x: 0.0, y: 0.0}, angular: {z: 0.35}}' ;;
    turn-right) command='{linear: {x: 0.0, y: 0.0}, angular: {z: -0.35}}' ;;
    stop)
        docker exec "$container_name" bash -lc 'source /opt/ros/humble/setup.bash; source /tmp/companion-demo/install/setup.bash; ros2 topic pub -r 20 --times 4 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"' >/dev/null
        exit 0
        ;;
    pose)
        exec docker exec "$container_name" bash -lc 'source /opt/ros/humble/setup.bash; source /tmp/companion-demo/install/setup.bash; timeout 5 ros2 topic echo /gazebo/odom --once'
        ;;
    stop-demo)
        docker stop "$container_name"
        docker rm "$container_name"
        exit 0
        ;;
    *)
        printf '用法：%s {forward|back|left|right|turn-left|turn-right|stop|pose|stop-demo}\n' "$0" >&2
        exit 2
        ;;
esac

zero='{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}'
release() {
    docker exec "$container_name" bash -lc "source /opt/ros/humble/setup.bash; source /tmp/companion-demo/install/setup.bash; ros2 topic pub -r 20 --times 4 /cmd_vel geometry_msgs/msg/Twist '$zero'" >/dev/null 2>&1 || true
}
trap release EXIT INT TERM
docker exec "$container_name" bash -lc "source /opt/ros/humble/setup.bash; source /tmp/companion-demo/install/setup.bash; ros2 topic pub -r 20 --times 8 /cmd_vel geometry_msgs/msg/Twist '$command'" >/dev/null
