#!/usr/bin/env bash
set -euo pipefail

# Keyboard teleop + STM32 chassis serial adapter smoke.
# Runs inside a dedicated ros2-dev container; the chassis end of the serial
# link is a PTY, so no hardware is involved.

package_dir=ros2_ws/src/soarm_teleop
for file in \
    "$package_dir/soarm_teleop/chassis_protocol.py" \
    "$package_dir/soarm_teleop/keyboard_teleop.py" \
    "$package_dir/soarm_teleop/stm32_chassis_serial.py" \
    "$package_dir/soarm_teleop/launch/keyboard_teleop.launch.py" \
    "$package_dir/soarm_teleop/launch/stm32_safe_base.launch.py" \
    "$package_dir/test/test_chassis_protocol.py" \
    "$package_dir/test/test_keyboard_teleop.py" \
    "$package_dir/test/test_stm32_chassis_serial.py" \
    "$package_dir/test/teleop_smoke_harness.py"; do
    test -f "$file"
done
grep -q 'python3-serial' docker/Dockerfile

# The chassis adapter needs pyserial, which the image only has after the
# Dockerfile change; the rebuild is cached up to the apt layer.
docker compose build ros2-dev

ros_domain_id="${ROS_DOMAIN_ID:-152}"
container="$(docker compose run -d --rm -e ROS_DOMAIN_ID="$ros_domain_id" ros2-dev bash -lc 'sleep infinity')"
on_exit() {
    status=$?
    trap - EXIT
    if (( status != 0 )); then
        docker logs "$container" > /tmp/teleop-smoke.log 2>&1 || true
        tail -120 /tmp/teleop-smoke.log >&2 || true
        echo 'Teleop smoke failed; full log: /tmp/teleop-smoke.log' >&2
    fi
    docker stop "$container" >/dev/null 2>&1 || true
    exit "$status"
}
trap on_exit EXIT

source_ros='source /opt/ros/humble/setup.bash && source /tmp/soarm_ros2/install/setup.bash'
# First container start builds the whole workspace into /tmp/soarm_ros2.
for _ in {1..600}; do
    if docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash; then
        break
    fi
    if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null)" != 'true' ]]; then
        echo 'ros2-dev container exited during the workspace build' >&2
        break
    fi
    sleep 1
done
docker exec "$container" test -f /tmp/soarm_ros2/install/setup.bash

echo '== unit tests =='
docker exec "$container" bash -lc \
    "$source_ros && python3 -m pytest -q -p no:cacheprovider \
     /workspaces/soarm-car/ros2_ws/src/soarm_teleop/test/test_chassis_protocol.py \
     /workspaces/soarm-car/ros2_ws/src/soarm_teleop/test/test_keyboard_teleop.py \
     /workspaces/soarm-car/ros2_ws/src/soarm_teleop/test/test_stm32_chassis_serial.py"

echo '== keyboard topic wiring + chassis PTY loopback =='
docker exec "$container" bash -lc \
    "$source_ros && python3 /workspaces/soarm-car/ros2_ws/src/soarm_teleop/test/teleop_smoke_harness.py"

echo 'Teleop smoke passed.'
