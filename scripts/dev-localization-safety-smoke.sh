#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

domain="${ROS_DOMAIN_ID:-173}"
docker compose run --rm --no-deps -e ROS_DOMAIN_ID="$domain" ros2-dev bash -lc '
source /tmp/soarm_ros2/install/setup.bash
python3 - <<"PY"
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from soarm_interfaces.msg import LocalizationHealth
from soarm_navigation.localization_cmd_gate import LocalizationCommandGate
from std_srvs.srv import Trigger

rclpy.init()
gate = LocalizationCommandGate()
harness = rclpy.create_node("localization_safety_harness")
callback_group = ReentrantCallbackGroup()


def execute(goal_handle, result_type):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result_type.Result()
        time.sleep(0.01)
    goal_handle.abort()
    return result_type.Result()


def cancel(_request):
    return CancelResponse.ACCEPT


servers = [
    ActionServer(
        harness, NavigateToPose, "/navigate_to_pose",
        execute_callback=lambda handle: execute(handle, NavigateToPose),
        cancel_callback=cancel, callback_group=callback_group),
    ActionServer(
        harness, NavigateThroughPoses, "/navigate_through_poses",
        execute_callback=lambda handle: execute(handle, NavigateThroughPoses),
        cancel_callback=cancel, callback_group=callback_group),
]
health_pub = harness.create_publisher(LocalizationHealth, "/localization/health", 10)
command_pub = harness.create_publisher(Twist, "/cmd_vel", 10)
safe_commands = []
harness.create_subscription(
    Twist, "/safe_cmd_vel",
    lambda message: safe_commands.append((time.monotonic(), message.linear.x)), 10)
navigate = ActionClient(harness, NavigateToPose, "/navigate_to_pose", callback_group=callback_group)
arm = harness.create_client(Trigger, "/localization_cmd_gate/arm", callback_group=callback_group)
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(gate)
executor.add_node(harness)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()


def publish_health(state, duration=0.25):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        message = LocalizationHealth()
        message.state = state
        health_pub.publish(message)
        time.sleep(0.03)


def publish_command(value):
    message = Twist()
    message.linear.x = value
    command_pub.publish(message)


try:
    assert navigate.wait_for_server(timeout_sec=3.0)
    assert arm.wait_for_service(timeout_sec=3.0)
    publish_health(LocalizationHealth.HEALTHY)
    publish_command(0.25)
    time.sleep(0.2)
    assert any(abs(value - 0.25) < 1e-9 for _, value in safe_commands), safe_commands[-10:]

    send_future = navigate.send_goal_async(NavigateToPose.Goal())
    deadline = time.monotonic() + 3.0
    while not send_future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    goal_handle = send_future.result()
    assert goal_handle is not None and goal_handle.accepted
    result_future = goal_handle.get_result_async()

    publish_health(LocalizationHealth.DEGRADED)
    deadline = time.monotonic() + 3.0
    while not result_future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert result_future.done(), "active Nav2 goal was not canceled"
    assert result_future.result().status == GoalStatus.STATUS_CANCELED
    time.sleep(0.1)
    assert safe_commands and abs(safe_commands[-1][1]) < 1e-9

    publish_health(LocalizationHealth.HEALTHY)
    arm_future = arm.call_async(Trigger.Request())
    deadline = time.monotonic() + 3.0
    while not arm_future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert arm_future.done() and arm_future.result().success, arm_future.result()
    time.sleep(0.1)
    assert abs(safe_commands[-1][1]) < 1e-9
    publish_command(0.33)
    publish_health(LocalizationHealth.HEALTHY, duration=0.1)
    time.sleep(0.1)
    assert any(abs(value - 0.33) < 1e-9 for _, value in safe_commands[-10:])
    assert result_future.result().status == GoalStatus.STATUS_CANCELED
    print("Localization safety integration smoke passed.")
finally:
    for server in servers:
        server.destroy()
    executor.shutdown()
    gate.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=2.0)
PY
'
