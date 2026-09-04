#!/usr/bin/env python3

import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from soarm_interfaces.msg import LocalizationHealth
from std_srvs.srv import Trigger


HEALTHY = LocalizationHealth.HEALTHY


class GatePolicy:
    """Fail-closed command authorization independent of ROS time."""

    def __init__(self, health_timeout, command_timeout):
        if (not math.isfinite(health_timeout) or health_timeout <= 0.0 or
                not math.isfinite(command_timeout) or command_timeout <= 0.0):
            raise ValueError('gate timeouts must be finite and positive')
        self.health_timeout = health_timeout
        self.command_timeout = command_timeout
        self.health_state = None
        self.health_receipt = None
        self.command_receipt = None
        self.ever_healthy = False
        self.latched = False
        self.cancellation_complete = True
        self.last_health_valid = False

    def update_health(self, state, receipt):
        self.health_state = state
        self.health_receipt = receipt

    def update_command(self, receipt):
        if not self.latched:
            self.command_receipt = receipt

    def health_valid(self, now):
        if self.health_state != HEALTHY or self.health_receipt is None:
            return False
        age = now - self.health_receipt
        return 0.0 <= age < self.health_timeout

    def evaluate(self, now):
        health_valid = self.health_valid(now)
        newly_latched = False
        if health_valid and not self.ever_healthy:
            self.ever_healthy = True
        elif self.last_health_valid and not health_valid:
            self.latched = True
            self.cancellation_complete = False
            self.command_receipt = None
            newly_latched = True
        self.last_health_valid = health_valid

        command_valid = self.command_receipt is not None and (
            0.0 <= now - self.command_receipt < self.command_timeout)
        return health_valid and self.ever_healthy and not self.latched and command_valid, newly_latched

    def confirm_cancellation(self):
        self.cancellation_complete = True

    def arm(self, now):
        if not self.health_valid(now) or not self.cancellation_complete:
            return False
        self.latched = False
        self.command_receipt = None
        self.last_health_valid = True
        return True


class LocalizationCommandGate(Node):
    def __init__(self):
        super().__init__('localization_cmd_gate')
        self.declare_parameter('health_timeout', 0.5)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.policy = GatePolicy(
            float(self.get_parameter('health_timeout').value),
            float(self.get_parameter('command_timeout').value),
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')
        self.latest_command = Twist()
        self.output = self.create_publisher(Twist, '/safe_cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel', self.save_command, 1)
        self.create_subscription(
            LocalizationHealth, '/localization/health', self.save_health, 1)
        self.create_service(Trigger, '~/arm', self.arm)
        self.cancel_clients = [
            self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal'),
            self.create_client(CancelGoal, '/navigate_through_poses/_action/cancel_goal'),
        ]
        self.cancel_futures = [None for _ in self.cancel_clients]
        self.cancel_confirmed = [False for _ in self.cancel_clients]
        self.last_cancel_sweep = 0.0
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.save_navigation_status, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_through_poses/_action/status',
            self.save_navigation_status, 10)
        self.create_timer(1.0 / publish_rate, self.publish_command)

    def save_command(self, message):
        now = time.monotonic()
        if not self.policy.latched:
            self.latest_command = message
        self.policy.update_command(now)

    def save_health(self, message):
        self.policy.update_health(message.state, time.monotonic())

    def save_navigation_status(self, message):
        active = any(status.status in (
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
        ) for status in message.status_list)
        now = time.monotonic()
        if (active and self.policy.latched and self.policy.cancellation_complete and
                now - self.last_cancel_sweep >= 0.25):
            self.begin_cancellation(now)

    def begin_cancellation(self, now=None):
        self.policy.cancellation_complete = False
        self.cancel_futures = [None for _ in self.cancel_clients]
        self.cancel_confirmed = [False for _ in self.cancel_clients]
        self.last_cancel_sweep = time.monotonic() if now is None else now

    def advance_cancellation(self):
        request = CancelGoal.Request()
        for index, client in enumerate(self.cancel_clients):
            if self.cancel_confirmed[index]:
                continue
            future = self.cancel_futures[index]
            if future is not None:
                if not future.done():
                    continue
                try:
                    result = future.result()
                except Exception as error:  # ROS transport failure: retry fail-closed.
                    self.get_logger().warn(f'Navigation cancellation failed: {error}')
                    self.cancel_futures[index] = None
                    continue
                if (result is not None and
                        result.return_code != CancelGoal.Response.ERROR_REJECTED):
                    self.cancel_confirmed[index] = True
                    continue
                self.cancel_futures[index] = None
            if client.service_is_ready():
                self.cancel_futures[index] = client.call_async(request)
        if all(self.cancel_confirmed):
            self.policy.confirm_cancellation()

    def publish_command(self):
        allowed, newly_latched = self.policy.evaluate(time.monotonic())
        if newly_latched:
            self.get_logger().error(
                'Localization health lost: stopping and latching actuator output.')
            self.begin_cancellation()
        if self.policy.latched and not self.policy.cancellation_complete:
            self.advance_cancellation()
        self.output.publish(self.latest_command if allowed else Twist())

    def arm(self, request, response):
        del request
        if self.policy.arm(time.monotonic()):
            response.success = True
            response.message = 'Gate armed; waiting for a fresh navigation command.'
        else:
            response.success = False
            response.message = (
                'Cannot arm until localization is fresh and Nav2 cancellation is confirmed.')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationCommandGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
