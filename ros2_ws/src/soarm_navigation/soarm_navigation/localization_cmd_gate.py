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

    def __init__(self, health_timeout, command_timeout, expected_map_id=''):
        if (not math.isfinite(health_timeout) or health_timeout <= 0.0 or
                not math.isfinite(command_timeout) or command_timeout <= 0.0):
            raise ValueError('gate timeouts must be finite and positive')
        self.health_timeout = health_timeout
        self.command_timeout = command_timeout
        self.expected_map_id = expected_map_id
        self.health_state = None
        self.health_map_id = ''
        self.health_receipt = None
        self.command_receipt = None
        self.ever_healthy = False
        self.latched = False
        self.cancellation_complete = True
        self.last_health_valid = False

    def update_health(self, state, receipt, map_id=''):
        self.health_state = state
        self.health_receipt = receipt
        self.health_map_id = map_id

    def force_stop(self):
        self.latched = True
        self.cancellation_complete = False
        self.command_receipt = None

    def update_command(self, receipt):
        if not self.latched:
            self.command_receipt = receipt

    def health_valid(self, now):
        if (self.health_state != HEALTHY or self.health_receipt is None or
                (self.expected_map_id and self.health_map_id != self.expected_map_id)):
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
        self.declare_parameter('expected_map_id', '')
        self.policy = GatePolicy(
            float(self.get_parameter('health_timeout').value),
            float(self.get_parameter('command_timeout').value),
            str(self.get_parameter('expected_map_id').value),
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
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_service(Trigger, '~/status', self.status)
        self.cancel_clients = [
            self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal'),
            self.create_client(CancelGoal, '/navigate_through_poses/_action/cancel_goal'),
        ]
        self.cancel_futures = [None for _ in self.cancel_clients]
        self.cancel_confirmed = [False for _ in self.cancel_clients]
        self.nav_nonterminal = [False for _ in self.cancel_clients]
        self.nav_active_ids = [set() for _ in self.cancel_clients]
        self.terminal_since_stop = [set() for _ in self.cancel_clients]
        self.drain_required = [False for _ in self.cancel_clients]
        self.last_cancel_sweep = 0.0
        self.cancel_epoch = 0
        self.status_seen_after_cancel = [False for _ in self.cancel_clients]
        self.status_generation = [0 for _ in self.cancel_clients]
        self.cancel_status_generation = [0 for _ in self.cancel_clients]
        self.cancel_goal_ids = [set() for _ in self.cancel_clients]
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            lambda message: self.save_navigation_status(message, 0), 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_through_poses/_action/status',
            lambda message: self.save_navigation_status(message, 1), 10)
        self.create_timer(1.0 / publish_rate, self.publish_command)

    def save_command(self, message):
        now = time.monotonic()
        if not self.policy.latched:
            self.latest_command = message
        self.policy.update_command(now)

    def save_health(self, message):
        self.policy.update_health(message.state, time.monotonic(), message.map_id)

    @staticmethod
    def _goal_id(status):
        return bytes(status.goal_info.goal_id.uuid)

    def save_navigation_status(self, message, index):
        self.status_generation[index] += 1
        active = {self._goal_id(status) for status in message.status_list if status.status in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)}
        self.nav_active_ids[index] = active
        self.nav_nonterminal[index] = bool(active)
        if self.policy.latched:
            newly_active = active - self.cancel_goal_ids[index]
            self.cancel_goal_ids[index].update(active)
            self.terminal_since_stop[index].update(
                self._goal_id(status) for status in message.status_list if status.status in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED))
            if newly_active:
                self.policy.cancellation_complete = False
                self.cancel_confirmed[index] = False
                self.cancel_futures[index] = None
            self._update_drain_evidence(index)

    def _update_drain_evidence(self, index):
        # A known goal requires its actual terminal status, not omission from a
        # stale/empty array. For an unused action server, a fresh cancel-all
        # response with no goals is authoritative empty-server evidence. The
        # task owner separately retains pending sends and refuses arm until done.
        self.status_seen_after_cancel[index] = bool(
            self.cancel_confirmed[index] and
            self.cancel_goal_ids[index] <= self.terminal_since_stop[index])

    def begin_cancellation(self, now=None):
        self.policy.cancellation_complete = False
        self.cancel_epoch += 1
        self.cancel_futures = [None for _ in self.cancel_clients]
        self.cancel_confirmed = [False for _ in self.cancel_clients]
        self.drain_required = [required or active for required, active in zip(self.drain_required, self.nav_nonterminal)]
        self.status_seen_after_cancel = [False for _ in self.cancel_clients]
        self.cancel_status_generation = list(self.status_generation)
        self.cancel_goal_ids = [set(goals) for goals in self.nav_active_ids]
        self.terminal_since_stop = [set() for _ in self.cancel_clients]
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
                if result is not None and result.return_code == CancelGoal.Response.ERROR_NONE:
                    self.cancel_confirmed[index] = True
                    self.cancel_goal_ids[index].update(bytes(goal.goal_id.uuid) for goal in result.goals_canceling)
                    self._update_drain_evidence(index)
                    continue
                self.cancel_futures[index] = None
            if client.service_is_ready():
                self.cancel_futures[index] = client.call_async(request)
        # Known goals require terminal evidence; unused action servers may never
        # publish a status array, so their fresh empty cancel-all reply is enough.
        if (all(self.cancel_confirmed) and all(self.status_seen_after_cancel) and
                not any(self.nav_nonterminal)):
            self.policy.confirm_cancellation()

    def publish_command(self):
        now = time.monotonic()
        allowed, newly_latched = self.policy.evaluate(now)
        if newly_latched:
            self.get_logger().error(
                'Localization health lost: stopping and latching actuator output.')
            self.begin_cancellation(now)
        # A goal can be accepted after an earlier drain completed. Keep a
        # bounded sweep alive while latched so a missed/late status callback
        # cannot create an uncontrolled command window.
        if (self.policy.latched and self.policy.cancellation_complete and
                now - self.last_cancel_sweep >= 0.25):
            self.begin_cancellation(now)
        if self.policy.latched and not self.policy.cancellation_complete:
            self.advance_cancellation()
        self.output.publish(self.latest_command if allowed else Twist())

    def stop(self, request, response):
        del request
        self.policy.force_stop()
        # Publish the zero command in the service callback as well as on the
        # periodic final-output path; Nav2 cancellation may take longer.
        self.output.publish(Twist())
        self.begin_cancellation()
        response.success = True
        response.message = 'Gate latched; final velocity output is zero while Nav2 drains.'
        return response

    def status(self, request, response):
        del request
        response.success = not self.policy.latched and self.policy.health_valid(time.monotonic())
        response.message = ('armed=%s latched=%s cancellation_complete=%s nonterminal_actions=%s post_stop_terminal=%s' % (
            str(response.success), str(self.policy.latched), str(self.policy.cancellation_complete),
            str(any(self.nav_nonterminal)), str(all(self.status_seen_after_cancel))))
        return response

    def arm(self, request, response):
        del request
        if not any(self.nav_nonterminal) and self.policy.arm(time.monotonic()):
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
