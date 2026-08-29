#!/usr/bin/env python3
"""Verify that the simulated mission action accepts and completes cancellation."""
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from soarm_interfaces.action import GoToPlaceAndGrasp


def wait(node, future, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError(f'future did not complete within {timeout}s')
    return future.result()


def main():
    rclpy.init()
    node = Node('sim_cancel_smoke')
    client = ActionClient(node, GoToPlaceAndGrasp, '/go_to_place_and_grasp')
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('mission action server unavailable')
        goal = GoToPlaceAndGrasp.Goal()
        goal.request_id = 'cancel-1'
        goal.place_id = 'demo_table'
        goal.object_id = 'demo_cube'
        handle = wait(node, client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError('valid cancellation test goal was rejected')
        conflicting_goal = GoToPlaceAndGrasp.Goal()
        conflicting_goal.request_id = 'conflict-1'
        conflicting_goal.place_id = 'demo_table'
        conflicting_goal.object_id = 'demo_cube'
        conflicting_handle = wait(node, client.send_goal_async(conflicting_goal), 10.0)
        if conflicting_handle.accepted:
            raise RuntimeError('concurrent mission should be rejected while resources are busy')
        cancel_response = wait(node, handle.cancel_goal_async(), 10.0)
        if not cancel_response.goals_canceling:
            raise RuntimeError('server did not accept cancellation')
        wrapped_result = wait(node, handle.get_result_async(), 10.0)
        if wrapped_result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError(f'expected CANCELED status, got {wrapped_result.status}')
        if wrapped_result.result.success:
            raise RuntimeError('canceled mission incorrectly reported success')
        print('Mission cancellation passed.')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
