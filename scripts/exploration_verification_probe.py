#!/usr/bin/env python3
"""Bounded live-control verifier for prior-map-assisted exploration.

The policy is separate from ROS callbacks so its safety gates can be CPU tested.
This tool never sends Nav2 goals or starts/stops simulation.
"""
import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

ACTIVE = {1, 2, 3}  # ACCEPTED, EXECUTING, CANCELING
CANCELED, SUCCEEDED = 5, 4
MOVING_LINEAR, MOVING_YAW = .02, .03
FRESH_SECONDS, STOP_SECONDS, STOP_STAMPS = 1.0, .5, 3


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def yaw_from_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_delta(new, old):
    return math.atan2(math.sin(new - old), math.cos(new - old))


class VerificationPolicy:
    """Accept only a fresh, newly observed action with measured movement."""
    def __init__(self, operation, observe_only=False):
        self.operation, self.observe_only = operation, observe_only
        self.started_wall = self.started_clock = None
        self.last_clock = self.last_clock_wall = None
        self.last_health = self.last_health_wall = None
        self.last_odom = self.active_uuid = self.active_goal_stamp = None
        self.triggered = False
        self.trigger_wall = self.trigger_response = self.terminal = self.result = None
        self.pretrigger = None
        self.flags, self.stop_stamps = [], set()
        self.stop_start = None
        self.stop_verified = False

    def start(self, wall, clock=None):
        self.started_wall, self.started_clock = wall, clock

    def clock(self, source, wall):
        if self.last_clock is not None and source <= self.last_clock:
            return False
        self.last_clock, self.last_clock_wall = source, wall
        return True

    def health(self, state, source_stamp, wall):
        self.last_health, self.last_health_wall = (state, source_stamp), wall

    def fresh(self, wall):
        return (self.last_clock is not None and wall - self.last_clock_wall <= FRESH_SECONDS and
                self.last_health is not None and wall - self.last_health_wall <= FRESH_SECONDS and
                self.last_health[0] == 1 and self.last_health[1] > 0)

    def odom(self, source_stamp, x, y, yaw, wall):
        if not all(math.isfinite(value) for value in (source_stamp, x, y, yaw)):
            return None
        if self.last_odom is None or source_stamp <= self.last_odom[0]:
            if self.last_odom is None or source_stamp > self.last_odom[0]:
                self.last_odom = (source_stamp, x, y, yaw, wall)
            return None
        old_stamp, old_x, old_y, old_yaw, _old_wall = self.last_odom
        dt = source_stamp - old_stamp
        linear = math.hypot(x - old_x, y - old_y) / dt
        yaw_rate = abs(angle_delta(yaw, old_yaw)) / dt
        self.last_odom = (source_stamp, x, y, yaw, wall)
        return {'stamp': source_stamp, 'linear': linear, 'yaw_rate': yaw_rate, 'wall': wall,
                'moving': linear > MOVING_LINEAR or yaw_rate > MOVING_YAW}

    def status(self, uuid, status, goal_stamp, wall):
        if not uuid or goal_stamp <= 0 or (self.started_clock is not None and goal_stamp <= self.started_clock):
            return
        if status in ACTIVE:
            if self.active_uuid is None:
                self.active_uuid, self.active_goal_stamp = uuid, goal_stamp
            elif uuid != self.active_uuid:
                self.flags.append('late_or_concurrent_goal:' + uuid)
                if self.triggered and not self.stop_verified:
                    self.result = 'FAIL'
                return
        if uuid != self.active_uuid:
            return
        if status == SUCCEEDED and not self.triggered:
            self.result = 'MISSED_WINDOW'
        elif status == CANCELED:
            self.terminal = 'CANCELED'
        elif status == SUCCEEDED:
            self.terminal = 'SUCCEEDED'

    def should_trigger(self, motion, wall):
        return (not self.observe_only and not self.triggered and self.result is None and
                self.terminal is None and self.active_uuid is not None and motion is not None and
                motion['moving'] and wall - motion['wall'] <= FRESH_SECONDS and self.fresh(wall))

    def trigger(self, wall, motion=None):
        self.triggered, self.trigger_wall = True, wall
        self.pretrigger = {'uuid': self.active_uuid, 'goal_stamp': self.active_goal_stamp,
                           'wall': wall, 'source_clock': self.last_clock,
                           'measured_motion': dict(motion) if motion is not None else None}

    def rpc_response(self, success, message):
        self.trigger_response = {'success': bool(success), 'message': str(message)}
        if not success:
            self.result = 'FAIL'

    def stopped_sample(self, velocity_zero, velocity_wall, motion, wall):
        if not self.triggered or self.trigger_response is None or not self.trigger_response['success']:
            return False
        zero = (velocity_zero and velocity_wall is not None and velocity_wall >= self.trigger_wall and
                wall - velocity_wall <= FRESH_SECONDS and motion is not None and not motion['moving'] and
                motion['wall'] >= self.trigger_wall and 0 <= wall - motion['wall'] <= FRESH_SECONDS and
                self.fresh(wall))
        if not zero:
            self.stop_start, self.stop_stamps = None, set()
            return False
        if self.stop_start is None:
            self.stop_start = motion['wall']
        self.stop_stamps.add(motion['stamp'])
        self.stop_verified = (motion['wall'] - self.stop_start >= STOP_SECONDS and
                              len(self.stop_stamps) >= STOP_STAMPS)
        return self.stop_verified

    def finalize(self, wall, budget):
        if self.result:
            return self.result
        if self.triggered and not self.stop_verified and wall - self.trigger_wall > 3.0:
            self.flags.append('stop_deadline_exceeded_3_wall_seconds')
            self.result = 'FAIL'
            return self.result
        if wall - self.started_wall >= budget:
            return 'TIMEOUT'
        if self.terminal == 'SUCCEEDED':
            return 'MISSED_WINDOW'
        return None


class ProbeNode:
    """ROS adapter. Imports are local so CPU policy tests require no ROS install."""
    def __init__(self, args):
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock as ClockMessage
        from rclpy.clock import Clock, ClockType
        from rclpy.node import Node
        from soarm_interfaces.msg import LocalizationHealth
        from std_srvs.srv import Trigger
        self.rclpy, self.Trigger = rclpy, Trigger
        self.node = Node('exploration_verification_probe')
        self.policy, self.args = VerificationPolicy(args.operation, args.observe_only), args
        self.policy.start(time.monotonic())
        self.events = deque(maxlen=300)
        self.event_count = 0
        self.event_stream = (args.output_dir / 'events.jsonl').open('x', buffering=1)
        self.last_clock_logged_wall = -math.inf
        self.latest_motion = None
        self.latest_zero_velocity = False
        self.latest_velocity_wall = None
        self.service = self.node.create_client(Trigger, '/prior_map_assisted_exploration/' + args.operation)
        self.resume_service = self.node.create_client(Trigger, '/prior_map_assisted_exploration/resume')
        self.future = self.resume_future = None
        self.resume_response = None
        self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, 30)
        self.node.create_subscription(Odometry, '/odom', self.on_odom, 30)
        self.node.create_subscription(Twist, '/safe_cmd_vel', self.on_velocity, 30)
        self.node.create_subscription(LocalizationHealth, '/localization/health', self.on_health, 30)
        self.node.create_subscription(ClockMessage, '/clock', self.on_clock, 30)
        self.node.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def event(self, kind, **data):
        record = {'wall': time.monotonic(), 'kind': kind, **data}
        self.events.append(record)
        self.event_count += 1
        self.event_stream.write(json.dumps(record, sort_keys=True) + '\n')

    def on_clock(self, message):
        source = stamp_seconds(message.clock)
        if self.policy.clock(source, time.monotonic()):
            if self.policy.started_clock is None:
                self.policy.started_clock = source
            wall = time.monotonic()
            if wall - self.last_clock_logged_wall >= .05:
                self.event('clock', source_stamp=source)
                self.last_clock_logged_wall = wall

    def on_health(self, message):
        source = stamp_seconds(message.stamp)
        self.policy.health(message.state, source, time.monotonic())
        self.event('health', state=message.state, source_stamp=source)

    def on_odom(self, message):
        pose = message.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        stamp = stamp_seconds(message.header.stamp)
        if (yaw is None or message.header.frame_id != 'odom_combined' or
                not all(math.isfinite(value) for value in (stamp, pose.position.x, pose.position.y))):
            self.latest_motion = None
            self.event('odom_invalid')
            return
        previous_stamp = self.policy.last_odom[0] if self.policy.last_odom else None
        motion = self.policy.odom(stamp, pose.position.x, pose.position.y, yaw, time.monotonic())
        if motion is not None:
            self.latest_motion = motion
            self.event('odom', frame_id=message.header.frame_id, **motion)
        elif previous_stamp is not None and stamp < previous_stamp:
            self.latest_motion = None
            self.event('odom_rollback', source_stamp=stamp)

    def on_velocity(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        self.latest_zero_velocity = all(math.isfinite(value) and abs(value) <= 1e-6 for value in values)
        self.latest_velocity_wall = time.monotonic()
        self.event('safe_cmd_vel', zero=self.latest_zero_velocity, linear_x=message.linear.x,
                   linear_y=message.linear.y, angular_z=message.angular.z)

    def on_status(self, message):
        wall = time.monotonic()
        for item in message.status_list:
            uuid = bytes(item.goal_info.goal_id.uuid).hex()
            goal_stamp = stamp_seconds(item.goal_info.stamp)
            self.policy.status(uuid, item.status, goal_stamp, wall)
            self.event('status', uuid=uuid, status=item.status, goal_stamp=goal_stamp)

    def _complete(self, result):
        self.write(result)
        self.node.destroy_node()
        self.rclpy.shutdown()

    def tick(self):
        wall = time.monotonic()
        if self.policy.should_trigger(self.latest_motion, wall):
            if not self.service.wait_for_service(timeout_sec=0.0):
                self.policy.result = 'FAIL'
                self.policy.flags.append('selected_service_unavailable')
            else:
                self.policy.trigger(wall, self.latest_motion)
                self.future = self.service.call_async(self.Trigger.Request())
                self.event('trigger_sent', operation=self.args.operation, uuid=self.policy.active_uuid)
        if self.future is not None and self.future.done():
            response = self.future.result()
            self.policy.rpc_response(response.success, response.message)
            self.event('trigger_response', success=response.success, message=response.message)
            self.future = None
            if self.args.operation == 'inject_source_drop' and 'injection=True' not in response.message:
                self.policy.result = 'FAIL'
                self.policy.flags.append('source_drop_service_is_toggle_or_not_armed')
        stopped = self.policy.stopped_sample(self.latest_zero_velocity, self.latest_velocity_wall,
                                             self.latest_motion, wall)
        if self.policy.terminal == 'CANCELED' and stopped and self.policy.result is None:
            if self.args.resume_after_verified_stop and self.resume_future is None and self.resume_response is None:
                if self.policy.fresh(wall) and self.resume_service.wait_for_service(timeout_sec=0.0):
                    self.resume_future = self.resume_service.call_async(self.Trigger.Request())
                    self.event('resume_sent')
                else:
                    self.policy.result = 'FAIL'
                    self.policy.flags.append('resume_requires_fresh_health_and_available_service')
            elif not self.args.resume_after_verified_stop:
                self.policy.result = 'PASS'
        if self.resume_future is not None and self.resume_future.done():
            response = self.resume_future.result()
            self.resume_response = {'success': bool(response.success), 'message': str(response.message)}
            self.event('resume_response', **self.resume_response)
            self.policy.result = 'PASS' if response.success else 'FAIL'
        result = self.policy.finalize(wall, self.args.budget)
        if result:
            self._complete(result)

    def write(self, result):
        summary = {'result': result, 'operation': self.args.operation, 'observe_only': self.args.observe_only,
                   'active_uuid': self.policy.active_uuid, 'pretrigger_measurement': self.policy.pretrigger,
                   'trigger_response': self.policy.trigger_response, 'resume_response': self.resume_response,
                   'terminal': self.policy.terminal,
                   'flags': self.policy.flags, 'events': self.event_count,
                   'stop_bound_wall_seconds': 3.0,
                   'note': 'Verifies moving cancellation and stop, not resumed motion; CPU cases are not live evidence.'}
        self.event_stream.flush()
        self.event_stream.close()
        (self.args.output_dir / 'summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
        print(json.dumps(summary, sort_keys=True))


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operation', choices=('pause', 'inject_source_drop'), default='pause')
    parser.add_argument('--observe-only', action='store_true')
    parser.add_argument('--resume-after-verified-stop', action='store_true')
    parser.add_argument('--budget', type=float, default=120.0)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    if not 0 < args.budget <= 120:
        parser.error('--budget must be in (0, 120] wall seconds')
    if args.resume_after_verified_stop and (args.operation != 'pause' or args.observe_only):
        parser.error('--resume-after-verified-stop requires active pause mode')
    if args.output_dir.exists():
        parser.error('--output-dir must not already exist (refusing overwrite)')
    args.output_dir.mkdir(parents=True)
    return args


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    import rclpy
    rclpy.init()
    probe = ProbeNode(args)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(probe.node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            probe.node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
