"""Real ROS/Gazebo consumer test using a labelled local LLM protocol fixture."""
import json
import math
import os
import time
import urllib.error
import urllib.request

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from soarm_interfaces.msg import LocalizationHealth

BASE = os.environ.get('SOARM_NAV_QA_URL', 'http://127.0.0.1:18190')
rclpy.init()
node = rclpy.create_node('room_navigation_http_consumer_qa')
messages, receipts, action_states = {}, {}, {}
report = {'planner': 'local protocol fixture, NOT real LLM inference'}
csrf = ''

def record(key, message):
    messages[key], receipts[key] = message, time.monotonic()

def statuses(message):
    for status in message.status_list:
        action_states[bytes(status.goal_info.goal_id.uuid).hex()] = status.status

for key, topic, kind in [('safe', '/safe_cmd_vel', Twist), ('truth', '/ground_truth/odom', Odometry),
                          ('health', '/localization/health', LocalizationHealth)]:
    node.create_subscription(kind, topic, lambda message, key=key: record(key, message), 20)
node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', statuses, 20)

def request(path, payload=None):
    headers = {'Origin': BASE, 'X-CSRF-Token': csrf}
    data = None
    if payload is not None:
        data = json.dumps(payload).encode()
        headers['Content-Type'] = 'application/json'
    req = urllib.request.Request(BASE + '/api/' + path, data=data, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=3) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f'{path} HTTP {error.code}: {error.read(4096).decode()}') from error

def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)

def wait_for(predicate, seconds, label):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        if predicate():
            return
    raise TimeoutError(label)

def task_state(targets, seconds):
    last = {}
    next_http = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        if time.monotonic() < next_http:
            continue
        last = request('health')
        next_http = time.monotonic() + .2
        if last['state'] in targets:
            return last
        if last['state'] in ('SAFETY_BLOCKED', 'FAILURE_AWAITING_INPUT'):
            raise RuntimeError('Unexpected task failure: ' + json.dumps(last, ensure_ascii=False))
    raise TimeoutError('Task state deadline: ' + json.dumps(last, ensure_ascii=False))

def speed():
    value = messages['safe']
    return max(abs(value.linear.x), abs(value.linear.y), abs(value.angular.z))

def pose():
    value = messages['truth'].pose.pose.position
    return value.x, value.y

try:
    initial = request('health')
    csrf = initial['csrf_token']
    assert initial['runtime'] == 'ROS', initial
    report['bundle_id'] = initial['bundle_id']
    request('save_room', {'room_id': 'qa_room', 'aliases': ['测试房间'],
                         'polygon': [[.5, .5], [1.5, .5], [1.5, 1.5], [.5, 1.5]],
                         'goal_pose': [1.0, 1.0, math.pi / 4]})
    wait_for(lambda: all(key in messages for key in ('health', 'safe', 'truth')), 10, 'ROS telemetry unavailable')
    assert messages['health'].state == 1
    assert messages['health'].map_id == initial['bundle_id'], 'health digest mismatch'
    report['health_map_id'] = messages['health'].map_id
    request('arm', {})
    wait_for(lambda: request('health').get('ready'), 5, 'HTTP readiness did not become true after arm')
    start = pose()
    report['start_truth_xy'] = start
    request('propose', {'text': 'fixture:roundtrip', 'fixture': False})
    proposed = task_state({'PROPOSAL'}, 10)
    report['confirmed_roundtrip_plan'] = proposed['plan']
    request('confirm', {'proposal_id': proposed['proposal_id']})
    completed = task_state({'SUCCESS'}, 120)
    spin_for(.5)
    report['roundtrip_final'] = completed
    report['roundtrip_truth_error_m'] = math.dist(start, pose())
    report['roundtrip_nav_succeeded_count'] = sum(status == 4 for status in action_states.values())
    assert report['roundtrip_nav_succeeded_count'] >= 2, report
    assert report['roundtrip_truth_error_m'] <= .4, report
    assert speed() <= .001, report
    previous_goals = set(action_states)
    request('propose', {'text': 'fixture:cancel', 'fixture': False})
    proposed = task_state({'PROPOSAL'}, 10)
    request('confirm', {'proposal_id': proposed['proposal_id']})
    motion_started = time.monotonic()
    wait_for(lambda: receipts['safe'] > motion_started and speed() > .03, 15, 'no nonzero safe output before cancellation')
    cancel_started = time.monotonic()
    request('stop', {})
    wait_for(lambda: receipts['safe'] >= cancel_started and speed() <= .001, 5, 'safe output did not stop')
    report['stop_to_observed_zero_s'] = time.monotonic() - cancel_started
    stopped = task_state({'CANCELED'}, 15)
    wait_for(lambda: any(key not in previous_goals and value == 5 for key, value in action_states.items()), 5, 'no new canceled Nav2 terminal result')
    report['stop_terminal'] = stopped
    report['action_states'] = action_states
    max_after_stop = 0
    end = time.monotonic() + 1
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        max_after_stop = max(max_after_stop, speed())
    report['max_safe_output_after_stop'] = max_after_stop
    assert max_after_stop == 0
    assert report['stop_to_observed_zero_s'] <= .5, report
    request('arm', {})
    wait_for(lambda: request('health').get('arm_status') == 'ARMED', 5, 'explicit rearm refused after terminal stop: ' + json.dumps(request('health')))
    wait_for(lambda: request('health').get('ready'), 5, 'not ready after explicit rearm')
    spin_for(.5)
    assert speed() <= .001, 'arming alone resumed motion'
    report['explicit_rearm_no_motion'] = True
    report['passed'] = True
except Exception as error:
    report['passed'], report['error'] = False, repr(error)
    try:
        request('stop', {})
    except Exception:
        pass
    raise
finally:
    print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()
