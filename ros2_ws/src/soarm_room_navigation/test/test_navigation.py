import json
import math
import re
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).parents[1]))
from soarm_room_navigation.controller import RoomNavigationController
from soarm_room_navigation.core import MapBundle, NavigationError, Plan, PlanAction, Room, RoomStore, TaskState, parse_plan
from soarm_room_navigation.http_gui import App, server
from soarm_room_navigation.planner import OpenAICompatiblePlanner, strict_json_loads


class Handle:
    def __init__(self): self.cancelled = False
    def cancel_goal_async(self): self.cancelled = True


class Transport:
    def __init__(self): self.calls = []; self.pending_cancelled = False; self.gate_stopped = False
    def send(self, pose, require_yaw=True, accepted=None, completed=None):
        if callable(require_yaw): require_yaw, accepted, completed = True, require_yaw, accepted
        self.calls.append((pose, require_yaw, accepted, completed))
    def cancel(self, handle): handle.cancel_goal_async()
    def cancel_pending(self): self.pending_cancelled = True
    def stop_gate(self, callback): self.gate_stopped = True; return True
    def accept(self, index=0): handle = Handle(); self.calls[index][2](handle); return handle
    def complete(self, outcome, index=0): self.calls[index][3](outcome)


def make_bundle(unknown=False):
    root = Path(tempfile.mkdtemp()); pixels = bytearray([255] * (40 * 30))
    pixels[(30 - 1 - 10) * 40 + 10] = 0  # furniture inside a valid room
    if unknown: pixels[(30 - 1 - 5) * 40 + 5] = 127
    (root / 'map.pgm').write_bytes(b'P5\n40 30\n255\n' + bytes(pixels))
    (root / 'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n')
    (root / 'map.pcd').write_bytes(b'# test pcd\nDATA binary\n123')
    return MapBundle.load(root / 'map.yaml', root / 'map.pcd')


def room(bundle):
    value = Room('study', ('书房', '学习室'), ((.2, .2), (2.8, .2), (2.8, 2.8), (.2, 2.8)), (2.2, 1.8, 0.0))
    assert value.valid(bundle); return value


def plan(bundle, actions, name='p'):
    return Plan(name, tuple(actions), 'test', bundle.bundle_id, 1)


class ModelTests(unittest.TestCase):
    def test_bundle_digest_roundtrip_furniture_and_square_goal(self):
        bundle = make_bundle(); self.assertTrue(room(bundle).valid(bundle))
        for cell in ((0, 0), (39, 0), (0, 29), (39, 29)):
            self.assertEqual(bundle.world_to_cell(*bundle.cell_to_world(*cell)), cell)
        self.assertFalse(bundle.footprint_free(1.05, 1.05, 0.0))
        self.assertFalse(Room('outside', ('外',), ((.2,.2),(2.8,.2),(2.8,2.8)), (2.95, 2.5, 0)).valid(bundle))
        old = bundle.bundle_id; Path(bundle.pcd_path).write_bytes(b'changed'); self.assertFalse(bundle.intact())
        self.assertNotEqual(old, MapBundle.load(bundle.yaml_path, bundle.pcd_path).bundle_id)
        with self.assertRaises(ValueError): MapBundle.load(bundle.yaml_path, bundle.pcd_path, 'filename-only')

    def test_plan_schema_ambiguity_and_canonical_id(self):
        bundle, rooms = make_bundle(), [room(make_bundle())]
        rooms = [room(bundle)]
        for payload in ({'actions': [{'action':'navigate','room_id':'missing'}]},
                        {'actions': [{'action':'wait','seconds':61}]},
                        {'actions': [{'action':'wait','seconds':1,'room_id':'study'}]},
                        {'actions': [{'action':'navigate','room_id':'study','tool':'shell'}]},
                        {'clarification':'x','actions':[]}):
            with self.assertRaises(NavigationError): parse_plan(payload, rooms, bundle, 'random', 1)
        one = parse_plan({'summary':'ok','actions':[{'action':'navigate','room_id':'study'}]}, rooms, bundle, 'a', 1)
        two = parse_plan({'summary':'ok','actions':[{'action':'navigate','room_id':'study'}]}, rooms, bundle, 'b', 1)
        self.assertEqual(one.proposal_id, two.proposal_id)

    def test_sequence_wait_return_fallback_and_safety_no_fallback(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5,.5,0))
        controller.propose(plan(bundle, [PlanAction('navigate','study','study'), PlanAction('wait', seconds=1), PlanAction('return_to_start')]))
        controller.confirm('p'); transport.accept(); transport.complete('aborted'); transport.accept(1); transport.complete('succeeded',1)
        self.assertEqual(controller.state, TaskState.RUNNING_WAIT); controller.tick(controller.wait_deadline + .1); transport.accept(2); transport.complete('succeeded',2)
        self.assertEqual(controller.state, TaskState.SUCCESS)
        unsafe = RoomNavigationController(bundle, [item], Transport(), lambda: True, lambda: True, lambda: (.5,.5,0))
        unsafe.propose(plan(bundle,[PlanAction('navigate','study','study')], 'safe')); unsafe.confirm('safe'); unsafe.transport.accept(); unsafe.transport.complete('canceled')
        self.assertEqual(unsafe.state, TaskState.SAFETY_BLOCKED); self.assertEqual(len(unsafe.transport.calls), 1)

    def test_cancel_pending_fallback_terminal_and_continuous_wait(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5,.5,0))
        controller.propose(plan(bundle,[PlanAction('navigate','study','study')], 'fall')); controller.confirm('fall'); transport.accept(); transport.complete('aborted')
        controller.stop(); self.assertTrue(transport.calls[1][0] and transport.pending_cancelled)
        handle = transport.accept(1); self.assertTrue(handle.cancelled); transport.complete('canceled',1)
        self.assertEqual(controller.state, TaskState.CANCELED)
        zero = [True]
        waiter = RoomNavigationController(bundle,[item],Transport(),lambda:True,lambda:zero[0],lambda:(.5,.5,0))
        waiter.propose(plan(bundle,[PlanAction('wait',seconds=1)],'wait')); waiter.confirm('wait'); zero[0] = False; waiter.tick()
        self.assertEqual(waiter.state, TaskState.SAFETY_BLOCKED)

    def test_cancel_during_accepted_primary_motion_waits_for_terminal(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle,[item],transport,lambda:True,lambda:True,lambda:(.5,.5,0))
        controller.propose(plan(bundle,[PlanAction('navigate','study')], 'motion')); controller.confirm('motion')
        handle = transport.accept(); controller.stop()
        self.assertEqual(controller.state, TaskState.CANCELING); self.assertTrue(handle.cancelled)
        transport.complete('canceled'); self.assertEqual(controller.state, TaskState.CANCELED)

    def test_stop_invalidates_late_plan_and_journal_interrupts(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        journal = str(Path(tempfile.mkdtemp()) / 'task.json')
        controller = RoomNavigationController(bundle,[item],transport,lambda:True,lambda:True,lambda:(.5,.5,0),journal)
        generation, _, _ = controller.begin_planning(); controller.stop(); controller.complete_planning(generation, plan(bundle,[PlanAction('navigate','study')]))
        self.assertEqual(controller.state, TaskState.CANCELED)
        interrupted_journal = str(Path(tempfile.mkdtemp()) / 'task.json')
        RoomNavigationController(bundle,[item],transport,lambda:True,lambda:True,lambda:(.5,.5,0),interrupted_journal).begin_planning()
        controller = RoomNavigationController(bundle,[item],transport,lambda:True,lambda:True,lambda:(.5,.5,0),interrupted_journal)
        self.assertEqual(controller.state, TaskState.FAILURE_AWAITING_INPUT); self.assertIn('INTERRUPTED', controller.reason)

    def test_recovery_marker_survives_restarts_until_stop_is_requested(self):
        bundle, transport = make_bundle(), Transport()
        path = Path(tempfile.mkdtemp()) / 'task.json'
        path.write_text('[]')
        for _ in range(2):
            controller = RoomNavigationController(bundle, [room(bundle)], transport,
                                                  lambda: True, lambda: True, lambda: (.5, .5, 0), str(path))
            self.assertTrue(controller.recovery_required)
            with self.assertRaises(NavigationError):
                controller.begin_planning()
        controller.begin_recovery_stop()
        self.assertTrue(transport.gate_stopped)
        self.assertEqual(transport.calls, [])
        self.assertIn('INTERRUPTED', controller.reason)

    def test_planning_only_supersedes_planning_or_proposal(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5, .5, 0))
        one, _, _ = controller.begin_planning(); two, _, _ = controller.begin_planning()
        self.assertNotEqual(one, two)
        controller.complete_planning(two, plan(bundle, [PlanAction('navigate', 'study')], 'new'))
        controller.confirm('new')
        for state in (TaskState.CONFIRMATION, TaskState.RUNNING_NAV, TaskState.CANCELING):
            controller.state = state
            with self.assertRaises(NavigationError): controller.begin_planning()

    def test_each_primary_step_gets_one_fallback_and_fallback_failure_stops(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5, .5, 0))
        controller.propose(plan(bundle, [PlanAction('navigate', 'study', 'study'), PlanAction('navigate', 'study', 'study')], 'two'))
        controller.confirm('two')
        transport.accept(); transport.complete('aborted'); transport.accept(1); transport.complete('succeeded', 1)
        transport.accept(2); transport.complete('aborted', 2); transport.accept(3); transport.complete('succeeded', 3)
        self.assertEqual(controller.state, TaskState.SUCCESS)
        failed = RoomNavigationController(bundle, [item], Transport(), lambda: True, lambda: True, lambda: (.5, .5, 0))
        failed.propose(plan(bundle, [PlanAction('navigate', 'study', 'study')], 'failed')); failed.confirm('failed')
        failed.transport.accept(); failed.transport.complete('aborted'); failed.transport.accept(1); failed.transport.complete('aborted', 1)
        self.assertEqual(failed.state, TaskState.FAILURE_AWAITING_INPUT)

    def test_cancel_timeout_and_late_callbacks_remain_unresolved(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5, .5, 0))
        controller.propose(plan(bundle, [PlanAction('navigate', 'study')], 'late')); controller.confirm('late'); controller.stop()
        controller.cancel_timeout(); self.assertEqual(controller.state, TaskState.CANCELING)
        late = transport.accept(); self.assertTrue(late.cancelled); self.assertEqual(controller.state, TaskState.CANCELING)
        transport.complete('canceled'); self.assertEqual(controller.state, TaskState.CANCELED)

    def test_deadlines_cancel_then_drain_before_fallback(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5, .5, 0), acceptance_timeout=.01, execution_timeout=.01)
        controller.propose(plan(bundle, [PlanAction('navigate', 'study', 'study')], 'deadline')); controller.confirm('deadline')
        controller.tick(controller.acceptance_deadline + .1); self.assertEqual(controller.state, TaskState.CANCELING); self.assertEqual(len(transport.calls), 1)
        controller = RoomNavigationController(bundle, [item], Transport(), lambda: True, lambda: True, lambda: (.5, .5, 0), execution_timeout=.01)
        controller.propose(plan(bundle, [PlanAction('navigate', 'study', 'study')], 'execution')); controller.confirm('execution'); handle = controller.transport.accept()
        controller.tick(controller.execution_deadline + .1); self.assertTrue(handle.cancelled); self.assertEqual(len(controller.transport.calls), 1)
        controller.transport.complete('canceled'); self.assertEqual(len(controller.transport.calls), 2)

    def test_nav_success_waits_for_zero_before_starting_wait(self):
        bundle, transport, zero = make_bundle(), Transport(), [True]
        controller = RoomNavigationController(bundle, [room(bundle)], transport,
                                              lambda: True, lambda: zero[0], lambda: (.5, .5, 0))
        controller.propose(plan(bundle, [PlanAction('navigate', 'study'), PlanAction('wait', seconds=1)], 'settle'))
        controller.confirm('settle')
        transport.accept()
        zero[0] = False
        transport.complete('succeeded')
        self.assertEqual(controller.state.value, 'SETTLING')
        with self.assertRaises(NavigationError):
            controller.begin_planning()
        zero[0] = True
        controller.tick()
        self.assertEqual(controller.state, TaskState.RUNNING_WAIT)
        self.assertFalse(transport.gate_stopped)

    def test_operator_stop_or_safety_loss_preempts_timeout_fallback(self):
        for intervention in ('stop', 'safety_loss'):
            with self.subTest(intervention=intervention):
                bundle, transport, healthy = make_bundle(), Transport(), [True]
                controller = RoomNavigationController(
                    bundle, [room(bundle)], transport, lambda: healthy[0],
                    lambda: True, lambda: (.5, .5, 0), execution_timeout=.01)
                controller.propose(plan(bundle, [PlanAction('navigate', 'study', 'study')], intervention))
                controller.confirm(intervention)
                transport.accept()
                controller.tick(controller.execution_deadline + .1)
                self.assertEqual(controller.state, TaskState.CANCELING)
                if intervention == 'stop':
                    controller.stop()
                else:
                    healthy[0] = False
                    controller.tick()
                    healthy[0] = True
                self.assertTrue(transport.gate_stopped)
                transport.complete('canceled')
                self.assertEqual(len(transport.calls), 1)
                self.assertEqual(controller.state, TaskState.CANCELED if intervention == 'stop' else TaskState.SAFETY_BLOCKED)

    def test_corrupt_journal_fails_closed_and_identical_replan_is_fresh(self):
        bundle, item, transport = make_bundle(), None, Transport(); item = room(bundle)
        path = Path(tempfile.mkdtemp()) / 'task.json'; path.write_text('{broken')
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5, .5, 0), str(path))
        self.assertIn('INTERRUPTED', controller.reason)
        controller = RoomNavigationController(bundle, [item], transport, lambda: True, lambda: True, lambda: (.5, .5, 0))
        controller.propose(plan(bundle, [PlanAction('navigate', 'study')], 'same')); controller.confirm('same'); transport.accept(); transport.complete('succeeded')
        controller.propose(plan(bundle, [PlanAction('navigate', 'study')], 'same'))
        self.assertEqual(controller.state, TaskState.PROPOSAL)


class AutomaticGoalTests(unittest.TestCase):
    def test_any_heading_clearance_includes_intersecting_cell_corners(self):
        bundle = make_bundle()
        for value in (1.4, 1.4000000000000001, 1.400000000000001):
            self.assertFalse(bundle.footprint_free_any_yaw(value, value))

    def test_blocked_center_search_uses_rotated_map_axes(self):
        original = make_bundle()
        yaml_path = Path(original.yaml_path)
        yaml_path.write_text(yaml_path.read_text().replace('[0.0, 0.0, 0.0]', '[4.0, 0.0, 1.5707963267948966]'))
        bundle = MapBundle.load(yaml_path, original.pcd_path)
        polygon = tuple((4.0 - y, x) for x, y in ((.4, .4), (1.7, .4), (1.7, 1.7), (.4, 1.7)))
        goal = bundle.safe_polygon_goal(polygon)
        self.assertTrue(bundle.footprint_free_any_yaw(*goal[:2]))
        self.assertNotEqual(goal[:2], (2.95, 1.05))

    def test_automatic_goal_with_fixed_heading_keeps_requested_yaw(self):
        bundle = make_bundle()
        polygon = ((1.4, .8), (2.6, .8), (2.6, 2.4), (1.4, 2.4))
        self.assertAlmostEqual(bundle.safe_polygon_goal(polygon, True, 1.2)[2], 1.2)

    def test_centroid_fallback_and_unconstrained_clearance(self):
        bundle = make_bundle()
        symmetric = ((1.4, .8), (2.6, .8), (2.6, 2.4), (1.4, 2.4))
        self.assertAlmostEqual(bundle.safe_polygon_goal(symmetric)[0], 2.0); self.assertAlmostEqual(bundle.safe_polygon_goal(symmetric)[1], 1.6)
        concave = ((.2, .2), (2.8, .2), (2.8, .8), (.8, .8), (.8, 2.8), (.2, 2.8))
        goal = bundle.safe_polygon_goal(concave)
        self.assertTrue(goal[0] < .8 or goal[1] < .8)
        self.assertTrue(bundle.footprint_free_any_yaw(*goal[:2]))
        blocked = ((.4, .4), (1.7, .4), (1.7, 1.7), (.4, 1.7))
        self.assertNotEqual(bundle.safe_polygon_goal(blocked)[:2], (1.05, 1.05))
        self.assertFalse(bundle.footprint_free_any_yaw(.05, .05))
        unknown = make_bundle(unknown=True)
        self.assertFalse(unknown.free(.55, .55)); self.assertFalse(unknown.free(-.01, .5))

    def test_no_safe_goal_and_legacy_roundtrip(self):
        bundle = make_bundle()
        with self.assertRaisesRegex(NavigationError, 'no safe'):
            bundle.safe_polygon_goal(((.9, .9), (1.2, .9), (1.2, 1.2), (.9, 1.2)))
        legacy = Path(tempfile.mkdtemp()) / 'legacy.json'
        RoomStore.save(legacy, bundle, [room(bundle)], 1)
        payload = json.loads(legacy.read_text()); payload['rooms'][0].pop('goal_mode'); payload['rooms'][0].pop('require_yaw'); legacy.write_text(json.dumps(payload))
        loaded, _ = RoomStore.load(legacy, bundle)
        self.assertEqual((loaded[0].goal_mode, loaded[0].require_yaw, loaded[0].goal_pose), ('manual', True, room(bundle).goal_pose))

    def test_controller_selects_position_only_for_auto_and_return(self):
        bundle, transport = make_bundle(), Transport()
        automatic = Room('study', ('书房',), ((.2, .2), (2.8, .2), (2.8, 2.8), (.2, 2.8)), (2.2, 1.8, 0), 1, 'auto', False)
        controller = RoomNavigationController(bundle, [automatic], transport, lambda: True, lambda: True, lambda: (.5, .5, 0))
        controller.propose(plan(bundle, [PlanAction('navigate', 'study'), PlanAction('return_to_start')]))
        controller.confirm('p'); self.assertFalse(transport.calls[0][1]); transport.accept(); transport.complete('succeeded')
        self.assertFalse(transport.calls[1][1])


class Nav2AssetTests(unittest.TestCase):
    def test_path_alignment_activation_fits_prediction_horizon(self):
        root = Path(__file__).parents[2]
        config = (root / 'soarm_navigation/config/house_nav2_mppi.yaml').read_text()
        alignment = config.split('      PathAlignCritic:', 1)[1].split('      PathFollowCritic:', 1)[0]
        def number(text, key):
            return float(re.search(r'\b' + key + r':\s*([0-9.]+)', text).group(1))
        resolution = number((root / 'soarm_simulation/maps/house_floorplan.yaml').read_text(), 'resolution')
        nominal_range = number(config, 'time_steps') * number(config, 'model_dt') * number(config, 'vx_max')
        activation_range = number(alignment, 'offset_from_furthest') * resolution * math.sqrt(2)
        self.assertLess(activation_range, nominal_range)

    def test_position_only_assets_and_adapter_selection(self):
        root = Path(__file__).parents[2] / 'soarm_navigation'
        config = (root / 'config' / 'house_nav2_mppi.yaml').read_text()
        strict = (root / 'behavior_trees' / 'navigate_holonomic.xml').read_text()
        through = (root / 'behavior_trees' / 'navigate_through_poses_holonomic.xml').read_text()
        position_only = root / 'behavior_trees' / 'navigate_position_only_holonomic.xml'
        self.assertIn('goal_checker_plugins: [goal_checker, position_only_goal_checker]', config)
        self.assertIn('yaw_goal_tolerance: 0.20', config); self.assertIn('yaw_goal_tolerance: 3.141592653589793', config)
        self.assertIn('goal_checker_id="goal_checker"', strict); self.assertIn('goal_checker_id="goal_checker"', through)
        self.assertIn('goal_checker_id="position_only_goal_checker"', position_only.read_text())
        from soarm_room_navigation import ros_node
        class Goal:
            def __init__(self): self.pose = None; self.behavior_tree = ''
        Navigate = type('Navigate', (), {'Goal': Goal})
        class Header:
            frame_id = ''; stamp = None
        class Position:
            x = y = 0
        class Orientation:
            z = 0; w = 1
        class Pose:
            position = Position(); orientation = Orientation()
        class Stamped:
            def __init__(self): self.header = Header(); self.pose = Pose()
        class Clock:
            def now(self): return self
            def to_msg(self): return 'stamp'
        class Future:
            def add_done_callback(self, callback): pass
        class Client:
            def __init__(self): self.goals = []
            def send_goal_async(self, goal): self.goals.append(goal); return Future()
        class Node:
            def __init__(self): self.client = Client(); self.position_only_behavior_tree = str(position_only)
            def get_clock(self): return Clock()
        with patch.object(ros_node, 'NavigateToPose', Navigate, create=True), patch.object(ros_node, 'PoseStamped', Stamped, create=True):
            node = Node()
            ros_node.RoomNavigationNode.send(node, (1, 2, 0), False, lambda *_: None, lambda *_: None)
            ros_node.RoomNavigationNode.send(node, (1, 2, 0), True, lambda *_: None, lambda *_: None)
        self.assertEqual(node.client.goals[0].behavior_tree, str(position_only)); self.assertEqual(node.client.goals[1].behavior_tree, '')

    def test_send_coerces_non_native_goal_coordinates_at_ros_boundary(self):
        from types import SimpleNamespace
        from soarm_room_navigation import ros_node
        class Numeric:
            def __init__(self, value): self.value = value
            def __float__(self): return self.value
        class Point:
            def __init__(self): self._x = self._y = 0.0
            @property
            def x(self): return self._x
            @x.setter
            def x(self, value):
                if type(value) is not float: raise AssertionError('ROS Point requires float')
                self._x = value
            @property
            def y(self): return self._y
            @y.setter
            def y(self, value):
                if type(value) is not float: raise AssertionError('ROS Point requires float')
                self._y = value
        class Goal:
            def __init__(self): self.pose = None; self.behavior_tree = ''
        class Stamped:
            def __init__(self):
                self.header = SimpleNamespace(frame_id='', stamp=None)
                self.pose = SimpleNamespace(position=Point(), orientation=SimpleNamespace(z=0.0, w=1.0))
        class Future:
            def add_done_callback(self, callback): pass
        class Node:
            def __init__(self):
                self.goals = []
                self.client = SimpleNamespace(send_goal_async=lambda goal: self.goals.append(goal) or Future())
                self.position_only_behavior_tree = ''
            def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp'))
        node = Node()
        with patch.object(ros_node, 'NavigateToPose', type('Navigate', (), {'Goal': Goal}), create=True), patch.object(ros_node, 'PoseStamped', Stamped, create=True):
            ros_node.RoomNavigationNode.send(node, (Numeric(1.25), Numeric(2.5), 0.0), True, lambda *_: None, lambda *_: None)
        self.assertEqual((node.goals[0].pose.pose.position.x, node.goals[0].pose.pose.position.y), (1.25, 2.5))

    def test_readiness_snapshot_reports_negative_latest_tf_without_clamping(self):
        from types import SimpleNamespace
        from soarm_room_navigation import ros_node
        now = time.monotonic()
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=100)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=2.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        class Node:
            def __init__(self):
                self.controller = SimpleNamespace(
                    bundle=SimpleNamespace(intact=lambda: True, bundle_id='bundle'), rooms=[object()])
                self.client = SimpleNamespace(server_is_ready=lambda: True)
                self.health_state = ros_node.LocalizationHealth.HEALTHY
                self.health_map_id, self.health_receipt = 'bundle', now
                self.gate_armed, self.gate_receipt = True, now
                self.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
            def get_parameter(self, name): return SimpleNamespace(value=.5)
            def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
        with patch.object(ros_node, 'LocalizationHealth', SimpleNamespace(HEALTHY=1), create=True):
            node = Node()
            with patch.object(ros_node, 'rclpy', SimpleNamespace(time=SimpleNamespace(Time=lambda: None))):
                rejected = ros_node.RoomNavigationNode._readiness_snapshot(node)
                transform.header.stamp.nanosec = 0
                accepted = ros_node.RoomNavigationNode._readiness_snapshot(node)
        self.assertFalse(rejected['ready']); self.assertLess(rejected['tf_age'], 0)
        self.assertTrue(accepted['ready']); self.assertEqual(accepted['tf_age'], 0.0)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.bundle, self.transport = make_bundle(), Transport()
        self.controller = RoomNavigationController(self.bundle, [], self.transport, lambda: True, lambda: True, lambda: (.5,.5,0))
        self.app = App(self.controller, self.bundle, str(Path(tempfile.mkdtemp()) / 'rooms.json'), OpenAICompatiblePlanner('', '', ''), fixture_allowed=True, runtime='DEMO')
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), server(self.app, 0)); self.port = self.http.server_address[1]
        self.http.server_close(); self.http = ThreadingHTTPServer(('127.0.0.1', self.port), server(self.app, self.port)); self.thread = threading.Thread(target=self.http.serve_forever, daemon=True); self.thread.start()
        self.token = json.loads(urlopen('http://127.0.0.1:%d/api/health' % self.port).read())['csrf_token']

    def tearDown(self): self.http.shutdown(); self.thread.join(); self.http.server_close(); self.app.close()
    def post(self, action, value, good=True):
        headers={'Content-Type':'application/json'}
        if good: headers.update({'Origin':'http://127.0.0.1:%d' % self.port, 'X-CSRF-Token':self.token})
        request=Request('http://127.0.0.1:%d/api/%s' % (self.port,action),data=json.dumps(value).encode(),headers=headers,method='POST')
        try:
            with urlopen(request) as response: return response.status,json.loads(response.read())
        except HTTPError as error: return error.code,json.loads(error.read())

    def test_busy_planner_has_no_queue_and_stop_invalidates_result(self):
        entered, release = threading.Event(), threading.Event()
        class DelayedPlanner:
            def plan(self, text, rooms):
                entered.set()
                release.wait(2)
                return {'summary': 'late', 'actions': [{'action': 'navigate', 'room_id': 'study'}]}
        self.controller.replace_rooms([room(self.bundle)])
        self.app.planner = DelayedPlanner()
        self.app.planner.status = 'TEST_PROTOCOL'
        try:
            self.assertEqual(self.post('propose', {'text': 'first'})[0], 200)
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.post('propose', {'text': 'second'})[1]['error'], 'PLANNER_BUSY')
            self.assertEqual(self.post('stop', {})[1]['state'], 'CANCELED')
            release.set()
            self.app.planning_future.result(timeout=1)
            self.assertEqual(self.controller.state, TaskState.CANCELED)
        finally:
            release.set()

    def test_new_identical_http_plan_can_be_confirmed_again(self):
        class Planner:
            def plan(self, text, rooms):
                return {'summary': 'same', 'actions': [{'action': 'navigate', 'room_id': 'study'}]}
        self.controller.replace_rooms([room(self.bundle)])
        identifiers = []
        for index in range(2):
            generation, revision, rooms = self.controller.begin_planning()
            self.app._plan_async(generation, revision, rooms, Planner(), '去书房')
            identifier = self.app.status()['proposal_id']
            identifiers.append(identifier)
            self.assertEqual(self.post('confirm', {'proposal_id': identifier})[0], 200)
            self.assertEqual(self.post('confirm', {'proposal_id': identifier})[0], 409)
            self.transport.accept(index)
            self.transport.complete('succeeded', index)
        self.assertNotEqual(*identifiers)

    def test_create_automatic_room_without_goal_fields(self):
        payload = {'room_id': 'auto', 'aliases': ['自动房间'],
                   'polygon': [[1.4, .8], [2.6, .8], [2.6, 2.4], [1.4, 2.4]]}
        code, result = self.post('save_room', payload)
        self.assertEqual(code, 200)
        saved = result['rooms'][0]
        self.assertEqual(saved['goal_mode'], 'auto')
        self.assertFalse(saved['require_yaw'])
        self.assertAlmostEqual(saved['goal_pose'][0], 2.0)
        self.assertAlmostEqual(saved['goal_pose'][1], 1.6)
        payload.update({'mode': 'update', 'expected_room_revision': saved['revision'],
                        'goal_mode': 'auto', 'require_yaw': True, 'goal_yaw': 1.2})
        code, result = self.post('save_room', payload)
        self.assertEqual(code, 200)
        self.assertTrue(result['rooms'][0]['require_yaw'])
        self.assertEqual(result['rooms'][0]['goal_pose'][2], 1.2)

    def test_room_create_update_conflicts_and_atomic_failure(self):
        automatic = {'room_id': 'auto', 'aliases': ['自动'], 'polygon': [[.2, .2], [2.8, .2], [2.8, 2.8], [.2, 2.8]]}
        status, result = self.post('save_room', automatic)
        self.assertEqual(status, 200); saved = next(room for room in result['rooms'] if room['id'] == 'auto')
        self.assertEqual((saved['goal_mode'], saved['require_yaw']), ('auto', False))
        def payload(room_id, alias, goal=(2.2, 1.8, 0)):
            return {'room_id': room_id, 'aliases': [alias],
                    'polygon': [[.2, .2], [2.8, .2], [2.8, 2.8]], 'goal_pose': list(goal)}
        self.assertEqual(self.post('save_room', payload('study', '书房'))[0], 200)
        self.assertEqual(self.post('save_room', payload('bedroom', '卧室'))[0], 200)
        loaded, _ = RoomStore.load(self.app.rooms_path, self.bundle)
        self.assertEqual([item.id for item in loaded], ['auto', 'study', 'bedroom'])
        before = Path(self.app.rooms_path).read_bytes()
        status, result = self.post('save_room', payload('study', '替换'))
        self.assertEqual((status, result['error']), (409, 'ROOM_EXISTS'))
        self.assertEqual(Path(self.app.rooms_path).read_bytes(), before)
        study = next(item for item in self.app.status()['rooms'] if item['id'] == 'study')
        update = payload('study', '书房新名')
        update.update({'mode': 'update', 'expected_room_revision': study['revision']})
        self.assertEqual(self.post('save_room', update)[0], 200)
        self.assertEqual(next(item for item in self.app.status()['rooms'] if item['id'] == 'study')['aliases'], ('书房新名',))
        self.assertEqual(next(item for item in self.app.status()['rooms'] if item['id'] == 'bedroom')['aliases'], ('卧室',))
        self.assertEqual(self.post('save_room', update)[1]['error'], 'STALE_ROOM')
        unknown = payload('missing', '未知')
        unknown.update({'mode': 'update', 'expected_room_revision': 1})
        self.assertEqual(self.post('save_room', unknown)[1]['error'], 'ROOM_NOT_FOUND')
        stable_rooms, stable_disk = list(self.controller.rooms), Path(self.app.rooms_path).read_bytes()
        failure = payload('kitchen', '厨房')
        with patch('soarm_room_navigation.http_gui.RoomStore.save', side_effect=NavigationError('PERSISTENCE_FAILED', 'test')):
            self.assertEqual(self.post('save_room', failure)[1]['error'], 'PERSISTENCE_FAILED')
        self.assertEqual(self.controller.rooms, stable_rooms)
        self.assertEqual(Path(self.app.rooms_path).read_bytes(), stable_disk)

    def test_http_same_origin_async_fixture_and_unavailable(self):
        self.assertEqual(self.post('save_room',{'room_id':'study','aliases':['书房'],'polygon':[[.2,.2],[2.8,.2],[2.8,2.8]],'goal_pose':[2.2,1.8,0]},False)[0],403)
        self.assertEqual(self.post('save_room',{'room_id':'study','aliases':['书房'],'polygon':[[.2,.2],[2.8,.2],[2.8,2.8]],'goal_pose':[2.2,1.8,0]})[0],200)
        self.assertIn(self.post('propose',{'text':'去书房','fixture':True})[1]['state'], ('PLANNING', 'PROPOSAL'))
        deadline=time.monotonic()+1
        while self.app.status()['state']=='PLANNING' and time.monotonic()<deadline: time.sleep(.01)
        self.assertEqual(self.app.status()['state'], 'PROPOSAL')
        update = {'mode': 'update', 'room_id': 'study', 'aliases': ['书房'],
                  'polygon': [[.2,.2],[2.8,.2],[2.8,2.8]], 'goal_pose': [2.2,1.8,0],
                  'expected_room_revision': self.app.status()['rooms'][0]['revision']}
        self.assertEqual(self.post('save_room', update)[1]['error'], 'GOAL_ACTIVE')
        self.assertIn(self.post('propose',{'text':'去书房'})[1]['state'], ('PLANNING', 'FAILURE_AWAITING_INPUT'))
        deadline=time.monotonic()+1
        while self.app.status()['state']=='PLANNING' and time.monotonic()<deadline: time.sleep(.01)
        self.assertEqual(self.app.status()['state'], 'FAILURE_AWAITING_INPUT')


class PlannerProtocolTests(unittest.TestCase):
    def test_openai_mock_protocol_and_strict_json(self):
        captured = {}
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                captured['authorization']=self.headers.get('Authorization'); captured['payload']=json.loads(self.rfile.read(int(self.headers['Content-Length']))); reply={'choices':[{'message':{'content':'{"summary":"ok","actions":[{"action":"navigate","room_id":"study"}]}'}}]}
                raw=json.dumps(reply).encode(); self.send_response(200); self.end_headers(); self.wfile.write(raw)
            def log_message(self,*args): pass
        http=ThreadingHTTPServer(('127.0.0.1',0),Handler); thread=threading.Thread(target=http.serve_forever,daemon=True); thread.start()
        try:
            planner=OpenAICompatiblePlanner('http://127.0.0.1:%d/v1' % http.server_address[1],'operator-model','operator-secret')
            self.assertEqual(planner.plan('去书房',[room(make_bundle())])['actions'][0]['room_id'],'study'); self.assertEqual(captured['authorization'],'Bearer operator-secret'); self.assertEqual(captured['payload']['response_format'], {'type':'json_object'})
            for raw in ('{"x":1,"x":2}', '{"x":NaN}', '['*17+'0'+']'*17):
                with self.assertRaises(ValueError): strict_json_loads(raw)
        finally: http.shutdown(); thread.join(); http.server_close()


if __name__ == '__main__': unittest.main()
