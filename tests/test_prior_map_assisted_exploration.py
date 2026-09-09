import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from prior_map_assisted_exploration import ExplorationSupervisor, FreshnessGate, LocalizationBootstrap, PointCloudAdapter, State

MAP_ID = 'house_widened_reference.pcd'


class Core:
    def __init__(self):
        self.revision = 0
        self.observed = set()
        self.reports = []

    def apply_observation(self, observation):
        self.revision = observation['revision']
        self.observed.add(observation['id'])

    def candidates(self, _pose):
        return {
            'status': 'ok', 'revision': self.revision,
            'observed_area_cells': len(self.observed),
            'reference_prior_connected_area_cells': 100,
            'candidates': [{'cluster_id': 'frontier-1', 'goal': [1.0, 2.0]}],
        }

    def report_result(self, cluster_id, gained, succeeded):
        self.reports.append((cluster_id, gained, succeeded))


class Navigator:
    def __init__(self):
        self.busy = False
        self.terminal = False
        self.result = None
        self.cancelled = False
        self.sent = []

    def compute_path(self, goal):
        return {'accepted': True, 'status': 'SUCCEEDED', 'frame_id': 'map', 'poses': [goal]}

    def safe(self, _goal):
        return True

    def send(self, goal):
        self.busy = True
        self.terminal = False
        self.sent.append(goal)
        return 'goal-uuid-1'

    def poll(self):
        if not self.terminal:
            return None
        self.busy = False
        return self.result

    def cancel(self):
        self.cancelled = True


class AsyncNavigator(Navigator):
    def __init__(self):
        super().__init__()
        self.path_ready = False

    def compute_path(self, _goal):
        return None

    def poll_path(self):
        if not self.path_ready:
            return None
        return {'accepted': True, 'status': 'SUCCEEDED', 'frame_id': 'map', 'poses': [[1., 2.]]}


class PlannerCancelNavigator(AsyncNavigator):
    def compute_path(self, goal):
        self.busy = True
        self.terminal = False
        return super().compute_path(goal)

    def cancel(self):
        self.cancelled = True

    def poll_cancel(self):
        if self.path_ready:
            self.busy = False
            self.terminal = True


class SupervisorTests(unittest.TestCase):
    def record(self, adapter, identifier, revision, stamp, receipt, clock, tf=True):
        return adapter.record(identifier, revision, stamp, [0., 0., .2], [0., 0., 0.], [(1., 0., .2)], receipt, clock, tf)

    def test_known_origin_bootstrap_waits_for_clock_and_local_odometry_only(self):
        bootstrap = LocalizationBootstrap()
        self.assertFalse(bootstrap.due(0.))
        bootstrap.clock_sample(1.)
        self.assertFalse(bootstrap.due(.1))
        bootstrap.odometry_sample(1.)
        self.assertTrue(bootstrap.due(.2))
        self.assertEqual(bootstrap.sent, 1)
        self.assertFalse(bootstrap.due(.4))
        bootstrap.clock_sample(2.)
        bootstrap.odometry_sample(2.)
        self.assertTrue(bootstrap.due(.8))
        bootstrap.healthy(True)
        bootstrap.clock_sample(3.)
        bootstrap.odometry_sample(3.)
        self.assertFalse(bootstrap.due(1.5))
        self.assertEqual(bootstrap.sent, 2)

    def test_explicit_floor_band_and_high_classification(self):
        adapter = PointCloudAdapter(MAP_ID, ground_z=1., floor_tolerance=.1, navigation_z_min=1.2, navigation_z_max=1.8)
        record = adapter.record('heights', 1, 1., [0., 0., 1.4], [0., 0., 1.], [(1., 0., 1.05), (1., 0., 1.5), (1., 0., 2.2)], 1., 1., True)
        self.assertEqual([item['kind'] for item in record.observation['returns']], ['floor', 'obstacle', 'floor'])
        self.assertEqual(record.observation['sample_policy'], 'full_message_uniform_index')

    def test_startup_waits_for_data_then_faults_only_after_grace(self):
        core, nav = Core(), Navigator()
        supervisor = ExplorationSupervisor(core, nav, MAP_ID, startup_grace=2.)
        supervisor.start(0.)
        supervisor.tick(.05)
        self.assertEqual(supervisor.state, State.READY)
        adapter = PointCloudAdapter(MAP_ID)
        supervisor.health('HEALTHY', MAP_ID, 1.)
        supervisor.enqueue(self.record(adapter, 'one', 1, 1., 1., 1.))
        supervisor.tick(1.)
        supervisor.health('HEALTHY', MAP_ID, 1.5)
        supervisor.enqueue(self.record(adapter, 'two', 2, 2., 1.5, 2.))
        supervisor.tick(1.5)
        self.assertEqual(supervisor.state, State.NAVIGATING)
        fault = ExplorationSupervisor(Core(), Navigator(), MAP_ID, startup_grace=.1)
        fault.start(0.)
        fault.tick(.2)
        self.assertEqual(fault.state, State.SENSOR_FAULT)

    def test_watchdog_rejects_stale_replay_wrong_map_missing_tf_and_paused_clock(self):
        adapter = PointCloudAdapter(MAP_ID, max_validated_rays=2)
        gate = FreshnessGate(MAP_ID)
        gate.health('HEALTHY', MAP_ID, 1.)
        self.assertTrue(gate.source(self.record(adapter, 'a', 1, 1., 1., 1.)))
        self.assertFalse(gate.source(self.record(adapter, 'replay', 2, 1., 2., 2.)))
        wrong = self.record(adapter, 'wrong', 3, 3., 3., 3.)
        wrong.observation['map_identity'] = 'other.pcd'
        self.assertFalse(gate.source(wrong))
        self.assertFalse(gate.source(self.record(adapter, 'tf', 4, 4., 4., 4., False)))
        self.assertFalse(gate.source(self.record(adapter, 'clock', 5, 5., 5., 1.)))
        split = FreshnessGate('house_widened_reference.pcd', 'core-map')
        split.health('HEALTHY', 'house_widened_reference.pcd', 10.)
        core_adapter = PointCloudAdapter('core-map')
        self.assertTrue(split.source(self.record(core_adapter, 'identity', 1, 1., 1., 1.)))

    def test_nonblocking_path_action_fsm_waits_before_navigation(self):
        core, nav = Core(), AsyncNavigator()
        supervisor = ExplorationSupervisor(core, nav, MAP_ID)
        adapter = PointCloudAdapter(MAP_ID)
        supervisor.start(0.)
        supervisor.health('HEALTHY', MAP_ID, 1.)
        supervisor.enqueue(self.record(adapter, 'one', 1, 1., 1., 1.))
        supervisor.tick(1.)
        supervisor.health('HEALTHY', MAP_ID, 2.)
        supervisor.enqueue(self.record(adapter, 'two', 2, 2., 2., 2.))
        supervisor.tick(2.)
        self.assertEqual(supervisor.state, State.PLANNING)
        self.assertEqual(nav.sent, [])
        nav.path_ready = True
        supervisor.tick(2.1)
        self.assertEqual(supervisor.state, State.NAVIGATING)
        self.assertEqual(nav.sent, [[1.0, 2.0]])

    def test_sensor_loss_during_planning_waits_for_planner_terminal(self):
        core, nav = Core(), PlannerCancelNavigator()
        supervisor = ExplorationSupervisor(core, nav, MAP_ID)
        adapter = PointCloudAdapter(MAP_ID)
        supervisor.start(0.)
        supervisor.health('HEALTHY', MAP_ID, 1.)
        supervisor.enqueue(self.record(adapter, 'one', 1, 1., 1., 1.))
        supervisor.tick(1.)
        supervisor.health('HEALTHY', MAP_ID, 2.)
        supervisor.enqueue(self.record(adapter, 'two', 2, 2., 2., 2.))
        supervisor.tick(2.)
        self.assertEqual(supervisor.state, State.PLANNING)
        supervisor.tick(4.)
        self.assertEqual(supervisor.state, State.CANCELLING)
        self.assertTrue(nav.cancelled)
        nav.path_ready = True
        supervisor.tick(4.1)
        self.assertEqual(supervisor.state, State.SENSOR_FAULT)

    def test_latest_bounded_queue_and_path_then_navigation_then_coverage(self):
        core, nav = Core(), Navigator()
        supervisor = ExplorationSupervisor(core, nav, MAP_ID)
        adapter = PointCloudAdapter(MAP_ID)
        supervisor.start(1029.)
        supervisor.health('HEALTHY', MAP_ID, 1030.)
        for number in range(1030):
            supervisor.enqueue(self.record(adapter, str(number), number + 1, number + 1., number + 1., number + 1.))
        self.assertEqual(len(supervisor.records), 1024)
        supervisor.tick(1030.1)
        self.assertEqual(core.revision, 1030)
        supervisor.health('HEALTHY', MAP_ID, 1031.)
        supervisor.enqueue(self.record(adapter, 'fresh', 1031, 1031., 1031., 1031.))
        supervisor.tick(1031.1)
        self.assertEqual(supervisor.state, State.NAVIGATING)
        self.assertEqual(nav.sent, [[1.0, 2.0]])
        supervisor.enqueue(self.record(adapter, 'coverage', 1032, 1032., 1032., 1032.))
        nav.terminal, nav.result = True, 'SUCCEEDED'
        supervisor.tick(1032.)
        self.assertEqual(supervisor.goals[0]['coverage_delta'], 1)
        self.assertEqual(core.reports[0], ('frontier-1', 1, True))

    def test_named_lock_accepts_only_two_launch_digests_and_locked_yaml(self):
        import json
        import shutil
        import tempfile
        package = Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'soarm_named_place_request'
        sys.path.insert(0, str(package))
        from soarm_named_place_request.prepare import RequestError, prepare
        baseline = Path('/home/gxxl/soarm-car-nav2-clean')
        paths = [
            'ros2_ws/src/soarm_simulation/maps/house_floorplan.pgm',
            'ros2_ws/src/soarm_simulation/maps/house_floorplan.yaml',
            'artifacts/house_widened_reference.pcd',
            'ros2_ws/src/soarm_navigation/soarm_navigation/launch/house_fastlio_nav2.launch.py',
            'ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml',
            'ros2_ws/src/soarm_navigation/behavior_trees/navigate_position_only_holonomic.xml',
            'ros2_ws/src/soarm_navigation/behavior_trees/navigate_through_poses_holonomic.xml',
        ]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'root'
            for relative in paths:
                target = root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(baseline / relative, target)
            request = Path(temporary) / 'request.json'
            request.write_text(json.dumps({'place_id': 'room-1', 'frame_id': 'map', 'require_heading': False}))
            self.assertEqual(prepare(request, root)['status'], 'prepared_for_serialization')
            launch = root / paths[3]
            corrected = Path('/home/gxxl/soarm-car-sensor-pipeline') / paths[3]
            shutil.copy2(corrected, launch)
            self.assertEqual(prepare(request, root)['status'], 'prepared_for_serialization')
            launch.write_text('selector overlay only')
            with self.assertRaisesRegex(RequestError, 'locked asset changed'):
                prepare(request, root)
            launch.write_text((baseline / paths[3]).read_text())
            yaml_path = root / paths[1]
            yaml_path.write_text(yaml_path.read_text().replace('origin: [-8.0, -7.25, 0.0]', 'origin: [0.0, 0.0, 0.0]'))
            with self.assertRaisesRegex(RequestError, 'locked asset changed'):
                prepare(request, root)

    def test_named_catalog_preparation_is_position_only(self):
        import json
        import tempfile
        package = Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'soarm_named_place_request'
        sys.path.insert(0, str(package))
        from soarm_named_place_request.prepare import prepare
        with tempfile.TemporaryDirectory() as temporary:
            request = Path(temporary) / 'request.json'
            request.write_text(json.dumps({'place_id': 'room-1', 'frame_id': 'map', 'require_heading': False}))
            prepared = prepare(request, Path('/tmp/soarm-intel-nav-restore-20260909'))
        self.assertEqual(prepared['status'], 'prepared_for_serialization')
        self.assertFalse(prepared['require_heading'])
        self.assertEqual(prepared['map_goal']['frame_id'], 'map')

    def test_pause_resume_requires_confirmed_cancel_and_fresh_records(self):
        core, nav = Core(), Navigator()
        core.export_state = lambda: {'state_version': 1}
        core.import_state = lambda state: self.assertEqual(state, {'state_version': 1})
        supervisor = ExplorationSupervisor(core, nav, MAP_ID)
        supervisor.start(0.)
        nav.busy = True
        checkpoint = supervisor.pause(1.)
        self.assertEqual(supervisor.state, State.CANCELLING)
        with self.assertRaisesRegex(RuntimeError, 'confirmed'):
            supervisor.resume(checkpoint, 1.1)
        nav.terminal, nav.busy = True, False
        supervisor.tick(1.2)
        self.assertEqual(supervisor.state, State.CANCELLED)
        supervisor.resume(checkpoint, 1.3)
        self.assertEqual(supervisor.state, State.READY)
        self.assertEqual(supervisor.gate.consecutive, 0)

    def test_source_loss_cancels_and_waits_for_terminal_result(self):
        core, nav = Core(), Navigator()
        supervisor = ExplorationSupervisor(core, nav, MAP_ID)
        adapter = PointCloudAdapter(MAP_ID)
        supervisor.start(0.)
        supervisor.health('HEALTHY', MAP_ID, 1.)
        supervisor.enqueue(self.record(adapter, 'one', 1, 1., 1., 1.))
        supervisor.tick(1.)
        supervisor.health('HEALTHY', MAP_ID, 2.)
        supervisor.enqueue(self.record(adapter, 'two', 2, 2., 2., 2.))
        supervisor.tick(2.)
        self.assertEqual(supervisor.state, State.NAVIGATING)
        supervisor.tick(4.)
        self.assertEqual(supervisor.state, State.CANCELLING)
        self.assertTrue(nav.cancelled)
        nav.terminal, nav.result = True, 'CANCELED'
        supervisor.tick(4.1)
        self.assertEqual(supervisor.state, State.SENSOR_FAULT)


if __name__ == '__main__':
    unittest.main()
