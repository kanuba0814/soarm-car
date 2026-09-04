import math
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from soarm_room_search.controller import SearchController, SimulationPathVerifier
from soarm_room_search.core import MapBundle, Room, State, Workflow, WorkflowError, scan_complete
BASE = Path(__file__).parents[1] / 'examples'
def bundle(): return MapBundle.load(BASE / 'demo.yaml', 'demo-v1')
def room(): return Room('study', ['书房'], [(.5,.5),(2.5,.5),(2.5,1.5),(.5,1.5)], (1.5,1.,0))
def observations(): return [{'x':1.5,'y':1.25,'height':.75,'width':1.,'quality':.9,'dock':(1.5,.75),'line_of_sight':True}]
class FakeHandle:
    def __init__(self): self.cancelled = False
    def cancel_goal_async(self): self.cancelled = True
class FakeNavigation:
    def __init__(self): self.accepted = None; self.completed = None; self.handle = FakeHandle()
    def send(self, pose, accepted, completed): self.accepted, self.completed = accepted, completed
    def accept(self): self.accepted(self.handle)
class Tests(unittest.TestCase):
    def ready(self):
        work = Workflow(); work.load(bundle(), [room()]); work.propose('去书房找书桌'); work.confirm_intent(); return work
    def test_floor_trinary_and_geometry(self):
        item = bundle(); self.assertEqual(item.world_to_cell(-.01, .1), (-1, 1)); self.assertTrue(item.free(1, 1)); self.assertFalse(item.free(0, 0))
        invalid = Room('', [''], [(1,1),(2,1),(1,1)], (1,1,float('nan'))); self.assertFalse(invalid.valid(item))
    def test_scan_telemetry_and_fail_closed_path(self):
        work = self.ready(); work.scan_goal(); work.at_scan(); work.start_scan()
        telemetry = [(1,1,i,i * math.pi / 4) for i in range(9)]; self.assertTrue(scan_complete(telemetry)); self.assertFalse(scan_complete(telemetry + [(1.2,1,9,0)]))
        work.finish_scan(telemetry); work.analyze(observations())
        with self.assertRaisesRegex(WorkflowError, 'path verifier'): work.preview(None, 1, 1, 1, 1)
        with self.assertRaisesRegex(WorkflowError, 'stale'): work.preview(SimulationPathVerifier(), 2, 1, 1, 1)
        work.preview(SimulationPathVerifier(), 1, 1, 1, 1); self.assertEqual(work.state, State.DOCK_PREVIEW)
    def test_goal_lifecycle_success_failure_and_cancel(self):
        work, nav = self.ready(), FakeNavigation(); controller = SearchController(work, nav, SimulationPathVerifier())
        controller.navigate_scan(room().scan_pose); self.assertEqual(work.state, State.NAVIGATING_TO_SCAN)
        with self.assertRaisesRegex(WorkflowError, 'previous'): controller.navigate_scan(room().scan_pose)
        nav.accept(); controller.cancel(); self.assertTrue(nav.handle.cancelled); self.assertEqual(work.state, State.CANCELED)
        work = self.ready(); nav = FakeNavigation(); controller = SearchController(work, nav, SimulationPathVerifier()); controller.navigate_scan(room().scan_pose); nav.accept(); nav.completed(False); self.assertEqual(work.state, State.FAILED)
        work = self.ready(); nav = FakeNavigation(); controller = SearchController(work, nav, SimulationPathVerifier()); controller.navigate_scan(room().scan_pose); nav.accept(); nav.completed(True); self.assertEqual(work.state, State.AT_SCAN_POSE)
if __name__ == '__main__': unittest.main()

from soarm_room_search.freshness import SensorFreshness
class FreshnessTests(unittest.TestCase):
    def test_missing_unsynced_stale_and_ready(self):
        gate = SensorFreshness(1., .05, 'camera')
        self.assertFalse(gate.ready(10.))
        gate.pointcloud(10., 'camera'); gate.camera_info(10.2, 'camera'); gate.transform(True)
        self.assertFalse(gate.ready(10.))
        gate.camera_info(10.02, 'camera'); gate.transform(False); self.assertFalse(gate.ready(10.))
        gate.transform(True); self.assertTrue(gate.ready(gate.cloud[2]))
        self.assertFalse(gate.ready(gate.cloud[2] + 2.))

from soarm_room_search.tabletop import decode_xyz, detect_tabletops
class TabletopTests(unittest.TestCase):
    def test_synthetic_table_and_noise(self):
        points = [(x / 20, y / 20, .75) for x in range(20, 41) for y in range(20, 37)]
        points += [(8., 8., .1), (float('nan'), 1., .7)]
        desks = detect_tabletops(points)
        self.assertEqual(len(desks), 1)
        self.assertEqual(len(desks[0]['docks']), 2)
        self.assertEqual(detect_tabletops([(1., 1., .2)]), [])
    def test_decode_xyz(self):
        import struct
        self.assertEqual(decode_xyz(struct.pack('<fff', 1., 2., 3.)), [(1., 2., 3.)])
