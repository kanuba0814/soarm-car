import math, sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]))
from soarm_room_search.core import *
BASE=Path(__file__).parents[1]/'examples'
def room(): return Room('study',['书房','学习室'],[(.5,.5),(2.5,.5),(2.5,1.5),(.5,1.5)],(1.5,1.,0))
def bundle(): return MapBundle.load(BASE/'demo.yaml','demo-v1')
def observations(): return [{'x':1.5,'y':1.25,'height':.75,'width':1.,'quality':.9,'dock':(1.5,.75),'line_of_sight':True}]
class TestCore(unittest.TestCase):
 def test_map_hash_coordinate(self):
  b=bundle(); self.assertEqual(len(b.image_sha256),64); self.assertEqual(b.world_to_cell(.15,.15),(1,1)); self.assertTrue(b.free(1,1)); self.assertFalse(b.free(0,0))
 def test_rooms(self):
  self.assertTrue(room().valid(bundle())); self.assertFalse(Room('x',['x'],[(0,0),(2,2),(0,2),(2,0)],(1,1,0)).valid(bundle()))
  w=Workflow(); self.assertRaises(WorkflowError,w.load,bundle(),[room(),Room('x',['书房'],[(.5,.5),(2,.5),(2,1.5)],(1,1,0))])
 def test_intent(self):
  self.assertEqual(parse_intent('去 书房 找书桌',[room()])['room_id'],'study')
  for s in ('去 客厅 找书桌','去书房找书桌然后转圈','找书桌'): self.assertRaises(WorkflowError,parse_intent,s,[room()])
 def test_state_scan_candidate_and_cancel(self):
  w=Workflow();w.load(bundle(),[room()]); self.assertRaises(WorkflowError,w.confirm_intent);w.propose('去书房找书桌');w.confirm_intent();w.scan_goal();w.at_scan();w.start_scan()
  y=[i*2*math.pi/16 for i in range(17)];w.finish_scan(y);w.analyze(observations()); self.assertRaises(WorkflowError,w.preview,0); c=w.preview(w.scan_revision);self.assertEqual(c[0]['score'],8.5);w.confirm_dock();w.dock_goal();w.arrived();self.assertEqual(w.state,State.ARRIVED);w.cancel();self.assertEqual(w.state,State.CANCELED)
 def test_scan_and_hard_reject(self):
  self.assertFalse(unwrap_complete([0,math.pi]))
  w=Workflow();w.load(bundle(),[room()]);w.propose('去书房找书桌');w.confirm_intent();w.scan_goal();w.at_scan();w.start_scan();w.finish_scan([i*math.pi/4 for i in range(9)]);w.analyze([dict(observations()[0],line_of_sight=False)]);self.assertRaisesRegex(WorkflowError,'no candidate',w.preview,w.scan_revision)
if __name__=='__main__':unittest.main()
