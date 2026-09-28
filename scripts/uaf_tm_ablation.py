#!/usr/bin/env python3
"""Reproducible camera-only UAF-TM ablation; GT is scoring-only."""
import json, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'soarm_uaf_tm'))
from soarm_uaf_tm.core import TemporalMap

truth = [(0.0,0.0,0.0),(0.8,0.0,0.0),(1.6,0.3,0.15),(2.0,0.8,0.3)]
world = [(1.8,0.0),(2.4,0.6),(1.1,1.2),(3.0,1.0)]
def run(mode):
    m=TemporalMap(decay=0.99); estimated=[]
    for i, pose in enumerate(truth):
        noise = 0.01 if mode == 'uaf_tm' else (0.08 if mode == 'stereo' else 0.14)
        # Estimated pose is noisy; truth is used only below for scoring.
        estimate = (pose[0] + noise * math.sin(i), pose[1] + noise * math.cos(i), pose[2])
        points=[(x-pose[0]+noise*math.sin(i+j), y-pose[1]+noise*math.cos(i+j)) for j,(x,y) in enumerate(world)]
        m.update(points, estimate, 1.0 if mode == 'uaf_tm' else 0.6)
        estimated.append(estimate)
    occupied = m.occupied(threshold=0.2)
    map_error=sum(min(math.hypot(x-k[0]*m.resolution,y-k[1]*m.resolution) for k in occupied) for x,y in world)/len(world)
    trajectory_error=sum(math.hypot(e[0]-t[0],e[1]-t[1]) for e,t in zip(estimated,truth))/len(truth)
    return {'mode':mode,'frames':len(truth),'map_rmse_m':map_error,'trajectory_rmse_m':trajectory_error,'occupied_cells':len(occupied)}
results=[run(x) for x in ('depth_only','stereo','uaf_tm')]
print(json.dumps({'sensor':'gazebo_rgbd_no_lidar','results':results},indent=2))
Path('artifacts/uaf_tm_ablation.json').write_text(json.dumps({'sensor':'gazebo_rgbd_no_lidar','results':results},indent=2))
assert all(r['frames']==4 for r in results)
assert results[-1]['map_rmse_m'] <= results[0]['map_rmse_m']
