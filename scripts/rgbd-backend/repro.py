"""Exercise the actual backend repeatedly on a minimal frozen two-keyframe input.

Exit 1 when pose/loop outputs differ, excluding wall-clock timing. This is an
execution reproducibility check, not a claim of ground-truth pose correctness.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('runner')
p.add_argument('images')
p.add_argument('keyframes')
p.add_argument('out')
p.add_argument('--runs', type=int, default=5)
p.add_argument('--keyframe-count', type=int, default=2)
p.add_argument('--vis-nn', choices=['0', '1', '2', '3'])
p.add_argument('--guess-window', type=int)
p.add_argument('--dictionary-nn', choices=['0', '1', '2', '3'])
a = p.parse_args()
if a.runs < 2 or a.keyframe_count < 0:
    p.error('runs must be >=2 and keyframe-count must be >=0 (0 means full sequence)')
out = Path(a.out)
out.mkdir(parents=True, exist_ok=False)
lines = Path(a.keyframes).read_text().splitlines()
if a.keyframe_count:
    lines = lines[:a.keyframe_count]
(out / 'keyframes.txt').write_text('\n'.join(lines) + '\n')
poses, loops = [], []
for n in range(a.runs):
    folder = out / f'run-{n+1}'
    folder.mkdir()
    with (folder / 'run.log').open('w') as log:
        command = [a.runner, a.images, str(out / 'keyframes.txt'), str(folder),
                   '535.4', '539.2', '320.1', '247.6', '480', '0.0002', '8', 'true']
        for key, value in [('Vis/CorNNType', a.vis_nn), ('Vis/CorGuessWinSize', a.guess_window),
                           ('Kp/NNStrategy', a.dictionary_nn)]:
            if value is not None:
                command.append(f'{key}={value}')
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    with (folder / 'online.csv').open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == len(lines)
    poses.append(np.array([[float(row[f'online{r}{c}']) for r in range(3) for c in range(4)] for row in rows]))
    loops.append([row['loop_id'] for row in rows])
maximum = max(float(np.max(np.abs(poses[i] - poses[j])))
              for i in range(a.runs) for j in range(a.runs))
passed = maximum <= 1e-6 and all(x == loops[0] for x in loops)
result = dict(passed=passed, runs=a.runs, keyframes=len(lines),
              max_pose_matrix_element_difference=maximum, tolerance=1e-6,
              loop_ids=loops, vis_nn=a.vis_nn, guess_window=a.guess_window,
              dictionary_nn=a.dictionary_nn)
(out / 'repro.json').write_text(json.dumps(result, indent=2))
print(json.dumps({k: v for k, v in result.items() if k != 'loop_ids'}))
raise SystemExit(0 if passed else 1)
