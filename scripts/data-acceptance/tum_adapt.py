"""Adapt a TUM RGB-D sequence to the shared unique-pair associations contract.

TUM rgb/depth are hardware-triggered; we still apply unique nearest-neighbour
pairing with an explicit max dt, mirroring the DC1 extraction. Writes
associations.txt into the sequence directory (native file names kept) plus an
adapt.json manifest. Calibration is taken from the upstream ORB-SLAM3 TUM yamls
(official TUM benchmark values); any site/yaml discrepancy is recorded in the
manifest, not silently resolved.
"""
import argparse, json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('seq')
p.add_argument('--max-pair-dt', type=float, default=0.02)
a = p.parse_args()

seq = Path(a.seq)
rgb = sorted((seq / 'rgb').glob('*.png'), key=lambda q: float(q.stem))
depth = sorted((seq / 'depth').glob('*.png'), key=lambda q: float(q.stem))
assert rgb and depth, f'no images under {seq}'
gt = []
gt_file = seq / 'groundtruth.txt'
if gt_file.exists():
    for line in gt_file.read_text().splitlines():
        if line.startswith('#') or not line.strip():
            continue
        v = line.split()
        gt.append(float(v[0]))
    gt_t = gt
else:
    gt_t = []

depth_t = [float(q.stem) for q in depth]
used = set()
pairs = []
for r in rgb:
    t = float(r.stem)
    j = min(range(len(depth_t)), key=lambda k: abs(depth_t[k] - t))
    if abs(depth_t[j] - t) > a.max_pair_dt or j in used:
        continue
    used.add(j)
    pairs.append((t, f'rgb/{r.name}', depth_t[j], f'depth/{depth[j].name}'))

dts = sorted(abs(t - td) * 1000 for t, _, td, _ in pairs)
gt_cov = 0
if gt_t:
    import bisect
    for t, _, _, _ in pairs:
        i = bisect.bisect_left(gt_t, t)
        c = [abs(gt_t[k] - t) for k in (i - 1, i) if 0 <= k < len(gt_t)]
        if c and min(c) <= 0.02:
            gt_cov += 1

manifest = {
    'sequence': seq.name,
    'rgb_frames': len(rgb), 'depth_frames': len(depth),
    'pairs': len(pairs), 'rejected_rgb': len(rgb) - len(pairs),
    'pair_dt_ms': {'median': dts[len(dts) // 2], 'p95': dts[int(0.95 * (len(dts) - 1))],
                   'max': dts[-1]},
    'gt_lines': len(gt_t), 'pairs_with_gt_within_20ms': gt_cov,
    'max_pair_dt_ms': a.max_pair_dt * 1000,
    'depth_scale': 0.0002,
    'calibration_source': 'vendor/orb_slam3 ORB_SLAM3/Examples/RGB-D/TUM*.yaml (official TUM values)',
    'note': 'TUM3 upstream yaml cx=320.1 differs from the TUM site value 320.7 by 0.6 px; '
            'the yaml value is used consistently for both ORB and dense runs.',
}
(seq / 'associations.txt').write_text(
    ''.join(f'{t:.6f} {rp} {td:.6f} {dp}\n' for t, rp, td, dp in pairs))
(seq / 'adapt.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest, indent=2))
