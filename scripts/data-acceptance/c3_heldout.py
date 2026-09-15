"""C3 held-out study: how do deterministic depth-source arms fill removed
measurements, stratified by distance and edge proximity?

Blocks of 5x5 valid pixels are removed (set invalid), each arm processes them,
and coverage plus conditional error are measured separately. Interior vs
near-edge strata separate "smooth surface completion" from "wrong-side-of-edge"
risk. Held-out blocks only probe already-measured surfaces; they cannot
represent true holes, glass or transparent objects (experiment-matrix B4 rule).
"""
import argparse, json
from pathlib import Path

import cv2
import numpy as np

import depth_arms

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('out')
p.add_argument('--frame-step', type=int, default=4)
p.add_argument('--arms', default='nearestfill,median5,edgegate')
p.add_argument('--blocks-per-stratum-bin', type=int, default=10)
p.add_argument('--min-depth', type=float, default=0.3)
p.add_argument('--max-depth', type=float, default=5.0)
p.add_argument('--depth-scale', type=float, default=0.001)
p.add_argument('--seed', type=int, default=17)
a = p.parse_args()

bag, out = Path(a.bag), Path(a.out)
out.mkdir(parents=True, exist_ok=False)
rng = np.random.RandomState(a.seed)
arms = a.arms.split(',')
BINS = [(0.3, 1.0), (1.0, 2.0), (2.0, 3.5), (3.5, 5.0)]
STRATA = ['interior', 'near_edge']
B = 5  # block size
half = B // 2

rows = [line.split() for line in (bag / 'associations.txt').read_text().splitlines()][::a.frame_step]
records = {arm: {s: {f'{lo}-{hi}': [] for lo, hi in BINS} for s in STRATA} for arm in arms}
frames_used = 0

for row in rows:
    d_raw = cv2.imread(str(bag / row[3]), -1)
    if d_raw is None:
        continue
    valid = depth_arms.valid_mask(d_raw, a.min_depth, a.max_depth, a.depth_scale)
    if valid.mean() < 0.3:
        continue
    d_m = d_raw.astype(np.float32) * a.depth_scale
    edge = depth_arms.depth_edge_mask(np.where(valid, d_m, np.nan), 0.05)
    edge_d = cv2.dilate(edge.astype(np.uint8), np.ones((5, 5), np.uint8))
    interior_ok = (cv2.dilate(edge.astype(np.uint8), np.ones((13, 13), np.uint8)) == 0)
    h, w = d_raw.shape
    picks = []
    for stratum in STRATA:
        for lo, hi in BINS:
            cand = np.nonzero(valid & (d_m >= lo) & (d_m < hi))
            if len(cand[0]) == 0:
                continue
            idx = rng.choice(len(cand[0]), min(4 * a.blocks_per_stratum_bin, len(cand[0])),
                             replace=False)
            taken = 0
            for k in idx:
                y, x = cand[0][k], cand[1][k]
                if y < half or y >= h - half or x < half or x >= w - half:
                    continue
                y0, x0 = y - half, x - half
                if any(abs(y0 - py) < B and abs(x0 - px) < B
                       for _, _, py, px in picks):
                    continue
                block_v = valid[y0:y0 + B, x0:x0 + B]
                if block_v.mean() < (1.0 if stratum == 'interior' else 0.8):
                    continue
                if stratum == 'interior' and not interior_ok[y0:y0 + B, x0:x0 + B].all():
                    continue
                if stratum == 'near_edge' and edge_d[y, x]:
                    # want blocks adjacent to, not on, an edge
                    continue
                if stratum == 'near_edge' and not edge_d[y0:y0 + B, x0:x0 + B].any():
                    continue
                picks.append((stratum, f'{lo}-{hi}', y0, x0))
                taken += 1
                if taken >= a.blocks_per_stratum_bin:
                    break
    if not picks:
        continue
    frames_used += 1
    masked = d_raw.copy()
    measured = {}
    for stratum, binname, y0, x0 in picks:
        blk = masked[y0:y0 + B, x0:x0 + B]
        # compare only pixels that carried a real measurement before masking
        measured[(stratum, binname, y0, x0)] = (
            d_raw[y0:y0 + B, x0:x0 + B].copy(), valid[y0:y0 + B, x0:x0 + B].copy())
        blk[:] = 0  # remove the measurement
    for arm in arms:
        filled = depth_arms.preprocess(masked, arm, a.min_depth, a.max_depth, a.depth_scale)
        filled_valid = depth_arms.valid_mask(filled, a.min_depth, a.max_depth, a.depth_scale)
        for stratum, binname, y0, x0 in picks:
            orig, ovalid = measured[(stratum, binname, y0, x0)]
            predicted = filled_valid[y0:y0 + B, x0:x0 + B] & ovalid
            coverage = float(predicted.sum() / ovalid.sum())
            rec = {'coverage': coverage}
            if predicted.any():
                err = (np.abs(filled[y0:y0 + B, x0:x0 + B].astype(np.float32)
                              - orig.astype(np.float32))[predicted]) * a.depth_scale
                rec.update({
                    'median': float(np.median(err)),
                    'p90': float(np.percentile(err, 90)),
                    'frac_gt_5cm': float((err > 0.05).mean()),
                    'frac_gt_10cm': float((err > 0.10).mean()),
                })
            records[arm][stratum][binname].append(rec)

summary = {'bag': bag.name, 'frames_used': frames_used, 'frame_step': a.frame_step,
           'arms': arms, 'blocks_per_stratum_bin': a.blocks_per_stratum_bin,
           'note': 'held-out blocks only represent previously measured surfaces', 'arms_summary': {}}
for arm in arms:
    summary['arms_summary'][arm] = {}
    for stratum in STRATA:
        summary['arms_summary'][arm][stratum] = {}
        for lo, hi in BINS:
            recs = records[arm][stratum][f'{lo}-{hi}']
            if recs:
                scored = [r for r in recs if 'median' in r]
                entry = {
                    'n_blocks': len(recs),
                    'mean_prediction_coverage': float(np.mean([r['coverage'] for r in recs])),
                    'frac_blocks_with_any_prediction': float(len(scored) / len(recs)),
                }
                if scored:
                    entry.update({
                        'median_of_block_medians_m': float(np.median([r['median'] for r in scored])),
                        'median_of_block_p90_m': float(np.median([r['p90'] for r in scored])),
                        'frac_scored_blocks_with_gt10cm_pixel':
                            float(np.mean([r['frac_gt_10cm'] > 0 for r in scored])),
                    })
                summary['arms_summary'][arm][stratum][f'{lo}-{hi}'] = entry
(out / 'c3_heldout.json').write_text(json.dumps(summary, indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
for ax, stratum in zip(axes, STRATA):
    width = 0.8 / len(arms)
    for ai, arm in enumerate(arms):
        xs, ys = [], []
        for ci, (lo, hi) in enumerate(BINS):
            e = summary['arms_summary'][arm][stratum].get(f'{lo}-{hi}')
            if e and 'median_of_block_medians_m' in e:
                xs.append(ci + ai * width - 0.4 + width / 2)
                ys.append(e['median_of_block_medians_m'] * 100)
        ax.bar(xs, ys, width=width, label=arm)
    ax.set_xticks(range(len(BINS)))
    ax.set_xticklabels([f'{lo}-{hi}' for lo, hi in BINS])
    ax.set_title(f'{stratum} blocks')
    ax.set_xlabel('distance bin (m)')
axes[0].set_ylabel('median fill error (cm)')
axes[0].legend()
fig.suptitle(f'C3 held-out fill error (predicted pixels only) {bag.name}')
fig.tight_layout()
fig.savefig(str(out / 'c3_heldout.png'), dpi=130)
print(json.dumps({'frames_used': frames_used,
                  'nearestfill_interior_1-2': summary['arms_summary']['nearestfill']['interior'].get('1.0-2.0'),
                  'nearestfill_near_edge_1-2': summary['arms_summary']['nearestfill']['near_edge'].get('1.0-2.0')}, indent=2))
