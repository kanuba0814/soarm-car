"""A3: depth validity, hole geometry, edge mixing and temporal flicker statistics.

Measures only what the recordings can support: validity vs range, hole sizes,
transition width at depth discontinuities, and per-pixel temporal variation on
near-static frame pairs (static according to the frozen hybrid odometry edges).
No known-distance ground truth exists, so NO depth-bias conclusion is drawn.
"""
import argparse, json
from pathlib import Path

import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('out')
p.add_argument('--frozen-hybrid-frames', required=True,
               help='frames.csv of the frozen rgbd-comparison hybrid run for static-pair selection')
p.add_argument('--frame-step', type=int, default=2)
p.add_argument('--min-depth', type=float, default=0.3)
p.add_argument('--max-depth', type=float, default=5.0)
p.add_argument('--static-step-m', type=float, default=0.005)
p.add_argument('--static-step-deg', type=float, default=0.2)
a = p.parse_args()

bag, out = Path(a.bag), Path(a.out)
out.mkdir(parents=True, exist_ok=False)
scale = 0.001

rows = [line.split() for line in (bag / 'associations.txt').read_text().splitlines()]
frozen = {}
with open(a.frozen_hybrid_frames) as f:
    header = f.readline().strip().split(',')
    for line in f:
        r = dict(zip(header, line.rstrip('\n').split(',')))
        frozen[int(r['i'])] = r

valid_frac = []
depth_hist = np.zeros(64, np.int64)  # 0.25 m bins to 16 m
row_band_valid = np.zeros((len(rows[::a.frame_step]), 3))
hole_areas = []
mix_widths = []
mix_samples = []
depth_all_samples = []

for fi, row in enumerate(rows[::a.frame_step]):
    d = cv2.imread(str(bag / row[3]), -1)
    if d is None:
        continue
    d_m = d.astype(np.float32) * scale
    ok = (d_m >= a.min_depth) & (d_m <= a.max_depth)
    valid_frac.append(float(ok.mean()))
    hist_bins = np.clip((d_m[d_m > 0] / 0.25).astype(int), 0, 63)
    depth_hist += np.bincount(hist_bins, minlength=64)
    for bi in range(3):
        band = ok[bi * d.shape[0] // 3:(bi + 1) * d.shape[0] // 3, :]
        row_band_valid[fi, bi] = float(band.mean())
    n, lab = cv2.connectedComponents((~ok).astype(np.uint8), connectivity=4)
    sizes = np.bincount(lab.ravel())[1:]  # drop background label 0
    hole_areas.extend(sorted(sizes[sizes > 0].tolist())[-50:])
    # Edge mixing: horizontal neighbour jumps > 0.3 m; count pixels strictly
    # inside the transition within a 7 px window.
    okf = d_m.copy(); okf[~ok] = np.nan
    dj = np.abs(okf[:, 1:] - okf[:, :-1])
    jump = np.nan_to_num(dj, nan=0.0) > 0.3
    ys, xs_ = np.nonzero(jump)
    if len(ys) > 400:
        sel = np.random.RandomState(17).choice(len(ys), 400, replace=False)
    else:
        sel = np.arange(len(ys))
    for k in sel:
        y, x = ys[k], xs_[k]
        lo_v, hi_v = okf[y, x], okf[y, x + 1]
        if not (np.isfinite(lo_v) and np.isfinite(hi_v)):
            continue
        lo, hi = min(lo_v, hi_v), max(lo_v, hi_v)
        win = okf[y, max(0, x - 3):min(d.shape[1], x + 5)]
        inside = np.isfinite(win) & (win > lo + 0.1) & (win < hi - 0.1)
        mix_widths.append(int(inside.sum()))
        mix_samples.append((float(lo), float(hi), float(lo_v), float(hi_v)))
    if fi % 5 == 0:
        depth_all_samples.append(okf[np.isfinite(okf)][::97][:2000])

# Temporal flicker on near-static consecutive pairs, by distance bin.
static_stats = []
used_static = 0
prev_row = None
for fi, row in enumerate(rows):
    r = frozen.get(fi)
    r_prev = frozen.get(fi - 1)
    static = (r is not None and r_prev is not None and r['accepted'] == '1'
              and float(r['step_m']) < a.static_step_m and float(r['step_deg']) < a.static_step_deg)
    if prev_row is not None and static:
        d0 = cv2.imread(str(bag / prev_row[3]), -1).astype(np.float32) * scale
        d1 = cv2.imread(str(bag / row[3]), -1).astype(np.float32) * scale
        ok0 = (d0 >= a.min_depth) & (d0 <= a.max_depth)
        ok1 = (d1 >= a.min_depth) & (d1 <= a.max_depth)
        both = ok0 & ok1
        if both.sum() > 1000:
            diff = np.abs(d1 - d0)[both]
            mid = 0.5 * (d0 + d1)[both]
            bins = np.clip((mid / 0.5).astype(int), 0, 9)

            def stat(b, fn):
                sel = diff[bins == b]
                return float(fn(sel)) if len(sel) else None

            static_stats.append({
                'i': fi,
                'per_bin_median_abs_diff': [stat(b, np.median) for b in range(10)],
                'per_bin_p25': [stat(b, lambda s: np.percentile(s, 25)) for b in range(10)],
                'frac_changed_gt_1cm': float((diff > 0.01).mean()),
                'frac_changed_gt_5cm': float((diff > 0.05).mean()),
            })
            used_static += 1
    prev_row = row

depth_concat = np.concatenate(depth_all_samples) if depth_all_samples else np.array([])
summary = {
    'bag': bag.name, 'frames_used': len(valid_frac), 'frame_step': a.frame_step,
    'valid_fraction': {'median': float(np.median(valid_frac)),
                       'p10': float(np.percentile(valid_frac, 10)),
                       'p90': float(np.percentile(valid_frac, 90))},
    'depth_histogram_0.25m_bins': depth_hist.tolist(),
    'depth_percentiles_m': {q: float(np.percentile(depth_concat, q)) for q in (5, 25, 50, 75, 95)}
    if len(depth_concat) else None,
    'valid_fraction_by_row_band_top_mid_bottom': {
        'median': np.median(row_band_valid, axis=0).tolist(),
        'p10': np.percentile(row_band_valid, 10, axis=0).tolist()},
    'hole_area_px': {'n': len(hole_areas),
                     'median': float(np.median(hole_areas)) if hole_areas else None,
                     'p90': float(np.percentile(hole_areas, 90)) if hole_areas else None,
                     'max': float(np.max(hole_areas)) if hole_areas else None},
    'edge_transition_width_px': {'n': len(mix_widths),
                                 'median': float(np.median(mix_widths)) if mix_widths else None,
                                 'p90': float(np.percentile(mix_widths, 90)) if mix_widths else None,
                                 'frac_ge2': float(np.mean(np.array(mix_widths) >= 2)) if mix_widths else None},
    'static_pairs_used': used_static,
    'static_pair_stats': static_stats[:120],
    'static_pair_median_per_bin': (np.nanmedian(
        [[np.nan if v is None else v for v in s['per_bin_median_abs_diff']] for s in static_stats],
        axis=0).tolist() if static_stats else None),
    'limitations': ['no known-distance ground truth: no bias conclusion',
                    'static-pair flicker includes any real scene motion that the '
                    'frozen odometry did not reject'],
}
(out / 'a3_stats.json').write_text(json.dumps(summary, indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(2, 2, figsize=(11, 7))
axes[0, 0].plot(np.arange(len(valid_frac)) * a.frame_step, valid_frac, lw=.8)
axes[0, 0].set_xlabel('pair index'); axes[0, 0].set_ylabel('valid fraction'); axes[0, 0].set_ylim(0, 1)
axes[0, 0].set_title('valid fraction over time')
axes[0, 1].bar(np.arange(64) * .25, depth_hist / max(1, len(valid_frac)), width=.25)
axes[0, 1].set_xlabel('depth (m)'); axes[0, 1].set_title('mean per-frame pixel count by depth bin')
if hole_areas:
    axes[1, 0].hist(np.log10(np.array(hole_areas) + 1), bins=40)
    axes[1, 0].set_xlabel('log10(hole area px + 1)'); axes[1, 0].set_title('invalid hole sizes (top 50 per frame)')
if mix_widths:
    axes[1, 1].hist(mix_widths, bins=np.arange(-.5, 9.5))
    axes[1, 1].set_xlabel('transition pixels'); axes[1, 1].set_title('edge mixing width at >0.3 m jumps')
fig.tight_layout(); fig.savefig(str(out / 'a3_stats.png'), dpi=130)
if static_stats:
    fig, ax = plt.subplots(figsize=(7, 4))
    med = np.array([[np.nan if v is None else v for v in s['per_bin_median_abs_diff']]
                    for s in static_stats])
    ax.boxplot(med, positions=np.arange(10))
    ax.set_xlabel('distance bin (0.5 m)'); ax.set_ylabel('median |d1-d0| (m)')
    ax.set_title(f'temporal flicker on {len(static_stats)} near-static pairs')
    fig.tight_layout(); fig.savefig(str(out / 'a3_flicker.png'), dpi=130)
print(json.dumps({k: summary[k] for k in ('valid_fraction', 'edge_transition_width_px',
                                          'static_pairs_used', 'hole_area_px')}, indent=2))
