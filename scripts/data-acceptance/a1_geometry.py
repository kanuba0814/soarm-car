"""A1: verify the RGB top-400-rows crop assumption for DC1 RGB-D.

Depth discontinuities should coincide with RGB intensity gradients at the row
mapping that makes both images geometrically aligned. We scan affine row maps
rgb_row = a * depth_row + b:
  a=1   : depth is a 400-row crop of the 480-row RGB view (b = crop offset)
  a!=1  : depth covers a different vertical FOV (e.g. a=1.2 same FOV, b=0)
Scoring is per-frame normalized so texture amount does not bias frames, and a
row-circular-shift permutation provides a noise floor. No thresholds here are
industry standards; they are explicit analysis choices recorded in the config.
"""
import argparse, csv, json, time
from pathlib import Path

import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('out')
p.add_argument('--frame-step', type=int, default=2)
p.add_argument('--edge-depth-m', type=float, default=0.05)
p.add_argument('--min-edge-pixels', type=int, default=150)
p.add_argument('--min-depth', type=float, default=0.3)
p.add_argument('--max-depth', type=float, default=5.0)
p.add_argument('--scales', default='1.0,1.05,1.1,1.15,1.2,1.25')
p.add_argument('--dx-scan', type=int, default=8)
a = p.parse_args()

bag, out = Path(a.bag), Path(a.out)
out.mkdir(parents=True, exist_ok=False)
rng = np.random.RandomState(17)

rows = [line.split() for line in (bag / 'associations.txt').read_text().splitlines()]
rows = rows[::a.frame_step]

valid = (a.min_depth, a.max_depth)


def load(idx_row):
    _, rgb_name, _, d_name = idx_row
    rgb = cv2.imread(str(bag / rgb_name), 0)
    d = cv2.imread(str(bag / d_name), -1).astype(np.float32) * 1e-3
    d[(d < valid[0]) | (d > valid[1])] = np.nan
    return rgb, d


def depth_edge_pixels(d):
    """Pixels whose right or bottom depth neighbour differs by > thr, both valid."""
    h, w = d.shape
    ok = np.isfinite(d)
    dv = np.zeros((h - 1, w), bool)
    dh = np.zeros((h, w - 1), bool)
    diff_v = np.abs(d[1:] - d[:-1])
    diff_h = np.abs(d[:, 1:] - d[:, :-1])
    dv = (diff_v > a.edge_depth_m) & ok[1:] & ok[:-1]
    dh = (diff_h > a.edge_depth_m) & ok[:, 1:] & ok[:, :-1]
    m = np.zeros((h, w), bool)
    m[:-1] |= dv
    m[1:] |= dv
    m[:, :-1] |= dh
    m[:, 1:] |= dh
    return m


def rgb_gradient(rgb):
    g = cv2.GaussianBlur(rgb, (3, 3), 0).astype(np.float32)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    return np.hypot(gx, gy)


def map_rows(vs, scale, b):
    return scale * vs + b


# Precompute candidate parameter grid: (scale, [b values covering 0..480-400*scale])
scales = [float(s) for s in a.scales.split(',')]
grid = []
for s in scales:
    b_max = int(round(480 - 400 * s))
    if b_max < 0:
        continue
    bs = list(range(0, b_max + 1)) if s == 1.0 else sorted(set(
        [0, b_max // 2, b_max, int(round((480 - 400 * s) / 2))]))
    grid.append((s, bs))

frame_stats = []
per_frame_rows = []
t0 = time.time()
for fi, row in enumerate(rows):
    rgb, d = load(row)
    if rgb is None or rgb.shape[0] != 480:
        continue
    emask = depth_edge_pixels(d)
    if emask.sum() < a.min_edge_pixels:
        continue
    grad = rgb_gradient(rgb)
    vs, xs = np.nonzero(emask)
    # Per-frame normalization: 75th percentile of gradient inside the mapped
    # window is the "strong edge" bar, so scores compare across frames.
    scores = {}
    for s, bs in grid:
        for b in bs:
            r = np.rint(map_rows(vs.astype(np.float32), s, b)).astype(int)
            k = (r >= 0) & (r < 480)
            if k.sum() < a.min_edge_pixels:
                continue
            vals = grad[r[k], xs[k]]
            # window for the percentile: the mapped band plus margin
            lo, hi = max(0, int(map_rows(0, s, b))), min(480, int(map_rows(399, s, b)) + 1)
            band = grad[lo:hi].ravel()
            scores[f'{s:.2f}:{b}'] = float((vals > np.percentile(band, 75)).mean())
    if not scores:
        continue
    # Noise floor: circular row shift of the gradient by random amounts.
    noise = []
    for _ in range(20):
        shift = int(rng.uniform(20, 460))
        gs = np.roll(grad, shift, axis=0)
        vals = gs[vs, xs]
        noise.append(float((vals > np.percentile(grad, 75)).mean()))
    best = max(scores, key=scores.get)
    frame_stats.append({'frame': fi, 'edges': int(len(vs)), 'best': best,
                        'best_score': scores[best], 'score_b0': scores.get('1.00:0'),
                        'noise_median': float(np.median(noise)),
                        'noise_p95': float(np.percentile(noise, 95))})
    per_frame_rows.append(scores)
    if fi % 200 == 0:
        print(f'frame {fi}/{len(rows)} elapsed {time.time()-t0:.0f}s', flush=True)

keys = sorted(per_frame_rows[0], key=lambda k: (float(k.split(':')[0]), int(k.split(':')[1])))
mat = np.full((len(per_frame_rows), len(keys)), np.nan)
for i, sc in enumerate(per_frame_rows):
    for j, k in enumerate(keys):
        if k in sc:
            mat[i, j] = sc[k]
agg = {k: {'median': float(np.nanmedian(mat[:, j])),
           'mean': float(np.nanmean(mat[:, j])),
           'p25': float(np.nanpercentile(mat[:, j], 25)),
           'p75': float(np.nanpercentile(mat[:, j], 75))}
       for j, k in enumerate(keys)}
# Bootstrap CI (frames resampled) for the a=1 curve and the winner.
boot = {}
for k in ['1.00:0', max(agg, key=lambda x: agg[x]['median'])]:
    j = keys.index(k)
    col = mat[:, j]
    col = col[np.isfinite(col)]
    med = [float(np.median(col[rng.randint(0, len(col), len(col))])) for _ in range(1000)]
    boot[k] = {'median': float(np.median(col)), 'ci5': float(np.percentile(med, 5)),
               'ci95': float(np.percentile(med, 95)), 'n': int(len(col))}

winner = max(agg, key=lambda x: agg[x]['median'])
noise = {'median_of_frame_medians': float(np.median([f['noise_median'] for f in frame_stats])),
         'median_of_frame_p95': float(np.median([f['noise_p95'] for f in frame_stats]))}

# Horizontal alignment check at the winner (a=1 winners only; others need resampling).
xscan = {}
if winner.startswith('1.00:'):
    b = int(winner.split(':')[1])
    dxs = {}
    for dx in range(-a.dx_scan, a.dx_scan + 1):
        vals = []
        for row in rows[::max(1, a.frame_step * 4)]:
            rgb, d = load(row)
            if rgb is None:
                continue
            emask = depth_edge_pixels(d)
            if emask.sum() < a.min_edge_pixels:
                continue
            grad = rgb_gradient(rgb)
            vs, xs = np.nonzero(emask)
            xi = xs + dx
            k = (xi >= 0) & (xi < 640) & (vs + b < 480)
            if k.sum() < a.min_edge_pixels:
                continue
            g = grad[vs[k] + b, xi[k]]
            vals.append(float((g > np.percentile(grad[max(0, b):min(480, b + 400)], 75)).mean()))
        if vals:
            dxs[dx] = float(np.median(vals))
    xscan = {'b': b, 'median_by_dx': dxs}

summary = {
    'bag': bag.name, 'frames_used': len(frame_stats), 'frame_step': a.frame_step,
    'edge_depth_m': a.edge_depth_m, 'min_edge_pixels': a.min_edge_pixels,
    'grid': {f'{s}': bs for s, bs in grid},
    'aggregate': agg, 'winner': winner, 'bootstrap': boot, 'noise_floor': noise,
    'xscan': xscan,
    'verdict_inputs': {
        'winner_vs_b0_median_gap': agg[winner]['median'] - (agg['1.00:0']['median'] if '1.00:0' in agg else float('nan')),
        'winner_above_noise': agg[winner]['median'] > noise['median_of_frame_p95'],
    },
}
(out / 'a1_scan.json').write_text(json.dumps(summary, indent=2))
with (out / 'a1_frames.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(frame_stats[0]))
    w.writeheader()
    w.writerows(frame_stats)

# Plot: score vs b per scale, noise floor band, winner marked.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(10, 5))
for s, bs in grid:
    xs_ = [b for b in bs if f'{s:.2f}:{b}' in agg]
    ys_ = [agg[f'{s:.2f}:{b}']['median'] for b in xs_]
    ax.plot(xs_, ys_, marker='.', label=f'scale={s}')
ax.axhline(noise['median_of_frame_medians'], color='gray', ls='--', label='noise median')
ax.axhline(noise['median_of_frame_p95'], color='gray', ls=':', label='noise p95')
ax.set_xlabel('row offset b (rgb_row = a*depth_row + b)')
ax.set_ylabel('fraction of depth edges on strong RGB gradient')
ax.set_title(f'A1 row-map scan {bag.name}')
ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(str(out / 'a1_curves.png'), dpi=130)

if xscan:
    fig, ax = plt.subplots(figsize=(7, 4))
    ks = sorted(xscan['median_by_dx'], key=int)
    ax.plot([int(k) for k in ks], [xscan['median_by_dx'][k] for k in ks], marker='.')
    ax.set_xlabel('column offset dx')
    ax.set_ylabel('median fraction')
    ax.set_title(f'A1 column scan at b={xscan["b"]}')
    fig.tight_layout()
    fig.savefig(str(out / 'a1_xscan.png'), dpi=130)

# Visual evidence tiles: depth edges over RGB at b=0 and at winner b.
tiles = []
for row in rows[::max(1, len(rows) // 4)][:4]:
    rgb, d = load(row)
    if rgb is None:
        continue
    emask = depth_edge_pixels(d)
    for b in ([0, int(winner.split(':')[1])] if winner.startswith('1.00:') else [0]):
        vis = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
        ys, xs_ = np.nonzero(emask)
        yy = np.clip(ys + b, 0, 479)
        vis[yy, xs_] = (0, 0, 255)
        if b == 0:
            cv2.rectangle(vis, (0, 0), (639, 399), (0, 255, 0), 1)
        cv2.putText(vis, f'b={b}', (10, 25), 0, 0.7, (0, 255, 0), 2)
        tiles.append(cv2.resize(vis, (480, 360)))
if tiles:
    while len(tiles) % 4:
        tiles.append(tiles[-1])
    cv2.imwrite(str(out / 'a1_overlay_samples.jpg'),
                np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))

print(json.dumps({'winner': winner, 'boot': boot, 'noise': noise,
                  'frames_used': len(frame_stats)}, indent=2))
