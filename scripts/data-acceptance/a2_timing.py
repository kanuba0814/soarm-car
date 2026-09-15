"""A2: timestamp semantics, RGB/depth re-pairing offset scan, motion error bounds.

Phase 1: header vs arrival latency per stream from the rosbag dump.
Phase 2: for each candidate time offset delta, re-pair RGB t with depth nearest
to t+delta (unique, max 30 ms), run the joint frontend via dense_gen, and
collect consistency metrics on (a) all edges and (b) the RGB-timestamp
intersection across all deltas. A consistency-optimal delta is NOT proof of
hardware sync; it only says which pairing minimises joint residuals here.
Phase 3: observed step/rotation speed distribution from the frozen hybrid run,
converted into first-order geometric error bounds for ms-level mispairing.
"""
import argparse, csv, json, subprocess, sys
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('timestamps_csv')
p.add_argument('out')
p.add_argument('--frozen-hybrid-frames', required=True)
p.add_argument('--deltas-ms', default='-24,-16,-12,-8,-4,-2,0,2,4,8,12,16,24')
p.add_argument('--max-pair-dt', type=float, default=0.03)
p.add_argument('--width', type=int, default=320)
a = p.parse_args()

bag, out = Path(a.bag), Path(a.out)
out.mkdir(parents=True, exist_ok=False)
deltas = [int(d) for d in a.deltas_ms.split(',')]

# ---- Phase 1: header vs arrival -------------------------------------------------
streams = {}
with open(a.timestamps_csv) as f:
    for r in csv.DictReader(f):
        streams.setdefault(r['stream'], []).append(
            (float(r['header_ts']), float(r['recv_ts']), r['path']))
lat = {}
for s, v in streams.items():
    h = np.array([x[0] for x in v]); rr = np.array([x[1] for x in v])
    d = (rr - h) * 1000
    mono = int((np.diff(h) <= 0).sum())
    dd = np.diff(h)
    lat[s] = {'n': len(v), 'recv_minus_header_ms': {
        'median': float(np.median(d)), 'p5': float(np.percentile(d, 5)),
        'p95': float(np.percentile(d, 95))},
        'header_monotonic_violations': mono,
        'header_interval_ms': {'median': float(np.median(dd) * 1000),
                               'p95': float(np.percentile(dd, 95) * 1000),
                               'max': float(dd.max() * 1000)}}

# ---- Phase 2: offset re-pairing scan --------------------------------------------
rgb = streams.get('rgb', [])
depth = streams.get('depth', [])
depth_t = np.array([x[0] for x in depth])


def reindex_pairs(delta_s):
    used = set(); pairs = []
    for t, _, rgb_path in rgb:
        j = int(np.argmin(np.abs(depth_t - (t + delta_s))))
        if abs(depth_t[j] - (t + delta_s)) > a.max_pair_dt or j in used:
            continue
        used.add(j)
        pairs.append((t, rgb_path, depth[j][0], depth[j][2]))
    return pairs


runs = {}
for d_ms in deltas:
    pairs = reindex_pairs(d_ms / 1000.0)
    assoc_dir = out / f'delta_{d_ms:+d}'
    assoc_dir.mkdir()
    assoc = assoc_dir / 'associations.txt'
    with assoc.open('w') as f:
        for t, rp, td, dp in pairs:
            f.write(f'{t:.9f} {rp} {td:.9f} {dp}\n')
    dense_out = assoc_dir / 'dense'
    cmd = [sys.executable, str(Path(__file__).parent / 'dense_gen.py'), str(bag), str(dense_out),
           '--assoc', str(assoc), '--width', str(a.width)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f'dense_gen failed at delta {d_ms}:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}')
    rowsm = []
    with (dense_out / 'frames.csv').open() as f:
        rd = csv.DictReader(f)
        for row in rd:
            rowsm.append({k: (float(v) if k not in ('segment',) else int(v)) for k, v in row.items()})
    ok = [r_ for r_ in rowsm if r_['success'] == 1]
    acc = [r_ for r_ in rowsm if r_['accepted'] == 1]
    runs[d_ms] = {
        'pairs': len(pairs), 'edges': len(rowsm), 'accepted': len(acc),
        'accepted_rate': len(acc) / max(1, len(rowsm)),
        'median_fb_m': float(np.median([r_['fb_m'] for r_ in ok])) if ok else None,
        'median_fb_deg': float(np.median([r_['fb_deg'] for r_ in ok])) if ok else None,
        'median_depth_agreement': float(np.median([r_['depth_agreement'] for r_ in ok])) if ok else None,
        'median_photo': float(np.median([r_['photo_median'] for r_ in ok])) if ok else None,
        'by_ts': {r_['timestamp']: r_ for r_ in rowsm},
    }

# Common-edge metrics: RGB timestamps that are a row in every run.
common_ts = None
for d_ms in deltas:
    s = set(runs[d_ms]['by_ts'])
    common_ts = s if common_ts is None else (common_ts & s)
common_ts = sorted(t for t in common_ts
                   if all(runs[d]['by_ts'][t]['success'] == 1 for d in deltas))
common = {}
for d_ms in deltas:
    rowsm = [runs[d_ms]['by_ts'][t] for t in common_ts]
    acc = [r_ for r_ in rowsm if r_['accepted'] == 1]
    common[d_ms] = {
        'edges': len(rowsm), 'accepted': len(acc),
        'accepted_rate': len(acc) / max(1, len(rowsm)),
        'median_fb_m': float(np.median([r_['fb_m'] for r_ in rowsm])),
        'median_fb_deg': float(np.median([r_['fb_deg'] for r_ in rowsm])),
        'median_depth_agreement': float(np.median([r_['depth_agreement'] for r_ in rowsm])),
        'median_photo': float(np.median([r_['photo_median'] for r_ in rowsm])),
    }

# ---- Phase 3: motion speed -> mispairing error bound ----------------------------
fr = []
with open(a.frozen_hybrid_frames) as f:
    rd = csv.DictReader(f)
    for row in rd:
        fr.append({k: float(v) for k, v in row.items()})
speed_t = np.array([r_['step_m'] / max(r_['dt'], 1e-3) for r_ in fr])
speed_r = np.array([r_['step_deg'] / max(r_['dt'], 1e-3) for r_ in fr])
bounds = {}
for ms in (8, 16, 24):
    t_err = speed_t * ms / 1000.0
    r_err = speed_r * ms / 1000.0
    bounds[f'{ms}ms'] = {
        'translation_cm': {'median': float(np.median(t_err) * 100),
                           'p90': float(np.percentile(t_err, 90) * 100)},
        'rotation_deg': {'median': float(np.median(r_err)), 'p90': float(np.percentile(r_err, 90))},
    }

summary = {
    'bag': bag.name, 'deltas_ms': deltas, 'max_pair_dt_ms': a.max_pair_dt * 1000,
    'width': a.width, 'header_vs_recv': lat,
    'scan': {str(d): {k: v for k, v in runs[d].items() if k != 'by_ts'} for d in deltas},
    'common_edges': {'n': len(common_ts), 'by_delta': {str(d): common[d] for d in deltas}},
    'motion_error_bounds': bounds,
    'motion_speed': {'translation_mps': {'median': float(np.median(speed_t)),
                                         'p90': float(np.percentile(speed_t, 90))},
                     'rotation_degps': {'median': float(np.median(speed_r)),
                                        'p90': float(np.percentile(speed_r, 90))},
                     'source': 'frozen hybrid frames.csv (accepted+rejected edges)'},
}
(out / 'a2_timing.json').write_text(json.dumps(summary, indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 3, figsize=(14, 4))
axes[0].plot(deltas, [runs[d]['accepted_rate'] for d in deltas], marker='o', label='all edges')
axes[0].plot(deltas, [common[d]['accepted_rate'] for d in deltas], marker='s', label=f'common n={len(common_ts)}')
axes[0].set_xlabel('depth time offset delta (ms)'); axes[0].set_ylabel('accepted rate'); axes[0].legend()
axes[1].plot(deltas, [runs[d]['median_depth_agreement'] for d in deltas], marker='o', label='all')
axes[1].plot(deltas, [common[d]['median_depth_agreement'] for d in deltas], marker='s', label='common')
axes[1].set_xlabel('delta (ms)'); axes[1].set_ylabel('median depth agreement'); axes[1].legend()
axes[2].plot(deltas, [runs[d]['median_fb_m'] * 100 for d in deltas], marker='o', label='fb cm all')
axes[2].plot(deltas, [common[d]['median_fb_m'] * 100 for d in deltas], marker='s', label='fb cm common')
axes[2].set_xlabel('delta (ms)'); axes[2].set_ylabel('median forward-backward error (cm)'); axes[2].legend()
fig.suptitle(f'A2 offset scan {bag.name}')
fig.tight_layout(); fig.savefig(str(out / 'a2_scan.png'), dpi=130)

fig, axes = plt.subplots(1, 2, figsize=(10, 4))
for ax, s in zip(axes, ('rgb', 'depth')):
    h = np.array([x[0] for x in streams[s]]); rr = np.array([x[1] for x in streams[s]])
    ax.hist((rr - h) * 1000, bins=60)
    ax.set_title(f'{s}: recv - header (ms), n={len(h)}')
    ax.set_xlabel('ms')
fig.tight_layout(); fig.savefig(str(out / 'a2_latency.png'), dpi=130)
print(json.dumps({'header_vs_recv': lat,
                  'best_common_accepted_delta': max(common, key=lambda d: common[d]['accepted_rate']),
                  'motion_error_bounds': bounds}, indent=2))
