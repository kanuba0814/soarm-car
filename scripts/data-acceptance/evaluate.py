"""Trajectory evaluation against ground truth (SE(3) ATE, fixed-interval RPE).

Contract rules implemented here:
- No interpolation across rejected edges / lost frames / segment breaks.
- Dense ATE is computed per contiguous accepted segment. ORB ATE over all OK
  frames is valid only for a single map; map events explicitly invalidate it.
- Coverage, initialization and loss intervals stay in the report.
- Alignment is SE(3) only (RGB-D is metric); no scale alignment anywhere.
"""
import argparse, csv, json, math
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('kind', choices=['dense', 'orb'])
p.add_argument('run_dir')
p.add_argument('groundtruth')
p.add_argument('out')
p.add_argument('--rpe-interval', type=float, default=1.0)
p.add_argument('--rpe-tolerance', type=float, default=0.05)
p.add_argument('--gt-max-dt', type=float, default=0.02)
p.add_argument('--false-accept-m', type=float, default=0.05)
p.add_argument('--false-accept-deg', type=float, default=2.0)
p.add_argument('--min-seg-frames', type=int, default=10)
a = p.parse_args()

out = Path(a.out)
out.mkdir(parents=True, exist_ok=True)


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


gt = []
for line in Path(a.groundtruth).read_text().splitlines():
    if line.startswith('#') or not line.strip():
        continue
    v = line.split()
    t = float(v[0])
    gt.append((t, np.array([float(v[1]), float(v[2]), float(v[3])]),
               quat_to_R(float(v[4]), float(v[5]), float(v[6]), float(v[7]))))
gt_t = np.array([g[0] for g in gt])


def gt_at(t):
    i = np.searchsorted(gt_t, t)
    best = None
    for k in (i - 1, i):
        if 0 <= k < len(gt):
            d = abs(gt_t[k] - t)
            if d <= a.gt_max_dt and (best is None or d < best[0]):
                best = (d, gt[k])
    return best[1] if best else None


def umeyama_se3(src, dst):
    """R, t minimizing ||R @ src + t - dst||; no scale for RGB-D trajectories."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, D, Vt = np.linalg.svd(S)
    d = np.sign(np.linalg.det(U) * np.linalg.det(Vt))
    R = U @ np.diag([1, 1, d]) @ Vt
    t = mu_d - R @ mu_s
    return R, t


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def make_pose(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def fixed_interval_rpe(sequences):
    errors = []
    for seq in sequences:
        times = np.array([fr['t'] for fr in seq])
        for i, fr in enumerate(seq):
            target = fr['t'] + a.rpe_interval
            j = int(np.searchsorted(times, target))
            candidates = [k for k in (j - 1, j) if i < k < len(seq)]
            if not candidates:
                continue
            k = min(candidates, key=lambda q: abs(times[q] - target))
            if abs(times[k] - target) > a.rpe_tolerance:
                continue
            other = seq[k]
            g0, g1 = gt_at(fr['t']), gt_at(other['t'])
            if g0 is None or g1 is None:
                continue
            est_rel = np.linalg.inv(fr['pose_T']) @ other['pose_T']
            gt_rel = np.linalg.inv(make_pose(g0[2], g0[1])) @ make_pose(g1[2], g1[1])
            err = np.linalg.inv(gt_rel) @ est_rel
            errors.append({'dt': other['t'] - fr['t'],
                           'translation_err_m': float(np.linalg.norm(err[:3, 3])),
                           'rotation_err_deg': rot_angle_deg(err[:3, :3])})
    return {
        'n': len(errors),
        'translation_err_median_m': float(np.median([e['translation_err_m'] for e in errors]))
        if errors else None,
        'translation_err_p90_m': float(np.percentile([e['translation_err_m'] for e in errors], 90))
        if errors else None,
        'rotation_err_median_deg': float(np.median([e['rotation_err_deg'] for e in errors]))
        if errors else None,
        'rotation_err_p90_deg': float(np.percentile([e['rotation_err_deg'] for e in errors], 90))
        if errors else None,
    }


if a.kind == 'dense':
    rows = []
    with (Path(a.run_dir) / 'frames.csv').open() as f:
        for r in csv.DictReader(f):
            rows.append(r)
    frames = []
    pose = np.eye(4)
    last_segment = None
    for i, r in enumerate(rows):
        T = np.eye(4)
        for rr in range(3):
            for cc in range(4):
                T[rr, cc] = float(r[f't{rr}{cc}'])
        segment = int(r['segment'])
        accepted = r['accepted'] == '1'
        if segment != last_segment:
            pose = np.eye(4)
        if accepted:
            pose = pose @ np.linalg.inv(T)
        frames.append({'i': int(r['i']), 't': float(r['timestamp']),
                       'accepted': accepted, 'success': r['success'] == '1',
                       'segment': segment, 'T_edge': T, 'pose_T': pose.copy(),
                       'pose': pose[:3, 3].copy()})
        last_segment = segment
    segments = []
    cur = []
    for fr in frames:
        if not cur or fr['segment'] != cur[-1]['segment']:
            if len(cur) >= a.min_seg_frames:
                segments.append(cur)
            cur = []
        if fr['accepted']:
            cur.append(fr)
    if len(cur) >= a.min_seg_frames:
        segments.append(cur)
    # poses are already integrated per segment in frames.csv (identity at reset).
    total_accepted = sum(1 for f in frames if f['accepted'])
    seg_report = []
    for seg in segments:
        pts, gts, assoc = [], [], []
        for fr in seg:
            g = gt_at(fr['t'])
            if g is not None:
                pts.append(fr['pose'])
                gts.append(g[1])
                assoc.append(fr)
        if len(pts) < a.min_seg_frames:
            seg_report.append({'frames': len(seg), 'gt_associated': len(pts), 'ate_rmse_m': None})
            continue
        P = np.array(pts); G = np.array(gts)
        R, t = umeyama_se3(P, G)
        residual = np.linalg.norm((R @ P.T).T + t - G, axis=1)
        seg_report.append({
            'frames': len(seg), 'gt_associated': len(pts),
            'duration_s': seg[-1]['t'] - seg[0]['t'],
            'start_t': seg[0]['t'],
            'ate_rmse_m': float(np.sqrt((residual ** 2).mean())),
            'ate_median_m': float(np.median(residual)),
            'ate_max_m': float(residual.max()),
        })
    rpe = fixed_interval_rpe(segments)
    # False accepts: accepted edges whose GT relative motion disagrees.
    false_accepts = []
    n_fa_checked = 0
    for i in range(1, len(frames)):
        fr, prev = frames[i], frames[i - 1]
        if not fr['accepted'] or fr['segment'] != prev['segment']:
            continue
        g0, g1 = gt_at(prev['t']), gt_at(fr['t'])
        if g0 is None or g1 is None:
            continue
        n_fa_checked += 1
        # est T maps prev-camera points to current-camera points; the GT
        # relative pose must be built the same way: T_gt = G_cur^-1 @ G_prev.
        R_gt_rel = g1[2].T @ g0[2]
        t_gt_rel = g1[2].T @ (g0[1] - g1[1])
        T = fr['T_edge']
        te = np.linalg.norm(T[:3, 3] - t_gt_rel)
        re = rot_angle_deg(T[:3, :3].T @ R_gt_rel)
        false_accepts.append(te > a.false_accept_m or re > a.false_accept_deg)
    summary = {
        'kind': 'dense', 'run': str(a.run_dir), 'edges': len(frames),
        'accepted': total_accepted, 'accepted_rate': total_accepted / max(1, len(frames)),
        'segments_ge_min': len(segments),
        'longest_segment_s': max((s['duration_s'] for s in seg_report if s.get('duration_s')),
                                 default=None),
        'segment_details': seg_report,
        'ate_rmse_best_segment_m': min((s['ate_rmse_m'] for s in seg_report
                                        if s.get('ate_rmse_m') is not None), default=None),
        'rpe_1s': rpe,
        'false_accept_checked_edges': n_fa_checked,
        'false_accept_rate': float(np.mean(false_accepts)) if false_accepts else None,
        'false_accept_thresholds': {'m': a.false_accept_m, 'deg': a.false_accept_deg},
        'rpe_interval_s': a.rpe_interval, 'rpe_tolerance_s': a.rpe_tolerance,
        'gt_max_dt_ms': a.gt_max_dt * 1000,
    }
else:
    rows = []
    with (Path(a.run_dir) / 'tracking.csv').open() as f:
        for r in csv.DictReader(f):
            rows.append(r)
    ok = [r for r in rows if int(r['state']) == 2]
    pts, gs = [], []
    for r in ok:
        g = gt_at(float(r['timestamp']))
        if g is not None:
            pts.append(np.array([float(r['x']), float(r['y']), float(r['z'])]))
            gs.append(g[1])
    R, t = umeyama_se3(np.array(pts), np.array(gs)) if len(pts) >= 3 else (None, None)
    if R is not None:
        P, G = np.array(pts), np.array(gs)
        residual = np.linalg.norm((R @ P.T).T + t - G, axis=1)
        ate = float(np.sqrt((residual ** 2).mean()))
    else:
        ate = None
    # RPE is restricted to contiguous OK-state spans. Map changes without a
    # tracking-state transition remain detectable only through map_events.
    ok_sequences, cur = [], []
    for r in rows:
        if int(r['state']) != 2:
            if cur:
                ok_sequences.append(cur)
                cur = []
            continue
        pose_T = make_pose(quat_to_R(float(r['qx']), float(r['qy']),
                                    float(r['qz']), float(r['qw'])),
                           np.array([float(r['x']), float(r['y']), float(r['z'])]))
        cur.append({'t': float(r['timestamp']), 'pose_T': pose_T})
    if cur:
        ok_sequences.append(cur)
    rpe = fixed_interval_rpe(ok_sequences)
    log = (Path(a.run_dir) / 'run.log')
    map_events = []
    if log.exists():
        for line in log.read_text(errors='ignore').splitlines():
            if any(k in line for k in ('New Map', 'Merge', 'Change to map')):
                map_events.append(line.strip()[:160])
    summary = {
        'kind': 'orb', 'run': str(a.run_dir), 'frames': len(rows),
        'ok_frames': len(ok), 'ok_rate': len(ok) / max(1, len(rows)),
        'gt_associated': len(pts),
        'ate_rmse_m_all_ok': ate,
        'rpe_1s': rpe,
        'map_events': map_events[:20], 'map_event_count': len(map_events),
        'note': 'ATE over all OK frames assumes a single map; map_events above flag breaks. '
                'With multiple maps this number is not a valid single-trajectory ATE.',
        'rpe_interval_s': a.rpe_interval, 'rpe_tolerance_s': a.rpe_tolerance,
        'gt_max_dt_ms': a.gt_max_dt * 1000,
    }

(out / 'evaluation.json').write_text(json.dumps(summary, indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(8, 5))
if a.kind == 'dense':
    ts = [f['t'] for f in frames]
    acc = [f['accepted'] for f in frames]
    ax.scatter(ts, [1 if x else 0 for x in acc], s=3, c=['g' if x else 'r' for x in acc])
    ax.set_yticks([0, 1]); ax.set_yticklabels(['rejected', 'accepted'])
    ax.set_xlabel('t (s)')
    ax.set_title(f"accepted edges timeline; coverage {summary['accepted_rate']:.3f}")
else:
    ts = [float(r['timestamp']) for r in rows]
    st = [int(r['state']) for r in rows]
    ax.scatter(ts, st, s=3, c=['g' if s == 2 else 'r' for s in st])
    ax.set_xlabel('t (s)'); ax.set_ylabel('tracking state')
    ax.set_title(f"state timeline; OK rate {summary['ok_rate']:.3f}")
fig.tight_layout(); fig.savefig(str(out / 'timeline.png'), dpi=130)
print(json.dumps(summary, indent=2))
