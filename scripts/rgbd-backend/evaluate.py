"""Score identical keyframes, keep graph components and online outputs distinct.

All numbers are offline experimental diagnostics. A candidate graph edge is not
an independently verified relocalization. No GT is passed back to the runner.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read_csv(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))


def matrix(row, prefix):
    pose = np.eye(4)
    pose[:3] = [[float(row[f'{prefix}{r}{c}']) for c in range(4)] for r in range(3)]
    return pose


def components(ids, links):
    parent = {i: i for i in ids}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for link in links:
        x, y = int(link['from']), int(link['to'])
        if x in parent and y in parent:
            parent[root(x)] = root(y)
    return {i: root(i) for i in ids}


def se3_errors(estimated, reference):
    """Translation ATE after rigid alignment; scale stays metric."""
    p = np.asarray(estimated)
    q = np.asarray(reference)
    pc, qc = p - p.mean(0), q - q.mean(0)
    u, _, vt = np.linalg.svd(qc.T @ pc)
    correction = np.diag([1., 1., np.linalg.det(u @ vt)])
    rotation = u @ correction @ vt
    aligned = pc @ rotation.T + q.mean(0)
    return np.linalg.norm(aligned - q, axis=1), aligned


def summarize_groups(poses, labels, gt, times):
    groups = []
    all_errors = []
    for label in sorted(set(labels.values())):
        ids = sorted(i for i in poses if labels[i] == label and i in gt)
        if len(ids) < 3:
            groups.append(dict(group=int(label), frames=len(ids), ate_rmse_m=None))
            continue
        errors, _ = se3_errors([poses[i][:3, 3] for i in ids], [gt[i][:3, 3] for i in ids])
        groups.append(dict(group=int(label), frames=len(ids), duration_s=times[ids[-1]] - times[ids[0]],
                           ate_rmse_m=float(np.sqrt(np.mean(errors**2))),
                           ate_p95_m=float(np.percentile(errors, 95))))
        all_errors.extend(errors)
    return dict(groups=groups, evaluated_nodes=len(all_errors),
                independently_aligned_groups_rmse_m=float(np.sqrt(np.mean(np.square(all_errors))))
                if all_errors else None,
                note='Per-group rigid alignments; pooled RMSE is NOT a single global trajectory ATE.')


def angle(rotation):
    return float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))


def rpe(poses, segments, gt, times):
    errors = []
    ids = sorted(i for i in poses if i in gt)
    ts = np.array([times[i] for i in ids])
    for i in ids:
        target = times[i] + 1.0
        k = int(np.searchsorted(ts, target))
        candidates = [ids[j] for j in (k-1, k) if 0 <= j < len(ids) and ids[j] > i]
        if not candidates:
            continue
        j = min(candidates, key=lambda v: abs(times[v] - target))
        if segments[i] != segments[j] or abs(times[j] - target) > 0.12:
            continue
        error = np.linalg.inv(np.linalg.inv(gt[i]) @ gt[j]) @ (np.linalg.inv(poses[i]) @ poses[j])
        errors.append([float(np.linalg.norm(error[:3, 3])), angle(error[:3, :3])])
    return dict(pairs=len(errors), interval_s=1.0, tolerance_s=0.12,
                translation_median_m=float(np.median(np.array(errors)[:, 0])) if errors else None,
                translation_p95_m=float(np.percentile(np.array(errors)[:, 0], 95)) if errors else None,
                rotation_median_deg=float(np.median(np.array(errors)[:, 1])) if errors else None)


def evaluate_run(run, gt_file):
    rows = read_csv(run / 'online.csv')
    times = {int(r['id']): float(r['timestamp']) for r in rows}
    segments = {int(r['id']): int(r['segment']) for r in rows}
    raw_all = {int(r['id']): matrix(r, 'raw') for r in rows}
    online_all = {int(r['id']): matrix(r, 'online') for r in rows}
    opt = {int(r['id']): matrix(r, 't') for r in read_csv(run / 'optimized-all.csv')}
    links = read_csv(run / 'links-all.csv')
    if set(opt) - set(raw_all):
        raise ValueError('unknown graph node')
    raw = {i: raw_all[i] for i in opt}
    online = {i: online_all[i] for i in opt}
    comp = components(opt, links)
    unique = {(min(int(e['from']), int(e['to'])), max(int(e['from']), int(e['to'])), int(e['type']))
              for e in links}
    cross = [e for e in unique if segments[e[0]] != segments[e[1]]]
    illegal = [e for e in cross if e[2] in (0, 6)]
    if illegal:
        raise ValueError(f'odometry bridges rejected edges: {illegal}')
    result = dict(selected_keyframes=len(rows), optimized_nodes=len(opt),
                  optimized_keyframe_coverage=len(opt) / max(1, len(rows)),
                  final_graph_components=len(set(comp.values())),
                  component_sizes=sorted([sum(c == v for c in comp.values()) for v in set(comp.values())], reverse=True),
                  graph_links=len(unique), non_neighbor_links=sum(e[2] not in (0, 6) for e in unique),
                  cross_segment_links=len(cross), cross_segment_neighbor_links=len(illegal),
                  backend_ms_p50=float(np.median([float(r['backend_ms']) for r in rows])),
                  backend_ms_p95=float(np.percentile([float(r['backend_ms']) for r in rows], 95)),
                  has_ground_truth=gt_file.exists(),
                  online_note='Causal backend snapshots at keyframes; cached frontend, not real-time scheduling.',
                  optimized_note='Each final component optimized separately from the completed read-only DB; retrospective, not online.',
                  uncertainty_note='Uniform uncalibrated graph weights; cannot certify reliability.')
    if not gt_file.exists():
        result['accuracy_verdict'] = 'UNMEASURED: no independent trajectory truth in DC1 bags'
        return result
    gt_values = np.loadtxt(gt_file)
    gt = {}
    for i, t in times.items():
        k = int(np.argmin(np.abs(gt_values[:, 0] - t)))
        if abs(gt_values[k, 0] - t) <= .02:
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_quat(gt_values[k, 4:8]).as_matrix()
            pose[:3, 3] = gt_values[k, 1:4]
            gt[i] = pose
    result['gt_associated_selected_nodes'] = len(gt)
    result['gt_associated_common_nodes'] = sum(i in gt for i in opt)
    result['raw_same_nodes_by_segment'] = summarize_groups(raw, segments, gt, times)
    result['online_same_nodes_by_segment'] = summarize_groups(online, segments, gt, times)
    result['optimized_same_nodes_by_segment'] = summarize_groups(opt, segments, gt, times)
    result['optimized_by_graph_component'] = summarize_groups(opt, comp, gt, times)
    result['raw_rpe_1s'] = rpe(raw, segments, gt, times)
    result['online_rpe_1s'] = rpe(online, segments, gt, times)
    result['optimized_rpe_1s'] = rpe(opt, segments, gt, times)
    link_errors = []
    for e in links:
        i, j = int(e['from']), int(e['to'])
        if int(e['type']) in (0, 6) or i not in gt or j not in gt or i > j:
            continue
        error = np.linalg.inv(np.linalg.inv(gt[i]) @ gt[j]) @ matrix(e, 't')
        link_errors.append(dict(from_id=i, to_id=j, cross_segment=segments[i] != segments[j],
                                translation_m=float(np.linalg.norm(error[:3, 3])),
                                rotation_deg=angle(error[:3, :3])))
    result['non_neighbor_link_gt_errors'] = link_errors
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output')
    p.add_argument('tum_datasets')
    a = p.parse_args()
    base = Path(a.output)
    report = {}
    for seq in sorted(base.iterdir()):
        if not (seq / 'input.json').exists():
            continue
        entry = dict(input=json.loads((seq / 'input.json').read_text()))
        for arm in ('closures', 'refine'):
            run = seq / arm
            if not (run / 'complete.json').exists():
                raise ValueError(f'incomplete run: {run}')
            entry[arm] = evaluate_run(run, Path(a.tum_datasets) / ('rgbd_dataset_' + seq.name) / 'groundtruth.txt')
            (run / 'evaluation.json').write_text(json.dumps(entry[arm], indent=2, allow_nan=False))
        report[seq.name] = entry
    (base / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    for seq, entry in report.items():
        for arm in ('closures', 'refine'):
            result = entry[arm]
            line = dict(sequence=seq, arm=arm, nodes=result['optimized_nodes'],
                        components=result['final_graph_components'], links=result['non_neighbor_links'],
                        cross_segment=result['cross_segment_links'])
            for kind in ('raw', 'online', 'optimized'):
                key = kind + '_same_nodes_by_segment'
                if key in result:
                    line[kind + '_rmse_m'] = result[key]['independently_aligned_groups_rmse_m']
            print(json.dumps(line))


if __name__ == '__main__':
    main()
