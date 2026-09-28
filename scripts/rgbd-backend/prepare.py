"""Freeze accepted dense poses and keyframes without crossing rejected edges."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def select_keyframes(rows, period=0.5):
    """Include both ends of every accepted span; retain all rejected input rows."""
    selected = set()
    previous = None
    last_key_time = None
    for k, row in enumerate(rows):
        if not row['accepted']:
            if previous is not None:
                selected.add(previous)
            previous, last_key_time = None, None
            continue
        if previous is not None and row['segment'] != rows[previous]['segment']:
            selected.add(previous)
            previous, last_key_time = None, None
        if last_key_time is None or row['timestamp'] - last_key_time >= period:
            selected.add(k)
            last_key_time = row['timestamp']
        previous = k
    if previous is not None:
        selected.add(previous)
    return sorted(selected)


def load_dense(path):
    rows = []
    pose = np.eye(4)
    segment = None
    with Path(path).open() as f:
        for raw in csv.DictReader(f):
            current = int(raw['segment'])
            accepted = raw['accepted'] == '1'
            if current != segment or not accepted:
                pose = np.eye(4)
            edge = np.eye(4)
            edge[:3] = [[float(raw[f't{r}{c}']) for c in range(4)] for r in range(3)]
            if accepted:
                if not np.isfinite(edge).all():
                    raise ValueError('non-finite accepted edge')
                pose = pose @ np.linalg.inv(edge)
            rows.append(dict(i=int(raw['i']), timestamp=float(raw['timestamp']),
                             accepted=accepted, segment=current, pose=pose.copy()))
            segment = current
    return rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument('associations')
    p.add_argument('dense_csv')
    p.add_argument('out')
    p.add_argument('--period', type=float, default=0.5)
    a = p.parse_args()
    if a.period <= 0:
        p.error('period must be positive')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    assoc = [s.split() for s in Path(a.associations).read_text().splitlines() if s.strip() and not s.startswith('#')]
    rows = load_dense(a.dense_csv)
    if [r['i'] for r in rows] != list(range(1, len(assoc))):
        raise ValueError('requires complete stride-1 dense input')
    selected = select_keyframes(rows, a.period)
    with (out / 'keyframes.txt').open('w') as f:
        for k in selected:
            row = rows[k]
            ar = assoc[row['i']]
            if abs(float(ar[0]) - row['timestamp']) > 1e-5:
                raise ValueError('association timestamp mismatch')
            vals = [row['i'] + 1, row['timestamp'], row['segment'], ar[1], ar[3]]
            vals += row['pose'][:3].ravel().tolist()
            f.write(' '.join(str(v) for v in vals) + '\n')
    meta = dict(input_frames=len(assoc), edges=len(rows), accepted=sum(r['accepted'] for r in rows),
                keyframes=len(selected), period_s=a.period,
                segments=len(set(r['segment'] for r in rows if r['accepted'])),
                hashes={str(Path(x).name): hashlib.sha256(Path(x).read_bytes()).hexdigest()
                        for x in (a.associations, a.dense_csv)})
    (out / 'input.json').write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta))


if __name__ == '__main__':
    main()
