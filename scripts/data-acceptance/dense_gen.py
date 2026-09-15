"""Generalized frame-to-frame joint RGB-D estimator.

Extends the frozen rgbd-comparison dense.py with:
  --row-offset  first RGB row of the window paired with depth row 0 (A1 scan)
  --rows        window height in input pixels (400 for DC1, 480 for TUM)
  --calib       fx,fy,cx,cy of the FULL RGB image; the window K subtracts the
                row offset from cy because rgb row r maps to window row r-offset
  --depth-scale metres per raw depth unit (DC1 0.001, TUM 0.0002)
  --depth-preproc deterministic depth-source arm for the C3 ablation

Keeps the same per-edge CSV contract: no integration across rejected edges,
segment counter marks broken continuity, health screen is a gross-failure
filter and NOT ground-truth accuracy.
"""
import argparse, csv, json, time
from pathlib import Path

import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('bag')
p.add_argument('out')
p.add_argument('--method', choices=['rgbd', 'icp', 'hybrid'], default='hybrid')
p.add_argument('--stride', type=int, default=1)
p.add_argument('--width', type=int, default=320)
p.add_argument('--limit', type=int, default=0)
p.add_argument('--row-offset', type=int, default=0)
p.add_argument('--rows', type=int, default=400)
p.add_argument('--calib', default='488.09033203125,488.09033203125,320.88531494140625,211.70437622070312')
p.add_argument('--depth-scale', type=float, default=0.001)
p.add_argument('--min-depth', type=float, default=0.3)
p.add_argument('--max-depth', type=float, default=5.0)
p.add_argument('--depth-preproc', choices=['none', 'median5', 'nearestfill', 'edgegate'], default='none')
p.add_argument('--assoc', default='', help='alternative associations file (A2 offset re-pairing); image paths still resolve from the bag dir')
p.add_argument('--seed', type=int, default=17)
a = p.parse_args()

cv2.setNumThreads(1)
cv2.setRNGSeed(a.seed)
bag, out = Path(a.bag), Path(a.out)
out.mkdir(parents=True, exist_ok=False)

fx, fy, cx, cy = (float(v) for v in a.calib.split(','))
cy_window = cy - a.row_offset
K = np.array([[fx, 0, cx], [0, fy, cy_window], [0, 0, 1]], np.float32)
scale = a.width / 640.0
K[:2] *= scale
create = {'rgbd': cv2.rgbd.RgbdOdometry_create, 'icp': cv2.rgbd.ICPOdometry_create,
          'hybrid': cv2.rgbd.RgbdICPOdometry_create}[a.method]
odom = create(cameraMatrix=K, minDepth=a.min_depth, maxDepth=a.max_depth, maxDepthDiff=.07)
meta = {'method': a.method, 'width': a.width, 'rows_input': a.rows, 'row_offset': a.row_offset,
        'calib_full': [fx, fy, cx, cy], 'K_window_scaled': K.tolist(), 'cv2': cv2.__version__,
        'depth_scale': a.depth_scale, 'depth_preproc': a.depth_preproc,
        'min_depth': a.min_depth, 'max_depth': a.max_depth, 'max_depth_diff': .07,
        'stride': a.stride, 'threads': 1, 'seed': a.seed, 'parameters': {}}
for attr in dir(odom):
    if attr.startswith('get'):
        try:
            v = getattr(odom, attr)()
            meta['parameters'][attr] = v.tolist() if isinstance(v, np.ndarray) else v
        except Exception:
            pass
(out / 'config.json').write_text(json.dumps(meta, indent=2, default=lambda v: type(v).__name__))

rows = [line.split() for line in (bag / (a.assoc or 'associations.txt')).read_text().splitlines()][::a.stride]
if a.limit:
    rows = rows[:a.limit]
assert rows, 'no associations'


def preprocess_depth(d_raw_u16):
    """Deterministic depth-source arms; see depth_arms.preprocess."""
    import depth_arms
    return depth_arms.preprocess(d_raw_u16, a.depth_preproc, a.min_depth, a.max_depth, a.depth_scale)


def read(row):
    rgb = cv2.imread(str(bag / row[1]), 0)
    d_raw = cv2.imread(str(bag / row[3]), -1)
    if rgb is None or d_raw is None:
        raise SystemExit(f'missing image {row[1]} / {row[3]}')
    if a.rows != rgb.shape[0]:
        rgb = rgb[a.row_offset:a.row_offset + a.rows]
    if rgb.shape[0] != a.rows or d_raw.shape[0] != a.rows:
        raise SystemExit(f'shape mismatch rgb {rgb.shape} depth {d_raw.shape} rows={a.rows}')
    d_raw = preprocess_depth(d_raw)
    d = d_raw.astype(np.float32) * a.depth_scale
    if a.width != 640:
        rgb = cv2.resize(rgb, (a.width, round(a.rows * scale)), interpolation=cv2.INTER_AREA)
        d = cv2.resize(d, (a.width, round(a.rows * scale)), interpolation=cv2.INTER_NEAREST)
    d[(d < a.min_depth) | (d > a.max_depth)] = np.nan
    return rgb, d


h, w = round(a.rows * scale), a.width
yy, xx = np.mgrid[0:h:4, 0:w:4]
xx = xx.ravel(); yy = yy.ravel()


def residual(src, dst, T):
    s, sd = src; d, dd = dst
    z = sd[yy, xx]; good = np.isfinite(z)
    x = xx[good]; y = yy[good]; z = z[good]
    pts = np.vstack(((x - K[0, 2]) * z / K[0, 0], (y - K[1, 2]) * z / K[1, 1], z))
    q = T[:3, :3] @ pts + T[:3, 3:4]
    ok = q[2] > .1
    u = np.rint(K[0, 0] * q[0] / np.maximum(q[2], .001) + K[0, 2]).astype(int)
    v = np.rint(K[1, 1] * q[1] / np.maximum(q[2], .001) + K[1, 2]).astype(int)
    ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
    ids = np.flatnonzero(ok)
    zerr = np.abs(q[2, ids] - dd[v[ids], u[ids]])
    finite = np.isfinite(zerr); agree = finite & (zerr < .07)
    phot = np.abs(s[y[ids], x[ids]].astype(float) - d[v[ids], u[ids]].astype(float))
    return (float(agree.sum() / max(len(z), 1)),
            float(np.median(zerr[finite])) if finite.any() else 99.,
            float(np.median(phot[agree])) if agree.any() else 255.)


def angle(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


fields = (['i', 'timestamp', 'dt', 'success', 'reverse_success', 'accepted', 'forward_ms',
           'reverse_ms', 'step_m', 'step_deg', 'fb_m', 'fb_deg', 'depth_agreement',
           'depth_median_m', 'photo_median', 'segment', 'x', 'y', 'z']
          + [f't{r}{c}' for r in range(3) for c in range(4)])
prev = read(rows[0]); pose = np.eye(4); segment = 0
with (out / 'frames.csv').open('w') as f:
    writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
    for i, row in enumerate(rows[1:], 1):
        cur = read(row); t = float(row[0]); dt = t - float(rows[i - 1][0])
        start = time.perf_counter()
        ok, T = odom.compute(prev[0], prev[1], None, cur[0], cur[1], None)
        ms = (time.perf_counter() - start) * 1000
        start = time.perf_counter()
        back, B = odom.compute(cur[0], cur[1], None, prev[0], prev[1], None)
        bms = (time.perf_counter() - start) * 1000
        ok = bool(ok) and T is not None and np.isfinite(T).all()
        back = bool(back) and B is not None and np.isfinite(B).all()
        T = T if ok else np.eye(4)
        fb = B @ T if ok and back else None
        trans = float(np.linalg.norm(T[:3, 3])); rot = angle(T[:3, :3])
        fbt = float(np.linalg.norm(fb[:3, 3])) if fb is not None else 99.
        fbr = angle(fb[:3, :3]) if fb is not None else 180.
        agree, derr, photo = residual(prev, cur, T) if ok else (0., 99., 255.)
        accepted = ok and back and fbt < .03 and fbr < 2 and agree > .5 and trans / dt < 3 and rot / dt < 180
        if accepted:
            pose = pose @ np.linalg.inv(T)
        else:
            segment += 1; pose = np.eye(4)
        result = dict(i=i * a.stride, timestamp=t, dt=dt, success=int(ok), reverse_success=int(back),
                      accepted=int(accepted), forward_ms=ms, reverse_ms=bms, step_m=trans, step_deg=rot,
                      fb_m=fbt, fb_deg=fbr, depth_agreement=agree, depth_median_m=derr,
                      photo_median=photo, segment=segment, x=pose[0, 3], y=pose[1, 3], z=pose[2, 3])
        result.update({f't{r}{c}': T[r, c] for r in range(3) for c in range(4)})
        writer.writerow(result)
        if i % 200 == 0:
            f.flush(); print(a.method, a.width, 'offset', a.row_offset, 'edge', i, 'accepted', accepted, flush=True)
        prev = cur
(out / 'complete.json').write_text(json.dumps({'frames': len(rows), 'edges': len(rows) - 1,
                                                'row_offset': a.row_offset, 'preproc': a.depth_preproc}))
