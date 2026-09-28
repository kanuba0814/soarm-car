#!/usr/bin/env python3
"""Offline smoke test for uncertainty-adaptive visual/lidar mapping fusion.

The research prototype fuses stereo depth (dense but uncertainty grows with range)
and active-lidar returns (sparse but geometrically reliable). A temporal gate rejects
inconsistent visual cells instead of averaging dynamic/invalid observations.
"""
import argparse
import json
import math
import numpy as np


def fuse(visual, lidar, visual_sigma, lidar_sigma, previous=None, gate=2.5):
    visual = np.asarray(visual, float); lidar = np.asarray(lidar, float)
    vs = np.asarray(visual_sigma, float); ls = np.asarray(lidar_sigma, float)
    # Local sparsity confidence: fewer lidar returns => trust vision more, but never
    # let a single noisy return dominate a dense stereo surface.
    density = np.clip(np.isfinite(lidar).astype(float), 0.0, 1.0)
    vw = (1.0 / np.maximum(vs, 1e-4) ** 2) * (1.0 - 0.55 * density)
    lw = np.where(np.isfinite(lidar), 1.0 / np.maximum(ls, 1e-4) ** 2, 0.0)
    estimate = (vw * visual + lw * np.nan_to_num(lidar)) / np.maximum(vw + lw, 1e-9)
    if previous is not None:
        previous = np.asarray(previous, float)
        innovation = np.abs(estimate - previous)
        temporal_sigma = np.sqrt(1.0 / np.maximum(vw + lw, 1e-9))
        bad = innovation > gate * np.maximum(temporal_sigma, 0.01)
        # Hold the stable map value for gated cells; this is the temporal consistency
        # contribution and is deliberately conservative for moving objects.
        estimate[bad] = previous[bad]
    return estimate


def main():
    p = argparse.ArgumentParser(); p.add_argument('--json', default='artifacts/visual_lidar_fusion_smoke.json')
    a = p.parse_args(); rng = np.random.default_rng(7); n = 240
    truth = 2.0 + 0.35 * np.sin(np.linspace(0, 5 * math.pi, n))
    visual_sigma = 0.015 + 0.012 * truth ** 2
    visual = truth + rng.normal(0, visual_sigma)
    lidar = np.full(n, np.nan); idx = rng.choice(n, 42, replace=False)
    lidar[idx] = truth[idx] + rng.normal(0, 0.018, len(idx))
    # emulate a transient visual outlier/dynamic occluder
    visual[105:118] += 0.55
    baseline = np.nan_to_num(lidar, nan=visual)
    # The prior map is an estimate, never ground truth: use an independent noisy
    # previous frame to make the gate evaluation honest.
    previous = truth + rng.normal(0, 0.012, n)
    fused = fuse(visual, lidar, visual_sigma, np.where(np.isfinite(lidar), .018, 1.), previous=previous)
    visual_rmse = float(np.sqrt(np.mean((visual-truth)**2)))
    fused_rmse = float(np.sqrt(np.mean((fused-truth)**2)))
    result = {'samples': n, 'lidar_returns': int(len(idx)),
              'visual_rmse': visual_rmse, 'fused_rmse': fused_rmse,
              'baseline_lidar_rmse': float(np.sqrt(np.mean((baseline-truth)**2))),
              'outlier_hold_rate': float(np.mean(np.abs(fused[105:118]-previous[105:118]) < .1)),
              'pass': bool(fused_rmse < visual_rmse)}
    print(json.dumps(result, indent=2));
    if a.json:
        import pathlib; pathlib.Path(a.json).parent.mkdir(parents=True, exist_ok=True); pathlib.Path(a.json).write_text(json.dumps(result, indent=2)+'\n')
    return 0 if result['pass'] else 1

if __name__ == '__main__': raise SystemExit(main())
