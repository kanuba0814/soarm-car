"""Render evidence already scored by evaluate.py, without connecting segments."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('run')
a = p.parse_args()
base = Path(a.run)
report = json.loads((base / 'summary.json').read_text())
stability = json.loads((base / 'reliability-summary.json').read_text()) if (base / 'reliability-summary.json').exists() else None
fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
seqs = ['freiburg1_xyz', 'freiburg1_room', 'freiburg3_long_office_household']
series = [('Dense only', 'raw', '#9aa6b2'), ('Causal replay output', 'online', '#ef9b3a'),
          ('Final optimized graph', 'optimized', '#167d9a')]
for j, (label, kind, color) in enumerate(series):
    values = [100 * report[s]['refine'][kind + '_same_nodes_by_segment']
              ['independently_aligned_groups_rmse_m'] for s in seqs]
    errors = np.zeros((2, 3))
    if stability and kind in ('online', 'optimized'):
        stats = stability['causal_ate_m' if kind == 'online' else 'final_ate_m']
        values[2] = 100 * stats['median']
        errors[:, 2] = [100 * (stats['median'] - stats['min']), 100 * (stats['max'] - stats['median'])]
    bars = axes[0].bar(np.arange(3) + (j-1)*.25, values, .24, label=label, color=color, yerr=errors, capsize=3)
    axes[0].bar_label(bars, fmt='%.1f', fontsize=8)
axes[0].set_xticks(range(3), ['TUM xyz', 'TUM room', 'TUM long office'])
axes[0].set_ylabel('SE(3) ATE RMSE (cm), identical keyframes')
axes[0].set_title('Refine arm: long office = median + range of 5 runs' if stability else
                  'Refine arm: retrospective accuracy is not online accuracy')
axes[0].legend(frameon=False, fontsize=8)
axes[0].grid(axis='y', alpha=.2)
bags = ['office-20260913-212016', 'office-20260913-212121']
x = np.arange(2)
for j, (label, values, color) in enumerate([
    ('Dense accepted segments', [report[b]['input']['segments'] for b in bags], '#9aa6b2'),
    ('After closures', [report[b]['closures']['final_graph_components'] for b in bags], '#ef9b3a'),
    ('After closures + refinement', [report[b]['refine']['final_graph_components'] for b in bags], '#167d9a'),
]):
    bars = axes[1].bar(x + (j-1)*.25, values, .24, label=label, color=color)
    axes[1].bar_label(bars, fontsize=8)
axes[1].set_xticks(x, ['DC1 212016', 'DC1 212121'])
axes[1].set_ylabel('Disconnected segments / graph components')
axes[1].set_ylim(0, max(report[b]['input']['segments'] for b in bags) + 5)
axes[1].set_title('DC1: no complete map; absolute accuracy unmeasured')
axes[1].legend(frameon=False, fontsize=8)
axes[1].grid(axis='y', alpha=.2)
for ax in axes:
    ax.spines[['top', 'right']].set_visible(False)
fig.tight_layout()
fig.savefig(base / 'evidence.png', dpi=160)
