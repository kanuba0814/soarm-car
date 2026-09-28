"""Summarize every specified repeat; no best-run selection."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from evaluate import evaluate_run

p = argparse.ArgumentParser()
p.add_argument('--groundtruth', required=True)
p.add_argument('--out', required=True)
p.add_argument('runs', nargs='+')
a = p.parse_args()
records = []
weights = None
sample_signature = None
for name in a.runs:
    path = Path(name)
    if not (path / 'complete.json').exists() or not (path / 'export-complete.json').exists():
        raise ValueError(f'incomplete run {path}')
    parameters = [s for s in (path / 'parameters.txt').read_text().splitlines()
                  if not s.startswith('Rtabmap/WorkingDirectory=')]
    if weights is not None and weights != parameters:
        raise ValueError('parameters differ across runs')
    weights = parameters
    result = evaluate_run(path, Path(a.groundtruth))
    (path / 'evaluation.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    # Same selected sample timestamps and raw poses, not just the same node count.
    import csv
    with (path / 'online.csv').open() as f:
        signature = [{k: v for k, v in row.items() if k in ('id', 'timestamp', 'segment') or k.startswith('raw')}
                     for row in csv.DictReader(f)]
    if sample_signature is not None and signature != sample_signature:
        raise ValueError('input poses / sampled nodes differ across runs')
    sample_signature = signature
    records.append(dict(run=str(path), nodes=result['optimized_nodes'],
                        components=result['final_graph_components'], non_neighbor_links=result['non_neighbor_links'],
                        raw_ate_m=result['raw_same_nodes_by_segment']['independently_aligned_groups_rmse_m'],
                        causal_ate_m=result['online_same_nodes_by_segment']['independently_aligned_groups_rmse_m'],
                        final_ate_m=result['optimized_same_nodes_by_segment']['independently_aligned_groups_rmse_m']))
summary = dict(runs=records, same_parameters_except_output_directory=True,
               identical_input_keyframes_and_raw_poses=True,
               input_signature_sha256=hashlib.sha256(json.dumps(sample_signature, sort_keys=True).encode()).hexdigest(),
               note='Repeated executions of one sequence; not independent scenes or a statistical generalization claim.')
for key in ('causal_ate_m', 'final_ate_m'):
    values = [r[key] for r in records]
    summary[key] = dict(min=min(values), median=float(np.median(values)), max=max(values))
Path(a.out).write_text(json.dumps(summary, indent=2, allow_nan=False))
print(json.dumps(summary, indent=2))
