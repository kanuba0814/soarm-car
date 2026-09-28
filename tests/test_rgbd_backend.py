"""Essential boundaries for offline experimental evidence."""
import importlib.util
import csv
from pathlib import Path
import tempfile
import unittest

import numpy as np


def load(name):
    path = Path(__file__).resolve().parents[1] / 'scripts/rgbd-backend' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class KeyframeTests(unittest.TestCase):
    def test_no_lost_frames_and_both_ends_of_every_fragment(self):
        rows = [dict(timestamp=t, accepted=ok, segment=seg) for t, ok, seg in
                [(0, True, 0), (.1, True, 0), (.2, False, 1),
                 (.3, True, 1), (.4, True, 1), (.5, False, 2),
                 (.6, False, 3), (.7, True, 3)]]
        self.assertEqual(load('prepare').select_keyframes(rows), [0, 1, 3, 4, 7])

    def test_alignment_cannot_hide_metric_scale_error(self):
        evaluate = load('evaluate')
        truth = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 1.]])
        exact, _ = evaluate.se3_errors(truth + 8, truth)
        scaled, _ = evaluate.se3_errors(2 * truth + 8, truth)
        self.assertLess(max(exact), 1e-12)
        self.assertGreater(np.mean(scaled), .5)

    def test_disconnected_graphs_are_never_pooled_into_single_ate(self):
        evaluate = load('evaluate')
        labels = evaluate.components([1, 2, 3, 4], [dict(**{'from': '1', 'to': '2'})])
        self.assertEqual(labels[1], labels[2])
        self.assertEqual(len(set(labels.values())), 3)

    def test_rejected_motion_is_not_integrated_into_next_segment(self):
        prepare = load('prepare')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'frames.csv'
            fields = ['i', 'timestamp', 'accepted', 'segment'] + [f't{r}{c}' for r in range(3) for c in range(4)]
            with path.open('w') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for i, (ok, seg, motion) in enumerate([(1, 0, 1), (0, 1, 99), (1, 1, 2)], 1):
                    edge = np.eye(4)
                    edge[0, 3] = motion
                    row = dict(i=i, timestamp=i, accepted=ok, segment=seg)
                    row.update({f't{r}{c}': edge[r, c] for r in range(3) for c in range(4)})
                    writer.writerow(row)
            rows = prepare.load_dense(path)
            self.assertEqual([r['pose'][0, 3] for r in rows], [-1, 0, -2])


if __name__ == '__main__':
    unittest.main()
