# A1 VisualConf Development v1

This frozen development run uses only the public package, frozen YOLO boxes, RGB-D, CameraInfo, and known source-time self pose. It is not navigation, a held-out layout result, or a claim of VisualConf improvement.

- Config/output hashes are tracked in `artifacts/companion-home-validation/evidence/a1-visualconf-dev-v1/`.
- Public runner: `scripts/run_a1_visualconf_public.py`.
- Private evaluator: `scripts/evaluate_a1_visualconf_private.py`.
- Taxonomy v1 explicitly maps only `KitchenTable_01` to dining table. `CoffeeTable_01` is coffee table and `Tablet_01` is tablet.

All eight views pass the public full-window 5 degree attitude gate. Public method execution creates 11 conservative tracks and all 24 records for four variants by six frozen Chinese queries. It accepted 12 records. Relation-enabled queries preserve `relation_unknown` abstentions instead of converting unknown evidence to zero.

Private evaluation reports eight definite retrieval cases, coverage 0.5 and four wrong accepted answers. Candidate recall, Top-1 and MRR are `NOT_COMPUTED` because this v1 private evaluator does not repair public tracks with GT identity; that required one-to-one constituent matching remains a concrete next evaluator seam. The one table detector box remains a poor KitchenTable match (IoU 0.13087), so no threshold was lowered to create a relation anchor.

Replay:

```bash
python3 scripts/run_a1_visualconf_public.py \
  --public-dir artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1/evaluator-package-final/public \
  --survey-dir artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1 \
  --yolo-results artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1/yolo/results.json \
  --config config/a1_visualconf_dev_v1.json --output-dir /tmp/a1-public-replay
python3 scripts/evaluate_a1_visualconf_private.py \
  --private-dir artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1/evaluator-package-final/private \
  --method /tmp/a1-public-replay/method.json --output-dir /tmp/a1-private-replay
```

Future held-out physical layouts must use this code/config/query version unchanged. Any threshold or evaluator identity-matching change starts a new development version.
