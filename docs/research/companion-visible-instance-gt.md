# Furnished House Visible-Instance GT Protocol

This is an evaluator-only development acquisition seam. It does not implement VisualConf, A1 scoring, navigation, or a policy input containing simulator labels. The continuous native source-time self-pose interpolation and two-axis camera calibration remain required from the prior stage.

## Runtime Overlay

The native Kuruma model and pinned asset clone are never edited. Only the generated runtime wrapper:

1. flattens the source world's 87 embedded model includes into equivalent runtime world includes;
2. attaches `ignition::gazebo::systems::Label` directly to each generated include with a stable unique semantic label ID 1-87;
3. mounts a generated `kuruma_eval` model copy that adds a segmentation sensor co-located with the RGB-D sensor, sharing FOV 1.047, 640x480, near 0.1 m, far 8 m, and 15 Hz;
4. bridges `/eval_segmentation/labels_map` and `/eval_segmentation/colored_map` only inside the evaluator capture container.

The official Fortress seam evidence is retained at [companion-segmentation-seam.md](/home/gxxl/soarm-car-companion-validation/docs/research/sources/companion-segmentation-seam.md). The SAP-Nav overlap warning is retained at [companion-sapnav-overlap-note.md](/home/gxxl/soarm-car-companion-validation/docs/research/companion-sapnav-overlap-note.md).

## Actual Smoke

`segmentation-smoke-overlay-e8974b6.tar.gz` proves the downstream decoder seam, not merely the topic declaration. `labels_map` was observed as `rgb8`, step 1920 for 640 pixels, with every semantic ID repeated in all three byte channels. The direct runtime front model label 251 is present while rear model label 250 is absent at the same alignment, including when real source furniture labels remain visible around the smoke models. This is the expected physical occlusion result: the foreground blocks the rear label rather than exposing a box-derived pseudo-mask. Its evaluator-private overlay manifest records native Kuruma SDF SHA-256 `143dd22a0047cdce511ac0b5c99dc5b96a7c3cc1a75116c585531f963de609fa`, generated `kuruma_eval` SHA-256 `926bc4cc8c0c8eb814df5f04a10c5f1c3c25ea303a1c7cad0d0780a9a8510e88`, 87-label map SHA-256 `e15c83d1d6b8f30eae91cbe05f7f0cc87130d7aeeeea52969ae3a03a9d565b13`, and exact sensor settings.

## Development Survey

`instance-survey-e8974b6.tar.gz` contains eight opaque observation IDs (`obs_001` through `obs_008`) declared in `config/companion_instance_survey.json` before detector inference. Positions were selected offline from source geometry around dining chairs/table and SofaC; this privileged evaluator survey design is never a policy search, detector-driven selection, navigation trajectory, or held-out test split.

For all eight saved observations, RGB/depth/CameraInfo/labels/colored-map timestamps are equal (0 ns RGB-label and RGB-colored deltas), and each source-time self pose passes the persistent interpolation/stability gate. The evaluator decoder rejects any labels map that is not exact replicated-channel `rgb8`, rejects unknown label IDs, and stores source identity/pose only in the private package.

Observed feasibility is real but development-only: `obs_001` through `obs_004` contain multiple distinct `ChairA` labels and `KitchenTable_01_001`; `SofaC_01_001` appears in `obs_003`, `obs_005`, `obs_007`, and `obs_008`. This does not create a relation score or claim a detector found two chairs.

## Public/Private Split

The exporter produces:

- `evaluator-package-final/public/`: RGB, metric depth, invalid mask, CameraInfo, source-time self pose, and opaque observation IDs only.
- `evaluator-package-final/private/`: decoded masks, stable label-ID to source model/URI/pose mapping, GT overlays, and detector-vs-visible-GT matching summary.

A negative leakage check confirms the public tree has no label files, private paths, source instance names, source poses, or GT mask payloads. The private detector summary uses only visible reporting-class masks as its denominator and greedy class-matched IoU >=0.5 boxes. It is visibility accounting only, not relation/navigation/task/accuracy scoring.

## Reproduction

After extracting the tracked archive under `artifacts/companion-home-validation/runtime/`:

```bash
/home/gxxl/miniconda3/envs/yolo/bin/python scripts/run_companion_yoloworld_smoke.py \
  --input-dir artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1 \
  --output-dir /tmp/instance-survey-yolo-replay \
  --model /home/gxxl/soarm-car/artifacts/perception-dev/models/yolov8s-worldv2.pt
python3 scripts/companion_evaluator_gt.py \
  --input-dir artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1 \
  --yolo-results artifacts/companion-home-validation/runtime/instance-survey-e8974b6-retry1/yolo/results.json \
  --output-dir /tmp/instance-survey-evaluator-replay
```

The evaluator process may access private labels during acquisition/export. Future policy/model processes must be mounted with the public package only, never the label topics, private package, source world, or source instance metadata.
