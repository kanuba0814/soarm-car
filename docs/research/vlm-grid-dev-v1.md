# DEV VLM/Grid Mini-Demo v1

This is a four-observation development-only comparison, not an accuracy evaluation or a navigation/action claim. It compares a local lightweight VLM's opaque-ID proposal with a conservative CPU RGB-D occupancy-grid filter and fallback.

## Findings

- **High severity, structured-output blocker:** SmolVLM-500M produced non-JSON candidate strings in 4/4 fixed-prompt selections. The raw outputs are preserved in `artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/results/results.json`; the hybrid gate rejected every one instead of accepting a guessed ID.
- **Medium severity, sparse-coverage constraint:** The per-frame grid left 65.6% to 85.3% of cells unknown after raycasting. Unknown is never free, and a 0.50 m occupied-cell padding was applied. Three frames had no feasible candidate approach; only `obs_002:1` was geometry-feasible.
- **Medium severity, frame limitation:** `artifacts/aws-knownselfpose-initial-placement-run-1/raw_cloud_frames.json` records LiDAR frames only. It does not establish a same-run camera/world transform for its `sensor_map.pgm`, so that saved map was deliberately not used. Each result instead uses a per-observation RGB-D raycast in the calibration self-pose world frame.
- **No semantic accuracy claim:** private labels were neither extracted nor read. There is no label-agreement or accuracy denominator in this artifact.

## Measured Tradeoffs

The local GPU run used `HuggingFaceTB/SmolVLM-500M-Instruct` at official revision `a7da5b986cb59b408707209984f360a5f4ad7e47`, with `float16`, greedy decoding, and `max_new_tokens=96`. The saved results include every local model-file SHA-256.

| Measure | Result |
| --- | --- |
| DEV observations | 4 original RGB-D observations (`obs_001` through `obs_004`) |
| Model load (warm local cache) | 0.63 s |
| Peak CUDA allocation | 1371.86 MiB |
| Eight prompt queries | 0.22 s to 0.67 s each |
| Structured selection valid | 0/4 |
| VLM-only usable selections | 0/4 |
| Hybrid VLM-gated feasible selections | 0/4; each rejected for invalid JSON |
| Geometry-only feasible approaches | 1/4 (`obs_002:1`) |
| Action execution | none; no ROS/Nav2 execution |

The plain-image description was deliberately independent from the annotated candidate-ID image. Candidate selection received only annotated public boxes and an opaque `NO_MATCH` option. The VLM never receives or emits a velocity, grid goal, or route. The CPU gate derives a depth-grounded target and searches an approach ring, rather than commanding the target centroid.

## Reproduction

The source archive is tracked at `artifacts/companion-home-validation/evidence/instance-survey-e8974b6.tar.gz` and must hash to `f89bec0a7eef59f7aaee016704b2716cb3149c20389dc62eb9863e376b578e69`. The extractor whitelists only RGB, depth, invalid-depth mask, calibration, self-pose, and already-sanitized boxes. It does not extract labels, masks, or evaluator-private members.

```bash
export PYTHON=/home/gxxl/.cache/soarm-perception-envs/conceptgraphs/bin/python
export PATH=/home/gxxl/.npm-global/bin:$PATH
hf download HuggingFaceTB/SmolVLM-500M-Instruct \
  --revision a7da5b986cb59b408707209984f360a5f4ad7e47 \
  --local-dir /home/gxxl/.cache/soarm-vlm-grid/models/smolvlm-500m \
  --include 'model.safetensors' --include '*.json' --include merges.txt --include vocab.json
$PYTHON scripts/prepare_vlm_grid_public_inputs.py \
  --archive artifacts/companion-home-validation/evidence/instance-survey-e8974b6.tar.gz \
  --metadata-dir artifacts/companion-home-validation/evidence/a1-visualconf-dev-v6/public-input \
  --config config/vlm_grid_dev_v1.json \
  --output-dir /tmp/vlm-grid-public-input
$PYTHON scripts/run_vlm_grid_demo.py \
  --public-dir /tmp/vlm-grid-public-input \
  --config config/vlm_grid_dev_v1.json \
  --model-dir /home/gxxl/.cache/soarm-vlm-grid/models/smolvlm-500m \
  --output-dir /tmp/vlm-grid-results
```

No token, authentication, remote-code trust, upload, or model package upgrade is used. The weights are intentionally outside Git; the pinned revision and file checksums in the result provide restoration evidence. Follow-up v2 retains this v1 evidence unchanged and records a bounded Qwen3-VL download attempt plus prompt-format comparison in `docs/research/vlm-grid-dev-v2.md`.

## Residual Risks

- A depth raycast is a sparse, single-view local observation, not a replacement for a reconciled persistent map.
- Candidate boxes are frozen YOLO detections and may target obstacles or partial surfaces.
- The model's failure to follow this JSON prompt is a measured result for this setup, not a general VLM capability conclusion.
- GPU availability remains shared with the Gazebo worker; this run did not stop or alter it.
