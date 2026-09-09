# DEV SmolVLM Crop/Context Comparison v1

This bounded, offline DEV experiment compares prepared candidate presentations on the same four frozen public RGB observations used by VLM grid v1/v2. It does not read heldout/private labels, evaluate semantic accuracy, authorize motion, run Gazebo, or use navigation/map data. Existing VLM artifacts remain unchanged.

## Setup

The local cached `HuggingFaceTB/SmolVLM-500M-Instruct` revision `a7da5b986cb59b408707209984f360a5f4ad7e47` ran once on CUDA using float16, greedy generation, and 32 output tokens. The fixed selection prompt and SHA-256 of the public manifest, detections, config, RGB content, and processor chat templates are retained in [results.json](../../artifacts/companion-home-validation/evidence/vlm-crop-context-v1/results/results.json).

For each observation, the CPU helper:

- preserves each supplied original floating-point `xyxy` and its source RGB content hash;
- clips only for extraction and rejects non-finite, out-of-bounds, or empty-after-clipping boxes instead of generating a placeholder;
- creates RGB full-image overlays, padded context crops, and readable integer-ID contact sheets;
- uses aspect-ratio-preserving `ImageOps.contain` resizing before padded tile placement.

The actual Smol processor accepted two images for the context condition: the annotated full image followed by the crop contact sheet. This is not a pure presentation-method comparison: full+crop consumed 22 or 30 image tiles and 1,497 or 2,027 input tokens, compared with 13 tiles/900 tokens for full annotations and 9 or 17 tiles/635 or 1,165 tokens for crop sheets.

## Measured Result

| Measure | Result |
| --- | --- |
| Frozen observations | 4 |
| Generation calls | 16 of 24 maximum |
| Strict integer-or-`NONE` response validity | 0/16 |
| Optional-period finite-label adapter validity | 16/16 |
| Cold model load | 2.125 s |
| First generation | 0.678 s |
| Subsequent generations | 0.184 s to 0.484 s |
| Peak CUDA allocated | 1,855.83 MiB |

All normal presentation modes emitted the same adapter label for each corresponding observation (`1.` in this single greedy run). That is not semantic correctness. The saved finite-label adapter maps only labels in the supplied whitelist, without fuzzy matching; `semantic_correctness` is explicitly `NOT_EVALUATED` and no output can authorize motion.

The one reversed-ID full-view control changed the selected opaque ID in every observation. This is a material positional/display-label sensitivity signal, not proof that any underlying candidate was correct or incorrect. Consequently, crop contact sheets and two-image context are now reusable local presentation capabilities, but this experiment does **not** prove improved grounding, selection robustness, or semantic accuracy. It preserves failure cases and raw outputs for later independently labeled evaluation.

## Evidence And Replay

The exported PNG overlays and contact sheets are under `artifacts/companion-home-validation/evidence/vlm-crop-context-v1/results/overlays/`; the plain JSON consumer artifact is `results.json` in the same directory. A fresh-process verifier confirms every candidate and selected ID maps to a supplied original detection:

```bash
PYTHON=/home/gxxl/.cache/soarm-perception-envs/conceptgraphs/bin/python
$PYTHON -m unittest tests/test_vlm_crop_context_prepare.py
$PYTHON scripts/run_vlm_crop_context_v1.py \
  --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input \
  --config config/vlm_crop_context_v1.json \
  --model-dir /home/gxxl/.cache/soarm-vlm-grid/models/smolvlm-500m \
  --output-dir <new-output-dir>
$PYTHON scripts/verify_vlm_crop_context_results.py \
  --results artifacts/companion-home-validation/evidence/vlm-crop-context-v1/results/results.json \
  --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input
```

## Residual Risks

- Four public DEV views and one deterministic run cannot estimate accuracy or generalize latency.
- The context condition has a larger visual-token budget than either single-image condition.
- The label-reversal result makes the current integer-label selection interface unsuitable for an action path without independent target truth and stronger robustness controls.
