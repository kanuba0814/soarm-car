# DEV VLM Format Comparison v2

This bounded follow-up answers whether the v1 `0/4` structured selections indicate bad RGB, visual failure, prompt/format failure, or a need for training. It replays the same four public development RGB-D observations. Version 1 remains immutable at `artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/results/results.json`.

## Measured Result

`HuggingFaceTB/SmolVLM-500M-Instruct` at revision `a7da5b986cb59b408707209984f360a5f4ad7e47` was run locally on CUDA with greedy decoding and 32 output tokens. Each image was opened as RGB at 640x480. The actual processor input recorded in the v2 result is `pixel_values=[1,13,3,512,512]` plus matching attention mask: thirteen RGB 512x512 image tiles, so the runner did not pass an empty image, a grayscale image, or an unresized raw tensor.

| Same four views | Result |
| --- | --- |
| Strict JSON candidate prompt | 0/4 exact JSON responses |
| Simple `integer-or-NONE` candidate prompt | 0/4 exact responses |
| Plain-image nonempty scene descriptions | 4/4 |
| Plain descriptions explicitly mentioning chairs/tables | 2/4 |
| Smol v2 warm local model load | 1.81 s |
| Smol v2 peak CUDA allocation | 1371.86 MiB |
| Per-query latency, 12 queries | 0.23 s to 0.68 s |

The raw selections were `1.`, `A.`, `1.`, and `0.` for both selection modes. The exact checker accepts only a stripped integer candidate label or `NONE`; it intentionally does not remove punctuation, map `A` to an integer, or infer a detection. Raw outputs and checker results are retained in `artifacts/companion-home-validation/evidence/vlm-grid-dev-v2/results/results.json`.

**Finding, high:** the v1 structured-output failure is replicated under a simpler output request, but the selection strings include plausible candidate-like tokens and the separate descriptions identify furniture in relevant views. This is evidence of format/interface unreliability, not proof that the RGB is wrong or that the model cannot recognize furniture. It is also not class correctness: private labels were not read, and no class-agreement result is treated as accuracy.

## Qwen Bounded Attempt

The official `Qwen/Qwen3-VL-2B-Instruct` revision is `89644892e4d85e24eaac8bacfd4f463576704203`; its `model.safetensors` is 4,255,140,312 bytes. Root-only Qwen plus root-only SmolVLM fit the 8 GB download cap. The root-only Qwen download was attempted without authentication, remote code, or environment changes, but timed out after 1,800 seconds with 402,443,084 weight bytes present. No Qwen load or inference was forced. The concrete record is `artifacts/companion-home-validation/evidence/vlm-grid-dev-v2/qwen3-vl-2b-download-attempt.json`.

Therefore no model-size VRAM/latency comparison can be claimed yet. A future retry must finish the pinned download, load a single RGB frame with a small token budget, and measure CUDA peak allocation before any four-view replay.

## Adjustment Versus Training

Do **not** train or start LoRA now. The small sample shows an interface-format problem but does not establish an image-domain failure, target-selection error, or enough training signal. The ordered next experiment is:

1. Complete the pinned Qwen 2B single-frame load/smoke measurement when transfer bandwidth permits.
2. For either model, compare constrained decoding or a native structured-output mechanism with crop-per-candidate prompts and a multi-view candidate aggregation, preserving all raw responses.
3. Only consider LoRA after those tests show reproducible visual grounding errors on a separately governed training/development split. It must not be justified merely by punctuation in a four-image prompt test.

## Design Comparison

| Approach | Primary role | Pros | Cons and boundary |
| --- | --- | --- | --- |
| CPU metric occupancy grid | Free/occupied/unknown reachability and approach routing | Deterministic, auditable, blocks unknown and padded obstacles | Sparse single-view coverage; cannot select semantic targets |
| Scene graph / semantic memory | Associate objects, relations, and multi-view evidence | Supports relational queries and revisits | Association drift and stale geometry; requires explicit uncertainty |
| Small VLM | On-demand semantic proposal or scene description | Low measured Smol memory and sub-second queries after first query | Here: exact format failure 4/4; never emits velocity or a route |
| Gated hybrid | VLM candidate -> depth target -> padded grid approach -> deterministic fallback | Separates semantics from safety; invalid VLM output is rejectable | Does not repair localization, map-frame, or controller failures |

This preserves the graduation design: deterministic occupancy/costmap and navigation remain the authority; semantic memory supplies evidence; a VLM is asynchronous and proposal-only.

## Primary Sources And Corrections

- [Qwen3-VL Technical Report, arXiv:2511.21631](https://arxiv.org/abs/2511.21631). The previously cited `2505.09388` is not the Qwen3-VL report.
- [Official Qwen3-VL repository](https://github.com/QwenLM/Qwen3-VL).
- [Official Qwen3-VL-2B model tree](https://huggingface.co/Qwen/Qwen3-VL-2B-Instruct/tree/89644892e4d85e24eaac8bacfd4f463576704203).
- [Official SmolVLM-500M model tree](https://huggingface.co/HuggingFaceTB/SmolVLM-500M-Instruct/tree/a7da5b986cb59b408707209984f360a5f4ad7e47).

The source correction artifact was archived into this document. No broad benchmark, heldout evaluation, ROS/Nav2 execution, or training was added.

## Residual Risks

- The Qwen performance comparison remains unmeasured because the pinned weight transfer did not complete in the bounded window.
- Four DEV observations are enough to expose the response-contract issue, not to estimate model quality.
- The existing RGB-D grid remains conservative and local; unobserved cells are blocked rather than inferred free.
- Prompt/crop/multi-view changes need a separately versioned comparison; they must retain failed raw outputs.
