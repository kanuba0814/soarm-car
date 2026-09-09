# A1 Held-Out Layout Protocol

Status: **READY_FOR_CAPTURE, NOT EXECUTED**. This protocol freezes the v4 A1 configuration, queries, public method, private evaluator, and original eight survey poses before renderer release. It covers three held-out physical layouts of one AWS Small House, not three independent homes.

## Layout Design

`config/a1_heldout_layouts_v1.json` declares seeds `101`, `103`, and `107`. Each changes three named dining-chair model poses using a deterministic radial outward shift of 0.3--0.6 m plus a fixed tangential offset. These are actual model-pose changes, not relabels or global rigid rotations. Source transforms are evaluator-private design inputs only; the method receives only captured public RGB-D, CameraInfo, source-time self pose, gates, and sanitized detections.

`prepare_a1_heldout_layout.py` requires the pinned source-world hash, refuses an existing destination, records original/changed poses, writes a generated-world hash, and applies declared changed-chair disc-separation and world-bound checks. These checks are not whole-house collision, wall-clearance, reachability, or renderer-feasibility certification.

## Frozen Capture And Scoring

Every layout uses the unchanged eight poses in `config/companion_instance_survey.json`, with no detector-driven pose adaptation. Each pose allows at most two capture attempts; failures remain archived. The source-time pose, full-window 5 degree attitude, RGB-D sync, and segmentation sync gates remain unchanged. A later capture creates eight views per layout. The frozen six queries and four variants create 24 planned records per layout, 72 total; these are planned rows, not results.

After each capture is fixed, the public broker and method run in the existing `bwrap` public-only mount boundary. The private evaluator then runs separately, unchanged. Raw capture archives, public inputs/output, private output, code/config/query/source/generated-world hashes, and per-layout failure records must be retained. Aggregation is per-layout and task-paired, reporting unknown/abstention counts. With only three layouts, no broad bootstrap significance claim is permitted.

## Execution Handoff

`plan_a1_heldout_testbatch.py` writes a resumable, overwrite-refusing batch plan only. The renderer owner must generate each locked world and runtime overlay, perform capture, then execute the recorded public isolation and private evaluation steps. No simulator, GPU, renderer, or capture action has been executed by this preparation change.

The private observed-surface uncertainty widening is a 2-voxel heuristic, not calibrated confidence. V4 corrects the documented RIGHT-positive axis so Q3 uses negative/left and Q4 positive/right; it also records missing definite-target Top-1 as false rather than removing it from the denominator.
