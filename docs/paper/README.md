# A1 Held-Out Paper Package

Start with [the full delivery status](../../DELIVERY.md). The separate [AWS navigation diagnostic report](../research/aws-navigation-validation-status.md) records aborted actions, not navigation success; it does not change the frozen A1 evaluation.

`thesis.md` is the editable Chinese manuscript. `references.bib` contains the eight cited primary entries without invented bibliographic fields. `results/` is generated evidence, not hand-edited prose.

## Contents

| Path | Purpose |
|---|---|
| `thesis.pdf` | Browser-rendered, visually checked reading edition |
| `thesis.md` | Editable manuscript and numerical traceability appendix |
| `publication-verification.json` | Document hashes, browser QA, archive receipt checks and test record |
| `environment.json` / `requirements-paper.txt` | Recorded environment and minimal paper-build dependencies |
| `references.bib` | Eight primary citations used by the manuscript |
| `results/a1-heldout-raw-rows.csv` | 72 evaluator rows, including unknown fields |
| `results/a1-heldout-by-variant.csv` | Separate V, V+R, V+M and V+R+M summaries |
| `results/a1-heldout-by-layout.csv` | Three-layout summary; layouts are one house, not independent homes |
| `results/a1-heldout-by-layout-variant.csv` | 12 paired layout/variant summaries |
| `results/a1-heldout-results.json` | Counts, definitions, archive SHA-256 traceability, and code commit |
| `results/pipeline.{svg,png}` | Public/private isolation pipeline |
| `results/pervariant-outcomes.{svg,png}` | Forced/accepted outcomes with unknown counts |
| `results/perlayout-failure-coverage.{svg,png}` | Layout failure and coverage conditions |
| `paper.html` | Offline standalone HTML, generated locally |

## Regenerate Tables And Figures

Run from the repository root. The command opens the three archived `.tar.xz` files read-only through Python `tarfile`; it does not extract, modify, rerun simulation, invoke a model, or write under `artifacts/`.

```bash
python3 scripts/generate_companion_paper_results.py
python3 scripts/build_companion_paper_html.py
```

The generator requires Python 3, `matplotlib`, and an installed local font; the HTML builder additionally requires `markdown-it-py`. Recorded versions are in `environment.json` and `requirements-paper.txt`. To rebuild the PDF, open the generated local `paper.html` in Chromium and print to PDF; the retained PDF was created with agent-browser and checked with Poppler. It checks exactly three archives, 24 evaluator rows per archive, four frozen variants, 72 unique scene/variant/query rows, 24 views, and 18 query-layout conditions. Results use `private-evaluation/evaluation.json.rows`, not only `paired-summary.json`.

## Full Offline Consumer Replay

Using the recorded Python environment (Python 3.12+ with NumPy/Pillow) and `bwrap`, run from the repository root:

```bash
python3 scripts/replay_companion_paper.py --seed 101
python3 scripts/replay_companion_paper.py --seed 103
python3 scripts/replay_companion_paper.py --seed 107
```

Each command verifies the frozen receipt, safely extracts to a new temporary directory, runs the public method without a private-data mount, and independently evaluates its output. It requires exact equality of all 24 evaluator rows and aggregate metrics with the archive. All three commands passed in the delivery verification. It uses frozen detections: no new inference, renderer, ROS, or robot motion is performed. Results are retained at the printed directory and are replays, not additional experimental samples.

## Archive Inspection And Replay Boundary

Inspect without extracting all payloads:

```bash
tar -tJf artifacts/companion-home-validation/evidence/a1-heldout-results-v6/a1-heldout-seed101.tar.xz
tar -xOJf artifacts/companion-home-validation/evidence/a1-heldout-results-v6/a1-heldout-seed101.tar.xz private-evaluation/evaluation.json | python3 -m json.tool
sha256sum artifacts/companion-home-validation/evidence/a1-heldout-results-v6/*.tar.xz
```

For a retained runtime archive, restore into a new directory and use the frozen consumers in order: public RGB-D/public input preparation, `bwrap` public runner, then private evaluator. The recorded command construction is in `scripts/run_a1_heldout_testbatch.py`; its public boundary mounts only `/public`, `/input`, `/config.json`, and code read-only, and binds an output parent. The private evaluator is deliberately outside that boundary. Do not mount `package/private`, source world metadata, segmentation labels, or evaluator outputs into the public method.

Scene regeneration is a separate, renderer-owned operation: `scripts/prepare_a1_heldout_layout.py` consumes `config/a1_heldout_layouts_v1.json`, writes layout locks under `artifacts/companion-home-validation/evidence/a1-heldout-layouts-v1/`, and refuses unsafe overwrite. The archived results already contain the frozen generated-world SHA-256 receipts. This paper package does not run scene generation, renderer, GPU, model inference, simulation, navigation, or physical robot motion.

## Locks, Assets, And Licenses

The frozen A1 query/rule version is `config/a1_visualconf_dev_v1.json` (`a1-visualconf-dev-v6-mrr-bound-correction`). The AWS source pin is in `config/aws_small_house_companion.json`; `config/aws_small_house_companion_assets.json` locks the 370 consumed source files, not the total Git file count. The pinned fork LICENSE is MIT. Raw sensor/calibration archives are indexed by `artifacts/companion-home-validation/evidence/MANIFEST.md`; held-out archives and their hashes are indexed by `results/a1-heldout-results.json`. The YOLO-World weight URL, release identity and enforced hash are in `scripts/run_companion_yoloworld_smoke.py` and each archived `yolo/results.json`; its AGPL-3.0 licensing requires attention before redistribution or deployment. Original `house_floorplan` geometry/maps are not the AWS study maps. No additional model or asset is fetched by either paper script.

## Traceability

Every numerical table claim maps to archive SHA-256, receipt member SHA-256, JSON path `private-evaluation/evaluation.json.rows`, and the generator commit in `results/a1-heldout-results.json`. The three archive hashes are recorded there. The historical `941ea2d` development scaffold is excluded; pre-test corrections `bde1f3e` (relation perturbation handling), `97cfd31` (RIGHT-positive relation-axis convention), and `ca8d021` (MRR upper-bound correction) are named in the manuscript. Unknown accepted identities remain neither correct nor wrong, so accepted error rates are bounds.
