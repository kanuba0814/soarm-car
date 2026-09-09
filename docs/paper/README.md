# A1 Held-Out Paper Package

`thesis.md` is the editable Chinese manuscript. `references.bib` contains the eight cited primary entries without invented bibliographic fields. `results/` is generated evidence, not hand-edited prose.

## Contents

| Path | Purpose |
|---|---|
| `thesis.md` | Editable manuscript and numerical traceability appendix |
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

The generator requires Python 3, `matplotlib`, and an installed local font. It checks exactly three archives, 24 evaluator rows per archive, four frozen variants, 72 unique scene/variant/query rows, 24 views, and 18 query-layout conditions. Results use `private-evaluation/evaluation.json.rows`, not only `paired-summary.json`.

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

The frozen A1 query/rule version is `config/a1_visualconf_dev_v1.json` (`a1-visualconf-dev-v6-mrr-bound-correction`). Source and asset provenance are recorded in `docs/upstream-manifest.yaml`, `docs/research/sources/SHA256SUMS`, `ros2_ws/src/soarm_simulation/models/house_floorplan/SOURCE.md`, and `ros2_ws/src/soarm_simulation/models/house_furniture/SOURCE.md`. Vendor licensing is retained with its source, including `vendor/wheeltec/SOURCE.md`, `vendor/wheeltec/SHA256SUMS`, and upstream LICENSE files. No additional model or asset is fetched by either paper script.

## Traceability

Every numerical table claim maps to archive SHA-256, receipt member SHA-256, JSON path `private-evaluation/evaluation.json.rows`, and the generator commit in `results/a1-heldout-results.json`. The three archive hashes are recorded there. The historical `941ea2d` development scaffold is excluded; pre-test corrections `bde1f3e` (relation perturbation handling), `97cfd31` (RIGHT-positive relation-axis convention), and `ca8d021` (MRR upper-bound correction) are named in the manuscript. Unknown accepted identities remain neither correct nor wrong, so accepted error rates are bounds.
