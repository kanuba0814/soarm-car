# Frozen A1 VisualConf Development Contract

Version: `a1-visualconf-dev-v1`. This compact contract is the binding final oracle handoff; raw exploration is intentionally not retained.

## Requirements

1. Public method input is limited to RGB-D, CameraInfo, source-time known self pose, public gate results, and sanitized frozen YOLO output. It may not access labels, source instance names/poses, private masks, or source geometry.
2. Recompute source-time and full-window attitude gates. A view with tilt above 5 degrees is `invalid_physical_placement`, retained in reporting, and excluded from eligibility. Stationarity is not a substitute for level placement.
3. Propose foreground from each box conservatively, recording component evidence, failure reason, and box/depth perturbation sensitivity. It is not an evaluator mask.
4. Associate same-class candidates one-to-one under 0.30 m XY, reprojection/depth gates, preserving ambiguous/split cases. Same-frame collapse requires both box IoU >=0.7 and component overlap >=0.8.
5. Score V, V+R, V+M, and V+R+M on identical eight-view budgets. V is maximum frozen detector confidence; singleton M is zero; unknown relation evidence causes relation-enabled abstention. Preserve raw rankings, 0.05 margin decision, answers, and reasons for Q1-Q6.
6. Private evaluation uses exact versioned taxonomy: `KitchenTable_01` is dining table; `CoffeeTable_01` is coffee table; `Tablet_01` is tablet. It must use private observed-mask surfaces independently, class-matched one-to-one IoU >=0.5 identity matching, contamination reporting, and frozen observed-surface relation semantics.

## Fixed Queries

- Q1 `找一把椅子。`
- Q2 `找到沙发。`
- Q3 `以观测 obs_001 的朝向为准，找餐桌左侧最靠左的一把椅子。`
- Q4 `以观测 obs_001 的朝向为准，找餐桌右侧最靠右的一把椅子。`
- Q5 `按本组观测到的物体表面间隙，找离餐桌最近的一把椅子。`
- Q6 `按本组观测到的物体表面间隙，找离餐桌最远的一把椅子。`

## Acceptance Matrix

| Requirement | Observable acceptance |
|---|---|
| Input boundary | Public method runs without private package; outputs contain no source names/GT IDs. |
| Pose/calibration/G4 | Timestamp/full-window attitude gates and exclusions are recorded. |
| Foreground | Every eligible box records component evidence or explicit failure and perturbation sensitivity. |
| Association | Tracks retain constituent detections and audit duplicate/ambiguous/split cases. |
| Ablations | Four variants emit all six query records on identical inputs. |
| Independent scoring | Versioned taxonomy and independent private matching/relation truth avoid use of predicted scores to choose GT. |
| Real outcome | Report candidate recall, Top-1, MRR, coverage, wrong accepts, and relation ambiguity/unknown reasons without filtering missing GT. |
| Freeze | Track code/config/query/input/output hashes and exact replay before held-out physical layouts. |

A negative outcome, including complete abstention, is measured evidence, not a reason to stop. No result is navigation, held-out-layout performance, or localization accuracy.
