# Companion Metric Pilot Evidence Manifest

These lossless archives are tracked so a Git checkout can restore raw RGB, metric float32 depth, masks, complete CameraInfo, Pose_V evidence, and launcher records without relying on ignored runtime paths. The first three archives exclude regenerated geometry/YOLO outputs; later archives include the consumer evidence described below. Raw-data consumers can be replayed independently.

| Archive | SHA-256 | Contents |
| --- | --- | --- |
| `first-stage-pilot-raw.tar.gz` | `b7c42066cfaae58319f53025aba4bec55d8695b4a768f5fb4a820e2dd1267afe` | Original successful three-view pilot `pilot-481eed1-retry5`, retained unchanged. |
| `metric-home-pilot-raw.tar.gz` | `27437a29063808d3893ba3f42c14f969ca06b9799ffb0a3ba58a76ffecb2d42a` | Fresh four-view adjacent/turn development home pilot `metric-pilot-cd88a17-retry2`. |
| `metric-panel-calibration-raw.tar.gz` | `c5ce58c019ece7a29e6023de2fc3e50d47c1c74aa71c21d2890c57d589fbf96b` | Earlier separate three-view runtime-only known-panel calibration `metric-calibration-cd88a17`. |
| `metric-persistent-home-pilot.tar.gz` | `bbf63d3360a4a567a22152c3a2209b45289e905d8393341e306fe804291f963d` | Accepted C3 replacement: four-view home pilot with persistent source-time Pose_V stream, geometry JSON/PNGs, and frozen YOLO JSON/overlays. |
| `metric-persistent-panel-calibration.tar.gz` | `8be9cda8badd724a490db29e28e85d6a4941200ec8df89b2b9fc65a86ed0a1c9` | Accepted C4 rotation replacement: persistent-stream three-view two-axis green-X/blue-Y calibration and panel calibration JSON. |

The original and earlier metric archives remain retained, but their endpoint-only native samples are **superseded for C3 pose acceptance**. The two persistent archives include both raw inputs and the measured consumer outputs required for review.

| Archive | SHA-256 | Contents |
| --- | --- | --- |
| `segmentation-occlusion-smoke-e8974b6.tar.gz` | `51d9abaebf7c565004dbf03f5c9602c3da8432e266462ee4c95bd3e5a2ec5241` | Runtime-only front/rear label occlusion smoke, including native-source pose, raw `rgb8` labels bytes, and synchronization evidence. |
| `instance-survey-e8974b6.tar.gz` | `f89bec0a7eef59f7aaee016704b2716cb3149c20389dc62eb9863e376b578e69` | Eight-view development survey with raw RGB-D/source pose/labels, frozen YOLO output, evaluator-private masks/instance mapping/matching summary, and separate public package. |
| `segmentation-smoke-overlay-e8974b6.tar.gz` | `63d90844fc7acfda01ac7af2b2d93cb65a9860b4501da63d537153f13c1a6db9` | Replacement direct occlusion smoke including generated-overlay model/label hashes, source furniture labels, and front label 251 blocking rear label 250. |
| `instance-survey-label-propagation-failure-e8974b6.tar.gz` | `d025858c936973428607b4a945ee2a465601eb3ca8b75ad2edcf33ed9de1145b` | Retained first eight-view survey failure: direct model-attached labels decoded as background only, proving why generated source models were flattened to world includes before retry. |
| `dino-development-comparison.tar.gz` | `c0f0553aada2018401f9c9bf0bc1aca56070edd479a4b92fc2e1535a30a7899f` | Four-frame boxes-only Grounding-DINO development comparison, raw inputs/results/provenance and the used consumer source; excluded from frozen YOLO A1 held-out metrics. |
