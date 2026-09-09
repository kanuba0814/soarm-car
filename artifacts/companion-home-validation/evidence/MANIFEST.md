# Companion Metric Pilot Evidence Manifest

These lossless archives are tracked so a Git checkout can restore raw RGB, metric float32 depth, masks, complete CameraInfo, Pose_V evidence, and launcher records without relying on ignored runtime paths. They intentionally exclude regenerated geometry/YOLO consumers; rerun those consumers against the extracted raw capture.

| Archive | SHA-256 | Contents |
| --- | --- | --- |
| `first-stage-pilot-raw.tar.gz` | `b7c42066cfaae58319f53025aba4bec55d8695b4a768f5fb4a820e2dd1267afe` | Original successful three-view pilot `pilot-481eed1-retry5`, retained unchanged. |
| `metric-home-pilot-raw.tar.gz` | `27437a29063808d3893ba3f42c14f969ca06b9799ffb0a3ba58a76ffecb2d42a` | Fresh four-view adjacent/turn development home pilot `metric-pilot-cd88a17-retry2`. |
| `metric-panel-calibration-raw.tar.gz` | `c5ce58c019ece7a29e6023de2fc3e50d47c1c74aa71c21d2890c57d589fbf96b` | Earlier separate three-view runtime-only known-panel calibration `metric-calibration-cd88a17`. |
| `metric-persistent-home-pilot.tar.gz` | `bbf63d3360a4a567a22152c3a2209b45289e905d8393341e306fe804291f963d` | Accepted C3 replacement: four-view home pilot with persistent source-time Pose_V stream, geometry JSON/PNGs, and frozen YOLO JSON/overlays. |
| `metric-persistent-panel-calibration.tar.gz` | `8be9cda8badd724a490db29e28e85d6a4941200ec8df89b2b9fc65a86ed0a1c9` | Accepted C4 rotation replacement: persistent-stream three-view two-axis green-X/blue-Y calibration and panel calibration JSON. |

The original and earlier metric archives remain retained, but their endpoint-only native samples are **superseded for C3 pose acceptance**. The two persistent archives include both raw inputs and the measured consumer outputs required for review.
