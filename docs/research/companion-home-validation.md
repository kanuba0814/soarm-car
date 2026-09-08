# Companion Home Renderer Validation

This is an environment and consumer-seam validation for relation-grounding research preparation. It makes no navigation, method-efficacy, map-correspondence, or real-hardware claim. FAST-LIO, Nav2, safety gates, and the current Kuruma model are unchanged.

## Pinned source and acquisition

The source is the TeamSOBITS Humble/Fortress fork of AWS RoboMaker Small House World:

- URL: `https://github.com/TeamSOBITS/aws_small_house_world.git`
- branch: `humble-devel`
- commit: `e2f90621f0fd1a0c496f4e7fcca06138808bc2ec`
- exact fork `LICENSE` SHA-256: `ddf9b5d9dd2fded18653e0ca4c5934008d4faea1161d754493b54a23c25eac14`
- consumed `worlds/small_house.world` SHA-256: `eb25db9b0a7ca10574fa6f908a10a47a6467c8f805785b54c8f3fa74980d6816`
- full mounted-source manifest: `config/aws_small_house_companion_assets.json` (370 files: LICENSE, world, all model SDF/config/DAE/material/texture files), SHA-256 `c8557b77a84f648803ffd2bab6e1ee1fb5ab052717b6fbfb2fe9f4654ad7d2cb`

The exact checked license text is MIT, copyright Amazon.com, Inc. or its affiliates. The fork metadata describes MIT-0, but the pinned file is the controlling retained license evidence. The official upstream is `https://github.com/aws-robotics/aws-robomaker-small-house-world` (MIT); TeamSOBITS is a community fork, not an AWS-maintained endpoint.

Acquire without overwriting an existing destination:

```bash
./scripts/acquire-aws-small-house.sh
```

The clone is deliberately ignored at `artifacts/baselines/aws_small_house_world`; `config/aws_small_house_companion.json` and its full content manifest are Git-recoverable source locks. The acquisition and validation commands check the remote-selected commit, clean tree, license/world hashes, manifest hash, exact file set, and every mounted model-file hash.

## Bounded validation

Use a new output directory and the existing Humble/Fortress image:

```bash
COMPANION_HOME_OUTPUT="$PWD/artifacts/companion-home-validation/runtime/run-001" \
  ./scripts/dev-companion-home-validation.sh
```

The script uses a unique owned container name, `ROS_DOMAIN_ID=173`, and an `IGN_PARTITION` derived from that name. It runs all Gazebo, bridge, and subscriber processes in one `--network none` container with `ROS_LOCALHOST_ONLY=1` and `IGN_IP=127.0.0.1`; host DDS reachability is disabled. It mounts the repository and source read-only, times out readiness and subscription phases, and removes only its owned container ID. It does not publish `/cmd_vel`, invoke Nav2, or contact hardware.

It creates a runtime-only wrapper world that retains the pinned furnished/textured source scene and adds the current `model://kuruma`. The real seam under test is:

```text
Gazebo Fortress rgbd_camera -> ros_gz_bridge -> rclpy Image/CameraInfo subscriber -> saved PNGs and JSON statistics
```

The three source-frame coordinates in the lock are neutral labelled diagnostic teleports. They are requested source-scene positions, not independently verified room semantics or observed poses; no teleport is reported as navigation success. After each pose acknowledgement, the subscriber drains queued messages, establishes a `/clock` fence, then accepts only RGB/depth messages at least 0.5 simulation seconds later and synchronized within 50 ms. It records discarded stale messages, requested versus unverified actual pose, RGB/depth/camera-info timestamps and frames, dimensions, intrinsics, varied RGB content, finite positive depth, saved RGB/depth visualizations, renderer details, and no geometry/texture errors in Gazebo output.

## YOLO-World Baseline Smoke

`/home/gxxl/miniconda3/envs/yolo/bin/python scripts/run_companion_yoloworld_smoke.py` runs only the verified local `yolov8s-worldv2.pt` on the three saved final RGB PNGs. It uses the pretrained COCO vocabulary without a custom text vocabulary, stores every predicted box/class/confidence and a boxes-only overlay, and reports household categories only as a reporting filter. The results record model URL/release asset ID/bytes/hash, package versions, CUDA device, input and output hashes, and measured wall latency. This is not semantic ground truth, accuracy, navigation, or task-success evidence. Ultralytics licensing is AGPL-3.0 unless separately commercially licensed; no deployment claim is made.

## Fresh Observations and Reproduction

Final isolated capture: `artifacts/companion-home-validation/run-final/evidence.json`, UTC `2026-09-08T13:29:59Z`. All three 640×480 RGB/depth pairs have identical within-pair timestamps and pass the post-pose fence. Finite positive depth ratios were 83.57%, 70.94%, and 70.01%. The retained renderer report identifies RTX 3070, not a software-rendering assumption. The working path is display-backed GLX using existing Xauthority; the optional `eglinfo` probe still reports initialization failure, so this is not a verified headless EGL solution. Final renderer text has only its trailing blank line normalized for Git; diagnostic content is unchanged. Earlier host-network evidence is explicitly superseded; it is not the final result.

Run the baseline from this worktree, using a **new** output directory:

```bash
/home/gxxl/miniconda3/envs/yolo/bin/python scripts/run_companion_yoloworld_smoke.py \
    --input-dir artifacts/companion-home-validation/run-final \
    --output-dir artifacts/companion-home-validation/runtime/yolo-repeat \
    --model /home/gxxl/soarm-car/artifacts/perception-dev/models/yolov8s-worldv2.pt
```

If the model is absent, retrieve `yolov8s-worldv2.pt` from the exact official release URL embedded in the script to a new local file, then pass `--model`; its size and SHA-256 are enforced before loading. The script does not require a Hugging Face account or a custom text encoder. The local environment and Docker image are pre-existing dependencies, not newly installed distributions; package versions and image identity are recorded. Reproduction on another machine requires rebuilding/provisioning a compatible environment; the local image tag alone is not a publicly downloadable image guarantee.

The fixed-COCO-vocabulary smoke used confidence threshold 0.10 and produced **9 raw boxes, not 9 correct objects**. Parent visual inspection of `diagnostic_b_rgb_boxes.png` shows a door/window-like region labelled `tv` (0.355) and a small upper-left region labelled `airplane` (0.139). These are qualitative failure observations, not frozen ground truth or accuracy scores. The three measured model-predict wall times were approximately 534 ms, 8.17 ms, and 7.91 ms; the first includes initialization overhead and three frames do not establish P95/real-time performance.

The current Kuruma SDF sets `camera_link` relative pose to `0.180 0 0.150 0 0 0`. Actual low-angle renders contain substantial furniture-side and door-frame occlusion. This motivates checking camera mounting/visibility before selecting tabletop small-object tasks, not silently raising the simulated sensor and claiming real-robot transfer.

The next scientific step is a small manually frozen set of visible target instances and matched observation budgets, with geometry/occupancy rebuilt for **this** scene before navigation. These initial diagnostic views are not independent homes, approved reachable goal poses, or method ablations. No instance-grounding or viewpoint-policy improvement has been demonstrated yet.

## Explicit limits

`house_floorplan` occupancy maps and any PCD are **NOT VERIFIED** for this furnished source scene and are not loaded by the validation. No hidden simulator semantic labels or ground truth are supplied to method inputs. A primitive-world comparison is deferred because this task validates the new scene consumer seam first; it is not needed to claim that the two maps correspond.

## Authorization Delta

The former design stop line that prohibited simulation/download validation is superseded by the current explicit authorization. This work is now authorized to download the open pinned source and run bounded local Docker/Gazebo Fortress ROS RGB-D validation. Physical robot use, uploads, pushes, navigation/method-efficacy claims, and changes to FAST-LIO/Nav2/safety/model remain out of scope.
