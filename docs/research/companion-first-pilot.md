# 真实 RGB-D / 观测位姿开发小试

本记录只覆盖隔离的开发集传感器与几何诊断。所有 `set_pose` 都是诊断放置，不是导航路径；没有调用 `/cmd_vel`、Nav2、scored method 或导航完成接口。原始模型、资产锁、FAST-LIO、安全链和网络隔离未改动。

## 可恢复证据

Git 跟踪的无损证据、consumer 输出和 SHA-256 在 [MANIFEST.md](/home/gxxl/soarm-car-companion-validation/artifacts/companion-home-validation/evidence/MANIFEST.md)。首阶段合同副本在 [companion-metric-pilot-contract.md](/home/gxxl/soarm-car-companion-validation/docs/research/companion-metric-pilot-contract.md)。

当前 C3/C4 接受证据目录为：

```text
artifacts/companion-home-validation/runtime/metric-persistent-a27e139-final
artifacts/companion-home-validation/runtime/metric-persistent-calibration-a27e139
```

对应的 tracked archives 包含 raw RGB、float32 meter depth、invalid mask、完整 CameraInfo、完整 native Pose_V stream、geometry/YOLO/panel consumer JSON 和图像。旧 `first-stage-pilot` 与 `metric-home-pilot-raw` archive 仍可恢复，但其 native Pose_V 只有约 1.1 秒间隔的 CLI endpoints，**已被明确 superseded，不能作为 C3 pose acceptance**。

真实消费链为：

```text
Gazebo Fortress rgbd_camera -> ros_gz_bridge -> rclpy persistence
-> persistent native Pose_V -> source-time interpolation -> reload z-buffer diagnostic -> frozen YOLO-World
```

容器保持 `--network none`、`ROS_LOCALHOST_ONLY=1`、独占 `IGN_PARTITION`、`IGN_IP=127.0.0.1` 和 owned-ID cleanup。源世界和镜像身份保存在每次 `evidence.json`；Kuruma SDF 未改动。

## C3: 持续原生姿态流

早期 bridge `TFMessage` 的 source header 为零，receipt-clock 差不能证明采集时间。新 run 启动独立、持续的 native `ign topic -e /world/default/dynamic_pose/info --json-output` stream，保存每条含 source timestamp 与 exact `kuruma` entity 的完整 envelope。

冻结的 C3 阈值为：source image bracket gap <= 50 ms，image 前后各 250 ms 连续稳定窗，窗内 source gap <= 50 ms，至少 4 samples，任一样本到 image-time interpolated pose 的位移 <= 10 mm、旋转 <= 0.01 rad。每帧以 source-time tight bracket 线性插值位置、SLERP quaternion，并把 `native_pose_at_image` 保存为 geometry 唯一使用的 pose；不再使用 pre/post endpoint。四帧的 bracket 均为 17 ms，稳定窗各 31 samples，最大 stream gap 17--18 ms，最大位移 0.48--0.77 mm，最大旋转 0.00177--0.00317 rad。bridge receipt clocks只保留为诊断。

已知 robot self simulator pose 仅为明确标记的受控研究条件（D2），绝不进入 planner。后续研究命名采用 **VisualConf**；目标身份/位姿/hidden meshes 始终仅供 evaluator，不能泄漏给方法。

## C4: 几何和旋转校准

旧 `a -> c` 的 10.8957 m 不是几何 accuracy：旧实现把 in-bounds 投影直接称为对应且重复加权 target pixel。新 reload 对所有有向对拒绝 nonfinite/behind-camera 点，为每个 target pixel 留一个最近 source Z-buffer 点，并分别报告 target-invalid、behind-nearer-surface、foreground 和 depth-compatible。`depth-compatible` 是描述性分类，不证明 same-surface 或 accuracy。

新 persistent home run 有四个 development views 和 12 个有向对。预先选择的 `overlap_base -> overlap_forward` 有 56.82% supported unique-pixel coverage：245 behind、97 foreground、35,782 descriptive compatible，descriptive absolute median `0.000791 m`、P95 `0.02150 m`。这些数值由保存 raw 重新消费而来，仍不能扩展为 home accuracy、地图、导航或方法结论。

为独立检查完整旋转/轴链，runtime wrapper 只在独立 calibration run 添加固定绿色 X-plane (`x=-5.0 m`) 和垂直的蓝色 Y-plane (`y=2.8 m`)。颜色 mask 在采集和残差之前固定：green `G>150,R/B<100`，blue `B>150,R/G<100`。persistent source-pose 校准后，green-X 在两个 yaw-zero views 均有超过 135k mask pixels，axis median error 为 `-0.00996 m`、`-0.01001 m`；blue-Y 在 90-degree view 有 97,824 mask pixels，median error `-0.00994 m`。两个独立 axes/headings 的 requirement 成立。它仅支持 runtime calibration 的声明 transform/depth convention，绝不替代 furnished-home accuracy 或提供方法目标。

冻结 YOLO-World 实际消费四张 persistent home RGB 并保留 9 个 raw boxes。没有目标 identity、关系 anchor、准确率或 test-split 声明。

## 精确重放

从 Git archive 恢复后，在含有解压后的 run 目录中执行：

```bash
python3 scripts/companion_reload_geometry.py \
  --input-dir artifacts/companion-home-validation/runtime/metric-persistent-a27e139-final \
  --output-dir /tmp/metric-persistent-geometry-replay
/home/gxxl/miniconda3/envs/yolo/bin/python scripts/run_companion_yoloworld_smoke.py \
  --input-dir artifacts/companion-home-validation/runtime/metric-persistent-a27e139-final \
  --output-dir /tmp/metric-persistent-yolo-replay \
  --model /home/gxxl/soarm-car/artifacts/perception-dev/models/yolov8s-worldv2.pt
python3 scripts/companion_panel_calibration.py \
  --input-dir artifacts/companion-home-validation/runtime/metric-persistent-calibration-a27e139 \
  --output-dir /tmp/metric-persistent-panel-replay
```

新 capture 使用默认 `config/companion_metric_pilot.json`。独立 panel run 必须设置 `COMPANION_CAPTURE_CONFIG=config/companion_metric_calibration.json` 和 `COMPANION_RUNTIME_CALIBRATION_PANEL=1`；该开关不能作为 home study、planner 或方法输入。
