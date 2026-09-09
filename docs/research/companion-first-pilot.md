# 真实 RGB-D / 观测位姿开发小试

本记录只覆盖隔离的开发集传感器与几何诊断。所有 `set_pose` 都是诊断放置，不是导航路径；没有调用 `/cmd_vel`、Nav2、scored method 或导航完成接口。原始模型、资产锁、FAST-LIO、安全链和网络隔离未改动。

## 可恢复证据

Git 跟踪的无损原始证据和 SHA-256 在 [MANIFEST.md](/home/gxxl/soarm-car-companion-validation/artifacts/companion-home-validation/evidence/MANIFEST.md)。其中包含原三视角小试、四视角新 home 小试和独立 panel 校准，因此 checkout 不依赖被忽略的 runtime 目录。首阶段合同副本在 [companion-metric-pilot-contract.md](/home/gxxl/soarm-car-companion-validation/docs/research/companion-metric-pilot-contract.md)。

成功的新 home 原始/消费者目录：

```text
artifacts/companion-home-validation/runtime/metric-pilot-cd88a17-retry2
```

- 原始采集: `capture.json`, `raw_native_dynamic_pose_samples.json`, `raw_bridge_dynamic_pose_stream.json`, `frames/*/{rgb.png,depth_m_float32.npy,depth_invalid_mask.npy,camera_info.json}`
- 修正后几何: `geometry/geometry.json` 和 12 张 directed visibility 图
- 冻结检测器: `yolo/results.json` 和四张 overlay
- 独立 runtime-only panel 校准: `artifacts/companion-home-validation/runtime/metric-calibration-cd88a17/panel-calibration/panel_calibration.json`

真实消费链为：

```text
Gazebo Fortress rgbd_camera -> ros_gz_bridge -> rclpy persistence
-> raw float32 depth/CameraInfo/native Pose_V -> reload z-buffer diagnostic -> frozen YOLO-World
```

容器保持 `--network none`、`ROS_LOCALHOST_ONLY=1`、独占 `IGN_PARTITION`、`IGN_IP=127.0.0.1` 和 owned-ID cleanup。源世界和镜像身份保存在每次 `evidence.json`；Kuruma SDF 未改动。

## C3: 原生姿态时间与稳定性

旧三视角 run 的 `TFMessage` bridge 丢失了 Pose_V source header（bridge header stamp 为零），其 0/1 ms receipt-clock 差不能证明采集时间同步。该限制已经更正。

新四视角 run 对每个图像都通过原生 `ign topic -e /world/default/dynamic_pose/info --json-output` 保存 `kuruma` 的前/后完整 Pose_V envelope。每个图像 ROS stamp 被两个**非零 native source stamp**夹住，且记录了精确实体、位移和旋转稳定区间；TF bridge receipt-clock 仍分开保存、不当作 source timestamp。四个稳定区间位移为 2.93--3.69 mm，旋转为 0.00860--0.01104 rad。此处采用的已知自身 simulator pose 仅是明确标记的受控研究条件，绝不进入 planner。

## C4: 修正的几何解释

旧 `a -> c` 的 10.8957 m 不是可解释的几何准确度：旧实现把 in-bounds 投影直接称为对应且允许多个 source 点重复加权同一 target pixel。保存的旧数据中，34,186 个候选有 33,380 个（97.64%）在更近 target 深度之后。

新 reload 消费者使用完整 observed quaternion，先拒绝 nonfinite/behind-camera 点，再对每个 target pixel 留一个最近 source Z-buffer 点，并对所有 **12 个有向对**分别保存：in-bounds 数、unique target pixel 数、target-invalid 数、behind-nearer-surface 数、foreground 数、以及 depth-compatible 数。`depth-compatible` 只是一项描述性分类，不能证明同一静态表面或 accuracy。它不再使用 `verified_pairs` 或“coverage 即 overlap”的表述。

新 home 的预先选择相邻对 `overlap_base -> overlap_forward` 有 56.74% supported unique-pixel coverage，6,144 behind、414 foreground、29,514 descriptive depth-compatible，absolute residual median 0.03589 m。反向 `overlap_forward -> overlap_base` 为 92.48% coverage，431 behind、9,686 foreground、50,379 compatible，median 0.04175 m。横移和转向方向仍含大量不可见/不兼容表面；全部 90 度方向无 projected support。这些事实支持“相邻视角有候选重叠”，不构成 home accuracy、地图或方法结论。

为独立检查固定相机链和深度单位，runtime wrapper 仅在独立校准 run 中添加了已知世界 `x=-5.0 m` 的绿色 static panel；它不是 home target、planner 输入或 evaluator GT。预先固定的绿色颜色选择在 `panel_base` 和 `panel_forward` 分别得到 world-X median error `-0.00341 m`、`-0.01441 m`，P95 absolute `0.00821 m`、`0.01821 m`。90 度 view 没有 panel pixel，记录为 `NOT_VERIFIED`。这支持声明的轴/米制深度链在该受控诊断中可解释，但不替代 furnished-home 几何验证。

## 感知与后续边界

冻结的本地 `yolov8s-worldv2.pt` 实际消费四张新 RGB 并保留所有 10 个 raw boxes。此小试没有目标身份、关系 anchor、准确率或 test-split 结论。

后续研究命名采用 **VisualConf**。已知 robot self simulator pose 可作为明确标记的受控条件（D2）；目标身份/位姿/hidden meshes 始终仅供 evaluator，不能泄漏给方法或 planner。当前阶段没有扩展到 A1 scoring、导航或 Nav2。

## 重跑

```bash
COMPANION_HOME_OUTPUT="$PWD/artifacts/companion-home-validation/runtime/metric-new" \
  ./scripts/dev-companion-home-validation.sh
python3 scripts/companion_reload_geometry.py \
  --input-dir artifacts/companion-home-validation/runtime/metric-new \
  --output-dir artifacts/companion-home-validation/runtime/metric-new/geometry
/home/gxxl/miniconda3/envs/yolo/bin/python scripts/run_companion_yoloworld_smoke.py \
  --input-dir artifacts/companion-home-validation/runtime/metric-new \
  --output-dir artifacts/companion-home-validation/runtime/metric-new/yolo \
  --model /home/gxxl/soarm-car/artifacts/perception-dev/models/yolov8s-worldv2.pt
```

运行独立 panel 校准需额外设置 `COMPANION_CAPTURE_CONFIG=config/companion_metric_calibration.json` 和 `COMPANION_RUNTIME_CALIBRATION_PANEL=1`。该开关只能用于校准，不得作为 home study 或方法输入。
