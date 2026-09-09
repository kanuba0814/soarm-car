# 首阶段真实 RGB-D / 观测位姿开发小试

本记录是开发集诊断，不是导航、方法精度、地图对应、任务成功或实物实验结论。使用的三个位置都是 `set_pose` 诊断放置，未调用 `/cmd_vel`、Nav2 或任何 scored method；请求位姿与独立观测位姿分别保存，前者不作为路径或实际位姿。

## 可恢复结果

成功小试目录（大型原始数据保持本地、被 Git 忽略）：

```text
artifacts/companion-home-validation/runtime/pilot-481eed1-retry5
```

- `capture.json` SHA-256 `5c434b8432d1a007cb9f00de520cba62a05eda81cbaa830dcf8dfb4a405e4d63`
- `evidence.json` SHA-256 `924699fd7dc8d0b2671e64bc1dab0ef11ce06f407a3e5d1d88179cbaf0df8acb`
- `raw_dynamic_pose_stream.json` SHA-256 `827a28bcb52fc8402b81ca052264e778c98464493d6510b40828845d392560c8`
- `geometry/geometry.json` SHA-256 `cb2dcf4b7a27f2dec3b658f248a9899aeb47736b8859b011a136d95b2fdced7c`
- `yolo/results.json` SHA-256 `a79cf4ac9ca8cc51fadb2038be9e7b2fa31cc8dfb1a90a771778876992098eb6`

每个视角保留原 RGB PNG、`float32` 米制 `depth_m_float32.npy`、单独无效掩码、深度预览、全部 `CameraInfo` K/P/D/R/encoding/step/endianness 和帧级时间关联。源世界、许可证、全内容 manifest 和容器镜像身份由 `evidence.json` 固定；运行时 Kuruma 模型 `ros2_ws/src/soarm_simulation/models/kuruma/model.sdf` SHA-256 为 `143dd22a0047cdce511ac0b5c99dc5b96a7c3cc1a75116c585531f963de609fa`。未修改原模型、资产锁、安全链或网络隔离。

## 实测小试

隔离容器使用 `--network none`、`ROS_LOCALHOST_ONLY=1`、独占 `IGN_PARTITION` 和 `IGN_IP=127.0.0.1`。真实消费链为：

```text
Gazebo Fortress rgbd_camera -> ros_gz_bridge -> rclpy subscriber
-> metric depth/calibration/observed pose persistence -> reload/projective geometry -> frozen YOLO-World
```

三个不同开发视角为 `diagnostic_a`、`diagnostic_b`、`diagnostic_c`。它们包含非共线平移和约 90 度以上的朝向变化。各 RGB/depth 对时间差为 0 ns，深度有效比率依次为 83.21%、70.86%、70.01%。所有图像在放置确认后至少 0.5 秒仿真时间的 fence 后接收。

`/world/default/dynamic_pose/info` 实际可用，并通过 `gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage` 桥接。Fortress 此消息在桥后没有源 header stamp（原始记录为零/空）；因此采集器在同一隔离 rclpy executor 中为每一条观测记录接收时的 `/clock`，并以此关联图像。三帧位姿-图像差分别为 0、1、1 ms。该缺口和原始流均保留，不能将请求坐标替代为观测值。

相机链严格按当前模型：model -> base `z=.0763`，base -> `camera_link` 为 `(.180, 0, .150)`。离线消费者明确使用 optical `+Z` 前、`+X` 右、`+Y` 下约定，并输出 world 点云及逐对 projective-depth 残差图。三对都有实际 projective 对应，但建议阈值均未通过：

| 视角对 | 覆盖率 | median m | P95 m | 状态 |
| --- | ---: | ---: | ---: | --- |
| a -> b | 14.78% | 0.0619 | 1.5740 | 未通过 |
| a -> c | 53.46% | 10.8957 | 11.1282 | 未通过 |
| b -> c | 69.74% | 0.2298 | 7.1374 | 未通过 |

这些数值来自跨视角投影到目标深度的对应，不是同一平面拟合或最近邻伪匹配。结果暴露相机轴/姿态链或放置后物理稳定性仍需诊断，不能作为几何准确性证据。

冻结的本地 `yolov8s-worldv2.pt`（SHA-256 `9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792`）实际消费三个新 RGB，保存全部 8 个原始框和叠图。`diagnostic_c` 有四个 `chair` 输出，但高度重叠，不能当成两个独立椅子；没有 `dining table`、`couch` 或其他独立关系锚点。因此 A1 的“两个不同椅子及桌/沙发等锚点”先决条件**不成立**，没有生成任何替代检测或精度分数。

## 重跑

新目录不能覆盖旧 run：

```bash
COMPANION_HOME_OUTPUT="$PWD/artifacts/companion-home-validation/runtime/pilot-new" \
  ./scripts/dev-companion-home-validation.sh
python3 scripts/companion_reload_geometry.py \
  --input-dir artifacts/companion-home-validation/runtime/pilot-new \
  --output-dir artifacts/companion-home-validation/runtime/pilot-new/geometry
/home/gxxl/miniconda3/envs/yolo/bin/python scripts/run_companion_yoloworld_smoke.py \
  --input-dir artifacts/companion-home-validation/runtime/pilot-new \
  --output-dir artifacts/companion-home-validation/runtime/pilot-new/yolo \
  --model /home/gxxl/soarm-car/artifacts/perception-dev/models/yolov8s-worldv2.pt
```

下一阶段 A1 的实际输入条件是：先确认相机 optical 轴与 `dynamic_pose` model-origin 定义，处理放置后的物理稳定性，再以冻结的可见实例标注确认真实关系锚点。不得把本小试扩展解释为 Nav2、SLAM、全场景 GT 或方法研究完成。
