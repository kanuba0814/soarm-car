# Livox MID-360 定位 vendor 快照

本目录保存“到指定地点抓取”任务中导航定位前半段的可复现源码基线，不是本项目原创代码。

## 原始调试归档

- 文件：`temple/localization_ws_portale.zip`（Git 忽略，保留原件）
- 大小：374,854,806 bytes
- SHA-256：`b1697dfff70d5915a959f557f93ef09d38ed1b1acedbc640ff9044a165db23a0`
- ZIP 完整性：229 entries，全量 `unzip -t` 通过
- 工作区：ROS 2 Humble，包 `fast_lio_localization`
- 已有构建证据：2026-08-15 在 Ubuntu 22.04/Humble x86_64 上 colcon build 成功；这不是 Raspberry Pi 5 arm64 或实机运行证明。

算法仓库的嵌入 Git 指向：

- URL：`https://github.com/myeongw002/FAST_LIO_LOCALIZATION_ROS2.git`
- branch：`main`
- HEAD：`3a005b965c566245c33a624b60a31235e77d3c44`

归档中的工作树相对该提交还有未提交修改，主要涉及全局定位 launch、地图路径和 Autoware 位姿/速度桥。这些修改已作为收到的 vendor snapshot 保留。

## Livox 数据入口

用户确认定位依赖 `msg_MID360_launch.py`。本目录固定官方 `livox_ros_driver2`：

- URL：`https://github.com/Livox-SDK/livox_ros_driver2.git`
- branch：`master`
- commit：`4a1def929e5b59c7a8122d19fce6efba581ce9f7`
- license：MIT（见 `src/livox_ros_driver2/LICENSE.txt`）

输入契约：

```text
msg_MID360_launch.py
  xfer_format = 1
  multi_topic = 0
  publish_freq = 10 Hz

/livox/lidar : livox_ros_driver2/msg/CustomMsg
/livox/imu   : sensor_msgs/msg/Imu
```

对应 FAST-LIO 配置必须使用：

```text
config/mid360.yaml
preprocess.lidar_type = 1
common.lid_topic = /livox/lidar
common.imu_topic = /livox/imu
```

`lidar_type = 4` 适用于标准 `sensor_msgs/PointCloud2` 路径，不能与官方 `msg_MID360_launch.py` 的默认 `xfer_format = 1` 混用。

## 已提取内容

- `src/FAST_LIO_LOCALIZATION_ROS2`：构建和运行所需源码、配置、launch、脚本、消息及编译依赖；排除嵌套 Git、论文/演示媒体、构建日志、缓存和大日志。
- `src/livox_ros_driver2`：官方固定提交的完整小型源码快照；额外生成 `package.xml` 作为 `package_ROS2.xml` 的副本，便于普通 colcon 工作区识别。
- `SHA256SUMS`：本快照全部文件校验值。

## 已知缺口

1. 原始 ZIP 没有真实 PCD 地图；`PCD/1` 只是 2-byte 占位文件，未提取。
2. `mapping.launch.py` 仅启动 FAST-LIO 里程计/建图；全局 PCD 定位还需 global map publisher、ICP、transform fusion 和初始位姿。
3. 完整全局定位 launch 当前以 `velodyne_localization.launch.py` 命名并默认加载示例参数，需要制作项目自己的 MID-360 bringup，不能直接照搬默认值。
4. 帧仍使用 `camera_init`、`body`；必须实测并定义 `map -> odom -> base_footprint -> base_link -> livox_frame`，不能直接把 `body` 当作底盘 `base_link`。
5. MID-360 外参 `[ -0.011, -0.02329, 0.04412 ]` 来自收到的上游配置，但没有证据表明它是本车安装外参；必须重新标定。
6. Livox 官方 README 明确称 driver 是调试工具、未建议直接量产；产品化前需验证重连、时间戳、长稳、资源和故障行为。
7. 算法仓库许可证标注不一致：根 `LICENSE` 为 GPL-2.0，而 `package.xml` 写 BSD、README 写 MIT。按 GPL-2.0/待澄清的保守策略管理，不对外发布二进制或源码集合，直到完成许可证核查。

## 与产品任务的 seam

定位模块向 Nav2/MHS mission 提供经过验证的新鲜定位状态，而不是直接接受 Agent 控制：

- FAST-LIO：`/Odometry`、`/cloud_registered`
- ICP correction：`/map_to_odom`
- fused pose：`/localization`
- TF：最终需规范为 `map -> odom -> base_footprint -> base_link`

`go_to_place_and_grasp` 只有在 Livox 数据、融合位姿、TF 和地图均健康且新鲜时才允许进入 `NAVIGATING`。
