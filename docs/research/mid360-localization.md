# Livox MID-360 定位资产勘察

检查日期：2026-08-28。原始资产只读检查，未修改 ZIP。

## 原始工作区

- 文件：`temple/localization_ws_portale.zip`
- 大小：374,854,806 bytes
- SHA-256：`b1697dfff70d5915a959f557f93ef09d38ed1b1acedbc640ff9044a165db23a0`
- 完整性：229 entries，`unzip -t` 全部通过
- ROS：Humble
- 包：`fast_lio_localization`
- 算法上游：`myeongw002/FAST_LIO_LOCALIZATION_ROS2` commit `3a005b9`

归档内含源码、嵌套 Git 历史和 2026-08-15 的 colcon 构建日志。日志证明该包曾在 Ubuntu 22.04、ROS 2 Humble、x86_64 上成功编译，但没有 Pi 5 arm64 构建、MID-360 实机运行、定位精度或长稳证据。

原 ZIP 约 358MB，主要体积来自 Git pack、论文、GIF/图片和日志。可运行最小快照已提取到 `vendor/localization/`，原 ZIP 由 `.gitignore` 排除。

## 确认的数据入口

用户确认依赖 Livox 的 `msg_MID360_launch.py`。官方文件固定于 `livox_ros_driver2` commit `4a1def9`：

```text
xfer_format = 1
multi_topic = 0
publish_freq = 10 Hz
frame_id = livox_frame
```

因此输入契约为：

```text
/livox/lidar : livox_ros_driver2/msg/CustomMsg
/livox/imu   : sensor_msgs/msg/Imu
```

FAST-LIO 对应配置：

```text
config/mid360.yaml
lidar_type = 1
lid_topic = /livox/lidar
imu_topic = /livox/imu
```

代码证据：`laserMapping.cpp` 在 `lidar_type == 1` 时订阅 `livox_ros_driver2/msg/CustomMsg`；其他类型订阅 `sensor_msgs/msg/PointCloud2`。MID-360 的 PointCloud2 handler 编号为 4，但只有在 Livox launch 同时使用 `xfer_format = 0` 时才能使用。

## 算法链

```text
msg_MID360_launch
  -> Livox CustomMsg + IMU
  -> FAST-LIO
  -> /Odometry + /cloud_registered
  -> global map publisher
  -> ICP global localization
  -> /map_to_odom
  -> transform fusion
  -> /localization
  -> Nav2
```

`mapping.launch.py` 默认只运行 FAST-LIO 和 RViz，不是完整的全局定位启动。`velodyne_localization.launch.py` 包含 ICP、map publisher 和 fusion，但名字及默认配置来自示例，必须制作项目自己的 MID-360 bringup。

## 收到的本地改动

相对嵌入上游 HEAD，工作树主要改动：

- `velodyne_localization.launch.py`；
- `velodyne_test.yaml` 的地图路径和 Livox 参数；
- `transform_fusion.py` 增加 Autoware pose/twist with covariance 输出；
- 新增 `awsim.yaml`；
- ikd-Tree submodule状态变化。

MID-360 基础 `mid360.yaml` 没有本地未提交差异，不能仅凭文件存在称为“本车已调外参”。

## 必须补齐

- 真正用于场地的 PCD 地图；ZIP 内没有地图。
- MID-360 固件、IP、主机网卡配置和时间同步。
- `base_link -> livox_frame` 实机外参。
- `camera_init/body` 到项目标准 TF 的转换。
- Pi 5 arm64 编译与资源测量。
- 实机重定位、漂移、协方差、退化场景、网络断开和重启测试。
- Nav2 消费的定位/TF 契约。

## 产品任务关系

`go_to_place_and_grasp` 在进入导航阶段前必须检查：

- `/livox/lidar` 和 `/livox/imu` 数据新鲜；
- PCD 地图加载成功；
- 全局定位有效且协方差在实测阈值内；
- TF 连续；
- 命名抓取地点存在于同一地图版本。

定位丢失或数据过期必须取消导航并停止底盘，不能依赖 Agent 判断。

## 风险

- Livox 官方 README 明确称 `livox_ros_driver2` 是调试工具，不建议原样用于量产；需针对本产品验证和加固。
- 定位包根许可证为 GPL-2.0，但 `package.xml` 标 BSD、README 标 MIT，存在许可证冲突。当前按内部、限制再分发的 vendor snapshot 管理。
- 示例 fusion 代码使用固定协方差，不是实测估计；不能直接作为导航健康判据。
