# Orbbec Astra S：ROS 2 Humble 与 Gazebo 接口

核对日期：2026-08-31。实体设备为 `2bc5:0402`、USB product `ASTRA S`，在 Fedora 44 宿主上以 USB 2.0 480 Mb/s 枚举。

## 驱动选择

Astra S 属于 OpenNI 代设备。本项目使用 Orbbec 官方 [`ros2_astra_camera`](https://github.com/orbbec/ros2_astra_camera)，而不是面向新设备的 OrbbecSDK v2 wrapper。实时检查表明该仓库默认且唯一分支是 `master`，不是 `main`；镜像固定提交 `f7e71d9ce806e788cb48d8580aac2c778fba4214`。官方仓库包含 OpenNI2 的 x64/arm/arm64 runtime、`astra.launch.xml`、udev 规则、CameraInfo 服务和点云实现。

官方 README 要求 gflags、glog、libusb、Eigen、image_geometry、camera_info_manager、image_transport、image_publisher，并要求源码安装 libuvc。本项目固定 libuvc `v0.0.7` 对应提交 `68d07a00e11d1944e27b7295ee69673239c00b4b`，全部在 `docker/Dockerfile` 中构建。

来源：

- <https://github.com/orbbec/ros2_astra_camera>
- <https://github.com/orbbec/ros2_astra_camera/blob/master/README.MD>
- <https://github.com/orbbec/ros2_astra_camera/blob/master/astra_camera/launch/astra.launch.xml>
- <https://github.com/libuvc/libuvc/tree/v0.0.7>

## USB 与 ROS 契约

官方/仓库保留的 legacy `56-orbbec-usb.rules` 对 `2bc5:0402` 设置 `astra_s`，但它还对大量 Orbbec 型号授予 `0666`。本项目的 `scripts/install-astra-udev.sh` 只匹配 Astra S，设置 `0660`、`video` group 和 `uaccess`；`scripts/astra-usb-device.sh` 从 sysfs 定位当前 bus/device number，`compose.astra.yaml` 只映射该 Astra S 字符设备，不映射整棵 USB bus，也不使用 `--privileged`。

硬件和仿真的算法公共子集为：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw       # 16UC1，毫米
/camera/depth/camera_info
/camera/depth/points          # PointCloud2 XYZ
```

实体驱动额外提供 IR、设备服务、extrinsics 和可选彩色点云；这些不属于无硬件自动测试的最低契约。frame 使用 `camera_link`、`camera_color_optical_frame` 和 `camera_depth_optical_frame`。

## Gazebo 选择

ROS 2 Humble 的 Gazebo Classic 插件可以直接配置 ROS camera/depth topics，但 Gazebo Classic 已是退役技术，并且本仓库已建立 Gazebo Fortress + `ros_gz` 的底盘、里程计、IMU 和 LiDAR 验证。为了避免维护两套物理引擎，本项目保留 Fortress：

1. SDF `rgbd_camera` 生成 RGB、depth 和 CameraInfo；
2. `ros_gz_bridge` 将原始消息放入 `/simulation/astra/*`；
3. `soarm_astra/sim_adapter.py` 统一 topic、frame 和 depth `16UC1` 编码；
4. ROS 官方 `depth_image_proc::PointCloudXyzNode` 从 depth + CameraInfo 生成 `/camera/depth/points`，避开 Humble `ros_gz_point_cloud` 的已知限制。

来源：

- <https://docs.ros.org/en/humble/p/ros_gz_sim_demos/>
- <https://gazebosim.org/docs/latest/ros2_integration/>
- <https://docs.ros.org/en/humble/p/depth_image_proc/>
- <https://docs.ros.org/en/humble/p/gazebo_ros_pkgs/>

## 2026-08-31 实机结果

官方驱动成功识别 `2bc5:0402`，RGB 640×480@30、IR 640×480@30（与 RGB 分别启动）和 depth 640×480@30 均能启动；IR 每帧 153,600 像素均有数据，RGB 也有有效 payload。可是 depth 连续多帧全部为零，因而点云没有有限 XYZ。显式启用 laser、关闭 RGB/IR、降到 depth-only 320×240@30 后仍全零；LDP 控制在该设备上返回不支持。

标定接口也未给出可用的出厂畸变数据：`/camera/get_camera_params` 返回的左右内参、畸变和外参全部为 NaN；RGB/depth `CameraInfo` 则退化为相同的通用 K/P，`plumb_bob` 的五个 D 系数全零。这些消息可用于记录接口，但不能证明出厂畸变标定已加载，也不应直接作为精确 RGB-D 配准或测量标定。

这把问题缩小到现场场景/遮挡、投射器、设备标定或深度硬件，而不是 Docker USB、OpenNI 枚举或 ROS topic。修复物理条件前，`scripts/dev-astra-smoke.sh` 应保持失败，不能把空 depth 报告为硬件通过。检查相机前方约 0.6–3 m 的漫反射物体、移除保护膜/镜头与投射器遮挡，并确认 IR 投射图案后重跑。

## 仿真边界

当前模型固定 640×480、30 Hz、约 60° 水平视场、0.1–8 m 范围，但安装位姿、内参、畸变、噪声和失效模式尚未从实体设备标定。Gazebo backend 适合接口回归和算法 smoke test，不代表 Astra S 光学、USB 带宽、曝光、遮挡、IR 干扰或断线恢复的硬件验证。后续应从实机保存 CameraInfo、支持模式和 rosbag，再收紧仿真参数。
