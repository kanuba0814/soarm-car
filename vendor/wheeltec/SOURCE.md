# WHEELTEC vendor snapshot

本目录是为新树莓派 5 系统保留的实机 ROS 2 Humble 底盘与奥比中光驱动快照，不是本项目原创代码。

## 权威来源

全盘备份：

- 文件：`/home/gxxl/Downloads/mmcfullback.img.gz`（不纳入 Git）
- 压缩文件 SHA-256：`c05c5775c96b99fc87d21e82f13a56ddc8201fec5769147dcb2e2a2de1f703db`
- 压缩/解压大小：19,650,522,808 / 62,534,975,488 bytes
- 镜像内容：Raspberry Pi OS 64-bit / Debian 12 Bookworm 宿主，Docker 内为 Ubuntu 22.04 arm64 + ROS 2 Humble。

从该镜像只读导出：

- `/home/wheeltec/wheeltec_ros2/src/turn_on_wheeltec_robot`
- `/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_msg`
- `/home/wheeltec/wheeltec_ros2/src/depend/serial_ros2`
- `/etc/udev/rules.d/{wheeltec_controller,wheeltec_gemini,56-orbbec-usb}.rules`

实机镜像内存在 `ros2_astra_camera`，其 launch API 与以下厂商归档完全对应，因此从归档提取，避免长期保留 62GB 分析副本：

- 归档：`JP5.1.1_常规ROS主控_humble_src_202409.zip`
- 路径：`/home/gxxl/Documents/wheeltec robot ROS资料/1.WHEELTEC ROS机器人通用资料/7.ROS源码/ROS2源码/Humble源码/JP5.1.1_常规ROS主控_humble_src_202409.zip`
- SHA-256：`7cd1abae9345986ff362278c150dcf28aa43aa08aab0f2f984b3e65be584f21c`
- 提取：`wheeltec_ros2/src/ros2_astra_camera`

底盘核心 `wheeltec_robot.cpp` 还与 2025-04 的 `JP6.2_常规ROS主控_src_250401.zip` 逐字节一致，SHA-256 为 `2e4826c77e0269f9f7cb46fab6ca3b82c595e93aa9cae70f5c3cdb8d46e8bea9`。

## 为什么保留这些包

`turn_on_wheeltec_robot` 实现 STM32 串口、`/cmd_vel`、里程计、IMU 和电池电压；它直接依赖 `wheeltec_robot_msg` 和 ROS package 名为 `serial` 的 `serial_ros2`。`ros2_astra_camera` 是备份系统实际配套的 legacy OpenNI 奥比中光驱动。导航、SLAM、UI 和大型第三方算法不属于最小硬件恢复闭环，暂不复制。

## 许可证边界

- `serial_ros2/package.xml` 声明 Apache License 2.0；保留其上游 README。
- `ros2_astra_camera` README 声明 Apache-2.0，但其内部二进制 OpenNI 库还需遵守随附许可证/厂商条款。
- `turn_on_wheeltec_robot` 与 `wheeltec_robot_msg` 的 `package.xml` 均为 `TODO: License declaration`，资料中未发现许可证文件。
- 在 WHEELTEC 明确授权前，这两包只作为合法获得设备资料的内部兼容快照使用，不应公开镜像、发布二进制或对外再分发。
- 本项目自己的文档和未来原创代码应与 vendor 目录分开许可。

## 修改规则

先保持快照不变。树莓派 5 兼容改动优先放在项目 overlay 包；确实必须修改 vendor 时，以独立补丁记录原因，并重新生成 `SHA256SUMS`。
