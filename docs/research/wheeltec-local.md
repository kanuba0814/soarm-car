# WHEELTEC 本地资料研究索引

资料根目录：`/home/gxxl/Documents/wheeltec robot ROS资料/1.WHEELTEC ROS机器人通用资料/`

勘察时共约 44GB、1305 文件；包含教程视频、PDF、源码归档、镜像工具和二进制，不适合复制入 Git。本页仅记录对本项目有决策价值的证据。

## 核实结论

- ROS2 底盘节点 `wheeltec_robot_node` 订阅 `/cmd_vel`，通过 STM32 UART3 以 115200 通信，发布里程计、IMU 和电池电压。
- 串口上行帧 24 字节、下行 11 字节，帧头 `0x7B`、帧尾 `0x7D`、BCC 异或校验。
- 轮编码器到三轴底盘速度的运动学在 STM32 内完成；ROS 驱动读取速度并积分，不应在 Pi 端重复实现车型运动学。
- 设备别名约定包括 `/dev/wheeltec_controller`、`/dev/wheeltec_lidar`、`/dev/wheeltec_mic`。
- 树莓派5厂商系统支持 ROS1 Melodic 与 ROS2 Humble Docker 容器，ROS2 工作区说明为 `/home/wheeltec/wheeltec_ros2`。
- Humble 资料归档含 `turn_on_wheeltec_robot`、`wheeltec_robot_msg`、`serial_ros2`、Nav2、讯飞语音和 `ros2_astra_camera`。
- 深度相机资料覆盖 Orbbec Astra、Pro/Pro Plus、Dabai、Gemini 等；树莓派使用时资料曾建议降低深度分辨率，但具体值需实测。
- WHEELTEC 当前语音资料说明标配 M2 阵列，并提供 ROS1/ROS2 教程；是否就是现有硬件仍需确认。

## 关键证据

| 主题 | 相对根目录路径 | 位置/摘要 |
|---|---|---|
| 串口协议 | `2.运动底盘的控制与运动学解析/4.串口控制讲解/串口通信控制与反馈_2025-5-6.pdf` | P3–7 帧，P9/12/18+ 串口；SHA-256 `56b9ed2f...155c70` |
| ROS2↔STM32 | `6.ROS2系列教程/1.ROS2入门与编程开发教程/9.ROS2与STM32运动底盘通信-turn_on_wheeltec_robot功能包/ROS2-WHEELTEC机器人的ROS和STM32通信过程.pdf` | P7–15 节点、话题、串口；SHA-256 `5cf2c5bc...f8e3` |
| Nav2 | `6.ROS2系列教程/2.ROS2机器人应用视频教程：上手使用、雷达与2D导航/6.ROS2 2D导航/ROS2 2D导航功能.pdf` | wheeltec_nav2、AMCL、Smac、RPP；SHA-256 `62a30419...39ad9` |
| Pi 5 Docker | `9.ROS主控软硬件与镜像资料/树莓派5/树莓派5_Docker镜像使用说明_2024.12.31.pdf` | ROS1/ROS2 容器命令和工作区；SHA-256 `32036969...a587a` |
| Humble 源码 | `7.ROS源码/ROS2源码/Humble源码/JP6.2_常规ROS主控_src_250401.zip` | 2025-04 源码；SHA-256 `b12dd038...170e` |
| 实机同代源码 | `7.ROS源码/ROS2源码/Humble源码/JP5.1.1_常规ROS主控_humble_src_202409.zip` | 与备份目录/API对应；SHA-256 `7cd1abae...f21c` |
| 奥比中光旧驱动 | 上述 2024-09 归档的 `wheeltec_ros2/src/ros2_astra_camera/` | 实机同代 legacy OpenNI；备份 udev 为 `2bc5:0610` |
| 语音 | `6.ROS2系列教程/5.ROS2声源定位与语音导航专题/语音资料说明与链接.txt` | M2 当前标配，ROS2 教程入口 |

完整勘察报告保存在本次 Pi 会话的 subagent artifact 中，仓库以本页作为稳定摘要。

## 源码快照选择

`vendor/wheeltec/` 的底盘三包直接来自本地全盘备份，奥比中光包来自与备份同代的 2024-09 归档；底盘核心又与 2025-04 通用包逐字节一致。厂商 `turn_on_wheeltec_robot` 和 `wheeltec_robot_msg` 未声明明确许可证，因此仅内部兼容使用；参见 `vendor/wheeltec/SOURCE.md` 与 `docs/research/vendor-image.md`。
