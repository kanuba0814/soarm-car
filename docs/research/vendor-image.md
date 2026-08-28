# WHEELTEC 树莓派5全盘备份勘察

检查日期：2026-08-28。所有文件系统检查均为只读；未运行 journal recovery，未修改原始备份。

## 文件身份

- 路径：`/home/gxxl/Downloads/mmcfullback.img.gz`
- 压缩大小：19,650,522,808 bytes
- 解压大小：62,534,975,488 bytes（约 58.24 GiB）
- SHA-256：`c05c5775c96b99fc87d21e82f13a56ddc8201fec5769147dcb2e2a2de1f703db`
- `gzip -t`：通过，CRC/数据流完整。
- 出生/修改时间：2026-08-28 10:37 / 11:06（本机时区）。

这不是未使用的纯出厂母盘：系统基础生成于 2024-03，启动文件与容器在 2024-12 前后更新，根文件系统中存在 2025-09 的使用记录。它是完整的原厂系统实机备份，适合作为恢复基准和迁移权威来源。

## 分区与宿主系统

MBR ID：`0xab97d749`

| 分区 | 起始扇区 | 大小 | 文件系统 | 标签 |
|---|---:|---:|---|---|
| 1 | 8192 | 512 MiB | FAT32 | `bootfs` |
| 2 | 1056768 | 57.7 GiB | ext4 | `rootfs` |

启动分区包含 `bcm2712-rpi-5-b.dtb`、`kernel_2712.img`，64-bit 模式开启，明确支持 Raspberry Pi 5。

宿主：

- Raspberry Pi OS / Debian GNU/Linux 12 Bookworm
- 主机名：`wheeltec`
- 内核/启动基线来自 pi-gen 2024-03-15
- Docker 数据完整存在于 `/var/lib/docker`

ext4 标有 `needs_recovery`，说明备份时文件系统可能处于挂载状态；gzip 本身完整。恢复到备用卡后应先离线运行文件系统检查，不应对唯一原件直接修复。

## Docker/ROS

镜像仓库：

- `ros2:wheeltec` → `sha256:fe02032d9692bf5e7a805dfb34a455a528f7a9096e48284379c9b0f3b69e82f5`
- `ros1:wheeltec`
- `ros_melodic:wheeltec`
- 上游 `ros:melodic-ros-core`

ROS2 镜像元数据：

- Linux arm64
- Ubuntu 22.04 base
- `ROS_DISTRO=humble`
- 创建时间：2024-05-21
- 容器名/镜像名：`ros2` / `ros2:wheeltec`
- 以 `--privileged --net=host` 运行并把宿主 `/dev` 映射进容器。

因此本备份已经包含需要的原厂容器镜像，不必再依赖身份不明的公网 R550 下载链接。

## 实机 ROS2 工作区

路径：`/home/wheeltec/wheeltec_ros2`

已从备份只读导出并纳入 `vendor/wheeltec/`：

- `turn_on_wheeltec_robot`
- `wheeltec_robot_msg`
- `serial_ros2`
- 与实机同代的 `ros2_astra_camera`
- 底盘与奥比中光 udev 规则

底盘核心驱动事实：

- `/dev/wheeltec_controller`
- USB `10c4:ea60`，序列号 `0002`
- 115200 baud
- `/cmd_vel` 输入、`PowerVoltage` 输出；里程计/IMU 见源码
- 实机 `wheeltec_robot.cpp` 与厂商 2025-04通用 Humble 版本哈希一致。

## 奥比中光

实机规则：

```text
KERNEL=="video*", ATTRS{idVendor}=="2bc5", ATTRS{idProduct}=="0610",
ATTRS{index}=="1", SYMLINK+="Astra_Gemini"
```

因此系统至少针对 Orbbec `2bc5:0610` 配置，设备别名为 `/dev/Astra_Gemini`。这比通用 launch 的默认 `Astra_S` 更有硬件识别价值，但最终型号仍应以相机铭牌和实机 `lsusb -nn` 确认。

备份中的相机 launch 是 legacy OpenNI 风格，提供 `Astra_S`、`Astra_Pro`、`Dabai`、`Gemini`；与 `JP5.1.1_常规ROS主控_humble_src_202409.zip` 的 `ros2_astra_camera` 对应。

## 不应纳入 Git 的敏感内容

备份包含用户 home、`.ssh`、网络配置、浏览器/远程桌面缓存、shell history 等。原始镜像必须留在受控本地存储；只把经过筛选的软件包、非秘密 udev 和不可变哈希写入项目仓库。
