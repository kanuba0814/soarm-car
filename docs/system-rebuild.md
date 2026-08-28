# 树莓派 5 系统重建与旧驱动迁移

## 目标

建立可重现的干净系统，同时保留原厂系统作为兼容基准。不要覆盖唯一一张旧 TF 卡；至少准备两张介质：`vendor-baseline` 和 `clean-system`。

## A. 旧系统只读取证（重装前）

把输出保存到不在 TF 卡上的目录：

```bash
mkdir -p evidence/{system,ros,devices,config,samples}
uname -a > evidence/system/uname.txt
cat /etc/os-release > evidence/system/os-release.txt
dpkg-query -W > evidence/system/dpkg.txt
python3 --version > evidence/system/python.txt 2>&1
docker image inspect $(docker image ls -q) > evidence/system/docker-images.json 2>/dev/null || true
docker ps -a --no-trunc > evidence/system/docker-containers.txt 2>/dev/null || true
lsusb -v > evidence/devices/lsusb-v.txt
udevadm info --export-db > evidence/devices/udev-db.txt
ls -l /dev/wheeltec* > evidence/devices/wheeltec-devices.txt 2>&1
```

进入厂商 ROS2 容器后：

```bash
printenv | sort > evidence/ros/environment.txt
ros2 doctor --report > evidence/ros/doctor.txt
ros2 pkg list > evidence/ros/packages.txt
ros2 node list > evidence/ros/nodes.txt
ros2 topic list -t > evidence/ros/topics.txt
ros2 service list -t > evidence/ros/services.txt
ros2 action list -t > evidence/ros/actions.txt
ros2 param dump /wheeltec_robot > evidence/config/wheeltec_robot.yaml
ros2 run tf2_tools view_frames
cp frames.pdf evidence/ros/
```

在车轮架空、有人看守的条件下记录短样本：

```bash
ros2 bag record -o evidence/samples/base-idle /odom /imu /PowerVoltage /tf /tf_static
```

话题名以 `ros2 node info /wheeltec_robot` 的实机输出为准。另行备份：

- `/home/wheeltec/wheeltec_ros2/src/`
- `/etc/udev/rules.d/`
- Docker compose/run 参数、容器 volume 和 device 映射
- `.bashrc`、systemd 服务、网络配置
- 相机、雷达、IMU、Nav2 和 URDF 参数
- SO-ARM101 未来生成的校准文件

## B. 原厂镜像保全

已找到本地全盘备份：`/home/gxxl/Downloads/mmcfullback.img.gz`。它是 62.53GB 的树莓派5完整卡镜像，gzip 完整性通过，SHA-256 为 `c05c5775c96b99fc87d21e82f13a56ddc8201fec5769147dcb2e2a2de1f703db`。详见 `docs/research/vendor-image.md`。

该备份已经包含 `ros2:wheeltec` 原厂容器，因此现阶段无需再下载身份不明的公开 R550 镜像。保全要求：

1. 原始 `.img.gz` 保持只读，不纳入 Git；最好再复制到另一块物理磁盘。
2. 第二份副本复制完成后重新计算 SHA-256，必须一致。
3. 只烧录到备用卡，不覆盖旧卡。
4. 根分区显示 `needs_recovery`；在副本/备用卡上离线检查文件系统，绝不直接修唯一原件。
5. 镜像包含 `.ssh`、网络、浏览器及历史记录等敏感数据，不公开上传。

公开官网的 R550 ROS2 链接未注明树莓派5/Docker，仍不能作为本备份的替代来源。

## C. 干净系统基线

首版坚持 ROS 2 Humble，以减少 WHEELTEC 驱动迁移变量。两种候选必须实测后择一：

- 厂商树莓派 OS 宿主 + ROS2 Humble 容器：最接近原系统。
- Ubuntu 22.04 arm64 + 原生 ROS2 Humble：依赖更透明，但需验证 Pi 5、相机和厂商二进制库。

不要在同一首版同时把系统迁到 Ubuntu 24.04/Jazzy；Jazzy 迁移另开阶段。

## D. 最小底盘构建

仓库已从实机备份保存：

- `vendor/wheeltec/src/turn_on_wheeltec_robot`
- `vendor/wheeltec/src/wheeltec_robot_msg`
- `vendor/wheeltec/src/serial_ros2`（ROS package 名为 `serial`）
- `vendor/wheeltec/src/ros2_astra_camera`（与实机同代的 legacy Orbbec 驱动）
- `vendor/wheeltec/udev/`（底盘与奥比中光规则）

构建目标：

```bash
source /opt/ros/humble/setup.bash
cd vendor/wheeltec
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select serial wheeltec_robot_msg turn_on_wheeltec_robot
```

厂商清单包含一些并非底盘串口节点实际需要的包；首次构建失败时先检查 CMake/manifest 过度依赖，不要复制整个 759MB 工作区来“解决”。

## E. 台架启动顺序

1. 底盘离地，电机使能关闭，急停可触达。
2. 安装并检查 udev，确认 `/dev/wheeltec_controller` 指向正确 VID/PID/序列号。
3. 只启动 `base_serial.launch.py`，先观察数据，不发速度。
4. 校验 `/odom`、`/imu`、电压的单位、方向、频率和时间戳。
5. 开启使能，以很小的短时速度分别测正反与旋转；每次自动回零。
6. 再接速度仲裁和 watchdog，最后才启动 Nav2。

## F. 奥比中光识别与驱动分支

在旧系统和新系统分别保存：

```bash
lsusb -nn | grep -i '2bc5\|orbbec'
udevadm info --query=all --name=/dev/video0
v4l2-ctl --list-devices
```

决策：

- 老 Astra/Pro/Pro Plus/Dabai/OpenNI 设备：先复用 WHEELTEC 归档中的 `ros2_astra_camera` 行为，再对照奥比中光官方 `main` 分支。
- Astra 2、Gemini 2/330 等当前设备：优先官方 `OrbbecSDK_ROS2` 的 `v2-main`。
- 不凭外观决定分支；以铭牌、USB PID 和官方支持表为准。
- 树莓派5未列在官方已测试平台；从低分辨率/低帧率开始，记录 USB 拓扑、CPU、温度和掉帧。

## 回退

任一迁移检查不通过，立即切回原厂基准卡；不要边修新系统边修改唯一旧系统。所有 vendor 源码改动必须以补丁和新哈希记录。
