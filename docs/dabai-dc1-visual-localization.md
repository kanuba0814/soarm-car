# Dabai DC1：驱动与 RGB-D bag 录制

## 已验证的启动路径

2026-09-13 实机：RGB `2bc5:0557`，深度 `2bc5:0657`，序列号 `CC1S74101GJ`。
使用本机已有 `soarm-car-astra-s-ros2-dev:latest` 镜像中的 OpenNI `astra_camera`，启动 `dabai_dc1.launch.xml`，显式指定 `uvc_product_id:=0x0557`。

原先 SDK v2.9.3 / `v2-main` 路径报 `Invalid descriptor index`、`Current found device(s): (0)`。USB 规则包含 PID 不代表 SDK 支持对应设备协议；官方 README 也明确旧 OpenNI 设备需另用 legacy 分支：
https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main
相同 DC1 PID 与错误另见 https://github.com/orbbec/OrbbecSDK_v2/issues/51 。

本次录制脚本直接复用已验证镜像，不使用 `compose.dabai-dc1.yaml` 或 `dabai_dc1_orbbec.launch.py`；这些文件保留供原 SDK/算法构建工作参考，不作为 DC1 的运行入口。无需重建 ORB-SLAM3 或手动填写会变化的 USB bus/device 编号。

## 录制

在本 worktree 中运行：

```bash
cd /home/gxxl/soarm-car/.claude/worktrees/dabai-dc1-visual-validation
./scripts/dabai-dc1-record-office.sh office-$(date +%Y%m%d-%H%M%S)
```

脚本自动启动相机、检查 10 秒真实数据，然后开始录制。缓慢移动，避免快速转动和运动模糊；回到起点后按 **Ctrl-C**，等待索引写入和自动校验结束。不要直接关闭终端。

固定时长测试（第二个参数为秒）：

```bash
./scripts/dabai-dc1-record-office.sh short-check-$(date +%Y%m%d-%H%M%S) 15
```

bag 存在 `recordings/<名称>/`；USB、驱动日志、镜像 ID、录制前检查和 bag 校验存在 `artifacts/perception-dev/dabai-dc1/<名称>/`。同名目录已存在时拒绝覆盖。原始数据约 43 MB/s，约 2.6 GB/min；每约 2 GiB 分卷。

单独启动、复查和停止：

```bash
./scripts/dabai-dc1-start.sh
./scripts/dabai-dc1-verify-bag.sh <bag名称>
./scripts/dabai-dc1-stop.sh
```

驱动容器为 `soarm-dabai-dc1-camera`，ROS_DOMAIN_ID 默认 `44`。可通过 `DC1_IMAGE`、`DC1_CONTAINER`、`DC1_ROS_DOMAIN_ID` 覆盖；所有终端须使用一致配置。插拔相机后先 stop，再 start。当前镜像是本机已有依赖；迁移机器前需导出该镜像，证据中记录其 SHA256 ID。

## 数据与检查范围

- RGB：640×480、rgb8、30 Hz；深度：640×400、16UC1、30 Hz，OpenNI 模式 `PIXEL_FORMAT_DEPTH_1_MM`（单位 mm）。640×480 深度模式经实测不支持，不要强设。
- 启用硬件 `depth_registration`，驱动报告两路 optical frame 和 K 相同；几何对齐精度尚未做标定板验证。保留两路原始尺寸和 CameraInfo，接入 ORB-SLAM3 前仍须验证对应像素区域并统一输入尺寸，不能直接拉伸深度图。
- 使用可靠传输录制 RGB、深度和 CameraInfo，以及 transient-local `/tf_static`。当前环境中尽力传输 RGB 实测只有约 4 Hz；可靠传输恢复约 30 Hz。
- 录制前与录制后检查真实图像消息、各自 CameraInfo、帧率至少 15 Hz（15–25 Hz 提示降速但允许录制，属于录制策略而非算法质量保证）、帧间断流不超过 0.5 秒、时间戳递增、非空深度、静态 TF，以及 RGB/深度最近帧时间差 p95 不大于 40 ms。时间配对是软件最近邻统计，不代表硬件同步。
- 不录制点云、机器人控制或导航话题。成功录制不等于 ORB-SLAM3 已通过测试；本次只处理驱动和录制链路。

原录制脚本只打印命令且预先创建 bag 目录，导致 rosbag 拒绝已有目录；现已改为实际执行录制，旁路保存证据，让 rosbag 自己创建输出目录。

## 实录验收结果

| bag | 停止方式 | RGB / 深度帧数 | RGB / 深度 Hz | 深度有效比例 | 最近帧差 p95 |
|---|---|---|---|---|---|
| `driver-smoke-20260913` | 15 秒自动停止 | 444 / 446 | 29.91 / 30.01 | 95.23% | 16.32 ms |
| `driver-ctrlc-20260913` | 约 23 秒，Ctrl-C | 689 / 691 | 29.91 / 30.01 | 95.15% | 16.03 ms |

两份 bag 均通过实际消息读取校验，包含两路 CameraInfo 和 4 个静态变换。
完整报告分别位于对应 `artifacts/perception-dev/dabai-dc1/<bag>/bag-check.json`。
