# 上游官方资料研究

访问日期：2026-08-28。关键页面的小型快照位于 `docs/research/sources/`，并有 SHA-256；大型代码仓库和镜像不纳入 Git。

## SO-ARM101 / LeRobot

- LeRobot 官方 SO-101 文档给出 follower 电机设置与校准命令。
- follower 使用 6 个 Feetech STS3215 串行舵机；每个电机需要唯一 ID，控制板/电机 baudrate 必须一致。
- 官方接口可直接配置、校准和编程 follower；但这不等于已有完整 ROS2/MoveIt follower-only 产品栈。本项目需要受限 ROS2 bridge、关节限位和 watchdog。
- LeRobot 仓库为 Apache-2.0；舵机、控制板和 CAD 的许可证/电源规格需分别核实。

来源：<https://huggingface.co/docs/lerobot/en/so101>、<https://github.com/huggingface/lerobot>

## ROS 2 / Nav2 / MoveIt 2

- ROS 官方对 Pi 的当前 Tier-1 路径是 arm64 Ubuntu；Jazzy 对应 Ubuntu 24.04。
- MoveIt 官方二进制支持 Humble/Ubuntu 22.04，也支持 Jazzy/Ubuntu 24.04。
- 本项目首版不追求最新，选择 Humble 是为了与已获得的 WHEELTEC 驱动保持同一发行版；Jazzy 只作为后续迁移候选。
- Nav2 负责定位、costmap、规划、控制与行为树；底盘 driver 仍只暴露速度/里程计边界。

来源：ROS 2 Raspberry Pi 指南、Nav2 Jazzy 文档、MoveIt 2 Binary Install（均已保存快照）。

## 奥比中光（优先深度相机）

奥比中光官方 `OrbbecSDK_ROS2`：

- 支持 ROS2 Foxy/Humble/Jazzy。
- 默认 `v2-main`；legacy OpenNI 设备使用 `main`。
- 支持表中 Astra 2、Astra Mini Pro/S Pro、Gemini 2/210/215/301/330/340 等新设备优先 v2；Astra Pro Plus 等旧设备不受 v2 支持或仅在旧分支维护。
- ARM64 SDK 库存在，但官方测试平台资料没有把 Raspberry Pi 5 列为保证平台；必须实机测试分辨率、编码格式、USB 和帧率。
- 官方仓库约 807MB，本规划阶段只保存 README、许可证和设备支持页，不克隆整库。

来源：<https://github.com/orbbec/OrbbecSDK_ROS2>、<https://orbbec.github.io/OrbbecSDK_ROS2/>

### 本项目的驱动选择规则

1. 用铭牌和 `lsusb -nn` 获得型号、VID/PID；Orbbec 常见 VID 为 `2bc5`。
2. 老 Astra/Pro/Pro Plus/Dabai：先复现 WHEELTEC 的 `ros2_astra_camera`，再对照官方 `main`。
3. Astra 2/Gemini 新系列：优先官方 `v2-main`。
4. 先 RGB/depth/camera_info，再开点云、对齐和高分辨率；不能一开始把所有 stream 拉满。

## Model Hardware Standard（MHS）

Anthropic 与 HHMI Janelia 正在研究预览 MHS：通过标准 driver、设备发现、自然语言 metadata/reference file，以及 MCP、CLI、code-file/API 三种控制面，让 agent 操作可编程物理设备。官方称 Hugging Face 正为 LeRobot 添加支持，Raspberry Pi 也在 Camera MHS Driver 测试后推进产品集成。

当前 MHS 仍是申请制预览，规范、SDK、许可证、schema、鉴权和 ROS 2 集成均未公开。本项目现在按公开架构实现 `provisional-mhs/v0`：MHS 能力映射到白名单 ROS 2 action/service，实时控制、安全限位、急停和 watchdog 不上移到 agent 层；未来以 official adapter 替换临时协议。详见 [`mhs.md`](mhs.md) 和 [`../mhs-implementation.md`](../mhs-implementation.md)。

来源：Anthropic MHS 公告、MHS 官方预览站、MCP 2025-06-18 规范（均已保存快照）。

## 设计建议（非上游承诺）

- Pi、底盘和机械臂分路供电，共地并按实测峰值选 DC/DC/保险；机械臂不能从 Pi 供电。
- 语音链采用离线 ASR → 白名单意图 → 状态机 → ROS action/service；硬件急停和 watchdog 独立存在。
- 抓取链采用 RGB-D 位姿 → planning scene → MoveIt IK/碰撞 → 受限执行，首版底盘停稳后才可伸臂。
