# soarm-car

以树莓派 5 为主控的 Kuruma 四轮麦克纳姆底盘项目。

> 当前阶段：Kuruma chassis-first。活动模型来自 `artifacts/kuruma.STEP`，默认描述/仿真/导航不安装机械臂，不能报告抓取或任务成功。机械臂安装尺寸和位置是下一项机械 study；传感器安装仍是 provisional。旧 WHEELTEC 底盘状态保留在 `backup/wheeltec-chassis`。

## 已确定

- 主控：树莓派 5。
- 底盘：`kuruma.STEP` 派生四轮麦克纳姆。外滚子包络半径 `0.0763 m`、宽 `0.0481 m`、轴距 `0.3472 m`、轮距 `0.4456 m`、3030 外框 `0.600 m` 方形。
- 机械臂：当前未安装且默认模型不包含臂；下一阶段研究安装孔位、位置和重心。
- 感知：已有深度相机；定位雷达确定为 Livox MID-360，已有 FAST-LIO Localization 调试工作区和官方 Livox driver 快照。
- 产品任务：`到指定地点抓取指定物体`；首版地点和物体均使用白名单。
- 交互：离线语音、Agent 和操作界面调用同一个 provisional MHS 任务。
- 系统：重建树莓派系统，不把旧厂商整机镜像当长期依赖；先迁出底盘最小驱动。
- ROS 基线：首版采用 ROS 2 Humble，以兼容 WHEELTEC 已有源码。Jazzy 作为后续迁移候选，不在首版混用。

## 仓库内容

- [`docs/product-goal.md`](product-goal.md)：唯一首版任务、成功定义、状态机和验收标准。
- [`docs/architecture.md`](architecture.md)：围绕端到端抓取任务的系统边界、ROS 节点与安全链路。
- [`docs/plan.md`](plan.md)：按风险排序的实施阶段和验收条件。
- [`docs/system-rebuild.md`](system-rebuild.md)：旧系统取证、新镜像重建和回退步骤。
- [`docs/hardware-checklist.md`](hardware-checklist.md)：装臂前必须测量/确认的机械、电气项目。
- [`docs/unknowns.md`](unknowns.md)：会改变设计的未决项。
- [`docs/research/wheeltec-local.md`](research/wheeltec-local.md)：44GB WHEELTEC 本地资料的证据索引。
- [`docs/research/vendor-image.md`](research/vendor-image.md)：本地树莓派5全盘备份、容器和实机配置勘察。
- [`docs/research/mid360-localization.md`](research/mid360-localization.md)：MID-360 数据入口、FAST-LIO 定位资产和缺口。
- [`docs/research/upstream.md`](research/upstream.md)：SO-ARM101、ROS 2、Nav2、MoveIt 2 等官方资料结论。
- [`docs/research/mhs.md`](research/mhs.md)：Anthropic MHS 预览、MCP 关系与公开未知项。
- [`docs/mhs-implementation.md`](mhs-implementation.md)：当前 provisional MHS 实现与未来官方替换设计。
- [`docs/upstream-manifest.yaml`](upstream-manifest.yaml)：上游来源与版本固定入口。
- [`vendor/wheeltec/`](../vendor/wheeltec/)：从实机备份提取的底盘驱动、udev，以及匹配的奥比中光驱动快照。
- [`vendor/localization/`](../vendor/localization/)：MID-360 的 `msg_MID360_launch` 数据入口与 FAST-LIO Localization 最小源码快照。
- [`config/mission-policy.yaml`](../config/mission-policy.yaml)：指定地点抓取任务的阶段、成功条件与安全策略。
- [`config/voice-policy.yaml`](../config/voice-policy.yaml)：只触发完整产品任务或停止的语音白名单。

## 产品闭环

```text
接收 place_id + object_id
  -> 导航到命名抓取点
  -> 底盘停稳并保持
  -> 识别物体并估计位姿
  -> MoveIt 规划和碰撞检查
  -> SO-ARM101 抓取并抬升
  -> 验证持物
  -> 报告成功或安全失败
```

实施仍按底盘、传感器、机械臂、抓取、MHS 任务编排逐层验收；每层必须可独立停止和回退，但所有阶段最终都服务于上述唯一闭环。抓取后配送、返回或放置到第二地点不属于首版必达范围。

## 本地 ROS 2 Humble 可视化开发环境

当前阶段以检视为主，不做 Nav2、MoveIt、BehaviorTree 或硬件任务编排。容器提供标准 ROS 2 Humble Desktop、RViz2、robot_state_publisher、MarkerArray 和 Foxglove Bridge；薄 Action 端点只把 Agent 请求显示出来，明确返回“未执行硬件”，不伪装导航或抓取成功。

宿主需要 Linux X11/XWayland 和 `/dev/dri`。首次构建后先确认容器使用真实 GPU，而非 llvmpipe：

```bash
./scripts/dev-build.sh
./scripts/dev-gpu-check.sh
./scripts/dev-start.sh
```

`dev-start.sh` 默认同时启动 RViz2 和 Foxglove Bridge：

- RViz2：显示 Grid、TF、RobotModel、`/development/scene`；
- Foxglove：连接 `ws://localhost:8765`；
- 当前 Intel ARL 开发机：复用已验证的 `local/arc140t-focal-dri:mesa-25.1.7` 图形运行时，并透传 `/dev/dri`；
- video/render GID 从宿主自动读取；
- Fedora Wayland：通过 XWayland socket 和当前 `XAUTHORITY` cookie 访问，不使用宽松的 `xhost +`。

`GRAPHICS_IMAGE` 是显式 build arg；迁移到其他开发机时应替换为该主机经过验证、具有相同 `/opt/mesa` 布局的图形运行时镜像。不要把当前 Intel 专用层当作 Pi 5 或任意 GPU 的通用方案。

可分别关闭界面：

```bash
RVIZ=false ./scripts/dev-start.sh
FOXGLOVE=false ./scripts/dev-start.sh
```

整车粗模型、SO-101 姿态滑块和内部状态显示：

```bash
./scripts/dev-view-model.sh
./scripts/dev-set-state.sh NAVIGATING
```

图形调整方法见 [`docs/model-visualization.md`](model-visualization.md)。

无 GUI 的接口 smoke test：

```bash
./scripts/dev-smoke.sh
```

## Orbbec Astra S 实体与仿真

开发机已验证的实体相机为 Orbbec Astra S `2bc5:0402`。ROS 2 Humble 容器从固定提交构建官方 legacy OpenNI 驱动；USB 仅在相机专用 Compose override 中透传，不给普通仿真容器 `--privileged`：

```bash
sudo ./scripts/install-astra-udev.sh  # 每台宿主只需一次
./scripts/dev-astra-start.sh          # 启动实体相机
./scripts/dev-astra-preview.sh        # RViz RGB/depth/点云预览
./scripts/dev-astra-check-running.sh  # 检查正在运行的有效 payload
./scripts/dev-astra-record.sh NAME    # 录制算法输入 rosbag
./scripts/dev-astra-smoke.sh          # 独占式实体完整验证
./scripts/dev-astra-sim-smoke.sh      # 无硬件 Gazebo 接口验证
```

以上多终端命令必须使用相同 `ROS_DOMAIN_ID`。`bags/` 是本地大文件目录，不纳入 Git。实体和 Gazebo backend 的算法公共接口为 `/camera/color/image_raw`、`/camera/depth/image_raw`、对应 CameraInfo 和 `/camera/depth/points`。仿真用于 Agent 自动接口/算法回归，不代表真实内参、噪声、IR、USB 断线或曝光行为；设计与来源见 [`docs/research/astra-s.md`](research/astra-s.md)。

ROS 2 Action `go_to_place_and_grasp` 和臂轨迹接口仅为发现兼容保留。`arm_enabled=false` 时它们拒绝所有目标，绝不伪造导航、抓取或任务成功；底盘控制使用支持 `linear.x`、`linear.y` 和旋转的 mecanum `/cmd_vel`。

镜像固定为 `osrf/ros:humble-desktop-jammy` 的 `linux/amd64` 开发环境。树莓派 5 是 arm64；此环境不构成 Pi 真机部署验证。
