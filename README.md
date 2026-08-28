# soarm-car

在 WHEELTEC 四轮移动底盘上安装 SO-ARM101，以树莓派 5 为主控，完成一个明确的产品任务：**机器人移动到指定地点，停稳后抓取指定物体**。

> 当前阶段：资产保全与端到端闭环准备。底盘车型、3D 雷达型号、相机准确型号和整机电气预算仍需实机确认，未通过分阶段安全验收前不得整机自主运行。

## 已确定

- 主控：树莓派 5。
- 底盘：图片所示 WHEELTEC 四轮金属底盘，实际运动学型号待铭牌/下位机配置确认。
- 机械臂：SO-ARM101 follower（从臂），安装在底盘上层。
- 感知：已有深度相机；定位雷达确定为 Livox MID-360，已有 FAST-LIO Localization 调试工作区和官方 Livox driver 快照。
- 产品任务：`到指定地点抓取指定物体`；首版地点和物体均使用白名单。
- 交互：离线语音、Agent 和操作界面调用同一个 provisional MHS 任务。
- 系统：重建树莓派系统，不把旧厂商整机镜像当长期依赖；先迁出底盘最小驱动。
- ROS 基线：首版采用 ROS 2 Humble，以兼容 WHEELTEC 已有源码。Jazzy 作为后续迁移候选，不在首版混用。

## 仓库内容

- [`docs/product-goal.md`](docs/product-goal.md)：唯一首版任务、成功定义、状态机和验收标准。
- [`docs/architecture.md`](docs/architecture.md)：围绕端到端抓取任务的系统边界、ROS 节点与安全链路。
- [`docs/plan.md`](docs/plan.md)：按风险排序的实施阶段和验收条件。
- [`docs/system-rebuild.md`](docs/system-rebuild.md)：旧系统取证、新镜像重建和回退步骤。
- [`docs/hardware-checklist.md`](docs/hardware-checklist.md)：装臂前必须测量/确认的机械、电气项目。
- [`docs/unknowns.md`](docs/unknowns.md)：会改变设计的未决项。
- [`docs/research/wheeltec-local.md`](docs/research/wheeltec-local.md)：44GB WHEELTEC 本地资料的证据索引。
- [`docs/research/vendor-image.md`](docs/research/vendor-image.md)：本地树莓派5全盘备份、容器和实机配置勘察。
- [`docs/research/mid360-localization.md`](docs/research/mid360-localization.md)：MID-360 数据入口、FAST-LIO 定位资产和缺口。
- [`docs/research/upstream.md`](docs/research/upstream.md)：SO-ARM101、ROS 2、Nav2、MoveIt 2 等官方资料结论。
- [`docs/research/mhs.md`](docs/research/mhs.md)：Anthropic MHS 预览、MCP 关系与公开未知项。
- [`docs/mhs-implementation.md`](docs/mhs-implementation.md)：当前 provisional MHS 实现与未来官方替换设计。
- [`docs/upstream-manifest.yaml`](docs/upstream-manifest.yaml)：上游来源与版本固定入口。
- [`vendor/wheeltec/`](vendor/wheeltec/)：从实机备份提取的底盘驱动、udev，以及匹配的奥比中光驱动快照。
- [`vendor/localization/`](vendor/localization/)：MID-360 的 `msg_MID360_launch` 数据入口与 FAST-LIO Localization 最小源码快照。
- [`config/mission-policy.yaml`](config/mission-policy.yaml)：指定地点抓取任务的阶段、成功条件与安全策略。
- [`config/voice-policy.yaml`](config/voice-policy.yaml)：只触发完整产品任务或停止的语音白名单。

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

图形调整方法见 [`docs/model-visualization.md`](docs/model-visualization.md)。

无 GUI 的接口 smoke test：

```bash
./scripts/dev-smoke.sh
```

唯一预留 Agent seam 是 ROS 2 Action `go_to_place_and_grasp`，不暴露 `cmd_vel`、关节轨迹、ROS graph 或任意 shell。当前只接受 `demo_table` / `demo_cube`，用于在 RViz/Foxglove 标出请求目标；实机执行后续由成熟 Nav2/MoveIt 接口替换，不在此阶段提前编排。

镜像固定为 `osrf/ros:humble-desktop-jammy` 的 `linux/amd64` 开发环境。树莓派 5 是 arm64；此环境不构成 Pi 真机部署验证。
