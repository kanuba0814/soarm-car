# Model Hardware Standard（MHS）研究预览

访问日期：2026-08-28。本文只依据 Anthropic/MHS 官方页面以及其直接引用的 MCP 官方规范；页面快照位于 `docs/research/sources/mhs/`。

## 当前状态

MHS 是 Anthropic 与 HHMI Janelia 发起的物理设备标准研究预览，目标是让 AI agent 通过统一接口发现并安全操作实验室、机器人和制造设备。当前属于**限量、申请制 research preview**；官方明确表示将在预览和安全评估之后开源。

截至访问日期，官方没有公开：

- MHS 规范正文、版本号或许可证；
- 驱动 SDK、官方源代码仓库或参考实现；
- wire schema、传输、鉴权、错误模型和 safety tag 语法；
- ROS 2、WHEELTEC、SO-ARM101 或 Raspberry Pi 5 的兼容性承诺。

因此，本项目不能依据宣传页推断或声称“MHS compliant”。产品当前采用公开架构思想实现明确标识的 `provisional-mhs/v0`，并把所有未公开的 schema、transport 和错误语义标记为项目扩展；官方实现发布后经 adapter seam 替换。详见 `docs/mhs-implementation.md`。

## 公开架构

官方公开描述的 MHS driver：

1. 位于操作系统与具有可编程接口的硬件之间；
2. 将厂商接口转换成简单的 `read` / `write` primitives；
3. 以标准格式提供设备发现；
4. 用自然语言 tags 补充代码无法表达的设备特征，例如机械臂重量；
5. 自动产生 device reference file，描述设备能测量什么、能调整什么以及会强制执行哪些安全限制；
6. 对 agent 提供三种控制面：MCP、CLI 和代码文件/API。

代码文件用于组合多个设备命令、执行长任务或高于在线模型推理频率的操作，让确定性程序运行闭环，而不是要求 agent 对每一步实时推理。

MHS 被描述为 model-agnostic，可由任意 agent harness 经 MCP 等标准协议访问。这里的“例如 MCP”不代表 MCP 本身就是 MHS，也不证明任意 MCP hardware server 符合 MHS。

## 与本机器人的分层

建议边界：

```text
AI / voice / operator
        |
        v
Provisional MHS Host（当前产品主能力层）
  - 设备发现与 reference
  - 白名单、有限参数的 read/write/run
  - 人工确认、租约与审计
        |
        v
ROS 2 task orchestrator / actions / services
  - Nav2 goal
  - bounded base motion
  - MoveIt/grasp action
  - camera observation
        |
        v
ROS 2 safety + drivers + MCU/servo controller
  - twist_mux / watchdog / timeout
  - collision and joint limits
  - emergency stop / power interlock
  - deterministic and high-rate loops
```

MHS 层不得直接暴露：

- 无限期或无界 `/cmd_vel`；
- 原始舵机寄存器写入；
- 绕过 MoveIt 碰撞检查的关节命令；
- 安全限位、急停、使能和供电控制的关闭接口；
- 任意 shell、任意 ROS topic publish 或任意参数写入。

WHEELTEC 底盘、SO-ARM101、深度相机和未来 3D 雷达应分别保留 ROS 2 device boundary。未来拿到 MHS 规范后，在这些边界之上暴露最小 capability，而不是重写底层驱动。

### 建议的能力形状（项目设计，不是 MHS 规范）

只读：

- `base.get_state`：里程计、是否停稳、电压、故障；
- `arm.get_state`：关节状态、抓手状态、故障；
- `camera.capture`：受限模式的 RGB/depth/camera_info；
- `safety.get_state`：急停、互锁、watchdog 和允许动作集合。

写入：

- `base.stop`：最高优先级；
- `base.nudge(direction, distance_or_duration)`：严格限幅、自动超时；
- `navigate_to(named_location)`：仅命名目标；
- `arm.execute_named_skill(name, bounded_args)`：预定义抓取技能；
- `task.cancel(id)`：取消 Nav2/MoveIt/任务状态机。

这些能力最终仍由 ROS 2 action/service 执行和拒绝非法状态。自然语言 metadata 只能帮助 agent 理解设备，不能成为安全约束的唯一执行点。

## 直接相关的官方生态信号

Anthropic 公告称：

- Hugging Face 正在为 LeRobot 添加 MHS 支持；
- Raspberry Pi 在 Camera MHS Driver 测试后，正为若干产品启用 MHS 集成；
- Universal Robots、Doosan Robotics 等厂商参与早期测试。

这些是方向信号，不构成 SO-ARM101、Pi 5、Orbbec 或本项目的现成支持。后续应监控 LeRobot 官方仓库与 Raspberry Pi 官方发布，而不是采用第三方同名“MHS”项目。

## MCP 的关系及安全边界

MCP 2025-06-18 公开规范使用 JSON-RPC 2.0，定义 host/client/server、capability negotiation，以及 resources/prompts/tools 等功能。它强调用户同意、数据隐私、工具安全和访问控制，但规范也明确协议本身不能强制执行全部安全原则。

所以，即使未来用 MCP 暴露 MHS capability，也必须由本项目本地执行：

- 身份认证、最小权限和网络隔离；
- 明确确认与可审计日志；
- 参数 schema、速度/位置/时长上限；
- 超时、取消、fail-closed；
- 独立急停、watchdog、碰撞与供电互锁。

## 采用门槛

首版采用 provisional 实现，但在以下条件满足前，不声称官方兼容，也不删除 provisional adapter：

1. Anthropic 发布规范/SDK，或本项目获准进入 preview；
2. 明确许可证、版本、driver schema、鉴权和错误语义；
3. official adapter 能稳定映射到受限 ROS 2 action/service；
4. 与 provisional adapter 共用的负面测试证明任意输入都无法绕过安全状态机；
5. MHS/MCP 进程断开、超时或崩溃时，机器人能安全停止。

## 官方来源

- Anthropic, “Previewing the Model Hardware Standard”: <https://www.anthropic.com/news/model-hardware-standard-research-preview>
- MHS official research-preview landing page: <https://www.modelhardwarestandard.com/>
- MCP specification 2025-06-18: <https://modelcontextprotocol.io/specification/2025-06-18>
- MCP authoritative TypeScript schema: <https://github.com/modelcontextprotocol/specification/blob/main/schema/2025-06-18/schema.ts>
