# Provisional MHS 实施设计

状态：产品当前采用。产品唯一首版任务是 `go_to_place_and_grasp(place_id, object_id)`；本设计依据 Anthropic 已公开的 MHS 架构实现该任务，并在官方实现发布后低风险替换。

> 本实现必须标识为 `provisional-mhs/v0`，不得宣称符合尚未公开的官方 MHS 规范。

## 决策

MHS 是产品的主要硬件能力层，而不是附属的 Agent 网关。ROS 2 Humble 保留为确定性执行与设备驱动总线；所有上层调用经 MHS host 和 MHS driver，不直接依赖 ROS topic/action/service。

```text
voice / local agent / operator UI / autonomous workflow
                         |
                         v
                    MHS Host
      discovery / reference / workflow / audit
                         |
               MHS Driver Interface
                 read / write / run
                         |
        capability policy + safety supervisor
                         |
                  ROS 2 adapters
                         |
      WHEELTEC / SO-ARM101 / Orbbec / lidar
```

官方 MHS 发布后，替换 MHS Host、driver protocol adapter 和 reference-file serializer；设备能力实现、ROS 2 adapters、安全状态机和硬件驱动保持不变。

## 深模块与替换 seam

### `HardwareStandard` 模块

这是上层唯一需要学习的接口：

```text
discover(filter?) -> DeviceSummary[]
describe(device_id) -> DeviceReference
read(device_id, capability, args?) -> Reading
write(device_id, capability, args, context) -> Result
run(workflow, context) -> TaskHandle
cancel(task_id, context) -> Result
watch(selector, cursor?) -> EventStream
```

接口之外隐藏：

- ROS 2 topic/action/service 名称；
- 串口、USB、DDS 和设备厂商协议；
- capability 参数校验；
- lease、TTL、幂等、审批和审计；
- 超时、取消、重试和安全收尾；
- MHS transport 与序列化格式。

替换 adapter：

1. `ProvisionalMhsAdapter`：现在的产品实现；
2. `OfficialMhsAdapter`：官方 SDK/规范发布后实现；
3. `InMemoryMhsAdapter`：仿真和接口测试。

只有出现第二个生产 adapter 时才抽象 transport 细节；当前先保持模块接口稳定，不为未知官方 schema 建造复杂兼容层。

## Provisional driver contract

### DeviceReference

最小字段：

```yaml
standard: provisional-mhs/v0
device_id: base-1
device_class: mobile_base
manufacturer: WHEELTEC
model: unknown
connection:
  locality: on_device
capabilities:
  - name: state
    operation: read
  - name: stop
    operation: write
  - name: nudge
    operation: write
    input_schema: {...}
physical:
  mass_kg: null
  frame: base_link
safety:
  enforced_limits: [...]
  required_conditions: [...]
  forbidden_states: [...]
metadata:
  natural_language: ...
```

所有未知物理量必须为 `null` 或缺失，不能由模型猜测。

### 操作语义

- `read`：读取状态或有限数据；不得产生运动副作用。
- `write`：单个、有限、可验证的设备状态改变。
- `run`：本项目扩展，用于可取消的长任务或确定性多设备 workflow；它不是已公开的官方 MHS primitive。
- `watch`：本项目扩展，用于状态、任务和故障事件；不承担运动控制闭环。

首版唯一公开的长任务：

```text
run("go_to_place_and_grasp", {place_id, object_id}, context)
```

MHS Host 负责把它作为一个原子产品任务执行和审计；调用方不能自行组合 `navigate` 与 `grasp` 绕过停稳和感知前置条件。

所有 `write` / `run` context 至少包含：

```text
request_id
principal
issued_at
expires_at
expected_state_version
reason
```

需要控制权的操作还包含 lease。重复 `request_id` 必须返回原结果，不得重复执行。

## 设备 driver 划分

### Mission driver

公开长任务：`go_to_place_and_grasp(place_id, object_id)`。

内部状态机依次调用命名导航、底盘停稳判断、目标感知、抓取规划、执行、持物验证和安全收尾。任一步失败都取消相关子任务并返回结构化阶段错误；只有物体被验证抓取并进入安全持物姿态才成功。

### WHEELTEC base driver

读取：

- `state`：pose/velocity/停稳状态；
- `power`：底盘电压与健康状态；
- `faults`。

写入：

- `stop`；
- `nudge(direction, max_speed, duration)`；
- `set_enabled` 只允许本地授权流程。

长任务：

- `navigate(named_location)`，内部映射 Nav2 action。

不暴露无限期速度、任意 `/cmd_vel` 或串口帧写入。

### SO-ARM101 driver

读取：关节、抓手、温度/负载（硬件可用时）、homed/fault 状态。

写入：`stop`、`home`、`set_gripper` 和严格限位的维护动作。

长任务：预定义 skill、规划轨迹和抓取；内部映射 MoveIt/trajectory action。放置或释放仅为人工维护/恢复能力，不是首版产品任务。禁止原始寄存器和绕过碰撞检查的任意关节写入。

### Camera driver

读取：设备信息、单帧 RGB/depth/camera_info、流状态。连续媒体保留在 ROS image transport；MHS 返回句柄或受限快照，不在控制消息中搬运持续视频。

### Safety driver

只读：急停、互锁、watchdog、活动 lease、当前允许能力。

写入：`stop_all` 和任务取消。急停复位、解除硬件互锁默认不允许远程执行。

## 通信实现

第一版部署在单台 Raspberry Pi 5：

- driver 进程间：Unix domain socket；
- provisional wire format：带版本字段的 JSON-RPC 2.0；
- 本机 Agent：MHS Host 内部接口；需要 MCP 时仅作为上层 adapter；
- ROS 2 DDS：限制在机器人内部网络/本机；
- 大图像：ROS image transport 或共享内存/文件句柄，不嵌入 JSON；
- 外部 UI：只连接 MHS Host 的受认证产品接口，不连接 driver socket 或 ROS graph。

选择 Unix socket 是当前实现决定，不推断官方 MHS transport。socket 文件使用独立 Linux 用户/组权限；每个 driver 有独立 endpoint 和最小权限。

## 安全不变量

1. MHS Host、Agent 或网络断开时，底盘速度在 watchdog 时限内归零。
2. 安全限制在 driver/safety supervisor 内确定性执行，natural-language metadata 只作说明。
3. `stop` 可抢占其他任务且不要求普通确认。
4. 运动命令必须有限幅、TTL、lease 和状态前置条件。
5. 底盘未停稳时拒绝伸臂抓取。
6. `go_to_place_and_grasp` 只有在持物验证通过并进入安全持物姿态后才能成功。
7. 导航、感知、规划或抓取任一步失败都不能被误报为完整任务成功。
8. 旧命令、重复命令、状态版本冲突和未知字段均 fail closed。
9. 官方 adapter 不能获得绕过这些不变量的内部入口。

## 官方替换流程

官方规范/SDK 发布后：

1. 冻结一组真实设备 contract tests 和故障注入场景；
2. 对照官方 device/reference/capability/error/lifecycle 模型制作差异表；
3. 实现 `OfficialMhsAdapter`，将官方对象映射到内部 capability kernel；
4. 同一测试集并行运行 provisional 与 official adapter；
5. 验证真实 ROS 2 action、串口 watchdog、取消和断线路径；
6. 逐设备切换，可随时回退；
7. 稳定后删除 provisional wire adapter，保留设备实现和安全内核。

若官方模型与我们的接口冲突，优先修改 adapter；只有官方语义证明内部 capability kernel 无法表达必要安全行为时，才迁移内部接口。

## 第一阶段交付顺序

1. `HardwareStandard` 类型、错误模型和 in-memory adapter；
2. safety driver 与审计事件；
3. WHEELTEC `state/stop/nudge` driver；
4. 断线 watchdog、TTL、幂等和 lease 测试；
5. Camera 单帧与目标位姿 read driver；
6. SO-ARM101 state/stop/home/grasp；
7. `go_to_place_and_grasp` mission driver、取消和阶段错误；
8. 完整任务的 contract、仿真、故障注入和实机测试；
9. 本地 Agent/voice adapter；
10. 官方 MHS 发布后的兼容 adapter。
