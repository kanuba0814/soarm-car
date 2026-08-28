# AI 语音交互与任务状态监控研究

## 结论

首版采用：**离线 ASR + 规则/普通 LLM 语义层 + 受限工具调用 + ROS 2 确定性任务状态机**。

普通 LLM 只负责理解、澄清、抽取 `place_id` / `object_id`、复述确认和解释结果；不生成 `/cmd_vel`、关节轨迹，也不根据传感器流自行判断任务成功。VLA 是从视觉与语言观测直接预测机器人动作的具身策略，适合未来研究“开放场景视觉抓取”等受限技能，但首版只有命名地点和白名单物体，没有必要引入 VLA 的训练数据、算力、适配和安全验证成本。

项目内依据：

- `docs/product-goal.md` 将首版输入限定为命名地点、白名单物体和 request ID，并禁止模型生成速度或关节轨迹。
- `config/voice-policy.yaml` 只允许完整抓取任务和最高优先级停止。
- `config/mission-policy.yaml` 已要求 TTL、控制租约、幂等、预期状态版本、数据新鲜度和安全互锁。
- `docs/mhs-implementation.md` 已把 `run/cancel/watch` 与 ROS 2 adapter 定义为统一 seam。
- 当前 `ros2_ws/src/soarm_fake/soarm_fake/fake_mission_server.py` 只是可视化预览，不执行或模拟硬件；其 Action 接口已提供 `stage/detail/progress_percent` 反馈字段。

## 推荐架构

```text
麦克风
  │
  ▼
离线 ASR ──“停止”旁路──────────────► Safety / cancel_all_and_stop
  │                                      （不等待 LLM）
  ▼
规则意图层 + 小型普通 LLM（可选）
  │  只做：意图、槽位、澄清、复述确认
  │  输出严格结构：
  │  {intent, place_id, object_id, request_id}
  ▼
Voice Orchestrator / MHS Host
  │  schema + 白名单 + TTL + lease + state_version
  │
  ├── read(robot_snapshot)
  ├── run(go_to_place_and_grasp)
  ├── watch(task_id)
  └── cancel(task_id)
  ▼
ROS 2 Mission Action / 确定性状态机
  ├── Nav2：导航、反馈、取消
  ├── Safety：停稳、急停、watchdog、限幅
  ├── Perception：目标 6D 位姿与新鲜度
  └── MoveIt/SO-ARM101：规划、执行、持物验证
  │
  ▼
结构化事件/结果 ──► TTS：“正在导航 / 未找到目标 / 已抓取并保持”
```

ROS 2 Action 本来就是“长任务 + 过程反馈 + 取消/抢占”的接口，因此应由任务状态机监控执行，而不是让 LLM 轮询原始 topic 或在对话上下文里模拟状态。[ROS 2 Actions](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html)

## 模型需要的上下文

不要输入整个 ROS graph、连续点云或视频。由 Context Builder 每次生成带版本和时间戳的最小快照：

```yaml
conversation:
  last_user_text: "去演示桌抓方块"
  unresolved_reference: null
request:
  request_id: voice-...
  intent: go_to_place_and_grasp
  place_id: demo_table
  object_id: demo_cube
policy:
  allowed_places: [demo_table]
  allowed_objects: [demo_cube]
  confirmation_required: true
robot:
  state_version: 42
  mission_state: IDLE
  estop: false
  safety_ready: true
  base_stationary: true
  battery_ok: true
localization:
  map_loaded: true
  valid: true
  pose_frame: map
  observed_at: ...
  age_ms: 80
sensors:
  lidar_fresh: true
  camera_fresh: true
arm:
  homed: true
  fault: null
active_task:
  task_id: null
  stage: null
  cancellable: true
capabilities:
  - go_to_place_and_grasp(place_id, object_id)
  - stop()
```

关键规则：

1. 所有位姿必须包含 frame、时间和新鲜度；tf2 本身按时间缓冲坐标系关系，不能把“某个坐标”当成脱离时间和 frame 的文本。[ROS 2 tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)
2. 上下文只供模型理解和回复；执行前由确定性代码重新读取权威状态并校验。
3. 旧 `state_version`、过期 TTL、未知字段、白名单外参数一律拒绝。
4. 原始图像只有在需要语义澄清时才给多模态 LLM/VLM；运动闭环仍不经过它。

## 后状态监控

```text
提交任务
  │
  ▼
Goal accepted? ──否──► 结构化拒绝 + TTS
  │是
  ▼
watch Action feedback / safety events
  │
  ├─ 每阶段检查：状态版本、超时、急停、定位/TF/传感器新鲜度
  ├─ NAVIGATING 后：验证到达命名位姿
  ├─ WAITING... 后：验证速度阈值内持续停稳
  ├─ PERCEIVING 后：验证 object_id、6D pose、时间戳、工作区
  ├─ PLANNING 后：验证无碰撞规划仍对应当前场景版本
  └─ EXECUTING 后：验证持物 + 安全持物姿态 + 底盘仍停稳
  │
  ├─ 任一失败/过期 ─► cancel Nav2/MoveIt → stop base → hold/retract arm
  │                                      └→ FAILED(stage, reason, recoverable)
  ▼
只有全部谓词成立 ─► SUCCEEDED
```

要区分四件事：**请求已接收、动作执行中、Action 返回、物理结果已验证**。发送 tool call 或 Action 返回成功都不能单独证明物体已经抓住。最终成功必须重新核对 `config/mission-policy.yaml` 的 `success_requires`。

建议事件结构至少包含：

```text
event_id, request_id, task_id, state_version,
stage, status, reason_code, observed_at,
source, data_age_ms, recoverable
```

模型只订阅阶段变化、异常和最终结果；高频里程计、点云、关节状态由 ROS 2 节点本地消费并汇总为谓词。

## LLM 与 VLA 的边界

| 能力 | 普通 LLM/规则层 | VLA | 首版选择 |
|---|---|---|---|
| 中文语音意图、澄清、复述 | 适合 | 不是重点 | 普通 LLM/规则 |
| 调用命名任务 | 适合，输出结构化参数 | 可做但浪费 | 普通 LLM |
| 导航速度与避障闭环 | 不应承担 | 研究上可预测动作 | Nav2/控制器 |
| 已知物体、固定桌面抓取 | 调用确定性 skill | 可作为未来 skill | MoveIt + 感知 |
| 开放视觉语义到动作 | 仅能规划/选工具 | VLA 的目标场景 | 后续研究 |

RT-2 把机器人动作表示为 token，并从图像与语言直接预测动作；这正说明 VLA 是低层/技能策略，不等同于对话 LLM。其论文也报告大模型推理约 1–3 Hz 且依赖云端 TPU，不能据此假设适合 Raspberry Pi 5 的首版实时控制。[RT-2 paper](https://proceedings.mlr.press/v229/zitkovich23a.html) [RT-2 project](https://robotics-transformer2.github.io/)

## 推荐落地顺序

1. 先不用 LLM：ASR → 白名单语法/槽位 → 复述确认 → MHS Action，验证全链路安全。
2. 加 Context Builder 和 `watch` 事件流，完成阶段反馈、取消、超时与最终物理谓词验证。
3. 再加入小型普通 LLM，仅改善同义表达、指代澄清和自然语言回复；保持同一严格 JSON schema。
4. 多模态 VLM 只用于“这是不是目标物体”等语义辅助，结果仍须由感知/策略层验证。
5. 只有固定方案无法覆盖开放抓取、且已收集本机示教数据并有独立 safety shield 时，再评估 VLA；把它封装成一个受限 grasp skill，不替换 MHS、Mission、Nav2 或 Safety。

## 参考资料

- [ROS 2 Humble: Actions](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html)
- [ROS 2 Humble: tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)
- [Nav2 NavigateToPose action](https://docs.ros.org/en/humble/p/nav2_msgs/action/NavigateToPose.html)
- [MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html)
- [RT-2: Vision-Language-Action Models](https://proceedings.mlr.press/v229/zitkovich23a.html)
- [Open X-Embodiment / RT-X](https://robotics-transformer-x.github.io/)
