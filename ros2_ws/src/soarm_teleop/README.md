# SOARM 键盘遥控与 STM32 底盘串口适配

一套键盘遥控，通过共享的 Twist 命令接口同时覆盖仿真和物理车；物理车侧补齐
STM32F405 麦轮底盘的串口适配节点。协议依据
`artifacts/下位机代码+测试脚本+接口文档.zip` 中的 `上位机接口交付说明.md`，
帧编码已按其第 5 节示例帧逐字节核对（见 `test/test_chassis_protocol.py`）。

## 命令链路

三种底盘目标都消费 `geometry_msgs/msg/Twist`，键盘节点因此只有一份实现：

| target | 话题 | 下游消费者 |
|---|---|---|
| `fortress` | `/cmd_vel` | Fortress 原生仿真 `MecanumDrive`（`simulation.launch.py`） |
| `ros2_control` | `/safe_cmd_vel` | `kuruma_ros2_control` 仿真档 `mecanum_drive_controller`（控制器参考话题已重映射，无门控） |
| `physical` | `/cmd_vel` | `localization_cmd_gate` → `/safe_cmd_vel` → `stm32_chassis_serial` → 下位机 USART1 |

物理链路严格遵守工作区约定：遥控/导航都只写 `/cmd_vel`，由失效关闭的安全门
唯一授权 `/safe_cmd_vel`，串口适配器是下位机链路的唯一写入者。任何组件都不得
绕过安全门直接暴露串口帧。

## 键位

```
平移:    u  i  o        原地旋转:   q (逆时针)   e (顺时针)
         j  k  l        立即停止:   k 或 空格
         m  ,  .        速度档位:   z (-10%)     x (+10%)
```

- 按住键持续移动（依赖终端自动重复），松开后最多 `hold_timeout`（默认 1 s）
  内衰减为零——这是死曼式保护，不是经典 teleop 的"按键保持直到下一键"。
- 按住平移键的同时点按 q/e 可以走弧线。
- 对角键（u/o/m/.）已归一化，合成速度不超过档位上限。
- `Ctrl-C` 退出，退出前发布一次零速度。

## 使用

所有命令在 `ros2-dev` 容器内执行（`docker compose run -it ros2-dev …`）。

### 仿真（Fortress 原生档）

```bash
# 终端 1
ros2 launch soarm_simulation simulation.launch.py
# 终端 2
ros2 launch soarm_teleop keyboard_teleop.launch.py target:=fortress
```

### 仿真（kuruma_ros2_control 档）

```bash
# 终端 1
ros2 launch soarm_simulation kuruma_ros2_control.launch.py
# 终端 2（该档控制器直接消费 /safe_cmd_vel，不经过安全门）
ros2 launch soarm_teleop keyboard_teleop.launch.py target:=ros2_control
```

### 物理车（STM32 麦轮底盘）

需要三个条件：定位栈健康（安全门只在 `/localization/health` 为 `HEALTHY`
且新鲜时放行）、串口设备已传入容器、安全门 + 串口适配器已启动。

```bash
# 终端 1：定位栈（沿用现有入口，例如视觉/FAST-LIO 定位启动文件）
# 终端 2：安全门 + STM32 串口适配（gate 参数与 wheeltec_safe_base 一致）
ros2 launch soarm_teleop stm32_safe_base.launch.py serial_port:=/dev/ttyUSB0
# 终端 3：确认安全门已放行
ros2 service call /localization_cmd_gate/status std_srvs/srv/Trigger {}
# 终端 3：键盘遥控（物理档默认 0.3 m/s、0.8 rad/s，比仿真档保守）
ros2 launch soarm_teleop keyboard_teleop.launch.py target:=physical
```

串口设备需要在容器内可见（compose 默认只透传渲染设备）；上实车时在
`compose.yaml` 增加该设备的 `devices:` 透传或用 `docker run --device`。
建议为底盘 USB 串口配置 udev 别名（如 `/dev/soarm_chassis`）避免插拔变号。

### 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `keyboard_teleop.max_linear` / `max_angular` | 按 target 预设 | 档位上限（m/s、rad/s） |
| `keyboard_teleop.hold_timeout` | 1.0 s | 键输入过期时间，过期即停 |
| `keyboard_teleop.speed_levels` | 10 | 速度档数（z/x 每步 10%） |
| `stm32_chassis_serial.port` / `baud` | `/dev/ttyUSB0` / 115200 | USART1 串口 |
| `stm32_chassis_serial.send_period` | 0.05 s | 速度帧周期（文档建议 50–100 ms，硬性 < 500 ms） |
| `stm32_chassis_serial.wz_scale` | 1.0 | `wz` 标定系数（固件角速度未标定，交付文档 8.2） |
| `stm32_chassis_serial.status_poll_period` | 0（关闭） | 状态轮询周期，最小 0.2 s（文档 8.5：≤5 Hz） |

## 串口适配器行为（对齐交付文档）

- 连接（含断线重连）后：先 `CMD_STOP` 再进入 `MODE_PC`（文档 8.4：模式切换前
  必须先停车）。
- 命令新鲜期内以 50 ms 周期流式发送 `CMD_SET_VELOCITY`（vx/vy 单位 m/s→mm/s，
  wz rad/s→mrad/s）；每帧都会刷新固件 500 ms 看门狗。
- 命令过期（上游安全门停止发布）：只发一次 `CMD_STOP`，随后保持静默——
  不用心跳去顶替已失效的命令源，让固件看门狗保持诚实的失效保护。
- 退出（Ctrl-C/SIGTERM）：`CMD_STOP` → `MODE_REMOTE`，把底盘交还遥控器。
- 收到 `CMD_STATUS_ACK` 时校验 mode：若底盘脱离 `MODE_PC` 会记 error 日志；
  `vx/vy/wz/battery/status/error` 字段固件尚未填充，一律不依赖（文档 8.8）。
- 不实现 `CMD_MOVE_DISTANCE` 位移队列：它只在全程不旋转时有效且无失败上报
  （文档 8.1/8.6），属于后续上层任务，不应混进速度遥控。

## 测试

```bash
# 单元测试（容器内）
python3 -m pytest -q ros2_ws/src/soarm_teleop/test/
# 冒烟：构建 + 单测 + 键盘节点话题级验证 + 串口适配器 PTY 回环联调
./scripts/dev-teleop-smoke.sh
```

冒烟脚本用伪终端（PTY）模拟下位机：校验握手顺序（STOP→MODE_PC）、
速度帧单位换算、命令过期停帧和退出时的遥控器交还（STOP→MODE_REMOTE），
不接触真实硬件。

## 边界

- `wz` 绝对值未标定，只能当"转动快慢"的相对量用（交付文档 8.2）。
- 键盘遥控与 Nav2 同时写 `/cmd_vel` 会互相抢命令源，不要并行。
- 本包不改动 `localization_cmd_gate` 的失效关闭逻辑；物理车能否移动取决于
  定位健康与安全门状态，这是设计要求而不是限制。
