# 系统架构

## 产品任务

架构只服务于一个首版端到端任务：接收 `place_id` 和 `object_id`，导航到预先登记的抓取地点，底盘停稳后识别并抓取目标，验证持物后报告结果。完整成功定义见 [`product-goal.md`](product-goal.md)。

## 原则

1. **先底盘，后叠加**：先恢复 WHEELTEC 最小运动闭环，再接传感器、机械臂和语音。
2. **硬件安全独立于智能层**：语音、视觉、Nav2 或 MoveIt 失效时，急停、使能和 watchdog 仍能停车。
3. **顶层提交完整任务**：上层调用 `go_to_place_and_grasp`，不能自行拼接无限期 `/cmd_vel` 或原始关节命令。
4. **不重复实现厂商底层**：STM32 已根据编码器和车型参数完成轮速运动学；ROS 端复用串口驱动。
5. **底盘停稳再抓取**：首版禁止移动中伸臂或抓取。
6. **端到端才算成功**：导航到位、抓手闭合都不是最终成功；必须验证物体已被安全抬起并保持。

## 计算与供电域

- 树莓派 5：ROS 2、设备驱动、任务状态机、离线 ASR、轻量视觉和监控。
- STM32 底盘控制器：电机闭环、车型运动学、编码器读取、板载 IMU 原始数据、底盘电压。
- SO-ARM101 控制板：6 个 STS3215 的串行总线。机械臂电源与 Pi 5 分路，不从 GPIO 或 Pi USB 给舵机供电。
- Livox MID-360：以太网接入；官方 `livox_ros_driver2` 的 `msg_MID360_launch.py` 发布 CustomMsg 点云和 IMU，FAST-LIO Localization 提供里程计与全局定位候选。IP、功耗和安装外参待实机确认。
- 深度相机：准确型号和功耗待确认；优先分配 USB 3。
- 实体安全：总电源、底盘电机使能、机械臂电源切断/保险和可触达急停。

## ROS 2 边界

```text
语音 / agent / operator -> provisional MHS Host
                                 |
                                 v
                     go_to_place_and_grasp
                         /               \
                        v                 v
                  Nav2 action        grasp action
                                          |          |
                                          v          v
voice / teleop / Nav2 cmd_vel -> twist_mux+safety -> WHEELTEC driver -> STM32
                                                   /    |     \
                                                /odom  /imu  /PowerVoltage

深度相机 -> RGB/depth/camera_info -> perception -> object pose
MID-360 -> msg_MID360_launch -> CustomMsg + IMU -> FAST-LIO localization -> Nav2
SO-ARM101 bridge <-> joint states / trajectory <-> MoveIt 2 planning scene
```

### 保留的厂商底盘边界

`turn_on_wheeltec_robot`：

- 串口：`/dev/wheeltec_controller`，115200 baud。
- 输入：`/cmd_vel` (`geometry_msgs/Twist`)。
- 输出：里程计、IMU、电池电压（实际话题名在旧系统取证后锁定）。
- STM32 上行 24 字节、下行 11 字节，`0x7B` 帧头、`0x7D` 帧尾、BCC 异或校验。
- 驱动退出会发送零速度，但首版仍需增加独立的命令超时与速度仲裁。

### MID-360 定位边界

固定数据入口：

```text
livox_ros_driver2 msg_MID360_launch.py
  -> /livox/lidar  livox_ros_driver2/msg/CustomMsg
  -> /livox/imu    sensor_msgs/msg/Imu
  -> fast_lio_localization config/mid360.yaml
  -> /Odometry + /cloud_registered
  -> ICP map correction + transform fusion
  -> /localization + map TF
  -> Nav2
```

`msg_MID360_launch.py` 默认 `xfer_format = 1`，因此 FAST-LIO 必须使用 `preprocess.lidar_type = 1`。若未来改为 PointCloud2，必须同步切换 launch 和 FAST-LIO 为 `lidar_type = 4`，禁止只改一侧。

进入导航前必须同时满足：真实 PCD 地图已加载、MID-360 点云/IMU 新鲜、全局定位有效、TF 连续且底盘坐标外参已标定。收到的 ZIP 没有真实地图，也没有证明 Pi 5 arm64 或 MID-360 实机全局定位已通过。

### 新增的最小项目模块

只在硬件参数确认后实现：

1. `soarm_car_bringup`：组合 launch、参数和设备检查。
2. `soarm_car_safety`：速度限幅、命令超时、急停、底盘停稳互锁、臂工作区限制。
3. `soarm_car_voice`：ASR 适配、白名单意图与确认状态机。
4. `soarm_car_manipulation`：SO-ARM101 ROS 2 bridge、MoveIt 配置、抓取执行与持物验证。
5. `soarm_car_mission`：实现 `go_to_place_and_grasp` 状态机，统一导航、停稳、感知、抓取、验证和失败收尾。

不要为每个传感器再包一层无意义 wrapper；直接使用其官方 ROS 2 driver。

## 坐标系

最小 TF 树：

```text
map -> odom -> base_footprint -> base_link
                              |-> lidar_link
                              |-> camera_link -> camera_*_optical_frame
                              |-> arm_base_link -> ... -> gripper_frame
```

安装后必须测量 `base_link` 到雷达、相机、臂底座的 6D 外参。不得拿示意 CAD 尺寸代替标定值。

## 语音动作模型

- 低风险、立即可撤销：`停止` 无需确认，最高优先级。
- 底盘点动：前进/后退/左转/右转以短脉冲执行，必须自动超时停车。
- 产品任务：只允许“到已命名地点抓取白名单物体”，重复播报地点和物体后确认。
- 抓取前置条件：底盘停稳、目标位姿新鲜、工作区有效、机械臂已准备、碰撞场景有效。
- 取消：任何阶段说“停止/取消”都终止整个任务、Nav2 和 MoveIt action，并进入安全停止或持物保持状态。

首版不采用自由文本直接生成运动轨迹，也不允许云端 LLM 绕过白名单状态机。

## Provisional MHS 能力层

产品首版以 Anthropic 已公开的 MHS 架构思想实现自己的硬件能力层，协议标识为 `provisional-mhs/v0`，不宣称符合尚未公开的官方规范。它是 voice、agent 和 operator 的主要硬件接口；ROS 2 仍是确定性执行和设备驱动总线。

- 上层只使用 `discover / describe / read / write / run / cancel / watch`。
- driver 将 capability 映射为有限参数的 ROS 2 action/service，不进入电机、舵机或传感器高频闭环。
- MHS Host 或网络断开时，ROS 2 safety、watchdog、急停和 MCU 闭环继续独立工作。
- 底盘与机械臂分别暴露能力；不提供任意 `/cmd_vel`、任意 topic publish、原始寄存器或关闭互锁的接口。
- `read` metadata 可描述重量、工作区和能力，但速度、关节、碰撞、供电和状态互锁由确定性代码强制执行。
- 顶层首要能力是 `go_to_place_and_grasp(place_id, object_id)`；状态、停止、点动、独立导航和机械臂技能仅用于其实现、调试和维护。

当前 provisional adapter 与未来 official adapter 共用同一个内部 capability/safety seam。官方 MHS 发布后替换 Host/protocol/reference serializer，不重写 ROS 2 adapters、设备能力和安全状态机。实施设计见 [`mhs-implementation.md`](mhs-implementation.md)，公开证据与未知项见 [`research/mhs.md`](research/mhs.md)。
