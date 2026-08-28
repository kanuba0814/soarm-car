# 实施计划

所有阶段最终服务于唯一产品任务：`到指定地点抓取指定物体`。每一阶段提供独立可验收能力，但不能把子系统通过误报为产品完成。产品成功和失败定义见 `docs/product-goal.md`。

## P0：规划、取证与资产保全（当前）

- [x] 初始化 Git 仓库。
- [x] 索引 WHEELTEC 通用资料，核实串口协议、ROS2 驱动和树莓派5 Docker 说明。
- [x] 从 Humble 2025-04 厂商归档提取最小底盘驱动及依赖。
- [x] 下载小体积关键官方文档并生成 SHA-256。
- [x] 找到树莓派5完整卡备份并验证 gzip、分区、系统、容器与 SHA-256。
- [x] 从备份只读提取实机底盘驱动、udev、容器元数据和奥比中光硬件规则。
- [ ] 把原始备份复制到另一块物理磁盘并复核 SHA-256。
- [ ] 若原卡仍可启动，补充实时 ROS graph、参数与短 bag（静态镜像不能替代运行态证据）。
- [ ] 确认底盘铭牌、STM32 固件车型和四驱运动学类型。

**完成条件**：即使旧 TF 卡损坏，也有原始镜像/备份、最小驱动、设备规则、参数与话题样本可恢复。

## P1：新系统与底盘最小闭环

1. 树莓派5安装厂商原镜像的副本，用于建立兼容基准；另备一张卡安装计划中的干净系统。
2. 首版固定 ROS 2 Humble；容器或宿主选择以 USB/udev、性能和复现测试决定。
3. 编译 `vendor/wheeltec/src`，先不启动雷达、相机、Nav2。
4. 固定 `/dev/wheeltec_controller`，验证 115200 串口。
5. 电机架空、使能关闭：检查里程计、IMU、电压和帧率。
6. 使能后低速验证前/后/转向、正负方向和零速停车。
7. 加入命令 watchdog、速度/加速度限制和急停输入。

**完成条件**：连续运行 30 分钟无串口失联；断开上层命令、节点退出或按急停时均停车；方向与 TF 符合 REP-103。必须验证 STM32 stale-command watchdog 或独立电机使能切断能够覆盖 host `SIGKILL`、USB 断开和 Pi 故障；只靠 ROS 节点析构发送零速不算通过。

## P1.5：Provisional MHS 能力层

1. 定义 `provisional-mhs/v0` 的 `DeviceReference`、capability、错误和事件模型；所有未知物理参数保持 `null`，禁止模型补猜。
2. 实现 `HardwareStandard` 模块和 in-memory adapter，冻结 `discover/describe/read/write/run/cancel/watch` contract tests。
3. 实现 Unix domain socket + 版本化 JSON-RPC 2.0 adapter；明确这是临时 wire format，不冒充官方协议。
4. 实现 safety driver、审计事件、TTL、幂等、state version、控制 lease 和 fail-closed 错误处理。
5. 实现 WHEELTEC `state/stop/nudge` driver，并通过真实 ROS 2、串口和断线 watchdog 验证下游 consumer seam。
6. 实现 Orbbec 单帧 read driver；连续图像不进入 JSON 控制面。
7. 实现 SO-ARM101 `state/stop/home/grasp` 和持物验证。
8. 实现唯一顶层任务 `go_to_place_and_grasp(place_id, object_id)`；独立导航和抓取只作为内部或维护能力。
9. voice、local agent 和 operator 只调用 MHS Host，不直接操作 ROS graph。
10. 维护 official adapter 差异表；官方 MHS 发布前不得声称兼容。

**完成条件**：三种 adapter（in-memory、provisional、故障注入 fake）通过同一 contract tests；完整任务的重复、过期、越权、状态冲突和中途失败均被正确拒绝或收尾；MHS Host 崩溃不会导致底盘或机械臂继续陈旧动作。详细设计见 `docs/mhs-implementation.md`。

## P2：机械与供电验证

1. 称重并测量上层板尺寸、孔位、轮距、轴距和重心。
2. 查询底盘有效载荷、SO-ARM101 总质量/最大工作半径、舵机峰值电流。
3. 设计低而居中的臂底座和加强板；校核最大伸展时静态倾覆裕度。
4. 机械臂电源、Pi 5 和传感器分别预算；共地、独立保险/开关、线缆应力释放。
5. 用配重代替机械臂做慢速制动、转弯和坡面稳定性测试。

**完成条件**：满载电压无异常跌落、线束不进入轮/关节、急停可同时禁止底盘和机械臂产生新动作。

## P3：MID-360 定位与深度相机

### P3.1 MID-360 数据入口

1. 固定 `livox_ros_driver2` 提交和 `msg_MID360_launch.py`；记录 MID-360 固件、IP、网络和时间同步。
2. 验证 `/livox/lidar` 类型为 `livox_ros_driver2/msg/CustomMsg`、`/livox/imu` 为 `sensor_msgs/msg/Imu`，频率、时间戳和 frame 连续。
3. 使用 `config/mid360.yaml` 的 `lidar_type = 1`；若切换 PointCloud2，必须同时修改 Livox `xfer_format` 与 FAST-LIO `lidar_type = 4`。
4. 在 Pi 5 arm64 实际构建并进行网络断开、驱动重启和至少 30 分钟长稳测试。Livox 官方将该 driver 定位为调试工具，产品化不能只验启动。

### P3.2 FAST-LIO 与全局定位

1. 先验证 `/Odometry` 和 `/cloud_registered`，再接 global map publisher、ICP correction 和 transform fusion。
2. 获取或建立真实 PCD 地图；原始 ZIP 不含可用地图。
3. 标定 MID-360 到 `base_link` 的外参，规范 TF 为 `map -> odom -> base_footprint -> base_link -> livox_frame`；不得直接沿用示例 `camera_init/body`。
4. 用已知初始位姿和重复路线评估重定位、漂移、协方差、退化场景和丢帧恢复。
5. 只向 Nav2 提供经过健康和新鲜度检查的定位；独立 Autoware 桥接输出不是首版必需依赖。

### P3.3 深度相机

1. 锁定型号、固件、官方 ROS2 驱动版本和 USB 配置。
2. 检查 RGB/depth/camera_info 的 frame、时间戳、带宽、CPU、温度和掉帧。
3. 测量相机到 `base_link` 和机械臂底座的外参；保存标定文件，不硬编码在 launch 中。

**完成条件**：MID-360、FAST-LIO 全局定位和深度相机同时运行至少 30 分钟，无网络/USB异常；地图、定位、TF 和图像均满足新鲜度；重复到达命名抓取地点的误差足以使目标落入相机视野和机械臂工作区。

## P4：SO-ARM101 follower 独立闭环

1. 按 LeRobot 官方流程设置电机 ID、baudrate 和校准。
2. 记录控制板型号、电源额定值、USB 标识和每个关节的软限位。
3. 先用 LeRobot 原生 API 验证读取与低速动作，再做 ROS2 bridge。
4. 发布 `joint_states`，接受受限的关节/轨迹目标；加入超时、限速和通信失联处理。
5. 建立准确 URDF/SRDF、自碰撞矩阵和底盘碰撞体。

**完成条件**：独立重复执行 home、预抓取、抓取、抬升和安全持物姿态；任何越界命令被拒绝，断开通信不会继续执行陈旧目标或导致未经评估的释放。

## P5：静态抓取闭环

1. 首版只抓一种已知物体、固定桌面高度和有限工作区。
2. 深度相机产生物体 6D 位姿；手眼标定写入版本化校准文件。
3. MoveIt 2 做 IK、碰撞检查和轨迹规划；底盘作为固定碰撞体。
4. 状态机：底盘已停稳 → 感知 → 预抓取 → 抓取 → 抬升 → 持物验证 → 安全持物。

**完成条件**：在底盘预先停放的受控场景连续执行 20 次，记录从感知到安全持物的成功率；目标不可达、消失、碰撞或抓取验证失败时安全退出。放置不属于本阶段成功条件。

## P6：离线语音

1. 先测试厂商讯飞 M2/现有麦阵列是否确为当前硬件；否则选择树莓派可实时运行的离线 ASR。
2. 只实现 `config/voice-policy.yaml` 中的白名单。
3. ASR 不直接控制执行器，只调用 MHS `go_to_place_and_grasp` 或 `stop`。
4. 同时复述地点和物体并确认；完整任务支持取消、超时、阶段反馈和声音结果。

**完成条件**：安静/电机噪声条件分别测试；误识别不能越过确认和安全层；“停止”始终抢占当前任务。

## P7：指定地点抓取集成

- 输入 `place_id + object_id` → Nav2 导航 → 到位并刹停 → 目标识别 → 抓取 → 抬升 → 持物验证 → 报告结果。
- 首版不做边走边抓、开放词汇任意物体、抓取后配送/放置或自由语言生成轨迹。
- 为每次运行记录 request ID、各阶段耗时、导航误差、感知/规划/抓取结果、CPU、RAM、温度、USB、供电和失败原因。

**完成条件**：受控场景连续执行至少 20 次完整任务并统计端到端成功率；任一子系统失败均转入定义的安全状态，不得把部分完成报告为任务成功。
