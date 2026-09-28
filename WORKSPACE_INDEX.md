# 工作区索引与复用边界

核对日期：2026-09-09。路径均为本机现有 Git worktree；HEAD 是核对时的基点，不代表包含未提交实现。

## 首要约定

- **最新资源授权（2026-09-10）：NVIDIA已恢复，取代此前Intel-only限制。** 已实查RTX3070、驱动610.57.04、8GiB显存约7.6GiB空闲；NVIDIA在card0/renderD129，Intel ARL仍在card1/renderD128。优先恢复已验证的NVIDIA渲染环境，但须读取实际OGRE renderer确认，不能仅凭设备存在。一次一套仿真；端到端验证前不并行大模型推理。Intel实验、CPU优化及失败记录全部保留。

- **导航已有成功实现，优先复用 `/home/gxxl/soarm-car-nav2-clean` 的现有入口。不得因为 AWS 研究支线失败而重写导航或声称整个项目没有导航能力。**
- 当前任务：小型 VL、语义栅格及其组合的离线开发验证；随后基于已有导航接口设计毕业演示。不是再造 Nav2，也不是扩大论文评估规模。
- 仿真采用球轮代替麦轮，允许近似全向运动学；不要求复现真实麦轮接触动力学。
- **面向使用者的程序按通用家庭机器人产品设计**，不出现“毕业设计／论文验证”式品牌或用途定位。任务、场景记忆、视觉分析、运行记录使用中性名称；真实/仿真、历史/实时和只读状态仍须清楚标示。研究用途只放在项目文档，不包装成程序外观。
- 每次启动先检查本表、目标 worktree 的 `git status` 和实际入口；只允许一个仿真所有者。模型、地图、定位 PCD、坐标系和镜像必须成套使用。
- 不修改其他 worktree 的用户未提交文件；不把实验改动混入历史冻结论文结果；不推送远端、不操作实物。

## 工作区分工

| 本机路径（均在 `/home/gxxl/`） | 分支 / 核对 HEAD | 用途与使用边界 |
|---|---|---|
| `soarm-car-graduation` | `integration/graduation-assistant` / `9b9dd2d` | **当前产品功能集成工作区**：恢复快照、命名请求、语义记忆、VL裁剪和只读工作台已合并。入口 http://127.0.0.1:8766；说明 `docs/robot-workspace.md`，批次追溯 `docs/research/unattended-graduation-experiments.md`。 |
| `soarm-car-nav-recovery` | `experiment/nav-baseline-recovery` / `33187ea` | 原导航当前状态恢复快照，两个独立恢复验证通过；已合并集成分支 `75cbe01`，未修改原导航。 |
| `soarm-car-nav2-clean` | `experiment/nav2-clean-baseline` / `41463f9` | **现有自主导航优先复用基线**：球轮 `kuruma_ros2_control`、FAST-LIO、Smac、全向 MPPI；有多房间成功记录。当前有修改及未跟踪脚本，不能仅凭 HEAD 还原当前成功实现。 |
| `soarm-car-companion-validation` | `research/companion-home-validation` / `08f5eb2` | 家具住宅 GUI、RGB-D、论文包及手动演示。最新演示是球轮 + Fortress `VelocityControl` 运动学近似，不是上述 ros2_control 导航基线。 |
| `soarm-car-vlm-grid` | `research/vlm-grid-design` / `5c58bb1` | 小型 VL + 公开 RGB-D 栅格开发小试；五项几何检查及有限ID适配已运行。不拥有导航/仿真进程；未接通端到端导航。设计说明 `docs/research/graduation-hybrid-design.md`；模型权重在独立缓存，输入输出和恢复锁由本分支记录。 |
| `soarm-car-main-integration` | `main` / `3c82261` | 已集成房屋净空、参考定位地图和 HM3D 场景；包含导航入口，本次未在此运行。 |
| `soarm-car` | `feature/kuruma-rethink` / `3490d7f` | 用户主工作区，有大量未提交的房间导航、定位、安全链和感知改动。**不是可任意清理的副本。** |
| `soarm-car-hm3d-apartments` | `feature/hm3d-apartment-worlds` / `9e8228a` | HM3D apartment_a/b 家庭场景与资产转换；资产许可和忽略文件需另行恢复。 |
| `soarm-car-larger-house` | `feature/larger-house-clearance` / `5148af6` | 房屋尺度/净空与定位场景历史分支。 |
| `soarm-car-companion-nav` | `research/companion-nav-knownpose` / `4035770` | AWS 已知自身位姿研究支线，保留 ABORTED 记录；**不作为现有导航的替代入口**，也不代表其他分支导航失败。 |

### 自主探索批次（进行中，NVIDIA恢复后继续）

| 工作树 | 分支 | 当前职责 |
|---|---|---|
| `soarm-car-intel-runtime` | `experiment/intel-runtime` | Intel阶段渲染、心跳与输入新鲜度调查/修复；现为历史实验，不拥有当前仿真。 |
| `soarm-car-sensor-pipeline` | `experiment/sensor-pipeline` | 原生点云转换与轨迹显示优化，保留真实等价/延迟验证及Intel参数试验；现不拥有仿真。 |
| `soarm-car-nvidia-runtime` | `experiment/nvidia-exploration-runtime` | 当前NVIDIA渲染和原导航恢复，成功后明确把唯一仿真/动作所有权交给探索运行工作树。 |
| `soarm-car-frontier-core` | `experiment/frontier-core` | CPU先验地图辅助前沿算法；观察网格与先验分离，评审/修正后才能实跑。 |
| `soarm-car-exploration-runtime` | `experiment/exploration-runtime` | 在原消费者任务选择处扩展探索，单一共享动作传输；等待Intel与核心验收后进行真实端到端验证。 |

验收契约：集成工作区 `docs/research/intel-exploration-contract.md`。模式明确为先验地图辅助探索，不称为未知环境SLAM。

### 本批辅助实验工作树（均在 `/home/gxxl/`，已合并者优先使用集成工作区）

| 工作树 | 分支 / 提交 | 状态 |
|---|---|---|
| `soarm-car-semantic-memory` | `experiment/semantic-memory` / `dca1be2` | 已评审、实测并合并；11观测记忆与过期/冲突处理。 |
| `soarm-car-vlm-crops` | `experiment/vlm-crop-context` / `7c95161` | 已合并；16次实测，发现编号敏感性，未证实语义增益。 |
| `soarm-car-nav-request` | `experiment/nav-request-identity` / `e2fe305` | 已修正并合并；真实ROS Goal序列化和伪造请求拒绝，不发送动作。 |
| `soarm-car-assistant-inspector` | `experiment/assistant-inspector` / `e788dbd` | UI初始分支，已合并；**运行用集成工作区**，其中已改为通用产品命名并修正HTTP HEAD响应。原8765服务已停止。 |
| `soarm-car-nav-service` | `experiment/reuse-nav-service` / `63b98e1` | 原始消费者 room-1→hub/mian 往返实际成功；只在独立运行副本添加NVIDIA/X11/EGL启动环境补丁，原导航实现不变。两次启动失败及最终成功均保留，已合并 `7c4046e`。 |

临时恢复验证工作树在 `/tmp/soarm-nav2-clean-restore-41463f9` 和 `/tmp/soarm-nav2-clean-parent-restore-20260909`，不是新导航实现。其他 worktree（如 Astra、rethink-test）通过 `git worktree list` 查询。

## 已有导航：查阅这些入口，不另写一套

工作目录：`/home/gxxl/soarm-car-nav2-clean`。

- 包启动入口：`ros2 launch soarm_navigation house_fastlio_nav2.launch.py`。这是容器内完成依赖/工作空间加载后的入口，**不是主机上直接执行的完整启动命令**。
- 现有编排脚本：`scripts/run-nav2-clean-comparison.sh`。
- 现有验证脚本：`scripts/dev-fastlio-house-nav2-smoke.sh`。
- 配置：`ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml`；行为树 `navigate_holonomic.xml`；需要保留该工作区的当前配套修改。
- 真实底盘下位机交付物：`artifacts/下位机代码+测试脚本+接口文档.zip`。该资料不是当前 Kuruma/Gazebo 仿真驱动，而是未来物理麦轮底盘的 STM32F405RGTx 固件、CAN 电机控制、上位机测试脚本和协议说明；ROS 2 应通过串口适配节点与其通信，不能把仿真 `ros2_control` 接口视为已接通实物。
- 已核对的物理接口：STM32 `USART1`（PA9/PA10），115200-8N1、无流控、小端、二进制帧；帧格式 `AA 55 LEN CMD DATA CRC FE`，CRC 为 `LEN+CMD+DATA` 按字节 XOR，`LEN` 只能为 1–64，解析错误静默丢弃。核心命令为 `0x00` 心跳、`0x01` 速度（`float vx/vy/wz`，vx/vy 为 mm/s，wz 为尚未标定的 mrad/s）、`0x02` 模式、`0x03` 停止、`0x04` 相对位移（mm）、`0x10` 状态查询；应答为 `0x11` 状态和 `0x12` 位移完成。
- ROS 接入硬约束：速度命令必须小于 500 ms 周期，建议 50–100 ms；进入/退出 `MODE_PC` 前必须先 `CMD_STOP`，退出交还遥控器；速度控制和位移队列不可混用。位移队列最多 8 段，只有成功完成才发 ACK，当前无失败/超时上报，且仅在全程不旋转时适用。心跳与位移共用超时计时器，位移执行期间不能发送心跳；状态查询不宜高频（固件应答会在中断上下文阻塞约 5 ms）。
- 当前固件状态字段中只有 `mode`、4 路 `w_target` 和 4 路 `w_actual` 可靠；速度、电池、status、error 字段目前是预留/恒零。`wz` 标定、位移阻塞/打滑检测、失败上报和电压采集仍是实物联调验收项。上层仍必须保持 `Nav2/teleop → 安全门（唯一 `/safe_cmd_vel`）→ 物理串口适配器 → 下位机`，不得直接暴露串口帧或绕过停止/超时互锁。

- 对应场景：`house_floorplan_ros2_control.sdf`；占据地图 `house_floorplan.{yaml,pgm}`；定位和房间数据由现有脚本选择（包括 `artifacts/house_widened_reference.pcd`、`artifacts/rooms-complete-reference.json`）。**不可套用到 AWS 或 HM3D 地图。**
- 成功记录：`artifacts/nav2-clean-direct-mppi-20260905T173147Z/summary.json`。已直接读取 `cases.direct_mppi.complete=true`、`passed=true`，分类为 `saved multiroom route passed with healthy localization`。这是历史真实运行记录，不是本次重新运行；其他尝试也有失败，不承诺每次成功。

### 恢复风险，不能忽略

该导航 worktree 当前存在已修改的控制器、启动文件、地图/world、定位配置，以及未跟踪的运行脚本和 `artifacts/`。因此 **`41463f9` 只是基点，不是完整成功快照**。后续如需迁移，先核对并单独归档现有工作状态；不能直接 checkout HEAD 后宣称已完整复现。此次只建立索引，未提交这些用户实现或改变其运行方式。

## 当前家具住宅演示

工作目录：`/home/gxxl/soarm-car-companion-validation`。

```bash
./scripts/start-companion-home-demo.sh
./scripts/companion-home-demo-control.sh forward
./scripts/companion-home-demo-control.sh left
./scripts/companion-home-demo-control.sh turn-left
./scripts/companion-home-demo-control.sh stop
./scripts/companion-home-demo-control.sh stop-demo
```

仅控制自有隔离容器 `soarm-companion-home-demo`。操作前确认当前实例及周围空间，避免重复启动。原始运行和操作说明见 `docs/companion-home-demo.md`；球轮运动观测见 `docs/companion-home-demo-observed-results.json`。这套手动 GUI 演示不等于原有自动导航基线。

## 接下来采用的接口边界

`文字/语音任务 → VL/语义记忆给出目标候选 → 同坐标系栅格及已有导航接口检查 → 原有导航执行 → 相机确认/反馈`。

VL 不直接发送底盘速度；不把单帧未知区域当作自由空间；未验证地图坐标转换时不混用旧地图。先修正图像输入、提示和目标约束，只有明确存在模型能力缺口且有独立数据时才考虑微调。历史研究分支结果保持不变，新开发另行标记。
