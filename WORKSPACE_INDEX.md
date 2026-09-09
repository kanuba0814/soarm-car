# 工作区索引与复用边界

核对日期：2026-09-09。路径均为本机现有 Git worktree；HEAD 是核对时的基点，不代表包含未提交实现。

## 首要约定

- **当前资源约束（2026-09-09最新核对）：仅Intel核显** `8086:7d51`（Arc Pro 130T/140T），DRI `card1/renderD128`；NVIDIA已不存在。新运行禁止沿用 `--gpus all`、NVIDIA GLX/EGL或CUDA假设。先验证实际Intel renderer；CPU软件渲染须如实标注。一次只跑一套仿真，CPU实验可适度并行，模型推理不与仿真争用核显。历史NVIDIA结果仅是历史记录。

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

### Intel自主探索批次（进行中）

| 工作树 | 分支 | 当前职责 |
|---|---|---|
| `soarm-car-intel-runtime` | `experiment/intel-runtime` | Intel渲染与传感器验证、运行环境交接；独占仿真直到明确释放。不得使用历史NVIDIA启动补丁。 |
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
- 机器人：`ros2_ws/src/soarm_simulation/models/kuruma_ros2_control/model.sdf`。已直接核对四轮 visual/collision 为半径 `0.0763 m` 球体，使用 `gz_ros2_control-system`，控制入口重映射至 `/safe_cmd_vel`。
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
