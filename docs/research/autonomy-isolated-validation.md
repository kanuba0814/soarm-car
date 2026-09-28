# 自主任务探索性验证与进程隔离研究

本报告研究 `/home/gxxl/soarm-car` 的当前未提交源码。主工作区与本 worktree 的 HEAD 均为 `41463f9a07dc317eb837558ad55b137fb92e35eb`，但 HEAD 不含 `soarm_room_navigation` 和 `docs/room-navigation.md`；不能用 worktree 原始 HEAD 的测试结果代替当前实现。未修改主工作区，仅在本 worktree 输出报告。

## 验证范围与证据

证据目录：`/tmp/soarm-gradexp-rem4bcak/`。其中 `source-status.txt` 记录主工作区脏状态，`source-sha256.json` 记录执行前复制源码的 SHA-256；`source/` 是独立可写快照，包含当前 ROS 源码、vendor、脚本、配置和必要 PCD，排除 Python 缓存。临时证据不保证跨机器或清理后留存。父会话已将关键日志、源码哈希、阶段结果和清理记录归档到 [`sources/graduation-autonomy/`](sources/graduation-autonomy/)，其中首次导航完整容器日志改名为 `navigation-first-container.log`。

选择两个已有验证，按依赖顺序执行，不新建测试框架，不重复父任务的纯 Python 测试：

1. `scripts/dev-localization-safety-smoke.sh`：真实 rclpy/消息/service/action 边界、真实 gate、受控假 action server。
2. `scripts/dev-room-navigation-smoke.sh`：真实 HTTP adapter → 本地协议 fixture → 任务控制器 → FAST-LIO/Gazebo/Nav2 → 真实 action 结果及 `/safe_cmd_vel`。fixture 不是真实 LLM。

原脚本无法直接满足本次隔离约束：`compose.yaml` 配置 `network_mode: host`、`ipc: host`、`/dev/dri`、X11 和主源码读写挂载。`docker/entrypoint.sh` 已将主要 build/install/log 放进容器 `/tmp/soarm_ros2`，但 `--symlink-install` 及 Python 运行仍可能触及源码树，故仍使用临时快照。

临时 `source/scripts/gradexp-safety.sh`、`gradexp-navigation.sh` 只适配运行环境，保留原始脚本以便对照。安全 smoke 仅构建 `soarm_interfaces`，以 `PYTHONPATH` 导入快照中的 gate；导航 smoke 使用原镜像入口构建，删除其中父任务已负责的 pytest 调用。fixture 移进导航容器 loopback，GUI/probe 也在同一容器内访问，主机没有模型服务进程。

## 实测结果

- **通过：真实 ROS 安全 smoke。** 健康状态命令 0.25 放行；操作员 stop 服务成功且真实 action 终态为 CANCELED；最终输出为零；DEGRADED 期间提交的新 goal 被取消；恢复 HEALTHY 后显式 arm 成功；arm 本身不恢复旧命令，发布新命令 0.33 后才观察到放行。耗时 10.65 秒，退出码 0，证据 `safety.log`、`results-attempt2.json`。
- **环境入口失败后修正：第一次选择构建 `soarm_interfaces soarm_navigation`，缺少后者依赖包的安装产物而退出，未进入 ROS 安全断言。** 改为只构建消息接口和直接导入当前 gate，使用新的 `...-safety2` 容器后通过。证据 `safety-build-attempt1.log`、`results-attempt1.json`。这不是安全逻辑失败，不隐去失败尝试。
- **Nav2 首次环境失败：** 本地全量源码构建完成，Gazebo `ign gazebo -r -s` 在未提供显示/设备的环境下发生 OGRE `RenderingAPIException: No Interface could be loaded` 并退出 134；ROS 随后无法确定传感器 topic 类型，阶段退出 1，耗时 138.40 秒。没有进入 HTTP/导航成功断言，不能判作任务编排逻辑失败。证据 `navigation-attempt1.log`、`navigation-evidence/container.log`、`results-attempt2.json`。本地探针确认 `ign gazebo --headless-rendering` 和 Mesa `swrast_dri.so` 已有，Xvfb 不存在；随后在新容器 `...-navigation2` 添加 `gz_args:='-r -s --headless-rendering'`，**按父任务要求在构建阶段中止，未验证 headless 修正效果及 Nav2 消费者。** 父任务报告执行 guard 的连接/超时问题，要求停止新增探索并立即有界收尾；本 lane 的日志保存与定名清理均执行成功。中止时日志止于 `Starting >>> soarm_fixed_map_localization`，没有 HTTP/Nav2 成功证据。证据 `navigation2-interrupted-container.log`。
- **未验证：** 真实 LLM 中文理解/泛化、ASR、RGB-D 目标搜索、任意房间覆盖、桌面停靠、物理急停与制动距离、硬件、最坏时延、断电恢复。本次未调用远程模型或硬件。

安全 smoke 的 action server 是测试替身，因此取消协议通过不等于 Nav2 所有行为树已通过。脚本先 stop 再发布 DEGRADED，不能把它解读为“单独由降级触发已运行目标取消”的独立实验；它直接验证的是 stop 取消和降级后新增目标取消。

## `room_search` 与导航编排的证据边界

`docs/room-search.md` 与 `soarm_room_search/http_gui.py` 明确将无硬件演示标为 `SIMULATION · MOCK PATH/RGB-D`；HTTP `analyze` 分支注入固定候选，`controller.py` 有 mock path verifier。因此 `scripts/dev-room-search-smoke.sh` 的 409/ARRIVED/cancel 只能证明 HTTP 阶段门控，价值低于本次两个 ROS 消费者验证，不额外运行并重复计数。

真实搜索入口在 `soarm_room_search/ros_node.py`：使用 `NavigateToPose` 和 `ComputePathToPose` action client、保存 goal handle 并取消，点云/TF/扫描 telemetry 才是感知证据。当前这条 RGB-D 搜索链不属于本次动态验证范围，不能从导航 roundtrip 推出桌面搜索成功。

当前导航控制器在 `soarm_room_navigation/controller.py` 维护 pending/handle、取消终态、确认身份、journal 和 SETTLING；`docs/room-navigation.md` 说明 stop 期间撤销超时 fallback、显式 rearm 不重放、position-only 行为树用于任意朝向到达和返回起点。这些是后续实验应针对的实现不变量；导航 smoke 的设计覆盖正常往返和运动中停止，但本轮未运行到这些断言；不能据此声称覆盖成功。本轮通过的是前述独立 ROS 安全 smoke，不覆盖全部并发、故障与恢复组合。

## 不打架的具体操作规则

1. **单一 ROS lane 与所有权清单。** 本次在启动前通知父任务，唯一拥有 `soarm-gradexp-rem4bcak-*` 容器；不停止或 exec 其他容器。记录前后容器、网络、进程快照；名称固定且唯一，禁止 pkill/killall、docker prune、按模糊搜索批量删除。
2. **源码快照与产物隔离。** 从未提交主工作区复制当前文件并保留 hash；只挂载快照到 `/workspaces/soarm-car`。生成目录位于容器 `/tmp` 或专用快照，父任务可独立跑 Python 测试，不能共享 ROS build/install/log。
3. **网络、IPC、域、分区分层隔离。** `--network none --ipc private` 给每个容器独立网络命名空间，仅保留自身 loopback；没有 host 网络、端口发布、外部服务、共享内存或设备。安全 ROS_DOMAIN_ID=211，导航=212；IGN_PARTITION 使用各自唯一容器名。若未来跨容器测试，创建同前缀的专用 bridge，禁止退回 host 网络。
4. **新实验新进程。** 安全初次尝试 `...-safety`，修正后 `...-safety2`，导航 `...-navigation` 和 headless 重试 `...-navigation2`，纯依赖探针 `...-probe`、`...-probe2`。每轮独立 journal/rooms 文件，不复用上次任务状态或 action server。
5. **限定资源与时间。** 复用本地 `soarm-room-search-ros2-dev:latest`（镜像 ID `a022a3b21155`），全部 `--pull never`。依赖探针 35 秒、安全阶段 180 秒、导航阶段 650 秒；外层 `timeout --signal=TERM --kill-after=15`，监督器另设有限 deadline。导航首次构建启动后将本次容器限制为 4 CPU、10 GiB 内存、12 GiB memory+swap；headless 重试在 docker run 时即指定相同限制以覆盖启动瞬间。
6. **清理由所有者 finally 完成。** 先保存日志，再对完整确定名称 `docker stop -t 10`、`docker rm`；嵌套 fixture 随导航容器退出，无独立主机 fixture PID。内层脚本 EXIT trap 加外层 finally 双重兜底。监督器 PID 写入 `runner.pid`。本次提前中止时先保存日志，终止自有监督器 21396 并清理 navigation2；复核发现 GNU timeout 及脚本 21397/21398 仍存活，校验 `/proc/PID/cmdline` 属于本次脚本后分别 SIGKILL，再次确认无活进程。不清理其他会话进程。提前中止不能假设父监督器的 finally 一定执行，必须由收尾者显式兜底。

本次 GUI=19194、fixture=19195，均为容器内 127.0.0.1；由于无发布端口，宿主机同端口也不会冲突。原脚本的主机端口 bind 探针保留但不能替代容器网络隔离证据；`navigation-inspect.json` 记录实际无 devices、无 PortBindings、private IPC 和唯一快照挂载。

## 最终资源核对

`cleanup-final.json` 记录最终核对：本次前缀容器剩余 **0**，监督器 21396 已不存在，自有 timeout/脚本 21397/21398 无活进程。`containers-before.txt` 与 `containers-after.txt`、`networks-before.txt` 与 `networks-after.txt` 均逐字相同；未创建 Docker network 对象（使用各容器独立 `none` 网络命名空间）。原有七个停止容器保持原状态：`soarm-astra-camera`、`soarm-robot-observability-dev`、`raicom-2026-oryx-navigation`、`rei-sim`、`mineru-gradio`、`mineru-api`、`autoware_env_v2_host`。本次没有主机 fixture 服务，没有发布端口。

`main-source-recheck.json` 显示快照中各原始文件与主工作区重新计算的内容哈希差异为空；主工作区未被本 lane 修改。临时源码、日志和报告保留供复核。父任务另报告主工作区 **38 tests + 2 subtests 通过**；这是父任务提供的结果，本 lane 没有重跑，也不据此宣称 Nav2 消费者通过。

## 毕业设计实验建议

优先复用已有编排与消费者脚本，按可观察结果分层记录，避免把 mock 演示、协议兼容、真实语言理解和真实运动混在同一个“成功率”里。

| 实验层 | 最小任务/故障 | 可报告指标 | 禁止外推 |
|---|---|---|---|
| 编排安全 | 正常多步、等待、停止、取消后显式 arm；后续增加 planner 迟返/重复确认/超时取消时 stop | 拒绝原因、goal UUID、终态、旧计划未重放、零输出观测时延 | ROS 零输出不等于物理制动 |
| 仿真导航 | 当前往返；后续按房间距离、转角、固定/任意朝向分组 | Nav2 终态、任务完成率、位置误差、耗时、停止时延分布 | 单次近距离往返不等于全屋覆盖 |
| 真实模型（另行授权） | 固定已知房间目录的中文指令集、歧义/越权/未知房间 | schema 合法率、语义任务正确率、澄清率、拒绝率、模型耗时 | fixture 的正确输出不是模型准确率 |
| 搜索感知 | ROS PointCloud2→TF→桌面候选→ComputePathToPose→确认停靠 | 检出/误检、可达性、停靠终态、真实扫描覆盖 | mock RGB-D 的 ARRIVED 不是感知/停靠实测 |

每次实验保存源 hash、镜像 ID、地图 bundle、随机种子（若适用）、ROS 域/分区、目标序列和终态时间线；先小批量发现失败类型，再在固定条件下报告重复次数和分布。当前无权启动硬件或真实 LLM，这些应作为明确未验证项和后续实验边界。
