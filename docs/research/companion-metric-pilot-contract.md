# 当前唯一建议下发的首阶段合同：真实 RGB-D / 独立位姿开发小试

根据主代理最新 steering，完整 A1/A2 合同仅作后续参考，当前只执行以下有界阶段。不把完整论文架构、Nav2 接入或完善全场景 GT 作为首个真实小试的前置条件。

Outcome:
在 /home/gxxl/soarm-car-companion-validation @481eed1 上实现并实际运行一次开发集采集小试，交付最多 12 个不同视角的真实 RGB、米制原始深度、完整标定、独立观测位姿、冻结 YOLO-World 输出，以及消费端重载/反投影与跨视角几何诊断。结果可失败，不能虚构通过；不产生导航或方法精度声明。

Binding decisions:
1. docs/research/companion-thesis-execution.md:5 已授权主代理本地无人值守仿真与方法实验；本阶段不再要求用户确认。
2. 使用现有固定场景、原有低位相机和已锁 YOLO 权重；所有已看过的图像/原始布局属于开发小试，不进入未见测试。
3. set_pose 只能做记录清楚的诊断放置；请求坐标不能充当实际位姿，不能计入导航路径。
4. 机器人自身的仿真真值位姿本阶段仅用于独立观测和几何诊断，不把它悄悄接给正式 planner。后续是否采用已知自身位姿的受控方法研究，由主代理在看到小试后显式决定。
5. 原始指标、失败和缺失项全部保留，不生成替代检测或真值分数。

Current scope:
这是主代理最新收缩：先一个真实小试，再迭代研究。当前不实现 Nav2、新地图、全场景实例分割评分器、300 条实验或论文结论。保留后续 A1/A2 目标，不将本阶段声称为其完成。

Targets and authority:
唯一 writer 工作树 /home/gxxl/soarm-car-companion-validation，写入前核对干净基线。最小改动接缝为 scripts/dev-companion-home-validation.sh（已有隔离容器/传感器采集）、scripts/prepare_companion_home_world.py（已有场景包装）、scripts/run_companion_yoloworld_smoke.py（已有冻结推理），可新增 scripts/companion_capture.py 将原内嵌采集逻辑提取为有界模块，并新增少量开发配置/证据说明。新增文件名是建议，不是声称现有接口。
禁止改主工作树、资产 clone、原底盘/相机模型、FAST-LIO、安全链、生产 Nav2 配置；禁止硬件、外部推送上传和破坏性清理。新数据使用独占未存在输出目录，不覆盖旧 run-final。复用 network none / ROS_LOCALHOST_ONLY / IGN_PARTITION 隔离。只清理本次拥有的容器。

Execution:
1. 复用现有显示支持的 Fortress 启动链和锁定资产。不要调用旧导航 smoke（不同场景且越过本阶段范围）。先保存真实主题列表和消息类型。
2. 独立位姿的首选探针来自 scripts/dev-fastlio-house-nav2-smoke.sh:186-195，旧场景读取 dynamic_pose/info；当前包装 world=default，先验证 /world/default/dynamic_pose/info 是否实际存在并带可解释仿真时间及 kuruma 实体。它是待测候选，不能假定可用。必要时用原生 /gazebo/odom 作交叉检查；kuruma/model.sdf:17 已配置该发布器，但 companion bridge 当前没有桥接。验证 frame/origin/base 定义，不能仅改 frame_id。
3. 保存原 RGB、float32 米制深度/无效掩码、完整 CameraInfo K/P/D/R、encoding/step/endianness、三者时间戳与 frame IDs、原始姿态流、requested 与 observed 两组位姿、消费图像对应的变换链。保留现有 >pose-fence +0.5s 和 RGB/depth <=50ms 条件；要求观测位姿与图像时间关联，不能用图像后单独一次 CLI 取样代替。
4. 相机名义链是 model->base z=.0763，再 base->camera (.180,0,.150)，不是简单在请求 z 上加 .150；实际 model z 用观测值。验证 optical frame 的轴向与深度单位。图像行填充与 endian 必须正确解析，原始深度不能由现有 uint8 深度预览恢复。
5. 先采 3 个不同位姿/朝向，确认链路，再扩展到最多 12 个不同开发视角；至少包含非共线平移和约 90 度朝向变化，以及有静态表面重叠的一对。每视角仅允许一次有限重试。采集进程总计最长一小时墙钟，单次等待最多 15 秒，启动等待有界。诊断位置不是安全导航目标，实际发生穿透/失稳必须剔为无效并留痕。
6. 用真正下游消费者重新打开保存的米制深度和 CameraInfo，反投影出点云，应用实测位姿；在重叠表面计算/可视化残差，保留真实对应点与覆盖比例。几何验证不需要先写全场景 GT mesh parser。阈值建议 median <=.03m、P95 <=.10m，仅对有可靠对应的静态表面；没有足够对应则 NOT_VERIFIED，不能强配近邻得到虚低误差。分别输出点云和 RGB/depth 对齐图；坐标轴、单位和重复高度误差必须暴露。
7. 原始 RGB 交给现有冻结 YOLOWorld（本地已验证 SHA 的 yolov8s-worldv2.pt），保存全部框/置信度与叠图。消费至少 3 张原始帧。检查是否观测到两个不同椅子候选，以及独立的桌/沙发等关系锚点；报告这一先决条件成立/不成立及对应证据，不把 SDF 名称变成检测结果，也不作精度计算。
8. 输出一份简短中文开发小试报告：哪些数据/几何证据通过，哪些失败，原始图像/深度/位姿/推理结果链接与哈希，准确重跑命令，以及下一步 A1 的实际输入条件。保留相机安装、模型及源世界 hash 和环境版本。检查最终 diff；只跑直接相关消费端验证，不新建通用测试框架。

Acceptance matrix:
ID | 独立可观测验收 | 反例/失败必须保留
C1 | 真实隔离 Gazebo->bridge->RGB/depth/CameraInfo->磁盘文件完成，资产和输入 hash 对应 | 假图像、旧 run 图片冒充本次、仅 topic list 不通过。
C2 | 原始米制深度重载与采集数组/无效掩码一致，完整标定可读 | 只有每帧归一化的 PNG 不通过。
C3 | 至少 3 个多样位姿有独立实测姿态和图像时间关联；最多 12 视角完整记录 | 请求位姿、自写期望坐标、无时间关联的单点快照不通过。
C4 | 保存正确的相机链及 optical 约定，真实重叠表面验证给出覆盖与残差/诊断图 | 只检验平滑性、同一拟合表面自证或无对应却给数值不通过。
C5 | 相同锁定检测器实际消费新 RGB、保存全部原始框和输出 hash | 资产 GT 生成检测、过滤误检后假称原始输出不通过。
C6 | 小试报告明确诊断传送、开发集、无导航/无方法准确率；最多12视角、重试/时间有界 | 把相邻帧当独立布局、把放置当导航或把计划当结果不通过。
C7 | 新目录可重载数据并重复消费端结果，已知缺口给出精确失败原因 | 只有 prose、hash 无可恢复数据或没有消费者输出不通过。

Consumer-seam validation:
必须实际贯通 Gazebo rgbd_camera -> ros_gz_bridge -> subscriber -> float32 depth/calibration/pose persistence -> independent reload/backprojection/transform diagnostics -> frozen YOLO on the same saved RGB。仅通过 Python 语法/JSON schema 不算完成。

Stop/escalate:
位姿主题/时钟无法核验、RGB-D消费失败、坐标约定无可靠证据，优先做局部诊断并保留输出；涉及更换传感器、加入新编码器、Nav2/SLAM、依赖全场景 GT 才能继续时，返回主代理具体选择，不扩大本阶段。若检测不到锚点，完成采集/几何结果并报告 A1 先决条件失败，不能制造目标。

Next stage, not current execution:
由小试决定 A1 的真实任务类别/关系锚点及 GT 标注规模；确认 VisualConf 基线命名与是否允许明确标注的已知自身位姿条件。A2 仍需 AWS 传感器地图、正确 footprint、真实 ComputePathToPose 路径和物理驱动/碰撞/停稳证据。完整候选后续合同见 /tmp/companion-oracle-contract-481eed1-u8l89e7u.md；其中 300 条均为未执行计划，不能作为启动首阶段前的全部门槛。
