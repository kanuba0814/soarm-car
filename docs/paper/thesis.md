# 论文基础草稿（未完成稿）

## 暂定题目

**面向家庭服务机器人的关系约束视觉目标定位与可达观察位选择：受控仿真平台、评估隔离与验证方案**

## 摘要（暂定）

家庭服务机器人接受“到沙发旁的小桌子确认水杯是否仍在”等指令时，不能把类别检索、到达某一坐标和目标确认混为同一件事。该任务至少包含指定实例的视觉定位、可行终端观察位选择，以及到位后的新观测确认。本文拟研究关系约束与去相关多视角证据是否能降低同类物体情境中的错目标风险，并研究在底盘可达性和固定资源预算约束下，观察位排序是否能提升可见终点确认。本文当前交付的是研究基础而非完成论文：已完成的量化证据仅来自隔离的 Gazebo Fortress RGB-D 消费链、机器人自身仿真位姿插值、几何重放诊断、两轴相机约定校准，以及评估端私有的可见实例真值解码。它们证明了若干后续实验的测量与隔离前提，**不**证明所提方法有效、导航成功、关系理解正确或真实硬件可用。方法比较、统计检验、导航结果和结论均须待冻结数据划分后据实际计算结果补写。

**关键词：** 家庭服务机器人；视觉语义导航；指定实例；主动观察；评估隔离；RGB-D

## 1. 引言

自然语言目标在家庭环境中常以属性、房间和相对关系限定实例，而不是只给出物体类别。例如，两个椅子都可被检测到时，“餐桌旁的椅子”要求系统保存关系证据并在证据不足时保留未知。即使对象候选正确，物体质心也未必是移动底盘的合法终点：足迹碰撞、占据栅格、路径可达性、地图版本及相机到地图变换都构成硬约束。到达后还应以新的稳定观测确认指定实例，不能以 Nav2 到位或旧地图中曾出现该物体替代确认。

本研究拟在现有 FAST-LIO/Nav2 作为基础设施的前提下，考察两个受限模块：A1 为关系证据和多视角支持度的候选重排序；A2 为安全可达观察位集合内、兼顾可见性代理、信息收益代理与路径代价的选择。研究不重新实现 SLAM，也不主张“纯视觉定位”或无激光导航。语音和闲聊入口亦不是算法贡献；运动必须仍受现有预览、确认、停止、取消和重新使能边界约束。

该切片的必要性在于：语义地图、开放词汇对象图、层级检索、语言引导导航及对话消歧均已有代表性工作，不能把它们的组合包装为先验创新。更严格地说，若关系重排序在关系任务上未改善，若等预算观察位策略无增益，或收益来自隐藏真值泄漏、额外模型调用或额外行走，则相应假设应被拒绝而非事后修改叙述。

## 2. 相关工作与差异边界

VLMaps 将预训练视觉语言特征融入三维重建并支持语言索引空间地图，且其实现要求 RGB、米制深度、位姿和相机标定输入 \cite{vlmaps2023}。它为同编码器的语义候选检索基线提供直接参照，但不等于关系指定实例的终端确认。ConceptGraphs 通过有位姿 RGB-D 序列的多视角关联构建开放词汇三维场景图 \cite{conceptgraphs2023}；因此，使用对象图或多帧融合本身不构成本文贡献。HOV-SG 已研究建筑、楼层、房间和物体层级的开放词汇场景图与长程导航 \cite{hovsg2024}，房间约束应被当作已有表示或基线因素。

面向语言和场景上下文的具身导航也并非空白。REVERIE 讨论远程具身视觉指代表达导航与目标落地 \cite{reverie2020}，SOON 给出带图探索的场景导向对象导航任务 \cite{soon2021}；二者的预采集仿真、观察和动作假设与本地 Gazebo 底盘、占据地图及新帧确认条件不同，故不直接比较数值。ZSON 提供以多模态目标嵌入实现零样本对象目标导航的参照 \cite{zson2022}，可启发冻结编码器的最近候选 B0，而非要求迁移其全部训练栈。CoIN 已把导航中的目标实例不确定性与开放式人机对话结合 \cite{coin2025}，故“检测到不确定就询问用户”不能作为独立创新；本研究首先固定文本输入或将交互单列为对照。

最新核查进一步限制 A2 的表述。SAP-Nav 已采用环状候选观察位、可导航区域筛选、射线可见覆盖、到达后再验证与有限重试 \cite{sapnav2026}。因此，“环状采样 + 可见性评分 + 新视图验证”不提出新颖性主张；若仅实现其局部公式，只能称为 SAP-Nav-inspired 几何基线，不能称完整复现。本文仅把代价敏感选择、去相关支持、缺失关系证据和最新终点身份不变量作为待测差异，未宣称首创。

## 3. 研究问题与可证伪假设

**RQ1：** 在同一视觉编码器、候选生成器、地图与最终确认器下，关系约束和去相关多视角证据能否改善指定实例定位，而不是只提升相邻重复帧的置信？

**H1：** 对同类歧义、关系约束和遮挡任务，A1 相较纯相似度 B0 提高 Top-1 或 MRR，并在接受样本上降低错目标率；不存在目标时不以强行选择换取表面成功。候选分数可定义为 `s=w_vV+w_rR+w_mM-w_uU`，其中关系缺失为未知而非满足，连续近邻帧不视为独立支持，深度/位姿或缺失证据进入惩罚项。未经开发集校准，该分数不得称为概率。反例是同一误检连续出现二十帧仍被错误累积为高置信。

**RQ2：** 在相同路径、观察次数、模型调用和时限预算下，合法观察位的效用排序能否改善新帧确认任务的成功与效率？

**H2：** A2 在先经 footprint、occupancy、路径、地图身份与变换新鲜度硬筛的集合中，按可见性/信息收益代理与路径代价排序，相较最近安全位、固定序、随机安全位和匹配预算的重复观察对照，提高合法观察位成功或确认目标成功，且不以额外预算解释收益。候选集合为空时必须报告不可达或待核验，不得静默换成同名对象。可见性代理不可读取仿真隐藏标签；真值只用于额外 oracle 诊断。

任务成功定义为：在预定预算内到达合法观察位，并由到位后的新稳定 RGB-D 帧正确确认指定实例。导航到位成功、视觉确认成功和总体任务成功分别报告。SPL 仅在已知从同一起点到“合法成功观察位集合”的最短可达路径时计算；不存在、不可达和歧义任务单独报告正确拒绝、错误成功与弃权，不在事后剔除。所有阈值只在开发集冻结，按场景或布局组划分，禁止同一录像相邻帧随机拆分；同一任务、布局和随机种子在各方法间配对，并按场景聚合及 bootstrap 区间。

公平比较要求四组共享视觉输入、检测器、地图、候选生成器、低层控制器及最终确认器；仅改变被检验的定位重排序或观察位策略。预算必须把实际行走距离、停稳后观察次数、视觉或语言模型调用和总时限一并计入。每组均拥有相同次数的有界再观察和同样的拒绝权限，避免“少执行”被误报为更安全，或“多看几次”被归因于方法设计。测试集一经查看不得反向调节权重、阈值或任务定义；若最短路径、可见性或实例身份只能由私有真值得出，应将其标为 evaluator/oracle 诊断，而不能进入可部署指标。

## 4. 实验平台、数据来源与当前证据

平台是受控仿真，不是实物机器人试验。当前真实消费者链为 `Gazebo Fortress rgbd_camera -> ros_gz_bridge -> rclpy persistence -> persistent native Pose_V -> source-time interpolation -> reload z-buffer diagnostic -> frozen YOLO-World`。运行时容器维持 `--network none`、`ROS_LOCALHOST_ONLY=1`、独占 `IGN_PARTITION` 与 `IGN_IP=127.0.0.1`；Kuruma 原始 SDF 未被修改。因而以下“真实”仅指真实执行并保存的仿真传感器、原生姿态流和下游解码器，不指真实世界硬件采集。

在 `companion-first-pilot.md`（验证工作树提交 `370b138`，相关原始运行标识 `a27e139`、`e8974b6`）记录的四个开发视角中，连续原生姿态流对每帧均给出 17 ms 的 source-time bracket、31 个稳定窗样本、最大流间隔 17--18 ms、最大平移 0.48--0.77 mm、最大旋转 0.00177--0.00317 rad；这是已冻结 C3 门槛下的**机器人自身仿真位姿**条件。它不进入 planner，不能当作真实定位精度或方法输入结果。

同一记录的重放几何诊断中，预选 `overlap_base -> overlap_forward` 有 56.82% supported unique-pixel coverage，计有 245 个 behind、97 个 foreground 和 35,782 个描述性 depth-compatible 像素；后者绝对中位残差为 0.000791 m、P95 为 0.02150 m。该 Z-buffer 消费者重放拒绝 nonfinite/相机后点，并以每个目标像素最近源深度处理遮挡。数值只是该受控视角对的描述性诊断，不能被改写为家庭场景几何精度、地图精度、导航精度或视觉方法性能。

独立校准运行在运行时 wrapper 中加入固定绿色 X 平面与蓝色 Y 平面。冻结色掩码后，两个 yaw-zero 视角的 green-X 各超过 135k 像素，轴中位误差为 -0.00996 m 与 -0.01001 m；90 度视角的 blue-Y 为 97,824 像素，中位误差 -0.00994 m。这仅支持运行时声明的 transform/depth convention 在两个独立轴向与朝向上的一致性，不能替代家具环境 accuracy。冻结 YOLO-World 对四张 persistent home RGB 产生 9 个原始框，但无目标身份、关系锚点、准确率或测试划分，故不得从中报告检测性能。

## 5. 可见实例真值、隔离与数据划分

`companion-visible-instance-gt.md`（提交 `370b138`；采集接缝 `e8974b6`）描述的是 evaluator-only acquisition seam。生成的运行时 world 为 87 个家具 include 赋稳定唯一语义 ID，并使用与 RGB-D 共位的 640x480、FOV 1.047、近远裁剪 0.1/8 m、15 Hz segmentation sensor。下游 smoke 实际解码 `labels_map`：其为 `rgb8`、640 像素时 step 1920、语义 ID 在三字节通道重复；前景 label 251 存在而后方 250 缺失，符合实际遮挡，而非由框伪造 mask。这是 GT 解码消费端通过的证据。

八个预声明的开发观测 `obs_001`--`obs_008` 中，RGB/depth/CameraInfo/labels/colored-map 时间戳相等，且自位姿均通过持续插值稳定门槛；其中若干观测可见多个 `ChairA` 或 `KitchenTable_01_001`，`SofaC_01_001` 出现在四个观测。这些是特权、离线、开发调查，不是 detector 驱动策略、导航轨迹、关系评分或留出测试。公开包只能包含 RGB、米制深度、invalid mask、CameraInfo、source-time self pose 与不透明 observation ID；私有包保存解码 mask、标签映射、源实例位姿和匹配汇总。已执行的负泄漏检查要求公开树不含标签、私有路径、源实例名、源位姿或 GT mask。未来 policy/model 只能挂载公开包，不能访问 label topics、私有包、源 world 或实例元数据。

## 6. 冻结留出 A1 的实际结果与结果章节边界

本次有且仅有 A1 的离线指定实例评估结果。三个 `.tar.xz` 留出归档各含 8 个固定观测、6 个冻结查询和四个变体，故是 24 个视图、18 个“查询--布局条件”、72 个“变体--查询--布局行”。三个 layout 是同一 TeamSOBITS AWS Small House 的椅子位姿扰动，绝不是三个独立家庭；因此下述统计只作描述，不给出 bootstrap 显著性、置信区间或优越性结论。归档中私有 `evaluation.json.rows` 是表格唯一数值来源，不能以较早的 `paired-summary.json` 替代原始行。

实现的变体是 V、V+R、V+M、V+R+M，不是把四者合并成一个“方法”。令轨迹 t 的视觉分数为 V(t)，关系项为 R_q(t)，多视角支持项为 M(t)，并令 U_q(t) 表示任何缺失、污染或扰动不稳定的关系/身份条件。公共方法的排序形式是 s_q(t)=V(t)+I_R R_q(t)+I_M M(t)，其中 I_R、I_M 分别由变体启用；U 不被当成正关系证据。R 的状态可以是 measured、unknown 或 not-required；unknown 的原因直接留在行字段 `public_relation_reason`。深度前景由 RGB 框、米制深度、无效掩码、地面去除与连通分量产生；轨迹关联使用相机位姿、重投影、深度兼容和阈值门。M 仅对相机中心和方位足够分离的 view group 计入支持，避免把相邻重复帧当独立证据。以上为已实现的启发式分数，并非经校准概率，也未因留出结果调节权重或阈值。

私有 evaluator 先用类别一致且 IoU>=0.5 的确定性一对一最大权匹配将公共检测成分与可见实例关联。一个轨迹只有每个成分匹配同一实例且无未匹配成分时才是 verified；多个实例为 contaminated，任何未匹配成分为 uncertain。私有 observed-surface voxel、obs_001 朝向、双向第十百分位表面间隙和不确定性加宽仅在 evaluator 内产生 Q3--Q6 真值，绝不回流到公共排序。若 GT 不是 definite，所有正确性字段均为 unknown，不可记成功或失败。

### 6.1 指标、分母与不确定性

对 definite 行，forced Top-1 是强制取首位的结果；若首位身份 uncertain，该行的 forced Top-1 也 unknown。候选召回同样允许 unknown。对于排名，精确 MRR 只定义为 private `mrr` 非空的已知子集平均值；它绝不可当成所有 definite 行的点估计。每一 definite 行还给出 mrr_lower_bound 与 mrr_upper_bound：下界只把已验证正确候选计入，上界允许最靠前 uncertain 候选可能正确。表中报告这些逐行边界的均值及其分母。

接受（accepted）是公共策略输出 `accepted_track_id`，并不等同强制排名。accepted_track_id 为空是拒绝而非错误；若接受轨迹的私有身份 uncertain，`accepted_correct` 为 null。于是 unknown accepted 既不算正确也不算错误。覆盖率的分母为 definite 行；已知错误率下界为 known-wrong/accepted-total，上界为 (known-wrong+accepted-unknown)/accepted-total。没有接受时错误率为不适用。这个定义有意给出宽区间，防止以“没有已知错误”伪称安全。

### 6.2 按变体结果

生成表 `results/a1-heldout-by-variant.csv` 显示每个变体各有 18 行、11 definite、7 unknown GT。四个变体已知 forced Top-1 都是 3/8=0.375，余下 3 个 definite 行的首位身份不确定，故不应比较为性能差异。V 的 MRR 精确已知子集为 0.000（3/18 行），边界为 0.4697--0.6364；V+R 为 0.000（3/18），0.4621--0.6364；V+M 为 0.000（3/18），0.4697--0.6364；V+R+M 为 0.000（3/18），0.4621--0.6364。精确 MRR 子集太小且所有已知值为零，边界又大量由 uncertain identity 决定，不能把任何差异解释为 R 或 M 的收益。

V 接受 3/11 definite 行，三者均 identity-unknown，错误率界为 0--1；V+R 接受 2/11，同样为 0--1；V+M 接受 3/11，其中已知正确 1、已知错误 2、unknown 0，错误率为 2/3；V+R+M 接受 2/11，其中已知正确 1、unknown 1，错误率界为 0--1/2。四者每个都有 10/18 relation-unknown 行。这些数值不支持“关系项提高定位”“多视角项降低误接受”或任何 established superiority。图 `pervariant-outcomes.svg/png` 特意将 unknown GT 和 unknown accepted 标在柱外，柱的 stacked display 只为并列显示，不能被读为共同成功率。

### 6.3 按布局结果与失败分析

按 layout 汇总的 24 行不能跨变体当作 24 个独立任务。seed101 有 12 definite、12 unknown GT；已知 forced Top-1 为 8/12=0.6667，精确 MRR 为 0/4，边界均为 0.6667；无 accepted。relation unknown 是 16/24，归档摘要的主因是 `perturbation_geometry_failed`。seed103 有 16 definite、8 unknown GT；forced Top-1 为 4/16=0.25，精确 MRR 0/4，边界 0.5--0.5；接受 4/16，其中 2 个已知错误，错误率 0.5；relation unknown 16/24，主因 `missing_or_ambiguous_anchor`。seed107 有 16 definite、8 unknown GT；仅 4 个 forced Top-1 已知且正确 0，精确 MRR 0/4，边界 0.28125--0.75；接受 6/16，全部 identity-unknown，错误率 0--1；relation unknown 8/24，主因 `relation_order_changes_under_perturbation`。图 `perlayout-failure-coverage.svg/png` 分别显示 unknown relation 和 unknown GT 覆盖，不能由其推出布局难度排序。

这里还有一项必须公开的数值不一致：`paired-summary.json` 的每布局 `mrr_known` 为 0.0，却同时给出非零的 `mrr_lower_bound`/`mrr_upper_bound`；原始行解释该现象，因为可精确判分 MRR 的少数行均为 0，而其他 definite 行为 partial identity uncertainty，只贡献边界。生成器不复用该含义容易误读的字段，而是从行级 `mrr`、`mrr_lower_bound`、`mrr_upper_bound` 重新计算，并在 JSON 中保存分母。这不是修改冻结输出，而是报告它们的语义差异。

原始 `941ea2d` 的 `a1-visualconf-dev-v1` 含 constant-R、假组件代理和 all-false evaluator，是无效脚手架，绝不作为结果、覆盖率或失败分析来源。留出前修复也必须保留：`bde1f3e` 修正 RIGHT-positive 轴与关系扰动语义，`ca8d021` 将 MRR 上界改为 `max(mrr_lower, uncertain_best)`，并写入 v6 冻结配置。它们发生在本次留出归档前；本报告没有重跑、调参、改动方法或 evaluator。

A2、观察位选择、闭环导航、路径/碰撞/接管、端到端延迟、实物迁移和任何 Nav2 成功均没有本包结果。导航部分明确待单独的物理与运动安全结果，不能由这里的固定 survey RGB-D 或离线 evaluator 推出。真实硬件没有运动；家庭图像应本地受控保存，不上传第三方 API。

## 7. 伦理、局限与可复现性

该研究面对家庭空间和潜在个人图像，应实行最小化采集、受控本地保存、按任务删除或保留审批记录，并明确不把未获授权的图像上传给第三方服务。真实运动不得由闲聊触发；模型输出不得直接给任意速度或坐标。存在不确定、变换过期、地图不一致或无合法视角时，系统应拒绝、停止或要求人工重新确认。仿真隐藏实例标签和源位姿只服务评估，若泄漏到候选选择，将使全部方法结论失效。

局限还包括：当前只有小规模开发视角而非家庭多日、多光照或跨布局数据；YOLO 框不等于身份、关系或检测准确率；受控自位姿不代表真实定位；深度 compatible 不证明 same-surface；SAP-Nav 等近期工作使主动视角新颖性主张仍不成立。本文应按失败也可发表的原则保留零增益与反例，不因实验不支持假设而叠加无关模块。

## 附录 A. 可复现实验包与重放边界

编辑稿为 `docs/paper/thesis.md`，八条已引用的一手文献条目在 `docs/paper/references.bib`。生成入口为 `python3 scripts/generate_companion_paper_results.py`；它仅以 Python `tarfile` 读取 `artifacts/companion-home-validation/evidence/a1-heldout-results-v6/a1-heldout-seed101.tar.xz`、`seed103.tar.xz`、`seed107.tar.xz`，读取成员 `private-evaluation/evaluation.json` 和 `receipt.json`，不解压或写入历史归档。随后运行 `python3 scripts/build_companion_paper_html.py` 可得到没有 CDN、没有外部资源的 `docs/paper/paper.html`。表、原始行、JSON 和 SVG/PNG 图均在 `docs/paper/results/`；包索引、离线 public `bwrap`、private evaluator 与 scene regeneration 的正确边界见 `docs/paper/README.md`。

冻结配置是 `config/a1_visualconf_dev_v1.json` 的 v6 版本。运行时 asset/source 锁、哈希和许可证分别在 `docs/upstream-manifest.yaml`、`docs/research/sources/SHA256SUMS`、house 的 `SOURCE.md`、`vendor/wheeltec/SOURCE.md` 与随源 LICENSE 文件中。重放 public consumer 必须使用 `scripts/run_a1_heldout_testbatch.py` 所构造的只读 `/public`、`/input`、`/config.json` bwrap 边界，随后才允许独立 private evaluator。scene regeneration 使用 `scripts/prepare_a1_heldout_layout.py` 和 `config/a1_heldout_layouts_v1.json`，是 renderer-owner 的单独工作；本包不执行它，也不执行模拟、GPU、模型、导航或物理运动。

## 附录 B. 数值主张追溯表

| 数值或结论 | 归档 SHA-256 与 JSON 字段 | 生成代码提交 | 限制 |
|---|---|---|---|
| 72 行、24 views、18 query-layout conditions、4 variants、3 layouts | 三个 archive SHA 在 `results/a1-heldout-results.json.archives[].archive_sha256`；`private-evaluation/evaluation.json.rows` | 结果 JSON 的 `code_commit`，本地生成器 `scripts/generate_companion_paper_results.py` | 三 layout 是一个 AWS Small House，不是独立 homes |
| 按变体 Top-1、exact MRR、MRR bounds、accepted 及错误率 bounds | 各 archive `private-evaluation/evaluation.json.rows[].forced_top1/mrr/mrr_lower_bound/mrr_upper_bound/accepted_track_id/accepted_correct` | 同上；聚合输出 `a1-heldout-by-variant.csv` | exact MRR 只在 `mrr != null` 的 known subset；unknown accepted 不算正确或错误 |
| 按布局 definite/unknown、coverage、relation unknown 与失败原因 | 同一 `rows[].gt_status/public_relation_status/public_relation_reason` | 同上；`a1-heldout-by-layout.csv` | 按变体配对，不能将 24 行当独立任务 |
| 归档输出与输入绑定 | archive `receipt.json.outputs`，尤其 `private-evaluation/evaluation.json` 与 `public-output/method/method.json` SHA | 基础执行提交 `bb2809d`，报告提交由结果 JSON 记录 | receipt 是冻结工件；生成器不会改写它 |
| source-time pose、两轴校准与私有 GT 隔离 | `docs/research/companion-first-pilot.md`、`companion-visible-instance-gt.md` 的命名证据与 archive capture/package 成员 | 这些为持出前消费者接缝，不是 A1 accuracy | 不外推至硬件、导航或物理世界 |

历史 `941ea2d` 的 development scaffold 含 constant-R、假组件代理和 all-false evaluator，明确排除。留出前的 `bde1f3e` 修正 RIGHT-positive/关系扰动；`ca8d021` 修正 `mrr_upper_bound=max(mrr_lower, uncertain_best)`，二者在 v6 结果之前完成。SAP-Nav 的环状候选、可导航筛选、可见覆盖、到达复核重叠仅用于限制新颖性叙述，未在本包复现，也未作为导航成果。

## 参考文献

文献条目见 `references.bib`。条目仅保留所供来源中可核实的字段；缺失作者全名、卷期、页码、DOI 或出版状态者不补猜。
