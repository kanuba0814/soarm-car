# 对话驱动家庭视觉导航：来源与选题约束

本轮为论文/官方实现阅读，未重现模型或测定论文数值。配套方案见 [研究预案](companion-visual-navigation-design.md)。以下是有针对性的来源筛选，不是穷尽式最新文献综述，不能据此宣称首次提出。

## 核心原始来源

| 来源 | 已发表工作解决什么 | 对本工作区的意义 |
|---|---|---|
| [VLMaps：Visual Language Maps for Robot Navigation](https://arxiv.org/abs/2210.05714)、[官方代码](https://github.com/vlmaps/vlmaps) | 将视觉语言特征映射到空间，支持语言目标索引/导航；代码说明 RGB、米制深度、位姿及相机标定输入 | 适合作为语言到空间检索参照；语言查询语义地图本身不是新贡献 |
| [ConceptGraphs 原论文](https://arxiv.org/abs/2309.16650)、[官方项目](https://concept-graphs.github.io/)、[官方代码](https://github.com/concept-graphs/concept-graphs) | posed RGB-D 多视角关联生成开放词汇 3D 对象图，用于感知与任务规划 | 工作区已有适配及候选证据，可优先复用；多帧融合或加对象图本身已有先例 |
| [HOV-SG，RSS 2024](https://www.roboticsproceedings.org/rss20/p077.html)、[官方项目](https://hovsg.github.io/)、[代码](https://github.com/hovsg/HOV-SG) | 楼层/房间/对象层级开放词汇表示与长程导航 | 作为层级语义检索相关工作；不要把新增房间层级称为原创。工作区准备脚本不代表完整复现成功 |
| [CoIN，ICCV 2025 官方论文入口](https://openaccess.thecvf.com/content/ICCV2025/html/Taioli_Collaborative_Instance_Object_Navigation_Leveraging_Uncertainty-Awareness_to_Minimize_Human-Agent_Dialogues_ICCV_2025_paper.html)、[官方项目](https://intelligolabs.github.io/CoIN/) | 在导航期间通过开放式人机对话主动消解目标实例不确定性，并减少用户输入 | 方向 C 的直接竞争工作；“不确定时问用户”不能单独作为创新 |
| [Uncertainty-Informed Active Perception](https://arxiv.org/html/2506.13367) | 不确定性感知/主动观察相关研究入口 | A2 后续需要逐项核查其状态表示、动作选择与停止策略；本轮不声称已排除与其方法重合 |

### 已核对的直接段落

ConceptGraphs 官方项目写明：

> “ConceptGraphs builds an open-vocabulary 3D scene graph from a sequence of posed RGB-D images.”

其原论文摘要写明：

> “ConceptGraphs is built by leveraging 2D foundation models and fusing their output to 3D by multi-view association.”

CoIN 的 CVF 摘要写明：

> “We thus introduce Collaborative Instance object Navigation (CoIN), a new task setting where the agent actively resolves uncertainties about the target instance during navigation in natural, template-free and open-ended dialogues with the human, minimizing user input.”

这些段落支持“场景图、多视角融合和对话消歧已有工作”的事实。**由此建议将论文贡献收窄到特定关系证据排序、等预算可达视角决策及实物噪声鲁棒性，是本项目的研究判断，不是来源证明其创新成立。** 自动 claim classifier 对包含这项研究判断的复合断言返回 unclear；已检查以上原文，不将该分类当作缺少相关工作的证据。

## 仿真、任务定义与数据条款

- [Habitat Challenge 2023](https://aihabitat.org/challenge/2023/)：ObjectNav / ImageNav 任务、Success / SPL 及平台设置。标准 ObjectNav 的目标类别/距离/可见性约定不等于自然语言关系指定实例；若使用本项目更严格的实例与新图像确认成功条件，应另报名称，不能拿数值直接与官方榜单比较。
- [Matterport3D 官方入口](https://niessner.github.io/Matterport/)：RGB-D/位姿/语义数据和申请条款入口。
- [ScanNet 官方仓库](https://github.com/ScanNet/ScanNet)：数据访问、标注与使用条款。
- [Replica 官方仓库](https://github.com/facebookresearch/Replica-Dataset)：数据与 Habitat 格式相关说明。

数据集、代码、预训练权重分别适用各自条款；代码仓库可访问不意味着场景或模型可随意下载/再分发。正式获取或发布前按实际版本逐项核查，不在本轮代用户接受条款。现有 Gazebo house 优先用于本工作区集成，Habitat 是以后增加公开评测的可选项，不是必须重建的起点。

## 选题结论与未解决问题

1. 首选一个既有对象图/视觉编码器为底座，做同表示、同预算的消融；不要一开始完整跑齐所有系统。
2. 方法贡献不能是聊天 API、地图 hash、人工确认、安全 gate 或简单管线连接。这些是重要工程前提，不是已证明的视觉导航算法创新。
3. 几何上需要注册、同步、内外参、米制深度、相机位姿到 map 的证据。相机首帧坐标、轨迹数值平滑、地图 hash 均不能独立证明可执行导航目标正确。
4. 算力优先给冻结视觉模型的特征/检测、多视角处理和后续消融；尚未核实当前 GPU 型号、空闲显存、依赖兼容性或端到端时延。
5. 当前所选 A1/A2 仍需在正式验证前对最近邻方法进行更细的公式/实现比较；可以验证一个具体优化是否有效，但不能预先承诺论文创新或性能增益。

研究员原始阅读摘要保留于会话产物；本文件为与工作区实际约束对齐后的精简结论。没有把其建议的“先 Habitat”自动变成本项目执行计划，也没有把旧报告中的机器配置当作本轮运行态。
