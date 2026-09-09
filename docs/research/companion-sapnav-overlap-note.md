# 新近相关工作核查：SAP-Nav（仅来源核查，未复现）

原始论文：https://arxiv.org/html/2608.12707v1 ，摘要页 https://arxiv.org/abs/2608.12707 。标题 SAP-Nav: Spatial Semantic Representation Meets Active Perception for Hierarchical Open-Vocabulary Object Navigation。作者 Jian Liu, Vidura Munasinghe, Bo Miao, U-Xuan Tan, Wenrui Ding, Na Zhao。本次读取原始 HTML，通过 page-local answer 提取方法；不代表独立复现。项目页 https://xuetongpei.github.io/SAP-Nav/ ，论文称代码将在接收后公开，本次未确认可运行官方代码。

关键重叠：Method / Active Viewpoint Verification 中，Sufficiency Evaluation 用 VLM 对可见性与视角分别打1–5分；Viewpoint Selection 式(4)围绕目标采样同心圆候选，半径0.8/1.2/1.6/2.0/2.4m，每圈24方位；仅保留已探索可导航区域，并用式(5)相机高度/物体顶高/垂直FOV限制距离。式(6)用在线2.5D高度图的射线可见覆盖最大化选择视角。到达后重新判断，最多三次，仍不足则使用此前最佳分数观测。目标验证检查类别和所需属性。Conclusion / Limitations 明确当前视角选择未考虑语义效用或运动成本。

因此，“环状观察位采样＋可见性打分＋到位再验证”不能作为本项目新颖贡献。明确的代价敏感、支持去相关、缺失关系证据与最新终点身份不变量可作为待验证差异，但仍不能凭此宣称首创。后续A2应至少包括透明的几何可见性对照；若只是参照公式的本地实现，应称SAP-Nav-inspired几何基线，不冒充完整SAP-Nav复现。我们不使用VLM自评分作概率。该工作已宣称真实Nav2部署，但未在本项目复现。

文献其他包：/home/gxxl/.pi/agent/sessions/--home-gxxl-soarm-car--/subagent-artifacts/outputs/c7069cc3-2797-46f7-8d45-3daaf1fdc075/research.md 。其中现有地图/Nav2可直接复用的一句话需要被本项目AWS地图未验证事实限定，不能照抄为已可运行。
