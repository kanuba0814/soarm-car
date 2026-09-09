# 毕业设计：复用导航的小型 VL 与语义栅格混合方案

## 结论与范围

选择 **已有导航基线 + 语义目标记忆 + 按需小型 VL 确认**，不重新实现导航，不将 VL 输出直接转换为速度。当前完成的是离线开发小试、演示底盘和输入检查；组合系统尚未端到端接通，不能称为自主服务已验证。

先阅读 `/home/gxxl/soarm-car/WORKSPACE_INDEX.md`（主工作区提交 `3ac2d5c`）。导航优先复用 `/home/gxxl/soarm-car-nav2-clean` 的 FAST-LIO/Smac/全向 MPPI 和现有入口：`house_fastlio_nav2.launch.py`、`scripts/run-nav2-clean-comparison.sh`。其成功记录为 `artifacts/nav2-clean-direct-mppi-20260905T173147Z/summary.json`。基点 `41463f9` **不包含当前全部未提交配置、地图和脚本**；复用时必须保留配套状态。AWS 研究分支的 ABORTED 不是整体导航未实现的证据。

## 联网研究与取舍

| 方式 | 可取之处 | 本项目取舍 |
|---|---|---|
| [VLMaps](https://vlmaps.github.io/) 视觉语言地图 | 将语义查询与空间位置连接 | 不搬入完整重型依赖；采用稀疏目标记录与现有几何地图分工。 |
| [ConceptGraphs](https://github.com/concept-graphs/concept-graphs)、[HOV-SG](https://github.com/hovsg/HOV-SG) 物体/房间图 | 多次观察、关系与房间层次可持久化 | 简化为物体候选、来源图像、坐标系、最近观察时间、置信/冲突状态；不声称新场景图理论。 |
| [VLFM](https://github.com/bdaiinstitute/vlfm) 语义前沿探索 | 语义决策与低层执行解耦 | 借鉴按目标选择观察方向；不将其 Habitat/Spot 栈重新移植为本项目导航。 |
| [SmolVLM-500M](https://huggingface.co/HuggingFaceTB/SmolVLM-500M-Instruct)、[Qwen3-VL-2B](https://huggingface.co/Qwen/Qwen3-VL-2B-Instruct) | 可本地运行的语义理解候选 | 已实测 Smol；Qwen 下载超时尚未实测，不能宣布 Qwen 更优。权重修订、许可、来源纠正见 v2 文档。 |

两项设计重点：**带来源和过期状态的语义目标记忆**；**候选受限、几何把关的按需 VL 确认**。这些与上述方法有重叠，作为毕业设计的平台化组合与工程优化，不夸大原创性。

## 实测发现与简单调整

1. 同四幅公开开发图像，SmolVLM 输入为 RGB 640×480，实际处理器生成13个512×512图像块；CUDA峰值分配约1372MiB，v2十二次请求约0.23–0.68秒。属于本机/本提示/本输入测量，不是通用性能保证。
2. 原始严格 JSON 和精确单ID格式均0/4通过，但原输出为 `1.`、`A.`、`1.`、`0.`。不能将标点格式失败当作模型没有视觉能力。
3. 新增受限适配器只接受候选集合中的数字ID或 `NONE`，允许一个句末句点；不做字母映射、模糊匹配或从长文本提取数字。重放原始响应，**格式可接受3/4**。这不是3/4目标正确，更不是导航成功。负例检查覆盖越界ID、多ID、额外文本和异常标点；所有输出默认不授权运动。
4. 单帧 RGB-D 栅格未知区域很多。修正占据优先级及整个底盘足迹必须处于观测自由区后，v3 **0/4具备可达接近点**；此前1/4为旧门控结果，已被取代。这说明单帧局部观测不足，不能据此否定已有导航地图或放宽未知区域规则。
5. Qwen2B下载1800秒仅完成约402MB/4.255GB；无推理结果。暂不反复等待下载阻挡已有模块设计。

格式适配重放：

```bash
python3 scripts/replay_vlm_label_adapter.py \
  --input artifacts/companion-home-validation/evidence/vlm-grid-dev-v2/results/results.json \
  --output /tmp/vlm-label-adapter-new.json
```

输出拒绝覆盖已有文件。原始响应/推理结果不可变，适配结果单独保存为 `vlm-grid-dev-v2/label-adapter-replay.json`。

## RGB 与训练决策

当前演示抓到的原始 RGB 与 rqt 预览一致，无明显通道互换；低位平视相机造成地面占比大、家具整体不完整，不能期待与 Gazebo 自由视角相同。原帧及元数据已保存在 companion-validation 的 `artifacts/companion-home-validation/evidence/demo-camera-inspection/`（提交 `08f5eb2`）。没有完成同位姿逐像素渲染对照，因此不宣称排除了所有图形问题。

**不立即训练。**先做受限输出接口、逐候选裁剪和转向后的新帧确认；全图保留上下文，裁剪只增强目标，不用图像放大冒充新增细节。改变相机高度/朝向必须同步外参，且不能冒充真实硬件安装。只有这些处理后仍出现可重复的语义错误、并有独立训练/开发数据时，再考虑 LoRA；不要为四幅图中的标点专门训练，也不要用旧测试布局调参后继续称其未见测试集。

## 下一阶段的最小演示闭环（尚未实施）

`中文任务 → 现有任务解析接口 → 语义候选ID → 现有地图/导航检查接近点 → 原有导航执行 → 新RGB确认与语音/文字反馈`。

- 先演示命名地点，不要求 VL 凭空生成坐标；目标缺失时提示未找到或追加观察。
- 先确定旧导航场景与目标语义记录的同一地图身份/坐标变换；不能把 AWS 物体坐标塞进 house_floorplan 地图。
- 语义记忆保存类别、候选位置、坐标系、证据图和时间；冲突/过期记录触发确认，不静默覆盖。
- VL 只在查询歧义或到达确认时调用；无合规结果则采用原有白名单地点/用户澄清，不臆造目标。
- 运动、急停、取消和控制安全链完全沿用现有实现。本轮未运行新的 Nav2 实验或修改已有导航分支。
