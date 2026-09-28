# 离线感知资产到低人工干预任务：只读研究

研究对象：主工作区 `/home/gxxl/soarm-car` 当前文件；本报告写入独立 perception-extension worktree，未修改主工作区、未提交。未启动容器、ROS、GPU、模型、设备或网络；未读取原始图片/视频、权重、pickle/PT、凭证；未重复 room tests。只读小型 JSON、源码和既有审查文本。以下相对证据路径均相对于**主工作区**，部分为未提交产物，独立 worktree 不一定包含。

**建议毕业设计主线：有证据的离线找物与清点候选，随后增加变化候选和房间检索。** 现有资产足够做可审计原型，但不足以承诺开放世界 80% 清点、跨次稳定身份或自主到物体位置。最大瓶颈是物理实例去重/命名及证据关联，其次是 RGB-depth 注册与相机坐标到导航地图的关系；不是再增加第九套模型。

## 实测与证据等级

本次“实测”仅指 JSON 解析/断言和轻量 CLI；模型质量结论引用既有审查，未重新看原始影像或运行模型。

| 已有资产 | 本次结构实测 | 能力与真实性边界 |
|---|---|---|
| FARM inventory | 35 条，35 唯一 ID；全部有 centroid、supporting_frames | 已有模型运行导出；CPU stride-5 诊断，非 35 个已验证物理物体；注册未验证 |
| ConceptGraphs inventory + audit | 23 条唯一 ID，均有 centroid、supporting_frames；audit 为按实例 ID 索引的 23 项字典，有 representative/observations | 模型融合对象及后处理深度投影 bbox；不是人工 mask/GT；最适合先做证据检索 |
| OVIR-3D inventory | 49 条唯一 ID，有 centroid，无 supporting_frames | 模型融合候选，缺 inventory ID→原始帧的审计链；不能靠名称/centroid 自动确认实例 |
| Open3DIS finalized inventory | 104 条唯一 ID，有 centroid，无 supporting_frames | 模型 proposals，不能解释成 104 个物理物体；视频包仅展示 2D proposal 阶段 |
| OpenFusion | 包 manifest 标记 geometry only、inventory N/A | 既有审查末尾纠正了旧 loader failure：后续 TSDF 成功；不是物体 inventory |
| FindAnything / HOV-SG / HomeRobot | 包 manifest 分别为 not-applicable / terminated / blocked-not-applicable | 缺标定同步双目 / 完整融合终止且 smoke 非持久 inventory / 非原生 B06 消费器且源阻塞；状态卡不是推理成功 |

四份 inventory 的 ID 唯一、centroid 三元且数值有限、**已提供的**帧索引在 `[0,212]` 内均通过断言；不验证 centroid 正确、不对缺帧证据的系统给通过背书。ConceptGraphs audit 的 `derived_from` 明示 `post-finalization map; not upstream detection overlay`。

证据：

- `artifacts/perception-dev/benchmark-runs/inventory-v1/{FARM-B06-inventory.json,OVIR-3D-B06-inventory.json,TEST_PLAN.md}`。
- `artifacts/perception-dev/results/ConceptGraphs-B06-stride5/{inventory.json,audit/instance_representatives.json}`。
- `artifacts/perception-dev/results/Open3DIS-B06-stride8/finalized/B06-canonical-inventory.json`。
- `artifacts/perception-dev/results/comparison/B06-visualization-package-manifest.json`；它的 completion 描述应与 inventory_boundary 一起读，不能当模型可靠性认证。

GT 的 25 条仍标记 `agent_draft_pending_independent_review`，不是冻结人工 annotation。既有审查报告支持的**草案分母召回下界**为 FARM 2/25=8%、ConceptGraphs 1/25=4%、Open3DIS 1/25=4%、OVIR-3D 0/25；无法定义可靠的物理实例 precision/F1。这些是既有 agent 审查结论，绝非本次视觉复核或正式性能评分。证据：`benchmark-runs/inventory-v1/B06-ground-truth-draft.json`（上述 perception-dev 前缀），`artifacts/perception-dev/reviews/{B06-inventory-independent-review.md,B06-ConceptGraphs-audit-review.md}`。Open3DIS 历史审查还记录 86 对 IoU≥0.5 重叠；不应以 proposal 数做库存数。

## 几何与评估的关键缺口

- 本次读取 `artifacts/perception-dev/datasets/B06-rgbd-v1/manifest.json`：213 帧，`global_camera_poses_available=false`、`rgb_depth_registration_verified=false`、`geometry_usable=false`；同步均值 10.825 ms、p95 17.150 ms、最大 281.179 ms，3 对超过 20 ms，重复深度 2 帧。
- 后续 `benchmark-runs/inventory-v1/B06-rgbd-odometry.json` 为 `trajectory_valid=true`、212/212 相邻对通过，最大步移 0.01053 m、转动 1.37294°。其 `world_frame=first_color_camera`，不是导航 `map`。`scripts/estimate_rgbd_odometry.py:63` 附近仅用求解成功与步移/转角阈值判定，无独立精度/漂移评测，不能解除注册或 map 对齐门禁。
- 源码风险：`scripts/export_astra_rgbd_dataset.py:199` 附近把 `geometry_usable` 只定义为全部同步通过，同时另两项注册/全局位姿固定为 false。因此未来新序列该标志可能为 true 但几何仍不可导航；消费者必须检查注册、坐标系/变换证据、单位及同步，不能只检查这个 bool。
- `scripts/evaluate_inventory.py` 是纯标准库，按外部 adjudication 的 accepted matches 计分。它检查预测 ID 唯一、一对一匹配与重复 ID 引用，却不强制 schema、sequence 一致、GT 冻结状态、别名正确或物理实例归并。不要让调用者把任意 accepted 标签直接变成“模型准确率”。本次仅运行 `--help`，没有伪造 adjudication 得分。

## 接入接口与最少新增工作

现有入口是 `soarm.inventory_prediction.v1`：`sequence_id/system/instances[]`，实例至少涉及 `instance_id/canonical_name/centroid`，各系统证据字段并不一致。`scripts/export_farm_inventory.py` 从官方 consumer 读取 state 导出，supporting frame 通过 stride 回映射；该脚本顶层导入 torch，故本次未执行或导入。只需先做**读取既有 JSON 的薄适配层**，不必重跑八系统。

建议新增只读候选记录（提案，不是现有 API）：`source_run + source_instance_id + sequence_id + name_candidates + evidence_refs + observed_at + coordinate_frame + geometry_status + map_bundle_id(optional) + decision/reason`。将不同运行的同名/同编号视为不同候选；证据缺失、身份冲突、未观测到均可表达 unknown，不能静默合并。先复用 ConceptGraphs representative bbox 与 FARM 帧索引，后续再补其他系统 evidence 映射。

导航接入点应是**房间/观察任务 proposal**：`ros2_ws/src/soarm_room_navigation/soarm_room_navigation/core.py` 的 `RoomStore/parse_plan` 已绑定 bundle 与 room revision；它不是任意物体坐标导航 API。`soarm_room_search/.../core.py` 的 `Workflow.analyze/preview/confirm_dock` 接收桌面假设，`.../tabletop.py:detect_tabletops` 是几何启发式，不识别书桌用途或桌上物品；不能直接把 inventory centroid 塞进该结构。后续先将候选关联到已知房间/已有观察位，再复用既有 proposal/确认/路径验证入口。

`docs/room-search.md` 演示的点云与路径验证是 mock；`docs/room-navigation.md` 的 fixture_demo/本地协议 fixture 也不是语言模型能力证据。源码有真实 ROS 安全接缝，但本研究未验证运行态。`deploy/pi5/README.md` 明确镜像默认 idle、驱动显式启动；没有据此证明 Pi5 上已经部署语义模型或持久记忆服务。

## 五条优先任务（建议，未实现）

| 优先级 | 用户可见结果 | 最少新增工作与机器验收 |
|---|---|---|
| 1 | 可审计证据目录 | 只读 JSON adapter + 有界 validator；记录来源/坐标语义/拒绝原因。验收：缺字段、跨序列、越界帧、未注册几何不会升级为可信目标；每条可回溯 source run |
| 2 | “找水瓶”等离线候选检索 | 别名匹配+证据引用+不确定返回；不引入新大模型。验收：命中已知 ID 与支持帧，不认识的词返回无证据；候选命中率与确认成功率分开计 |
| 3 | 有界场景清点草案 | 先一次性冻结小范围实例粒度/别名/排除规则，再只复核冲突；输出保留/重复/背景/未知。验收：人工冻结 GT/adjudication 后复用 evaluator，报告 recall、precision、重复和弃权，并记录人工修改次数/每次用时 |
| 4 | 两次观察的变化候选 | 增加快照与保守关联，输出新增/可能移动/本次未见/未知；B06 单序列不能伪造跨日变化证据。验收：同快照重放零变化；新采合法成对数据后测误报与漏报，遮挡不直接判“丢失” |
| 5 | 语义记忆辅助房间导航 | 候选→房间/已有观察位 proposal，绑定地图与房间版本、失效时间。验收：陈旧地图/定位异常/缺变换拒绝执行；到达只表示到观察位，必须另有新观察才能说找到物体 |

低人工干预应优先减少反复标房间、反复看全视频和全量逐帧标注；保留一次范围冻结和少量歧义裁定，并测量人工介入次数。现有低召回下界下，取消确认不会自动带来可信自主能力。

## 命令、未验证项与隐私/安全边界

执行命令类别：有界 `find ... -maxdepth 2`/定点 `rg -n` 列脚本与小型 JSON；`read` 读取上述源码/说明；检查 evaluator 完整源码后运行：

```sh
python3 /home/gxxl/soarm-car/scripts/evaluate_inventory.py --help
```

另执行标准库 `python3 - <<'PY'`：用 `json.loads(Path(...).read_text())` 解析表中四份 inventory，断言 ID 唯一、centroid 为三个有限数、已有 supporting_frames 为整数且 `0<=frame<213`，打印字段覆盖数量；解析 dataset manifest/odometry/audit/package manifest 仅输出汇总字段。四份断言通过。最初额外查看 audit 样例时误按列表索引产生 `KeyError: 0`，随后确认它是 ID→record 字典并成功读取；未修改数据。没有运行评分、生成新测试或加载原始视觉资源。

未验证：原生模型与权重的完整重现、产物 hash 内容一致性、最终人工 GT、实际实例精度、RGB-depth 注册/外参、全局定位精度、跨序列关联、真机安全、Pi5 时延/内存与无人值守成功率。历史报告和包装脚本会残留旧状态；本报告以具体当前 JSON 为结构事实，历史视觉结论单独标识。未大规模搜索 artifact，因此不声称全仓库绝无其他实现。

建议仅保留必要类别/相对证据引用，原始影像留本地受控目录；报告不复制个人场景内容或凭证。截图/裁剪/模型上传不是本次授权内容。语义结果只能作为 proposal，不能发布 Twist、绕过定位/路径门禁或把相机系 centroid 当作 map 目标；跨地图、过期和无观测证据应拒绝自动升级。物理急停和真机校验仍由现有部署流程负责。
