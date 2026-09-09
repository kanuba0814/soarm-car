# 家庭服务机器人研究交付索引

## 先说明结论

已交付一份经过数据追溯、代码检查和实际仿真采集支撑的 **A1 离线视觉目标辨识研究稿**及配套材料；**尚未达到“方法改进已成立、仿真导航与实物闭环全部通过”的论文终稿目标**。

冻结评估包含同一住宅的三个未见家具布局、24 个真实仿真 RGB-D 视角、18 个查询—布局条件和四个变体的 72 条记录。数据未证明改进优于基线，且包含弃权、身份不确定和错误接受。不得将 72 行当成 72 个独立场景。

## 阅读入口

- [论文 PDF](docs/paper/thesis.pdf)
- [可编辑论文 Markdown](docs/paper/thesis.md)
- [本地 HTML 排版版](docs/paper/paper.html)
- [论文配套包与重放说明](docs/paper/README.md)
- [分方法、分布局数据与图](docs/paper/results/)
- [浏览器、PDF、测试及归档核查](docs/paper/publication-verification.json)
- [导航验证失败与未通过项](docs/research/aws-navigation-validation-status.md)

## 交付内容

| 内容 | 状态与边界 |
|---|---|
| 文献、方向、冻结查询和协议 | 已记录；不宣称首创或保证发表。 |
| 家具场景、RGB-D、自位姿时间关联、双轴校准 | 已实际验证并保存原始数据；自位姿是显式已知位姿条件，不是实物定位结果。 |
| 私有实例真值及公共方法隔离 | 已实际解码、遮挡验证和受限文件系统重放。 |
| V、V+R、V+M、V+R+M 的留出评估 | 已运行；未知/失败保留，未证明优越性。 |
| 源码、冻结配置、原始数据、生成表格与图 | 在本地 Git 中保存；大模型权重以可恢复来源和校验锁记录。 |
| 新场景传感器建图、Nav2 工程尝试 | 已运行但动作中止，未通过到达、取消或碰撞安全验收。 |
| A2 主动观察位效果、语音聊天整机闭环、实物迁移 | 本次未取得验证证据，不能写成论文成果。 |

原始传感器/标定/实例数据索引：`artifacts/companion-home-validation/evidence/MANIFEST.md`。三份留出原始归档：`artifacts/companion-home-validation/evidence/a1-heldout-results-v6/`。已知无效的早期脚手架结果保留历史，但明确排除出论文。

## Git 与复现

当前交付分支：`research/companion-home-validation`；交付工作树：`/home/gxxl/soarm-car-companion-validation`。导航隔离分支已经本地合并，原始主工作树的既有脏改动未纳入本次修改。没有推送、上传或发布。

可从此仓库创建新的分离工作树恢复已跟踪材料（目标目录须不存在）：

```bash
git worktree add --detach /path/to/new-reproduction research/companion-home-validation
cd /path/to/new-reproduction
python3 scripts/generate_companion_paper_results.py
python3 scripts/build_companion_paper_html.py
```

纸稿生成依赖见 `docs/paper/requirements-paper.txt`。原始模型推理及 ROS 仿真依赖不等同于纸稿依赖，必须按各自环境/模型/资产记录配置。离线重放无需操控机器人；不要把未通过的仿真研究入口直接用于硬件。

若继续追求原先完整目标，关键工作是解决受控 Nav2 的真实到达/停止证据、确定能在独立数据中得到支持的方法改进，再开展有人监督的实物标定与迁移。现有论文应按其实际范围使用，不能改写成成功闭环或性能提升的证明。
