# AWS 场景导航验证：结果与未通过项

## 结论

本轮**没有通过导航到达验收**。已实际运行 Gazebo、原生位姿流、激光建图、Nav2 规划与控制尝试，但保留的导航动作结果是 `status=6`（ABORTED），不是 `status=4`（SUCCEEDED）。这些记录不能用来证明 A2、视觉目标到位确认、取消安全性或实物可用性。它们作为独立工程失败证据保留，不计入 A1 论文的离线指标。

实现首先在隔离工作树 `/home/gxxl/soarm-car-companion-nav`、分支 `research/companion-nav-knownpose` 完成，再本地合并到论文交付分支。未修改主工作树已有改动、原始 Kuruma 模型、生产 FAST-LIO 或安全链；未推送远端或运行实物。

## 已有真实证据

- 固定 AWS 源提交 `e2f90621f0fd1a0c496f4e7fcca06138808bc2ec`；370 个**被消费文件**的 manifest 校验通过。源仓库所有 Git 文件数不等于这个子集数。
- 真实激光 `PointCloud2` 构建 320×320、0.05 m/格的局部占据地图，不复用 `house_floorplan` 地图或隐藏碰撞网格。
- 原生 `Pose_V` envelope 时间戳是自位姿来源。ROS `TFMessage` 桥接丢失的零时间戳只用于诊断，不作为新鲜位姿。
- 修复了生命周期同步等待阻塞 TF、错误接受 `inactive`、时钟不一致、零 twist、激光完整 frame 名与初始放置不一致等问题。最终激光 frame 为 `kuruma/livox_frame/mid360`。
- Nav2 生命周期曾达到 active，并产生真实 `ComputePathToPose` 返回；最终研究配置采用 0.50 m 半径的保守足迹。路径仍须通过未知/占据与扫掠检查，未为让实验通过而清空障碍。
- 检查了速度平滑器的全向配置，并记录 Nav2 到最终速度话题的消费链。节点级诊断通过不等于导航通过。

## 必须保留的失败解释

| 证据 | 实际结论 |
|---|---|
| `artifacts/aws-knownselfpose-radius-run-1/pilot_report.json` 的 `routes` | 两个动作均 `status=6`；“执行了两条路线尝试”不等于成功完成两条路线。 |
| 同一旧报告的 `trajectory_path_length_m` | 混入调查/初始放置阶段，**不得引用为导航行走距离**。旧 `footprint_m` 事件字段也不作为最终配置依据。 |
| 同一旧报告的 `fresh_images` | 保存了帧，但动作失败，不能称为成功到达后的目标确认。 |
| `artifacts/aws-knownselfpose-smoother-run-1/navigate_result_0.json` | 最后修正运行仍为 ABORTED；记录 `nav_cmd_count=155`、`final_cmd_count=380`，仅证明命令消费发生，不证明正确到达。 |
| 所有 `collision_assessment` | `UNKNOWN`；没有独立、完整验证的接触信息，不能把缺失报告当作零碰撞。 |

当前尚未验证：成功到达两条路线、成功终点的新帧视觉身份确认、运动中取消到真实停稳的完整证据，以及接触/碰撞安全性。控制器进展失败还需独立诊断；本记录不把可能原因写成已证明根因。

## 恢复与使用限制

代码入口为 `scripts/run-aws-knownselfpose-physicalsim.sh`、`scripts/aws_knownselfpose_pilot.py`、`config/aws_knownselfpose_nav2.yaml`；新研究 launch/BT 独立于生产入口。`artifacts/aws-knownselfpose-*` 中保留参数、地图、JSON 和压缩运行证据。

节点级检查：`bash scripts/test-aws-knownselfpose-node-only.sh`，使用本地已存在的 ROS 镜像、无主机网络，不启动 Gazebo。它在交付分支合并后已重新通过。

实际仿真入口仍是**未通过验收的研究工具**，不能部署到机器人、作为已验证控制接口，或直接继承到硬件。实物阶段需要有人监督、物理急停、标定和独立安全验收。该限制不改变冻结 A1 留出数据及其离线评价。
