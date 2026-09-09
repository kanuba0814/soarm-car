# Nav2 基线恢复快照

本目录的 `recovery/nav2-clean-41463f9/` 固化了只读源工作树
`/home/gxxl/soarm-car-nav2-clean` 在 `41463f9` 的已跟踪二进制补丁和限定的未跟踪导航实现。
归档不包含 `.git`、凭据、镜像缓存、下载内容或历史容器/raw log；`manifest.json` 列出每个包含项的 SHA-256、排除原因和大文件摘要。

生成（目标必须不存在，工具拒绝覆盖）：

```bash
python3 scripts/export_nav_baseline.py \
  --source /home/gxxl/soarm-car-nav2-clean \
  --output recovery/nav2-clean-41463f9
```

离线恢复到全新的可丢弃工作树（不会启动 ROS、仿真器或 GPU）：

```bash
python3 scripts/export_nav_baseline.py --verify \
  --output recovery/nav2-clean-41463f9 \
  --restore /tmp/soarm-nav2-clean-restore-41463f9
```

验证会以 `41463f9` 建立 detached worktree，应用二进制补丁，拒绝路径穿越/链接归档成员，并核对入口 launch、MPPI、sphere wheel/house 模型、控制器、定位 gate、地图 YAML/PGM、PCD、房间目录和路由消费者的 SHA-256。还会解析 YAML、编译 Python、检查 PCD 头和保存路线目录。它只验证可恢复性，不重跑 Nav2；运行时仍需 ROS 2 Humble、Nav2、FAST-LIO 依赖及 manifest 所记 Docker 镜像。
