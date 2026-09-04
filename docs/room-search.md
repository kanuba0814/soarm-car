# 找书桌并停靠观察

## 演示（无 ROS / 无硬件）

```bash
scripts/dev-room-search.sh --port 8088
# 浏览器 http://127.0.0.1:8088 ，或 scripts/dev-room-search-smoke.sh
```

页面顶部始终显示 **SIMULATION**。其 RGB-D observation 是 mock；不会移动机器人，更不代表相机、ASR 或定位成功。按钮依次调用 HTTP core API：载入地图、保存房间、proposal、确认、扫描导航/扫描、分析、候选确认、停靠导航、到达、取消/重置。非法阶段返回 JSON `409`。

## 真机启动

先启动已有安全导航链（它是唯一的 `/cmd_vel -> localization_cmd_gate -> /safe_cmd_vel` 链）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select soarm_room_search
source install/setup.bash
ros2 launch soarm_navigation house_fastlio_nav2.launch.py
ros2 launch soarm_room_search room_search.launch.py
```

`room_search_node` 仅创建 Nav2 `navigate_to_pose` action client，绝不发布 `Twist` 或写 `/cmd_vel`。真实接入必须将经验证的 `localization_healthy=true` 和当前 localization `map_id` 传入；健康、map_id 缺失或传感器超时会取消 action 并转为 `BLOCKED`。扫描是在固定 XY 的四个 `NavigateToPose` 航向（每段 pi/2）完成，core 以解缠 yaw 与八扇区覆盖验证一周。

## 地图、房间和 ASR

MapBundle 记录 occupancy YAML/PGM SHA256、resolution/origin 和 localization map_id。房间编辑 API 使用简单非自交多边形、唯一 ID/别名；scan pose 必须在房间内且 footprint+margin 位于已知自由区。示例为 package `examples/demo.yaml`。

ASR 引擎是外部可替换的命令行适配点（例如 `whisper.cpp -m model.bin -f input.wav`）；缺少引擎必须报告 `NOT_READY`，不得把音频当成功意图。首版只接受 `去 <唯一房间别名> 找书桌`，未知、歧义或附加运动命令拒绝，并且 GUI 确认前不得调用 Nav2。

RGB-D 适配器应只向 `desk_hypotheses()` 输入结构化观测（位置、height、width、quality、dock、line_of_sight），不能读取 Gazebo 桌子真值。停靠候选要求当前 revision、房间内、质量、footprint+margin 已知自由、桌距、视线和可插拔路径可达性全部通过；否则 `NO_SAFE_DOCK`，评分无权覆盖拒绝。

## 真机前不可伪造的标定清单

* occupancy YAML/图像 SHA256 与 localization map_id 一致；map→base_link 坐标和机器人 footprint/margin 实测。
* 相机内外参、RGB-D 时间同步、书桌高度/宽度阈值及 observation freshness。
* Nav2 到扫描/停靠 pose 的真实 action 成功、取消传播到 localization gate、急停与失定位/失传感器 BLOCKED。
* ASR 麦克风、模型、语言和命令退出码；没有引擎时 UI 必须显示 NOT_READY。
