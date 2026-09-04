# 找书桌并停靠观察

本模块实现以下受限任务：在已验证的预建地图上，由操作员定义命名房间；离线 ASR 或文本生成白名单意图；Nav2 到房间扫描点；原地分四段旋转一周采集 RGB-D 点云；自动提取保守桌面候选；结合占据地图和 Nav2 路径结果选择停靠观察点；经操作员确认后导航到达。

它只报告“到达书桌观察停靠位”，不报告抓取或操作成功。

## 无硬件演示

```bash
scripts/dev-room-search.sh --port 8088
# 打开 http://127.0.0.1:8088
scripts/dev-room-search-smoke.sh
```

演示页醒目标记 **SIMULATION**，点云和路径验证均为 mock，不会移动机器人。非法阶段返回 HTTP 409。

## 房间 GUI

1. 载入地图（真机由 ROS 参数载入，GUI 不能替换已验证地图）。
2. 在画布点击至少三个点，输入唯一 `room_id`、逗号分隔别名和扫描位姿。
3. 保存时验证：ID/别名唯一、简单非自交多边形、顶点在地图自由区、扫描点在房间内且整车 footprint 位于已知自由区。
4. 每个 ID 单独新增或更新；JSON 同时保存地图 YAML/PGM 哈希和定位 `map_id`，地图变化后拒绝载入旧房间。

## ROS2 Humble 真机入口

先启动现有 FAST-LIO/Nav2 安全链，再启动本节点：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select soarm_room_search
source install/setup.bash

ros2 launch soarm_navigation house_fastlio_nav2.launch.py
ros2 launch soarm_room_search room_search.launch.py \
  occupancy_yaml:=/absolute/path/to/map.yaml \
  expected_map_id:=/absolute/path/to/localization-map.pcd \
  rooms_json:=/absolute/path/to/rooms.json \
  pointcloud_topic:=/camera/depth/points \
  camera_info_topic:=/camera/depth/camera_info \
  camera_frame:=camera_depth_optical_frame \
  gui_port:=8088
```

首次没有 `rooms_json` 时仍可打开 GUI 定义并保存房间；完成前 readiness 为 `NOT_READY`。服务只绑定 `127.0.0.1`。

### 真实执行边界

- 唯一运动路径：`room_search_node → NavigateToPose → /cmd_vel → localization_cmd_gate → /safe_cmd_vel`。本包不发布 `Twist`。
- 订阅 `/localization/health`；状态必须新鲜、为 `HEALTHY`，且 `map_id` 精确匹配。
- Nav2 goal 接受后保存真实 goal handle。定位、地图、RGB-D 或 TF 失效会调用 `cancel_goal_async()`；同一时刻禁止第二个运动 goal。
- 扫描固定初始 XY，串行发送四个不超过 `π/2` 的航向目标；每段仅在 Nav2 成功后继续。实际 `map→base_footprint` telemetry 必须满足累计 `2π`、八扇区覆盖和中心漂移限制。
- PointCloud2 根据字段偏移解码并经 TF 变换到 `map`。保守 baseline 按高度、体素连通、尺寸、平面高度方差和支持点数提取桌面；不读取 Gazebo 家具真值。
- 候选必须通过房间、质量、整车 footprint、距离、占据地图视线和 `ComputePathToPose` 成功/非空路径检查。路径结果尚未返回时保持待规划，失败时 `NO_SAFE_DOCK`，评分不能覆盖硬拒绝。
- 最终停靠必须再次由 GUI 确认；Nav2 只有返回 `SUCCEEDED` 才进入 `ARRIVED`。

## ASR

`CommandASR` 是本地命令行引擎适配点（例如 whisper.cpp）。可执行程序及参数只能在服务启动配置中提供，HTTP 请求不能提供命令。引擎缺失、超时或非零退出均 `NOT_READY/ASR_FAILED`。首版只接受：

```text
去 <唯一房间别名> 找书桌
```

ASR 仅产生 proposal，必须由 GUI 确认后才允许运动。

## 真机启用前必须实测

- 占据地图与定位 PCD 身份、现场一致性及定位恢复；
- `map/base_footprint/camera_*` 外参、点云 frame、深度 scale、同步与延迟；
- 实车 footprint、安全余量、制动距离、原地旋转漂移和轮胎打滑；
- 各类桌面材质、光照、遮挡和多桌场景的误检/漏检；
- 麦克风、中文 ASR 模型、底盘噪声下识别率和机载性能；
- 物理急停与独立安全装置。GUI“停止”不能替代物理急停。

仿真、单元测试和容器构建均不能替代上述实测。
