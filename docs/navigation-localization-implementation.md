# 已有导航与定位：实际实现与证据

核对日期2026-09-09。实现工作区 `/home/gxxl/soarm-car-nav2-clean`，基点 `41463f9` **加当前配套未提交文件**。此文档依据代码和保存运行记录，不是本次重新启动导航的结果。

## 1. 两条数据链

```text
仿真 LiDAR + IMU
  → sim_adapter → /livox/lidar、/livox/imu
  → FAST-LIO → /Odometry、/cloud_registered
  → Open3D点到点ICP（与/global_map所载PCD匹配）
  → /map_to_odom + /localization/health
  → transform_fusion → /odom、map→odom_combined→base_footprint
```

```text
已有房间目录/任务消费者 → NavigateToPose(map系)
  → Nav2行为树 → SmacPlanner2D全局路径
  → Omni MPPI局部速度 → 速度处理链 → /cmd_vel
  → localization_cmd_gate → /safe_cmd_vel
  → ros2_control mecanum_drive_controller → 四个球轮关节
```

现有demo容器中的Nav2安装显示 controller→cmd_vel_nav→velocity_smoother→cmd_vel 的重映射；历史导航日志有 smoother 激活。但该容器 `:hm3d` 镜像不同于原导航默认镜像，**尚未核对原镜像安装文件的逐项重映射**，不能把它当本次已确认的运行时链路。

## 2. 定位如何实现

不是直接使用 Gazebo 真值导航，也不是单独一个“FAST-LIO”标签：

1. FAST-LIO结合激光和IMU产生连续局部里程计与已注册点云。
2. `global_localization.py` 订阅 `/cloud_registered`、`/Odometry`、`/global_map`、`/initialpose`，以Open3D点到点ICP对齐到全局PCD参考地图。
3. 接受的校正发布为 `/map_to_odom`；健康信息用于TF和速度门控。
4. `transform_fusion.py` 融合校正与连续里程计，Nav2使用它发布的 `/odom` 和TF。

代码入口（相对原导航工作区）：

- `ros2_ws/src/soarm_simulation/soarm_simulation/sim_adapter.py:29`
- `vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/scripts/global_localization.py:122`（订阅），`:172`（初始化变换），`:302`（发布校正）
- `vendor/localization/src/FAST_LIO_LOCALIZATION_ROS2/scripts/transform_fusion.py:57`（订阅/融合）

### 初始化与真值的边界

`nav2-clean-route-consumer.py:171` 等待真值遥测，然后在`:177`发布**固定零位置、单位四元数**的 `/initialpose`，并非把每时刻Gazebo真值直接喂给算法。因此成功记录以已知原点出生为条件，不证明任意初始位置的全局重定位。真值还用于就绪与评分，不能笼统写成“完全只用于评分”。

## 3. 导航如何实现

- 启动入口：`ros2_ws/src/soarm_navigation/soarm_navigation/launch/house_fastlio_nav2.launch.py`。
- 配置：`ros2_ws/src/soarm_navigation/config/house_nav2_mppi.yaml`。
- 全局规划：SmacPlanner2D；全局costmap使用配套PGM静态层和膨胀层。
- 局部控制：MPPI的Omni运动模型，生成vx/vy/wz；局部滚动costmap使用实时激光障碍体素层及膨胀层。
- 当前BT：`navigate_position_only_holonomic.xml`，FollowPath选择 `position_only_goal_checker`。**到达位置不等于满足目录中可能要求的终点朝向**。
- 安全入口：`localization_cmd_gate.py:91`等处订阅定位健康与速度，输出 `/safe_cmd_vel`，异常时闩锁/取消；不应旁路这个已有控制链。
- 驱动：`models/kuruma_ros2_control/model.sdf:17` 的 `gz_ros2_control-system`，mecanum controller速度引用重映射到 `/safe_cmd_vel`。
- 模型四轮为半径0.0763m球体。控制器里的有效运动学参数不能与视觉球半径机械地混为同一含义。

当前AWS手动GUI演示采用独立 `VelocityControl` 近似全向运动学；它不经过这套自主导航控制链，不能拿手动移动替代Nav2成功。

## 4. 两张地图各做什么

| 数据 | 用途 | 已核对事实 |
|---|---|---|
| PCD | 激光全局ICP定位 | 参考文件 `artifacts/house_widened_reference.pcd` |
| YAML + PGM | Nav2全局占据栅格 | `maps/house_floorplan.yaml`，0.05m分辨率 |
| 实时激光 | 局部障碍更新 | 不用参考PCD冒充实时传感器 |

`scripts/generate_house_reference_pcd.py:124` 从同一碰撞盒几何生成PCD和PGM，`:75`栅格化对应地图YAML。已有配套来源，不能误称完全无关系；当前文件哈希与成功快照吻合。但是这不是独立的几何再生成校验，也不能把AWS RGB-D坐标自动等同于这张地图。

**成功导航场景是简化、去家具、加宽开口的house_floorplan。** 它不是当前家具丰富的AWS GUI场景。新语义功能应先落在已有地图/接口上，再明确迁移新场景。

## 5. 实际成功记录

来源：`artifacts/nav2-clean-direct-mppi-20260905T173147Z/summary.json`。

| 目标原始ID | 动作状态 | 定位估计的终点位置误差 | 途中估计相对真值的最大位置差 |
|---|---:|---:|---:|
| room-1 | 4 SUCCEEDED | 0.149m | 0.060m |
| room-2 | 4 SUCCEEDED | 0.142m | 0.050m |
| room-3 | 4 SUCCEEDED | 0.165m | 0.057m |
| mian | 4 SUCCEEDED | 0.150m | 0.094m |

目录ID `mian` 按原样保留，不擅自改成 `main`。这些是历史四条运行，不是稳定性统计、家具场景验收或硬件结果。

## 6. 本次辅助模块的接入位置

只在目标请求前增加命名地点/地图身份检查，在语义侧增加可查询记忆、过期及冲突反馈。先验证真实 `NavigateToPose.Goal` 消息构造和序列化，不另外实现规划器/控制器，不默认执行动作。

完整启动还涉及现有runner的镜像、容器内构建、环境加载、初始化和goal consumer；不要在无ROS主机直接复制一条 `ros2 launch` 便宣称可复现。导航恢复快照实验会保存配套源码/资产和缺失依赖说明。
