# 机器人模型与图形化调整

当前模型用于状态和布局检视，不是机械 CAD、碰撞模型或实机外参。

## 模型构成

- WHEELTEC：依据用户提供的厂商 `mini_mec_moveit_six` 资料确定约 173 mm 轴距、198 mm 轮距和四麦克纳姆轮比例；显示使用简单 URDF 几何。
- SO-101：依据 TheRobotStudio 官方 `so101_new_calib.urdf` 的关节结构、名称、限位和尺寸顺序制作轻量姿态模型。
- 相机：准确型号未确认，暂为前部可调占位件。
- MID-360：圆柱包络占位。
- 蓝色箭头方向为反转后的新车头 `base_link +X`；机械臂默认位于旧相机区，即新车尾。

## 打开模型

```bash
./scripts/dev-build.sh
./scripts/dev-view-model.sh
```

会同时打开：

1. RViz 整车检视窗口；
2. Joint State Publisher GUI 滑块窗口；
3. Foxglove Bridge `ws://localhost:8765`。

GUI 中主要调节：

- `arm_shoulder_pan`
- `arm_shoulder_lift`
- `arm_elbow_flex`
- `arm_wrist_flex`
- `arm_wrist_roll`
- `arm_gripper_opening`
- `layout_arm_x`：机械臂安装点沿车身前后移动
- `layout_camera_x`：相机候选位置沿车身前后移动

这些 `layout_*` 是可视化安装滑块，不能接入真机控制。

## 显示内部状态

另开终端执行：

```bash
./scripts/dev-set-state.sh NAVIGATING
./scripts/dev-set-state.sh PERCEIVING
./scripts/dev-set-state.sh EXECUTING_GRASP
./scripts/dev-set-state.sh HOLDING_OBJECT
./scripts/dev-set-state.sh FAILED
```

RViz 中悬浮文字和底盘光晕会改变颜色。状态主题为标准：

```text
/robot_state            std_msgs/msg/String
/development/robot_state_markers
```

当前只是状态可视化，不负责状态机编排。

## 后续固化

在 RViz/GUI 中找到合适布局后，记录滑块值，再写回：

```text
ros2_ws/src/soarm_description/launch/view_model.launch.py
```

相机型号、实际安装孔位和物理测量确认前，不把可视化值当作标定外参或碰撞安全依据。
