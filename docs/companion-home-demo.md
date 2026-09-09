# AWS 小屋可视化演示

此演示仅运行在 Docker 隔离的 Fortress 仿真中（`--network none`、`ROS_LOCALHOST_ONLY=1`），不会连接实机，也不启动自主导航。它显示固定的 AWS furnished small house、Kuruma 底盘，以及独立的 RGB 相机预览窗口。

## 启动

在 companion validation 工作树执行：

```bash
./scripts/start-companion-home-demo.sh
```

启动器输出容器名、容器 ID、日志目录、ROS domain 和 Ignition partition。Gazebo 主窗口为可交互的三维房屋视图；可用鼠标旋转、平移和缩放。`rqt_image_view` 窗口显示 Kuruma 的 `/camera/image`。

## 手动驾驶

另开终端执行以下任何一个命令。每次运动都是 8 个 20 Hz 消息，约 0.4 秒，速度最多为 `0.12 m/s` 或 `0.35 rad/s`；命令退出、中断或结束时都会再发送零速度。

```bash
./scripts/companion-home-demo-control.sh forward
./scripts/companion-home-demo-control.sh back
./scripts/companion-home-demo-control.sh left
./scripts/companion-home-demo-control.sh right
./scripts/companion-home-demo-control.sh turn-left
./scripts/companion-home-demo-control.sh turn-right
./scripts/companion-home-demo-control.sh stop
./scripts/companion-home-demo-control.sh pose
```

`pose` 从仿真的物理 `/gazebo/odom` 读取当前位姿，可在一次短脉冲前后比较位置，确认不是传送。请在 Gazebo 视角中确认周围留有空间后再移动；这是手动、低速可视化演示，不是碰撞规避或自主导航功能。

## 结束

关闭 Gazebo 窗口会结束容器。也可显式停止仅由该启动器拥有的容器：

```bash
./scripts/companion-home-demo-control.sh stop-demo
```

运行日志位于启动器报告的新建 `/tmp/soarm-companion-home-demo.*` 目录（或新建的 `SOARM_DEMO_OUTPUT` 指定位置）。
