# 命名地点请求锁定与序列化

本变更只准备既有 Nav2 `NavigateToPose` 消费端可读取的目标 JSON。它不创建 ROS 节点、不创建 action client、不发送 action，也不发布速度命令。输出状态只能是 `prepared_for_serialization`；这不表示可达、action-ready 或已经执行导航。

## 输入、输出和边界

输入必须严格为：

```json
{"place_id":"room-1","frame_id":"map","require_heading":true}
```

输出 `map_goal` 保留目录中原始 `x`、`y`、yaw 派生的四元数，使用既有消费者的 `map` 坐标系构造方式。目录为 `artifacts/rooms-complete-reference.json`，其 SHA-256 是 `742399c8fb8150fb55d78d230f9d90785193dff466c9e79690314266a49859d5`，并保留字面 ID `mian`。`require_heading` 必须为 `true`：position-only BT 没有严格终端朝向保证。

RGB-D 或其他外来 frame 在没有已验证变换时被拒绝，绝不默认 identity transform。资产哈希只证明输入身份；它不是几何、物理安全或可达性认证。已安装 velocity smoother 的映射差距没有被猜测，保持未知。

## 锁定证据

`ros2_ws/src/soarm_named_place_request/asset_identity_lock.json` 记录：

- 成对 PGM：`7d9c0b348928d1cb11c3ae033191b05989d35078ae34f3527c5779e0b731e08b`
- 从同一 collision boxes 生成的 PCD：`f930a35e03ad641d2ad25174afa19f6fe9d070e38ae1bfe63391a5f3afac9a12`
- Nav2 YAML、NavigateToPose/ThroughPoses BT、launch 文件的 SHA-256 和 YAML 解释结果
- 验证镜像 `soarm-car-nav2-ros2-dev:latest`，ID `sha256:15012136e44c6a439c137ac62a67c0f38f09f55a357ba03a916a48895f6e4dc9`，以及安装的 nav2_msgs/navigation2/rclpy 版本

当前 worktree 的 PGM 是不同哈希，且没有锁定 PCD，所以默认命令会以 `not_ready` 失败。这是预期的身份保护，不能替换为其他地图或传感器数据。

## 精确重放

```bash
cd /home/gxxl/soarm-car-nav-request
PYTHONPATH=ros2_ws/src/soarm_named_place_request \
  python3 ros2_ws/src/soarm_named_place_request/test/test_consumer_serialization.py
```

测试在临时目录从锁定 collision boxes 生成 PCD，使用锁定 PGM 和本分支锁定的 launch/config/BT 文件，依次写入 `room-1`、`room-2`、`room-3`、`mian` 请求 JSON，再由新的 CLI 重新加载。随后它以 `--network none`、无 GPU/设备、`--rm` 的独占容器运行实际 `nav2_msgs.action.NavigateToPose.Goal` 的 `rclpy.serialization.serialize_message`/`deserialize_message`，并检查字段回放。测试还经由同一 CLI 拒绝未知 ID、变更 PGM、缺失 PCD、变更目录、外来 RGB-D frame 且无变换、非有限坐标、缺失 frame 和不支持的 heading。
