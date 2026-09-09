# 命名地点请求锁定与序列化

本模块只准备既有 Nav2 `NavigateToPose` 消费端可读取的 JSON。它不创建 ROS 节点、不创建 action client、不发送 action，也不发布速度命令。输出仅为 `prepared_for_serialization`，不表示可达、action-ready 或已经执行导航。

## 实际导航契约

权威资产根目录是只读的 `/home/gxxl/soarm-car-nav2-clean`，当前 HEAD 为 `41463f9` 且含未提交的成功导航资产。`house_fastlio_nav2.launch.py` 实际选择 `navigate_position_only_holonomic.xml`；其 `FollowPath` 指定 `position_only_goal_checker`。锁定 MPPI YAML 的该 checker 为 `SimpleGoalChecker`、`xy_goal_tolerance: 0.2`、`yaw_goal_tolerance: pi`、`stateful: false`。

因此输入必须严格为：

```json
{"place_id":"room-1","frame_id":"map","require_heading":false}
```

`require_heading: true` 被以 `unsupported_heading` 拒绝。外来 RGB-D 或其他 frame 也被拒绝，绝不把缺失的变换当作 identity。允许的 ID 仅为 `room-1`、`room-2`、`room-3`、字面 `mian`；目录中的 `demo_target` 不是导航请求目标。

## 锁定和下游验证

包内固定 manifest 和固定 catalog 不接受 `--lock` 或 `--catalog` 覆盖。manifest 记录 source PGM `7d9c0b348928d1cb11c3ae033191b05989d35078ae34f3527c5779e0b731e08b`、由同一 collision boxes 生成的 `house_widened_reference.pcd` `f930a35e03ad641d2ad25174afa19f6fe9d070e38ae1bfe63391a5f3afac9a12`，以及实际 launch/config/position-only BT/through-poses BT 的哈希。catalog SHA-256 是 `742399c8fb8150fb55d78d230f9d90785193dff466c9e79690314266a49859d5`。

每个输出携带固定 manifest SHA-256。`--verify-prepared` 在下游 ROS goal 构造前再次检查当前资产、严格 schema、有限且非布尔数、单位四元数、精确 catalog pose 和 allowlist，因此篡改的 `x/y/z`、四元数、`NaN`、布尔数或 manifest 都不能到达 ROS 消费者。资产身份不是几何、可达性或物理安全认证；未证实的 smoother 重映射保持未知。

## 重放

```bash
cd /home/gxxl/soarm-car-nav-request
PYTHONPATH=ros2_ws/src/soarm_named_place_request \
  python3 ros2_ws/src/soarm_named_place_request/test/test_consumer_serialization.py
```

测试对真实 source asset root 写入四个 `require_heading:false` 请求 JSON，随后以锁定不可变镜像 `sha256:15012136e44c6a439c137ac62a67c0f38f09f55a357ba03a916a48895f6e4dc9`、`--network none`、无 GPU/设备和 `--rm` 容器调用实际 `rclpy.serialization.serialize_message`/`deserialize_message`。容器中先调用 reusable verifier，随后才构造 `NavigateToPose.Goal`。它还经 CLI 验证 true-heading、demo_target、外来 frame、变更/缺失资产、伪造目标/四元数/NaN/布尔值/manifest 和 `--lock` 覆盖都被拒绝。

提交 `fd21c11` 是保留历史但未合并的无效 yaw-enforcing prototype；后续修复提交恢复本页描述的原始 position-only 契约。
