# Named-place request preparation

This CPU-only module prepares a JSON payload for the existing Nav2 `NavigateToPose` consumer. It never creates a ROS node or action client and never sends actions or velocity commands.

The pinned source contract is position-only: the authoritative dirty launch selects `navigate_position_only_holonomic.xml`, which selects `position_only_goal_checker`. Requests must therefore contain exactly:

```json
{"place_id":"room-1","frame_id":"map","require_heading":false}
```

Only `room-1`, `room-2`, `room-3`, and literal `mian` are accepted. `require_heading:true`, foreign frames without verified transforms, and `demo_target` are rejected. The package has no `--catalog` or `--lock` override; its catalog and manifest are pinned. `--asset-root` is verified against every manifest hash and exists only to point at the authoritative read-only source assets.

Use `--verify-prepared` immediately before constructing the ROS goal. It rechecks the pinned manifest/current assets, strict JSON types, finite non-boolean coordinates, unit quaternion, allowlist, and exact catalog pose. Asset identity does not certify reachability, geometry, collision safety, or smoother behavior.

Run the exact consumer seam:

```bash
PYTHONPATH=ros2_ws/src/soarm_named_place_request \
python3 ros2_ws/src/soarm_named_place_request/test/test_consumer_serialization.py
```

The test first confirms Docker image ID `sha256:15012136e44c6a439c137ac62a67c0f38f09f55a357ba03a916a48895f6e4dc9`, then runs that immutable image with `--network none` and invokes the installed `rclpy.serialization` on real `nav2_msgs.action.NavigateToPose.Goal` messages. No navigation execution occurs.
