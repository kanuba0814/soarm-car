# Named-place request preparation

`prepare_named_place_request` is a CPU-only producer for the existing Nav2 `NavigateToPose` consumer. It reads a named-place request, verifies the catalog and baseline asset lock, and emits only `prepared_for_serialization`. It never creates a ROS node, action client, navigation invocation, or velocity command.

## Request and output

The request JSON has exactly these fields:

```json
{"place_id":"room-1","frame_id":"map","require_heading":true}
```

The producer requires `frame_id: map` and `require_heading: true`. A foreign RGB-D frame is rejected when no verified transform is supplied; it is never treated as an identity transform. Position-only operation is rejected because the selected behavior tree cannot guarantee terminal heading.

`artifacts/rooms-complete-reference.json` is the preserved catalog, including literal ID `mian`. `asset_identity_lock.json` records the catalog hash, paired generated PGM/PCD hashes, Nav2 config/BT/launch references, and the installed image/package evidence. Identity verification is not geometry, reachability, or physical-safety certification. The exact installed smoother remapping is not established by this producer.

## Replay

The checked-out worktree intentionally does not contain the locked PCD and its current PGM hash differs, so the default command returns `not_ready` with the mismatched or missing identity evidence:

```bash
python3 -m soarm_named_place_request.prepare --request request.json --output prepared.json
```

For the CPU-only compatibility replay used by the test, generate the paired assets from the locked collision boxes into a temporary asset root, retain the matching Nav2 files, then run the producer and deserialize the output using the installed ROS image:

```bash
PYTHONPATH=ros2_ws/src/soarm_named_place_request \
python3 ros2_ws/src/soarm_named_place_request/test/test_consumer_serialization.py
```

This uses image `soarm-car-nav2-ros2-dev:latest` ID `sha256:15012136e44c6a439c137ac62a67c0f38f09f55a357ba03a916a48895f6e4dc9`, `--network none`, no GPU/devices, and a unique short-lived container. It verifies `rclpy.serialization.serialize_message` and `deserialize_message` against the installed `nav2_msgs.action.NavigateToPose.Goal`; it does not execute a navigation action.
