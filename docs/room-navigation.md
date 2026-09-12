# Chinese room navigation

`soarm_room_navigation` implements text → OpenAI-compatible planning → full GUI preview → explicit confirmation → deterministic Nav2 execution. It supports preset room goals, sequential visits, bounded waits, return to the task-start pose, and one preconfirmed fallback per navigation step. It does **not** implement object search, grasping, voice input, new SLAM, arbitrary model-supplied coordinates, or velocity control.

## Start the house simulation

The existing local Docker development image and packaged `soarm_simulation/maps/house_floorplan_reference.pcd` are required. The container entrypoint builds the ROS Humble workspace; host ROS installation is unnecessary.

Configure a provider you authorize before launching. Never put its key in the GUI or repository:

```bash
export SOARM_NAV_LLM_BASE_URL='https://operator-approved.example/v1'
export SOARM_NAV_LLM_MODEL='operator-selected-model'
export SOARM_NAV_LLM_API_KEY='operator-supplied-secret'

docker compose run --rm --name soarm-room-navigation \
  -e ROS_DOMAIN_ID=195 -e IGN_PARTITION=soarm_room_navigation \
  -e SOARM_NAV_LLM_BASE_URL -e SOARM_NAV_LLM_MODEL -e SOARM_NAV_LLM_API_KEY \
  ros2-dev bash -lc 'ros2 launch soarm_room_navigation room_navigation.launch.py mode:=simulation'
```

`room_navigation.launch.py` is the supported operator entrypoint. It starts the Gazebo GUI, the checked-in room-navigation RViz profile, FAST-LIO/Nav2, and the WebUI. Its finite one-shot initializer waits for received `/Odometry` and `/global_map` messages, then sends the configured `/initialpose` six times at `(initial_x, initial_y, initial_yaw)=(0, 0, 0)` to cover subscriber discovery. It does not wait for Nav2 activation, use truth inputs, send goals, arm the robot, or replay work. Change the pose or timeout with `initial_x:=... initial_y:=... initial_yaw:=... initialization_timeout:=45.0`; an unavailable input logs an error and exits the initializer after the timeout.

The operator defaults use the checked complete catalog at `/workspaces/soarm-car/artifacts/room-navigation-live/rooms-complete-reference.json` and journal at `/workspaces/soarm-car/artifacts/room-navigation-live/task-complete-reference.json`. The launch creates only their parent directory and never overwrites an existing catalog. `mode:=hardware` and any other mode fail explicitly because hardware mode is not implemented. The legacy `house_room_navigation.launch.py` remains headless and requires manual initialization for the smoke workflow.

Do not wait for Nav2 activation before supplying an initial pose: Nav2 depends on the resulting map TF. The room node uses simulation time for TF freshness, while safety/deadline timers use monotonic wall time.

Open **http://127.0.0.1:8090**. The saved-room list shows each alias, ID, and revision; select a room then choose **编辑已选房间** to load its polygon and goal with its ID locked. **新增房间** clears the draft and generates a distinct default ID. New rooms default to **自动安全中心** with no final-heading requirement: the service uses the polygon's shoelace centroid when it is safely clear, otherwise the deterministic nearest safe in-polygon occupancy-cell center. Automatic any-heading goals require conservative circumscribed-footprint clearance; no safe candidate is rejected without discarding the draft. Choose a manual target point and optionally fixed arrival heading only when needed. Creating with an existing ID is rejected, while an edit requires the revision that was loaded, so a stale editor cannot replace a newer room. Old saved records that lack this metadata retain their explicit manual position and fixed heading. Once localization and Nav2 are ready, enter text, inspect **every step and fallback**, and confirm. Stop is independent of the model request. Use the explicit arm button after a stop, then submit a **new** plan; arming does not replay the old one. Status includes `ready` and asynchronous `arm_status` (`REQUESTED`, `ARMED`, or a refusal reason).

End only this simulation with Ctrl-C in the launch terminal, or `docker stop soarm-room-navigation`. Do not use broad container/process cleanup commands.

### Map selection and identity

Launch parameters include `occupancy_yaml`, `localization_pcd`, `alignment`, `rooms_path`, `task_journal`, `gui_port`, `health_timeout`, `cancel_timeout`, `goal_acceptance_timeout` (5 s), and `goal_execution_timeout` (120 s). Paths passed to Docker must be container-visible. The house launch still uses the fixed house world: selecting another file is **not** evidence of alignment with that world.

A canonical SHA-256 bundle digest covers the occupancy YAML/image, PCD, map geometry, and declared fixed-frame alignment. It reaches global localization, transform fusion, localization health, the command gate, room catalog, and task controller. Changed files invalidate readiness and old proposals. **Hashes establish identity, not geometric alignment or mapping coverage.** No new mapping session is performed by this feature.

The older standalone `house_fastlio_nav2.launch.py` retains its legacy default identity for existing workflows. The new room entrypoint is the digest-bound path. Room polygons may contain furniture; their separate goals must lie inside the polygon and pass a conservative footprint check covering the configured square chassis. Active tasks/proposals block room edits.

## Model contract

The backend sends only known room IDs/aliases and the bounded action schema, with `response_format={"type":"json_object"}`. Providers must support this OpenAI-compatible JSON mode. The model has no ROS, shell, browser, file, or tool authority. Raw duplicate keys, nonfinite numbers, deep/oversized JSON, unknown actions/fields/rooms, and unbounded waits are rejected. HTTP redirects are rejected to avoid forwarding credentials to another endpoint.

One model request may run at a time; additional requests receive `PLANNER_BUSY` rather than accumulating a queue. Stop remains available and invalidates any late result. Each confirmation token binds immutable plan content to a specific request and map/room revision: duplicate or stale confirmation fails, while a fresh identical task is allowed.

Without configured credentials the planner is visibly unavailable. `fixture_demo:=true` is an explicitly labelled one-room deterministic demo, **not** a live model. The integration smoke instead exercises the actual OpenAI-compatible HTTP adapter against a local protocol fixture, also **not** real language understanding.

## Execution and safety

- Actions: `navigate(room_id, fallback_room_id?)`, `wait(1..60 seconds)`, and `return_to_start(fallback_room_id?)`.
- The backend captures a fresh task-start pose only after confirmation. The model cannot supply it. Return-to-start uses the position-only Nav2 tree because the captured pose is safe but does not impose a final heading.
- Ordinary rejection/abort, or an execution timeout followed by real cancellation completion, may use the displayed fallback once for that step. Fallback failure stops and asks the operator. Safety, localization, map, TF, transport, or external-cancel failures never trigger fallback.
- An operator stop or safety loss while a timed-out goal is draining **revokes** its pending fallback, even if safety later recovers.
- Fixed-heading rooms use the existing strict `goal_checker`; any-heading rooms use `navigate_position_only_holonomic.xml` and `position_only_goal_checker` (same XY tolerance, yaw tolerance pi), so Nav2 completion semantics, rather than only the sent quaternion, remove the final rotation requirement. Loading the added checker/tree requires restarting the Nav2 controller after deployment.
- Arrival can precede the final zero-command callback. `SETTLING` waits up to one second for fresh zero output before continuing or declaring success; an unmet deadline fails closed. Waits continuously require valid safety and zero output.
- `/safe_cmd_vel` remains single-owner. The operator-stop service latches zero; the task retains pending/accepted goals until their real terminal result. Cancellation timeout stays unresolved, not falsely complete.
- The gate tracks known canceled goal IDs and requires their terminal status, including treating `CANCELING` as active. An unused action server may never publish status; a **fresh empty cancel-all response** supplies empty-server evidence. The task owner separately retains pending sends and refuses arm while unresolved. This assumes the cooperative local ROS graph, not hostile arbitrary ROS clients.
- Rearm is explicit and can be refused until health/map/TF and cancellation conditions are met. No automatic resume occurs after stop, map change, failure, or restart.
- The atomic journal records unresolved work, not a replayable program. Existing corrupt/unreadable journals fail closed; recovery remains marked across repeated restarts until the new node requests a gate stop. The gate must drain before explicit rearm. Abrupt process loss is not a hardware emergency-stop system.

The GUI binds loopback. Mutating JSON requests require matching Host/Origin and the per-process CSRF token. It exposes no generic ROS command endpoint. `停止导航` is not a certified physical emergency stop or a braking-distance guarantee.

## Verification

```bash
PYTHONPATH=ros2_ws/src/soarm_room_navigation:ros2_ws/src/soarm_room_search \
  python3 -m pytest -q -p no:cacheprovider \
  ros2_ws/src/soarm_room_navigation/test \
  ros2_ws/src/soarm_room_search/test/test_core.py

# Existing ROS fake-action safety integration, including late goals and rearm:
./scripts/dev-localization-safety-smoke.sh

# Real house simulation, isolated ROS domain/partition, local model fixture:
timeout 650 ./scripts/dev-room-navigation-smoke.sh
```

The last script owns and cleans up only its fixture process/container. It prints an evidence directory. Default test ports are 18194/18195; override `SOARM_NAV_QA_GUI_PORT` and `SOARM_NAV_QA_MODEL_PORT` if needed. It checks actual HTTP room creation/planning/confirmation, Nav2 outward and return success, a one-second wait, matching health digest, moving stop, zero output, terminal cancel, and explicit arm without motion. `/ground_truth/odom` is used only by the scoring probe, never navigation/control.

Retained passing consumer evidence: [`artifacts/room-navigation-implementation-AkBDjz/`](../artifacts/room-navigation-implementation-AkBDjz/). The checked-in smoke completed in 94 seconds: two successful navigation actions, return-position error 5.6 cm, observed stop-to-zero command latency 16 ms, terminal cancellation, and explicit rearm without motion. Source tests passed 26 tests plus 2 subtests; the ROS gate suite passed 8 tests. These are bounded observations, not reliability or worst-case guarantees.

Browser visual QA remains blocked: installed agent-browser 0.33.2 is unsupported by the wrapper (requires >=0.35). No browser upgrade or alternate browser automation was used. No live model credentials were supplied; natural-language accuracy is **not** verified. A nearby roundtrip does not prove arbitrary-room coverage, collision-free full-house navigation, physical robot behavior, or worst-case stopping latency.

`vendor/localization/SHA256SUMS` predates existing local vendor patches. A whole-file checksum check has existing mismatches and is not a passing build criterion. This task's worker updated the changed launch-file entry; restoration of that entry to the historical value was blocked by the execution guard and was not retried through another tool.
