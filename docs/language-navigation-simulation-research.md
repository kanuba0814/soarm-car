# Language room navigation in simulation: implementation research

## Scope and conclusion

The requested vertical slice is feasible, but the repository does **not** yet implement it end to end. The closest code is `soarm_room_search`, whose domain is "go to a room, rotate, detect a desk, preview a dock, then navigate to the dock." Pure room navigation should not inherit that desk/RGB-D workflow.

The smallest defensible slice is:

1. finish one simulation mapping session and freeze its localization PCD;
2. bind that PCD to one occupancy YAML/image and one coordinate-frame declaration in an immutable, content-addressed map bundle;
3. define named room polygons and one safe navigation pose per room on the actual occupancy image;
4. accept only a unique room alias, require GUI confirmation, and send one `NavigateToPose` goal in `map`;
5. make GUI stop latch the final command gate and keep the task in `CANCELING` until Nav2 reports a terminal result;
6. require an explicit re-arm and a new goal after stop, localization loss, or map change.

No grasping, desk detection, RGB-D scanning, or open-vocabulary language belongs in this slice. This is a proposed implementation, not a claim that those changes already exist. The older grasp-only intent policy in `config/voice-policy.yaml` must be explicitly extended for `navigate_to_room`; room navigation must not inherit arm or RGB-D readiness requirements.

Development should first use the existing map to prove a room goal and cancellation, then integrate fresh mapping/finalization. The user's eventual operating order remains mapping → room labeling → navigation.

## What exists, and what it proves

| Area | Source evidence | What is proved | What is not proved |
|---|---|---|---|
| Occupancy map | `ros2_ws/src/soarm_simulation/tools/generate_house_floorplan.py`, `ros2_ws/src/soarm_simulation/maps/house_floorplan.yaml` | A deterministic PGM and Gazebo structure are generated from the same pinned Sweet Home 3D source. | The PGM is not produced by FAST-LIO mapping, and no artifact manifest binds it to a particular PCD. |
| PCD mapping | `scripts/dev-generate-house-pcd.sh`, `FAST_LIO_LOCALIZATION_ROS2/config/soarm_house_mapping.yaml` | The mapping recipe starts at the declared house origin, waits for the first FAST-LIO pose, drives a bounded open-loop arc, and saves a PCD expressed in FAST-LIO `camera_init`. | The script is a recipe, not retained runtime proof; it does not emit a PCD hash, occupancy hash, frame transform, or bundle ID. |
| Localization/Nav2 | `soarm_navigation/launch/house_fastlio_nav2.launch.py`, `FAST_LIO_LOCALIZATION_ROS2/config/soarm_house_localization.yaml`, `FAST_LIO_LOCALIZATION_ROS2/scripts/transform_fusion.py` | The configured PCD is loaded for localization; `transform_fusion` is designed as the sole operational `map -> odom_combined -> base_footprint` owner and suppresses map output unless health and configured `map_id` match. | `map_id` is currently the literal string `fastlio_house_floorplan.pcd`, not a content digest. Replacing bytes at the same path passes the identity check. |
| Room semantics | `soarm_room_search/core.py` (`MapBundle`, `Room`, `RoomStore`) | Room IDs/aliases, simple polygons, scan poses, free-space/footprint checks, and YAML/image hashes exist and have unit coverage. | The room record does not hash the PCD. `map_id` is caller-supplied. Boundary vertices alone being free does not prove a polygon contains no wall. |
| GUI | `soarm_room_search/http_gui.py` | A localhost HTTP control surface and serialized ROS command queue exist. | The canvas does not render the loaded map and hard-codes demo coordinates (`x=0.5..2.5`, `y=0.5..1.5`), so it is not a general map editor. The workflow and text grammar are desk-search-specific. |
| Goal serialization | `soarm_room_search/controller.py`, `soarm_room_search/ros_node.py` | Pending and accepted goals are guarded against a second goal; late acceptance after cancellation is canceled. | `cancel()` immediately clears handles and enters `CANCELED` without waiting for cancel acknowledgement or the terminal action result. A new task can therefore start while the old Nav2 goal is still canceling. |
| Final velocity safety | `soarm_navigation/localization_cmd_gate.py` and its tests | Stale/unhealthy localization zeros `/safe_cmd_vel`, latches, sweeps Nav2 cancellation, and requires explicit arm after cancellation confirmation. | There is no operator-stop input to this sole final command owner. GUI cancel only requests Nav2 cancellation; it does not synchronously latch actuator output. |
| Tests | `soarm_room_search/test/test_core.py`, `soarm_navigation/test/test_localization_cmd_gate.py` | Pure state, geometry, freshness, late-goal, and localization-gate policies have focused tests. Parent evidence reports `8 passed` for room-search core and a passing mock HTTP smoke including invalid transition, arrival, and cancel. | These tests and mocks do not prove Gazebo/FAST-LIO/Nav2 integration, common map coordinates, cancellation latency, or motion. Existing smoke scripts are executable specifications unless their produced evidence is retained and reviewed. |

## Hardest gap: occupancy/PCD common identity and frame

There are currently two different map production paths:

- the occupancy PGM is rasterized from pinned source geometry in canonical Gazebo world coordinates;
- the localization PCD is accumulated by FAST-LIO in `camera_init`, whose origin is established when FAST-LIO initializes.

The current simulation happens to spawn Kuruma at world pose `(0, 0, 0)` (`soarm_simulation/worlds/house_floorplan_ros2_control.sdf`), and the PCD recipe waits for initial odometry before motion. That makes alignment plausible, not an identity guarantee. A changed spawn pose, delayed initialization, changed LiDAR extrinsic, regenerated PGM, or overwritten PCD can silently invalidate room coordinates while every current string comparison still passes.

### Required bundle invariant

Navigation may become `READY` only for one immutable bundle containing at least:

```text
bundle_id = SHA-256(canonical manifest)
occupancy_yaml_sha256
occupancy_image_sha256
localization_pcd_sha256
frame_id = "map"
T_map_mapping_origin          # explicit SE(3), identity only when demonstrated
map resolution/origin/size
mapping configuration digest
simulation world/model source digests
```

The coordinator should bind `LocalizationHealth.map_id`, the semantic-room file, the selected map-server files, and GUI status to the same `bundle_id`, not rely on filename equality. The existing Nav2 map server does not gain a custom identity field automatically: the coordinator validates the files it loads. Map files must not be overwritten in navigation mode. A digest proves content identity, not geometric alignment; frame alignment needs separate evidence. For the first fixed planar world, explicit frame metadata is enough; a general-purpose SE(3) registration framework is not required.

For the minimal simulation slice, keep the existing source-generated occupancy map and explicitly prove `T_map_camera_init = identity` for the declared zero spawn. Do not claim that FAST-LIO generated the occupancy map. If a later requirement demands occupancy derived from sensor mapping, FAST-LIO alone is insufficient: it publishes/saves point clouds, not a 2-D `OccupancyGrid`. That needs a separate mapping/projection pipeline with observed free-space semantics, and its transform into the localization PCD must be recorded in the same bundle.

### TF invariants

- Exactly one operational chain: `map -> odom_combined -> base_footprint -> base_link -> livox_frame`.
- FAST-LIO `/Odometry` and `/cloud_registered` remain in `camera_init`; consumers must not relabel their headers.
- `transform_fusion` remains the only publisher of operational `map -> odom_combined` and `odom_combined -> base_footprint`.
- No `map` edge is published before a healthy correction for the active `bundle_id`.
- Dynamic robot ground truth is scoring-only and must not drive localization, navigation, health, task control, or command gating. This does not prohibit the existing static world geometry used to generate occupancy, or declared spawn/extrinsic metadata; neither is a live robot-state feedback channel.
- Room polygons and room goal poses are stored in the exact `map` frame of their bundle. A bundle change invalidates them; there is no automatic coordinate migration.

ROS frame semantics reference: [REP-105 Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html).

## Reuse boundary: room search versus room navigation

Reuse these transport-neutral parts after tightening their contracts:

- PGM/YAML loading and pixel-to-world conversion from `MapBundle`;
- room ID/alias uniqueness, polygon validation, footprint validation, and `RoomStore` concepts;
- the localhost-only HTTP transport pattern and serialized command queue;
- `NavigateToPose` action adapter structure and localization-health freshness checks.

Do not reuse the current `Workflow` state machine as the room-navigation state machine. Its legal sequence is coupled to `找书桌`, four scan goals, RGB-D freshness, tabletop analysis, dock preview, and a second confirmation. Adding a "room-only" branch would create many meaningless or bypassable states. Use a dedicated small controller/package, or first extract a narrow `semantic_map` module shared by both packages.

The room editor must render the real PGM with its actual aspect ratio and implement pixel/world conversion from the map resolution, origin, image height and any supported origin rotation. A fit-to-view canvas is sufficient initially; pan/zoom is optional. Validate polygon geometry and map extent, then separately validate the goal pose and its full footprint in known free space. **A semantic room may contain occupied cells such as furniture:** rejecting every occupied polygon interior would incorrectly reject ordinary rooms. Wall-crossing warnings can assist labeling but are not a substitute for goal/path safety. A polygon centroid is not generally collision-free or reachable.

Nav2 action reference: [NavigateToPose action](https://api.nav2.org/actions/humble/navigatetopose.html). Map server reference: [Nav2 Map Server configuration](https://docs.nav2.org/configuration/packages/configuring-map-server.html).

## Safe mode and task state machines

Keep system mode separate from the per-navigation task.

```text
SystemMode:
OFFLINE -> MAPPING -> FINALIZING -> SEMANTIC_EDIT
OFFLINE -> LOCALIZING -> READY
READY -> OFFLINE
(any mode fault) -> FAULT

RoomTask:
IDLE -> PROPOSED -> CONFIRMED -> NAVIGATING
NAVIGATING -> SUCCEEDED | FAILED
(any nonterminal task) -> CANCELING -> CANCELED
(any nonterminal task) -> STOPPING -> STOPPED
```

Binding transition rules:

- Mapping and localization/Nav2 are mutually exclusive; entering `MAPPING` requires no active task and a latched stop.
- `FINALIZING` computes/validates the immutable bundle before semantic editing or localization can select it.
- `READY` requires active map server, fresh `HEALTHY` localization for the exact bundle, complete TF, active Nav2 lifecycle nodes, and an armed command gate.
- Room editing is rejected during `NAVIGATING`, `CANCELING`, or `STOPPING`.
- A goal is sent only after a unique alias proposal and explicit GUI confirmation. Language never supplies coordinates.
- Cancel sends the action cancel request, enters `CANCELING`, retains the handle, rejects all new goals, and reaches `CANCELED` only after the action is terminal. Timeout becomes `STOPPING/FAULT`, not success.
- Stop additionally latches the sole final command gate to zero immediately, then cancels Nav2. Recovery requires explicit operator re-arm and a new goal; never auto-resume.

The existing localization-loss behavior is a useful basis, but the gate needs an operator-stop/latch input so `/safe_cmd_vel` remains single-owner. Cancellation semantics reference: [ROS 2 Actions design](https://design.ros2.org/articles/actions.html). Managed-mode reference: [ROS 2 Managed Nodes](https://design.ros2.org/articles/node_lifecycle.html).

## Recommended minimal vertical slice

1. **Map finalizer:** add an offline command that accepts the existing house occupancy YAML and a newly saved PCD, checks finite/nonempty geometry and declared frame alignment, writes digest-named immutable artifacts plus the manifest, and returns `bundle_id`.
2. **Semantic map editor:** adapt the current localhost GUI to show the actual occupancy image and save `room_id`, aliases, polygon, and one `goal_pose` bound to `bundle_id`. No scan/detection/dock controls.
3. **Room navigation node:** implement the two state machines above, exact localization bundle check, unique Chinese alias parser (`去 <房间>` for the first slice), confirmation, one active `NavigateToPose`, structured status, and an audit event sequence.
4. **Stop seam:** add a stop/latch operation at `localization_cmd_gate`; both localization loss and GUI stop use it. Keep the existing explicit arm behavior.
5. **Consumer smoke:** start the actual simulation mapping/finalization/localization/Nav2/GUI path, define one room, navigate to it, and exercise cancel while moving. Retain the bundle manifest, TF ownership inspection, action events, stop latency, final pose, and absence of ground-truth consumers.

Implement this for one world, one map bundle, one or two rooms, and text input first. ASR is only an adapter that produces the same proposal and adds no state-machine privileges.

## Alternatives ruled out for this slice

- **Reuse all of `soarm_room_search`:** rejected because desk search, sensor freshness, scan rotations, detection, docking, and double navigation are a different task contract.
- **Use only the existing `map_id` filename:** rejected because content can change at the same path without invalidating localization or room semantics.
- **Assume `camera_init == map`:** rejected unless the finalized manifest and runtime experiment demonstrate the declared transform for that mapping run.
- **Generate occupancy by naive PCD height projection:** rejected because occupied hits alone do not establish observed free space versus unknown space; Nav2 safety depends on that distinction.
- **Edit rooms or hot-swap maps during navigation:** rejected because costmaps, localization, semantic coordinates, and active goals could refer to different map epochs.
- **Mark cancel complete after calling `cancel_goal_async()`:** rejected because request dispatch is not terminal action completion.
- **Let a GUI/LLM publish velocity or coordinates:** rejected; the GUI selects named semantics and the deterministic Nav2/safety chain owns motion.
- **Use dynamic simulator robot truth as localization/control input:** rejected; it remains scoring-only. Configured static scene/spawn metadata is a separate, explicitly declared map construction input.

## Acceptance criteria

### Artifact and unit criteria

- Changing one byte in YAML, PGM, PCD, mapping configuration, or declared frame transform changes/rejects `bundle_id`.
- Localization health with a filename-equal but digest-different map cannot authorize TF or motion.
- The editor round-trips pixels/world coordinates at all four map corners within half a cell and rejects out-of-bounds/self-intersecting polygons, duplicate aliases, and occupied/unknown/unsafe goal footprints.
- A valid semantic room containing furniture remains labelable; only the navigation target and route require traversable space. Room goal poses remain stable after save/reload.
- Illegal mode/task transitions return conflict without changing state.
- Late goal acceptance after cancel is canceled and cannot change `CANCELING/CANCELED` to success.
- No second goal is accepted until the first goal is terminal.
- Stop immediately latches final output to zero; arm is rejected until cancellation is confirmed and localization for the same bundle is fresh.

### Runtime consumer criteria

- One fresh mapping run produces a PCD and finalized bundle; localization loads the exact PCD digest and Nav2 map server loads the exact occupancy digest.
- TF inspection proves one owner and a continuous `map -> odom_combined -> base_footprint` chain; no map edge exists before healthy localization.
- A room drawn on the GUI corresponds to the same physical simulation room, and its stored goal reaches `SUCCEEDED` without collision.
- While the robot is moving, GUI stop causes `/safe_cmd_vel` to become zero within the safety budget (target `<= 0.5 s`), Nav2 becomes terminal canceled, no new goal is accepted during cancellation, and motion does not resume without explicit arm plus a new request.
- Localization loss or bundle mismatch performs the same fail-closed stop/cancel behavior.
- Runtime graph inspection proves localization/navigation/control nodes do not subscribe to `/ground_truth/*`.
- Initial demonstration requires one real room-goal result plus a cancellation while moving, with retained action/telemetry evidence. Multi-room repetition and reliability measurements are a later validation phase, not a prerequisite for this bounded research.

## Runtime experiment status

### Team runs (2026-09-05)

- `scripts/dev-room-search-smoke.sh` passed its **mock** HTTP workflow (409 invalid transition, arrival, cancel); room-search core tests passed (`8 passed`). Browser QA could not start: installed agent-browser 0.33.2 is unsupported by the wrapper. No visual/browser verification is claimed.
- The full real smoke built 11 packages in 48.5 seconds, established FAST-LIO localization and operational TF, and activated Nav2. Importantly, `/tmp/soarm-fastlio-house.g2gOEG/container.log` contains `Initial global localization succeeded` at line 443, navigation lifecycle active at line 468, `Reached the goal!` at line 537 and `Goal succeeded` at line 538. The first Nav2 goal therefore did execute successfully; the agent summary omitted that log evidence.
- The full smoke then failed in its **scoring query**, not its navigation result: the `ign topic` read of `/world/house_floorplan_ros2_control/dynamic_pose/info` timed out after 10 seconds. See `/tmp/sim-runtime-task3/smoke.stderr`. No complete four-goal metrics were produced.
- The follow-up probe in `/tmp/sim-runtime-task3/nav-action-1788582326/` waited for Nav2 lifecycle activation **before** publishing `/initialpose`. Nav2 itself awaited the map transform produced after localization initialization. This creates a startup-order dependency deadlock; the retained script never reached the initial-pose command. **That run is not evidence of nondeterministic FAST-LIO startup.** The supported smoke publishes initial pose before waiting for Nav2 activation.
- All team-owned containers were removed. No robot hardware or safety configuration was changed.

### Parent bounded follow-up

A temporary probe reused the supported smoke's initialization ordering and **passed** one real `NavigateToPose` goal followed by cancellation of a second goal after nonzero safe velocity commands appeared. It read the existing scoring-only `/ground_truth/odom` bridge rather than depending on the unavailable dynamic-pose CLI read. These messages never fed navigation.

Retained evidence: `artifacts/language-navigation-research-QVFZzg/` (metrics, container log, TF/odometry inspection, gate tests, input digests and temporary probe sources). Input digests record this experiment's assets; they are **not** an implemented map-bundle protocol.

| Observation | Result |
|---|---|
| First target | `map`: `(1.0, 1.0, pi/4)` |
| First action result | `4` / `SUCCEEDED` |
| Estimated pose to goal | `0.2042 m` |
| Simulator truth to goal | `0.1635 m` |
| Estimated pose to simulator truth | `0.0641 m` |
| Localization health before/after | `HEALTHY`, map ID `fastlio_house_floorplan.pcd` |
| Second goal | Accepted; nonzero `/safe_cmd_vel` observed before cancel |
| Cancel response / terminal result | return code `0`; status `5` / `CANCELED` |
| Cancel request to observed zero safe output | `0.1595 s` |
| Maximum safe output during following one second | `0.0` |
| Container localization gate tests | `4 passed in 0.06s` |
| Probe process | exit `0`; container removed |

This cancellation measurement is an observed **command-output** latency, not a physical braking-distance guarantee, a GUI stop-latch test, or a worst-case timing bound. The direct probe exercised Nav2 cancellation, not the existing room-search GUI's early `CANCELED` transition. One nearby successful goal does not prove full-house coverage, collision absence, general frame alignment, or long-run reliability.

This research does **not** yet demonstrate fresh sensor mapping → finalized map bundle → room GUI → language task → navigation end to end. Existing PCD reuse, a direct Nav2 test and a mock GUI must not be represented as that integrated result.

### Implementation order after this experiment

1. Parameterize selection of the occupancy/PCD/semantic assets instead of hard-coding the house map. Preserve the verified startup order: sensor/FAST-LIO readiness → initial pose → accepted localization/map TF → Nav2 active → command gate ready. Merely starting processes is not `READY`.
2. On those existing assets, implement one room-goal controller with `PROPOSED → CONFIRMED → NAVIGATING → SUCCEEDED` and terminal-result-aware cancellation. Reuse geometry and ROS adapter patterns, not desk-search states. Unknown/ambiguous aliases and a second active request must not cause motion.
3. Render the actual occupancy map in the existing localhost GUI; define one room and a separate safe goal pose, then connect text `去<房间>` and explicit confirmation to that controller. Browser verification remains required once the tool version limitation is resolved.
4. Integrate a **new sensor mapping session**, saved-map selection and semantic invalidation. The existing open-loop PCD capture only covers an initial local arc; it is not full-house exploration or a sensor-built 2-D occupancy pipeline. First record declared map/frame metadata and content identity; then validate alignment and coverage before enabling goals. If both maps must come from live sensing, a separate free/occupied/unknown occupancy mapping pipeline is still needed; naive point-cloud height rasterization is not sufficient.
5. Add the explicit operator-stop latch to the existing single-owner velocity gate and test it separately from ordinary action cancellation. Keep physical emergency-stop claims out of the GUI contract.

No application code, control policy, dependency versions, or hardware configuration was changed by this research.
