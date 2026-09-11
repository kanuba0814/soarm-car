# NVIDIA exploration recovery verification

Run: `artifacts/runtime-recovery-20260911/` (2026-09-10/11 local time).
This is actual, known-origin, **prior-map-assisted** simulation evidence, not synthetic exploration or unknown-world SLAM.

## Results

- RTX 3070 confirmed by OGRE `GL_RENDERER`; fixed image and restored navigation assets retained in the receipt. No planner, controller, footprint, or freshness threshold was relaxed.
- `normal-003`: three automatically selected frontiers reached successfully, with navigation-band observed-cell gains **5939, 76, 10**. Nonempty successful ComputePath results, real ROS action UUIDs, target coordinates, revisions, consumed observations and checkpoints are saved.
- The supervisor reached `frontiers_exhausted` (`no_frontier`), released exploration ownership and used existing prepare/verify plus the same action transport to reach named place `mian` successfully. This exercises the structured named-place handoff, not free-form language interpretation.
- `pause-probe-003`: measured motion before cancellation, matching UUID terminal `CANCELED`, safe command and measured stop within the existing 3-second boundary, then accepted resume. Subsequent new planning and successful exploration in `normal-003` prove actual resumed execution.
- `watchdog-probe-006`: only this supervisor's cloud input was suppressed. No direct fault/cancel call was made by the injector. The unchanged freshness watchdog detected `source receipt stale or unavailable`, canceled the moving action and reached `sensor_fault`. Probe result PASS; no replacement goal was dispatched. Runs 004/005 are retained as earlier direct-injection checks, not substituted for this silent-input result.
- All 159 consumed observations from `normal-003` replay from **zero coverage** to the identical observed/visible/obstacle sets, accepted ID list and final revision. See `replay-verification.json`.
- Latest live progress records preserve the prior-connected reference count and explicitly identify the candidate selection revision. These reference counts are not a certification of world completeness; observed cells and the prior-connected reference have different domains.
- CPU verification: 34 core/transport/supervisor/sensor-policy tests and 9 probe-policy tests passed. Shell syntax, Python compilation and whitespace checks passed.

The final container was stopped after confirmed safe fault termination. Historical containers and failed artifacts were not deleted. No real robot, global dependencies, remote publishing or parallel model inference was used.

## Fixes and retained failures

1. Clear a pending terminal marker when canceling with no active action; resume retains explicit fresh gate authorization and a new reacquisition deadline.
2. Check timer freshness using a monotonic timestamp obtained **after** a pending cloud retry. The earlier timer snapshot could predate the cloud receipt and falsely report negative age (`normal-001/002`).
3. Vectorize only the existing Manhattan approach enumeration and cache immutable prior component sizes. Three recorded inputs produce exactly identical complete shortlists before/after; candidate time dropped from roughly 0.18s to 0.10s without profiling (`candidate-equivalence.json`). Preserve the radius and arrival-tolerance envelope.
4. Supply genuine ROS UUIDs and retain successful planning paths and segment coverage boundaries. Keep bounded progress during planning/navigation/canceling.
5. Let the probe return from its callback before destroying the ROS node. The old probe wrote TIMEOUT but deadlocked during callback destruction (`pause-probe-001`); `probe-exit-smoke` demonstrates bounded process exit.
6. Make source-drop injection suppress input only, so the live result actually exercises the watchdog.

An initial adapter startup failed because the temporary copied package lacked its frozen SDF. The deployed model was linked from the existing audited navigation workspace; no model identity check was bypassed. The named-place catalog was likewise copied unchanged before prepare/verify. A host replay initially could not write into a root-owned container evidence subdirectory; its report is instead saved in the host-owned run root.

## Evidence and reproduction boundaries

`acceptance-summary.json` indexes the results, source hashes, asset base/diff, renderer and action/path links. `normal-003` predates the progress-format-only update and silent-injector update; the latter were independently exercised in `watchdog-006`, with no normal-navigation algorithm change. Path UUID links are matched by successful endpoint, not claimed to be a single shared UUID between the two different ROS actions.

The launcher `scripts/start-exploration-recovery-runtime.sh` requires explicit audited `NAV2_ASSET_ROOT`, a unique `EXPLORATION_CONTAINER`, a new absolute `EXPLORATION_RUNTIME_OUTPUT`, and current `DISPLAY`/`XAUTHORITY`. It reuses the fixed image and existing navigation launch, refuses an existing container/output, and keeps the container network isolated. Inspect build completion and the actual renderer before sending goals. Use the existing exploration task entry point or the recorded ROS adapter/probe commands; never launch a second navigation owner.

Limitations: position-only goals do not prove camera scanning; this bounded reference scene does not establish generalized exploration, SLAM, live visual grounding, speech or free-form language execution. The product workbench remains its documented offline/read-only view. The missing permission-system package prevented subagent review; no permission bypass or framework mutation was attempted.
