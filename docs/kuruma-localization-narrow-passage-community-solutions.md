# Research: Kuruma localization and narrow-passage community solutions

## Summary
The observed permanent rejection is primarily a **health-contract / estimator-integration bug**, not evidence that FAST-LIO cannot support global correction: a one-shot ICP layer with hard scalar gates can deadlock after odometric drift exceeds its basin or after the score changes with viewpoint. FAST-LIO should remain the high-rate local odometry source; global registration should be a slower, independently validated measurement that updates `map→odom`, never the odometry estimate itself. The 0.781 m bottleneck is separately a geometric/planning limitation: a 0.62 m square has little yaw tolerance, so heading-aware approach and a real footprint are mandatory.

## Findings
1. **Mapping/loop closure is not fixed-map localization.** FAST-LIO-SAM-SC-QN combines FAST-LIO2, ScanContext candidate detection, Quatro robust global registration, Nano-GICP refinement, and GTSAM/iSAM2 pose-graph optimization; its README explicitly describes keyframes and loop detection. This is a useful architecture reference, but it is a SLAM backend, not by itself a production fixed-prior-map localizer. [FAST-LIO-SAM-SC-QN README](https://github.com/engcang/fast-lio-sam-sc-qn/blob/master/README.md) (repository accessed 2025-02-14; verify commit before adoption). FAST-LIO2 itself is the appropriate high-rate LiDAR-inertial frontend, but global corrections belong outside its local odometry contract. [FAST_LIO](https://github.com/hku-mars/FAST_LIO)

2. **Fixed-map precedent exists in Autoware NDT.** Autoware's NDT scan matcher consumes a point cloud and an initial pose, publishes pose/covariance, diagnostics, score, iteration count and execution time, and exposes a service for Monte-Carlo initial pose estimation. Its validation rejects stale/empty points, missing TF/map, excessive iterations, low score, or excessive initial-to-result movement. This is directly applicable as a contract model for Kuruma, although the package is not a drop-in Humble Nav2 node and should be ported or wrapped in C++. [Autoware NDT README](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/README.md) (main branch; pin a reviewed commit). It also documents multi-start covariance estimation and warns that computation can spoil healthy behavior—strong evidence for multi-scan/multi-start health rather than one threshold.

3. **Registration choices on Pi 5.** `small_gicp` is a C++17, header-only, low-dependency ICP/point-to-plane/GICP/VGICP implementation with OpenMP/TBB parallel paths; its README claims up to 2× speed over fast_gicp and provides Python bindings, but explicitly says GPU implementations are not included. Use it as a bounded native refinement library, not Python/Open3D in the control-critical path. [small_gicp README](https://github.com/koide3/small_gicp/blob/master/README.md). `fast_gicp` offers multithreaded CPU and CUDA variants, but CUDA is unsuitable as a Pi assumption. [fast_gicp](https://github.com/SMRT-AIST/fast_gicp). Autoware NDT's dynamic map loading (small grid PCDs) is a useful memory pattern for long maps, but importing all of Autoware is excessive.

4. **Health should be temporal and fail-closed.** Adopt the Autoware-style checks: synchronized timestamps and fresh `map→odom` TF; nonempty/adequate-range cloud; bounded optimizer iterations/time; score plus Hessian/observability or covariance; and bounded innovation relative to FAST-LIO. Require 2–3 consecutive compatible scans (and preferably a stationary multi-scan confirmation) before publishing a correction. During rejection, keep local odometry for short motion but mark localization degraded; after a timeout, stop/replan rather than silently continue. Relocalization while stopped is safer because scan motion distortion and control coupling are minimized. Never use scoring ground truth, spawn pose, perfect clock, or simulator noise parameters in these checks.

5. **Humble has the needed pre-alignment tool.** Nav2 Humble's Rotation Shim Controller sits between controller server and a primary controller, rotates in place toward the rough heading of the new path, then hands off. It explicitly supports differential and omnidirectional robots and non-kinematically-feasible planners such as NavFn, Theta* and Smac 2D. Parameters include angular threshold, forward sampling distance, rotational velocity/acceleration, collision simulation, and optional goal-heading rotation. [Humble Rotation Shim README](https://api.nav2.org/nav2-humble/html/md_nav2_rotation_shim_controller_README.html) This is the lowest-risk remedy for the current SmacPlanner2D heading blind spot; verify the installed Humble package version and use a real footprint during collision simulation.

6. **MPPI can use holonomic motion and orientations, but does not create heading-aware plans.** Humble MPPI supports `Omni`; `PathAlignCritic.use_path_orientations` defaults false and is useful when paired with feasible planners whose paths contain orientations. Enabling it with Smac 2D does not manufacture valid orientations, and MPPI alone does not guarantee the required 18° alignment. Full-footprint obstacle scoring (`consider_footprint=true`) costs more CPU. [Humble MPPI docs](https://docs.ros.org/en/humble/p/nav2_mppi_controller/index.html) Use Rotation Shim first, then test MPPI Omni with orientations only after a planner/waypoint source actually supplies them. Spin is a BT recovery for recovery-time rotation, not a substitute for route generation or an automatically inserted corridor maneuver. [Nav2 Spin recovery](https://docs.nav2.org/configuration/packages/bt-plugins/actions/Spin.html)

7. **Humble State Lattice is the orientation-aware long-term candidate.** The Nav2 Humble Smac README explicitly supports arbitrary non-circular omnidirectional robots with SE(2) footprint collision checking and configurable minimum control sets. The installed `nav2_smac_planner` is version `1.1.20` and ships 5 cm-resolution `omni/output.json` primitives for 0.5 m and 1.0 m turning radii, so this is not merely a Jazzy/Kilted feature. Rotation Shim is still the lower-risk first step; an isolated State Lattice profile should then test whether the shipped omni primitives produce the required pre-alignment without exceeding Pi-class planning budgets. Route-level pre-alignment waypoints remain a deterministic fallback for known bottlenecks. [Nav2 Humble Smac README](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/README.md)

8. **The 0.60 m obstacle minimum is unsafe as a blind-zone policy.** Livox specifies MID-360 minimum detection range 0.1 m, but data from 0.1–0.2 m is reference-only and low-reflectivity/thin targets from 0.1–1 m are not guaranteed. Thus 0.60 m is not a sensor blind radius; it discards potentially useful close returns, while still not proving safety. The robot footprint, mounting occlusion, temporal accumulation, and collision monitor must define safety. [Livox MID-360 specifications](https://www.livoxtech.com/mid-360/specs). Nav2 obstacle/voxel layers mark/clear from observations within configured raytrace/obstacle ranges; validate these ranges and sensor frame, and use Collision Monitor as an independent velocity-stop envelope, not as a localization fix. [Nav2 obstacle layer](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html), [Collision Monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)

## Evidence table
| Evidence | Version/date | Implication | URL |
|---|---|---|---|
| Rotation Shim supports Omni, rotates to path heading, collision sim | Nav2 Humble docs, package 1.1.x | Immediate pre-alignment candidate | https://api.nav2.org/nav2-humble/html/md_nav2_rotation_shim_controller_README.html |
| MPPI Omni, orientation critic, footprint cost and compute notes | Nav2 Humble docs, package 1.1.20 | Useful local controller; not heading planner | https://docs.ros.org/en/humble/p/nav2_mppi_controller/ |
| State Lattice supports non-circular omni robots and ships omni primitives | Nav2 Humble; installed package 1.1.20 verified locally | Orientation-aware SE(2) planning candidate for the square chassis | https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/README.md |
| NDT score/diagnostics, rejection, multi-start covariance, dynamic PCD loading | Autoware main (pin commit) | Health-contract/template for C++ fixed-map localizer | https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/README.md |
| ScanContext + Quatro + Nano-GICP + GTSAM loop backend | FAST-LIO-SAM-SC-QN README (pin commit) | Loop closure reference; not fixed-map runtime | https://github.com/engcang/fast-lio-sam-sc-qn |
| C++17, low dependencies, parallel CPU, no GPU in small_gicp | current repository README (pin release/commit) | Pi-friendly refinement path | https://github.com/koide3/small_gicp |
| 0.1 m minimum; 0.1–0.2 m reference; caveats to 1 m | Livox product specs | Do not treat 0.60 m as physical blind zone | https://www.livoxtech.com/mid-360/specs |

## Options A/B/C
### A — Harden current fixed-map localizer (recommended first)
Keep FAST-LIO + `map→odom` correction, replace scalar one-shot gate with bounded C++ registration (small_gicp or NDT), covariance/Hessian observability, innovation limit, timestamp/TF checks, and consecutive multi-scan confirmation. Keep Open3D only offline/debug. Scope: medium (roughly 1–3 weeks engineering plus field tuning). Portable: high; Pi: likely feasible with downsampling, one worker, capped map tile and 1–2 Hz correction. Rollback: disable correction node and restore FAST-LIO `odom` path. Test: injected 0.39 m drift must recover without deadlock, while a wrong-map/large-jump case must remain rejected.

### B — Descriptor-assisted global relocalization
Add ScanContext candidate retrieval and robust global registration (Quatro/TEASER++ or bounded small_gicp), then hand the result to A's health gate. Scope: large (3–6 weeks; dependency/licensing and ROS 2 port review). Portable: high if native and sensor-only; Pi: descriptor plus sparse registration may fit, but benchmark peak CPU/RAM. Rollback at candidate generator boundary. Test: cold starts across route and kidnapped-robot cases, with false-positive rate measured; no correction on ambiguous geometry.

### C — Pose-graph/SLAM backend
Adopt FAST-LIO-SAM/LIORF-style loop closure and graph optimization, periodically aligning the live graph to a prior map. Scope: very large and invasive; community implementations are commonly ROS1/Catkin or research repositories. Pi: risky memory/latency and graph growth. Rollback is difficult (backend changes map/odom semantics). Use only if repeated route loop closure is a requirement, not as the first response to a deadlocked gate. Test graph consistency, relocalization after long loops, bounded memory, and continued high-rate odometry.

## One recommended staged route
1. Establish interfaces and replay tests: `odom→base_link` remains FAST-LIO; localization publishes only validated `map→odom`; ground truth remains scoring-only. Add diagnostics and a watchdog.
2. Implement A in native C++ (or a minimal C++ registration node), tile/downsample PCD, and validate multi-scan stationary relocalization. Keep the Python prototype archived and do not restore it to the runtime path.
3. Add Rotation Shim around the existing controller in Humble as the low-risk immediate fix, retaining the measured 0.62 m polygon footprint. Require collision simulation during rotation and verify the bottleneck entry yaw.
4. In an isolated planner profile, replace SmacPlanner2D with the installed Smac State Lattice `omni` primitives. If it meets planning latency and corridor success gates, let the lattice path provide meaningful orientations to MPPI; otherwise retain SmacPlanner2D plus explicit pre-alignment route waypoints.
5. Replace the arbitrary obstacle minimum with measured sensor/mounting validation; retain Collision Monitor as fail-safe. Benchmark Pi CPU/RAM and end-to-end latency before field deployment.
6. Only then trial localization Option B; defer C.

**Do not do:** feed ground truth into TF or corrections; use known spawn pose/perfect clock; tune Gazebo noise as a hardware remedy; silently overwrite FAST-LIO odometry; accept one high ICP fitness score; rely on Smac 2D or `use_path_orientations` to solve heading; or use a 0.60 m filter as collision assurance.

## Acceptance matrix
| Area | Falsifiable acceptance test |
|---|---|
| Localization accuracy/health | On recorded sensor-only route with ~0.39 m induced drift, correction converges within agreed time and final error target; 3 consecutive compatible scans required; stale TF, bad score, degenerate Hessian, wrong tile and large innovation are rejected and diagnostic state changes. |
| Narrow corridor | With 0.62 m square footprint and 0.781 m bottleneck, approach yaw stays within ±18° (prefer margin, e.g. ±12°); no footprint collision in repeated forward and reverse trials; failed alignment stops rather than pushes through. |
| Command safety | Collision Monitor stops for occupied test zones; no command is emitted when localization is stale beyond timeout or registration is ambiguous; recovery/Spin cannot bypass safety envelope. |
| CPU/latency | Pi 5-class hardware sustains required Nav2/FAST-LIO rates and RTF ≥0.8 on representative route; registration has bounded wall time and memory, no Python/Open3D callback blocking. |
| No truth dependency | Runtime launch contains no `/ground_truth`, scoring topic, spawn-pose initialization, or simulator-only clock/noise dependency; tests replay Livox/IMU/TF/PCD only. |

## Open decisions
- Exact Pi 5 model, RAM, OS/compiler, thermal envelope, and required correction rate/latency.
- MID-360 mounting height/orientation, occlusion by chassis, IMU extrinsics, PTP availability, and measured point density in the corridor.
- Whether routes have repeatable stop points suitable for stationary relocalization and whether reverse travel is required.
- Target localization error, maximum permitted degraded-localization duration, and whether a human/operator recovery action is available.
- Which ROS 2 Humble Nav2 patch level is installed; verify Spin/Rotation Shim/MPPI plugin parameters against that exact package set.

## Gaps
No primary source found that establishes a universally valid ICP fitness threshold for this MID-360/map/footprint, nor a Pi-5 benchmark for the exact FAST-LIO + registration stack. FAST-LIO-SAM-SC-QN is a community research implementation and its ROS version/dependencies must be audited before use. Hardware bag replay and corridor tests are required to set thresholds.

## Sources
- Kept: FAST-LIO upstream — LiDAR-inertial odometry foundation (https://github.com/hku-mars/FAST_LIO)
- Kept: FAST-LIO-SAM-SC-QN — concrete ScanContext/Quatro/GICP graph architecture (https://github.com/engcang/fast-lio-sam-sc-qn)
- Kept: Autoware NDT scan matcher README — fixed-map interfaces and diagnostics (https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/README.md)
- Kept: Nav2 Humble Rotation Shim and MPPI docs — version-specific controller behavior (https://api.nav2.org/nav2-humble/html/md_nav2_rotation_shim_controller_README.html; https://docs.ros.org/en/humble/p/nav2_mppi_controller/)
- Kept: Livox MID-360 specs — manufacturer range caveats (https://www.livoxtech.com/mid-360/specs)
- Kept: small_gicp — native CPU registration option (https://github.com/koide3/small_gicp)
- Dropped: generic blog/tutorial comparisons — secondary, unpinned, and not evidence for Humble or Pi behavior.

## Supervisor coordination
Completed; no decision request needed.
