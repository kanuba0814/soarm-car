# Bounded Existing Navigation Reuse Check

- Runtime source: `/tmp/soarm-nav2-clean-parent-restore-20260909`, `41463f9a07dc317eb837558ad55b137fb92e35eb` with its pre-existing dirty snapshot.
- Image inspected before run: `soarm-car-ros2-dev`, `sha256:1da9a18609c4664d7441d9f334c95e971f0795aa3815f6707ed761f2ce93d227`.
- Unchanged runner: `scripts/run-nav2-clean-comparison.sh`, SHA-256 `2e471c5befc343690c9e124bd4e7de392fce771bc8049a512389cbf66f7fd61b`.
- Command: `NAV2_CLEAN_CASE=direct_mppi NAV2_CLEAN_ROUTE=room-1 ROS_DOMAIN_ID=246 IGN_PARTITION=soarm-nav2-reuse-direct-mppi-20260909 timeout 1500 scripts/run-nav2-clean-comparison.sh`.
- Result: exit `1`; startup blocked by invalid Fast DDS domain before any route action. `runtime-summary.json` has no case metrics.
- Companion restored: `soarm-companion-home-demo`, image `soarm-car-ros2-dev:hm3d`, ROS domain `174`, Ignition partition `soarm-companion-home-demo-ign`. Its post-restore topic probe and process record are retained locally.

No navigation source, driver, map, controller, behavior tree, tolerance, TF, or consumer source was edited. The recovered runtime received only its runner-generated artifact directory; this Git commit contains only this compact evidence directory.
