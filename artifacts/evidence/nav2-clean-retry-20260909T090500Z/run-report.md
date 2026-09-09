# Startup-Corrected Existing Navigation Reuse Attempt

- Runtime source: `/tmp/soarm-nav2-clean-parent-restore-20260909`, head `41463f9a07dc317eb837558ad55b137fb92e35eb`.
- Pinned image inspected before launch: `soarm-car-ros2-dev`, `sha256:1da9a18609c4664d7441d9f334c95e971f0795aa3815f6707ed761f2ce93d227`. Image environment contains no `ROS_DOMAIN_ID`; the unchanged runner supplies `-e ROS_DOMAIN_ID=211`.
- Command: `NAV2_CLEAN_CASE=direct_mppi NAV2_CLEAN_ROUTE=room-1 ROS_DOMAIN_ID=211 IGN_PARTITION=soarm-nav2-retry-20260909T090500Z timeout 1500 scripts/run-nav2-clean-comparison.sh`.
- Result: exit `1`. Valid DDS reached lifecycle readiness and started the unchanged consumer. The simulator renderer then blocked sensor/FAST-LIO readiness, preserving an empty route rather than sending an action.
- Step statuses: `room-1` not submitted; `hub` (`mian`) not attempted.
- Companion GUI restored: `soarm-companion-home-demo`, `soarm-car-ros2-dev:hm3d`, domain `174`, partition `soarm-companion-home-demo-ign`.

This is a separate evidence-only record. Prior failure commit `8ca93d1` is retained unmodified. No navigation implementation, action client, or runtime source behavior was changed.
