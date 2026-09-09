# Display-Backed Renderer Parity Fallback

- Immutable source: `/home/gxxl/soarm-car-nav2-clean`, head `41463f9a07dc317eb837558ad55b137fb92e35eb`.
- New owned runtime: `/tmp/soarm-nav2-renderer-parity-20260909`; source comparison outside `.git` and generated `artifacts` found only the owned runner overlay.
- Pinned image: `soarm-car-ros2-dev`, `sha256:1da9a18609c4664d7441d9f334c95e971f0795aa3815f6707ed761f2ce93d227`.
- Overlay: `--gpus all`, `/dev/dri`, X11 socket/Xauthority mount, `DISPLAY=:0`, NVIDIA graphics capabilities, NVIDIA GLX/EGL settings, runtime directory, and NVIDIA EGL vendor file before the existing launch command. Exact diff: `renderer-overlay.diff`.
- Probe: same pinned image and parity environment reported direct GLX rendering with NVIDIA RTX 3070 and OpenGL 4.6. Full `eglinfo` exit was nonzero only because its X11 EGL platform subcheck failed; device-platform EGL was NVIDIA-capable.
- Command: `NAV2_CLEAN_CASE=direct_mppi NAV2_CLEAN_ROUTE=room-1 ROS_DOMAIN_ID=211 IGN_PARTITION=soarm-nav2-renderer-parity-20260909T091000Z DISPLAY=:0 XAUTHORITY=/run/user/1000/xauth_ZPyIRJ timeout 1500 scripts/run-nav2-clean-comparison.sh`.
- Result: passed. Existing consumer recorded `room-1` status `SUCCEEDED` (`4`) and `hub` / `mian` status `SUCCEEDED` (`4`), both without cancellation.
- Telemetry: clock and source FAST-LIO stamps positive and monotonic at initialization. Route health was healthy (`1`) with zero rejections. `room-1`: 43.516 simulation seconds, 0.13414 m truth goal gap, 0.05720 m maximum localization-to-truth error. `hub`: 20.101 simulation seconds, 0.12562 m truth goal gap, 0.09014 m maximum localization-to-truth error.
- GUI restored: `soarm-companion-home-demo`, `soarm-car-ros2-dev:hm3d`, domain `174`, partition `soarm-companion-home-demo-ign`.

This evidence commit contains no owned runtime source file. Immutable source and the two earlier recovered runtimes were not modified. The overlay is retained as evidence only and is not a navigation-algorithm change.
