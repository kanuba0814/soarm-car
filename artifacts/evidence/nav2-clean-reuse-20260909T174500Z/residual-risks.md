# Residual Risks

- No `NavigateToPose` goal was accepted or completed. There is no action status, physical source-odometry trajectory, path, estimated-to-truth gap, cancellation flag, or FAST-LIO health result for this invocation.
- The unchanged consumer publishes a fixed zero `/initialpose` and then waits for healthy localization. The intended run therefore assumes that fixed initial pose; it does not derive it from ground-truth odometry. Targets are position-only even though the serialized goal contains yaw `0.0`.
- The required single-run budget was consumed. A future execution must choose a distinct Fast DDS-valid ROS domain (`0..232`) and a new Ignition partition; this result must not be represented as navigation success.
- The first GUI restore attempt from the service worktree was refused because its AWS asset is absent. The companion was subsequently restored from its recorded original worktree, `/home/gxxl/soarm-car-companion-validation`, with the original GUI identity and environment.
