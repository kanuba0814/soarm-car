# Residual Risks

- `room-1` was not submitted and `hub`/`mian` was not attempted. No Nav2 action status, plan, physical source-odometry trajectory, localization-to-truth error, cancellation flag, or FAST-LIO health sample exists for this retry.
- `/clock` was not established sufficiently for the existing consumer's pre-action gate. Lifecycle node existence alone is not evidence of viable simulator sensing or localization.
- The concrete valid-DDS dependency block is renderer initialization on `/dev/dri/card0` (`eglInitialize failed`, OpenGL 3.3 unavailable, Gazebo signal 139). The authorized retry budget is consumed; no renderer, source, model, map, controller, tolerance, BT, TF, or consumer changes were made to work around it.
- The unchanged consumer assumes its fixed zero `/initialpose` and position-only targets. This retry did not advance far enough to exercise that assumption.
