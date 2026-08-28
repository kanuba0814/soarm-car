# Visualization asset provenance

## WHEELTEC mini mecanum chassis

Identity and proportions were established only from the user-supplied WHEELTEC material, not community models:

- Archive: `/home/gxxl/Documents/wheeltec robot ROS资料/1.WHEELTEC ROS机器人通用资料/5.ROS1系列教程/1.ROS基础干货视频教程/10.精细URDF模型配置使用（暂无视频）/URDF原始导出功能包.zip`
- Best match: `URDF原始导出功能包/mini_mec_moveit_six`.
- Evidence: vendor model wheelbase about 173 mm, track about 198 mm and four 75 mm mecanum wheels; closest supplied asset to the pictured chassis.

The repository intentionally does **not** retain the vendor CAD meshes. The displayed chassis and wheels use simple URDF primitives with approximately the same proportions because the current purpose is state/layout review, not mechanical CAD. This also avoids unclear vendor mesh redistribution terms.

## SO-101

- Reference: local checkout `/home/gxxl/SO-ARM100/Simulation/SO101`
- Upstream: `https://github.com/TheRobotStudio/SO-ARM100.git`
- Commit: `fda892cba81032c46c40976a48c9ceadbf40a9ca`
- Reference model: `so101_new_calib.urdf`
- License: Apache-2.0 in the upstream repository.

The displayed arm is a lightweight primitive approximation. It preserves the official model's recognizable six-joint structure, link-scale order, joint names and approximate limits so RViz can show posture without carrying 16 MB of detailed meshes.

## Sensor visuals

- Camera: dimensioned placeholder because the exact Orbbec product model remains unidentified. It must be replaced after confirming the nameplate/model.
- MID-360: simple visual envelope only. Livox publishes an official MIT xacro in `Livox-SDK/livox_laser_simulation`, but the current layout needs only a non-simulated envelope and frame.

## Coordinate and layout status

`base_link +X` is the proposed new vehicle front after reversing the previous front/rear convention. The arm defaults to the previous camera area (new rear). Camera and arm layout joints are visual calibration controls only and must never enter the hardware control interface.
