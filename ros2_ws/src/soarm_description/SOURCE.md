# Kuruma model provenance

The active chassis is derived from `artifacts/kuruma.STEP` (AP203 SolidWorks 2022, millimetres), represented with reproducible primitives rather than an exported mesh. Coordinate mapping is STEP +Z to ROS +X forward, STEP +X to ROS +Y left, and STEP +Y to ROS +Z up.

Measured parameters are the four wheel outer roller envelope radius `0.0763 m`, envelope width `0.0481 m`, planar wheelbase `0.3472 m`, track `0.4456 m`, and 3030 perimeter `0.600 x 0.600 m`. The central wheel disc (`0.07144 m` radius, `0.024 m` width) is not used as the rolling envelope. The CAD suspension poses are asymmetric by about 19 mm vertically and 16 mm longitudinally, so the model deliberately uses symmetric planar kinematics.

The frame perimeter, electronics envelope, motor/suspension placeholders, and wheel envelopes are recognizable geometry. Mass distribution and suspension height remain provisional. `camera_link`, `gyro_link`, and `livox_frame` are provisional navigation mounts retained for existing topic and TF consumers; their placements are not STEP-derived. There is no arm geometry or arm mount in this chassis-first model. The next mechanical study is arm mounting dimensions and position.
