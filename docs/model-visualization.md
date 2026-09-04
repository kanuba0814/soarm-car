# Kuruma model visualization

The default display is the chassis-first Kuruma model derived from `artifacts/kuruma.STEP`. It uses primitive geometry for the measured `0.600 x 0.600 m` 3030 perimeter, central electronics envelope, motor/suspension placeholders, and four mecanum wheel envelopes. Wheel radius is `0.0763 m`, wheelbase is `0.3472 m`, and track is `0.4456 m`. The symmetric planar wheel placement intentionally does not encode the CAD's approximately 19 mm suspension-height and 16 mm longitudinal asymmetry.

Camera, IMU, and MID-360 frames are visibly retained as provisional navigation mounts, not mechanical measurements. There is no displayed arm or arm mount. Arm mounting dimensions and placement are the next mechanical study.

```bash
./scripts/dev-build.sh
./scripts/dev-view-model.sh
```

The prior WHEELTEC chassis configuration is available at `backup/wheeltec-chassis`.
