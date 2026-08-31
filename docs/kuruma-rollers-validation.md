# Kuruma explicit-roller validation

## Status

The explicit 64-roller model is dynamically stable but is not promoted to the
default algorithm-development model. It misses the declared real-time factor
acceptance threshold of `0.8` on the target Fortress 6.18 / DART 5.4 runtime.
The reduced-order `kuruma` model remains the default.

## Consumer seam

Validation uses `scripts/dev-kuruma-rollers-smoke.sh`, which launches the actual
`kuruma_rollers.sdf` world through ROS 2 and Fortress. It verifies:

- four hub joints and 64 passive roller joints;
- STEP-derived roller ring, spacing, and wheel handedness;
- isolated positive X, positive Y, and positive yaw motion;
- passive roller position changes on every wheel during lateral motion;
- 10 seconds of zero-command drift;
- finite state after more than 30 seconds of simulation;
- DART's `/world/kuruma_rollers_lab/stats` real-time factor.

## Configuration A: triangle-mesh collision

Each roller used the generated 340-vertex, 640-face OBJ for both visual and
collision geometry.

| Metric | Result |
| --- | ---: |
| X command displacement `(x, y, yaw)` | `(0.3040 m, 0.0032 m, 0.0026 rad)` |
| Y command displacement `(x, y, yaw)` | `(-0.0023 m, 0.3545 m, -0.0017 rad)` |
| Yaw command displacement `(x, y, yaw)` | `(0.0003 m, 0.0005 m, 0.6446 rad)` |
| Zero-command drift `(x, y, yaw)` | `(-0.000031 m, 0.000006 m, 0.000062 rad)` |
| Real-time factor | `0.3213` |

Directionality, passive motion, finite state, and drift passed. Performance
failed.

## Configuration B: measured capsule collision

The visual OBJ remains unchanged. Each collision is the equivalent measured
capsule: radius `0.0092268 m`, cylindrical section `0.0195464 m`, and overall
length `0.038 m`.

| Metric | Result |
| --- | ---: |
| X command displacement `(x, y, yaw)` | `(0.4963 m, 0.00003 m, 0.00002 rad)` |
| Y command displacement `(x, y, yaw)` | `(0.0110 m, 0.5224 m, 0.0004 rad)` |
| Yaw command displacement `(x, y, yaw)` | `(0.0034 m, -0.0020 m, 1.0458 rad)` |
| Zero-command drift `(x, y, yaw)` | `(0.0 m, 0.0 m, 0.0 rad)` |
| Real-time factor | `0.6225` |

The capsule collision improved both directionality and performance, but the
real-time factor still failed the `0.8` threshold.

## Decision

Stop contact-parameter tuning after these two evidence-driven configurations.
The model is retained as an isolated contact-physics profile and failure
evidence, not deleted or silently weakened. Further promotion work requires one
of:

1. a measured Kuruma trajectory dataset showing that explicit contacts reduce a
   relevant model error enough to justify sub-real-time execution;
2. a physics-engine or contact-reduction change that reaches `RTF >= 0.8` while
   preserving 64 independently rotating rollers; or
3. a separate offline contact-study workflow where real-time execution is not a
   requirement.

Friction, mass, inertia, damping, motor torque, and floor properties remain
provisional until measured on the physical chassis.
