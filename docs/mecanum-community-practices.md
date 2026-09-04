# Community practice for mecanum simulation

## Question

What model architecture is currently used for mecanum robots when the simulator
must support controller, localization, and navigation development, rather than
only demonstrate motion from `cmd_vel`?

## Short answer

The common production-oriented approach is layered fidelity, not one maximally
detailed wheel model:

1. Use real chassis dimensions and wheel visuals, but a reduced collision model
   with directional friction for routine navigation and integration tests.
2. Command the four wheel joints through a controller interface with limits and
   feedback, rather than treating chassis velocity as the complete actuator
   model.
3. Compute operational odometry from wheel state and keep simulator world pose
   as separate ground truth. This preserves slip as an estimator error instead
   of hiding it.
4. Use explicit passive rollers only for tests whose acceptance criteria depend
   on roller contact transitions, vibration, load transfer, or surface effects.

This pattern appears independently in Gazebo, `ros2_control`, Isaac Sim, and
Webots. Explicit rollers exist in community models, but they are not the default
architecture in the maintained simulator examples inspected here.

## Evidence from maintained platforms

### Gazebo Sim

The current upstream `mecanum_drive.sdf` uses detailed left/right wheel meshes
for visuals, but each wheel collision is a sphere. Mecanum behavior comes from
`mu=1`, `mu2=0`, and alternating `fdir1` directions expressed in the chassis
frame. The `MecanumDrive` system then commands four revolute wheel joints. This
is a deliberate contact approximation: the visual wheel and the physical wheel
do not need the same representation.

The earlier Fortress example uses the same essential model. Its design issue
states that explicit roller contacts are possible but more expensive, and the
demo was created to exercise DART's chassis-fixed anisotropic friction. A later
upstream discussion also confirms that the plugin sets wheel velocities while
the contact solver produces chassis forces; it does not simply apply a wrench to
the chassis.

Sources:

- [Current Gazebo Sim mecanum example](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/mecanum_drive.sdf)
- [Fortress mecanum example](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/mecanum_drive.sdf)
- [Gazebo mecanum demo design issue](https://github.com/gazebosim/gz-sim/issues/579)
- [Gazebo plugin behavior discussion](https://github.com/gazebosim/gz-sim/issues/2261)

### ROS 2 control

The maintained `mecanum_drive_controller` accepts `TwistStamped` forward,
lateral, and yaw velocity references. It exposes velocity commands for four
wheel joints, consumes wheel velocity states, and publishes odometry and TF.
This separates body-level command generation from the simulator hardware
interface and matches the interface used by a physical robot.

`gz_ros2_control` gained a Jazzy/Harmonic mecanum example in December 2024. Its
configuration includes a command timeout, wheel geometry, four joint names,
joint-state broadcasting, and odometry. Open issues are useful warnings rather
than evidence of a universal plug-and-play model: one reports missing lateral
motion on a Humble setup, and another shows that unbounded extreme commands can
destabilize the Harmonic demo. Limits and version-specific end-to-end tests are
therefore part of the model contract.

Sources:

- [`mecanum_drive_controller` user documentation](https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)
- [`gz_ros2_control` mecanum example merge](https://github.com/ros-controls/gz_ros2_control/pull/451)
- [Humble lateral-motion report](https://github.com/ros-controls/gz_ros2_control/issues/578)
- [Harmonic unbounded-command report](https://github.com/ros-controls/gz_ros2_control/issues/495)

### Isaac Sim

Isaac Sim's maintained `HolonomicController` formulates wheel command
generation as a quadratic program. It consumes wheel radii, positions,
orientations, mecanum angles, wheel axes, and body velocity targets, then emits
joint velocity commands. It also exposes maximum linear, angular, and wheel
speeds. This is a geometry-aware controller abstraction; the documentation does
not claim that it is a per-roller contact model.

Source: [Isaac Sim mobile robot controllers](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_simulation/mobile_robot_controllers.html#holonomic-controller)

### Webots

The maintained KUKA youBot model shows the same visual/physics split. Its wheel
PROTOs render multiple angled sub-wheel meshes, but use one cylindrical
`boundingObject` per wheel and assign an interior/exterior contact material.
Webots `ContactProperties` supplies asymmetric Coulomb friction and
`frictionRotation`; its documentation names youBot as the example. The contact
documentation also warns that contact-joint count has a significant performance
effect in the default ODE engine.

Sources:

- [Webots KUKA youBot model](https://github.com/cyberbotics/webots/blob/master/projects/robots/kuka/youbot/protos/Youbot.proto)
- [Webots interior wheel model](https://github.com/cyberbotics/webots/blob/master/projects/robots/kuka/youbot/protos/InteriorWheel.proto)
- [Webots contact properties](https://github.com/cyberbotics/webots/blob/master/docs/reference/contactproperties.md)

## Explicit roller models in the community

`rigbetellabs/mecanum_in_gazebo` is a concrete explicit-geometry example. Its
ROS 1 / Gazebo Classic URDF contains four hub joints and 36 roller joints, with
individual meshes, inertias, transmissions, and contact properties. It proves
that the topology is practical, but it is a single 2024 import rather than a
maintained cross-version reference, and it provides no quantitative accuracy or
real-time benchmark. It should not be treated as evidence that explicit rollers
are automatically more accurate.

Source: [Explicit roller Gazebo model](https://github.com/rigbetellabs/mecanum_in_gazebo)

## What fidelity means for upstream algorithms

More geometric detail only improves the effects represented by that geometry.
An explicit roller model driven by ideal joint velocities can still be a weak
motor-control model. A beautiful CAD mesh with ground-truth odometry can still
be a weak localization model. The relevant fidelity dimensions are separate:

| Development target | Required model behavior | Usually unnecessary |
| --- | --- | --- |
| Nav2 and planning | footprint, collision envelope, holonomic kinematics, command limits, stable real-time execution | explicit roller contacts |
| Localization and SLAM | realistic sensor frames/noise, encoder-derived odometry, slip-induced disagreement, separate ground truth | motor electrical dynamics |
| Chassis control | wheel state feedback, acceleration and torque limits, latency, saturation, payload response | full CAD visual geometry |
| Roller/contact research | passive roller joints, contact transitions, load distribution, surface and compliance calibration | high-throughput navigation worlds |

## Decision for Kuruma

Use two physical models and one common robot interface:

### Routine algorithm model

- Keep the measured `0.600 x 0.600 m` frame, measured wheel locations, honest
  chassis collision members, and real wheel visuals.
- Keep a reduced wheel collision with chassis-fixed anisotropic friction.
- Replace direct `MecanumDrive` actuation when controller fidelity becomes the
  target: use `ros2_control`-style four-wheel commands or a Fortress-compatible
  force controller with measured speed, acceleration, torque, and latency
  limits.
- Publish wheel-encoder odometry as the estimator input. Publish Gazebo world
  pose only on a clearly named ground-truth topic for scoring.
- Add calibrated IMU and lidar noise/bias separately from wheel physics.

This model should remain the default for Nav2, localization, CI, and interactive
development because repeatability and real-time factor are acceptance criteria.

### Explicit-roller model

- Keep `kuruma_rollers` isolated until its DART consumer tests pass.
- Use the STEP-derived 16 rollers per wheel, correct handedness, passive joints,
  low-complexity collision meshes, and measured mass/inertia when available.
- Evaluate it with joint-state evidence, trajectory error, zero-command drift,
  contact stability, and real-time factor. Visual inspection is not acceptance.
- Use it for contact-sensitive controller and estimator experiments, not as an
  automatic replacement for the routine model.

### Promotion rule

Do not promote the explicit model because it looks more physical. Promote it
only if, on the target Fortress 6.18 / DART 5.4 installation, it meets a declared
real-time threshold and reduces error against measured Kuruma trajectories for
the phenomena under test. Without physical measurements, it is a more detailed
hypothesis, not a validated digital twin.

## Immediate implementation order

1. Finish the isolated roller smoke test and collect corrected X, Y, yaw, idle,
   roller-joint, stability, and real-time-factor evidence.
2. Preserve the reduced model as the navigation default and explicit model as a
   selectable physics profile.
3. Introduce four-wheel controller and encoder-odometry interfaces shared by
   simulation and future hardware.
4. Add command limits and fault-safe timeouts before tuning contact coefficients.
5. Record physical straight, lateral, yaw, coast-down, and payload trajectories;
   calibrate both models against the same dataset.

## Remaining uncertainty

No maintained source found provides universal friction, contact, timestep,
roller inertia, or motor values for this wheel. Simulator examples establish an
architecture, not Kuruma's parameters. Those values must come from the actual
wheel, motor, payload, floor, and deployed DART build.
