# Research: High-fidelity mecanum simulation in Fortress / gz-sim6 with DART 5.4

## Summary
For Fortress (`gz-sim6` / `ign-gazebo6`), upstream's mecanum example models each wheel as a spherical collision and obtains roller-like behavior from anisotropic friction: `mu=1`, `mu2=0`, and an `fdir1` expressed in the chassis frame at +/-45 degrees. This is a deliberately efficient approximation, not explicit passive roller geometry. The stock `MecanumDrive` converts body `cmd_vel` to four wheel angular velocity commands; it does not implement motor torque or a roller assembly.

For motor/actuator experiments, use four `JointController` systems (velocity mode for kinematic command following, force mode for PID torque-like behavior) or a custom actuator system. For physical contact behavior, retain DART contact/friction and validate against measured trajectories and slip; do not assume that a velocity command is equivalent to a torque-controlled motor. The primary sources do not establish a universally correct DART 5.4 solver iteration/timestep recipe, so those values must be benchmarked for the target model.

## Findings
1. **Fortress's official mecanum example uses implicit rollers, not explicit passive roller links.** The `ign-gazebo6` example gives each wheel a spherical collision (`<sphere><radius>0.3</radius>`) and a revolute wheel joint. There are no roller links, roller joints, or individual roller contacts. Its visual mesh is commented out. This is verified directly in the upstream SDF. [Fortress example SDF](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/mecanum_drive.sdf)

2. **The example's roller effect is anisotropic friction in a fixed frame.** Wheel contacts use `<mu>1.0</mu>`, `<mu2>0.0</mu2>`, and `fdir1 ignition:expressed_in="chassis">1 -1 0</fdir1>` or `1 1 0`, alternating by wheel. The upstream demo PR explains that the directions are chassis-fixed and 45 degrees from the wheel revolute axis, with friction 1 in one direction and 0 in the perpendicular direction. This depends on DART support for a friction direction expressed in another body/frame. [Example SDF](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/mecanum_drive.sdf) [Mecanum demo PR](https://github.com/gazebosim/gz-sim/pull/683) [DART custom-direction issue](https://github.com/dartsim/dart/issues/141)

3. **`MecanumDrive` is a velocity-command system, not a torque motor model.** Fortress source computes `angularLength = 0.5 * (wheelSeparation + wheelbase)` and sets wheel speeds to `(lin-lat-ang*angularLength)/r`, `(lin+lat+ang*angularLength)/r`, `(lin+lat-ang*angularLength)/r`, and `(lin-lat+ang*angularLength)/r`. In `PreUpdate` it writes `JointVelocityCmd` components for the four configured joints. It subscribes to `/model/<name>/cmd_vel` by default and can apply acceleration/velocity/jerk limiters. [MecanumDrive.cc, ign-gazebo6](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/mecanum_drive/MecanumDrive.cc)

4. **The plugin's velocity commands and anisotropic contact are complementary mechanisms.** The joint velocity command establishes wheel spin, while DART contact friction transfers contact impulses to chassis motion and constrains the permitted tangential direction. The upstream maintainer explicitly describes the approach as a constraint-solver contact velocity method and says the mecanum behavior is encoded by friction parameters, rather than a wrench applied by the plugin. [Upstream discussion, issue 2261](https://github.com/gazebosim/gz-sim/issues/2261)

5. **Explicit rollers are a higher-cost alternative with additional failure modes.** Upstream's mecanum demo issue notes that simulating individual wheels/rollers can work but reduces performance because of the number of contacts; the demo was created specifically to demonstrate anisotropic friction. This is an upstream engineering observation, not a quantitative DART 5.4 benchmark. Explicit rollers are appropriate when roller geometry/contact transitions are themselves under test, but should be compared against the anisotropic baseline with profiling and trajectory tests. [Demo issue](https://github.com/gazebosim/gz-sim/issues/579)

6. **`JointController` supports either direct velocity commands or force/PID mode.** The Fortress source accepts one or more `joint_name` elements. With `use_force_commands=false` (default), it writes `JointVelocityCmd`; with it true, it reads `JointVelocity`, computes PID output, and writes `JointForceCmd`. Defaults include `p_gain=1`, `i_gain=0`, `d_gain=0`, and command limits +/-1000, so force mode requires deliberate tuning and saturation selection. [JointController.cc, ign-gazebo6](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/joint_controller/JointController.cc) [Fortress API](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointController.html) [Fortress example](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/joint_controller.sdf)

7. **`VelocityControl` is not an actuator substitute.** Its source writes model/link linear and angular velocity command components, which directly imposes velocity at the model or link level. It is useful for controlled kinematic experiments, but it bypasses wheel-motor dynamics and should not be used to claim motor/contact fidelity. [VelocityControl.cc, ign-gazebo6](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/velocity_control/VelocityControl.cc)

8. **SDFormat defines the relevant friction/contact knobs, but engine support is the limiting contract.** SDF 1.12 documents `mu`, `mu2`, `fdir1`, ODE slip parameters, contact `kp`/`kd`, `max_vel`, `min_depth`, and `contact_surface_layer`; `fdir1` defines the principal direction and `mu`/`mu2` bound friction in the two directions. The exact Fortress/DART 5.4 support should be verified in the installed `gz-physics`/DART build, because the Gazebo physics documentation warns that support varies by engine/version. [SDF collision spec](https://sdformat.org/spec/1.12/collision/) [SDF physics spec](https://sdformat.org/spec/1.12/physics/) [Gazebo physics documentation](https://gazebosim.org/libs/physics/)

9. **Timestep is an accuracy/performance control, not a fidelity guarantee.** The upstream Fortress example uses `max_step_size=0.001` s and real-time factor 1. The Fortress docs state that smaller steps improve calculation accuracy but require more computation. No primary source found specifies a DART 5.4 iteration count or contact stiffness that is correct for all mecanum models; high stiffness, deep penetration correction, excessive contacts, and aggressive force PID can destabilize or reduce real-time performance. [Fortress example SDF](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/mecanum_drive.sdf) [SDF worlds docs](https://gazebosim.org/docs/fortress/sdf_worlds/)

## Verified behavior vs recommendations
**Verified:** stock Fortress example uses spheres plus anisotropic friction; wheel joints are revolute; `MecanumDrive` writes four velocity commands using the standard mecanum inverse mapping; `JointController` offers velocity and PID force modes; SDF exposes friction/contact parameters.

**Recommendations:** begin with the upstream anisotropic sphere baseline for efficient chassis-level motion. Add explicit passive rollers only if roller contact geometry, impacts, or load transfer are acceptance requirements. For motor realism, replace `MecanumDrive`'s direct velocity actuation with four force/PID controllers or a custom torque/current model, while retaining the friction model. Tune timestep, contact parameters, and PID together from measured tests; never infer fidelity from visual motion alone.

## Unresolved uncertainties
- Whether the exact deployed Fortress binary's DART 5.4 and `gz-physics` build honors every custom `fdir1 expressed_in` path exactly as the source/example implies; test the installed build.
- No upstream quantitative comparison was found between explicit rollers and anisotropic friction for DART 5.4 (error, contact count, or real-time factor).
- The example's `mu=1`, `mu2=0` are demonstration values, not validated material parameters; wheel load distribution and tire/roller compliance remain model-specific.
- `JointController` force mode is PID force output, not a documented motor electrical model, gearbox, torque-speed curve, or current limiter.

## Recommended prototype acceptance criteria
- **Kinematics:** command isolated +X, +Y, and +yaw motions; measured chassis twist stays within a project-defined tolerance (suggest initial target <=5% steady-state magnitude error and <=5 deg heading error over a fixed test window).
- **Wheel mapping:** log all four joint velocities and verify signs/magnitudes against the source equations for at least 5 mixed twists, including zero and opposite wheel speeds.
- **Contact behavior:** with force mode, verify static hold, acceleration, braking, and lateral translation under at least two payloads; report slip ratio and chassis drift, not only odometry.
- **Stability:** run 60 s repeated commands at the chosen timestep; require no explosive energy/velocity growth, persistent penetration, wheel chatter, or NaN/Inf state.
- **Performance:** record real-time factor and step-time percentiles with contact counts for baseline versus explicit-roller candidate; set an application threshold (for example, RTF >= 0.9 on target hardware).
- **Regression:** repeat tests with friction direction signs swapped and with `mu2` nonzero to catch frame/sign/configuration errors; archive SDF, engine versions, and logs.

## Sources
- Kept: [gz-sim Fortress mecanum example](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/mecanum_drive.sdf) — canonical model and physics settings.
- Kept: [`MecanumDrive.cc` on ign-gazebo6](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/mecanum_drive/MecanumDrive.cc) — exact command mapping and component writes.
- Kept: [`JointController.cc` on ign-gazebo6](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/joint_controller/JointController.cc) — velocity versus force/PID behavior.
- Kept: [gz-sim issue 2261](https://github.com/gazebosim/gz-sim/issues/2261) — maintainer explanation of friction/contact approach.
- Kept: [gz-sim issue 579 / PR 683](https://github.com/gazebosim/gz-sim/issues/579) — rationale and performance warning for anisotropic friction.
- Kept: [SDFormat collision spec](https://sdformat.org/spec/1.12/collision/) and [physics spec](https://sdformat.org/spec/1.12/physics/) — parameter semantics.
- Dropped: third-party tutorials and search summaries — not needed where upstream source/spec evidence exists.

## Gaps
There is no primary-source DART 5.4 benchmark establishing an explicit-roller fidelity threshold, universal solver iteration count, or validated friction coefficients. Resolve these with an instrumented A/B prototype on the deployed Fortress package and physical reference data.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Created only the mandated research artifact; scope is limited to the requested Fortress/DART mecanum physics report."
    },
    {
      "id": "criterion-2",
      "status": "satisfied",
      "evidence": "Report contains primary-source URLs, verified-versus-recommended distinctions, uncertainties, and executable prototype acceptance criteria."
    }
  ],
  "changedFiles": [
    "/home/gxxl/.pi/agent/sessions/--home-gxxl-soarm-car--/subagent-artifacts/outputs/954ac4ba-4fcc-441a-91de-9c4413a971f8/research.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [],
  "validationOutput": [
    "Primary-source URLs were fetched and the Fortress branch SDF, MecanumDrive.cc, JointController.cc, and VelocityControl.cc were inspected."
  ],
  "residualRisks": [
    "Exact installed DART 5.4/gz-physics support and numerical tuning still require an in-environment A/B prototype."
  ],
  "noStagedFiles": true,
  "diffSummary": "Added a concise cited research report as the sole artifact.",
  "reviewFindings": [
    "no blockers"
  ],
  "manualNotes": "Runtime artifact path is authoritative; the requested repository docs path was not modified."
}
```