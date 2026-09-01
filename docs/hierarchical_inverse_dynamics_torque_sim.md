# R1 hierarchical inverse-dynamics torque simulation runbook

This runbook covers only the ROS-free, fixed-base simulation app
`mcl_hierarchical_inverse_dynamics_torque_sim`. It cannot publish effort to SynRobot or authorize
hardware motion.

## Build and run

Build Core and Sim first, then configure Lab against their install prefix and enable
`MCL_BUILD_HIERARCHICAL_INVERSE_DYNAMICS_TORQUE_SIM`. For the standard installed runtime:

```bash
MCL_INSTALL_PREFIX=/path/to/lab/install \
  apps/hierarchical_inverse_dynamics_torque_sim/scripts/run_headless.sh \
  --duration 1.0 --retarget-x 0.01
```

The default loop targets 1 kHz and uses ProxQP. Its three active levels are dual-hand position
Primary, orientation Secondary, and nominal actuator posture Tertiary. No Terminal pass is
registered. The Cartesian acceleration gains are `Kp=100 s^-2`, `Kd=20 s^-1`, the critically
damped acceleration form derived from the existing `10 s^-1` tracking rate.

Pause freezes planner, optional admittance, HID, simulation, feedback, and metrics together:

```bash
apps/hierarchical_inverse_dynamics_torque_sim/scripts/run_headless.sh \
  --paused --single-steps 1 --retarget-x 0
```

One requested step performs exactly one full control tick and one `mj_step()`.

For interactive control, use the installed keyboard launcher:

```bash
MCL_INSTALL_PREFIX=/workspace/install/algorithm \
  apps/hierarchical_inverse_dynamics_torque_sim/scripts/run_keyboard.sh
```

The launcher follows the `hierarchical_kinematics_step` planned-profile contract: it selects the
installed binary unless `MCL_BINARY` overrides it, uses the installed R1 assets,
passes through `launch_app.sh`, and preserves trailing CLI argument priority.
It also enables the same interactive presentation shape by default:

- full-screen alternate-screen TUI with Monitor, Motion, Solver,
  Requirements, Joints, System, and app-local Dynamics pages;
- Foxglove WebSocket at `ws://127.0.0.1:8765` using the existing MCL state,
  planning, and execution channel contracts;
- a native MuJoCo/GLFW window rendering the torque-driven state.

Left/Right selects the hand; W/S, A/D, and Q/E translate it; N selects the TCP
rotation axis; I/U rotates; Up/Down changes the step; R resets from measured
MuJoCo FK; Space freezes/resumes the whole pipeline; X or Esc exits. A changed
target replans from the current Cartesian P/V/A reference instead of restarting
from zero motion.

The MuJoCo hand markers are also target inputs: Ctrl+left-drag moves in the
view plane, Ctrl+Shift+left-drag changes depth, and Ctrl+right-drag rotates.
They update the same app-local target source used by the keyboard. Space keeps
TUI/window event handling responsive while freezing the planner, admittance,
HID solve, torque write, `mj_step()`, feedback, and control metrics.

Presentation can be selected independently:

```bash
MCL_UI=none MCL_VIZ=none MCL_MUJOCO_VIEWER=off \
  apps/hierarchical_inverse_dynamics_torque_sim/scripts/run_keyboard.sh \
  --duration 0.1
```

`MCL_VIZ_HOST`, `MCL_VIZ_PORT`, and `MCL_UI_RATE_HZ` override the default
`127.0.0.1`, `8765`, and `100 Hz`. `MCL_MCAP=/path/to/run.mcap` records the
same RenderBatch stream; an existing path is rejected rather than overwritten.
The Foxglove stream publishes input, goal, planner reference, measured FK/
execution, and measured joint state without adding a torque command topic.

## Model contract

The actuator command contains exactly the 20 curated R1 joints in `R1RobotConfig::joint_names`
order. The same profile owns the effort magnitudes; a regression test compares them with URDF
effort, MJCF `actuatorfrcrange`/motor range, and compares Core/MuJoCo bias forces at the nominal
state. No torque-rate limit is enabled because there is no authoritative R1 source in this slice.

The MJCF free joint is held by `fixed_base_weld`. Collision meshes are present, but this V1 fixed
base app disables MuJoCo contact dynamics because the controller registers no environment or
self-contact constraints. Contact-enabled/free-flyer experiments must explicitly model the same
contacts in HID and must not reuse this switch as a deployment shortcut.

## Acceptance evidence

Headless output reports solve-time P50/P95/P99, maximum TCP tracking error, minimum normalized torque
margin, maximum joint velocity/acceleration, maximum commanded torque, Core/MuJoCo acceleration
model error, and base speed. A smoke run fails fast on any rejected/non-finite HID result; it does
not retry with a previous tick torque.

For window validation, first confirm the actual GLX renderer, then run a
bounded smoke:

```bash
DISPLAY=:0 glxinfo -B
MCL_DURATION=0.1 MCL_UI=none MCL_VIZ=none \
  apps/hierarchical_inverse_dynamics_torque_sim/scripts/run_keyboard.sh
```
