# Hierarchical inverse dynamics torque simulation

This app is the ROS-free fixed-base R1 torque slice. At 1 kHz it advances:

`CartesianPlanner P/V/A -> optional CartesianAdmittance -> HID -> qfrc_applied -> one mj_step -> measured state`.

The R1 hierarchy has three active levels: dual-hand position Primary,
dual-hand orientation Secondary, and nominal posture Tertiary. There is no
Terminal pass. `--paused` freezes the complete pipeline; `--single-steps N`
advances exactly `N` control ticks while paused. The named base weld is enabled
and contact dynamics are explicitly disabled because this fixed-base V1 app
has no registered collision/contact constraints. The app never publishes
hardware commands.

Run the installed binary through:

```bash
run_headless.sh --duration 0.1 --retarget-x 0.01
```

See [the runbook](../../docs/hierarchical_inverse_dynamics_torque_sim.md) for model, metrics, and
safety boundaries.

For interactive keyboard control, run:

```bash
apps/hierarchical_inverse_dynamics_torque_sim/scripts/run_keyboard.sh
```

This launcher opens the full-screen planned-style TUI, a Foxglove WebSocket
server at `ws://127.0.0.1:8765`, and the native MuJoCo window by default. The
TUI exposes Monitor, Motion, Solver, Requirements, Joints, System, and
Dynamics pages. The Dynamics page shows the selected/fallback HQP level and
the 20 torques in actuation order. Set `MCL_UI=none`, `MCL_VIZ=none`, or
`MCL_MUJOCO_VIEWER=off` to disable one presentation surface independently.

The controls match the `hierarchical_kinematics_step` planned profile: Left/Right selects the arm,
W/S moves X, A/D moves Y, Q/E moves Z, N selects the local rotation axis,
I/U rotates, Up/Down changes the translation step, R resets the selected target,
Space pauses the complete planner/HID/MuJoCo pipeline, and X or Esc exits.
In the MuJoCo window, Ctrl+left-drag moves a hand marker in the view plane,
Ctrl+Shift+left-drag adds depth, and Ctrl+right-drag rotates it. Dragging and
keyboard edits feed the same Cartesian planner target revision.
