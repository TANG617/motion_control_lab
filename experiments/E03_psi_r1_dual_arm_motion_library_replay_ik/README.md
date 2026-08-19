# E03: PSI R1 dual-arm motion-library batch replay IK

E03 evaluates every direct `*.mcap` child of the PSI R1 dual-arm motion library with one
fixed ordinary MCC IK policy. The default library is:

```text
/workspace/fixtures/datasets/psi_r1_dual_arm_motion_library/
```

Each filename stem is its action ID. Action IDs must match
`^[A-Za-z0-9][A-Za-z0-9._-]*$`; nested files and non-MCAP files are ignored, while any
symlink makes the library invalid. At run start, E03 sorts the direct MCAP children by
filename and records each file's size and SHA-256. Files added after that snapshot are not
part of the run. A snapshotted file that changes fails as `input_changed`.

Every action must provide `/mc/ik/joint_states`, `/mc/ik/target/left_pose`, and
`/mc/ik/target/right_pose`. Its first JointState initializes all 20 configured joints by
name with zero velocity. Each action creates a fresh model, solver, and state; later
JointState samples remain input evidence while IK evolves from `previous_solution`.
Pose pairing uses `header_stamp`, nearest within 5 ms, and unmatched samples are errors.
The 10 ms ServoStep and TCP offsets are fixed in the shared replay engine and are not CLI
parameters.

Build and run headless:

```bash
cmake -S . -B /workspace/build/motion_control_lab
cmake --build /workspace/build/motion_control_lab --target mcl_e03_batch_replay_ik -j8

/workspace/build/motion_control_lab/mcl_e03_batch_replay_ik \
  --urdf /workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf
```

For continuous Foxglove playback, configure visualization support and add `--visualize`:

```bash
cmake -S . -B /workspace/build/motion_control_lab \
  -DMCL_BUILD_E03_REPLAY_VISUALIZATION=ON \
  -DCMAKE_PREFIX_PATH="/workspace/install/motion_control_core;/workspace/install/motion_control_viz"
cmake --build /workspace/build/motion_control_lab --target mcl_e03_batch_replay_ik -j8

/workspace/build/motion_control_lab/mcl_e03_batch_replay_ik \
  --urdf /workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf \
  --visualize \
  --playback-rate 10
```

Connect Foxglove to `ws://127.0.0.1:8765`. E03 opens one server for the entire batch,
publishes each action's initial frame immediately, and does not wait for Space. The global
visualization sequence is monotonic across action boundaries. It publishes the Lab-wide
five-topic contract documented in
[Foxglove IK visualization contract](../../docs/foxglove_ik_visualization_contract.md).
Visualization MCAP recording is intentionally not part of E03 v1.

Runs are append-only under `runs/<run-id>/`. An action stops after writing its first failed
solve row, but the batch continues with later actions. The process exits successfully only
when the library is non-empty and every action completes with zero rejected solves. `runs/`
and `results/` are local generated-data boundaries; promote reviewed evidence separately.
