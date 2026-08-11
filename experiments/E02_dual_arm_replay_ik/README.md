# E02: PSI R1 dual-arm canonical replay IK

E02 replays the fixed PSI R1 MCAP dual-arm TCP targets through the MCC Red-only IK
topology. The first `/mc/ik/joint_states` sample initializes the 20 configured joints by
name. Each TCP target is converted with the declared `0.1 m` tool offset before solving.

By default, every invocation creates an append-only directory under `runs/` named
`<UTC timestamp>__<definition hash>`. Use `--output-root` to redirect the parent directory
for tests, or `--output-dir` for a one-off exact destination.

From the Lab repository root, build replay visualization support with:

```bash
cmake -S . -B /workspace/build/motion_control_lab \
  -DMCL_BUILD_DUAL_ARM_REPLAY_VISUALIZATION=ON \
  -DCMAKE_PREFIX_PATH="/workspace/install/motion_control_core;/workspace/install/motion_control_viz"
cmake --build /workspace/build/motion_control_lab --target mcl_dual_arm_replay_ik -j8
```

Start a live, real-time replay:

```bash
/workspace/build/motion_control_lab/mcl_dual_arm_replay_ik \
  --urdf /workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf \
  --input /workspace/fixtures/raw/motion_control-psi_r1-20260801-101533_0.mcap \
  --left-stream /mc/ik/target/left_pose \
  --right-stream /mc/ik/target/right_pose \
  --timestamp-source header_stamp \
  --pairing-policy nearest \
  --nearest-tolerance-ms 5 \
  --unmatched-policy drop_with_diagnostics \
  --execution-mode realtime \
  --state-policy previous_solution \
  --servo-period-ms 10 \
  --visualize \
  --record-visualization-mcap
```

Connect Foxglove to `ws://127.0.0.1:8765`, configure the R1 URDF, and then press Space
in the runner terminal. The app publishes a complete five-channel initial frame while waiting; the replay
clock begins only after Space. `--visualize` implies this gate. Use
`--no-wait-for-space` for unattended visualization.

Published channels:

- `/mc/ik/joint_states`
- `/mc/ik/target/left_pose`
- `/mc/ik/target/right_pose`
- `/mc/fk/pose/left_end_effector`
- `/mc/fk/pose/right_end_effector`

The target topics preserve the MCAP TCP inputs. The FK topics are actual end-effector poses
computed from the joint positions in the same visualization sample; they are not converted
targets. See the Lab-wide
[Foxglove IK visualization contract](../../docs/foxglove_ik_visualization_contract.md).

When requested, `visualization.mcap` is stored beside `trace.csv`, `status.json`, and
`manifest.json` in the run directory. Completed run directories are append-only local
evidence; reviewed outputs can later be promoted deliberately into `results/`.
