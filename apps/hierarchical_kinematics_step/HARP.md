# Dual-arm Transformer posture reference

MCL uses MCC's native `posture_reference::Predictor` and its `model.pt2` package.
There is no Python model process, socket inference, or Mamba compatibility.
Python launchers only construct the C++ command; offline calibration and report
scripts do not run a network.

## Profiles and tasks

| Profile | Left link4 | Right link4 |
|---|---|---|
| `posture-reference-task` | HARP | HARP |
| `posture-reference-task-left` | HARP | disabled |
| `posture-reference-task-right` | disabled | HARP |

All three use CartesianTrajectoryGenerator -> HKS (`planned`, `pose-primary`, no joint OTG).
Both end-effector position and orientation tasks are Primary, with a shared
per-arm progress scale. Enabled link4 position tasks and the existing Yellow
posture coupling are Secondary. Yellow posture/collision configuration remains
identical in all three modes, including its coupling on the enabled arms.
The default non-arm joints are inactive; Red is 1000 Hz, Yellow 100 Hz.
A disabled model task leaves the normal Yellow posture preference in control.
The model always receives both wrists and predicts both arm angles.

These are app-local profiles of the existing executable. Solver bounds,
acceptance policy and preservation tolerances are unchanged. Native solver
failures and Primary-only selections are retained in the evidence.

## Input and calibration

The input is `[1,30,18]` FP32 at 100 Hz, oldest first. Each frame is left followed
by right, each `px,py,pz,R00,R01,R10,R11,R20,R21`. The rotation columns are
**row-interleaved**. Coordinates are meters; normalization is inside the model.
The synchronous pair comes from the accepted control step's planner EE reference,
not the incoming unplanned goal or the executed robot FK.

The adapter uses the `body_link4` origin and `diag(-1,-1,+1)` relative axes.
Incoming TCP targets first use the existing TCP offset inverse to obtain EE
poses. The wrist center is `(0,0,-0.097)` m in the EE frame. Wrist orientation is
converted with a fixed left/right rotation stored in `RobotOptions`:

```
reference_from_base = reference_from_torso_axes * inverse(base_from_torso)
p_wrist = reference_from_base * base_from_EE * wrist_in_EE
R_wrist = R_reference_from_base * R_base_from_EE * C_side
C_side = transpose(R_robot_EE_zero) * R_human_wrist_zero
```

`config/r1_wrist_calibration.json` records the numeric matrices and source hashes.
They come from URDF zero pose and SMPL-H neutral, zero-beta rest joints using the
same torso-basis construction as preprocessing. This is a declared fixed
zero-pose correspondence, not an empirical hand-sensor calibration. It is never
re-estimated from a run's initial pose. Reproduce it offline using
`scripts/harp/calibrate_wrist_axes.py --urdf URDF --smplh smplh.tar.xz --output NEW_JSON`.
No SMPL-H files are needed at runtime. Model manifests must use the current R1
reference morphology and unmodified SMPL-H wrist-axis convention; initial robot
FK geometry is compared to the manifest shoulders and lengths.

Enabled elbow targets are reconstructed in the reference torso coordinates:
`n=normalize(W-S)`, `u=normalize(project([-1,0,0], perpendicular_to=n))`,
`v=n cross u`, `E=center+radius*(cos(phi)*u+sin(phi)*v)`; then transformed to the
solver base frame. There is no alternate-axis fallback. Invalid/reach-degenerate
circles, radius below 1 mm or axis projection below 1e-3 fail the run.

## Scheduling and failure behavior

One ordinary SCHED_OTHER C++ worker owns the predictor and warms it before
periodic workers start. Red publishes the complete dual-arm window to a latest
value mailbox and never waits for inference. A skipped request does not remove
frames from the next window. Startup repeats the initial dual wrist pair 30 times.
Both predictions share generation, sequence, active sample time and latency.
Sampling follows accepted-control active time at 10 ms boundaries. Pause freezes
that time; restart creates a new worker/history. Results older than 100 ms,
model failures and worker failures propagate to the existing failure path.
No source switching, filtering, synthetic angle or task-weight adjustment occurs.

## Build and launch

Build/install the current MCC with `MOTION_CONTROL_CORE_BUILD_POSTURE_REFERENCE=ON`
and its staged native runtime, then build MCL with `MCL_BUILD_HARP=ON`. The predictor
uses SONAME 1; an old single-arm SDK cannot compile this client. Normal launchers
use `/workspace/install/algorithm/bin/mcl_hierarchical_kinematics_step`.
`MCL_BINARY` explicitly selects an isolated test executable. Launchers never build.

From this app directory, for the package delivered on 2026-09-17:

```bash
scripts/profiles/posture_reference_task/run_keyboard.py \
  --harp-model /workspace/components/motion_control_core/python/posture_reference/runs/dual_arm_transformer_deploy_20260917_025645/packages/cuda

scripts/profiles/posture_reference_task_left/run_mcap_interactive.py \
  --harp-model /workspace/components/motion_control_core/python/posture_reference/runs/dual_arm_transformer_deploy_20260917_025645/packages/cuda

scripts/profiles/posture_reference_task_right/run_csv_realtime.py \
  --harp-model /workspace/components/motion_control_core/python/posture_reference/runs/dual_arm_transformer_deploy_20260917_025645/packages/cpu \
  --harp-device cpu --input /path/to/input.csv \
  --elbow-record /workspace/runs/mcl_posture_reference_example/consumed.jsonl \
  --output-dir /workspace/runs/mcl_posture_reference_example/app --replay-trace
```

Each profile also provides `run_mcap_headless.py`. Native CLI exposes the same
profile names; request uses `app_config.profile` and ordinary `app_config.options`.
`--describe-capabilities` and `--dump-resolved-options` do not load the model.
CUDA is the default; CPU requires an explicit CPU package. The model path is
always explicit. Use fresh result directories. Online CSV/MCAP uses realtime
playback, because asynchronous inference must have physical time to complete.

## Recording, replay and Foxglove

`--elbow-record P` writes `dual-arm-elbow-consumption.v1` rows to P, evaluated
540-float windows to `P.windows.jsonl`, model provenance to `P.model.json`, and
loaded runtime paths/hashes to `P.runtime.json`. `P.calibration.json` saves the
fixed adapter matrices, reference frame and calibration source, including for
keyboard runs without a replay manifest. Rows carry both angles, enabled
sides, accepted EE references, torso transform, consumed elbow targets, raw and
executed elbow positions, TCP errors and native hierarchy diagnostics.
Sequence zero contains startup inference and is excluded from steady timing.

Replay with the same profile and input, changing the source to
`--elbow-reference recorded --elbow-recorded P`. New records require their model
sidecar and matching task sides. Recorded replay uses the C++ reader, including
when HARP support is disabled. Old single-arm/PiM records are explicitly rejected;
historical files and [archived evaluations](../../docs/archive/pim/PIM_IK_EVALUATION.md)
are preserved and are not relabeled as Transformer results.

Import `foxglove/harp.layout.json` for live HARP or
`foxglove/recorded_elbow_reference.layout.json` for recorded replay. Left/right
`/mcl/harp/{left,right}/{angle,scene}` topics show the prediction, participation,
elbow circle, reference and raw/executed targets. A disabled side still publishes
its predicted angle with `task_enabled=false`; it does not draw an active target.
Recorded channels use `/mcl/elbow_reference/...` and source `recorded`.
TCP mirroring mirrors TCP input only; it never mirrors the predicted arm angle.
In the reference profiles, manual elbow editing is disabled so that each side keeps its selected model/Yellow ownership.

## Verification

`scripts/harp/verify_online.py --consumed P --run APP_RUN --output REPORT.json`
checks all 30 frames against independent torso/EE/wrist conversion, window
overlap, synchronized consumption and task masks. Optional `--cpp-executable`,
`--model`, `--device` compare the exact windows with the independent MCC native
consumer (maximum absolute output difference <= 1e-5). It reports timing,
tracking, elbow errors, angle increments and native failures without inference
in Python. Three-profile comparisons must use identical inputs and initial
states; geometric/model validity does not certify hardware control performance.
