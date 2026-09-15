# Native HARP, phase 1

HARP is the native left-arm posture-reference source. Its model and reusable
`Predictor` live in MCC `posture_reference`; this app owns scheduling, active
time, reference sampling, recording, robot geometry and HQP task construction.
The worker evaluates full 30x9 FP32 windows with LibTorch/AOTInductor. It does
not start or embed a Python model process.

Build/install MCC with its optional component and staged runtime first, then:

```bash
colcon build --base-paths labs/motion-control-lab \
  --build-base build/algorithm --install-base install/algorithm --merge-install \
  --cmake-args -DMCL_BUILD_HARP=ON
```

The native options are `--elbow-reference harp`, `--harp-model DIRECTORY` and
`--harp-device cuda|cpu` (default CUDA). The app rejects unknown phase-1 frames
and non-100-Hz sampling. `planned/run_harp_{csv,mcap,keyboard}.py` provide the
existing planned + pose-primary recipes, fixed waist, and ordinary right-arm
tasks. Explicit launcher arguments retain highest priority. Model directory is
required; there is no implicit setup or download at launch.

To generate the small reachable input used in acceptance, first invoke the CSV
recipe with `--dump-resolved-options` and save its JSON. Then use
`scripts/harp/make_reachable_motion.py --resolved-options OPTIONS.json
--output reachable.csv --duration-s 30`. It uses the resolved R1 model/default
configuration and emits 100 Hz bilateral TCP poses from a small joint motion.

```bash
MCL_RT_PRIORITY='' python scripts/profiles/planned/run_harp_csv.py \
  --harp-model /workspace/runs/harp_phase1/models/cuda \
  --input /path/to/reachable.csv \
  --elbow-record /path/to/harp-run/consumed.jsonl \
  --output-root /path/to/harp-run/app --replay-trace
```

The empty RT prefix above reproduces non-RT acceptance. Native Red/Yellow/UI
affinities remain the recipe defaults. The inference worker explicitly uses
SCHED_OTHER and is warmed on its own thread before periodic workers start.
Control does not call Torch or wait for inference. Sampling occurs at 10 ms
boundaries in accepted-control active time; inference takes the latest full
window and may skip intermediate requests. Each window still contains 30
uniformly sampled frames. Startup fills the history with initial EE pose.

Results carry generation, sequence, active sample time and result-ready latency.
The app checks generation and the existing 100 ms age bound. Pause freezes
active time. A restarted run constructs a new ElbowReference, buffers and
Predictor after joining the previous worker; model state and in-flight results
do not cross runs. The class is initialized once per run. Worker failures retain
the original error for the control exception path. Inference errors, stale
results and invalid shoulder-wrist geometry stop the run, with no source switch.

The control cycle recomputes the shoulder from current state and converts the
predicted arm angle using the current accepted planner EE reference. It adds the
existing left link4 Secondary task. Primary pose tasks, right-arm reference,
hard bounds, solver acceptance and preservation tolerances retain their existing
semantics. This is a posture reference, not joint-command authority.

`--elbow-record P` produces `P` (actual consumption), `P.windows.jsonl` (full
FP32 input for each evaluated window plus prediction/identity/latency), and
`P.model.json` (model manifest), and `P.runtime.json` (SHA256 of actual loaded shared libraries after warmup). Sequence zero also records startup warmups;
exclude it from steady-state timing statistics. Enable replay trace to join
window last frames to `reference_left_pose`, not `left_goal`: the latter is the
incoming goal before Cartesian planning. The existing trace records selected
priority, task preservation and constraint evidence. HARP visualization uses
`/mcl/harp/left/{angle,scene}` and HARP schema/marker names.

```bash
python scripts/harp/verify_online.py --consumed /path/to/consumed.jsonl \
  --run /path/to/app/run-id --output /path/to/verification.json
```

The verifier preserves native success/failure status, checks feature order,
startup filling, history overlap across skipped requests, actual consumed
predictions and age, and reports Secondary-selected versus Primary-only counts.
`scripts/harp/record_reference_fixture.py` records a successful run's accepted
references as a fixed derived 30-second MCAP fixture, with source hashes and an
explicit synthetic epoch. It is not a hardware recording; original MCAP failures
remain separate evidence. Neither tool changes control parameters or retimes
existing recordings.

Optional lifecycle acceptance:

```bash
cmake -S /workspace/labs/motion-control-lab -B /workspace/build/algorithm/motion_control_lab \
  -DMCL_BUILD_HARP=ON -DMCL_HARP_TEST_MODEL=/workspace/runs/harp_phase1/models/cuda
cmake --build /workspace/build/algorithm/motion_control_lab \
  --target test_hierarchical_kinematics_step_harp_lifecycle
ctest --test-dir /workspace/build/algorithm/motion_control_lab -R harp_lifecycle --output-on-failure
```

Phase 1 does not validate learned posture quality, guarantee 1 kHz deadlines,
train a new model, or support waist-aware/bilateral model inputs. Original
recordings with waist motion can become unreachable when replayed with a fixed
waist; that is an explicit failure, not evidence for relaxing hard bounds.

The retired `pim-ik` online source, Python model/server and Unix socket manager have been removed.
Old `--pim-*` and `--elbow-socket` options fail with an explicit migration message; select HARP with an explicit installed MCC model.
Legacy consumption records remain readable through `recorded`. Historical PiM prose is in the [archive](../../docs/archive/pim/PIM_IK_EVALUATION.md); obsolete inspection tools are retained only in the cleanup source snapshot.
Model export/packaging/reference inference are owned by MCC; MCL does not keep a second model implementation.

## Visualization and recorded replay

Live HARP keeps `/mcl/harp/left/{angle,scene}`, `mcl.harp.Angle`, and the existing
`harp_*` marker IDs. Use `foxglove/harp.layout.json`.
Recorded replay publishes only `/mcl/elbow_reference/left/{angle,scene}` with
`mcl.elbow_reference.Angle` and `elbow_reference_*` marker IDs; use
`foxglove/recorded_elbow_reference.layout.json`. Its source is always `recorded`:
existing consumption rows do not reliably identify the originating model.
No PiM topic is republished or duplicated in a new run. Old MCAP layouts and
inspection tools are no longer installed. Their source is preserved in the
[cleanup snapshot](../../docs/archive/README.md). This does not change recorded values,
time, generation, pause, exhaustion or failure handling.
