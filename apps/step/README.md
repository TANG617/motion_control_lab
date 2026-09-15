# step public execution

`mcl_step` retains its existing `teleop` and CSV/MCAP `replay` entry and exposes finite JSON batch and
snapshot execution. Both entries construct the existing `MccServoSolver` or
`PlacoServoSolver` and call the same `solve` implementation. There is no study solver,
experiment-number branch, hidden acceptance override or fallback.

```bash
BIN=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_step
"$BIN" --describe-capabilities
"$BIN" batch --input /absolute/canonical.json --config /absolute/options.json \
  --execution /absolute/windows.json --output /absolute/new/output
"$BIN" --request /absolute/request.json --dump-resolved-options
"$BIN" --request /absolute/request.json
```

Request schema is `execution_request.v1`, app ID `mcl_step`, execution structure
`servo_step`, JSON input. The request mode rejects extra algorithm CLI flags.
`--describe-capabilities` performs no model/backend initialization. Both normal
batch and request support read-only `--dump-resolved-options`; the existing
interactive parser also supports it before model initialization.

`app_config` keys:

- `urdf_path`: explicitly selected model, bound to input `model.locator` and SHA256.
- `solver`: `mcc` (default) or `placo`; `backend`: `proxqp` (MCC default) or
  `eiquadprog` (required for PlaCo).
- `rate_hz`: default 100; sets the native ServoStep period.
- `task_layout`: only `dual-arm-hard-pose`; `algorithm`: fields from this app's
  `AlgorithmOptions` with its unchanged compiled defaults, serialized fully.
- `target_space`: `frame` (default) or `tcp`. TCP input converts through the actual
  fixed R1 TCP transform, exactly as normal step replay; raw targets are the
  resulting solver frame targets. Input frames/root/offsets must agree with the app.

The word weighted identifies the single-level solver topology, not soft task
semantics: these apps register hard left/right pose tasks. Their task enforcement,
MCC/PlaCo gains, native acceptance and TargetSolve policies differ from old study
methods. Renaming an old study method does not establish equivalence or admission.
No task-weight/hierarchy/warm-start setting is invented when the real app does not
provide it.

Canonical JSON input contains `model:{locator,sha256}`, ordered `joint_names`,
optional `initial_state:{q,v}`, and nonempty `samples`. A sample may contain `q,v`,
`source_time_s`, `targets:{left,right}` with `position:[x,y,z]` and a 3x3 `rotation`.
An omitted target explicitly means hold that arm's current frame pose, useful for
standalone numerical debugging. Joint order and vector dimensions must match R1.
Optional initial state defaults to the app's normal posture, with zero velocity.

Execution settings are `cold_calls` (1), `warmup_calls` (0), `measured_calls`
(sample count minus cold), `state_mode=snapshot|evolving` (evolving). The complete
sequence repeats input samples cyclically; each call retains its sample index.
Snapshot injects the declared sample state (or declared initial state) before each
call, without reinitializing backend workspaces. Evolving mode follows exactly the
normal app's committed state. Call windows count actual app IK invocations, not
process startup; TargetSolve native inner calls additionally appear individually.

Observation `mode` is `minimal|full|cpp-new` (full). All modes keep every attempt,
native return status, call duration, returned candidate q/v (including rejected
returns), final commit and full resulting q/v. Rejected returned state is explicitly
labelled and does not imply a selected optimization candidate. Full enables MCC native linearization capture and stores actual task matrices,
bounds, enforcement and candidate vectors (PlaCo stores its native task rows); cpp-new additionally measures thread-local C++ allocation
count/bytes only during the native call. It does not count malloc, allocations on
other threads or total process memory. The compiled allocator hook is present in
all modes; only cpp-new enables counting. Observation/JSON writing is outside
`native_call_ms`. `app_solve_with_observer_ms` includes observer cost and FK/error
processing; initialization is separately recorded. These fields must not be
interchanged in timing analysis.

Each new output contains source binding, exact request and resolved options,
capabilities, model hash/mapping with actual configured bounds, `call_plan.json`,
`raw.jsonl` and (on normal completion) `summary.json`. A flushed `attempt_begin`
precedes native execution. MCC rejection is recorded before the original error
propagates; PlaCo exceptions propagate without fabricated native status. The call
plan's unobserved suffix is `not-run`; a missing return is not an accepted call.
TargetSolve's existing returned-infeasible behavior remains a rejected call, and
its batch exits 1 if any call was rejected. No rejected candidate is committed.

Generated evidence and old experimental results are not overwritten. New app
runs require fresh task-equation and acceptance audits before scientific use.

```bash
python apps/step/tests/public_execution.py "$BIN" /absolute/canonical.json \
  --output /absolute/new/test-evidence
```

This bounded check covers both MCC backends and PlaCo, all observation modes,
normal/request deterministic equality, call-window denominators, input hashes,
unknown configuration, no-overwrite, and native unreachable-target failure. It
retains the command, stdout/stderr and exit status of every invocation.
