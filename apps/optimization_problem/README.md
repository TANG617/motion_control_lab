# Explicit matrix optimization

`mcl_optimization_problem` executes a finite matrix QP or lexicographic HQP directly
through MCC's public optimization API. It has no robot model, control period,
planner, hardware transport or experiment-number dispatch. Existing robot apps
cannot express this topology without inventing a robot loop. The app is reusable
for solver debugging, rank-deficient numerical examples, equation audits and
priority-preservation checks.

```bash
BIN=${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_optimization_problem
"$BIN" --describe-capabilities
"$BIN" --input apps/optimization_problem/fixtures/two_level.json \
  --config apps/optimization_problem/fixtures/hqp.json --output /absolute/new/output
"$BIN" --request /absolute/request.json --dump-resolved-options
"$BIN" --request /absolute/request.json
```

Normal and request entry call the same `solve(problem, options)`. Output directories
must not exist. Request schema is `execution_request.v1`, app ID
`mcl_optimization_problem`, structure `explicit_matrix`, JSON input; execution and
observation objects are empty. There is no process-loop timing benchmark here.
`--describe-capabilities` constructs no backend. Resolved-options mode creates no
output or solver.

Input has `lower` and `upper` variable bounds and `tasks`. Each task contains a
rectangular `A`, target vector `b`, optional name/unit/weight/enabled/hard and
integer `priority`. The equation is `A*x = b`. MCC weighted combines enabled soft
squared residuals; MCC HQP registers levels in ascending priority, defaulting to
one successive level per task. `hard:true` registers a hard equation, not a very
large weight. Bounds remain hard. Preservation tolerance is explicit and defaults
to `1e-7`; regularization defaults to `1e-8`. These defaults are this functional
app's configuration, not inherited admission for any historical study.

Options: `solver=mcc|placo`, `mode=weighted|hqp`,
`backend=eiquadprog|proxqp`, `regularization`, `preservation_tolerance`.
PlaCo supports weighted eiquadprog only; unsupported combinations fail explicitly.
No emulated PlaCo hierarchy or backend exists. `analytic` wrapper input remains
readable as an explicit payload shape; it does not select algorithm behavior.

Output keeps `execution_request.json`, `binding.json`, `resolved_options.json`,
`capabilities.json`, `attempt_begin.json` and `result.json`. Results record native
status, full candidate, native-call wall time and all available hierarchy pass
iterates/statuses. A rejected result is written and the process exits 1. An API
exception propagates with original stderr and leaves attempt evidence; no candidate
or continuation is fabricated. Pure optimization never commits robot state.

Independent small-array tests check residuals/objectives rather than arbitrary
multi-solution vector equality. A separate deterministic normal/request comparison
checks the same configured execution path:

```bash
python apps/optimization_problem/tests/public_execution.py "$BIN" \
  --output /absolute/new/test-evidence
```

Tests cover both MCC backends, multiple minimizers, rank deficiency, two and three
levels, hard infeasibility, read-only dump, no-overwrite and override rejection.
Each subprocess's command, stdout, stderr and status is retained when `--output`
is supplied. This app does not establish cross-backend scientific superiority.
