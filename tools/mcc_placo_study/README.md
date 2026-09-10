# E06–E12 experiment evidence tools

These tools prepare and run independent experiment units, preserve native evidence and
perform independent per-unit checks. They do not execute Analysis or paper stages.

The frozen app interface is [IMPLEMENTATION_CONTRACT](../../docs/mcc_placo_study/IMPLEMENTATION_CONTRACT.md).
`study.py --help` documents selection, phase, dry-run, preflight, bounded smoke and output controls.
Installed apps default to `/workspace/install/algorithm/bin/`; `MCL_INSTALL_PREFIX` changes the prefix,
and `MCL_BINARY` overrides the actual executable. No implicit build or standalone fallback.

```
python3 tools/mcc_placo_study/study.py --dry-run --inventory-output /tmp/study-matrix.json
python3 tools/mcc_placo_study/study.py --phase confirmatory --preflight
python3 tools/mcc_placo_study/study.py --experiment E06 --case <case-id> --method <method-id> --repeat 0 --smoke --timeout 60
python3 tools/mcc_placo_study/check_inventory.py <run-dir>/inventory.json
python3 -m unittest discover -s tools/mcc_placo_study/tests -v
```

Each invocation creates an exclusive timestamp/hash run directory. All required selected identities
remain in inventory even when input, executable, capability or environment is unavailable. Every
unit runs in a separate process group; native stdout/stderr and partial files survive failure.
SIGINT stops the active unit and preserves the remaining not-run suffix. A timeout is a recorded
interruption, not a successful shortened measurement. A process lock serializes study invocations;
the formal operator must also stop external benchmarks/builds/tests.

The outer status records process execution; native solver status stays in raw/native files and
independent validation is separate. `validation_command` in a unit may invoke an experiment-owned
checker after execution, outside timing. All recursive output artifacts are hashed. Native baseline
adapters use unit.binary plus `{binary}` in command so the native executable and libraries are
fingerprinted in addition to the Python process.

`inputs.py` independently evaluates fixed-base URDF trees with NumPy, including origin RPY and
mimic relations. It does not use MCC, PlaCo or Pinocchio. Study joints follow explicit full order;
other joints are neutral zero. This checks numerical implementation independently, but cannot
prove the shared source URDF is physically correct. Exact geometry needs its own pinned mesh/pair
inventory; model-only FK does not establish collision safety.

`metrics.py` owns study-metrics.v1 scalar definitions. Per-sample metric_row.v2 records retain
input/model/config hashes, session/split/window/component, native attempt identity and source line.
Null means unavailable or not-applicable with a reason, never a substitute zero. Source inventories
include tracked and untracked file content hashes (excluding generated run trees), Git scope/head,
dirty status, actual dynamic libraries and CMake/build-command artifacts.

Formal preflight verifies an immutable campaign_freeze.v1 reference with exact definition hash,
input/model/candidate/preregistration/resource/exposure/split artifacts, numeric threshold sources,
and a host RT attestation for the current boot and available physical cores. This implementation
session executes development only. Freezing declarations does not authorize a campaign, access a
holdout, establish scientific correctness, or promote results.

Development acceptance (actual commands, full dry-run, original failures and bounded native smoke) is recorded in [DEVELOPMENT_ACCEPTANCE](../../docs/mcc_placo_study/DEVELOPMENT_ACCEPTANCE.md). A successful process exit is distinct from native full quality and independent validation. Analytic result records use the experiment oracle while generic FK remains unavailable. Empty/begin-only logs never supply successful result coverage. Bounded `--smoke` is development-only.

`relabel_metrics.py --definition <definition> --run <run-dir> --output <new-dir>` applies only declared metric roles into a separate immutable correction artifact. It does not recompute values or run an analysis. This was used for the development E06/E11/E12 primary-role correction; old metric files are retained. See the acceptance record for the authoritative corrected locators.

## Source control and input preparation

Commit app code, generators, tests, contracts, conversion recipes, experiment declarations and
protocol documentation. `inputs/generated/`, `inputs/superseded/`, local `inputs/inventory-*/`,
E12's generated `inputs/required_units.json`, archived `generator_snapshots/` and execution `runs/`
are ignored. Ignoring these paths does not delete or invalidate local evidence. A deliberately
promoted fixture or result needs a separate review and destination.

The committed `definition.json` files retain the complete declared matrix, including unavailable
arms. Their prepared input locators and hashes belong to the original workspace; cloning this
repository alone does not provide the model, canonical inputs, raw recordings or run evidence.
Prepare inputs before input-dependent contract tests or execution. From the Lab root in a fresh
checkout with the intended R1 model available:

```bash
python3 experiments/E06_solver_semantic_parity/generate.py --model /workspace/models/r1.cos.urdf
python3 experiments/E07_hqp_priority_and_redundancy/generate.py --model /workspace/models/r1.cos.urdf
python3 experiments/E08_constraints_scaling_and_degeneracy/generate.py --model /workspace/models/r1.cos.urdf
python3 experiments/E09_solver_cost_and_scalability/prepare.py --model /workspace/models/r1.cos.urdf
python3 experiments/E10_multirate_scheduling_and_coupling/prepare.py --model /workspace/models/r1.cos.urdf
python3 experiments/E11_planning_ik_otg_error_propagation/prepare.py --model /workspace/models/r1.cos.urdf
python3 experiments/E12_recorded_motion_holdout/data.py inventory --locator /mnt/mcap_dataset --output experiments/E12_recorded_motion_holdout/inputs/inventory-development-20260910
python3 experiments/E12_recorded_motion_holdout/prepare.py --model /workspace/models/r1.cos.urdf --inventory experiments/E12_recorded_motion_holdout/inputs/inventory-development-20260910/dataset_inventory.json
python3 tools/mcc_placo_study/study.py --dry-run --inventory-output runs/mcc_placo_study/matrix-development.json
```

These commands create local inputs and update declarations with resolved locators and hashes;
they do not reproduce an old evidence run or grant holdout access. Review the resulting matrix
and dataset identities before preparing a new campaign. Do not regenerate inputs or declarations
used by an active or frozen run; use a separate checkout for fresh preparation. The current
`campaign.py` recipe expects the named development inventory and must not silently substitute
a different dataset or session split.

## Full non-RT development campaign

`campaign.py prepare --output <new-directory>` snapshots the E06–E12 development matrix,
installed runtimes, input inventory, session split and conversion recipe. The campaign keeps
declared development repetitions (E09: three; others: one), uses full windows without `--smoke`,
and extends E10 to ten seconds to include its declared injection at five seconds. This duration
change is recorded in the campaign snapshot; solver settings and production-static remain frozen.

`campaign.py run --plan <new-directory>/plan.json` acquires the study execution lock, verifies
the snapshots and runtime hashes, attempts each E12 conversion, then runs E06 through E12 serially.
Valid historical/controlled E12 inputs use their full source interval plus declared settling.
Confirmatory arms stay unavailable; holdout access, candidate search, formal RT evaluation,
cross-run research analysis and paper work are excluded. Native failures and invalid inputs are
retained and do not stop subsequent units. A plan can start only once; it is not a retry command.

Progress is the number of terminally classified conversions and units, plus completed inventory
checks. Failed/unavailable units count as processed. The fixed total is not a success rate or time
estimate. `progress.json` contains local detail; only lines of the form
`PROGRESS: 123/7360 stage=E09` are intended for RunBuoy regex progress. Raw logs are local.
`campaign_status.json`, `conversion_inventory.json`, `checks/`, `admission/` and per-experiment
`execution/*/*/inventory.json` retain status, actual commands, validation and hashed raw evidence.
The complete before/after-conversion matrices are stored as preflight JSON. All required missing
arms remain in those inventories. Any required failure/unavailability makes the campaign exit
nonzero even after all seven experiment waves have been processed.

Each native unit has a one-hour process timeout and each conversion a ten-minute timeout.
A 16 GiB free-space reserve prevents starting further units when storage is low; existing
evidence is never deleted. Space-guard omissions remain unavailable with a reason. Cancellation
retains partial output. Infrastructure exceptions are recorded separately and never relabelled
as successful experiment evidence.
