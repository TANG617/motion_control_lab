# Parallel verification development acceptance — 2026-09-11

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

The development orchestrator supports `--verification-workers 8`. Native units remain serial:
acquire at most eight units, verify that batch in separate processes, then acquire the next batch.
No native acquisition overlaps a verifier, including E09/E10. Only checker processes receive
single-thread BLAS/OpenMP settings. Native affinity, solver settings, tolerances, full windows,
production-static and required matrix identities are unchanged by this patch.

The user explicitly authorized continuing this development campaign despite the installed runtime
change: “直接继续跑吧，暂时豁免换版本继续跑”. The continuation freezes current runtime hashes,
preserves old results and records this one-time transition; manifests are marked as containing
mixed native runtime versions with formal comparability not established. Future drift remains blocked.
No kernel, device, native build, commit, push, formal experiment or holdout operation was performed.
Analysis and paper work remain deferred.

## Implementation and checks

- `verification_pool.py` and `verification_worker.py` provide bounded queues/processes,
  checker-only thread settings, durable pending/result records and process-group cleanup.
- `study.py` records the scheduling policy and native/checker monotonic intervals. Cancellation
  preserves completed native work for checker-only continuation.
- `campaign.py` aggregates out-of-order completions centrally. Local progress lists each active
  checker's sample count; RunBuoy lines contain only counts and fixed phase labels. Percentages
  represent terminal classifications, including failures/unavailability, not time or success rate.
- `continue_campaign.py` preserves scientific failures and bounded timeouts, rechecks interrupted
  verification only, and validates the original evidence chain across multiple reuse wrappers.
- Native runtime files are hashed at startup. Before each native unit their file signatures are
  checked, with rehashing on change; drift blocks subsequent acquisition and is recorded. This
  detects normal external reinstalls; it is not atomic executable/library isolation and does not
  eliminate the check-to-exec race or malicious same-signature mutation.

The acceptance evidence directory is
[`parallel_verification_upgrade/20260911T065345Z`](../../../runs/mcc_placo_study/parallel_verification_upgrade/20260911T065345Z).
It retains initial failed test attempts, corrected regression results, exact argv/cwd records,
old frozen tool bytes, checkpoint indices and runtime mismatch evidence. The initial checkpoint
index exposed a multi-generation wrapper bug; `checkpoint-index-final.json` is its corrected
replacement, without changing the original evidence.

Relevant commands, run from the Lab root:

All 120 tests passed after the authorized runtime-transition extension: 47 public orchestration/contract
tests, 34 E06–E08 tests, six E09 tests, 12 E10 tests, four E11 tests and 17 E12 tests.
The initial use of unittest discovery for E06–E08
found no pytest functions; the corrected pytest invocation and its 34-test result are retained.

```bash
python3 -m unittest discover -s tools/mcc_placo_study/tests -p 'test_*.py' -v
python3 -m pytest experiments/E06_solver_semantic_parity/tests experiments/E07_hqp_priority_and_redundancy/tests experiments/E08_constraints_scaling_and_degeneracy/tests -q
python3 -m unittest discover -s experiments/E09_solver_cost_and_scalability -p 'test_*.py' -v
python3 -m unittest discover -s experiments/E10_multirate_scheduling_and_coupling -p 'test_*.py' -v
python3 -m unittest discover -s experiments/E11_planning_ik_otg_error_propagation/tests -p 'test_*.py' -v
python3 -m unittest discover -s experiments/E12_recorded_motion_holdout/tests -p 'test_*.py' -v
```

The complete frozen development matrix was dry-run with eight verification workers; exact seven
`--definition` arguments are in `checks/full-matrix-command.json`. These are declaration/input
availability counts, **not runtime-freeze readiness**:

| Experiment | Required | Ready in dry-run | Unavailable |
|---|---:|---:|---:|
| E06 | 21 | 20 | 1 |
| E07 | 565 | 565 | 0 |
| E08 | 293 | 258 | 35 |
| E09 | 5,400 | 1,431 | 3,969 |
| E10 | 193 | 185 | 8 |
| E11 | 372 | 372 | 0 |
| E12 | 459 | 15 | 444 |
| Total | 7,303 | 2,846 | 4,457 |

## Real raw-evidence smoke

`real-smoke/validate.py` rechecked four complete E07 and four complete E08 units: 40,000 attempts,
80,000 raw records. All 202 compared derived files matched byte-for-byte after mapping the copied
raw path in `metrics.jsonl`. Original source hashes remained unchanged. No native app was launched,
no samples/checks were omitted and no tolerance was relaxed. The result is in `real-smoke/result.json`.

Eight workers were simultaneously active. Elapsed batch time was 142.201 seconds. The previous
individual app-checker durations summed to 734.843 seconds (about 5.17 times this batch duration).
This is an operational development comparison, not a controlled performance experiment or an ETA.

## Runtime transition and preserved checkpoint

RunBuoy run `01a08e5b-cbe4-759c-adec-a873a9864112` was cancelled through its public CLI and confirmed
`CANCELLED` at 06:53:51 UTC. There are 627 native/terminal records in the preserved checkpoint:
626 terminal reuse records and one completed native unit whose checker was interrupted. The
cancelled 252-unit E08 not-run suffix is not mistaken for completed experiments.

The one interrupted checker was then completed in `validation-only-smoke/`: 6,000 attempts,
113.982 seconds, zero native launches, unchanged raw/native marker bytes and a passing 74-artifact
inventory. Its validated record can replace only that checkpoint entry, giving 627 terminal reuse
records. The old interrupted output remains preserved.

Eight installed artifacts, including `mcl_baseline` and `libmotion_control_core.so.0.4.0`, differ
from the frozen runtime inventory. Their observed replacement times are around 06:02 UTC;
`runtime-drift.json` contains expected/current hashes, and `runtime-drift-timeline.json` contains
the timestamp evidence. Searches of install/build/temporary copies and deleted process mappings
found no matching original artifacts. No current install file was overwritten.

Twenty-three native completion markers fall at/after the earliest observed replacement. These
records are conservatively flagged for runtime-provenance review and retained. The original
per-binary environment cache cannot establish which library bytes each later process loaded.
The E08 raw files used by the equivalence smoke are in this interval; checker equivalence does
not certify their original native runtime identity.

`checks/continuation-prepare-command.json` records the actual continuation preparation attempt.
It initially failed on the frozen native hash check before creating a new campaign directory.
After inspecting that concrete failure, the user authorized a one-time runtime transition without
repeating old native units. The new `runtime-transition.json` retains both inventories, exact
hash differences and the authorization. The manifests and inventories preserve original per-unit
run identities and explicitly label mixed runtime versions; they do not assert an unchanged
experimental condition.

The finalized continuation is
[`nonrt_parallel_continuation_20260911T071817Z`](../../../runs/mcc_placo_study/nonrt_parallel_continuation_20260911T071817Z).
Its `launch-readiness.json` verifies current native/tool hashes and records 627 terminal reuse
units, zero pending checkers and 2,220 remaining preflight-ready native units. The full 7,303-unit
matrix and all 50 input conversion classifications are retained. `runbuoy-launch-command.json`
stores exact local argv/cwd, `runbuoy-dry-run.json` stores the successful launch preview, and
`runbuoy-handoff.json` records the detached launch response when started. Progress uses an
explicit batch counter and active-worker count; raw logs and detailed identities remain local.

The authorized continuation uses a fresh output directory:

```bash
python3 tools/mcc_placo_study/continue_campaign.py --parent-plan runs/mcc_placo_study/nonrt_continuation_20260911T024315Z/plan.json --output <new-directory> --verification-workers 8 --verification-evidence <accepted-upgrade-evidence.json> --accept-runtime-transition
python3 tools/mcc_placo_study/campaign.py run --plan <new-directory>/plan.json
```

Without the explicit transition flag, changed runtime hashes still reject continuation. Formal
readiness additionally requires the existing RT attestation and complete protocol/input/config
freeze; this development change does not provide either or authorize formal execution.
