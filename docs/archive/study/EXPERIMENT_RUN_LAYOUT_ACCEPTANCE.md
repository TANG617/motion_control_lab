# Experiment-owned runs and fresh 16-worker campaign — 2026-09-11

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

The [project mapping](PROJECT_MAPPING.md) now follows the architecture's recommended ownership:
new experiment outputs physically live in `experiments/Exx_*/runs/<run-id>/`. Campaign directories
hold frozen plans, input preparation, progress, shared admission metadata and an explicit
`execution_index.json` pointing to each actual experiment run. No inventory is selected by an
ambiguous glob over old sibling runs.

The new full development campaign is
[`nonrt_fresh16_20260911T080119Z`](../../../runs/mcc_placo_study/nonrt_fresh16_20260911T080119Z).
It starts from zero, with no old unit reuse, old admission reuse, conversion reuse or mixed-runtime
waiver. All 50 conversion jobs and all 7,303 declared experiment units are accounted for again.
Installed executable/library and tool hashes are frozen afresh. Runtime drift remains blocked.

The full matrix dry-run retains E06=21, E07=565, E08=293, E09=5,400, E10=193, E11=372 and E12=459
units. Before fresh input conversion, 2,836 are ready and 4,467 unavailable. E12 conversion results
and the complete post-conversion matrix will be recorded by the campaign. These counts describe
input/executable availability, not scientific success or formal readiness.

## Validation

- 131 tests passed: 58 public orchestration/contract tests and 73 experiment tests. The final
  six-test layout check additionally confirms generated run scripts are excluded from frozen
  orchestration source inventories.
- Process fixtures demonstrate 16 simultaneous checkers, a bounded queue of 32 acquired units,
  dynamic replacement after completion, a 33rd-unit tail batch, and no native/checker overlap.
- Cancellation with 16 active and 16 queued checks preserves all 32 completed native units for
  checker-only continuation. Existing evidence bytes remain unchanged.
- A bounded raw-only smoke rechecked 16 complete E08 units (96,000 attempts). All 404 compared
  derived files matched byte-for-byte after the raw-path mapping in `metrics.jsonl`. No native
  solver was launched, no samples were dropped, and no numerical tolerance changed. All original
  source file hashes remained unchanged. Peak concurrency was 16; batch wall time was 232.159 s.
- The smoke verifies independent recomputation of historical raw records, not the historical
  runtime identity or a scientific performance claim. No analysis, figures or paper work ran.

Exact commands, logs, artifact hashes and full comparisons are retained under
[`layout_and_parallel16_upgrade/20260911T075055Z`](../../../runs/mcc_placo_study/layout_and_parallel16_upgrade/20260911T075055Z),
including `checks/`, `real16-smoke/` and the recovery records.

## Historical evidence and recovery

RunBuoy run `01a08f58-6c19-7105-bd99-06ea8d20073a` was cancelled through the public CLI and confirmed
stopped at 07:51:01 UTC. Its original output directories remain untouched. The recovery checkpoint
contains 723 records: 715 terminal reuse records and eight native-complete units whose verification
was interrupted. The fresh campaign does not consume this checkpoint.

Each Exx `runs` directory has explicit `legacy-*` links and a versioned `legacy-index-*.json`
pointing to historical run directories. This preserves frozen absolute paths and hashes, including
partial evidence. `recovery-source-relocations.json` retains the exact old tool bytes. To restore
an old checkpoint later, stop the new campaign and prepare an explicit continuation of the chosen
old plan; the continuation reads legacy locations and writes its new runs inside Exx.

## Commands

```bash
python3 tools/mcc_placo_study/campaign.py prepare \
  --output runs/mcc_placo_study/nonrt_fresh16_20260911T080119Z \
  --verification-workers 16 --verification-batch-size 32
python3 tools/mcc_placo_study/campaign.py run \
  --plan runs/mcc_placo_study/nonrt_fresh16_20260911T080119Z/plan.json
```

The prepare command above has already run and intentionally refuses to overwrite its directory.
`full-matrix-dry-run-command.json` stores all seven exact definition arguments. The RunBuoy launch
argv, safe regex preview and detached handoff are stored beside the frozen plan. Do not invoke
the run command again while the managed campaign is active.

Native acquisition is serial; verification uses 16 single-thread workers over each 32-unit batch.
E09/E10 acquisition has no concurrent verifier. This remains a non-RT development campaign, with
formal experiments, analysis and paper work deferred. No devices, kernel changes, builds,
commits, pushes or publications were performed for this change.
