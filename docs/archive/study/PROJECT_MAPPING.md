# E06–E12 implementation mapping

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

This maps [the experiment architecture](../../experiment_architecture.md) to the study implementation.
Experiment evidence belongs to the experiment. Campaign directories contain orchestration state
and explicit references to experiment runs.

| Role | Current implementation |
|---|---|
| Declaration / canonical input | `experiments/Exx_*/definition.json`, immutable referenced input descriptors and canonical files |
| Execution / unit verification | `tools/mcc_placo_study/study.py`, experiment-local native app and `verify.py` |
| Experiment artifact root | `experiments/Exx_*/runs/<run-id>/` |
| Run contents | `Exx.manifest.json`, `inventory.json`, `units/<Exx>/<case>/<arm>/<repeat>/`, source/build and scheduling inventories |
| Campaign workspace | `runs/mcc_placo_study/<campaign-id>/` |
| Campaign-to-run mapping | `execution_index.json`: explicit experiment, run ID/directory, state and finalized manifest/inventory hashes |
| Runtime identity | Frozen executable/library inventory, checked before execution and monitored between native units |
| Historical discovery | `Exx/runs/legacy-*` links and versioned `legacy-index-*.json` with original locators/hashes |
| Results / analyses / publication | Deferred; no promotion, scientific aggregation or paper generation |

`study.py` defaults to experiment-owned roots and splits a multi-experiment selection into separate
runs. An explicit `--output-root` remains available for temporary tests or deliberate overrides.
Campaign v4 freezes the exact experiment output roots and records each newly created run immediately.
It does not search for the first inventory in a directory that may contain older runs.

Historical global run directories remain at their original locations. Their absolute locators and
content hashes are preserved; historical links do not move, rewrite or duplicate evidence. They
are explicit compatibility entries, not new executions. New native evidence is physically written
inside the experiment directory. Generated runs and legacy links remain ignored by Git.

## Fresh development campaign

```bash
python3 tools/mcc_placo_study/campaign.py prepare \
  --output runs/mcc_placo_study/<new-campaign-id> \
  --verification-workers 16 --verification-batch-size 32
python3 tools/mcc_placo_study/campaign.py run \
  --plan runs/mcc_placo_study/<new-campaign-id>/plan.json
```

Fresh preparation does not attach a reuse index, previous unit results, prior E06 admission,
converted-input reuse or a runtime-transition waiver. All declared development units are attempted
from the beginning and all input conversion jobs are performed again. Existing immutable canonical
input descriptors can be referenced without regenerating or modifying them. Failed and unavailable
units remain in the complete required matrix.

The new campaign freezes the currently installed native runtime as its starting identity. Earlier
mixed-version campaigns remain separate historical evidence, not extra samples in the fresh run.

## Verification scheduling

Native acquisition remains serial. Up to 32 acquired units form a bounded batch; 16 independent
checker processes consume that queue, replacing each finished worker until the queue is drained.
Only then does the next native acquisition batch begin. This also excludes native/checker overlap
for E09/E10. Native solver configuration and affinity are unchanged. Checker BLAS/OpenMP threads
are limited to one; checkers can use the process's full permitted CPU set.

`--verification-workers` supports 1–32, with batch size between the worker count and 256. The default
is one worker and a batch of one. The current full campaign explicitly selects 16/32. Parallel
batching remains development-only. Cancellation preserves native-complete queued units so a future
continuation can run only their missing verification.

RunBuoy progress reports real terminal classifications, batch completions, active workers and
queued jobs. Local `progress.json` includes individual checker identity and sample progress. It is
not an ETA or a scientific success rate.

## Recovery

The cancelled older campaign and frozen copies of its tools remain preserved. If recovery is later
requested, stop the fresh campaign before preparing a continuation of the chosen old plan:

```bash
python3 tools/mcc_placo_study/continue_campaign.py \
  --parent-plan <explicit-old-campaign>/plan.json \
  --output <new-recovery-campaign-directory> \
  --verification-workers 16 --verification-batch-size 32
```

Recovery reads both legacy and indexed experiment-owned layouts, verifies prior evidence and
creates new experiment-owned run directories. Completed successes, scientific failures and
unavailability remain terminal. Completed native work with interrupted verification is rechecked;
unproven native attempts remain preserved as partial evidence. Runtime identity still must match,
or require a separately explicit development transition authorization. The fresh campaign does
not automatically invoke recovery or merge its results with earlier campaigns.
