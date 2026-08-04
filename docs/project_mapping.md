# Motion Control Lab 实现映射

本文将
[《可复用实验架构：从零创建、分析与发布》](./experiment_architecture.md)
中的抽象角色映射到当前仓库。这里只记录已经存在的实现；尚未落地的能力明确标记为
planned。

| 抽象角色 | 当前实现 |
|---|---|
| Raw data | `data/raw/`；当前无业务 MCAP |
| Canonical data | `data/canonical/`；MCAP→CSV schema 与 normalizer planned |
| Definition format | `contracts/definitions/experiment.v1.schema.json` + 每个实验的 `definition.json` |
| Definition validator | `tests/validate_contracts.py definition` |
| Experiment executor | 当前为 `e01_placo_smoke`；canonical timeline 驱动的通用调度 CLI planned |
| Execution adapter | `adapters/execution/` 中的 append-only artifact store、manifest writer 与通用 dependency provenance |
| Solver backend contract | `adapters/solver/` 中的 `IkSolverBackend` 和中立 result/diagnostics；当前范围为 R1 IK |
| MCC method adapter | `adapters/motion_control_core/` 中的 R1 profile、solver session 与 MCC state ownership |
| PlaCo method adapter | planned；未来实现相同 `IkSolverBackend`，不进入 Viz/TUI |
| Solver A/B runner | planned；未来以相同 target timeline 和 `dt` 驱动两个 backend，并按 `backendId` 分流 artifact |
| Interactive preview | `adapters/interactive/` + `apps/r1_*_ik_tui_teleop.cpp`；wall-clock/TUI 开发路径，不是正式 Experiment runner |
| Solver source | `third_party/placo/` 中的普通 vendored 源码；直接参与主工程构建 |
| Metric evaluator | E01 执行器内的最小 metric evaluator；领域公共 evaluator planned |
| Manifest contract | `contracts/manifests/run_manifest.v1.schema.json` |
| Metric row contract | `contracts/metrics/metric_row.v1.schema.json` |
| Artifact root | `experiments/<experiment>/runs/<run-id>/` |
| Result promotion | 人工复核后写入 `experiments/<experiment>/results/`；promotion command planned |
| Analysis collector | planned |
| Static renderer | 正式 artifact-only renderer planned；现有 `motion_control_lab_plot_core_planning` 仅为可选 API smoke app |
| Publisher / Release index | planned |

## 当前数据流

```text
E01 definition + synthetic URDF
            |
            v
 vendored placo C++ source + source fingerprint
            |
            v
 append-only experiment run
   manifest + input copy + trace + status + metrics + report
            |
            v
 contract and hash validation
```

E01 使用合成模型，只用于验证依赖、求解器 API 和证据链。真实 MCAP、
canonical CSV 和业务机器人模型接入后，应建立新的输入合同和实验，而不是把
E01 的 fixture 扩展成业务 benchmark。

MCC 交互预览采用另一条非证据主链：

```text
TUI input -> InteractiveRunner -> IkSolverBackend
                                  |       |
                                  |       +-> current implementation: R1IkSolverSession -> MCC
                                  |
                                  +-> VisualizationFrame -> Viz FrameSink
```

它与未来的 headless ReplayRunner 共享 MCC session 语义，但不共享 wall-clock pacing。
正式实验的 `dt` 必须来自 canonical 时间轴，Viz publish/render cadence 不得反向决定
算法输入时间。

未来 PlaCo/MCC A/B 复用 `IkSolverBackend` 的输入输出合同，但不复用交互调度：A/B
runner 应先用同一 `JointState` reset 两个 backend，再由同一 canonical target timeline
生成两份 solve 调用，保留独立 backend state，并在 trace、metric 和 manifest 中强制
记录 `backendId`。当前只保留接口，未实现
PlaCo backend、A/B 调度、阈值判定或命令行入口。
