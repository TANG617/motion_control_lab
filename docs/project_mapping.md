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
| MCC interactive apps | `apps/single_arm_ik/`、`apps/dual_arm_ik/` 使用 `RedOnly`；`apps/grouped_dual_arm_ik/` 固定使用 Red/Yellow/Green |
| Cartesian planning preview | `apps/cartesian_planning/` 读取版本化 JSON，调用 Core MoveLine planner，输出 CSV/PNG 并循环发布 Foxglove；不加载 robot model 或调用 IK |
| Interactive support | `adapters/interactive/` 提供 CLI、TUI、wall-clock scheduler、SPSC latest mailbox、periodic worker/Fault 和 Viz helpers |
| Solver A/B runner | planned；需要时由正式 Experiment 的 canonical timeline 单独设计，不预留交互 backend 接口 |
| Interactive preview | 独立 app topology + 共享 interactive support；wall-clock/TUI 开发路径，不是正式 Experiment runner |
| Solver source | `third_party/placo/` 中的普通 vendored 源码；直接参与主工程构建 |
| Metric evaluator | E01 执行器内的最小 metric evaluator；领域公共 evaluator planned |
| Manifest contract | `contracts/manifests/run_manifest.v1.schema.json` |
| Metric row contract | `contracts/metrics/metric_row.v1.schema.json` |
| Artifact root | `experiments/<experiment>/runs/<run-id>/` |
| Result promotion | 人工复核后写入 `experiments/<experiment>/results/`；promotion command planned |
| Analysis collector | planned |
| Static renderer | 正式 artifact-only renderer planned；`mcl_cartesian_planning` 只渲染本次开发预览结果，现有 `motion_control_lab_plot_core_planning` 仍为可选 API smoke app |
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
Single/Dual/Grouped app main.cpp
        |-- MCC builder + typed task topology
        |-- MCC request + solver state
        |-- TUI input/render
        |-- wall-clock scheduler
        +-- VisualizationFrame -> Viz FrameSink
```

Grouped 入口的计算数据流为：

```text
Green worker --soft proposal--> Yellow worker --soft proposal--> Red worker
     ^ measured snapshot             ^ measured snapshot              |
     +-------------------------------+--------------------------------+
                                      latest Red authoritative state

UI thread --targets--> Red + Yellow
UI thread <--latest Red output-- Red
```

solver worker 之间和 UI 之间使用 bounded latest-value mailbox，不等待对方；deadline 和
Starting/Running/Fault 属于 Lab 外层，不进入 MCC。Lab 的 deadline policy 支持用于能力验证的
`strict` 和用于非实时主机交互调试的 `monitor`；后者保留统计并跳过过期 release，但不会放宽
rejected attempt 或 worker exception。

当前 grouped app 的 Yellow proposal 使用 Soft 双手 Cartesian task，并以更低权重对
`left_arm_link4`/`right_arm_link4` 施加三轴 PositionTask，固定初始 X/Z、将左右 Y 分别向外偏置。
这是应用层近似，不新增 MCC 单轴 task；Red 双手 task 与三组 joint limits 仍为 Hard。

单臂和双臂入口显式拥有不同的 task topology、solver config、typed handles 和状态更新，
只共享参数解析、终端输入、wall-clock 调度、frame 映射和 sink 创建。正式实验的 `dt`
必须来自 canonical 时间轴，不能复用交互 scheduler；未来若实现 PlaCo/MCC A/B，应从
正式 Experiment 的真实需求重新设计，而不是让交互入口提前承担 backend-neutral 合同。
