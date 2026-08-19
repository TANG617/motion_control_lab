# Motion Control Lab 实现映射

本文将
[《可复用实验架构：从零创建、分析与发布》](./experiment_architecture.md)
中的抽象角色映射到当前仓库。这里只记录已经存在的实现；尚未落地的能力明确标记为
planned。

| 抽象角色 | 当前实现 |
|---|---|
| Raw data | `data/raw/`；当前无业务 MCAP |
| Canonical data | `data/canonical/`；MCAP/CSV 现在经相同 typed contracts 与 timeline 消费，正式 dataset promotion 仍 planned |
| Definition format | `contracts/definitions/experiment.v1.schema.json` + 每个实验的 `definition.json` |
| Definition validator | `tests/validate_contracts.py definition` |
| Experiment executor | `e01_placo_smoke` 与可选 `e04_opensot_smoke` 执行同一 R1 左臂位置任务；E02 由 `mcl_servo_step replay` 执行 PSI R1 双臂 canonical replay |
| Execution adapter | `adapters/execution/` 中的 append-only artifact store、manifest writer 与通用 dependency provenance |
| Physical source | `adapters/data/source/` 中平级的 `McapSource` / `CsvSource`；只处理物理格式 |
| Typed decoder | `adapters/data/decoder/` 中的 registry、ROS2 CDR Pose/JointState 和 CSV mapping decoder |
| Temporal projector | `adapters/data/temporal/` 中的 timestamp selection、严格时间校验、immutable Timeline、projection 与 ReplayClock |
| Semantic projector | `adapters/data/projection/dual_arm_timeline.*`；只消费两个 `TypedStream<StampedPose>` |
| Replay plan | `mcl_replay_plan`；预加载输入并输出 timeline trace/manifest，不运行 solver |
| MCC interactive apps | `apps/single_arm_servo_step/`、`apps/servo_step/`、`apps/target_solve/` 各自拥有完整的普通 `KinematicsSolver` topology 和配置；`apps/grouped_servo_step/` 与 `apps/planned_grouped_servo_step/` 各自拥有完整 Red/Yellow topology |
| Cartesian planning preview | `apps/cartesian_planning/` 读取版本化 JSON，调用 Core `CartesianPlanner`，输出 CSV/PNG 并循环发布 Foxglove；不加载 robot model 或调用 IK |
| Interactive runtime | `mcl_interactive_runtime` 提供独立 TUI console/presentation、source-mode controls、wall-clock scheduler、SPSC latest mailbox、periodic worker/Fault 与 rolling percentiles，不依赖 Viz |
| Preview transport | `mcl_preview_transport` 提供 Lab-owned CLI/config policy 与 WebSocket/MCAP/Null sink factory |
| IK preview projection | `mcl_ik_preview` 将 Lab `IkDebugFrame` 的 target、FK 与 joint state 投影为通用 `RenderBatch`；app-specific extension 留在 app |
| Replay support | `adapters/replay/` 提供 solver-agnostic timeline loader、ReplayClock、provenance 与 v2 artifact mechanics；solver loop 与失败语义仍由 app 拥有 |
| IK visualization contract | `contracts/visualization/*.json` 是唯一来源；build-tree generator 与 `motion_control_lab::visualization_contracts` 暴露 topic/ChannelSpec，并由 C++ conformance 检查 collection 对齐 |
| Solver A/B runner | planned；需要时由正式 Experiment 的 canonical timeline 单独设计，不预留交互 backend 接口 |
| Interactive preview | 独立 app topology + 共享 interactive support；E02 的 Foxglove sink 是只观察 canonical replay 的可选输出，不替换 ReplayClock |
| Solver source | `third_party/placo/` 直接参与主工程构建；`third_party/OpenSoT/` 只在 E04 开启时通过隔离的 external project 构建 |
| Metric evaluator | E01/E04 执行器内的最小 metric evaluator；领域公共 evaluator planned |
| Manifest contract | `contracts/manifests/run_manifest.v1.schema.json` |
| Metric row contract | `contracts/metrics/metric_row.v1.schema.json` |
| Artifact root | `experiments/<experiment>/runs/<run-id>/` |
| Result promotion | 人工复核后写入 `experiments/<experiment>/results/`；promotion command planned |
| Analysis collector | planned |
| Static renderer | 正式 artifact-only renderer planned；`mcl_cartesian_planning` 只渲染本次开发预览结果，现有 `motion_control_lab_plot_core_planning` 仍为可选 API smoke app |
| Publisher / Release index | planned |

## 当前数据流

```text
E01 / E04 definition + /workspace/models/r1.cos.urdf
            |
            +--> vendored PlaCo C++ source
            +--> isolated OpenSoT C++20 bridge (optional)
            |
            v
 solver source fingerprint + shared model hash
            |
            v
 append-only experiment run
   manifest + input copy + trace + status + metrics + report
            |
            v
 contract and hash validation
```

E01/E04 使用同一个固定 R1 模型和按名称映射的关节状态，只用于验证依赖、
求解器 API 和证据链。两者的 solver-native regularization 与 step policy 是显式允许差异；
它们不是性能 benchmark。

MCC 交互预览采用另一条非证据主链：

```text
Single/Dual/Grouped app main.cpp
        |-- MCC builder + typed task topology
        |-- MCC request + solver state
        |-- TUI input/render
        |-- wall-clock scheduler
        +-- Lab projection -> RenderBatch -> Viz RenderSink
```

Grouped 入口的计算数据流为：

```text
Yellow worker --soft proposal/coupling--> Red worker
      ^                                      |
      +--------------------------------------+
               latest Red authoritative state

UI thread --targets--> Red
UI thread <--latest Red output-- Red
```

solver worker 之间和 UI 之间使用 bounded latest-value mailbox，不等待对方；deadline 和
Starting/Running/Fault 属于 Lab 外层，不进入 MCC。Lab 的 deadline policy 支持用于能力验证的
`strict` 和用于非实时主机交互调试的 `monitor`；后者保留统计并跳过过期 release，但不会放宽
rejected attempt 或 worker exception。

当前两个 grouped app 的 Red 使用双手 scaled Hard position/orientation task。Yellow 使用 4 对
app-local R1 link pair 的 Soft self-collision requirement（minimum/influence distance
0.30/0.35 m，gain 2 s^-1，weight 100），当前不注册 posture task。Yellow→Red coupling weight
为 10。raw grouped 的 Red 是 Hard position/velocity、Yellow 仅 Hard position；planned grouped
额外为 Red 注册 PSI R1 acceleration limits，Yellow 仍仅 position。collision margin 是诊断，
不构成硬安全授权。

每个 app 显式拥有自己的 task topology、solver config、typed handles 和状态更新；相同配置也不
跨 app 合并。它们只共享 R1 固定参数、终端/TUI 初始化、wall-clock 调度、frame 映射和 sink 创建。正式实验的 `dt`
必须来自 canonical 时间轴，不能复用交互 scheduler；未来若实现 PlaCo/MCC A/B，应从
正式 Experiment 的真实需求重新设计，而不是让交互入口提前承担 backend-neutral 合同。

## Canonical replay 数据流

```text
MCAP channel/schema/payload ----> ROS2 CDR decoder --\
                                                    +--> TypedStream<StampedPose>
CSV header/row -------------> configured CSV decoder-/              |
                                                                    v
                                             original timestamp validation
                                                                    |
                                                                    v
                                      exact/nearest DualArmTimeline pairing
                                                                    |
                                                                    v
                                         preserve/fixed-period one-to-one retime
                                                                    |
                                                                    v
                                    ReplayClock -> optional headless MCC IK
```

MCAP 与 CSV 是同层 source backend；semantic projector 不查看 physical format。时间顺序
固定为：选择 logical timestamp、严格校验原 stream、在原时间上配对、形成 semantic
frame，最后 projection。`fixed-period` 仅将第 `i` 帧标为 `i * period_ns`，不执行
interpolation/resampling，也不会补齐缺失的左/右样本；unmatched 事实在 retime 前已经
成为 error 或 diagnostics。

`mcl_replay_plan` 与三个 ServoStep app 的 replay 子命令在启动 clock 前完整读取和解码输入。
realtime 使用独立于 solver rate 的 projected target timeline；solver 消费 latest target，未消费
的旧帧计入 dropped counter。TUI pause 只冻结 timeline，solver/planner 继续跟踪当前 goal；
resume 重置 clock 原点以避免追赶暂停期间的 deadline。Viz 只消费 solver 输出，不推进 timeline
或修改算法 `dt`。所有 IK preview/replay 的 topic 与 FK 数据一致性遵循
[`foxglove_ik_visualization_contract.md`](./foxglove_ik_visualization_contract.md)。E01/E04 的
R1 smoke 语义和输入由 pair contract 固定。
