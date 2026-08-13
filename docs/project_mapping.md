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
| Experiment executor | `e01_placo_smoke`；E02 由 `mcl_dual_arm_replay_ik` 执行 PSI R1 双臂 canonical replay |
| Execution adapter | `adapters/execution/` 中的 append-only artifact store、manifest writer 与通用 dependency provenance |
| Physical source | `adapters/data/source/` 中平级的 `McapSource` / `CsvSource`；只处理物理格式 |
| Typed decoder | `adapters/data/decoder/` 中的 registry、ROS2 CDR Pose/JointState 和 CSV mapping decoder |
| Temporal projector | `adapters/data/temporal/` 中的 timestamp selection、严格时间校验、immutable Timeline、projection 与 ReplayClock |
| Semantic projector | `adapters/data/projection/dual_arm_timeline.*`；只消费两个 `TypedStream<StampedPose>` |
| Replay plan | `mcl_replay_plan`；预加载输入并输出 timeline trace/manifest，不运行 solver |
| MCC interactive apps | `apps/single_arm_ik/` 使用 `RedOnly`；`apps/dual_arm_ik_servo_step/`、`apps/dual_arm_ik_target_solve/` 复用一个固定 topology 的 `KinematicsSolver`；`apps/grouped_dual_arm_ik/` 固定使用 Red/Yellow |
| Cartesian planning preview | `apps/cartesian_planning/` 读取版本化 JSON，调用 Core MoveLine planner，输出 CSV/PNG 并循环发布 Foxglove；不加载 robot model 或调用 IK |
| Interactive support | `adapters/interactive/` 提供 CLI、TUI、wall-clock scheduler、SPSC latest mailbox、periodic worker/Fault 和 Viz helpers |
| IK visualization contract | `contracts/visualization/foxglove_ik.v1.json` + `foxglove_ik_v1.hpp` 固定五条 Foxglove topic，并要求 FK 与同帧 joint state 一致 |
| Solver A/B runner | planned；需要时由正式 Experiment 的 canonical timeline 单独设计，不预留交互 backend 接口 |
| Interactive preview | 独立 app topology + 共享 interactive support；E02 的 Foxglove sink 是只观察 canonical replay 的可选输出，不替换 ReplayClock |
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

当前 grouped app 的 Red 使用双手 Hard position/orientation task。Yellow 使用固定 initial pose 的
Soft posture task（weight 10）和 10 对 curated R1 link pair 的 Soft self-collision requirement
（minimum/influence distance 0.02/0.07 m，gain 20 s^-1，weight 1）。Yellow→Red coupling weight
为 1，两组 joint position/velocity limits 均为 Hard。collision margin 是诊断，不构成硬安全授权。

单臂和双臂入口显式拥有不同的 task topology、solver config、typed handles 和状态更新，
只共享参数解析、终端输入、wall-clock 调度、frame 映射和 sink 创建。正式实验的 `dt`
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

`mcl_replay_plan` 与 `mcl_dual_arm_replay_ik` 在启动 clock 前完整读取和解码输入。
后者的 realtime 路径使用 absolute monotonic deadline 并记录 lateness/miss；batch 不
sleep。E02 的可选 Foxglove sink 在等待阶段先发布初始帧，空格 gate 结束后才建立 replay
clock 原点，因此人工等待不计入 lateness。Viz 只消费 solver 输出，不推进 timeline 或修改
算法 `dt`。现有 interactive preview 仍使用 TUI/Viz 和 interactive scheduler，服务人工调试，
不是 canonical replay 或证据执行器。所有 IK preview/replay 的 topic 与 FK 数据一致性遵循
[`foxglove_ik_visualization_contract.md`](./foxglove_ik_visualization_contract.md)。E01 的合成
PlaCo smoke 语义和输入保持不变。
