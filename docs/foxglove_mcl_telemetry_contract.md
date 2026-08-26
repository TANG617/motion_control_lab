# MCL Foxglove Topic 与 Telemetry 合同

版本：`mcl.telemetry.v1`

本合同是 Motion Control Lab 的唯一输出命名合同。所有新输出使用 `/mcl`；不双发、
不提供 `/mc` alias，也没有 legacy compatibility flag。命令行中仍出现的 `/mc/...`
只用于选择既有 MCAP 数据集中的输入流，不是 MCL 输出 topic。

机器可读 topic 合同位于
[`contracts/visualization/`](../contracts/visualization/)，Protobuf wire schema 位于
[`contracts/telemetry/mcl_telemetry_v1.proto`](../contracts/telemetry/mcl_telemetry_v1.proto)。

## Topic 表

| Topic | 消息类型 | 语义/发布条件 |
|---|---|---|
| `/mcl/run/info` | `mcl.telemetry.v1.RunInfo` | 启动时及每秒重发 resolved run 元数据。 |
| `/mcl/events` | `foxglove.Log` | rejection、fallback、projection、deadline miss、queue drop、fault、EOS 的状态边沿。 |
| `/mcl/cartesian/input/left` | `foxglove.PoseInFrame` | decoder/source 给 app 的左原始 pose；replay 保留 TCP/input frame 语义。 |
| `/mcl/cartesian/input/right` | `foxglove.PoseInFrame` | 右原始 pose。 |
| `/mcl/cartesian/goal/left` | `foxglove.PoseInFrame` | TCP→EE、frame 转换或 teleop 积分后的左 EE goal。 |
| `/mcl/cartesian/goal/right` | `foxglove.PoseInFrame` | 右 EE goal。 |
| `/mcl/cartesian/reference/left` | `foxglove.PoseInFrame` | Cartesian planner 当前提交给 IK 的左 reference。 |
| `/mcl/cartesian/reference/right` | `foxglove.PoseInFrame` | 右 reference。 |
| `/mcl/cartesian/ik/left` | `foxglove.PoseInFrame` | accepted raw IK positions 的左 EE FK。 |
| `/mcl/cartesian/ik/right` | `foxglove.PoseInFrame` | accepted raw IK positions 的右 EE FK。 |
| `/mcl/cartesian/execution/left` | `foxglove.PoseInFrame` | committed post-OTG state 的左 EE FK。 |
| `/mcl/cartesian/execution/right` | `foxglove.PoseInFrame` | committed post-OTG state 的右 EE FK。 |
| `/mcl/joints/ik` | `foxglove.JointStates` | accepted raw IK 完整 P/V。 |
| `/mcl/joints/execution` | `foxglove.JointStates` | committed post-OTG 完整 P/V。 |
| `/mcl/nullspace/elbow/scene` | `foxglove.SceneUpdate` | null-space app 的 link4 target、raw IK FK、executed OTG FK 小球以及 target→execution 误差线。 |
| `/mcl/telemetry/tracking/cartesian` | `mcl.telemetry.v1.CartesianTracking` | 每个 control/IK attempt 的 goal→reference→IK→execution 误差链。 |
| `/mcl/telemetry/tracking/joints` | `mcl.telemetry.v1.JointTracking` | 仅 committed tick；逐关节 IK、raw target、projected target、execution P/V/A/J。 |
| `/mcl/telemetry/solver/ik` | `mcl.telemetry.v1.SolverTelemetry` | 每个 IK attempt；`solver_kind` 和 `passes[]` 区分普通 IK/HKS。 |
| `/mcl/telemetry/solver/avoidance` | `mcl.telemetry.v1.SolverTelemetry` | avoidance 原生频率。 |
| `/mcl/telemetry/planner/cartesian` | `mcl.telemetry.v1.PlannerTelemetry` | Cartesian replan/step。 |
| `/mcl/telemetry/planner/joint` | `mcl.telemetry.v1.PlannerTelemetry` | 每个 committed tick 一条 `operation=plan+step`；失败时记录失败的单项 operation。 |
| `/mcl/telemetry/safety/collision` | `mcl.telemetry.v1.CollisionTelemetry` | avoidance accepted attempt 的全局和逐 pair 距离。 |
| `/mcl/telemetry/coupling/avoidance_to_ik` | `mcl.telemetry.v1.CouplingTelemetry` | 每个 IK attempt 的 proposal revision、age 与消费状态。 |
| `/mcl/telemetry/worker/control` | `mcl.telemetry.v1.WorkerTelemetry` | control worker 每次 release 的调度与执行时间。 |
| `/mcl/telemetry/worker/avoidance` | `mcl.telemetry.v1.WorkerTelemetry` | avoidance worker 每次 release 的调度与执行时间。 |
| `/mcl/telemetry/transport` | `mcl.telemetry.v1.TransportTelemetry` | UI publish tick 的 queue depth/drop/age、encode/write time 与字节数。 |
| `/mcl/telemetry/replay` | `mcl.telemetry.v1.ReplayTelemetry` | 仅 replay mode；原始 clocks、timeline、状态机和 settling predicates。 |

planner-only app 还可发布 `/mcl/cartesian/scene`（`foxglove.SceneUpdate`）来显示路径和
坐标轴；它不是 OTG app 的必需 topic。

`mcl_planned_hierarchical_step_otg` 与
`mcl_planned_hierarchical_step_otg_nullspace` 默认发布上表中除 replay topic 外各自具备的
全部状态与数值流；`/mcl/events` 仅在事件发生时出现。null-space app 还发布
`/mcl/nullspace/elbow/scene`：绿色为 enabled target，蓝色为 raw HKS FK，橙色为 executed OTG
FK，绿色连线表示 target→execution 误差；link4 与 Yellow objective 数值仍记录在通用 HKS
`tasks[]` 中。replay mode 额外启用 `/mcl/telemetry/replay`。

## 时间合同

- `PoseInFrame.timestamp`、`JointStates.timestamp`、Protobuf
  `context.timestamp` 与对应 MCAP/WebSocket `log_time` 都是同一个实际事件的
  wall-clock time。
- `context.run_time_ns` 使用该 run 唯一的 steady-clock origin，可跨 control、avoidance
  和 UI worker 比较；`sequence` 是 run 内全局递增序号。
- `context.emit_time` 只由非实时发送线程在编码前写入。
  `emit_time - timestamp` 是 telemetry queue/encode 前等待时间，不是 solver latency。
- 原始 replay header/log/publish time 只进入 `ReplayTelemetry`，不得成为当前输出的
  `log_time`。
- latest-state pose topic 之间不承诺 attempt 原子性。严格逐 tick 比较只使用单条
  `CartesianTracking` 或 `JointTracking`。
- rejected attempt 可以产生 solver、worker、tracking 和 event，但 `committed=false`，
  不产生新的 `JointTracking`、`/mcl/joints/execution` 或 execution FK 状态。

## Foxglove 面板建议

1. Robot State 使用 `/mcl/joints/execution`；需要查看 raw IK 时切换到
   `/mcl/joints/ik`。
2. 3D 面板同时选择 input、goal、reference、IK、execution pose，颜色按链路阶段固定。
3. Plot 直接展开 `joints[].name`、`arms[].side`、`passes[].label`、
   `tasks[].name/components[].label` 与 `pairs[].label`，这些字段是稳定 dynamic-series
   label。
4. State Transitions 使用 `/mcl/events`、`context.outcome`、planner `state/operation`
   和 coupling `state`。
5. 诊断 transport 对控制的影响时，同时绘制 control `execution_ms/overrun_ms`、
   transport `oldest_sample_age_ms/write_time_ms` 和 `dropped_samples`。

## 传输边界

control 与 avoidance 各自通过容量固定的 SPSC 队列非阻塞提交。UI/发送线程统一按
`timestamp, sequence` 排序，补 `emit_time`、序列化 Protobuf，并交给通用
`motion_control_viz` WebSocket/MCAP transport。`--viz none` 不创建 telemetry clock、
queue 或 encoder，控制回调也不执行 telemetry 投影。
