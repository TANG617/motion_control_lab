# E11：Planning–IK–OTG 误差传播

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

当前执行入口已迁移到可复用 app：见 [公开执行与映射](../../../tools/app_execution/README.md)。
`definition.json` 绑定新 app/方法；原声明及输入生成器保存在 `declarations/`。
新准备入口：`python3 prepare.py --output /absolute/new-definition.json`；
执行与 dry-run 使用 `tools/app_execution/run.py`，不再启动按实验编号命名的程序。
旧 run、validation、分析及失败不变；下面保留的研究设计/历史验收描述适用于原方法，
不能作为新 app 方程或耗时结论。完整旧新配置差异见每单元的 `migration.differences`。


状态：`implemented`；开发测试与有界 smoke 已执行，正式实验未执行。公共定义：[PROTOCOL](../../mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](../../../experiments/E11_planning_ik_otg_error_propagation/inputs/README.md)。消费：[A03](../../../analyses/A03_end_to_end_and_recorded_motion/README.md)。对应 G03/G06/G10/G14；C5；F09。

## 问题与假设

IK 侧的精度、连续性或计算改善能否保留到 committed execution state？CartesianPlanner 和 Joint OTG 分别怎样改变跟踪误差、滞后与 P/V/A/J？

假设是某项 IK 改善在相同下游条件下仍能改善预声明执行指标；被 OTG 抵消或导致更大滞后的情况同样构成主要结果。输入目标、规划 reference、raw IK 和 post-OTG execution 不是同一条轨迹。

## 链路矩阵

| pipeline | 数据流 | 配对含义 |
|---|---|---|
| direct | canonical target → IK → 明确运动学积分 | 无输入/输出规划的基础 |
| cartesian | target → CartesianPlanner → IK → 积分 | 固定 IK 方法，隔离输入规划 |
| joint | target → IK → JointPlanner OTG → committed state | 固定 IK 方法，隔离关节规划 |
| cartesian-joint | target → CartesianPlanner → IK → JointPlanner OTG | 完整链路及交互效应 |

每个可支持方法分别运行四条链路：placo-controlled、mcc-weighted、E07–E10 选定的 mcc-hqp；production-static 单独作历史整体链路参考。现有 hierarchical app 的 profile 不包含全部矩阵，缺失链路需在 app-owned 实验入口实现，不改变生产 app 的默认 profile。

对齐 Cartesian/Joint planner 版本、同步方式、limits、input P/V/A policy、post-IK projection 及反馈模型。任何 clipping/projection 必须在 raw IK、raw target、projected target、execution 四处留下状态和改变量，不用 downstream clipping 修饰 solver 可行性结果。

## 场景与时钟

由 E06 对齐的 R1 模型产生可达的低幅正弦与分段平滑目标。开发矩阵为 TCP 振幅 0.01/0.03 m、频率 0.2/0.5/1 Hz；构造前用独立 FK/局部参考筛选并记录包络，不按候选效果筛选。另有停止—保持—继续与显式目标 step 的压力组，step 不混成满足平滑输入假设的场景。

默认每条 10 s，完整窗口和预先标记的运动/保持/转换窗口分别统计。目标源 100 Hz，控制 1 kHz，feedback 为声明的理想积分/post-OTG 真值。额外反馈延迟 5/20 ms 为单独敏感性条件，不能混入 planner-only 因果对照。

本实验无力矩、接触力或真实 plant；MuJoCo 若使用 setKinematicState/forward 仅作为运动学一致性核验/显示。若要研究动力学或硬件，另立实验与 claim。

## 指标与误差归因

Primary：相同输入时刻的 goal→execution position_error/orientation_error，以及 target_execution_lag。Guardrail：executed joint limits、raw IK hard_violation_excess、完整质量与窗口覆盖。Secondary：joint_vaj、次级任务收益保留量。

分别计算 goal→reference、reference→IK、IK→execution、goal→execution 的同时刻误差；非线性向量范数不能简单相加当成总误差。原始输入与 planner goal 的 frame/TCP 变换单独校验。lag 可作诊断，但主误差不事后时间平移。

qdot/qddot/jerk 有原生样本时优先记录，并保持 raw/committed 来源；缺失时的离线差分注明公式、dt、窗口和边界处理。不能将噪声放大的有限差分尖峰自动称为真实执行冲击。

## 产物与实现接口

app-local planner/IK loop 输出原子 stage tracking record、每级 q/v/a、目标 sequence 和 committed 状态。生成 stage_error_trace.csv、joint_stream.csv、projection_events.csv、pipeline_summary.csv。A03 生成 F09 四阶段叠图及 paired pipeline effects。

参考提取与统计不能调用私有 app 内部对象；跨 app 通过 canonical input、resolved config 和 artifact 配对。输入解码、timeline、图形 transport 可以复用机械组件。

## 验收

零运动一致性、已知 TCP 偏置、已知固定延迟、planner-only/joint-only/完整链路四个代表条件贯通；有意产生 projection 和 rejected tick 验证记录。HOLD 期间继续统计实际保持轨迹，不按 solver attempts 删掉误差区间。

只有两条相同 IK 方法且其他条件一致的管线可称 planner 消融；整体不同链路只称迁移比较。独立反馈模型中的改善不能直接外推到真实 robot tracking。

## Implemented development interface (2026-09-10)

`apps/study_e11` owns the real MCC weighted/two-layer HQP and PlaCo topologies,
CartesianPlanner retargeting, JointPlanner OTG and ideal state evolution. The two-layer
identity is position Primary, orientation/posture Secondary. Its later E07–E10 campaign
selection is **pending**; the implementation does not claim an already selected winner.
No production-static source or algorithm settings are changed. Its separate process adapter
keeps the original 100 Hz whole-chain semantics and does not confer controlled parity.

`definition.json` contains 370 ordinary factor cells (10 source families × 3 delays ×
3 controlled methods × 4 pipelines, plus 10 historical cells), and 2 explicitly labelled
**development-only** downstream fault fixtures. Reachable sine envelopes use an independent
URDF FK joint7 circular-motion witness at requested 1/3 cm TCP excursion; no candidate
result filters the input. Inputs retain full-order witness joints and generator source hashes.
State integration is explicitly `q_committed += dt * native_velocity` for direct/Cartesian
arms; joint arms consume native post-OTG P/V/A. Delayed feedback supplies the IK capture
state; it does not turn the kinematic experiment into a physical closed loop.

Each tick keeps goal/reference/raw IK/raw downstream/projected/committed state, native IK
acceptance separately from downstream acceptance, and HOLD even after a native rejected
attempt. `attempt_begin` is flushed before planning/IK so a thrown error keeps the triggering
identity. Run wall timestamps bound the complete pipeline; virtual source/release/capture
clocks are separately labelled and never produce hardware/RT latency claims. Stage XML FK,
raw and executed limit checks, source/state versions, P/V/A/J derivative provenance and
fixed root-X lag diagnostics run outside native execution. A zero-variance or insufficient
lag window remains unavailable; no primary error is shifted to improve tracking.

```bash
python3 experiments/E11_planning_ik_otg_error_propagation/prepare.py
python3 tools/mcc_placo_study/study.py --experiment E11 --dry-run --inventory-output /tmp/e11-matrix.json
python3 tools/mcc_placo_study/study.py --experiment E11 --method mcc-hqp-2 --case zero-a0-f0.2-delay0 --repeat 0 --smoke --timeout 60
python3 tools/mcc_placo_study/study.py --experiment E11 --case fixture-projection --case fixture-otg-rejection --repeat 0 --smoke --timeout 60
python3 -m unittest discover -s experiments/E11_planning_ik_otg_error_propagation/tests -v
```

The fault fixtures run real IK and intentionally transform only the subsequent joint
planner target. The projection fixture records that explicit transformation and bounds
projection; the unprojected fixture is expected to retain native OTG rejection and HOLD.
Their process failure is evidence, not a successful ordinary-motion outcome. The public
acceptance inventory records actual smoke/test results. Formal execution remains separate:
freeze method selection, source/config/model hashes, thresholds/windows, and host RT/core/
frequency attestation, then execute all selected units serially. Scientific analysis,
figures, reports and paper work are **deferred**.

### Development acceptance evidence

The final bounded native smoke is
[runs/mcc_placo_study/development_acceptance/smoke/20260910T110546.488837Z-f7861e12e2](../../../runs/mcc_placo_study/development_acceptance/smoke/20260910T110546.488837Z-f7861e12e2).
It selects zero motion and 1 cm/0.2 Hz motion with 20 ms feedback delay, all three
controlled methods and all four pipelines (24 units × 25 ticks), plus both 3-tick
fault fixtures. **25 units completed; one intentionally failed with exit 2.** All
26 native traces passed independent state/FK/stage checks. The expected failure is
JointPlanner's rejection of the injected right_arm_joint7 target 1.1472 rad beyond
its 1.0472 rad upper bound; native IK remains accepted/full and all three execution
ticks remain HOLD. The separate projection fixture records 1.1472→1.0472 explicitly
and commits its native OTG outputs. This is development failure-path validation.

The earlier smoke `20260910T105511.949137Z-f7861e12e2` is retained. It exposed an
independent lag measurement defect for non-binary constant-valued signals. The shared
metric now rejects exact zero-range overlap windows before mean subtraction; regression
tests and the new smoke preserve the corrected measurement chain. All twelve final
zero-input method/pipeline units report lag **not-applicable: zero variance**. No primary
tracking error was shifted, and no candidate algorithm was changed to correct measurement.

Four focused E11 tests pass, including known TCP offset, delay identities and C2 quintic
knot continuity. The original discontinuous piecewise-smooth generator output is retained
under [inputs/superseded/e339e9c97026a9fb](../../../experiments/E11_planning_ik_otg_error_propagation/inputs/superseded/e339e9c97026a9fb), with its
original bytes, descriptor, relocated valid descriptor and explicit supersession reason.
The active source now uses continuous P/V/A knots at 0/2.5/5/7.5/10 seconds. Full matrix
inventories are in the parent development acceptance directory; smoke does not execute
the full 10-second/370 ordinary-cell campaign. Code: implemented/tested/smoke-executed.
Formal readiness: pending preregistration/freeze/platform gates. Formal execution: none.

补充 historical production-static adapter 一单元已实际 completed/validated：`20260910T112020.700612Z-07be1f3d89`。完整命令与追加证据见 [最终开发验收](../study/DEVELOPMENT_ACCEPTANCE.md)。
