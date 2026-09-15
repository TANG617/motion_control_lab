# E06：Solver 语义与比较条件对齐

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

当前执行入口已迁移到可复用 app：见 [公开执行与映射](../../../tools/app_execution/README.md)。
`definition.json` 绑定新 app/方法；原声明及输入生成器保存在 `declarations/`。
新准备入口：`python3 prepare.py --output /absolute/new-definition.json`；
执行与 dry-run 使用 `tools/app_execution/run.py`，不再启动按实验编号命名的程序。
旧 run、validation、分析及失败不变；下面保留的研究设计/历史验收描述适用于原方法，
不能作为新 app 方程或耗时结论。完整旧新配置差异见每单元的 `migration.differences`。


状态：`implemented-development`。真实 app、完整声明和输入生成器已实现；正式运行、研究分析和论文均未执行。

权威公共定义：[PROTOCOL](../../mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](../../../experiments/E06_solver_semantic_parity/inputs/README.md)。输出由 [A01](../../../analyses/A01_priority_and_robustness/README.md) 消费。对应 G02/G03/G04/G06；为 C1–C6 提供比较准入；F02、T01。

## 问题与可证伪假设

在相同物理模型、任务目标和约束下，placo-controlled 与 mcc-weighted 是否表达可比较的优化问题和结果质量？假设是在共同支持的语义子集中，两者都能通过独立 FK/约束检查；它不要求相同关节解，也不预设性能或精度相等。

production-static 的历史语义独立审计，用来量化“从原链路迁移”的差异，不能通过改变它的任务、mask 或终止参数制造 parity。

## 必需 arms 与矩阵

| case family | 方法 / 变化因素 | 固定条件与目的 |
|---|---|---|
| model_mapping | production-static、placo-controlled、mcc-weighted | 20 full/16 active 的初始生产对齐组；root、TCP、joint 名称和所有 limits |
| target_pose | placo-controlled/mcc-weighted，各 TargetSolve | 零误差、小可达目标、中等可达目标；每个状态相同 seed，分别记录收敛/预算停止 |
| servo_snapshot | placo-controlled/mcc-weighted，各 ServoStep | 相同 q/qdot/target、1 ms period；零/非零 feedback velocity；普通 soft 任务 |
| enforcement | hard、soft、shared scaled 的共同支持子集 | 逐项审计方程、gain、normalization、regularization；不支持项 not-applicable |
| representation | 物理 DOF 相同、内部变量数记录 | PlaCo floating-base/mask 行与 MCC active-space reduction 的差异显式披露 |

可达姿态从初始状态及 active joint range 的 ±1%、±5% 有界扰动生成 FK，固定 seed；超出 joint limits 的候选生成时拒绝并记录，不运行时 clamp。每个 arm 使用同一组生成后的目标。几何比较用完整模型；纯数值模型-only 观察另标 geometry policy。

单步任务 gain/控制公式不能只按 API 参数同名对齐；须从实际 task rows 核验。如果某 enforcement 不能表达同一个问题，该行输出 semantic_mismatch 及具体差异，后续不得当作纯 backend 比较。

## 指标与窗口

- Primary：逐条件 parity 审计通过率，position_error、orientation_error 的独立核验结果。
- Guardrail：hard_violation_excess、非有限输出、缺失/错序 joints、错误 frame/TCP、错误 execution mode。
- Diagnostic：实际变量/等式/不等式/scale/slack 数目、native 状态、iteration 数、full_quality_rate。
- 窗口：每个冻结 snapshot 和每个 TargetSolve 整体调用；初始化/零误差调用单独保留。时间数据只用于开发诊断，不输出性能结论。

公式与单位引用公共字典。parity 不是求解器输出向量相同：先比较模型 FK/Jacobian、目标/约束数学含义，再检查每个结果满足相应任务。模型 FK 一致性的容差要在声明写明，不能借此放宽物理限位。

## 实现接口与产物

experiment app 直接构造各 solver；输入是 canonical snapshot 和 arm config，输出 input/target hashes、原生结果、独立核验所需 q/v、task/constraint 描述及状态。公共 layer 不持有 solver。`definition.json` 与 `generate.py` 已实现，并引用真实生成的 canonical descriptor。

每 run 除公共 manifest/trace/status 外，生成 method_semantics.csv、parity_checks.csv、model_mapping.json 和 failure 表。字段必须能解释每个 mismatch 的源配置和目标配置，而不只有 bool。

## 验收及失败

开发验收需包含故意错误的 joint order、TCP offset、mode、shared/per-arm scale 的反例，确认审计能识别它们；包含多解的有效输出，确认不会因 joint vector 不同判失败。原生 solver 失败原样保留。

E07–E12 的每个 causal comparison 必须引用具体 E06 审计 artifact。未通过条件只允许单独描述性展示，paired causal effect 为 unavailable；不靠调参隐藏语义不一致。输入/输出正确性通过不是算法更优的证据。

## 开发执行与验收入口（2026-09-10）

所有 solver/task/loop 实现在 `apps/study_e06/`，通过标准 colcon 产物 `mcl_study_e06` 执行；不复用其他 app 私有实现。`generate.py` 生成 README family 内离散扫描的 `definition.json`、真实 canonical inputs/descriptors 及 case inventory，输入与 generator/model 文件均有 SHA-256。

```bash
python3 experiments/E06_solver_semantic_parity/generate.py --model /workspace/models/r1.cos.urdf
python3 -m pytest -q experiments/E06_solver_semantic_parity/tests
python3 tools/mcc_placo_study/study.py --experiment E06 --phase development --dry-run --inventory-output /tmp/E06-matrix.json
# 仅在主 agent 的串行开发验证闸门中运行选定 case 的 smoke：
python3 tools/mcc_placo_study/study.py --experiment E06 --case CASE --method METHOD --repeat 0 --phase development --smoke --timeout 60
```

`verify.py --request <unit/request.json>` 在 native 计时外自动调用；输出独立 XML FK、hard-set 原始违反、同状态 Jacobian、原生 reference 与独立 final residual、解析主动集参考、独立 bounded least-squares 主任务参考及质量/事件表。后者只构成单元正确性核验，没有跨 run 科研统计。非唯一解按语义目标/约束判断，保持容差不加 epsilon。

数学边界：两库实际位置 task 都控制 frame origin，使用 `p_frame_goal = p_tcp_goal - R_goal * offset`。该共同生产式变换与真正的 TCP Jacobian 任务有差异，`method_semantics`/parity 中明确披露；实际 TCP 误差由独立 FK 复算。MCC 保持现有 1e-8 numerical regularization，不追加目标修复。PlaCo native throw-on-failure、MCC post-solve acceptance、TargetSolve 停止政策的差异保留，不将不同 accepted policy 宣称纯 backend parity。

所有动态回灌均为明确的理想运动学 committed-state；HOLD 保留原 q，原生拒绝不消费失败 iterate。smoke 只覆盖显式选定且有界的前缀，不能代表 4 s / 6 s 完整窗口或正式结果。正式前仍需模型/输入/配置/阈值及 E06 配对准入冻结、环境预检和新建输出目录；本任务不执行正式 campaign。

production-static 使用本目录 `production_adapter.py` 启动冻结 `mcl_baseline replay`；只转换 canonical CSV/native trace，不能注入任意 q/qdot 或修改生产配置。原 app 缓冲 trace 到退出的历史限制保留，崩溃仍保留首个 stderr 与进程证据。`admit_pairs.py --evidence-root RUN_ROOT --output NEW_JSON` 消费精确 unit 身份及 artifact hash 做方法配对准入，保留失败/缺失/ambiguous 单元；无统计或效果估计。

## 实际开发验收证据

代码：implemented；专项测试：7 passed；真实 smoke：21 个声明单元中 20 completed / 独立检查通过，1 个 PlaCo scaled 能力差异 unavailable。正式运行就绪：未完成正式冻结/准入/RT 条件；formal-executed：false；分析和论文：deferred。

[真实 E06 manifest](../../../runs/mcc_placo_study/development_acceptance/smoke/20260910T104933.419463Z-93358841be/E06.manifest.json) 与相邻 inventory、逐单元 command/raw/native status 为权威证据。18 条实际 PlaCo 原生位置 task row 审计全部通过，独立 Jacobian 最大差 2.6533e-10，b 最大差 6.6613e-16；这只验证行构造，不能抹去原生接受/TargetSolve/TCP 数学差异。

```bash
python3 tools/mcc_placo_study/study.py --experiment E06 --phase development --smoke --repeat 0 --timeout 60 --output-root runs/mcc_placo_study/development_acceptance/smoke
```

冻结 baseline 的 replay/配置测试和本实验真实 replay 已通过。额外历史 `apps.mcl_baseline_tui_projection` 仍失败：默认关闭 Requirements page，但旧测试同时期待四页与该页独有 scale 表；源定位 `contracts/presentation/ik_app_snapshot.hpp:270`、`components/tui/standard_ik_tui.cpp:831`、`apps/baseline/tests/tui_projection.cpp:79`。未修改冻结 baseline、solver 或共享展示组件来隐藏该独立问题。
