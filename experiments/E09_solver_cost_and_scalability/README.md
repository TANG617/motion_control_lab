# E09：Solver 计算成本与规模扩展

状态：`implemented-development-accepted`；正式运行就绪与正式实验完成均未声明。公共定义：[PROTOCOL](../../docs/mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../docs/mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](inputs/README.md)。消费：[A02](../../analyses/A02_runtime_and_scheduling/README.md)。对应 G03/G09/G13；C3；F05/F06、T03。

## 问题与假设

在给定质量、任务与约束条件下，MCC 的完整 IK 调用成本如何与 PlaCo 比较？HQP 每增加语义层的代价是什么，backend/装配/观测各占多少？假设由配置冻结后预声明的主要成本指标判断，允许 MCC 在简单问题或部分 backend 上更慢。

比较分两条：共同数学问题和可用 backend 的归因实验；各方法合理 native 配置的整体能力实验。当前 HKS backend 的实际支持与数值前提必须查源码并运行准入案例，不为统一 backend 给目标矩阵加正则。

## 条件矩阵

| factor | 值 / 条件 | 用途 |
|---|---|---|
| mode | TargetSolve、ServoStep 独立表 | 不混用收敛迭代与控制 tick |
| task workload | 双手位置；加朝向；加 posture；加 link4 | 同样的物理目标与 enabled 行 |
| layers | weighted、2 层、3 层；Terminal disabled 为主条件 | 拆分层级成本；Terminal 若研究则另立 arm |
| active set | 远离边界、近边界、冻结的活动集变化序列 | 尾延迟和 warm-start 效果 |
| backend | 各路径正式支持的 ProxQP/eiquadprog；PlaCo 原生 backend | 对应 E06 audit；缺支持标 N/A |
| warm start | native on/off（有能力时） | 稳态复用收益，不以启动 dummy solve 冒称 warm start |
| observation | minimal timing、full diagnostics/telemetry | 单独量化观测成本 |

先采用 E06 的 16-active-joint 条件。额外 DOF 只在同物理 R1 模型下显式变更 active set，不能复制任务行制造虚假的“复杂任务泛化”。重复/秩亏行压力案例单独归类，不混入真实任务规模曲线。

输入为固定顺序 snapshots，所有方法收到相同 q/qdot/target/constraint series。每个测量单元新进程，cold、warm-up、steady windows 使用 DATA_ANALYSIS 的预声明规则。arm 顺序采用 seed 固定的平衡顺序；timed 运行串行。

## 计时边界与指标

Primary 为完整 ik_call_time 的 P99，按相同 full-quality 条件解释，同时报告全部 attempt 的时间和质量分布。不能删掉失败耗时后只展示成功更快。

完整计时从 canonical request 进入 app-owned IK adapter 开始，到可消费结果提取结束；解码、输入预加载、模型构建、独立 oracle、日志序列化和 renderer 不在内。若某方法必须在调用内 FK/拷贝/状态转换，则计入并在边界表注明；不让另一方法等价工作移到计时外。

Secondary：P50/P95/max、CPU time、分配数、任务规模曲线。Guardrail：hard violation、full/partial/rejected 质量和完整样本覆盖。Diagnostic：assembly/各 QP pass/warm-start 的细分时间；尚未同口径暴露的字段标 unavailable。

允许复用同一进程内数值 workspace，但各方法拥有独立实例。装配和 backend 细分 instrumentation 属于单独 observation condition，不能从两个不同运行相减宣称精确的同 tick 时间分解。

## RT 环境与执行政策

正式测量等待用户重启后预检通过。记录同类型核心、频率与调度政策、cpuset、依赖和 executable hash。CPU 编号由实际预检确定，冻结后不静默换核。当前仅实现代码、dry-run 和有界 smoke；即使 smoke 的样本足够多也不能晋升为正式计时。

测量前预加载输入，warm-up 不写成隐藏的默认操作。min/full observation 记录缓冲丢样；原始样本或兼容直方图保存到 artifact，不能使用 TUI 最后 4096 样本作全 run P99。alloc instrumentation 若显著扰动时间，时间与分配分别测并披露。

## 产物与验收

输出 timing_boundaries.json、call_timing.csv/声明的 histogram、quality_counts.csv、workload_inventory.csv、environment.json；A02 生成 F05/F06。每个点关联完整任务/约束规模和 source hash。

验证手工计时数组的 nearest-rank、冷启动分离、失败 attempt 计入、直方图上界标签和缺样拒绝。开发 smoke 至少覆盖一组共同任务、2/3层开销及观测开关；正式全矩阵仍是后续执行任务。未经测量不能填“零分配”或“1 kHz 保证”。

## 实现入口与开发验收合同（2026-09-10）

真实执行入口为 `apps/study_e09/` 的独立 `mcl_study_e09`，直接调用
`KinematicsSolver`、`HierarchicalKinematicsSolver` 和 vendored PlaCo。
`production-static` 没有修改。`prepare.py` 生成三个各 20 帧的 frozen snapshot
输入及 `experiment.v2` 声明；当前矩阵为 **1800 cells，477 可执行、1323 明确不可用**。
不可用 cell 保留原因：TargetSolve HQP、PlaCo HQP/ProxQP、无原生 warm-start
开关的 backend、当前 physical workload 没有对应语义层。额外 14/20 active-DOF
是同一 R1 的实际 active joints 变化，没有重复行充当 physical workload。

每个方法的 `regularization=1e-8` 为当前原生配置默认值；MCC 配置接口拒绝 0。
本声明没有通过给 HQP 目标额外加正则来令某 backend 成功。E06 parity 准入前，
全部性能 cell 只能作 descriptive 记录；不能直接解释成共同数学问题的 causal speedup。
TCP target 用完整 reference rotation 转成 EE target，独立核验仍在 TCP 上计算；
position-only workload 的这一语义需要纳入 E06 准入。

三种 observation 身份为 `minimal`、`full`、`cpp-new`。前两者的 Core API 均计算其
强制 native diagnostics；full 额外保留 task 明细，不能声称是 Core instrumentation
开关。`cpp-new` 在完整 IK adapter 区间观测当前线程的 replaceable C++
new/new[]/aligned new 次数和请求字节；**不包括直接 malloc/calloc/realloc、Eigen
直接 malloc 或其他线程**。此 arm 单独计时并保留开关身份；关闭时分配字段为 null。
装配阶段没有同口径 public hook，保留 null/reason；QP pass 时长直接保留 native 字段。
全部 attempt（包括 rejected）逐条 flush；cold/warmup/steady 不隐藏预热调用。

从 Lab 根目录准备、检查和开发 smoke：

```bash
python3 experiments/E09_solver_cost_and_scalability/prepare.py --model /workspace/models/r1.cos.urdf
python3 experiments/E09_solver_cost_and_scalability/test_measurements.py
python3 tools/mcc_placo_study/study.py --experiment E09 --dry-run --inventory-output runs/e09-matrix.json
python3 tools/mcc_placo_study/study.py --experiment E09 --case interior-ServoStep-w0-l1 --arm mcc-proxqp-warm1-minimal --repeat 0 --phase development --smoke --timeout 60 --output-root runs/e09-smoke
```

相同 case 的 `mcc-proxqp-warm1-cpp-new` 用于 allocation scope 开关核验。
2/3 层 smoke 选择 `interior-ServoStep-w2-l2` 与 `interior-ServoStep-w2-l3`。
全矩阵必须 dry-run，开发 smoke 不等于正式矩阵执行。
`verify.py --request <request.json>` 对单元核对原始 row 数、窗口、时长与缺样；
公共 verifier 独立复算 URDF FK/限位。计数器专项 CTest 为
`apps.mcl_study_e09_allocation`，手工 fixture 观测 3 次 / 226 bytes，并测试关闭/重置。

正式阶段在重新核对 RT、冻结物理核心/输入/配置/预算和 E06 准入后，以独立进程串行
执行（`--phase pilot` / `confirmatory`，不带 `--smoke`）。正式计划需要每进程
10000 measured calls 和 10 repeats。正式实验、A02、跨运行统计、图表与论文均 deferred。

## 本次已完成的开发验证记录

2026-09-10，匹配安装产物 `/workspace/install/algorithm/bin/mcl_study_e09`
完成 **14 个真实执行单元并通过独立核验，另 2 个 PlaCo HQP 单元按声明 unavailable**。
每可执行单元 11 个真实请求（cold 1、warmup 2、steady 8），共 154 attempts。
证据根目录为：

`runs/mcc_placo_study/development_acceptance/smoke/20260910T110024.558252Z-f60fca063b/`

选择矩阵的实际 CLI 形状如下；每单元的精确 argv、cwd、20 s timeout 和 request
在对应 `units/.../command.json`，源/运行库/输入哈希见该 run 的 inventories。

```bash
python3 tools/mcc_placo_study/study.py --experiment E09 \
  --case interior-ServoStep-w0-l1 --case interior-TargetSolve-w0-l1 \
  --case interior-ServoStep-w2-l2 --case interior-ServoStep-w2-l3 \
  --arm mcc-proxqp-warm1-minimal --arm mcc-proxqp-warm1-full \
  --arm mcc-proxqp-warm1-cpp-new --arm placo-eiquadprog-warm0-minimal \
  --repeat 0 --phase development --smoke --timeout 20 \
  --output-root runs/mcc_placo_study/development_acceptance/smoke
```

原始记录确认 TargetSolve 不是零迭代空调用：MCC 每次 1 次迭代，PlaCo 每次 2 次后
`no-progress`，其 **feasible-suboptimal** 原样保留，不能因进程完成就称目标收敛。
HQP 两层/三层分别实际请求并完成 2/3 passes。每个 MCC case 在 minimal/full/cpp-new
三种观测条件下 q 与 native status/disposition/quality 相同（本开发样本 q 差为 0）；
这核验观测链，不产生速度或算法优劣结论。C++ allocation scope 实际观测到正计数
（例如 weighted ServoStep 每调用 48–51 次），关闭字段为 null；计数范围不扩展到 malloc。

纯测量测试 **5/5** 通过；`apps.mcl_study_e09_allocation` CTest 通过。
完整 study gate **24/24** 通过的日志在同 acceptance 根目录，另有 workspace baseline
既存 TUI projection 回归失败记录；没有改动 production-static。

首轮 `20260910T105501.188102Z-f60fca063b` 的 **14 个 constructor crash 与 2 个
unavailable** 全部保留。原因是本 app 误读 `input.model.path`，canonical artifact
合同实际字段是 `locator`；修复字段读取后重跑，未改变 solver 配置、预算、容差或算法。
首轮证据不覆盖、不删除，也不用于代替后续有效执行结果。

代码与上述开发子集已验收；正式运行还缺 RT/同类型物理核心确认、显式 CPU ID、
配置/输入/阈值及 E06 parity 冻结。正式全矩阵未执行；分析、图表与论文 deferred。

补充 Eiquadprog warm0 weighted/HQP3 两单元已实际 completed/validated：`20260910T112019.371547Z-a59f0f09c4`。完整命令与追加证据见 [最终开发验收](../../docs/mcc_placo_study/DEVELOPMENT_ACCEPTANCE.md)。
