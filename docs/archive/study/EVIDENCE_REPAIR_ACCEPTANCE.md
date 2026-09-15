# E06–E10 定向补证开发验收

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

验收日期：2026-09-12。本文记录代码、开发补跑和证据充分性三个不同层面的状态。独立构建、定向补跑和核验修订已完成；正式 RT 实验未开展。A01/A02 全来源分析及仅表重绘均已闭合；原始实验、既有分析和第一次失败分析全部保留。

## 状态与来源边界

| 层面 | 当前结论 | 限制 |
|---|---|---|
| 代码与开发验证 | 公共补证合同、独立安装、原始证据采集、A01/A02 修订、E08 前向 verifier v2 已实现；82 项 C++、266 项 Python 测试通过 | 通过测试不代表候选算法在全部困难输入上可行 |
| 新增定向补跑 | `supplement02` 全部 308 项已处理并闭合：300 项完成且核验通过，8 项执行或核验失败 | 全部为非 RT 开发运行；失败保留 |
| 既有数据来源 | `nonrt_fresh16_20260911T080119Z` 的 E06–E10 共 6,472 项独立保留 | 不与补跑合并计算成功率，不回写旧核验 |
| A01/A02 分析程序 | 全来源 dry-run、完整分析与仅表重绘均通过；入口见下方 | 图表、比值、准入结论必须以闭合分析 manifest 为准 |
| 正式运行就绪 | 尚不能确认 | RT 平台、调度、核资源、频率与正式冻结条件未满足完整证据要求 |
| 正式实验完成 | 否 | 不将此次开发补跑或异常 smoke 晋升为正式实验 |
| 科学证据充分性 | 部分补齐，部分明确失败或仍受适用范围限制 | 无总体优胜排名，无统计显著性或 RT 结论 |

旧 E06–E10 的 6,472 项经固定 manifest 逐项对账为 **2,442 项完成且核验通过、17 项执行或核验失败、4,013 项不可运行**。17 项包括 completed/failed 15、crashed/failed 1、failed/failed 1。这是单元状态计数，不是求解成功率。旧 campaign 状态文件还覆盖其他实验，总数不同，不能拿其全 campaign 分母替换此处的 E06–E10 范围。[原 execution index](../../../runs/mcc_placo_study/nonrt_fresh16_20260911T080119Z/execution_index.json)

`production-static` 保留原身份；统一接受策略的公平对照另立 `common-servo-v1` 方法身份。算法配置继续 app-local；本轮未放宽 hard tolerance、修复候选算法、修改旧输入或静默接纳失败迭代。没有提交、推送或发布。

## 新增 308 项矩阵与执行验收

| 阶段 | required 单元 | 完成且核验通过 | 执行或核验失败 | 原始 run |
|---|---:|---:|---:|---|
| E06 公共接受合同与行审计 | 12 | 12 | 0 | [E06 manifest](../../../experiments/E06_solver_semantic_parity/runs/20260912T080829.024766Z-e167cac5c3/E06.manifest.json) |
| E08 几何、候选和初始状态补证 | 56 | 48 | 8 | [E08 manifest](../../../experiments/E08_constraints_scaling_and_degeneracy/runs/20260912T080834.447430Z-bc4e890adb/E08.manifest.json) |
| E09 比较问题与计时前审计 | 24 | 24 | 0 | [audit manifest](../../../experiments/E09_solver_cost_and_scalability/runs/20260912T081303.403736Z-e800d6a998/E09.manifest.json) |
| E09 成本计时 | 216 | 216 | 0 | [timing manifest](../../../experiments/E09_solver_cost_and_scalability/runs/20260912T081314.450250Z-cc01f5c7dd/E09.manifest.json) |
| 合计 | **308** | **300** | **8** | [闭合 execution index](../../../runs/mcc_placo_study/evidence_repair_20260912/supplement02/execution_index.json) |

E08 的 8 项为 7 项 completed/failed 和 1 项 crashed/failed。E07 本轮使用既有轨迹做分析补证，没有新增 native 实验阶段。E10 只有另立的观察容量 smoke，不在 308 项内。

[完整矩阵 dry-run](../../../runs/mcc_placo_study/evidence_repair_20260912/matrix-dry-run-full.json) 声明 `read_only=true`、`required_units=308`。[执行区间审计](../../../runs/mcc_placo_study/evidence_repair_20260912/execution_audit.json) 对账 308 个 native 进程与 308 个核验任务：native 进程之间无重叠，native 与核验区间无重叠；核验阶段峰值并发 16。该记录证明本批实际串行 benchmark 与分阶段并行核验，不证明机器没有其他系统负载。

19 份受保护来源经闭合审计确认未变。补跑所用 1,464 份源码、构建、输入和 runtime 文件按原哈希归档，[frozen closure manifest](../../../runs/mcc_placo_study/evidence_repair_20260912/supplement02/frozen_closure/manifest.json) 记录原位置、快照位置和各自 SHA-256。前向 verifier v2 是闭合后另立版本；原 308 项声明、raw 和 validation 不因核验修订而改写。

## 构建、测试与保留的开发失败

独立安装前缀为 `/workspace/install/evidence-repair-20260912`，Lab 构建目录为 `/workspace/build/evidence-repair-20260912/motion_control_lab`。每个 run 的环境记录保存实际 executable、动态库路径和 SHA-256。原生产安装身份不被补证替换。[实际构建、执行与复现命令](../../../runs/mcc_placo_study/evidence_repair_20260912/commands.md)

| 验证 | 结果 | 日志 |
|---|---|---|
| Core kinematics assembler | 15 passed | [assembler](../../../runs/mcc_placo_study/evidence_repair_20260912/test_motion_control_core_kinematics_assembler.log) |
| Core hierarchical kinematics solver | 18 passed | [HKS](../../../runs/mcc_placo_study/evidence_repair_20260912/test_motion_control_core_hierarchical_kinematics_solver.log) |
| Core weighted kinematics solver | 49 passed | [weighted](../../../runs/mcc_placo_study/evidence_repair_20260912/test_motion_control_core_kinematics_solver.log) |
| 最终 Python 集成测试 | **266 passed，1 warning，28.46 s** | [最终日志](../../../runs/mcc_placo_study/evidence_repair_20260912/integrated-final-tests.log) |

C++ 三个测试程序退出码均为 0，[实际执行参数](../../../runs/mcc_placo_study/evidence_repair_20260912/core-tests.json) 可复查。Python 的唯一 warning 为 Matplotlib Axes3D 不可用；本轮 SVG/PDF/PNG 均为 2D，不据此声称支持 3D。较早的 264 项日志不是最终总数。

开发过程中发生过的 E06 JSON bool 序列化失败、构建参数被包元数据覆盖、GitHub TLS 下载失败及冻结检查回归失败均保留。`smoke01` 和 `supplement01` 不被覆盖或算作成功的主批次；`supplement01` 仅准备，实际完整补跑为 `supplement02`。冻结检查修订为 native 执行前重哈希运行库，保留失败测试及后续通过日志。[smoke01](../../../runs/mcc_placo_study/evidence_repair_20260912/smoke01/smoke_index.json)、[修复后的 smoke02](../../../runs/mcc_placo_study/evidence_repair_20260912/smoke02/smoke_index.json)、[冻结检查原失败](../../../runs/mcc_placo_study/evidence_repair_20260912/freeze-integration-tests.log)、[后续通过](../../../runs/mcc_placo_study/evidence_repair_20260912/freeze-integration-tests-fixed.log)

E10 的 normal-capacity smoke 通过，cap8 overflow smoke 保留 failed。容量不足导致的数据缺失必须继续表现为 overflow/不可估计，不能把丢失尾部当作低延迟。[E10 smoke inventory](../../../experiments/E10_multirate_scheduling_and_coupling/runs/20260912T080507.985492Z-3943d6a706/inventory.json)

## E08 异常归因与修订边界

旧 12 项缩放异常必须从旧 raw 自身解释，新 56 项单独分析。`e08-final-scale-reanalysis.v1` 记录原始核验残差、primary optimum scale、同次选中 pass 的精确 final scale、纠正后的残差、commit/disposition、原始行号与文件哈希。旧轨迹未记录原生完整矩阵，明确标为 `unavailable-original-trace`；原生最坏分量取自旧 `native_constraints`，同时保留独立几何和固定差分 `1e-5/1e-6/1e-7` 核对。

| 原异常类别 | 原始证据的诊断 | 保留的限制 |
|---|---|---|
| 4 个 HKS3 单元：左右 offset 0.05 × shared/per-arm | 每单元原有 2 条缩放失败，其中 1 条 primary/final scale 操作数假阳性，另 1 条最终已提交方程仍超 `1e-7` | 合计 4 条真实超差、4 条操作数假阳性；不把整个失败单元改成通过 |
| 8 个 weighted 单元：左右 offset 0.15/0.3 × shared/per-arm | 27,152 条旧失败检查全部对应原生拒绝后的 HOLD 返回；零占位向量不是原生优化失败迭代 | hard-feasibility rejection 是真实拒绝行为；不能把错误方程检查消除解释为求解成功 |
| 3 个 MCC initial-infeasible 单元 | 输入已超过关节上界 0.01 rad，拒绝后保持原输入；没有提交新的越界状态 | 旧公共位置核验失败保留；初始状态问题与候选/提交问题分开 |
| 1 个 PlaCo initial-infeasible 单元 | 原生 `placo::problem::QPError`，无完整候选 | 崩溃、stderr 与 begin 证据保留，无 fallback；不猜测不存在的解 |

旧 12 项缩放异常共 27,160 条失败检查，上表为该原始检查集合的重算，不能代替全矩阵质量统计。新 56 项中四个 canonical Q0 正控制均通过；它们是独立输入，不是裁剪或修复旧 initial-infeasible 输入。

已确认的核验缺陷是把 HKS 的 `task_scales.weighted_progress_scale`（primary optimum）与最终候选混用；最终 scale 在同次 `native_constraints` 的 `progress-*` 行中。绝对 drift 没有符号，不能据此恢复 final scale。前向 `e08-verifier.v2` 已采用精确选中 pass 与 final scale，并拒绝将未捕获的失败候选伪装为零向量。旧 v1 文件及全部闭合核验产物保持不变。

独立 v2 对已有 HKS3 left/shared 完整 6,000-call raw 重核，记录完整观察，原生/几何真实已提交超差各 1，退出码 1：

| attempt_sequence | primary optimum scale | final selected scale | 最终原生最大残差 | 原阈值判定 |
|---:|---:|---:|---:|---|
| 1000 | 0.9999999763738117 | 0.9999999623247314 | 9.951406023528397e-8 | 通过；原操作数假阳性 |
| 4000 | 0.9999999977750156 | 0.9999999716301242 | 1.0397834428882424e-7 | 失败；真实已提交超差 |

[v2 独立重核 summary](../../../runs/mcc_placo_study/evidence_repair_20260912/verifier_v2_recheck/e08_evidence_summary.json) 同时绑定原 request/raw、v2 脚本及几何计算依赖的哈希。退出码 1 表示保留真实候选失败，不是分析程序异常。真实超差仍需未来单独修复候选约束验收路径；本轮没有修改算法行为，也未改变 `1e-7` 阈值。

## 三个有界复现探针

这些 probe 都标 `smoke=true`，与旧 6,472 项和新 308 项分开记账。结果包括反例，不能只展示能复现的探针。

| 探针 | 结果 | 可支持的解释 | 证据 |
|---|---|---|---|
| HKS3：提取原 seq4000 状态，fresh solver 单调用 | exit 0，未复现失败 | 仅有状态不足以重现原运行历史；不能据此否认旧失败 | [execution](../../../runs/mcc_placo_study/evidence_repair_20260912/reproduce_hqp3/execution.json)、[definition](../../../runs/mcc_placo_study/evidence_repair_20260912/reproduce_hqp3/definition.json) |
| HKS3：原始输入前缀 4,001 调用 | exit 1，seq4000 精确复现 | left-position 残差 `1.0397834428882424e-7`，超差量 `3.978344288824247e-9`，仍 committed；运行历史有关，但未隔离全部内部原因 | [execution](../../../runs/mcc_placo_study/evidence_repair_20260912/reproduce_hqp3_prefix/execution.json)、[definition](../../../runs/mcc_placo_study/evidence_repair_20260912/reproduce_hqp3_prefix/definition.json) |
| PlaCo：原 initial-infeasible 单调用 | exit 1，复现原生 QPError 崩溃 | 原始输入即可触发异常；不构造替代解 | [execution](../../../runs/mcc_placo_study/evidence_repair_20260912/reproduce_placo/execution.json)、[definition](../../../runs/mcc_placo_study/evidence_repair_20260912/reproduce_placo/definition.json) |

## E06/E07 的新增证据与剩余边界

E06 新共同 ServoStep 的 12 单元实际方程、外部接受合同和观测完整性通过，六组同 case 对照准入。仅支持这些完整方法和共同子集的比较；旧方法接受策略差异、TargetSolve 和 production-static 的准入边界仍保留。

E07 全部 565 单元完整重算，3,755 个任务成本行中 3,723 项可测、32 项未声明任务保留不适用。2,670 个原始/加权配对结果包括 692 项完整非零参考、1,936 项零参考（绝对差保留）、32 项任务不适用、10 项 fixture 缺少 primary-only。全部 292 个 snapshot 与 272 条演化轨迹的 0/1/3 s 状态组成 1,108 个参考；917 个获数值证书，191 个因独立驻点/互补性证书不足保持 unavailable。

[独立最终核对](../../../runs/mcc_placo_study/evidence_repair_20260912/E07-independent-final-audit/result.json) 不调用优化器或被测 app，从原 raw 重建任务与硬界，复算 1,443 层 KKT；最大字段差 `5.55e-17`，未发现错误授证。SciPy 内部 TRF 数值警告保留在完整运行日志；所有最终参考值和乘子有限，警告不被等同于参考有效或无效。

旧 Core 两份关键策略源码已从冻结 git revision 恢复，哈希与当前源码一致；历史运行库哈希存在，但编译时源码到库的完整证明仍缺失。因此这些参考证书适用于明确声明的离线数学问题，不能据目标差裁定旧原生求解的最优性。这一缺项没有被重新构建的库替代。

## A01/A02 最终分析入口

两次完整分析依次完成。A02 固定消费 A01 计算 run 的准入表；A01 最终阅读版仅调整 F04 显示刻度，不改变准入或数值表。

| 分析 | 全来源分析 | 最终阅读入口 | 仅表重绘核对 |
|---|---|---|---|
| A01 | [20260912T084208.697350Z-6be7f5dfbe](../../../analyses/A01_priority_and_robustness/runs/20260912T084208.697350Z-6be7f5dfbe/manifest.json) completed | [中文报告](../../../analyses/A01_priority_and_robustness/runs/20260912T085612.471568Z-6be7f5dfbe/report.md) · [图表索引](../../../analyses/A01_priority_and_robustness/runs/20260912T085612.471568Z-6be7f5dfbe/figures/index.json) | 39 张 CSV 逐字节一致；[重绘 manifest](../../../analyses/A01_priority_and_robustness/runs/20260912T085612.471568Z-6be7f5dfbe/manifest.json) |
| A02 | [20260912T085515.750881Z-6a689ade69](../../../analyses/A02_runtime_and_scheduling/runs/20260912T085515.750881Z-6a689ade69/manifest.json) completed | [中文报告](../../../analyses/A02_runtime_and_scheduling/runs/20260912T090018.813844Z-6a689ade69/report.md) · [图表索引](../../../analyses/A02_runtime_and_scheduling/runs/20260912T090018.813844Z-6a689ade69/figures/index.json) | 23 张 CSV 逐字节一致；[重绘 manifest](../../../analyses/A02_runtime_and_scheduling/runs/20260912T090018.813844Z-6a689ade69/manifest.json) |

计算与重绘 run 的 manifest SHA-256（顺序 A01 计算、A01 重绘、A02 计算、A02 重绘）：

- `20260912T084208.697350Z-6be7f5dfbe`：`ea053bd7c45233948259bdf5ea6ba6b600759b8404bf0640d695394f7bb72412`。
- `20260912T085612.471568Z-6be7f5dfbe`：`6e398906f0c67c6ca3456f608abdda3e3857308e8a1139c950d72661e21be9d0`。
- `20260912T085515.750881Z-6a689ade69`：`85c93d4d88a6ce0228f2f75905a39ccabef9b33e7c8164f4195fe8b2dcaef455`。
- `20260912T090018.813844Z-6a689ade69`：`14155f508cec1cd447db88a16a90bc3e288419e76eb7b4eecf34090924d796df`。

[最终来源/产出/链接对账](../../../runs/mcc_placo_study/evidence_repair_20260912/analysis-delivery-audit.json) 检查两组计算与重绘的全部 manifest 产出，报告无失效本地链接；旧 6,472 与新 308 项分别满足计数合同，19 份受保护旧记录哈希未变。每次完整分析已校验全部旧 raw/validation 来源；重绘只读取已有分析表。

A01 dry-run 校验 43,107 个 artifact，覆盖旧 879 + 新 68；A02 校验 62,761 个 artifact，覆盖旧 5,593 + 新 240。A01 的 32 张图和 A02 的图表均保留 CSV 源表、SVG/PDF 与 PNG；F02/F03/F04/F05 代表图经过目视检查。最初失败 A01 `20260912T082931.660488Z-6be7f5dfbe` 保留，空软目标矩阵修复后新建完整 run。

E09 新增 24 项全 snapshot 审计、216 项计时全部观测完整，108 对共同方法配对准入；3,969 项旧 E09 能力缺项仍保留。E10 cap8 为预期故障，正常性能图排除它，原失败计数不变。参考优化器不可用、旧构建编译证明缺失和 TargetSolve 对齐继续明确列为缺证据。

[A02 独立最终核验](../../../runs/mcc_placo_study/evidence_repair_20260912/A02-independent-final-audit/result.json) 重读全部 185 个 E10 有时序单元：计划 6,114,264、保留 6,114,207、skipped 2,779、已观测 miss 6,278、unknown 57、not-run 0；逐 worker 的计划分母、缺样尾部分位数 unavailable 与比例下界全部吻合。这些是该固定非 RT 窗口的观测计数，不代表实时保证。108 个 E09 比值的分子/分母与表中原始 P50 逐行一致。

[完整交付索引](../../../runs/mcc_placo_study/evidence_repair_20260912/README.md) · [25 项逐问题结论/哈希/定位](../../../runs/mcc_placo_study/evidence_repair_20260912/issue_register.csv) · [可复现命令](../../../runs/mcc_placo_study/evidence_repair_20260912/commands.md)。每个问题分别标为证据已补齐、真实算法问题、预期故障、能力不支持或仍缺证据；缺项不等于分析程序失败。

## 环境缺项与正式后续

记录到的环境为容器、`6.8.0-139-generic` / `PREEMPT_DYNAMIC`，未发现 `/sys/kernel/realtime`，`sched_rt_runtime_us=950000`。运行环境快照中调度器为 0，CPU 0–23 可见，governor 为 powersave，core type 未获确认。主机 RT、核类型、频率和线程调度的完整确认仍缺失。[实际运行环境快照](../../../experiments/E06_solver_semantic_parity/runs/20260912T080829.024766Z-e167cac5c3/units/E06/model_mapping/mcc-weighted-common-servo-v1/0/environment.json)

这些观测不能证明 RT 能力，也不能仅靠重启就把既有非 RT 数据变成正式证据。本轮没有重启、改内核或接触真实设备。验收时主 agent 记录磁盘约 89 GiB 可用；这只是当时容量，不是足够容纳任意正式矩阵的保证。

正式实验另行启动，顺序为：

1. 使用已冻结的 A01/A02 本轮分析，保留明确失败和不准入项；对真实 HKS3 已提交超差的后续算法修复另开身份和验证任务，不回写此轮证据。
2. 在宿主机核验 RT 内核、权限、实际线程调度、绑核、核类型、频率与后台负载条件；记录全部实际值，不能以容器内可见参数替代宿主确认。
3. 冻结正式输入、模型、配置、方法/接受合同、观测设置、二进制及依赖哈希；先核对容量与完整矩阵 dry-run，再运行有界 smoke。
4. 按协议串行运行 benchmark，保持 native 与离线核验/重计算不重叠；运行结束后再次对账 required 单元、失败、缺失、原始证据和哈希。
5. 正式执行需要另立 run 与声明，不覆盖旧批次、不自动晋升 `results/`。A03、跨运行科研统计、显著性推断及论文工作继续 **deferred**。
