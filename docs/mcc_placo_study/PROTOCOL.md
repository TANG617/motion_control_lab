# 统一研究协议

状态：`planned`。版本：`study-protocol.v1`。这是 E06–E12 的数学、方法和指标定义权威入口；数据与统计规则见 [DATA_ANALYSIS](DATA_ANALYSIS.md)。下面的数字是协议选择或开发矩阵，均不是测量结果。

研究定位为 **MCC: A Practical Framework for Hierarchical Whole-Body Kinematic Control**，以 MCC 框架设计为主线，用受控 PlaCo 对比和 MCC 消融检验具体性质。Core 计算、应用调度和运动学反馈模型必须分别归属；本协议不产生硬件部署结论。当前实现阶段覆盖实验、采集和独立正确性核验；A01–A03、跨 run 统计及论文工具/撰写在实验 campaign 结束后进行。下面预登记的指标、统计和接受规则仍约束实验，不能因分析延期而推迟到看到结果后再选择。

## 1. 研究对象与方法身份

| method_id | 定义 | 允许的比较 |
|---|---|---|
| production-static | 现有冻结 PlaCo TargetSolve 基线，保持原任务、mask、终止条件与 100 Hz | 历史迁移链路，不充当所有公平比较的唯一基线 |
| placo-controlled | vendored PlaCo，按实验对齐后的任务与约束；独立记录 TargetSolve/ServoStep | 与相同目标/约束的 MCC 配对 |
| mcc-weighted | 普通 KinematicsSolver 的受控加权 QP | 归因 HQP 与普通 QP 的差异 |
| mcc-hqp-2 | 两个语义层；位置优先，朝向/posture/link4 同层折中 | 现 app 语义的受控实例 |
| mcc-hqp-3 | 位置 > 朝向 > posture/link4；实验专用明确配置 | 层级表达实验，不能冒称现 app 默认 |
| independent-reference | 小规模解析解/主动集枚举及离线数学复算 | 核验语义，不是实时性能竞争者 |

method、execution mode、task profile、backend、scheduler、观测政策、参数 hash 共同确定 arm。所有跨方法数字必须绑定 E06 parity 审计结果。

固定 R1 物理模型与各比较单元的 full/active joint names、URDF/mesh hashes、root frame、TCP offset、初始 q/qdot、units。16 active joints 为生产对齐起点；扩大自由度另设 factor。可以使用不同内部变量表示，但须报告 floating-base mask、结构缩减、额外 scale/slack 的实际维度，不假称相同矩阵规模。

输入到 EE/TCP 的变换、旋转误差、task gain、residual normalization、enforcement、scale grouping、数值正则、软约束、收敛/接受政策均进入对齐表。不能仅因 YAML 中存在某参数就断言已生效，需对应实际任务注册及 resolved config。

## 2. 数学与可主张性质

ServoStep 求解 joint velocity v，以当前测量状态线性化，任务期望速度为 feedforward + Kp·error，输出按固定 solver period 积分。TargetSolve 求解无时间语义的 joint delta，并可能多次重线性化；其耗时与成功条件单独报告，不能直接用一次 HQP tick 与完整 TargetSolve 总时长主张库级加速。

普通 soft task 的语义为归一化残差的 weighted squared-L2；Hard 集在所有 HQP 层生效。层级结果在声明容差带内保持上层语义，不锁定整个 v 向量，不把正则化代价当成任务目标。有限容差允许 drift，不能宣称精确零空间或严格零误差。

对 scaled task，保存同 tick 的 feasible reference、baseline task velocity、desired velocity、scale 及实际 equation。MCC 的 finite weighted progress 不是独立的 exact max-scale 算法。各 arm 按自己的显式缩放方程核验，跨 arm 的共同任务效果用 FK/error/progress 比较；shared scale 与 per-arm scale 是独立因素，不混入 HQP 的唯一因果比较。

HQP 核验分三层：

1. 2–4 个变量的小规模凸问题，用独立解析/主动集枚举比较目标值、可行性、层级保持；包括唯一解、非唯一解、秩亏、冲突、不活动/活动约束。多个关节解同样最优时比较目标和约束，不比较向量完全相同。
2. R1 snapshot 从同一冻结 q、task、hard bounds 独立构造 FK/Jacobian/residual；不从待核验 diagnostics 直接复制结论。共用 Pinocchio 属于模型依赖共性，需声明不能排除共同模型错误。
3. evolving trajectory 独立检查积分后/OTG 后 FK 和实际约束。单步线性化保持不能替代这一层。

若使用 Core 诊断导出，默认关闭，仅传递事实，不改变 warm start、目标、约束或接受政策；关闭时做行为等价检查。所有参考计算位于计时区间外。

## 3. 统一指标字典

每项 scalar 指标携带 unit、版本、window、side/joint/task/worker 和归约方式。以下 family 的具体展开由实验声明固定；不同物理量不得未经归一化合成一个最大值。

| metric_id / family | 定义、单位与方向 |
|---|---|
| position_error | 同时刻参考点与独立 FK 点的欧氏距离，m，越小越好；左右臂分别统计 |
| orientation_error | 相对旋转 R_refᵀ R_fk 的主旋转角 [0,π]，rad，越小越好；不使用欧拉角相减 |
| preservation_drift | 同 tick 同矩阵，最终残差减该层完成时残差的逐分量绝对值；原生速度单位。scaled equation 与 scale 的保持分别核验 |
| preservation_ratio | max_i(drift_i / declared_tolerance_i)，无量纲；≤1 表示声明带内，正容差强制显式 |
| secondary_gain | (cost_primary_only − cost_with_secondary) / cost_primary_only；无量纲。分母为 0 时相对值 not-applicable，仍报告绝对差 |
| reference_objective_gap | 小问题 candidate objective − independent optimum，按层和原生代价单位报告 |
| hard_violation | 每个物理约束的 max(lower−value, value−upper, 0)；分别保留位置/速度/加速度/制动类别和原生单位 |
| hard_violation_excess | max(hard_violation − accepted_tolerance, 0)，原生单位；任何正值记为核验失败，不用新 epsilon 放宽 |
| task_scale | 每个 scale group 的原生值及 weighted objective；无量纲，不自动代表真实运动进度 |
| progress / stall_duration | 目标误差降低量和沿预定运动方向的位移；m/rad。stall 用声明的进度阈值和连续窗口定义，s |
| recovery_time | 困难输入解除到连续满足声明跟踪阈值/无拒绝窗口的时间，s；未恢复保留 censored 与观测时长 |
| ik_call_time | 适配后的完整 IK 请求、求解、结果提取时长，ms；同样的 FK/核验边界在 arm 间对齐 |
| assembly_time / qp_time | 数值装配、各 pass backend 时间，ms；只有同口径 instrumentation 才跨方法比较 |
| release_to_finish | 完成时间 − 计划 release 时间，ms；包含唤醒和执行延迟 |
| deadline_miss_rate | 未在该 release 的 deadline 前产生规定结果的次数 / 计划 release 总数；skipped release 计 miss |
| consecutive_misses | 计划 release 序列中最长连续 miss 长度，ticks，同时报告持续时间 ms |
| proposal_age / state_age | 消费时刻 − proposal 产生时刻、消费时刻 − proposal 所用状态采集时刻，ms；两者分开 |
| cpu_time | worker CPU time 的和，s，并报告 wall duration、线程数和核配置；线程 CPU 时间不可等同 wall time |
| allocation_count | 明确观测条件内动态分配次数/字节，不能把观测关闭时的缺失写成 0 |
| joint_vaj | q/v/a/j 的原生输出与注明方式的派生估计，rad、rad/s、rad/s²、rad/s³；逐关节统计 |
| target_execution_lag | 预定事件响应或声明的相关估计所得 lag，s；显式窗口和符号，不能事后平移输出改善主精度指标 |
| full_quality_rate | 达到所声明全部任务/层级质量的 attempt 数 / 全部真实 attempt 数；另报 scheduled coverage |
| valid_completion_rate | 满足完整动作窗口、质量和 guardrail 的 recording 数 / 全部 required recording 数 |
| source_coverage | source produced/paired/consumed/dropped，分别保留分母；不得把未消费帧隐去 |

位置/姿态误差和约束违反报告时间加权 RMS、P95、最大值及越阈持续时间；固定周期权重为 dt，非均匀时按样本持有区间长度。数值精度曲线不做事后 lag 补偿。性能样本分位数按调用样本、不是按时长加权；nearest-rank 定义固定，详见 DATA_ANALYSIS。

## 4. 状态与失败

每个 attempt 保存 operation Status、native QP status、disposition、solution_quality、selected_priority、highest_completed_priority、requested/completed passes 和 committed。统一质量类别为 full、partial-hierarchy、feasible-suboptimal、rejected、not-run；HOLD 是执行状态，crashed/interrupted 是单元状态，不能强塞入同一个互斥枚举。

full 要求当前 arm 声明的全部层级或 TargetSolve 收敛条件完成并通过独立 guardrail。普通单步 QP 的数值 full 与动作 target reached 分开。保持旧命令的 tick 不能当成新解，但该时段的执行跟踪误差仍计入演化实验。run completed、candidate accepted 与 scientific accepted 分开。

实验子进程遇到算法异常按原生政策失败，外围 orchestrator 保存部分证据并按声明继续下一单元，不增加 app 内吞异常或自动修复。required arm 缺失时保留 unavailable pair，并报告覆盖率；禁止删掉失败后排名。

## 5. 执行条件与冻结

默认开发 profile：固定周期 1 ms 的 ServoStep、100 Hz target source、初始 qdot=0；这是实验起点，不覆盖 production-static 的 100 Hz/TargetSolve 合同。每项实验显式列出偏离。一次性 cold start 及首次真实请求计入 cold window，warm-up 次数和稳态窗口在正式前冻结，不能看曲线后裁剪。

phase 分为 development、pilot、confirmatory。开发 smoke 可缩小矩阵，必须记录 selection；正式 required matrix 不得自动退回 smoke。默认开发 seed 为 20260910。正式前冻结输入/分组 hash、搜索范围、每方法相同调参预算、指标及阈值、timing windows、重复次数和配对方式。

工程起点为每个可调方法最多 30 个预先登记候选，在 development/tuning 会话评估，按 guardrail→full-quality coverage→主要指标的既定次序筛选；无可行候选保留失败，不追加定向搜索以挽救某个方法。无需调参的方法不强制填满预算。任何扩展搜索产生新的 exploration version，不能访问保留集。

位置、姿态、stall、recovery、非劣效界等业务阈值没有现成统一权威值，不能由实现 agent 编造“机器人安全阈值”。必须在 pilot 结束的 preregistration 中有数值、来源/理由；缺失时拒绝 confirmatory，代码仍可完成。硬约束/保持容差直接来自冻结配置，不允许由数据反推放宽。

E09/E10 正式计时及 E12 保留验证只允许在本工作站的已确认 RT 环境。预检记录内核 RT 配置、boot identity、CPU 核类型、cpuset、affinity、调度政策、频率政策、容器/宿主边界和实际库。不能只凭 uname 名字或配置了 1 kHz 判通过。固定实际可用且同类型的物理核心；不在文档中猜 CPU 编号。

所有 timed benchmark 串行，停止并行构建、测试、分析和其他测量。最小计时记录与完整 telemetry 是分开的 observation arms。严格截止期运行与 monitor 压力诊断分开；monitor 结果不能算严格截止期验收。正式主张是给定条件下的实测行为，不是 WCET、调度可行性或稳定性证明。

## 6. 变更政策

参数、方法、任务或指标变更有新声明 hash；正式保留集看过后不能重新用于同一假设的未经披露调参。算法问题保存复现和诊断，原版本留存；本代码任务不授权语义修复。Lab 只交换版本化 artifact，不 import 根仓 Python 内部实现，也不要求 Core 依赖 Lab。

论文贡献与状态见 [CLAIM_EVIDENCE_MATRIX](../../paper/CLAIM_EVIDENCE_MATRIX.md)。
