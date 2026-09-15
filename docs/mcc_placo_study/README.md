# MCC 框架与 PlaCo 对照研究

> 更新日期：2026-09-14

论文定位为 **MCC: A Practical Framework for Hierarchical Whole-Body Kinematic Control**。
贡献涵盖 Core 计算和参考应用架构；Practical 是待验证性质，不表示已完成硬件部署。

## 当前状态

| 项目 | 状态与范围 |
|---|---|
| E06–E12 | 现役声明使用可复用公开 app，已做开发接口/迁移验证；不表示正式实验完成 |
| A01/A02 | 已完成固定非 RT 历史批次的探索性分析及定向补证；结论仅适用于原方法和原数据 |
| 新应用方法 | 不继承旧方程准入、外部接受策略或耗时优势；需重新准入和冻结 |
| A03 | deferred |
| 正式 RT 计时与独立保留集 | 等待平台、资源、输入与研究准入；本 cleanup 不执行 |
| paper | 框架与论点计划保留，正文与正式统计工作 deferred |

## 阅读与合同

1. [审阅与 gap](REVIEW.md)、[研究协议](PROTOCOL.md)：数学语义、方法与准入。
2. [数据分析合同](DATA_ANALYSIS.md)：时钟、统计单位、来源与评价。
3. [公开执行接口](../../tools/app_execution/README.md)、各 Exx README：当前执行方式。
4. [A01](../../analyses/A01_priority_and_robustness/README.md)、[A02](../../analyses/A02_runtime_and_scheduling/README.md)、[A03](../../analyses/A03_end_to_end_and_recorded_motion/README.md)：分析归属。
5. [论文框架](../../paper/PAPER_FRAMEWORK.md)、[论点证据](../../paper/CLAIM_EVIDENCE_MATRIX.md)、[图表计划](../../paper/FIGURE_TABLE_PLAN.md)。

| 实验 | 问题 | 分析 |
|---|---|---|
| [E06](../../experiments/E06_solver_semantic_parity/README.md) | 方程、模式与接受条件的可比性 | A01 |
| [E07](../../experiments/E07_hqp_priority_and_redundancy/README.md) | 优先级与冗余利用 | A01 |
| [E08](../../experiments/E08_constraints_scaling_and_degeneracy/README.md) | 约束、缩放与困难状态 | A01 |
| [E09](../../experiments/E09_solver_cost_and_scalability/README.md) | 求解成本与规模 | A02 |
| [E10](../../experiments/E10_multirate_scheduling_and_coupling/README.md) | 多频率、调度与耦合 | A02 |
| [E11](../../experiments/E11_planning_ik_otg_error_propagation/README.md) | 规划/IK/OTG 误差传播 | A03 |
| [E12](../../experiments/E12_recorded_motion_holdout/README.md) | 录制会话与独立保留验证 | A03 |

旧 study/campaign/补证执行器已退役，A01/A02 需要的离线读取、指标与独立核验保留。
历史命令和阶段验收见 [归档](../archive/README.md)，来源缺项见 [本地证据索引](../archive/local_evidence.md)。
失败、能力缺项和来源不足保留；核验通过不改写原生失败，cleanup 不晋升或发布结果。
