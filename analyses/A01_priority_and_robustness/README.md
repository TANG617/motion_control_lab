# A01：优先级与鲁棒性证据分析

状态：`planned`。只消费已有 evidence，不运行 solver 或重新生成实验输入。

阶段：实验 campaign 结束并冻结来源与全单元状态清单后，再实现和运行本 Analysis。当前只保留评价协议及证据字段要求；实验代码阶段不创建本 Analysis 的声明、执行器、统计/图表或报告。开发所需逐单元独立核验属于 E06–E12，不触发此阶段。

来源：[E06](../../experiments/E06_solver_semantic_parity/README.md)、[E07](../../experiments/E07_hqp_priority_and_redundancy/README.md)、[E08](../../experiments/E08_constraints_scaling_and_degeneracy/README.md)。公共数学：[PROTOCOL](../../docs/mcc_placo_study/PROTOCOL.md)；统计/存储：[DATA_ANALYSIS](../../docs/mcc_placo_study/DATA_ANALYSIS.md)。负责 C1/C2，F02/F03/F04、T01。

## 问题与输入闸门

区分能力表达、数值实现正确性和具体场景的效果。分析声明固定每个 source run/result ID、manifest hash、metric version 和 phase，禁止 latest。未通过 E06 的条件不能混入因果比较；小规模 oracle、R1 snapshot、evolving 和 collision diagnostic 是不同 strata。

schema/hash/单位/关键参考缺失时，保存 validation failure，相关配对 unavailable。允许失败 run 用于失败分析，但不让缺失或被污染的数据支持正面效果。

## 配对与计算

以相同 input/model/task profile/window/repeat condition 配对各方法，primary-only 参考必须来自同 case。按 task 和 side 核验 preservation_ratio，并附原生速度单位 drift，不以一个混合单位总量替代。

逐 case 计算 secondary 绝对/相对收益、oracle objective gap、hard_violation_excess、progress、recovery/censored 状态和完整质量覆盖。按层、权重、困难类型聚合。比例为 0 的参考保留 not-applicable，相同最优目标下不同关节向量不是不一致。

先提供全部 required 单元质量/覆盖表，再提供完整配对条件下的效果表。合成 case/seed 为独立单位，bootstrap 方法和 seed 引用 DATA_ANALYSIS；没有独立重复的单一解析例只作确定性核验。

## 输出

- sources/inventory.json：冻结来源、声明/模型/版本及允许差异。
- evaluation/parity_table.csv、priority_checks.csv、paired_effects.csv、robustness_cells.csv、coverage.csv、validation.json。
- F02：条件与能力对齐；F03：保持、收益、权重/层级扫描；F04：困难窗口的约束/进度/恢复与失败。
- T01：方法能力与不能配对的原因；C1/C2 decision rows：supported、contradicted、inconclusive 的证据及适用范围。
- report.md/report.html：所有点链接到具体 case/run，负面条件和失败有同等导航入口。

输出路径和图形格式遵循 [图表合同](../../paper/FIGURE_TABLE_PLAN.md)。论文不从此报告自由摘取没有 claim ID 的 headline。

## 验收

用人工可核验 fixtures 检查越界 drift、零分母、多解、缺少高层参考、partial fallback、未恢复、missing pair 与哈希变化。结果可从相同来源和分析声明重建；不启动 app、修改 sources、随机挑成功窗口或用 diagnostics 自身断言代替独立核验。
