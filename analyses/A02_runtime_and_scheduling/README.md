# A02：运行成本与调度证据分析

状态：`planned`。只消费固定来源，不重新运行性能实验。

阶段：实验 campaign 结束并冻结来源与全单元状态清单后，再实现和运行本 Analysis。当前只保留评价协议及证据字段要求；实验代码阶段不创建本 Analysis 的声明、执行器、统计/图表或报告。开发所需逐单元独立核验属于 E06–E12，不触发此阶段。

来源：[E09](../../experiments/E09_solver_cost_and_scalability/README.md)、[E10](../../experiments/E10_multirate_scheduling_and_coupling/README.md)，并引用 E06 对齐证据。公共指标：[PROTOCOL](../../docs/mcc_placo_study/PROTOCOL.md)；统计：[DATA_ANALYSIS](../../docs/mcc_placo_study/DATA_ANALYSIS.md)。负责 C3/C4、F01/F05/F06/F07/F08、T03。

## 输入分层与预检

按平台/boot、核心类型和数量、execution mode、phase、scheduler、quality、观测配置、task workload、backend 分层。非 RT、monitor 压力、virtual schedule 与 confirmatory strict 证据不得混表排名。

preflight 核验 source hashes、计时边界、样本完整性、原始数组或 histogram 分辨率、计划 release 总量、skipped release 和提前退出后缀。缺样时保留原生计数，但缺失尾部统计不得伪造。

## 统计与归因

按每 run 计算 cold/steady 的 P50/P95/P99/max 和样本数；用 DATA_ANALYSIS 定义的 nearest-rank 或明确桶上界。不平均 P99 产生总体 P99。不通过删除 rejected/partial calls 改善完整时间分布；可以另外显示成功条件下的分布并标分母。

每 case/repeat 配对后比较完整 IK 成本、full-quality coverage 与 CPU time；backend、warm start、observability 分别成 factor。报告实测 elapsed 最大值及观测时长，不称 WCET。

E10 用 synchronous-full→synchronous-divided 衡量降频，用 divided→asynchronous-shared-core 观察异步结构，用 shared-core→two-core 观察额外资源。全部比较同时附质量和状态/参考年龄，不在输入不同或反馈模型不同的条件下计算纯调度 speedup。

deadline 分母为计划 releases；提前失败报告 time-to-failure 和窗口未完成，不能只在前缀内计算低 miss 后宣告合格。连续 miss 从完整 release 序列求；proposal age 与 captured state age 各自统计，并画与误差的关联，关联本身不是因果证明。

## 输出与图形

输出 timing_quantiles.csv、paired_runtime_effects.csv、deadline_coverage.csv、resource_tradeoffs.csv、observation_overhead.csv、validation.json 和 report.md/html。

- F01：源自冻结配置的线程/算法图，注明 diagram 非测量曲线。
- F05：完整时间分布、尾部和成本分解，质量类别同时可见。
- F06：问题规模/层级与成本，附实际变量和约束规模。
- F07：预声明扰动窗口的 release/执行/proposal timeline。
- F08：同资源配对结果、资源增加收益和年龄—质量关系。
- T03：实际工作站、RT 内核、调度、核心、依赖、观测和运行预算。

C3/C4 的最终 decision rows 固定来源和适用范围。完整图/表字段见 [paper 合同](../../paper/FIGURE_TABLE_PLAN.md)。

## 验收

手工耗时数组/直方图、skipped release、持续 miss、非 RT 混入、缺样和不同核资源的反例必须触发正确分层/失败。由固定 artifact 重建图表，不连接当前运行进程，不因机器当前已换内核而改写旧来源的环境身份。
