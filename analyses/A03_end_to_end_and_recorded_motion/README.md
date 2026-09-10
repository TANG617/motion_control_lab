# A03：完整链路与真实动作分析

状态：`planned`。只分析固定 evidence，不重新执行任何 solver、planner 或模拟器。

阶段：实验 campaign 结束并冻结来源与全单元状态清单后，再实现和运行本 Analysis。当前只保留评价协议及证据字段要求；实验代码阶段不创建本 Analysis 的声明、执行器、统计/图表或报告。开发所需逐单元独立核验属于 E06–E12，不触发此阶段。

来源：[E11](../../experiments/E11_planning_ik_otg_error_propagation/README.md)、[E12](../../experiments/E12_recorded_motion_holdout/README.md)，并引用 E06/候选冻结信息。指标：[PROTOCOL](../../docs/mcc_placo_study/PROTOCOL.md)；配对与统计：[DATA_ANALYSIS](../../docs/mcc_placo_study/DATA_ANALYSIS.md)。负责 C5/C6、F09/F10、T02/T04。

## 分层与对齐

historical migration、controlled pipeline ablation、recorded case study、未暴露 holdout 分开。输入/frame/模型、feedback model、pipeline hash、phase、source interval 和 settling 窗口是准入条件。不能把旧记录产生的 q 当成候选标准答案。

严格使用同一 tracking record 的 goal/reference/IK/execution，不能拼接不同 attempt 的 latest messages。跨方法按 canonical time/window 对齐，不能按“第 N 个成功 solve”对齐；HOLD/拒绝期间仍纳入演化轨迹误差。

## 分析方法

E11 逐阶段独立计算误差和 lag，按固定 IK 方法配对 planner-only、joint-only、两者和 direct。非线性误差范数不相加。joint derivative 的原生与差分来源保持区分；projection 事件前后单列。

E12 先核验 session split/exposure/candidate freeze，再算每动作指标和失败类型。按 session 聚合和 bootstrap；没有合格 holdout 时输出案例研究并将泛化结论设 evidence-insufficient。披露全部 required arms/recordings，效果表的条件性分母不得省略。

固定分析策略不得重新选择候选、改变阈值或因为结果不好重新切动作。实际 motion extent、速度/空间包络、录制长度和缺失情况进入 dataset coverage。低覆盖的困难类型不代表已经验证。

## 产出

sources/inventory.json、stage_effects.csv、recording_metrics.csv、session_effects.csv、paired_coverage.csv、failure_matrix.csv、claim_decisions.csv、validation.json，以及 report.md/html。

F09 显示完整预声明时间窗的四阶段轨迹/误差和 pipeline 配对效果；F10 显示逐动作差异、会话级区间、失败与覆盖。T02 描述来源/划分/曝光/包络，T04 同列效果、质量、覆盖、时序和主要限制。论文生成只消费这些冻结表，不重新计算另一套数字。

## 验收

检查已知固定时延、不同输入窗口、source dropped、HOLD、projection、未完成 settling、相邻片段误分组、已暴露 holdout 和 missing pair。hash 不一致或分组污染时相关确认性证据失败；描述性输出可以保留，但不可转成 C6 支持。

没有真实输入的开发验证用明确标记 fixtures，不能借同名 input_id 混进论文。图例明确 execution 是声明的运动学模型输出，避免视觉上暗示真实硬件结果。
