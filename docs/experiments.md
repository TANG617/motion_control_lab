# MCL 研究愿景

MCL 用 R1 遥操作、输入回放和独立数值实验研究运动控制框架：任务优先级与冗余、
约束与困难状态、计算成本、多频率耦合，以及 Cartesian planning → IK → joint planning 的完整链路。

MCC/PlaCo 对照以实际方程、输入、约束、接受策略和资源条件的准入为前提。
优势、持平、退化、失败和证据不足使用同一证据标准，不预设 MCC 获胜。
控制轨迹是候选输出，不是 ground truth；过程完成也不表示跟踪或实时性通过。

具体研究问题、实验与分析进度统一见 [研究索引](mcc_placo_study/README.md)。
工程扩展遵循 [应用架构](app_component_architecture.md) 和 [证据生命周期](experiment_architecture.md)。
