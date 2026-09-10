# E08 输入合同

状态：`generated-development`。输入已生成，正式冻结尚未执行。

从 E06 的同一模型和 E07 的任务公式产生 [E08 矩阵](../README.md)；保存实际 limits、q/qdot、Jacobian 特征、期望目标、事件时间和 shared hard-set feasibility 分类。近奇异样本通过不依赖任一候选运行结果的采样/排序得到，保留全部候选筛选记录。

collision 子集需要相同 mesh hashes、pair list 和精确几何距离查询；缺几何时仅此子集 unavailable。不得将 soft collision requirement 改成硬安全约束。

formal 前冻结用于 stall/recovery 的数值阈值、持续窗口与来源理由。存在未知可达性/初态可行性时保留 unknown；不根据某 solver 失败反推目标不可达。遵循 [数据合同](../../../docs/mcc_placo_study/DATA_ANALYSIS.md)。
