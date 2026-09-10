# E11 输入合同

状态：输入准备代码已实现；正式输入/配置冻结和正式运行仍待单独启动。

canonical 输入保留 source pose、TCP/EE goal、可用的真实或构造 P/V/A、事件标签和 target period。轨迹生成参数、smoothness 假设及不可行生成候选的过滤记录一并保存。

每个 arm 独立状态演化，共用初始 q/qdot、反馈模型、planner limits 和目标序列。若 feedforward 由差分或 lookahead 得来，记录转换政策和因果性；真实导数、因果估计、使用未来样本的离线参考不可混用。

正式前冻结 [链路矩阵](../README.md)、stage frames、projection 政策、误差/lag 窗口和业务阈值。公共来源/统计规则见 [DATA_ANALYSIS](../../../docs/mcc_placo_study/DATA_ANALYSIS.md)。
