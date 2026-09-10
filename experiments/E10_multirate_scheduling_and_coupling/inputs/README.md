# E10 输入合同

状态：`implemented`。真实 canonical/descriptor 输入由本实验 `prepare.py` 生成；输入与模型哈希绑定到 `definition.json`。

canonical 输入包含目标轨迹、明确的 feedback delivery schedule、proposal delay/failure schedule 和负载事件，不仅是一串 pose。开发轨迹使用 E07 的 hold+secondary、已知可达的低幅正弦和 E08 的有界困难窗口；每对 scheduler arm 共享目标/注入 hash。

使用同初始状态在明确运动学反馈模型中独立演化。分别记录真值状态、交付给 solver 的测量状态及其捕获时间。迟到 reference 与迟到 feedback 是不同 factor；不把 target 源频率当成反馈频率。

正式前冻结核心资源、worker rates、release 顺序、期限政策、注入窗口、观察配置和样本预算。实际 CPU IDs 来自 RT 环境预检，不硬编码本次容器看到的编号。见 [矩阵](../README.md) 与 [数据合同](../../../docs/mcc_placo_study/DATA_ANALYSIS.md)。
