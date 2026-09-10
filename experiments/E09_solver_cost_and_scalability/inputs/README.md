# E09 输入合同

状态：`implemented`。真实 canonical/descriptor 输入由本实验 `prepare.py` 生成；输入与模型哈希绑定到 `definition.json`。

使用 E06/E08 已审计的 frozen snapshots，保留状态顺序及相同 replay count；不将各方法自行演化出来的不同 q 当成纯计算成本比较。每个 workload 保存 task rows、constraint counts、active joints、backend 和 enabled topology 的声明。

输入解码/模型加载在计时前完成；cold model construction 单独测量时另设 window。所有正式 count/window/arm order、核心资源和观察条件在运行前冻结。未完成 E06 parity 的 cell 只能 descriptive，不出 causal speedup。

详见 [数据与时间统计合同](../../../docs/mcc_placo_study/DATA_ANALYSIS.md)。
