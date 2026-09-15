# 固定来源的探索性分析

A01/A02 只消费已结束的 `nonrt_fresh16_20260911T080119Z` 批次。原始 Exx `runs/` 是只读来源；本目录不拥有求解器、实验运行或输入转换。

每个 Analysis 的 `definition.json` 明确列出 campaign plan/status/execution index 和各来源 manifest/inventory 哈希。`--dry-run` 逐文件校验全部声明证据和完整矩阵，不创建分析 run。实际执行再次验证，按单元顺序处理数据，并记录来源在分析期间未改变。

```bash
python3 analyses/A01_priority_and_robustness/run.py --dry-run
python3 analyses/A01_priority_and_robustness/run.py
# A02 definition 的 admission_source 必须先固定到上述 A01 的 manifest、parity_table 和 pair_admission。
python3 analyses/A02_runtime_and_scheduling/run.py --dry-run
python3 analyses/A02_runtime_and_scheduling/run.py
python3 analyses/A01_priority_and_robustness/run.py --render-only <explicit-A01-run-directory>
python3 analyses/A02_runtime_and_scheduling/run.py --render-only <explicit-A02-run-directory>
```

每次调用新建 `analyses/Axx_*/runs/<UTC时间-声明哈希>/`，不会覆盖旧输出。`--render-only` 验证指定 Analysis manifest 的全部输出，只复制分析源表并重新绘图，不访问 Exx 或运行任何实验。Analysis `results/` 需后续复核，本工具不自动晋升。

新增 `analysis.v1` 声明和按 `run_kind` 区分 E/A 身份的 `run_manifest.v3`；v1/v2 实验文件不迁移，冻结的 experiment-only reader 不改变。公共 `tests/validate_contracts.py` 可识别新合同。

数值定义复用 `study-metrics.v1`。source plan、raw artifact、旧 validation 均保持原样。A01/A02 仅进行描述性分析；不 bootstrap、不显著性检验、不生成 HTML、不生成论文内容，不宣称 RT 性能。缺项和候选失败是合法分析输入；来源哈希损坏、身份不明确或分析程序错误才使分析执行失败。

依赖：Python 3、NumPy、Matplotlib、jsonschema；分析代码和实际依赖版本写入 manifest。CSV/JSON 是数字的权威产物；SVG/PDF 是静态图，PNG 是预览。每张图附来源表哈希、字段、选择规则和分析身份，数值不能从图反推。测试不启动被测程序：

```bash
python3 -m pytest tools/mcc_placo_study/analysis/tests analyses/A01_priority_and_robustness/tests/test_analysis.py analyses/A02_runtime_and_scheduling/test_runtime_analysis.py
```
