# A02：运行成本与调度探索性分析

实现并消费固定 `nonrt_fresh16_20260911T080119Z` 批次的 E09–E10 共 5,593 个声明单元。仅读取源文件，不启动实验 app，不修改旧 validation。入口依赖已完成 A01 的 manifest、parity_table 和 pair_admission 哈希；缺少这些固定来源时拒绝执行。

```bash
python3 analyses/A02_runtime_and_scheduling/run.py --definition analyses/A02_runtime_and_scheduling/definition.json --dry-run
python3 analyses/A02_runtime_and_scheduling/run.py --definition analyses/A02_runtime_and_scheduling/definition.json
python3 analyses/A02_runtime_and_scheduling/run.py --render-only /absolute/path/to/completed/A02/run
python3 -m unittest discover -s analyses/A02_runtime_and_scheduling -p 'test_*.py' -v
```

每次产生独立 `runs/<UTC-声明哈希>/`，含源清单、全矩阵状态、evaluation CSV、F05–F08 与 T03 的 SVG/PDF/PNG 和中文 `report.md`。`--render-only` 从已完成的分析表创建新的 run，不读原始实验轨迹。生成目录 ignored，不自动晋升 results。

## 统计和来源边界

- E09 逐过程保留 cold、warmup、steady，按 nearest-rank 计算 P50/P95/P99/max。包含 rejected/partial 调用；原生质量分母为预声明调用数，native_quality_counts 保留完整原生分类（含 partial-hierarchy、feasible-suboptimal），不把 native full 当作独立质量保证。调用序列、窗口或保留不完整时尾部为 unavailable。`timing_ecdf.csv` 保存全量统计的固定百分位显示网格，不合并过程、不平均 P99。
- 实际问题维度来自 raw 的 native/public 字段；未知自由变量、slack、后端重写行保持空值。一个过程维度变化时，F06 不把最后一条维度当作全过程维度。原生 pass 时间之和保持自己的诊断身份，不当作共同 assembly 时间。
- E10 将 `planned_releases.csv` 与冻结配置网格、时间线逐 worker 对照。skipped 为 miss；计划 release 是分母。提前失败的未运行后缀与溢出造成的未知缺样分别计数。缺样、非法区间、重复/额外 release 或溢出均禁止尾部估计；连续 miss 在缺样时为已观测下界；精确 miss 比例置空，另列 observed/planned 下界。duration、start-lag、release-to-finish 分别计算，后者包含从计划 release 到 finish 的全路径。
- proposal age、captured-state age、feedback age、TCP 误差来自全部 primary 调用与独立 FK 结果；最大值、分位数及非空样本数保留。质量 full 的含义仍为原生分类，不代替独立任务达成判定。
- 资源来自当次 `resource_summary.json` 和 `environment.json`，保留 effective CPU、线程数、OS 调度、拓扑、boot、依赖与非 RT 身份。CPU 选择记录不冒充独立线程亲和性测量，未知 core type 不补值。
- 逐 case/repeat 单因素比较使用公共身份/控制条件检查及固定 A01 方法政策。backend、warm start、observation、coupling、scheduler 及显式 scheduler/rate 联合变化分别标记；资源变化不能称为纯调度收益。MCC–PlaCo 严格因果对照被阻止，production-static 单列历史基线。

## 输出导航

- `coverage.csv` / `combined/units.csv`：全部 required 单元，包括 unavailable。
- `timing_quantiles.csv`、`timing_ecdf.csv`、`cost_components.csv`、`problem_dimensions.csv`：F05/F06 源表。
- `deadline_coverage.csv`、`resource_tradeoffs.csv`、`representative_timeline.csv`：F07/F08 源表。
- `paired_runtime_effects.csv`、`observation_overhead.csv`：准入理由、受控变化、质量/资源伴随量。
- `T03_platform.csv`：完整冻结环境与库身份；T03 图为去重后的资源选择摘要。
- `a01_parity_table.csv`、`a01_pair_admission.csv`：被消费的固定 A01 证据副本。

代表时间线选输入 `input_id` 场景族内字典序首个 case、repeat 0，保留全部 arm 和完整声明窗口。仅绘图以固定分箱 min/max 压缩，保留离散事件变化和首末点；统计读取全部原始行。F07 按场景族/virtual 或真实线程分别出页，避免数百逐单元图片。每张图附源表哈希、字段、范围与显示规则。

上述固定批次只有非 RT 描述性观察；不作 bootstrap、显著性检验、总体优胜、WCET 或实时性结论。A03、F01 架构图、正式科研实验及论文工作 deferred。

## 历史分析验收入口

已完成全来源 dry-run、完整分析及仅表重绘；验收入口如下。

- [中文报告](runs/20260911T120633.450516Z-6b4beeefc5/report.md)
- [图表来源与哈希](runs/20260911T120633.450516Z-6b4beeefc5/figures/index.json)
- [全矩阵验收、测试、命令和待解决问题](../../docs/archive/study/ANALYSIS_ACCEPTANCE.md)

## v2 定向补证

`analysis.v2` 仍保持旧 E09–E10 共 5,593 个单元；`supplemental_units` 单独记录 E09 的 24 个审计单元和 216 个计时单元。新增表 `supplemental_audit`、`supplemental_timing`、`supplemental_dimensions`、`supplemental_quality` 与 `supplemental_method_pairs`，不改旧计数。

新方法 `mcc-weighted-common-servo-v1` / `placo-common-servo-v1` 仅在实际 native task 方程、权重/归一化、硬边界与 regularization 通过独立模型审计、全部 20 原始快照覆盖并固定审计哈希后开放计时。两者采用同一外部接受合同，原生拒绝始终拒绝；完整方法身份不等于隔离 backend 因果比较。计时保留原 cold/warmup/steady 数量和三种 observation、三个 repeat；audit 的矩阵采集关闭后才运行 timing。

旧 E10 的 cap8 `fixture-observation-overflow` 仍为 failed，分类为预期失败 fixture。v2 图依据 `base_evidence_classification.csv` 排除全部 fixture、能力缺项与未解决异常，但源表和状态计数保留；分类表也进入图的来源哈希。E10 新增两个仅用于开发验收的容量 smoke 配置，不进入 E09 240 单元补证或任何科学样本。

```bash
python3 -m unittest discover -s experiments/E09_solver_cost_and_scalability -p 'test_e09_evidence.py' -v
python3 -m unittest discover -s experiments/E10_multirate_scheduling_and_coupling -p 'test_e10_evidence.py' -v
python3 -m unittest discover -s analyses/A02_runtime_and_scheduling -p 'test_*.py' -v
```

补证配对指标 `complete_method_median_time_ratio = left_p50_ms / right_p50_ms`，分子、分母分别对应同一行显式 `left_method_id`、`right_method_id` 的 steady 窗口逐进程 P50 IK 耗时（ms）。比值大于 1 表示左侧方法更慢，小于 1 表示左侧方法更快；未准入或分母为零时比值为空。表内保留两侧原始 P50、分子分母定义与方向说明，不将方法排序当作优胜排序。

## 2026-09-12 定向补证交付

[最新定向补证中文报告](runs/20260912T090018.813844Z-6a689ade69/report.md) · [图表和来源索引](runs/20260912T090018.813844Z-6a689ade69/figures/index.json) · [完整补证验收](../../docs/archive/study/EVIDENCE_REPAIR_ACCEPTANCE.md)。入口为明确固定 run，不使用隐式 latest。完整计算 run 与仅表重绘均保留，CSV 数值逐字节一致。

```bash
python3 analyses/A02_runtime_and_scheduling/run.py --definition analyses/A02_runtime_and_scheduling/definition.evidence_repair.json --dry-run
python3 analyses/A02_runtime_and_scheduling/run.py --definition analyses/A02_runtime_and_scheduling/definition.evidence_repair.json
```
