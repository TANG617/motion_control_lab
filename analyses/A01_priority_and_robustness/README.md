# A01：优先级与鲁棒性探索性分析

实现状态：可执行。仅消费已结束的 `nonrt_fresh16_20260911T080119Z` 中 E06–E08 的 879 个 required 单元。固定 execution index、manifest、inventory 及逐 artifact 哈希，绝不解析 latest 或旧批次，不运行实验 app、solver、FK、输入生成器或新的核验。

```bash
python3 analyses/A01_priority_and_robustness/run.py --definition analyses/A01_priority_and_robustness/definition.json --dry-run
python3 analyses/A01_priority_and_robustness/run.py --definition analyses/A01_priority_and_robustness/definition.json
python3 analyses/A01_priority_and_robustness/run.py --render-only /absolute/path/to/completed/A01/runs/RUN_ID
python3 -m unittest discover -s analyses/A01_priority_and_robustness/tests -v
```

每次实际执行由公共设施新建 `runs/<UTC-声明哈希>/`，保存 sources、combined、evaluation、figures、report.md、anomalies.md 和 `run_manifest.v3`。render-only 校验已有分析的全部输出哈希，仅从 CSV 表重建新 run，不读取实验源数据。来源写入禁止；failed/partial/unavailable 保留且不能自动支持正面结论。所有生成物 ignored，不提升到 results。

E06 导出逐字段 parity_table 与逐 case pair_admission。字段映射通过不能代替可比性；TCP surrogate / true TCP 差异和 MCC–PlaCo native acceptance 差异保留。production-static 为历史基线，不能进入公平受控优势估计。pair_admission 的 `scope=method-policy` 行不授予准入；只有具有相同 case/input/model/window/repeat/session/split/component、完成且核验通过、受控因素一致的具体 case 配对才可能 admitted。

E07 将解析 oracle、R1 snapshot、R1 evolving 和 fixture 分层。逐任务报告 residual drift 的物理单位、原 tolerance 归一化 ratio、完整/部分层级及 objective gap。解析多解按目标和可行性比较，不比较关节向量是否相等。次任务 `paired_effects` 仅使用同 case/method 的实际 primary-only 对照成本；原 `secondary_effects` 的独立 bounded-LS 参考另外保存，不能冒充候选 primary-only。若 primary-only 未记录相同任务成本，收益 unavailable，而不是零收益。每个数值单元是该 case 的进程内描述，不把 ticks 当作独立重复。

E08 流式处理完整约束、缩放、TCP误差和进度/恢复记录。16 项异常逐个保留：scale 方程超差的原 source_line、raw attempt、committed/disposition、原 tolerance；初始越界与拒绝后冻结 measured state 分开解释；崩溃保留原 stderr 及未生成结果的事实。所有失败缩放检查保存在 anomaly_scale_checks.csv，矩阵等完整证据仍由原文件哈希定位。诊断不放宽阈值，不重分类旧 validation。snapshot 无恢复时间，未恢复为 censored，零方向为 not-applicable，soft collision 不能作为硬碰撞保证。

F02 真实语义矩阵与全 required 覆盖、T01 记录的模式/缩放/约束配置、F03 分层保持及同任务收益/扫描、F04 完整异常和恢复覆盖均保存 SVG/PDF/PNG 与 CSV 来源 sidecar。曲线按冻结 generator case 前缀分 family，各 state_mode 选 case_id 字典序首项、repeat 0；不同 arm_id 不连接为同一条轨迹。全量计算，显示固定分桶 min/max 及每次离散事件前后点，事件线来自原记录，不补造窗口事件。每图保留源表字段、分析身份和哈希。

报告为中文探索性描述，记录所有失败、缺项与证据限制，不生成统计显著性、bootstrap、HTML、论文或总体优胜排名。原开发恢复阈值仍暂定。A03、正式实验与论文 deferred。

## 历史分析验收入口

已完成全来源 dry-run、完整分析及仅表重绘；验收入口如下。

- [中文报告](runs/20260911T120932.727337Z-0a77c1e2b1/report.md)
- [图表来源与哈希](runs/20260911T120932.727337Z-0a77c1e2b1/figures/index.json)
- [全矩阵验收、测试、命令和待解决问题](../../docs/archive/study/ANALYSIS_ACCEPTANCE.md)

当前报告来自原始分析的独立重绘 run；仅修正报告中 E07/E08 汇总行的范围计数，CSV 逐字节相同。A02 继续固定引用原始 A01 的准入表及 manifest，原 run 保留不变。

## 定向补证 analysis.v2

`analysis.v2` 保留原 879 个 A01 来源单元，新增 E06/E08 来源放在单独的
`ctx.supplemental_units`。旧 v1 结果不被改写；当前补证结论位于
`report_E06_E07_evidence.md` 和 `report_E08_evidence.md`，由主报告链接。

E07 的 `e07_evidence.py` 调用实验目录中的 `reanalyze_evidence.py`，只读所有
565 个旧单元的原始输出。禁用的次任务仍按相同目标和权重评价；snapshot 要求
输入线性化序列一致，演化对照只解释为各自完整轨迹窗口的实际输出表现。
16 个只读离线 worker 各使用一个数值库线程，不启动被测 app。

独立参考按全部非 fixture snapshot 及声明的任务切换时刻求解：原硬界、原保持
容差不变，当前 Core 源码中的保持带半宽为声明容差的 0.5 倍；SLSQP 收敛后还检查
原硬可行性、KKT 驻点（1e-6）和互补性（1e-9），证据不足保持 unavailable。
多解按目标和可行性比较。原始成本与加权成本、零参考及全部分母分别记录。
E06 新方法按 manifest-owned 实际方程审计和具体合同哈希准入，不能仅靠方法名。

旧 Core source inventory 未覆盖优化器源码（问题
`provenance-Core-source-inventory-old`）。新参考合同记录当前源码哈希，
数值证书只适用于离线声明问题；对旧原生最优性的裁定保持 unavailable。
完整配置未声明的 link4/posture 任务标记 not-applicable，不能混入 primary-only 收益。

```bash
python3 -m pytest -q experiments/E06_solver_semantic_parity/tests/test_e06_evidence.py analyses/A01_priority_and_robustness/tests/test_evidence_repair.py analyses/A01_priority_and_robustness/tests/test_analysis.py
```

## 2026-09-12 定向补证交付

[最新定向补证中文报告](runs/20260912T085612.471568Z-6be7f5dfbe/report.md) · [图表和来源索引](runs/20260912T085612.471568Z-6be7f5dfbe/figures/index.json) · [完整补证验收](../../docs/archive/study/EVIDENCE_REPAIR_ACCEPTANCE.md)。入口为明确固定 run，不使用隐式 latest。完整计算 run 与仅表重绘均保留，CSV 数值逐字节一致。

```bash
python3 analyses/A01_priority_and_robustness/run.py --definition analyses/A01_priority_and_robustness/definition.evidence_repair.json --dry-run
python3 analyses/A01_priority_and_robustness/run.py --definition analyses/A01_priority_and_robustness/definition.evidence_repair.json
```
