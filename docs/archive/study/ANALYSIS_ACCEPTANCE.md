# A01/A02 探索性分析开发与执行验收

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

验收日期：2026-09-12 UTC。A01、A02 第一版程序、全来源 dry-run、两次依次执行的完整分析及仅表重绘均已完成。唯一实验来源为关闭的 `nonrt_fresh16_20260911T080119Z`；未重新运行实验、修改候选算法或核验阈值，未晋升 results、提交、推送或发布。

## 阅读入口

- [A01 中文报告](../../../analyses/A01_priority_and_robustness/runs/20260911T120932.727337Z-0a77c1e2b1/report.md)：E06 语义准入、E07 优先级、E08 困难场景。
- [E08 全部异常诊断](../../../analyses/A01_priority_and_robustness/runs/20260911T120932.727337Z-0a77c1e2b1/anomalies.md)：原始文件和行号、attempt、committed、方程、容差及所需补验。
- [A02 中文报告](../../../analyses/A02_runtime_and_scheduling/runs/20260911T120633.450516Z-6b4beeefc5/report.md)：E09 逐过程成本、E10 调度与资源。
- [39 组图表与全部 CSV 索引](../../../runs/mcc_placo_study/analysis_acceptance_20260911/FIGURE_INDEX.md)：F02–F08、T01、T03，SVG/PDF 和 PNG；每图可追溯源表、字段、窗口、显示规则及哈希。
- [最终来源与输出审计](../../../runs/mcc_placo_study/analysis_acceptance_20260911/final-audit.json)、[65 项测试记录](../../../runs/mcc_placo_study/analysis_acceptance_20260911/final-tests.log)、[重绘数值一致性](../../../runs/mcc_placo_study/analysis_acceptance_20260911/render-reproducibility.json)。

## 全矩阵对账

下表计数单位是声明单元，不是求解成功率。所有 unavailable、执行失败和核验失败都进入覆盖表。

| 实验 | required | 完成且核验通过 | 执行或核验失败 | 不可运行 |
|---|---:|---:|---:|---:|
| E06 | 21 | 20 | 0 | 1 |
| E07 | 565 | 565 | 0 | 0 |
| E08 | 293 | 242 | 16 | 35 |
| E09 | 5400 | 1431 | 0 | 3969 |
| E10 | 193 | 184 | 1 | 8 |
| 合计 | **6472** | **2442** | **17** | **4013** |

17 项失败进一步分为 completed/failed 15、crashed/failed 1、failed/failed 1。A01 覆盖 879 单元，A02 覆盖 5593 单元。来源由 definition 中固定的 execution index、各 Exx manifest/inventory 和逐文件哈希绑定，没有隐式 latest 或旧批次混入。

## 实现及验收证据

新增 `analysis.v1`、`run_manifest.v3`，公共读取、配对、流式统计与绘图代码在 `tools/mcc_placo_study/analysis/`。A01/A02 各自拥有声明、入口、测试和 README；公共合同验证器增加新版本分支，旧 experiment manifest v1/v2 读取行为不变。

| 检查 | A01 | A02 |
|---|---|---|
| 全来源只读 dry-run | 通过，65.208 s | 通过，56.646 s |
| 完整分析执行 | 通过，410.825 s | 通过，275.877 s |
| 图表组 / 当前报告 run 输出文件 | 29 / 139 | 10 / 61 |
| 仅表重绘 | 通过 | 通过，10.455 s |
| 重绘前后 CSV | 16 张逐字节相同 | 15 张逐字节相同 |

65 项最终测试覆盖来源损坏、重复身份、禁止 latest、错配、缺项、接受策略、零分母、多解、primary-only 缺失、初始越界、censored 恢复、nearest-rank、warm-up、skipped、连续 miss、提前退出、溢出、资源分层、确定性数值和仅表重绘。测试验证读取分析过程不启动被测 app，dry-run 不创建 run。初次测试的同名模块收集冲突及错误路径日志保留；修正测试文件名/命令后全部通过。

[四份原始/重绘 manifest 验证](../../../runs/mcc_placo_study/analysis_acceptance_20260911/manifest-validation.json)通过；最终审计验证全部输出及图表源表哈希。完整分析在入口校验声明的原始证据哈希，结束时核对来源文件未变化；仅表重绘不访问 Exx 原始轨迹。[49 个冻结实验工具文件最终哈希检查](../../../runs/mcc_placo_study/analysis_acceptance_20260911/final-frozen-tools.json)全部保持原样。现有实验代码工作树改动属于前序任务，本轮不据此修改实验行为。

环境有 Matplotlib 多版本导致 Axes3D 不可用的警告；本轮全部使用二维 Agg 静态图，SVG/PDF/PNG 生成与重绘通过。已人工检查 F04 异常总览和 F08 资源—年龄—质量预览。未调整环境安装或 RT 配置。

## Run 身份及报告修正

A01 完整计算 run 为 `20260911T115830.986753Z-0a77c1e2b1`。最终阅读入口为其独立重绘 run `20260911T120932.727337Z-0a77c1e2b1`：原报告的 E07 保持汇总行数误含 E08，E08 恢复行数误含 E06/E07，现分别按实验范围显示为 1827、514。原始数值表正确，修正版 CSV 逐字节相同；新增范围测试通过。旧 run 未覆盖，修正版 manifest 用 render_source 绑定原始 manifest。

A02 完整计算 run 为 `20260911T120633.450516Z-6b4beeefc5`，其准入来源仍固定为原始 A01 的 manifest、parity_table 和 pair_admission，未偷偷改绑。A02 仅表重绘验收 run 为 `20260912T034007.159161Z-6b4beeefc5`。所有 run 及验收生成物保持 ignored。

## 实际执行命令

工作目录 `/workspace/labs/motion-control-lab`。以下完整分析按 A01 后 A02 执行；A02 执行前已将 A01 实际 manifest/准入表及哈希写入其 definition。默认声明即各自目录的 definition.json，也支持显式 `--definition`。

```bash
python3 -u analyses/A01_priority_and_robustness/run.py --dry-run
python3 -u analyses/A01_priority_and_robustness/run.py
python3 -u analyses/A02_runtime_and_scheduling/run.py --dry-run
python3 -u analyses/A02_runtime_and_scheduling/run.py
python3 analyses/A01_priority_and_robustness/run.py --render-only /workspace/labs/motion-control-lab/analyses/A01_priority_and_robustness/runs/20260911T115830.986753Z-0a77c1e2b1
python3 analyses/A02_runtime_and_scheduling/run.py --render-only /workspace/labs/motion-control-lab/analyses/A02_runtime_and_scheduling/runs/20260911T120633.450516Z-6b4beeefc5
python3 -m pytest -q tools/mcc_placo_study/analysis/tests tools/mcc_placo_study/tests/test_contracts.py analyses/A01_priority_and_robustness/tests/test_analysis.py analyses/A02_runtime_and_scheduling/test_runtime_analysis.py
```

精确 argv、cwd、耗时、退出码与原始日志见 [A01 命令](../../../runs/mcc_placo_study/analysis_acceptance_20260911/A01-commands.json)、[A02 命令](../../../runs/mcc_placo_study/analysis_acceptance_20260911/A02-commands.json)、[A01 报告重绘](../../../runs/mcc_placo_study/analysis_acceptance_20260911/A01-report-render-command.json)、[A02 重绘](../../../runs/mcc_placo_study/analysis_acceptance_20260911/A02-render-only-command.json)、[最终测试命令](../../../runs/mcc_placo_study/analysis_acceptance_20260911/final-tests-command.json)。

## 证据不足与后续条件

| 问题 | 定位 | 当前处理与所需补齐 |
|---|---|---|
| MCC–PlaCo 接受策略不同 | A01 parity_table / pair_admission | 阻止严格因果比较和跨框架加速比；后续另立对齐方法身份并冻结配置。production-static 保留历史基线。 |
| E07 实际 primary-only 次任务成本缺失 | A01 paired_effects | 801 行配对中无完整可用收益对照；保持 unavailable。后续采集同 case/method 的实际成本，独立 bounded-LS 参考不能替代。 |
| E08 12 项缩放方程失败 | anomalies.md / anomaly_scale_checks | 4 项 HQP-3 存在 committed 超差，8 项 weighted 的失败检查对应未提交结果。需分离原生任务行、有限差分与失败输出语义；不改变旧失败状态或容差。 |
| E08 3 项硬限位失败 | anomalies.csv 的初始状态与 raw attempt | 初始已越界约 0.01 rad，MCC 未提交新状态；需明确初始状态/拒绝后保留状态的核验语义，不能解释成接受了新的越界命令。 |
| E08 PlaCo 崩溃 | anomalies.md 中 stderr 定位 | 保留原生 QPError 与结果缺失；候选修复及新实验需另立版本。 |
| E10 观测溢出/缺样 | A02 deadline_coverage / resource_tradeoffs | 1 个源单元记录溢出，尾部分位数 unavailable；保留 planned/skipped/unknown/not-run 分母与连续 miss 下界。后续另轮补齐可靠观测。 |
| 非 RT 与不可运行组合 | T03_platform、两份 coverage | 本轮不证明实时性或总体优势。正式实验前检查 RT、实际 CPU/线程/亲和性/调度、输入与配置冻结，并单独串行 benchmark。 |

分析程序与本轮探索性分析执行均已完成；科学证据尚不充分，正式实验就绪条件未由本轮验收，正式实验未完成。A03、bootstrap/显著性检验、HTML、F01 架构图、论文工具与论文正文均 deferred。无需为本轮分析重启或重新运行实验；正式实验另行授权启动。
