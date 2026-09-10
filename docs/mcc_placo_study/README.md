# MCC 框架论文与对照实验研究入口

状态：`development-validation`；正式 campaign 未执行。协议版本：`study-protocol.v1`。建立日期：2026-09-10。

论文标题为 **MCC: A Practical Framework for Hierarchical Whole-Body Kinematic Control**。研究以提出并验证 MCC 框架为主线，说明层级任务/约束语义、多频率参考执行架构、完整运动学链路三组设计贡献。PlaCo 公平对照与 MCC 消融用于检验这些设计的价值、代价及适用条件。优势、持平、退化和证据不足使用同一证据标准。

框架叙述覆盖 Core 计算库及基于它构建的参考应用架构，职责保持分离；“Practical”是待证实的工程性质，不表示已验证硬件部署。具体设计分组 D1–D3 与 C1–C6 的对应见[论文框架](../../paper/PAPER_FRAMEWORK.md)。

E06–E12 已有独立 app、完整机器声明、输入生成/转换及证据设施，正在完成开发验收。实际范围和证据见 [开发验收记录](DEVELOPMENT_ACCEPTANCE.md)。A01–A03 与 paper 保留协议占位，未实现或执行。已有 E01–E05、交互 app 及本次 smoke 均不充当新研究的正式证据。

## 阅读与权威顺序

1. [审阅与 gap 登记](REVIEW.md)：问题、修正、负责实验与验收。
2. [统一研究协议](PROTOCOL.md)：方法、数学语义、指标、实验准入。
3. [数据与分析合同](DATA_ANALYSIS.md)：时钟、输入、统计、artifact、未来接口。
4. 下表各实验、Analysis README：具体矩阵、指标角色和交付。
5. [论文框架](../../paper/PAPER_FRAMEWORK.md)、[论点证据矩阵](../../paper/CLAIM_EVIDENCE_MATRIX.md)、[图表合同](../../paper/FIGURE_TABLE_PLAN.md)。
6. [全部实验的 subagent 实现 prompt](IMPLEMENTATION_PROMPT.md)：仅实现 E06–E12 和必要证据设施；Analysis/论文阶段延期。

用户本轮指令与适用 AGENTS.md 优先。通用生命周期继承 [实验架构](../experiment_architecture.md)，app 代码归属继承 [Apps 约束](../../apps/AGENTS.md)。本研究公共数学和数据定义只在 PROTOCOL、DATA_ANALYSIS 维护；各实验引用它们，不复制默认值。协议冲突先记录并统一修订，不能让不同执行器自行解释。

## 已冻结的研究边界

- 论文贡献是 MCC 框架的具体系统设计与实证；不预设新 HQP 算法、理论创新或 MCC 胜出。
- 主要对照为 MCC 与仓库 vendored PlaCo；OpenSoT 仅作背景和方法参考。
- R1 运动学 whole-body IK、规划与调度耦合；无硬件、力矩、接触稳定性或通用机型结论。
- 正式计时和真实保留集验证等待本工作站重启并核验实时内核。当前仅可在后续代码阶段做开发验证与有界 smoke。
- 当前实施范围仅为 E06–E12、输入/证据合同、独立核验及开发测试，主要在 Lab，允许必要的 Core 只读诊断扩展，算法修复另立候选。
- A01–A03、跨 run 统计、科学图表和交互分析报告、paper 工具及正文均在实验 campaign 结束后再实现和执行。实验阶段保留其字段、指标、配对与图表规范，不提前生成结果。
- 不提交、推送、发布，不替作者选择投稿 venue、作者名单或数据公开许可。

## 研究索引

| 实验 | 问题 | Analysis | 论文证据 |
|---|---|---|---|
| [E06](../../experiments/E06_solver_semantic_parity/README.md) | 比较条件与模式是否对齐 | A01 | 全部论点的准入；F02、T01 |
| [E07](../../experiments/E07_hqp_priority_and_redundancy/README.md) | 优先级保持与冗余利用 | A01 | C1；F03 |
| [E08](../../experiments/E08_constraints_scaling_and_degeneracy/README.md) | 限位、缩放、退化与恢复 | A01 | C2；F04 |
| [E09](../../experiments/E09_solver_cost_and_scalability/README.md) | 完整 IK 成本与问题规模 | A02 | C3；F05、F06 |
| [E10](../../experiments/E10_multirate_scheduling_and_coupling/README.md) | 分频、并行、耦合的独立收益 | A02 | C4；F01、F07、F08 |
| [E11](../../experiments/E11_planning_ik_otg_error_propagation/README.md) | 规划至执行的误差传播 | A03 | C5；F09 |
| [E12](../../experiments/E12_recorded_motion_holdout/README.md) | 真实录制会话中的效果与覆盖 | A03 | C6；F10、T02、T04 |

分析入口：[A01](../../analyses/A01_priority_and_robustness/README.md)、[A02](../../analyses/A02_runtime_and_scheduling/README.md)、[A03](../../analyses/A03_end_to_end_and_recorded_motion/README.md)。平台证据为 T03。

## 实施顺序与状态

文档及评价规则登记 → 公共实验合同及独立核验 → E06/E07/E09 小规模贯通 → 全部 E06–E12 代码与开发验收 → RT/数据预检 → 开发与调参 campaign、冻结正式声明 → 单独启动正式实验 → campaign 状态清单与证据冻结 → A01–A03 实现及分析 → 图表/论文工具 → 论文撰写与科学复核。

本次实现 prompt 的终点是实验代码和开发验收，并交付正式运行命令与证据字段。开发/调参所需单元级核验与候选筛选遵循预登记规则，不属于延期的跨 run 科研分析。正式 campaign 不由代码任务自动启动。campaign 结束必须登记全部 required 单元的成功、失败、缺失和未运行原因；结束不要求结果有利，也不允许把仍未运行的实验标成完成。

代码完成、正式运行就绪、正式实验完成、分析完成与论文证据完整分别报告。环境、数据或科学结果不满足时，继续可独立完成的实验工程工作，但不能提前转去实现延期的 Analysis/论文或把待执行阶段标成已完成。本次开发证据保存在 `runs/mcc_placo_study/development_acceptance/`；未晋升 `results/`。

## 实验工具入口

公共接口见 [实现合同](IMPLEMENTATION_CONTRACT.md)，命令与字段说明见 [工具 README](../../tools/mcc_placo_study/README.md)。默认运行 `/workspace/install/algorithm/bin/mcl_study_e##`；`MCL_INSTALL_PREFIX` 可切换安装前缀，`MCL_BINARY` 为最高优先级覆盖，不回退旧 standalone binary。

```bash
python3 tools/mcc_placo_study/study.py --dry-run --inventory-output /tmp/study-development-matrix.json
python3 tools/mcc_placo_study/study.py --phase confirmatory --preflight
python3 tools/mcc_placo_study/study.py --experiment E06 --case target_pose_zero --repeat 0 --smoke --timeout 20
python3 tools/mcc_placo_study/check_inventory.py <run-dir>/inventory.json
```
