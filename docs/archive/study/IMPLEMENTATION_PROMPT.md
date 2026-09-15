# E06–E12 全部实验的一次性 subagent 实现 prompt

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

状态：`planned`。范围修订：2026-09-10。本文是后续实验代码阶段的完整任务指令；当前只修订文档，不执行其中代码。使用时将“任务指令”至“最终交付”交给主 agent，或要求其完整读取本文执行。

## 任务指令

在 `/workspace/labs/motion-control-lab` 完成 E06–E12 全部实验的代码、输入准备、公共证据设施、独立正确性核验、相关测试和有界开发 smoke。研究目标为 **MCC: A Practical Framework for Hierarchical Whole-Body Kinematic Control**：通过受控 PlaCo 比较和 MCC 消融，验证 MCC 的层级任务设计、多频率执行架构及完整运动学链路。不得预设胜出。

本任务到实验实现与开发验收为止。A01–A03 的实现和运行、跨 run 统计分析、科学图表/HTML 报告、论文数字和表格生成、paper build、正文与 methods draft 全部留待实验 campaign 结束后单独进行。不能因旧研究文档列有这些最终产物而提前实现它们。保留其协议、指标、C/F/T ID 和字段需求，以免遗漏采集；这些是未来消费合同，不是本次编码清单。

先读取适用的 AGENTS.md，包括 `/workspace/AGENTS.md` 和 Lab 的 `apps/AGENTS.md`；核对所有 Git scopes、dirty tree、现有构建/安装产物。再读取：

- [研究入口](../../mcc_placo_study/README.md)、[审阅登记](../../mcc_placo_study/REVIEW.md)、[统一协议](../../mcc_placo_study/PROTOCOL.md)、[数据合同](../../mcc_placo_study/DATA_ANALYSIS.md)。
- E06–E12 的 README 与 inputs/README，完整理解全部 required arms/cases、指标、窗口和失败处理。
- A01–A03 与 [paper](../../../paper/README.md) 文档，仅用于确认后续需要哪些证据字段和结论边界。
- [实验架构](../../experiment_architecture.md)、现行 app/component 边界和相关源码。不能从历史文档推断当前 solver 行为。

以更新后的协议为工作合同。已有无关改动不得恢复或覆盖；发现合同冲突先统一修正文档并记录原因，不能让子 agent 各自解释。持续完成实际集成，不在只有计划、stub 或孤立模块时结束。

## 本次范围与延期范围

| 本次必须完成 | 实验结束后再进行 |
|---|---|
| E06–E12 真实 app、输入生成/转换、完整场景和方法选择 | A01–A03 的声明、执行器及研究分析 |
| 实验声明、版本化 manifest、逐样本/单元指标身份、原始时序和 artifact 哈希 | 跨 run 配对效果、bootstrap/置信区间、显著性或优劣判断 |
| 解析参考、独立 FK/约束/残差复算、测量实现的正确性测试 | F01–F10/T01–T04 渲染、交互分析报告、论文 evidence index |
| 环境预检、数据/会话/曝光/划分清单、冻结与防覆盖、执行状态清单 | paper 工具、生成数字、表格、正文、摘要、结论和 methods draft |
| 测试、非正式数值验证、全矩阵 dry-run、有界开发 smoke、正式运行命令 | 整批正式实验另行启动；其结束后才进入上述分析和论文阶段 |

独立核验属于实验可信性保障：在计时区间外计算原生残差、约束违反、逐样本误差、状态及缺失原因，并保留核验结果和原始输入。允许单元级计数/测量摘要帮助验收，禁止把它扩展成 A01–A03 的跨运行研究分析。指标定义保持唯一权威；以后 Analysis 应复用这些纯计算函数和冻结数据，不另造公式。用于 E06 的方法配对准入、输入流配对和未来配对身份登记属于本次必需内容。

## 分工与公共接口闸门

使用 subagents。主 agent 加最多三个并发子 agent；每个子 agent 必须获得明确的文件所有权、输入/输出接口、验收条件和禁止修改清单。不同 agent 不并发修改公共文件。

Wave 0 由主 agent 完成并验证：

1. 按 Apps 约束盘点全部 app 目录及直接源文件，确定 E06–E12 实际执行 app 的归属；记录 ownership。
2. 冻结公共 canonical input、实验声明、manifest 版本、metric row 身份、status 和子进程 artifact 合同。支持旧 E01–E05/v1 读取；现阶段完成 experiment 所需读写。新 manifest 为将来的 E/A 身份留出明确版本化扩展，不能沿用必须给 Analysis 填 E## 的设计，也不在此阶段实现 analysis.v1 或 A 执行器。
3. 确定 case、method/arm、repeat、session、split、window、component、input/model/config hash 的身份与缺失状态。配置解析必须输出 resolved declaration，不能只保存命令行或 Git HEAD。
4. 用有明确 fixture 标签的最小输入验证“启动独立单元 → 原始证据 → hash/status/逐样本核验”的公共链路。此步不调用 Analysis 或 paper，不把 fixture 记成真实实验结果。

公共接口通过后再启动 Wave 1：

| 子 agent | 文件所有权及责任 |
|---|---|
| A | E06–E08 目录、被分配的 app-local 实现与专项测试；语义审计、小问题独立参考、优先级/冗余、约束/缩放/退化 |
| B | E09–E10 目录、被分配的 app-local 实现与专项测试；计时边界、规模矩阵、调度/耦合、状态年龄、运行环境预检 |
| C | E11–E12 目录、被分配的 app-local 实现与专项测试；规划/IK/OTG 链路、反馈模型、录制转换、会话/曝光/划分与覆盖 |

主 agent 同时维护公共输入和证据设施、依赖配置、顶层 CMake、必要的 Core 只读诊断和集成。子 agent 对公共合同和 Core 的需求发给主 agent，不自行修改公共文件、不采用私有替代协议绕过冲突。主 agent 冻结变更后通知所有消费者。

Wave 2 仅做实验集成与交叉审阅：A 检查语义、独立参考和失败核验；B 检查计时口径、预检与调度事件完整性；C 检查输入、状态演化、会话隔离和 E11/E12 证据链。检查他人文件时先交问题清单，由原 owner 或主 agent 修改，避免并发覆盖。本轮不分配 Analysis、绘图或论文任务。

最后由主 agent 验证全部 E06–E12 的声明、required 矩阵、真实执行入口、相关测试和有界 smoke。公共接口依赖未完成不能假装子模块已集成。

## 七项实验的最低交付

各 README 是具体矩阵权威；以下用于防止漏项，不允许替代或缩小其中 required 场景。

- E06：production-static 与受控方法分别登记；模型/full-active joints/TCP/单位/mode/enforcement/scale/归一化/接受政策审计；独立核验；输出 method_semantics、parity_checks 和具体 mismatch 原因，作为后续比较准入。
- E07：primary-only、weighted、HQP 两层/三层及其必要对照；冲突、冗余、秩亏、任务切换；解析/主动集参考、同线性化保持与非线性轨迹误差分开。多解按目标/约束核验，不要求关节向量相同。
- E08：限位、不可达、奇异附近、非零初速度、缩放语义和退化/恢复；区分初始不可行与 solver failure；保留停滞、未恢复、partial 和 rejected。软碰撞代价不能被标成硬碰撞安全保证。
- E09：完整 IK 调用与可比成本分解、cold/warm/steady 窗口、task/layer/constraint/变量规模、backend/观测条件；采集全量样本或声明桶宽的直方图、CPU/分配和丢样计数，不只保存平均值或滑动窗口。
- E10：同步全频、同步分频、异步分频、核资源/负载控制；任务、反馈、求解、输出频率独立；记录所有计划 release、start/finish、skipped/deadline、proposal/state 版本与消费关系；提供确定性虚拟时序语义测试，真实时间模式单独预检。
- E11：direct、Cartesian planning、joint OTG 和完整链路的必要对照；保存 goal/reference/IK/execution 原子记录；固定状态演化与反馈模型、延迟、初始化和导数来源；投影及 HOLD 事件明确，不能将理想积分当作物理闭环。
- E12：不可变 raw MCAP、canonical 转换和 hashes、来源/会话/曝光/重叠去重、development/tuning/holdout 分组、等预算候选搜索的编排与冻结设施；保留 required recording/method/repeat 清单和全类别执行状态。实现搜索及冻结能力不代表本任务自动执行完整调参 campaign 或访问 holdout。

## 实现边界

代码主要在 Lab。solver、任务、planner、coupling、结果解释和 failure policy 留在具体 app；实验脚本只编排公开输入输出。允许业务代码重复，不跨 app include/link，不 import app-private launcher recipe，不 import 根仓 Python 实现。不创建通用 solver facade、mode-driven 控制框架或全局 mcl executable。不修改生产包、HAL、消息合同或硬件逻辑。

Core 只允许确有必要的只读诊断扩展，默认关闭，不改变求解目标、约束、warm start、迭代或接受行为；验证关闭时行为一致。实现开启/关闭观测对照，并以有界开发验证检查其测量链，正式开销结论等待正式运行。发现算法问题时保存最小复现、源码/运行物版本与失败证据并报告；不得静默修复、加入改变目标的 epsilon I、放宽硬约束/容差、提高迭代预算掩盖失败或消费未通过验收的 iterate。

production-static 保持冻结，受控配置另立 method identity。严格区分 TargetSolve/ServoStep、weighted/HQP、full/partial/suboptimal、Core 三层能力/现 app 两层配置、算法/调度/观测成本。必要的错误处理位于外围进程编排和证据检查；不向 app 增加吞异常或自动修复路径。

## 本地编排与证据交付

提供 study-local Python 编排入口，支持 experiment、method/arm、case、repeat、phase、output root、dry-run 和只读 preflight。实际命令、默认安装产物、MCL_BINARY override 与示例写回研究 README。它只启动独立实验单元，不进入热路径或拥有任务语义。此阶段 CLI 不提供 analysis/paper 子命令，不创建占位实现。

每单元独立进程、状态和输出目录。已有目录拒绝覆盖；保留首个原生错误、stdout/stderr、退出状态与部分证据。按声明继续后续单元，任一必需单元失败时整批返回非零；用户中断停止后续执行。批次清单保留 not-run/unavailable/interrupted/crashed 原因，不把整个批次伪报成功。测量验证失败可以追加独立 validation 状态，不能改写原生 solver 状态。

生成可校验的真实 experiment definition 和 resolved declaration，输入引用实际存在的 descriptor；缺输入时可完成模板/生成器代码并报告 preflight missing，不能伪造可执行声明。记录源码及 dirty 内容、构建/安装库和可执行文件、输入/模型/配置的 hashes；原始数据不可变，结果缓存可重建，不自动晋升 results。

为后续 Analysis 交付版本化 evidence inventory，列出精确 source locators、hashes、schema/metric 版本、required/实际单元、字段说明、观测完整性、质量核验和缺失原因。交付清单是实验产物，不生成 Analysis run 或 paper evidence index。现有 Foxglove/MCAP 可用于检查真实运行记录；不新增研究 dashboard、论文 renderer 或分析报告系统。

## 验证与执行条件

必须验证：解析、多解、rank-deficient、活动约束案例；独立 FK/残差/限位复算；已知位移/旋转、手工 release/deadline 序列；缺失/失败配对身份、零分母、旧版读取、hash 篡改、seed 重现、状态版本/过期与重置、窗口边界、观测 overflow、用户中断及子进程失败。测试设施用 mock 必须有标签；实验 app smoke 必须调用真实 MCC/PlaCo，不能只跑 mock 即称代码完成。

使用匹配的实际构建/安装库完成每项实验的有界 headless smoke，TUI 只在相关时做 PTY smoke。核对 launcher 未选中旧 standalone binary。完整 required 矩阵必须可 dry-run，给出 selection、单位数、输入与环境缺项；smoke 只验证其声明子集，不能代表全部矩阵执行完毕。正式计时范围的验证避免与构建、测试或其他 benchmark 并行。

本次只运行开发测试、非正式数值验证和有界 smoke。当前工作站重启并核验 RT 内核前不得正式计时或执行保留集；即使发现已是 RT 环境，本实现任务也只完成预检和开发验证，正式 campaign 留待单独启动。不得改内核、重启、修改宿主调度配置或接触真实设备。

缺模型/输入/阈值、holdout 已暴露、正式环境未满足时，preflight 显式失败并列原因；继续完成独立代码和可执行测试。不得填零、删除失败配对、选择性补跑直到胜出、调整指标保住结论或把 smoke 当正式实证。正式 benchmark 必须串行，且与构建、测试、研究分析及其他 benchmark 不重叠。

## 最终交付

持续到 E06–E12 全部代码已集成且相关检查完成；遇到外部阻塞应列出具体受影响单元、证据与仍可独立完成的工作，不能因未到 RT 内核而停止其余实现。

交付改动清单、app/公共接口所有权、相关测试与真实 smoke 证据、完整矩阵 dry-run、实际可复制命令、环境/数据前置条件、证据字段和哈希清单、算法问题最小复现，以及尚待单独执行的调参/冻结/正式实验步骤。逐实验报告 implemented、tested、smoke-executed 和 formal-executed，区分实验代码完成、正式运行就绪和正式实验完成。

明确报告 A01–A03、统计/图表、paper 工具和论文撰写均为 deferred；不启动这些下一阶段。实验结束以冻结 campaign 清单为依据，保留失败/缺失/未运行状态，不要求所有方法成功，更不意味着论文证据完整。将本次实际实现的设施状态写回相关 README，C1–C6 和 D1–D3 不因代码完成而被标成已验证。

保留工作区无关修改。未经用户另行要求，不提交、推送、发布、晋升研究结果，不替作者决定 venue、作者名单或数据公开许可。
