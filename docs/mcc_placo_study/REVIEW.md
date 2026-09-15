# 研究设计审阅与 gap 登记

状态：持续维护的研究准入问题。原审阅日期：2026-09-10；工程状态更新：2026-09-14。本表的“修正”是约束，不表示已经通过实验验证。

严重性：P0 会使主要结论无效；P1 会限制归因、复现或外部有效性。关闭分为 specification-closed 和 evidence-closed；目前只完成前者。

| ID | 级别 | Gap / 风险 | 必需修正 | 负责范围 | 关闭证据 |
|---|---|---|---|---|---|
| G01 | P0 | 已有 HQP/多频率思想或“能运行”被当成框架创新 | 以 MCC 框架为论文主体，区分已知方法、具体设计 D1–D3 与实证 C1–C6；可落地性质按证据限定 | paper、E06–E12 | RELATED_WORK、设计与消融证据对应；不凭运动学回放声称硬件部署 |
| G02 | P0 | 历史生产参数等同公平基线 | 冻结生产方法，另外设置等预算调参的受控 placo/MCC | E06/E12 | 方法审计表、开发集搜索清单、冻结配置 |
| G03 | P0 | 模型、自由度、scale、误差与模式不一致 | 共同物理模型和任务语义审计，无法对齐项独立成 factor | E06 | 每个比较单元的 parity 表，未对齐配对标 unavailable |
| G04 | P0 | solver 自证正确 | 小问题独立解析/主动集枚举，R1 独立 FK/Jacobian/残差核验 | E06–E08、A01 | oracle 案例、离线复算差异、故障注入检查 |
| G05 | P0 | 容差带称为精确零空间；有限权重 scale 称为最大可达进度 | 分开优化保持、非线性轨迹误差、weighted progress 和参考最优值 | E07/E08 | 保持容差、实际 drift、scale/误差图 |
| G06 | P0 | fallback/可行次优混入完整 HQP 成功 | 结果质量、命令是否提交、run 完成分别编码 | 全部 | 全类别分母、原生状态、selected/completed pass |
| G07 | P0 | 降频、加核心、旧参考共同变化 | 同步全频/同步分频/异步分频；固定核资源与计算次数，年龄单独扫描 | E10 | release 时间线、CPU 时间、proposal/state revisions |
| G08 | P0 | 保留集泄漏与 tick 伪重复 | 会话分组，开发/调参/保留分离，正式前冻结；失败保持完整配对 | E12、全部 Analysis | split/hash、候选冻结、会话级统计及失败覆盖 |
| G09 | P0 | 实测平均/最大值变成硬实时证明 | RT 环境门、完整运行分位数、跳过 release、最长超时链及观测开销 | E09/E10、A02 | 平台 manifest、全量/直方图定义、严格运行结果 |
| G10 | P0 | 理想积分/运动学投影被写成硬件闭环 | snapshot 与 evolving 分开；反馈与模拟模型显式 | E11/E12 | 状态更新方程、时序、阶段 FK；结论边界 |
| G11 | P1 | Analysis 声明与 manifest 身份不完整 | 实验阶段完成实验身份/旧版读取并保留 E/A 扩展设计；实验结束后实现独立 Analysis 声明与读写 | 分阶段公共合同 | 实验合同先验收；E/A 完整读写及迁移检查留到 Analysis 阶段，不提前标 evidence-closed |
| G12 | P0 | 手填数字、挑选正面图、证据漂移 | C/F/T 稳定 ID，固定 run/analysis/hash，图表读 artifact | 全部、paper | claim→table/figure→metric→source 校验 |
| G13 | P1 | 诊断开销混入性能差异 | 计时边界相同，最小/完整观测独立条件，记录队列丢样 | E09/E10 | 成对观测开销测量与样本完整性 |
| G14 | P1 | 平台/数据/模型的外推过宽 | R1、指定工作站和已覆盖动作包络，限制安全/全局最优/普适性主张 | E08/E12、paper | 数据覆盖表与 limitations |
| G15 | P1 | 独立实现产生相互冲突的接口 | 公共接口集中维护，算法归属 app，被测进程串行执行 | 应用架构与公开执行合同 | 集成检查、所有权表、完整小规模纵向运行 |

## 当前源码事实与边界

- [baseline solver](../../apps/baseline/solver.cpp) 冻结生产 revision 42ed3ce3a19f5a7346874a31ec659c0298751137，包括 20 个 full joints、16 个 active joints。不能直接把其他 app 默认 mask 当成相同条件。
- [PlaCo 源码 provenance](../../third_party/placo/MOTION_CONTROL_LAB.md) 记录 v0.9.23 快照及本地变动。比较对象是这个可追溯快照，不是所有版本的 PlaCo。
- [HKS app](../../apps/hierarchical_kinematics_step/README.md) 当前所有 profile 均为 position Primary、orientation/posture/link4 Secondary；每臂独立 position scale。Yellow 注册 posture 和 soft collision。Core 可表达三层，不等于该 app 已在使用三层。
- 该 app 可接受经过可行性核验的 Primary MAX_ITER 并跳过 Secondary；这种输出不能算完整层级收敛。最终状态需同时检查 native status、disposition、quality、selected/completed pass。
- 当前校验器支持公开 app 执行声明、analysis.v1/v2 与 manifest v1/v2/v3；旧实验读取合同保留。A01/A02 已有非 RT 探索性分析，A03 与正式统计状态见 [研究索引](README.md)。
- E05 的 P95/P99 是 1 μs 直方图桶上界；交互 TUI 的滑动窗口与完整 run 分布不能直接混用。
- 2026-09-10 查看外部 /mnt/mcap_dataset 可见 50 个直属 MCAP；没有在本阶段解码、会话分组或认证独立样本数。正式运行前重新盘点并校验内容。

## 尚未具备的科学证据

目前无 E06–E12 的运行数据、通过阈值、性能改善或泛化结论。候选搜索、保留集清单、平台配置和业务接受阈值必须由代码阶段产生的可审计探索与正式声明冻结流程落实。缺项时 formal preflight 失败，不静默选择有利默认值。

相关工作只完成 [初始来源登记](../../paper/RELATED_WORK.md)，不是完整的新颖性检索。数据能否公开、作者和投稿格式在发布阶段确定，不由实现过程推定。

公共修正规则见 [PROTOCOL](PROTOCOL.md)、[DATA_ANALYSIS](DATA_ANALYSIS.md)。每个实验的 README 列出对应 G/C/F/T ID。

阶段约束：当前 prompt 仅实现 E06–E12 与可信证据采集、单元级独立核验和开发验收。A01–A03、科研统计/图表及论文设施和撰写在实验 campaign 结束后进行；对应 gap 在本阶段保留待验收，不能以未实现分析为由伪造闭环或提前展开延期工作。
