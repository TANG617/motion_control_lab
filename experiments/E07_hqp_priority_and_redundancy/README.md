# E07：HQP 优先级保持与冗余利用

状态：`implemented-development`；正式运行和分析 deferred。公共定义：[PROTOCOL](../../docs/mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../docs/mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](inputs/README.md)。消费：[A01](../../analyses/A01_priority_and_robustness/README.md)。对应 G01/G03–G06；C1；F02/F03、T01。

## 问题与假设

加入次级任务时，MCC HQP 能否在声明容差内保持上层结果，并利用可用冗余改善次级目标？当冗余不足时，行为是否跟随明确的优先级配置？

假设只针对成功完成声明层级的 tick；partial/suboptimal 仍计入失败覆盖，不因条件化核验而被删除。若主任务 drift 越界、未获得预期次级收益或优先级配置未改变冲突结果，保留反证。

## 对照与场景

arms 为 placo-controlled、mcc-weighted、mcc-hqp-2、mcc-hqp-3；每种拓扑再设 primary-only 参考。优先级比较保持同一 backend 可用性与 scale 分组；不能把 per-arm scale 收益全算成 HQP 收益。

| case family | 明确构造 | 验证点 |
|---|---|---|
| analytic_redundant | 变量 x=(x1,x2)，L1: x1=1；L2: x2→1；box [-2,2] | 可兼容次级目标改善且主层保持 |
| analytic_conflict | 同 L1；L2: x1+x2→0，固定 x2=0 | 次级让步；交换两层得到可解释的另一结果 |
| analytic_rank_deficient | 重复/线性相关行、显式 disabled 任务及非唯一关节解 | 不通过额外 epsilon 改目标；比较 semantic optimum |
| r1_hold_tcp | 双手位置保持，link4 目标沿各坐标轴 ±0.02/±0.05 m | 有冗余时主位置与肘部收益同时核验 |
| r1_orientation_conflict | 双手位置目标相同，朝向旋转 0.1/0.3 rad，叠加 posture/link4 | 2 层同级折中与 3 层朝向优先的差异 |
| r1_weight_sweep | 次级权重相对基准 ×0.01/0.1/1/10/100 | weighted sensitivity 与 HQP preservation 同图 |
| r1_task_transition | 有/无 link4、posture 的显式 enabled 切换 | 固定 topology 的数值切换和输出连续性；不声称在线重排层级 |

上述解析案例的 L1/L2 表示有序的 soft squared-L2 目标；box 和指定 x2=0 才是跨层硬约束。因此交换冲突目标的层级可以改变最优结果，不会悄悄移动硬约束。

hard-primary placo 条件作为正面对照：可行的 hard 位置约束下也应能优化次级任务。对不可行/soft/scaled 主任务另列条件，不将其混入 hard-primary 比较。

R1 冻结 snapshots 与 4 s evolving hold 场景都运行：0–1 s 主任务，1–3 s 次级目标，3–4 s 撤销次级目标。该窗口为预声明，不能看结果后移动。场景矩阵采用 family 内的离散扫描，不把所有因素做无意义全笛卡尔积。

## 指标

- Primary：preservation_ratio，以及通过主任务 guardrail 时的 secondary_gain；同时给绝对 cost change。
- Guardrail：hard_violation_excess、full_quality_rate、实际 position/orientation_error、source/attempt 覆盖。
- Diagnostic：按层 reference_objective_gap、scale、task residual、非唯一解状态、joint_vaj、权重敏感度、完整调用成本。

保持核验使用同 tick common state/matrix 的 residual_at_level 与 residual_final，逐分量按声明容差归一。Tertiary/Terminal 的数值 regularization 不代表新的语义优先级。离线核验与参考解不计入控制调用耗时。

## 实现和产出

小规模 oracle 独立解析/枚举活动集，不调用 MCC hierarchy executor。枚举不可行条件和多解也保留明确结果。R1 task matrices 独立复算，并声明共享模型库的共同错误边界。

实验 app 输出所有 declared task、layer、enabled、reference residual、final residual、scale 和质量状态；需要额外 Core 诊断时由主 agent 处理。生成 priority_checks.csv、secondary_effects.csv、task_transition_trace.csv、oracle_checks.csv。A01 生成 F03 的 drift/secondary improvement 成对面板及权重扫描，不能只截取表现最好的时段。

## 验收

解析兼容、冲突、重排、rank-deficient 和至少一个 R1 hold 场景贯通；故意扰动 final residual 的 fixture 应被核验器拒绝。primary-only cost=0 时不得除零。缺失高层参考或低层未执行时指标明确 unavailable/partial，而不是 drift=0。

“支持三层配置”与“该三层配置在本实验更好”分开报告。扩展任务所需的配置/代码改动只作工程描述，无受控用户研究时不声称开发效率统计提升。

## 开发执行与验收入口（2026-09-10）

所有 solver/task/loop 实现在 `apps/study_e07/`，通过标准 colcon 产物 `mcl_study_e07` 执行；不复用其他 app 私有实现。`generate.py` 生成 README family 内离散扫描的 `definition.json`、真实 canonical inputs/descriptors 及 case inventory，输入与 generator/model 文件均有 SHA-256。

```bash
python3 experiments/E07_hqp_priority_and_redundancy/generate.py --model /workspace/models/r1.cos.urdf
python3 -m pytest -q experiments/E07_hqp_priority_and_redundancy/tests
python3 tools/mcc_placo_study/study.py --experiment E07 --phase development --dry-run --inventory-output /tmp/E07-matrix.json
# 仅在主 agent 的串行开发验证闸门中运行选定 case 的 smoke：
python3 tools/mcc_placo_study/study.py --experiment E07 --case CASE --method METHOD --repeat 0 --phase development --smoke --timeout 60 --output-root /workspace/runs/mcc-placo-development
```

`verify.py --request <unit/request.json>` 在 native 计时外自动调用；输出独立 XML FK、hard-set 原始违反、同状态 Jacobian、原生 reference 与独立 final residual、解析主动集参考、独立 bounded least-squares 主任务参考及质量/事件表。后者只构成单元正确性核验，没有跨 run 科研统计。非唯一解按语义目标/约束判断，保持容差不加 epsilon。

数学边界：两库实际位置 task 都控制 frame origin，使用 `p_frame_goal = p_tcp_goal - R_goal * offset`。该共同生产式变换与真正的 TCP Jacobian 任务有差异，`method_semantics`/parity 中明确披露；实际 TCP 误差由独立 FK 复算。MCC 保持现有 1e-8 numerical regularization，不追加目标修复。PlaCo native throw-on-failure、MCC post-solve acceptance、TargetSolve 停止政策的差异保留，不将不同 accepted policy 宣称纯 backend parity。

所有动态回灌均为明确的理想运动学 committed-state；HOLD 保留原 q，原生拒绝不消费失败 iterate。smoke 只覆盖显式选定且有界的前缀，不能代表 4 s / 6 s 完整窗口或正式结果。正式前仍需模型/输入/配置/阈值及 E06 配对准入冻结、环境预检和新建输出目录；本任务不执行正式 campaign。

开发事件覆盖另有 `fixture_compressed_events`（仅 development）：18 个 1 ms tick 内在 3 ms 开始困难/次级任务、12 ms 撤销，覆盖全部事件类型；它明确标记 fixture，不能替代正式完整窗口。

拒绝模型明确采用 `freeze-measured-q-and-v`：未接受 tick 保留原测量 q 和 v，且不消费失败 iterate。这是冻结测量状态的理想诊断策略，不代表物理停止 plant；HOLD 时 q 的时间差分与保留的测量 v 可能不一致，两者不得混为物理导数。

## 实际开发验收证据

代码：implemented；专项测试：7 passed；真实 smoke：53/53 单元 completed / 独立检查通过，包含 40 个小问题 oracle 校验。正式运行就绪：正式冻结/准入/RT 尚未完成；formal-executed：false；分析和论文：deferred。

[修正后真实 E07 manifest](../../runs/mcc_placo_study/development_acceptance/smoke/20260910T105533.501826Z-861f19d76c/E07.manifest.json)。完成进程不代表完整层级：R1 HQP3 hold 样本保留一次 Tertiary `PROXQP_SOLVED:HARD_FEASIBILITY_TOLERANCE_EXCEEDED`，当前 tick 选择 Secondary，requested 3 / completed 2，明确 `partial-hierarchy`。未放宽硬容差、增加预算或消费失败 Tertiary iterate。

[首轮失败证据](../../runs/mcc_placo_study/development_acceptance/smoke/20260910T104943.607018Z-861f19d76c/E07.manifest.json) 保留全部 20 个 analytic HKS 装配崩溃：app 在添加 level 前注册 shared bounds，违反 public API 顺序。修复只调整注册顺序，原矩阵/目标/约束不变。另修复 checker 将 NotRun 的预分配零 residual buffer 当作 optimum 的错误；缺失层级现在 unavailable，而非假 drift 或零最优解。

```bash
python3 tools/mcc_placo_study/study.py --experiment E07 --case analytic_redundant --case analytic_conflict --case analytic_conflict_swapped --case analytic_rank_deficient --case analytic_active_bound --case hard_primary_positive_control --case r1_hold_tcp_left_0_0.02_snapshot --case fixture_compressed_events --repeat 0 --phase development --smoke --timeout 60 --output-root runs/mcc_placo_study/development_acceptance/smoke
```

压缩事件 window 测量修正另存于 [独立重核验](../../runs/mcc_placo_study/development_acceptance/revalidation/a_directional_window_20260910T105954.035461Z/E07/revalidation.json)：只读取原 raw，SHA-256 相同，不重新运行候选；原测量文件保留。
