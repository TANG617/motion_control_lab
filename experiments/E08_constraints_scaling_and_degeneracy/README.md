# E08：约束、缩放与退化场景

状态：`implemented-development`；正式运行和分析 deferred。公共规则：[PROTOCOL](../../docs/mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../docs/mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](inputs/README.md)。输出：[A01](../../analyses/A01_priority_and_robustness/README.md)。对应 G03–G06/G14；C2；F04。

## 问题与假设

在共同硬约束下，MCC 的任务分层和缩放配置是否能保持有效进度、明确退化并恢复？零 scale 的可行输出不自动算完成，soft collision 的低 cost 不自动算安全。

可证伪假设为候选在指定困难条件中满足硬约束且改善预声明 progress/recovery 指标。任何 hard violation、拒绝增加、较差进度或不能恢复都进入主报告。

## 对照和矩阵

使用 E06 对齐后的 placo-controlled、mcc-weighted、mcc-hqp-2/3。每种方法只运行共同支持的约束子集；某 backend 无法原生表达的加速度/制动约束不得通过输出 clipping 假装等价。共同问题失败与能力缺口分开，完整 MCC 约束能力另做明确的 native-policy 条件。

| 因素 | 场景/扫描 | 控制 |
|---|---|---|
| joint_limit | 所选 arm/torso 关节距上下界为其范围 0.5%、2%、10% | 相同初始状态、hard limits，记录 active rows |
| moving_state | qdot 为对应 vmax 的 0、±0.25、±0.75；目标继续/反向/停止 | 状态是否满足制动可行域提前分类；不可行状态不混成 solver bug |
| unreachable_one_arm | 另一手保持可达；单手目标沿固定方向增加 0/0.05/0.15/0.30 m | 不能仅凭距离宣称全局不可达，记录参考可行性判据 |
| scale_group | shared vs per-arm scale | 同方法消融；仅 native 支持时跨方法匹配 |
| near_singular | 按独立 Jacobian 奇异值对固定采样状态分桶 | 先生成/冻结再跑方法，报告每桶实际覆盖，不按结果挑选 |
| conflicting_tasks | position/orientation/posture 冲突与优先级交换 | 与 E07 相同任务公式和保持容差 |
| recovery | 困难目标解除并回到已知可达基准 | 固定事件窗口与恢复阈值 |
| collision_diagnostic | 有明确几何 pair 的接近/退出 influence region | soft 目标，独立 exact distance；不做硬安全主张 |

每个动态 case 为 6 s：0–1 s 基准，1–4 s 困难目标，4–6 s 恢复。若窗口结束未恢复，保存 censored 和剩余误差。不为通过而延长某个方法的运行。

## 指标

- Primary：hard_violation_excess（按约束类别）、progress/stall_duration 和 recovery_time。
- Guardrail：完整质量率、拒绝/部分层级比例、双臂实际跟踪、完整窗口覆盖。
- Diagnostic：scale/equation residual、baseline feasible velocity、各关节裕量、singular values、active constraint 变化、collision distance/shortfall、joint_vaj。

scaled preservation 和实际沿轨迹进度分别展示。近奇异用数值特征描述；无法获得全局 workspace 证明时采用“reference-infeasible under the stated local model”，不夸大成全局不可达。

## 实现和产物

生成器在运行前给每个 q/qdot 标注 shared hard-set feasibility，区分可行输入、不可行初态和未知。独立 verifier 复算速度盒/制动条件及积分后状态，保留原生 solver gate 与外部核验的差异。

输出 difficult_case_inventory.csv、constraint_trace.csv、progress_recovery.csv、quality_events.csv 和 oracle/reference 记录。optional collision 流缺失时相关指标 unavailable，不能填距离 0。A01 按 case family、side、constraint 类别出 F04，失败与恢复 censored 保持可见。

## 验收

至少验证已知可行边界、初态已不可行、零进度、单臂冲突、动态反向及未恢复条件。错误接受/错误拒绝通过独立简单问题核验；R1 仅提供局部模型证据时注明范围。实现不得修改 solver 接受政策、限位或迭代预算来令该实验通过。

## 开发执行与验收入口（2026-09-10）

所有 solver/task/loop 实现在 `apps/study_e08/`，通过标准 colcon 产物 `mcl_study_e08` 执行；不复用其他 app 私有实现。`generate.py` 生成 README family 内离散扫描的 `definition.json`、真实 canonical inputs/descriptors 及 case inventory，输入与 generator/model 文件均有 SHA-256。

```bash
python3 experiments/E08_constraints_scaling_and_degeneracy/generate.py --model /workspace/models/r1.cos.urdf
python3 -m pytest -q experiments/E08_constraints_scaling_and_degeneracy/tests
python3 tools/mcc_placo_study/study.py --experiment E08 --phase development --dry-run --inventory-output /tmp/E08-matrix.json
# 仅在主 agent 的串行开发验证闸门中运行选定 case 的 smoke：
python3 tools/mcc_placo_study/study.py --experiment E08 --case CASE --method METHOD --repeat 0 --phase development --smoke --timeout 60 --output-root /workspace/runs/mcc-placo-development
```

`verify.py --request <unit/request.json>` 在 native 计时外自动调用；输出独立 XML FK、hard-set 原始违反、同状态 Jacobian、原生 reference 与独立 final residual、解析主动集参考、独立 bounded least-squares 主任务参考及质量/事件表。后者只构成单元正确性核验，没有跨 run 科研统计。非唯一解按语义目标/约束判断，保持容差不加 epsilon。

数学边界：两库实际位置 task 都控制 frame origin，使用 `p_frame_goal = p_tcp_goal - R_goal * offset`。该共同生产式变换与真正的 TCP Jacobian 任务有差异，`method_semantics`/parity 中明确披露；实际 TCP 误差由独立 FK 复算。MCC 保持现有 1e-8 numerical regularization，不追加目标修复。PlaCo native throw-on-failure、MCC post-solve acceptance、TargetSolve 停止政策的差异保留，不将不同 accepted policy 宣称纯 backend parity。

所有动态回灌均为明确的理想运动学 committed-state；HOLD 保留原 q，原生拒绝不消费失败 iterate。smoke 只覆盖显式选定且有界的前缀，不能代表 4 s / 6 s 完整窗口或正式结果。正式前仍需模型/输入/配置/阈值及 E06 配对准入冻结、环境预检和新建输出目录；本任务不执行正式 campaign。

能力缺项单独声明：PlaCo feasible-reference shared/per-arm scaling、MCC native acceleration/braking 对照不能假造共同数学问题；collision 子集缺冻结 mesh hashes/pair list/独立 exact-distance provider 时 unavailable，不能填距离零或声称硬碰撞安全。近奇异输入由独立 finite-difference position Jacobian 对固定 12 个候选排序分桶，全部候选与实际奇异值保留。

开发事件覆盖另有 `fixture_compressed_events`（仅 development）：18 个 1 ms tick 内在 3 ms 开始困难/次级任务、12 ms 撤销，覆盖全部事件类型；它明确标记 fixture，不能替代正式完整窗口。

拒绝模型明确采用 `freeze-measured-q-and-v`：未接受 tick 保留原测量 q 和 v，且不消费失败 iterate。这是冻结测量状态的理想诊断策略，不代表物理停止 plant；HOLD 时 q 的时间差分与保留的测量 v 可能不一致，两者不得混为物理导数。

## 实际开发验收证据

代码：implemented；专项测试：8 passed；真实 smoke：25 个声明单元，19 completed / 独立检查通过、2 个明确能力差异 unavailable、3 个 MCC 初态已不可行单元 completed / 独立约束检查失败、1 个 PlaCo 初态不可行 native QPError 崩溃。正式运行就绪：正式冻结/准入/RT 尚未完成；formal-executed：false；分析和论文：deferred。

[最新真实 E08 manifest](../../runs/mcc_placo_study/development_acceptance/smoke/20260910T105602.548978Z-32af7bd026/E08.manifest.json) 与完整原始输出保留上述负面结果。初态负例超过 URDF 上界约 0.01 rad，MCC 拒绝后冻结原测量状态，外部检查仍报告该初始违反；不能把它归因为 solver 引入越限。PlaCo 保留原 `QPError: Infeasible QP`，begin-only trace 不再被 checker 标为通过。

48 条独立 scaled equation 检查通过。native acceleration/braking 的 HQP2 样本保留 ticks 4–7 Secondary `PROXQP_PRIMAL_INFEASIBLE`，当前 tick 只接受 Primary，完整层级没有完成。所有约束和预算保持原声明。

```bash
python3 tools/mcc_placo_study/study.py --experiment E08 --case joint_limit_left_arm_joint4_upper_0.005 --case moving_state_0.25_reverse_common --case moving_state_0.25_reverse_native-policy --case near_singular_high --case unreachable_one_arm_left_0.15_shared --case initial_infeasible --case fixture_compressed_events --repeat 0 --phase development --smoke --timeout 60 --output-root runs/mcc_placo_study/development_acceptance/smoke
```

[首轮证据](../../runs/mcc_placo_study/development_acceptance/smoke/20260910T105013.430338Z-32af7bd026/E08.manifest.json) 保留 snapshot recovery 无时间区间及 NotRun reference checker 错误。修正后 snapshot/未覆盖撤销事件明确 not-observed，压缩 fixture 撤销后仅观察 6 ms，小于声明 100 ms dwell，恢复结果保持 censored，未填零。

[压缩 window 独立重核验](../../runs/mcc_placo_study/development_acceptance/revalidation/a_directional_window_20260910T105954.035461Z/E08/revalidation.json) 使用 canonical `event_windows.challenge` 选预定方向，替换误用完整场景 1 s 窗口的测量代码。它绑定原 raw/request/旧检查与新检查哈希；raw 字节相同，原产物未覆盖。
