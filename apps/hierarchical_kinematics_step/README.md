# mcl_hierarchical_kinematics_step

这是历史 Red/Yellow HKS 控制链及双臂姿态参考的单一入口。`--profile` 必选，并决定完整 topology；
阶段能力不能用独立开关组合：

- `posture-reference-task` / `posture-reference-task-left` / `posture-reference-task-right`：planned＋pose-primary，分别启用双臂、仅左、仅右 Transformer 肘姿态任务；
- `hierarchical`：直接 target -> legacy HKS；
- `planned`：CartesianPlanner -> legacy HKS；
- `planned-otg`：CartesianPlanner -> legacy HKS -> JointPlanner；
- `planned-otg-nullspace`：双臂 position Primary > soft orientation/posture/link4 Secondary HKS + JointPlanner；
- `planned-otg-nullspace-admittance-kinematic-sim`：再增加导纳、MuJoCo 运动学投影、viewer
  与完整 replay telemetry gate。

默认 `--hqp-layout position-first` 的 Red 保持两级：Primary 只有双臂位置任务，每臂独立的 position scale；
Secondary 在保持 Primary 位置结果的条件下，优化软姿态、Yellow posture 和启用的 link4。
姿态与 Secondary 内其余任务通过权重折中，不再要求满足 scaled orientation 等式。
关节位置、速度、加速度与制动硬约束保持各 profile 原配置。

`planned-otg-nullspace` 新增可选 `--hqp-layout pose-primary`：双臂完整 TCP 位姿
进入 Primary，同臂位置/姿态共用 scale；Secondary 保留 link4 与 Yellow posture。
双臂在线推理统一使用 MCC 原生 `posture_reference::Predictor`，入口、安装及完整窗口合同见 [HARP.md](HARP.md)。
`planned + pose-primary` 沿用同一 HARP 任务拓扑，输出接受的 IK P/V。
当前可视化按肘部参考功能命名；实时 HARP 保留原 topic，录制回放使用 `foxglove/recorded_elbow_reference.layout.json`，只标为 recorded。

旧单臂 PiM/HARP recorded 格式已移除，历史录制及失败记录原样保留；其[历史运行评估](../../docs/archive/pim/PIM_IK_EVALUATION.md)不代表当前 HARP 模型结果。

position-first 的软姿态配置由以下参数控制（历史 profile 适用，新姿态参考 profile 固定 pose-primary）：

- `--red-secondary-task-tcp-orientation-weight`：默认 `100`；
- `--red-secondary-task-tcp-orientation-servo-gain-per-s`：默认 `10`；
- `--red-secondary-task-tcp-orientation-residual-normalization-radps`：默认 `1`。

姿态代价为 `0.5 * weight * ||(J_angular * qdot - desired_angular_velocity) / normalization||²`。
position 的历史 `*-cartesian-progress-*` 配置名继续保留，现仅控制 position scale。
position-first 使用
`--red-secondary-task-tcp-orientation-preservation-tolerance-radps`；它是 Core 注册任务所需的
后续层保持容差，两级配置中没有后续层，不作为姿态跟踪误差上限。

`options.hpp` 是 app 的唯一 typed 配置入口，也完整持有 R1 robot 配置。可用
`--dump-resolved-options` 在加载模型前输出 profile、能力、robot、solver、planning、replay、
binary argv 与 launcher provenance 的完整 JSON。

批量交互回放可显式开启 `--replay-exit-on-fault`：Replay 遇到致命错误后关闭输出、
写入失败产物并退出，即使 TUI 开启也不等待人工退出 FAULT HOLD。默认关闭，
`--no-replay-exit-on-fault` 恢复交互检查行为。该选项不改变求解或错误判定。
可复用批量入口见 [E05](../../experiments/E05_real_scene_planned_mcap_batch_replay/README.md)。

Red HKS 默认允许 Core 验收 constraint-feasible 的 Primary `MAX_ITER` 最后迭代：该 tick
发布新输出、标记 `feasible-suboptimal`，并跳过 Secondary；任何 task equation、scale box
或 joint hard bound 超限仍会拒绝并 HOLD。使用
`--no-red-accept-feasible-primary-max-iterations`（或原生二进制别名
`--red-reject-primary-max-iterations`）可恢复严格拒绝策略，
`--red-accept-feasible-primary-max-iterations` 可显式恢复默认策略。

Red 与 Yellow 的 QP 数值配置相互独立。`planned-otg` 的 Red ProxQP 历史实际默认
`maximum_iterations=1000`，两个 nullspace profile 为 `200`；Yellow 默认为 `1000`。
例如调整 interactive MCAP 的 Red 迭代预算：

```bash
scripts/profiles/planned_otg/run_mcap_interactive.py \
  --red-proxqp-maximum-iterations 500
```

对应的 regularization、absolute/relative/primal-infeasibility tolerance 和 warm start 使用
`--red-qp-*`、`--red-proxqp-*`、`--yellow-qp-*`、`--yellow-proxqp-*` 参数。Cartesian 与
Joint planner 的 schedule budget 分别使用 `--cartesian-maximum-sample-count` 和
`--joint-maximum-sample-count`。这些值都会进入 `--dump-resolved-options`，并直接写入实际
solver/planner request。

五个 profile 默认使用 `/workspace/models/Psi_R1_visual_collision.urdf`。该 URDF 的 mesh
引用均为同目录下的 `meshes/<name>.obj`，不依赖 `products/synrobot` 中的 robot description。

控制链为：

```text
TCP goal -> nominal EE Cartesian OTG P/V/A
         -> EE-to-TCP control-point transform
         -> CartesianAdmittance + viewer drag wrench
         -> TCP-to-EE control-point transform
         -> HKS pose/twist -> joint target projection -> JointPlanner OTG
         -> committed q/qdot -> MuJoCo setKinematicState + forward
```

nominal planner sample 与 compliant command 分开保存。compliance offset 不反馈给
Cartesian planner，因此拖拽松开后会连续回到原规划轨迹。HKS 消费 compliant pose 和 twist；
compliant Cartesian acceleration 仅用于遥测和连续性检查，不会当作 joint acceleration。

控制点变换包含刚体偏置的完整线速度与线加速度项，其中 `r` 是 base frame 下从 EE 原点到
TCP 的向量：

```text
v_tcp = v_ee + omega x r
a_tcp = a_ee + alpha x r + omega x (omega x r)
```

MuJoCo 只消费 committed OTG joint position/velocity 并执行 `forward()`。本 app 不调用
`mj_step`，也不写 torque。viewer 刷新率不进入控制闭环。

安装后 headless smoke：

```bash
/workspace/install/algorithm/bin/mcl_hierarchical_kinematics_step \
  --profile planned-otg-nullspace-admittance-kinematic-sim teleop \
  --mujoco-model /workspace/install/algorithm/share/motion-control-lab/robots/r1/mujoco/mjcf/r1.xml \
  --no-mujoco-viewer --ui none --viz none --deadline-policy monitor --duration 0.25
```

或使用固定 profile 的 app-local Python recipe：

```bash
apps/hierarchical_kinematics_step/scripts/profiles/planned_otg_nullspace_admittance_kinematic_sim/run_keyboard.py \
  --no-mujoco-viewer --ui none --viz none --duration 0.25
```

每个 profile 提供 `run_keyboard.py`、`run_mcap_interactive.py`、`run_mcap_headless.py` 和
`run_csv_batch.py`；完整 launcher overrides 直接写在对应脚本的私有 `_RECIPE` 中。MCAP
interactive 使用 realtime、TUI、start-paused 和 Foxglove；MCAP headless 使用 batch 并关闭
TUI/Viz/viewer/terminal。`hierarchical` 的两个 MCAP recipe 要求显式 `--input`，其他 profile
继续使用默认 tracker fixture。

这些脚本是可执行 preset，不提供 Python import API。experiment 需要自己的配置时应创建自己的
`Recipe`，不要 import 或修改现有脚本中的 `_RECIPE`。脚本只依赖 Python 标准库。唯一支持的
环境变量是 `MCL_BINARY`、`MCL_INSTALL_PREFIX`、`MCL_LD_LIBRARY_PATH`、`MCL_CPU_SET`、
`MCL_RT_PRIORITY`，其余配置全部使用 argparse。所有 interactive recipe 默认绑定
`127.0.0.1:8765`；并行运行时用 `--port` 显式覆盖。

viewer 交互沿用运动学导纳 app：`Ctrl+左键` 平面拖动 TCP handle，
`Ctrl+Shift+左键` 深度拖动，`Ctrl+右键` 旋转；旋转导纳需显式启用
`--angular-admittance`。环境弹簧/阻尼和力限幅由 `--environment-*` / `--maximum-*`
参数配置，它们与 Core `CartesianAdmittance` 内部 M/D/K 是两组不同参数。

可观测性：

- `/mcl/cartesian/nominal/{left,right}`：planner nominal EE reference；
- `/mcl/cartesian/reference/{left,right}`：最终提交给 HKS 的 compliant EE reference；
- `/mcl/telemetry/compliance/cartesian`：每臂 TCP control point 的 nominal/command PVA、
  calculation/control-point frame、raw/filtered/applied wrench、offset、compliance
  twist/acceleration、饱和状态和 drag 状态。

pause/start-paused 会冻结 Cartesian planner、导纳、HKS 和 JointPlanner 整条计算链，`.` 单步让
整条链共同推进。导纳推进后的 HKS、JointPlanner 或 executed FK 失败为 fatal；导纳之前的
teleop Cartesian replan infeasible 保持原 app 的 recoverable 行为。拖拽释放和普通 retarget
不会隐式 reset filter 或 compliance state。

### Scaled 速度修正合同

五个 profile 均使用 MCC 默认的“速度修正完成比例”，无需开关。每拍先用当前 profile 的
request velocity 与位置、制动、速度、加速度硬边界最终交集构造共同基准
`qdot_b = clip(qdot_request, lower, upper)`，再求解
`J qdot = J qdot_b + s (v_ff + Kp error - J qdot_b)`。
两臂位置保留独立 scale，Secondary 的 soft orientation/posture/link4 不变。
`s=0` 表示基准运动，可能仍在运动；`s=1` 表示实现本拍目标速度，并不表示位置或姿态误差归零。
有限权重仍会与原有正则化折中，不承诺 scale 为数学可行区间的最大值。

`manifest.task_scale` 输出按 Red active joint 顺序排列的共同关节基准、最大投影改变量
与两臂位置基准速度；Null-space 页显示投影改变量、TCP 基准速度和修正比例语义。
任务等式残差/层级保持偏差与 TCP 真实跟踪误差分别显示。故障日志中的基准属于失败当拍。
Red raw IK request 与 OTG 执行状态沿用各 profile 现有来源，不增加上一拍命令缓存。
逐关节投影只对当前关节 box 与 scaled 任务提供共同可行起点；其他耦合硬约束不在此保证内。

`manifest.red_timing` 记录整段运行的 QP 求解、worker 执行与 release-to-finish 的 P95/P99
及最大值，并分别给出 Red deadline miss 和 skipped release。分位数是 1 us 直方图桶的上界；
最大值为实际测量值。worker 执行时间遵循 scheduler 的定义，不包含结束后的 observer。
`manifest.tracking` 分别记录实际跟踪误差、scaled 等式残差与已接受关节约束的违背量，
并记录最后的关节速度/加速度；scaled 残差为 L2 norm，验收仍逐分量使用原有容差。
回放结束与等待静止沿用各 profile 的现有行为。

## Native HARP posture reference

`--elbow-reference harp` consumes the optional MCC `posture_reference` library
in an ordinary C++ worker, using synchronized dual-wrist 30x18 windows and two arm-angle outputs. See
[HARP.md](HARP.md) for build, comparison profiles, fixed wrist calibration, recording and lifecycle checks.

## 公开批量执行入口

能力查询不加载模型或 solver：`mcl_hierarchical_kinematics_step --describe-capabilities`。
`--request FILE [--dump-resolved-options]` 使用 `execution_request.v1`，禁止额外 CLI override。
`app_config` 包含 `profile`、`target_space:frame|tcp` 与 `options`；后者的键为公开 CLI 名（不带 `--`），值为标量或布尔。
配置经过同一 `parseOptions`，任务经同一 `configureSolver`，调用同一 `SolverRuntime::solveRed/solveYellow`。
`tracking` 不参与配置或数值选择。

JSON `snapshot` / `trajectory` 对应 `hierarchical` profile。输入为 `joint_names`、可选 `initial_state:{q,v}` 和 `samples`。
每个 sample 可带 `q/v`、`source_time_s`、`targets.left/right:{position:[x,y,z],rotation:[[...],[...],[...]],enabled:true}`、`elbow_targets.left/right:[x,y,z]`。
未指定 Cartesian target 保持当前 FK；初始状态不裁剪。snapshot 使用该输入状态；trajectory 只提交原生接受的实际输出。
普通入口可用 `--profile hierarchical teleop --batch-input INPUT --batch-output NEWDIR --batch-settings SETTINGS`，与请求入口共用批量驱动。
`target_space` 默认 `frame`，指 app end-effector frame；`tcp` 指乘过 app TCP offset 的物理 TCP。普通入口使用 `--target-space`。
规划 profile 的公开请求通过原有 `runLoop` 的 CartesianPlanner/JointPlanner；JSON 由 app 将 frame/TCP 明确转换到物理 TCP CSV，并以公开 `--initial-state` 保留输入 q/v。原 CSV/MCAP `replay` 继续直接读取。
转换表和实际 solver target 分别留证，不根据实验编号选择坐标语义。

新增命名布局 `position-orientation-posture` 将 Yellow posture/link4 放到 Tertiary，位置 Primary、姿态 Secondary。
`primary-only` 禁用 orientation/link4/coupling 请求，任务仍以同一构造路径注册；默认 position-first 保持不变。

批量 `execution` 支持 `schedule:sync|divided|async_same_core|async_dual_core`、`timing_mode:virtual|real`、`yellow_divisor`、`red_cpu/yellow_cpu`。
真实 async 使用独立普通 Yellow worker 与最新状态邮箱；要求明确同核或双核身份。virtual 明确记录为模拟释放顺序，不证明线程实时性。
`planned_releases` 明确总释放数；`sample_selection:cycle|source_time` 固定输入选择规则。
`warmup_calls` 指 cold 之后 warm-up 调用数；每次调用保留，cold/steady 耗时在原生求解处测量。
迟到超过一周期保留 skipped；首次 Red 拒绝保留原始诊断及未运行后缀；不会改为通过。
输出包含 `raw.jsonl`、`model_mapping.json`、`resolved_options.json`、`summary.json`；请求另存来源哈希和追踪身份。

开发验收：`tests/execution_contract.py --binary /ABS/mcl_hierarchical_kinematics_step --output /ABS/NEWDIR`。

`observation.capacity` 为本次调用观测缓冲容量。容量不足保留 overflow 事件、失败状态、已采原始诊断及 not-run 后缀；不产生可用于性能判断的完整尾部分位数。

公开 `--raw-journal FILE` 默认关闭；请求运行自动启用 `native_calls.jsonl`。
日志在相同 `SolverRuntime::solveRed/solveYellow` 调用前后逐条 flush，包含输入状态/实际 task targets、原生状态、候选、各 pass 和 selected priority；异常前证据不会依赖退出时才生成的 trace。
日志的 `committed:false` 仅表示 solver 边界，规划/OTG 后的最终提交仍以原有 replay trace 为准。
观测在求解原生计时区间外；`call_elapsed_ns` 则包含公开 app 调用的观测成本。此同步 full-observation 路径仅用于明确标记的非 RT 证据，不宣称无观测扰动。

`observation.mode` 明确支持 `minimal|full|cpp-new`；普通 CLI 对应 `--observation-mode`。
minimal 保留每个调用的原生状态、候选 q/v、selected priority 和提交边界，省略重型任务/约束数组；full 完整保留；cpp-new 另外在原生 MCC 调用区间计数本线程 C++ `new/new[]`（含 aligned）次数和字节，不声称覆盖 `malloc`、Eigen 内部分配或其他线程。
未知 `app_config` / `execution` / `observation` 字段在 dump 前即拒绝。JSON 输入若声明 `model.locator/sha256`，必须与最终应用 URDF 一致。
规划 replay 的 execution 仅接受 `target_period_ms`，observation 仅接受 `mode`；批量 scheduling/capacity 不静默用于另一条链。

原 replay 在 worker 启动前保存 `replay/release_plan.json`，停止并 join 后保存 `release_counts.json`；请求 summary 和原生 manifest 引用这些证据。有限输入帧的计划、ReplaySource 原生选择/丢弃计数、未选择后缀独立列出；选择不是求解成功或控制提交。
控制 worker 没有固定调用计划，运行到 data-dependent settling、故障或 UI 停止；UI duration 也是停止条件，不能乘频率伪造计划调用数。因此 planned worker releases 与 not-run worker suffix 明确 unavailable。
Red/Yellow 的实际 callback iterations（含 idle/拒绝/异常，排除启动 warmup）、deadline misses 和原生 skipped counter 全部保留。现有 scheduler 的 skipped counter 统计超时后向前推进 release 的次数，包括推进到未来槽的一步，故不能直接与 callback iterations 相加当作精确 deadline 分母。本次未修改调度和旧计数，仅登记语义限制。recorded reference 的实际消费覆盖不能用 ReplaySource 计数替代。
