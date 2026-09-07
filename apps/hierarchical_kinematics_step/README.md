# mcl_hierarchical_kinematics_step

这是五条历史 Red/Yellow HKS app 的单一入口。`--profile` 必选，并决定完整 topology；
阶段能力不能用独立开关组合：

- `hierarchical`：直接 target -> legacy HKS；
- `planned`：CartesianPlanner -> legacy HKS；
- `planned-otg`：CartesianPlanner -> legacy HKS -> JointPlanner；
- `planned-otg-nullspace`：双臂 Cartesian Primary > posture/link4 Secondary HKS + JointPlanner；
- `planned-otg-nullspace-admittance-kinematic-sim`：再增加导纳、MuJoCo 运动学投影、viewer
  与完整 replay telemetry gate。

`options.hpp` 是 app 的唯一 typed 配置入口，也完整持有 R1 robot 配置。可用
`--dump-resolved-options` 在加载模型前输出 profile、能力、robot、solver、planning、replay、
binary argv 与 launcher provenance 的完整 JSON。

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
