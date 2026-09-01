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

`scripts/profiles/<profile>/config.py` 完整列出该 profile 的 Python launcher overrides；每个
profile 提供 `run_keyboard.py`、`run_mcap_interactive.py`、`run_mcap_headless.py` 和
`run_csv_batch.py`。MCAP interactive 使用 realtime、TUI、start-paused 和 Foxglove；MCAP
headless 使用 batch 并关闭 TUI/Viz/viewer/terminal。`hierarchical` 的两个 MCAP recipe 要求
显式 `--input`，其他 profile 继续使用默认 tracker fixture。

每个 recipe 模块均导出 `build_command(argv)` 与 `run(argv)`，供 experiments import；profile
和 source 已由模块固定，不能通过参数改写。模块只依赖 Python 标准库。唯一支持的环境变量是
`MCL_BINARY`、`MCL_INSTALL_PREFIX`、`MCL_LD_LIBRARY_PATH`、`MCL_CPU_SET`、
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
