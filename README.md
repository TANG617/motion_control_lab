# Motion Control Lab

> **仓库迁移通知**
>
> Motion Control Lab（MCL）已合并到
> [motion_control_playground](https://github.com/TANG617/motion_control_playground)，
> 在 `labs/motion-control-lab/` 下继续开发和维护，由该工作区直接管理源码。
> 后续代码更新、问题反馈和贡献请前往新仓库。
> 本仓库停止独立维护，将归档保留，供查询历史提交和复现实验版本。

MCL 是 R1 运动控制的原生应用与可复现实验仓库。具体 app 直接使用 MCC/PlaCo，
共享层负责输入、调度、展示和证据。Core/Sim/Viz 与生产系统具有独立生命周期。

## 选择入口

| app | 用途 |
|---|---|
| [baseline](apps/baseline/README.md) | 冻结的 PlaCo production-static 对照 |
| [step](apps/step/README.md) | 双臂 MCC/PlaCo ServoStep |
| [target](apps/target/README.md) | 双臂 MCC/PlaCo TargetSolve |
| [hierarchical_kinematics_step](apps/hierarchical_kinematics_step/README.md) | Red/Yellow、规划、OTG、HARP 与运动学仿真 |
| [planned_kinematics_step](apps/planned_kinematics_step/README.md) | CartesianTrajectoryGenerator → weighted IK → JointPtpTrajectoryGenerator |
| [joint_path_planning](apps/joint_path_planning/README.md) | 显式 IK → OMPL 关节避障 → 保持路径的时间参数化与 Foxglove 展示 |
| [optimization_problem](apps/optimization_problem/README.md) | 显式矩阵 QP/HQP 数值调试 |
| [hierarchical_inverse_dynamics_torque_sim](apps/hierarchical_inverse_dynamics_torque_sim/README.md) | 固定基座 R1 力矩仿真 |
| [psibot_teleop](apps/psibot_teleop/README.md) | 可选 Linux Psi SDK 客户端 |
| [replay_plan](apps/replay_plan/README.md) | 输入时间线和回放计划检查 |

## 最小构建

先安装匹配的 Core、Sim、Viz 及原生依赖，详见 [构建与运行](docs/build_and_run.md)。
从 Lab 根目录使用独立 CMake 安装：

```bash
cmake --preset dev -DCMAKE_PREFIX_PATH=/path/to/native/install -DCMAKE_INSTALL_PREFIX="$PWD/out/dev"
cmake --build --preset dev -j4
cmake --install build/dev
./out/dev/bin/mcl_optimization_problem --describe-capabilities
```

Workspace 用户从 `/workspace` 使用 `colcon build --packages-up-to motion_control_lab`；
app-local launcher 默认选择安装产物，`MCL_BINARY` 可显式指定独立构建产物。

## 文档与研究

从 [文档索引](docs/README.md) 进入架构、运行和合同说明。
[研究索引](docs/mcc_placo_study/README.md) 统一维护实验、分析与论文状态。
E05 批量回放见 [实验入口](experiments/E05_real_scene_planned_mcap_batch_replay/README.md)，
E06–E12 使用 [公开 app 执行链](tools/app_execution/README.md)。
跨电脑复跑的固定路径与输入快照见 [实验输入同步说明](experiments/README.md)。
历史设计、迁移与验收见 [归档](docs/archive/README.md)。
