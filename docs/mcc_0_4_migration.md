# MCC 0.4 app 迁移

MCL 的 MCC 调用方要求 `motion_control_core 0.4`，需要重新构建 core 和 Lab。各 app 继续直接
持有 solver 和 task，Red/Yellow 调度、层级、preservation、QP backend、gain、权重和限位策略保持原值。

| 调用方 | 迁移内容 |
|---|---|
| `mcl_step`、`mcl_single_arm_step` | `execution = ServoStepOptions{dt}`；Cartesian tolerance 放入 `convergence` |
| `mcl_target` | 迭代、soft budget 和 improvement 放入 `TargetSolveOptions`；启动时 posture 保存在 app 的 request 中，每次完整提交 |
| `mcl_hierarchical_kinematics_step` | Yellow/Red 显式配置 ServoStep；FK 配置 TargetSolve；Yellow 默认姿态按 active-joint 顺序存入 app request 并复用 |
| E03 legacy replay | 默认构建中的 MCC ServoStep 调用同步迁移配置字段 |

固定 posture 目标由 app 保存，不能改为每帧测量姿态，否则会改变原有回归默认姿态的行为。
`target` 和 Yellow 的 posture 仍只作 regularization，不注册收敛判据；diagnostics 的 role 为
`Regularization`、tolerance 为 0。Red 继续从 Yellow 的有效输出构造 coupling posture；尚无有效
输出时仍提交尺寸正确的目标并设置 `enabled=false`。

每次 MCC 请求包含全部已注册 position/orientation/posture handle，禁用任务也必须提交有效目标。
复用 request 时更新当次 state 和 Cartesian targets，保持固定 posture，避免重复追加目标。

## 删除的无效 ServoStep 参数

以下参数原来只写入 ServoStep 不使用的 TargetSolve 配置，现从 native CLI、Python launcher 和
resolved options 中删除。旧脚本需移除这些参数；继续传入会报告未知参数。

- `mcl_single_arm_step`：`--maximum-iterations`。
- `mcl_hierarchical_kinematics_step`：`--yellow-maximum-iterations`、
  `--minimum-position-improvement-m`、`--minimum-orientation-improvement-rad`。

`mcl_target` 的迭代和 improvement 参数仍有效，继续保留。`--red-proxqp-maximum-iterations`
控制 QP 后端迭代次数，也继续保留；它与 ServoStep 每次推进一步是不同层面的参数。

## 构建

工作区标准入口和 app-local 启动路径保持不变：

```bash
cd /workspace
COLCON_DEFAULTS_FILE=/workspace/.vscode/workspace/algorithm_colcon_defaults.yaml \
  colcon build --packages-up-to motion_control_lab
```

独立 CMake 构建需将新 core、sim、viz 和 shared eiquadprog 的安装 prefix 提供给
`CMAKE_PREFIX_PATH`。运行该构建的 app 时显式指定 `MCL_BINARY`，避免继续运行旧 install 产物。
本机不使用 `/workspace` 布局时，可通过 `-DE01_MODEL=/absolute/path/r1.cos.urdf` 指定 E01 与
双臂 solver regression 使用的同一 URDF；默认值仍是 `/workspace/models/r1.cos.urdf`。

## 2026-09-07 本机验证

使用独立 `/tmp/mcl-api04-build`，Release、macOS arm64、AppleClang 21、Pinocchio 4.1、
MuJoCo 3.11、Protobuf 7.35.1。core 来自此次重构的 0.4 安装 prefix；`otool -L` 确认 apps
链接 `libmotion_control_core.0.4.dylib`。TUI、Foxglove transport 与可选 plotting 均启用。
Docker daemon 不可用，未执行 `/workspace` 容器的 colcon gate。

为了在本机验证，临时 prefix 中安装了 sim，MuJoCo 来自独立 Python 环境，补齐其动态库搜索路径；
Matplotlib 放在独立临时 Python package 目录。本机构建沿用 core 验收时的 Pinocchio 4.1
`pinocchio::fcl` preinclude；这些临时环境不写入 app 启动脚本。

| 项目 | 结果 |
|---|---|
| 全部 9 个 app、默认 E03 和测试目标 | 编译通过 |
| app/component 静态边界 | 通过 |
| single-arm / step / target options 与 dual-arm CLI | 通过；废弃的 single-arm iterations 参数被拒绝 |
| Step / Target solver semantics | 两个 MCC backend 和 PlaCo 路径通过；Target 请求复用、拒绝后恢复通过 |
| HKS options | 通过；删除的三个参数被拒绝，显式 mesh search path 合同通过 |
| HKS Python launcher | 20 个 profile/source recipes 及 native 参数合同通过 |
| HKS solver | 用本机实际 `models/Psi_R1_visual_collision.urdf` 运行通过；固定 posture、位移恢复、无 Yellow 值时的禁用 coupling、非法 Yellow 请求后恢复均覆盖 |
| HKS control / joint-target / retarget-clamp / visualization / telemetry / TUI projection / rejection-policy / replay-pipeline | 通过 |
| 单臂短运行 | 在 PTY 下 `--ui none --viz none --duration 0.15` 正常退出；该 app 的键盘输入仍要求 TTY |
| HKS planned 短运行 | 实际 URDF、`--ui none --viz none --duration 0.15` 正常退出 |
| E03 replay IK integration、Cartesian planning smoke | 通过 |

构建还暴露并修正了 app 内两处兼容问题：将 JSON 的 `size_t` 关节索引显式转换为
`Json::UInt64`，以及将新版 Protobuf descriptor 的 string view 显式转换为 `std::string`。
HKS options 测试改用显式 mesh search path，不再假设宿主机存在 `/workspace/models/meshes`。

**全量 Lab CTest 尚未全部通过。** 首次 88 项运行是 56 通过、32 失败；修复临时 MuJoCo 动态库
搜索路径后，HKS subcommands 复测通过，修正环境依赖的 options 测试后该项复测通过。
HKS solver 注册的 CTest 仍写死 `/workspace/models/Psi_R1_visual_collision.urdf`，其相同测试
binary 已用本机路径单独通过。其余未解决结果包括：

- E01/E04/baseline 等测试与 manifests 的 `/workspace` 路径依赖，E03 library directory 缺失。
- macOS `script` 不支持测试脚本使用的 `-c`；终端 PTY 测试失败或超时。
- `baseline_tui_projection` 的现有显示断言失败。
- MuJoCo R1 model consistency / torque-sim 测试报告 collision mesh `at least 4 vertices required`。

这些结果没有计作通过，未修改机器人 mesh、动力学行为、PTY 驱动或实验 manifests 来消除失败。
本次也未做硬件、Linux SDK 或容器运行验证。

本机原始证据：`/tmp/mcl-api04-build-delivery.log`、`/tmp/mcl-api04-ctest.log`、
`/tmp/mcl-api04-options-retest.log`、`/tmp/mcl-api04-loader-retest.log`、
`/tmp/mcl-api04-hks-solver.log` 和 `/tmp/mcl-api04-hks-smoke.log`。
