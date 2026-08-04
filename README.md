# Motion Control Lab

Motion Control Lab 是面向机器人遥操作 whole-body IK 的可复现实验仓库。研究愿景见
[docs/experiments.md](docs/experiments.md)，实验与证据生命周期见
[docs/experiment_architecture.md](docs/experiment_architecture.md)。

当前仓库提供第一条端到端纵向切片：

- 仓库内可直接修改的固定版本 placo C++ 源码；
- Experiment、run manifest 与 tidy metric 的首版合同；
- append-only run artifact store；
- `E01_placo_smoke` 实验及其合成二连杆 URDF；
- 对实验定义、run manifest、artifact 哈希和 placo 求解结果的自动检查。

仓库还提供可选的 MCC 交互预览运行时。它拥有 R1/MCC solver session、TUI 输入和
wall-clock pacing，并把算法快照映射为 `motion_control_viz::VisualizationFrame`。
该路径用于开发调试，不替代由 canonical timeline 驱动的可复现实验执行器。

Runner 依赖 Lab 自己的 `motion_control_lab::IkSolverBackend`，当前唯一实现仍是 MCC。
该 R1 范围的中立合同保留了 `backendId`、joint state/reset、目标、解、状态和诊断，供未来
PlaCo/MCC A/B 共用输入与结果映射。PlaCo adapter、双后端 A/B runner、CLI 选择和
差异判定目前均为 planned；默认应用没有增加后端选择，也没有改变原有开发流程。

E01 是基础设施 smoke test，只证明 placo C++ 求解链与证据落盘链能够跑通，不是
算法性能结论。

## 环境要求

当前开发环境以 Apple Silicon macOS 为基线，不需要 ROS2 或容器。

```bash
brew install cmake pinocchio
```

placo 源码已经保存在 `third_party/placo/`，配置时不下载 placo。首次配置仍需
联网获取固定版本的 eiquadprog 和 jsoncpp；Pinocchio 使用本机安装，这两个小型
依赖下载到 CMake build tree。

## 构建与测试

```bash
cmake --preset dev
cmake --build --preset dev --target e01_placo_smoke -j8
ctest --preset dev
```

手动运行 E01，并把证据写入 build tree：

```bash
./build/dev/e01_placo_smoke --output-root ./build/manual-runs
```

不指定 `--output-root` 时，run 会写入
`experiments/E01_placo_smoke/runs/`。每次执行创建新的
`<UTC timestamp>__<definition hash>` 目录，拒绝覆盖已有 run。

## 可选 MCC 交互预览

先分别安装 `motion_control_core` 和 `motion_control_viz`，再通过 CMake package
消费它们；Lab 不猜测 `../../components` sibling 路径：

```bash
cmake -S ../../components/motion_control_core -B /tmp/mcc_build \
  -DCMAKE_INSTALL_PREFIX=/tmp/mcc_install
cmake --build /tmp/mcc_build -j8
cmake --install /tmp/mcc_build

cmake -S ../../components/motion_control_viz -B /tmp/mcv_build \
  -DCMAKE_INSTALL_PREFIX=/tmp/mcv_install
cmake --build /tmp/mcv_build -j8
cmake --install /tmp/mcv_build

cmake -S . -B build/mcc-preview \
  -DMCL_BUILD_MCC_PREVIEW=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/mcv_install"
cmake --build build/mcc-preview \
  --target r1_single_arm_ik_tui_teleop r1_dual_arm_ik_tui_teleop -j8
```

从交互终端运行：

```bash
./build/mcc-preview/r1_single_arm_ik_tui_teleop \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20
./build/mcc-preview/r1_dual_arm_ik_tui_teleop \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 --mcap /new/run/path/preview.mcap
```

有 Foxglove sink 时连接 `ws://127.0.0.1:8765`。MCAP 路径存在时会拒绝覆盖。
默认不录制 MCAP，只有显式传入 `--mcap <path>` 才会录制；`--no-mcap` 可用于显式
关闭或覆盖前面给出的 `--mcap`。
若安装的 Viz 没有 Foxglove target，应用仍可用 null sink 运行。

旧的 Core planning matplotlib smoke app 已迁到 Lab，并通过
`MCL_BUILD_MCC_PLOTS=ON` 显式启用；它需要当前 Python 环境包含 NumPy 和
matplotlib。

## 目录

```text
adapters/execution/       通用 artifact store、manifest 与 SHA-256
adapters/interactive/     TUI 输入、交互式 runner 和 Viz sink composition
adapters/solver/          R1 IK backend-neutral contract；A/B runner planned
adapters/motion_control_core/ MCC solver session 与 R1 profile
apps/                     交互预览和可选 planning plot composition roots
contracts/                definition、manifest、metric 合同
data/raw/                 原始数据占位；不得静默改写
data/canonical/           规范数据占位
experiments/              Experiment 实例、run 与 reviewed result
analyses/                 只消费已有证据的 Analysis
docs/                     愿景、通用架构和项目实现映射
third_party/placo/        仓库内直接构建和修改的 placo 源码
tests/                    合同与端到端检查
```

具体实现与通用架构角色的对应关系见
[docs/project_mapping.md](docs/project_mapping.md)。

## placo 源码策略

placo 固定到 `v0.9.23` 对应提交
`e6c288604639d67b979a16cb2ad26913413c8e3a`，源码作为普通文件保存在
[`third_party/placo/`](third_party/placo/)；它不是 Git submodule，也不包含
嵌套 `.git` 目录。

为适配 Lab，CMake 关闭 Python bindings，源码中还包含当前 Pinocchio 4 /
Eigen 5 所需的 API 兼容修改。完整来源和修改清单见
[`third_party/placo/MOTION_CONTROL_LAB.md`](third_party/placo/MOTION_CONTROL_LAB.md)。

可以直接编辑 `third_party/placo/src/` 做算法实验。CMake 会直接重新编译这份
源码，并将 C++ build inputs 的内容指纹写入 run manifest 的
`environment.dependencies.placo`；兼容字段 `environment.placo_revision` 仍保留。
升级上游版本或修改算法后必须创建新的 run，
不能复用旧证据。
