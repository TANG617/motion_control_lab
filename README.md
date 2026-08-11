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

仓库还提供可选的 MCC 交互预览运行时。单臂和双臂入口使用 grouped solver 的
`RedOnly` profile；三层双臂入口使用独立 Red/Yellow/Green worker。它们复用 TUI、Viz 和
wall-clock pacing；算法快照统一映射为
`motion_control_viz::VisualizationFrame`。
该路径用于开发调试，不替代由 canonical timeline 驱动的可复现实验执行器。
MCC builder、task 注册、typed handles、request 组装和状态更新直接位于各入口的
`main.cpp`，便于逐入口阅读和调试；共享层不定义 solver backend。

E01 是基础设施 smoke test，只证明 placo C++ 求解链与证据落盘链能够跑通，不是
算法性能结论。

## 环境要求

当前开发环境以 Apple Silicon macOS 为基线，不需要 ROS2 或容器。

```bash
brew install cmake pinocchio
```

placo 源码已经保存在 `third_party/placo/`，配置时不下载 placo。Pinocchio 使用本机安装，
jsoncpp 首次配置时按固定版本下载到 CMake build tree。若 `CMAKE_PREFIX_PATH` 中已有
eiquadprog package，Lab 会复用它；否则 standalone PlaCo 实验按固定版本构建 static fallback。

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

先按 MCC README 的 Strategy A 准备独立的 shared eiquadprog package（下例前缀为
`/tmp/eiq_install`），再分别安装 `motion_control_core` 和 `motion_control_viz`。Lab 通过
CMake package 消费三者，不猜测 `../../components` sibling 路径：

```bash
cmake -S ../../components/motion_control_core -B /tmp/mcc_build \
  -DCMAKE_INSTALL_PREFIX=/tmp/mcc_install \
  -DCMAKE_PREFIX_PATH=/tmp/eiq_install
cmake --build /tmp/mcc_build -j8
cmake --install /tmp/mcc_build

cmake -S ../../components/motion_control_viz -B /tmp/mcv_build \
  -DCMAKE_INSTALL_PREFIX=/tmp/mcv_install
cmake --build /tmp/mcv_build -j8
cmake --install /tmp/mcv_build

cmake -S . -B build/mcc-preview \
  -DMCL_BUILD_SINGLE_ARM_IK=ON \
  -DMCL_BUILD_DUAL_ARM_IK=ON \
  -DMCL_BUILD_GROUPED_DUAL_ARM_IK=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/mcc-preview \
  --target mcl_single_arm_ik mcl_dual_arm_ik mcl_grouped_dual_arm_ik -j8
```

每个入口可以独立配置。只构建单臂入口：

```bash
cmake -S . -B build/single-arm \
  -DMCL_BUILD_SINGLE_ARM_IK=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/single-arm --target mcl_single_arm_ik -j8
```

只构建双臂入口：

```bash
cmake -S . -B build/dual-arm \
  -DMCL_BUILD_DUAL_ARM_IK=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/dual-arm --target mcl_dual_arm_ik -j8
```

从交互终端运行：

```bash
./build/mcc-preview/mcl_single_arm_ik \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20
./build/mcc-preview/mcl_dual_arm_ik \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 --mcap /new/run/path/preview.mcap
./build/mcc-preview/mcl_grouped_dual_arm_ik \
  --urdf /path/to/Psi_R1_rev1.urdf \
  --red-rate 100 --yellow-rate 50 --green-rate 10 --ui-rate 20 \
  --deadline-policy monitor
```

三层入口固定使用 `RedYellowGreen` profile，要求
`red-rate > yellow-rate > green-rate > 0`。每组 period 同时是该 worker 的 deadline。默认
`--deadline-policy strict`：任一 rejected attempt、deadline miss 或 worker exception 都触发
first-writer Fault、停止并 join 三个 worker、保留最后 accepted Red state、关闭 sink 并返回非零。

macOS 等非实时开发环境可显式使用 `--deadline-policy monitor`。该模式记录 deadline miss 并继续，
同时跳过已经过期的 release，避免 worker 无间隔追赶；TUI 会显示各组累计 miss 和 skipped release。
rejected attempt 和 worker exception 在 monitor 模式下仍然触发 Fault。正式时序能力验证应使用
`strict`。

Green、Yellow、Red 启动前会顺序预热一次；正式 run 中三者完全异步，不等待 source 的下一条
结果。TUI、Viz 和 MCAP 只在 `ui-rate` 线程运行，不进入 Red solver 路径。

当前三层示例中，Red 保持双手 Hard Cartesian task；Yellow 使用高权重 Soft 双手 task，并对
`left_arm_link4`、`right_arm_link4` 添加弱 Soft 肘部外展偏好。肘部目标保持初始 X/Z，左侧 Y
增加 `0.25 m`、右侧 Y 减少 `0.25 m`。这是基于现有三轴 PositionTask 的近似，肘部权重仅为
`0.1`，不会取代 Hard joint limits 或 Green coupling。

也可以设置 `MOTION_CONTROL_URDF`，省略每次运行的 `--urdf`。

有 Foxglove sink 时连接 `ws://127.0.0.1:8765`。MCAP 路径存在时会拒绝覆盖。
默认不录制 MCAP，只有显式传入 `--mcap <path>` 才会录制；`--no-mcap` 可用于显式
关闭或覆盖前面给出的 `--mcap`。
若安装的 Viz 没有 Foxglove target，应用仍可用 null sink 运行。

旧的 Core planning matplotlib smoke app 已迁到 Lab，并通过
`MCL_BUILD_MCC_PLOTS=ON` 显式启用；它需要当前 Python 环境包含 NumPy 和
matplotlib。

## 笛卡尔 MoveLine 规划预览

`mcl_cartesian_planning` 是独立于 IK app 的 JSON 驱动预览入口。它只调用
`motion_control::core::CartesianMoveLinePlanner`，不读取 URDF、不创建 RobotModel，也不调用
FK/IK。多个 `segments` 表示同一次请求中的并行同步 frame move，不是连续路点。

先安装启用了 Foxglove 的 `motion_control_viz >= 0.3` 和 `motion_control_core`，然后单独构建：

```bash
cmake -S . -B build/cartesian-planning \
  -DMCL_BUILD_SINGLE_ARM_IK=OFF \
  -DMCL_BUILD_DUAL_ARM_IK=OFF \
  -DMCL_BUILD_GROUPED_DUAL_ARM_IK=OFF \
  -DMCL_BUILD_CARTESIAN_PLANNING=ON \
  -DCMAKE_PREFIX_PATH="/path/to/mcc-install;/path/to/eiq-install;/path/to/mcv-install"
cmake --build build/cartesian-planning --target mcl_cartesian_planning -j8
```

用仓库内的双 frame 示例生成完整 CSV、3D 路径图和运动曲线，并循环发布到 Foxglove：

```bash
./build/cartesian-planning/mcl_cartesian_planning \
  --request apps/cartesian_planning/example_request.json \
  --output-dir build/cartesian-planning/output
```

Foxglove 连接 `ws://127.0.0.1:8765`，场景位于 `/mc/cartesian/scene`，原始 pose 位于
`/mc/cartesian/pose/<frame_name>`。默认按规划时间循环播放至 Ctrl-C；`--once` 只播放一次，
`--playback-rate` 只改变展示速度。仅生成离线结果时使用：

```bash
./build/cartesian-planning/mcl_cartesian_planning \
  --request apps/cartesian_planning/example_request.json \
  --output-dir build/cartesian-planning/output \
  --no-live --force
```

请求合同见 `apps/cartesian_planning/request.schema.json`。位姿 quaternion 固定为 `xyzw`；
省略 `current_twist` 和 `current_acceleration` 时按零值处理。当前入口只支持 Core 已有的
MoveLine，不增加 circle、spline、blend 或 waypoint 拼接语义。

## 目录

```text
adapters/execution/       通用 artifact store、manifest 与 SHA-256
adapters/interactive/     共享 CLI、TUI、wall-clock scheduler 和 Viz helpers
apps/common/              仅共享无 topology 决策的 IK 工具和 R1 被动配置
apps/cartesian_planning/  JSON 驱动的纯 Cartesian MoveLine 规划、渲染和 Foxglove 回放
apps/<entry>/             直接拥有 MCC topology、solve/worker loop、main、CMake 和 help test
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
