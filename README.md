# Motion Control Lab

Motion Control Lab 是面向机器人遥操作 whole-body IK 的可复现实验仓库。研究愿景见
[docs/experiments.md](docs/experiments.md)，实验与证据生命周期见
[docs/experiment_architecture.md](docs/experiment_architecture.md)，app 与 TUI、Viz、scheduler、
teleop、replay 的现行职责边界见
[docs/app_component_architecture.md](docs/app_component_architecture.md)。

当前仓库提供第一条端到端纵向切片：

- 仓库内可直接修改的固定版本 placo C++ 源码；
- Experiment、run manifest 与 tidy metric 的首版合同；
- append-only run artifact store；
- `E01_placo_smoke` 的 PlaCo R1 IK smoke；
- 可选、隔离构建的 `E04_opensot_smoke` OpenSoT ROS2 R1 IK smoke；
- 对实验定义、共享 R1 输入、run manifest、artifact 哈希和求解结果的自动检查。

仓库还提供可选的 IK 交互预览运行时。单臂入口直接使用普通 MCC `KinematicsSolver`；
显式区分的双臂 ServoStep/TargetSolve 入口可以通过 `--solver mcc|placo` 选择实现，并在 MCC
实现中通过 `--backend proxqp|eiquadprog` 选择 QP backend；各 app 拥有完整的 solver topology、
任务和结果解释。hierarchical 双臂入口使用 MCC 独立 Red/Yellow worker。它们只复用 R1 固定参数、
TUI 与 wall-clock pacing；Lab-owned IK projection 把算法快照映射为通用
`motion_control_viz::RenderBatch`，再交给 WebSocket、MCAP 或 Null transport。
`mcl_baseline` 是独立的 PlaCo-only production-static 对照入口：它冻结生产
`motion_control` revision `42ed3ce3a19f5a7346874a31ec659c0298751137` 的有效任务、权重、
初始姿态、关节 mask、限位和 TargetSolve 终止条件，不读取 `/etc/robot/software.yaml`，也不
启用 adaptive reach、continuity 或 elbow-pole 动态策略。
所有 IK app 的 Foxglove topic 与 FK 一致性要求由
[MCL Foxglove Topic 与 Telemetry 合同](docs/foxglove_mcl_telemetry_contract.md)统一定义。
该路径用于开发调试，不替代由 canonical timeline 驱动的可复现实验执行器。
共享层已拆为 solver-neutral 的 input/presentation/runtime contracts，以及 scheduler、terminal
frontend/key router、keyboard/cartesian teleop、TUI renderer、ReplaySource、preview
projection/transport、run artifacts、R1 config、机械 helpers 和薄 App Scaffolding targets。
每个交互 app 直接拥有自己的 backend topology、任务、solver 配置、可选 planning、主循环、
诊断解释以及 TUI/Viz snapshot；运动控制 app 以 `main/options/solver/[planning]/loop` 为主要
结构，只有实际规划职责才建立 `planning.*`，即使配置相同也在各自目录中显式保留。旧 `apps/common/` 与
`adapters/interactive/` 聚合层已经删除。MCAP/CSV 经相同 typed pipeline 和单一
`ReplaySource`；solver loop、trace 解释与失败语义仍由各 app 自己拥有。

E01 和 E04 都固定使用 `/workspace/models/r1.cos.urdf` 及同一左臂可达位置任务，
分别验证 PlaCo 和 OpenSoT 求解链与证据落盘链，不构成算法性能结论。

## 环境要求

当前开发环境以 Apple Silicon macOS 为基线，不需要 ROS2 或容器。

```bash
brew install cmake pinocchio
```

placo 与 FTXUI 源码已经分别保存在 `third_party/placo/` 和 `third_party/ftxui/`，配置时不下载
这两个依赖。Pinocchio 使用本机安装，jsoncpp 首次配置时按固定版本下载到 CMake build tree。
若 `CMAKE_PREFIX_PATH` 中已有
eiquadprog package，Lab 会复用它；否则 standalone PlaCo 实验按固定版本构建 static fallback。
OpenSoT 源码位于 `third_party/OpenSoT/`，默认不开启；只有显式设置
`MCL_BUILD_E04_OPENSOT_SMOKE=ON` 时才在独立嵌套工程中使用 ROS Jazzy、XBot2、
MoveIt 和 C++20 构建，Lab 目标继续保持 C++17。

## 构建与测试

### Workspace colcon 构建与运行产物

> 本节更新日期：2026-08-26

在 Motion Control Workspace 中，Lab 的标准构建入口是 `/workspace` 下的 `colcon build`。
当前 algorithm defaults 将 build tree 放在 `/workspace/build/algorithm`，将 merged install tree
放在 `/workspace/install/algorithm`：

```bash
cd /workspace
COLCON_DEFAULTS_FILE=/workspace/.vscode/workspace/algorithm_colcon_defaults.yaml \
  colcon build --packages-up-to motion_control_lab
```

所有 app-local 启动脚本默认执行
`${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_<app>`，因此上述构建完成后无需再对
`labs/motion-control-lab/build/` 执行第二次 standalone build。启动脚本不会隐式构建，也不会因为
Lab standalone build 中存在同名旧文件而选择它。需要验证其他 install prefix 时设置
`MCL_INSTALL_PREFIX`；需要有意运行 standalone 或临时 executable 时显式设置 `MCL_BINARY`：

```bash
MCL_BINARY=/workspace/build/algorithm/motion_control_lab/mcl_hierarchical_kinematics_step \
  apps/hierarchical_kinematics_step/scripts/run_keyboard.py \
  --profile planned-otg-nullspace
```

`COLCON_PREFIX_PATH` 用于已安装 package 的发现，不决定本次 build/install 目录，也不决定启动脚本
选择哪个 executable；这些分别由 `COLCON_DEFAULTS_FILE` 与上述 binary 路径合同决定。不要把
`/workspace/build/algorithm/motion_control_lab`、`/workspace/install/algorithm` 和
`labs/motion-control-lab/build` 的时间戳或测试结果混为同一构建。

standalone CMake 仍是受支持的独立开发路径，但必须显式选择其 executable：

```bash
cmake --preset dev
cmake --build --preset dev --target e01_placo_smoke -j8
ctest --preset dev
```

不构建 FTXUI 与 Foxglove transport 的 headless/Null sink 组合是受支持的独立构建形状：

```bash
cmake -S . -B build/headless \
  -DBUILD_TESTING=ON \
  -DMCL_ENABLE_TUI=OFF \
  -DMCL_ENABLE_FOXGLOVE_TRANSPORT=OFF
cmake --build build/headless -j8
ctest --test-dir build/headless --output-on-failure
```

此配置仍使用 `motion_control_viz::RenderBatch` 通用合同，但不链接 Foxglove transport，也不
启动网络端口。需要 TUI 的 PTY tests 只在 `MCL_ENABLE_TUI=ON` 时注册。

## IK app CPU affinity（Linux）

IK app 在 Linux 构建中默认启用固定 CPU affinity；非 Linux 构建默认关闭。Linux 上需要
显式的非绑核调试构建时，可在配置阶段传入 `-DMCL_ENABLE_CPU_AFFINITY=OFF`：

```bash
cmake -S . -B build/no-affinity -DMCL_ENABLE_CPU_AFFINITY=OFF
cmake --build build/no-affinity -j8
```

已有 build 目录继续使用其 CMake cache；从旧的默认关闭配置迁移时，需要在该目录重新配置一次
`-DMCL_ENABLE_CPU_AFFINITY=ON`。

CPU 编号由各 app 的源码常量持有，不提供运行时覆盖；TUI 会显示当前编译版本实际请求和
生效的 CPU。启用后，若请求 CPU 不在进程启动时的 cgroup/cpuset allowed set 内，app
会立即失败并打印请求、允许和实际 CPU 集。该能力只限制线程运行位置，不负责 CPU 独占、
SMT sibling 隔离、IRQ affinity 或实时调度优先级。交互 IK 的 TUI System 页显示每个角色的
启用状态、线程 ID、requested CPU 和内核核验后的 effective CPU；显式关闭的构建明确显示
`disabled`，worker 完成绑定前显示 `pending`。

所有交互 IK 的 Solver 与 System 页还会显示 IK calculation time 的
90th、95th、99th percentile。
统计采用 nearest-rank 定义和最近 4096 次实际求解的固定容量滑动窗口，同时显示窗口样本数
与进程启动后的累计样本数；暂停或 Grouped worker idle 不会生成零耗时样本。
在 MCC/PlaCo 比较中，只有包含 backend 调用与 app 侧迭代/结果提取的总 IK 耗时及其分位数是
跨后端同口径性能指标。MCC 的 QP timing、residual、active set 和 warm start 等内部诊断不与
PlaCo 对比；PlaCo 未公开的字段在 TUI 中显示 `-`，不写成数值零。

## ROS-free 数据源与 canonical replay

Lab 的 replay 输入链分为五个单向依赖的层次：

```text
McapSource / CsvSource
        -> typed decoder -> TypedStream<StampedPose / StampedJointState>
        -> 原始时间校验与 stream alignment
        -> DualArmTimeline semantic projector
        -> preserve / fixed-period timestamp projection
        -> ReplayClock -> headless IK runner
```

- `adapters/data/source/` 只理解 MCAP 的 channel/schema/chunk/index/zstd 和 CSV 的
  header/row；source 不解码 ROS message，也不知道 dual-arm 或 IK。
- `adapters/data/decoder/` 负责 ROS2 CDR 或可配置 CSV 列到 typed sample 的转换。
  CDR 路径不依赖 ROS2、rclcpp、rosbag2 或生产消息包；typed sample 分别保留
  header、log、publish 和 configured-column 时间 provenance。
- `adapters/data/temporal/` 只处理 typed stream 的时间选择、严格单调校验和最终
  projection；不存在静默 timestamp fallback。
- `adapters/data/projection/` 在原始 logical timestamp 上完成 exact/nearest 双臂配对，
  不感知输入来自 MCAP 还是 CSV。
- `ReplayClock` 的 realtime deadline 是
  `run_start + projected_time / playback_rate`，使用 monotonic absolute deadline；
  batch 模式不 sleep。

`fixed-period` 是一对一 retime：第 `i` 个已配对 semantic frame 的 projected time 为
`i * period_ns`。它不插值、不生成或丢弃 sample、不改变 frame 顺序，也不改写 sample
内部任何原始 timestamp。左右流必须先按原始 logical timestamp 完成配对，之后才能
retime；真正的 resampling/interpolation 若需要，应作为未来独立 transform。

默认构建包含不依赖 Core/Viz 的 inspect/replay-plan 入口：

```bash
./build/dev/mcl_replay_plan \
  --input /path/to/input.mcap \
  --input-format mcap \
  --left-stream /mc/ik/target/left_pose \
  --right-stream /mc/ik/target/right_pose \
  --timestamp-source header_stamp \
  --timestamp-policy fixed-period \
  --period-ms 10 \
  --execution-mode batch \
  --output-dir /tmp/mcl-replay-plan
```

它会在运行时钟开始前完整读取、解压、解码和投影输入，并写出 `trace.csv` 与带输入
SHA-256、decoder、时间/配对策略和统计的 `manifest.json`。

CSV 使用同一组上层参数，将 `--input-format` 改为 `csv`。未提供 `--csv-mapping` 时，
canonical dual-arm CSV 使用 `timestamp_ns` 以及 `left_frame_id,left_x,...,left_qw`、
`right_frame_id,right_x,...,right_qw`。自定义列使用 JSON：

```json
{
  "schema_version": "mcl.csv_mapping.v1",
  "streams": {
    "left": {
      "timestamp_column": "time_ns",
      "timestamp_target": "header_stamp",
      "frame_id_column": "reference_frame",
      "columns": {"x":"lx","y":"ly","z":"lz","qx":"lqx","qy":"lqy","qz":"lqz","qw":"lqw"}
    },
    "right": {
      "timestamp_column": "time_ns",
      "timestamp_target": "header_stamp",
      "frame_id_column": "reference_frame",
      "columns": {"x":"rx","y":"ry","z":"rz","qx":"rqx","qy":"rqy","qz":"rqz","qw":"rqw"}
    }
  }
}
```

E02 现在直接使用普通 Step app 的 replay 子命令；headless 与 TUI replay 使用同一套
solver/task 配置：

```bash
cmake -S . -B build/replay-ik \
  -DMCL_BUILD_SINGLE_ARM_STEP=OFF \
  -DMCL_BUILD_STEP=ON \
  -DMCL_BUILD_TARGET=OFF \
  -DMCL_BUILD_HIERARCHICAL_KINEMATICS_STEP=OFF \
  -DCMAKE_PREFIX_PATH="/path/to/mcc-install;/path/to/eiq-install"
cmake --build build/replay-ik --target mcl_step -j8
```

`mcl_step replay` 支持 `batch|realtime`，并要求显式提供
`--target-period-ms`。第 `i` 帧的 projected time 固定为 `i * target_period`，不插值也不改变
帧数；solver 周期由独立的 `--rate` 定义。MCAP 输入默认读取
`/mc/ik/joint_states` 的第一帧，按 joint name 映射为初始
position（初始 velocity 仍为零）；可用 `--initial-joint-state-stream` 选择其他 JointState
topic。R1 target stream 的输入语义是 TCP pose；runner 使用左右各自的 `0.1 m` TCP offset
转换为 end-effector target 后再构造 IK request。replay solver 消费最新 target，允许丢弃
尚未消费的旧帧并在 `ik_replay_manifest.v2` 中记录计数。`--ui tui`（默认）保留 renderer，
Space 暂停/继续，`.` 单帧推进。`--ui none` 只禁用渲染；replay 的 terminal input 由正交的
`--terminal-input/--no-terminal-input` 控制，所以 headless 模式也可以按需保留 pause/resume/step。

不指定 `--output-dir` 时，runner 会在
`experiments/E02_dual_arm_replay_ik/runs/` 下创建新的
`<UTC timestamp>__<definition hash>` 目录。`--output-root` 可以只重定向 run 父目录，
`--run-id` 可用于可控测试；`--output-dir` 保留为一次性调试的精确目录覆盖。

实时观察不需要另一个 binary；选择 TUI 和 Foxglove 参数即可：

```bash
cmake -S . -B build/replay-ik-viz \
  -DMCL_BUILD_SINGLE_ARM_STEP=OFF \
  -DMCL_BUILD_STEP=ON \
  -DMCL_BUILD_TARGET=OFF \
  -DMCL_BUILD_HIERARCHICAL_KINEMATICS_STEP=OFF \
  -DCMAKE_PREFIX_PATH="/path/to/mcc-install;/path/to/eiq-install;/path/to/mcv-install"
cmake --build build/replay-ik-viz --target mcl_step -j8
```

运行 `mcl_step replay --execution-mode realtime --ui tui ...`，Foxglove 连接
`ws://127.0.0.1:8765`。如需记录 visualization stream，传入 `--mcap <path>`；否则使用
`--no-mcap`。该 MCAP 只是可选开发预览录制，不是正式实验 trace、manifest 或 artifact。
完整 E02 命令见
[`experiments/E02_dual_arm_replay_ik/README.md`](experiments/E02_dual_arm_replay_ik/README.md)。
五条固定 topic 见
[MCL Foxglove Topic 与 Telemetry 合同](docs/foxglove_mcl_telemetry_contract.md)。

E03 将同一个固定 replay IK engine 应用于 PSI R1 双臂动作库。默认只扫描
`/workspace/fixtures/datasets/psi_r1_dual_arm_motion_library/` 的直接 `*.mcap` 子项，按文件名
排序并在运行前固化 size/SHA-256。每个动作从自己的首帧 JointState 独立初始化，首个错误后
停止该动作，但批次继续执行后续动作；只有所有动作完成且没有 rejected solve 时批次才成功。

```bash
cmake --build /workspace/build/motion_control_lab --target mcl_e03_batch_replay_ik -j8
/workspace/build/motion_control_lab/mcl_e03_batch_replay_ik \
  --urdf /workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf
```

也可通过
`experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/scripts/run_mcap_replay.sh` 启动，使用
`MCL_LIBRARY_DIR`、`MCL_OUTPUT_ROOT`、`MCL_URDF` 设置 batch preset，并以 trailing arguments
覆盖本次运行参数；manifest 会记录 launcher、原始 argv、resolved config 和每个 action 的
size/SHA-256。

默认产物位于
`experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/runs/<run-id>/`。增加
`--visualize --playback-rate <rate>` 后会用一个 Foxglove server 连续播放整个动作库，不等待
空格，也不默认录制 visualization MCAP。完整合同和目录结构见
[`experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/README.md`](experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/README.md)。

手动运行 E01，并把证据写入 build tree：

```bash
./build/dev/e01_placo_smoke --output-root ./build/manual-runs
```

不指定 `--output-root` 时，run 会写入
`experiments/E01_placo_smoke/runs/`。每次执行创建新的
`<UTC timestamp>__<definition hash>` 目录，拒绝覆盖已有 run。

手动构建和运行 E04：

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build/opensot -DMCL_BUILD_E04_OPENSOT_SMOKE=ON
cmake --build build/opensot --target e04_opensot_smoke -j8
ctest --test-dir build/opensot --output-on-failure -R 'contracts.r1_smoke_pair|experiments.e04_opensot_smoke'
```

## IK 交互预览与 production baseline

### PlaCo production-static baseline

默认构建包含 `mcl_baseline`；它没有 `--solver` 或 `--backend` 选项。teleop 固定以 100 Hz
运行，外部目标保持 TCP 语义，进入 PlaCo task 前去除左右各 `0.1 m` TCP offset：

```bash
cmake --build build/mcc-preview --target mcl_baseline -j8
./build/mcc-preview/mcl_baseline teleop \
  --urdf /workspace/models/r1.cos.urdf \
  --ui tui --no-mcap
```

Replay 固定要求 `--target-period-ms 10`，并始终从冻结的生产 initial pose 启动：

```bash
./build/mcc-preview/mcl_baseline replay \
  --urdf /workspace/models/r1.cos.urdf \
  --input /path/to/targets.csv --input-format csv \
  --left-stream left --right-stream right \
  --timestamp-source csv_timestamp --target-period-ms 10 \
  --execution-mode batch --ui none --no-mcap \
  --output-dir /tmp/mcl-baseline-run
```

每个 replay run 写出 `baseline_config.json`、`trace.csv`、`status.json` 和 `manifest.json`。
配置 artifact 使用 `mcl.placo_baseline_config.v1`，manifest 固化其 SHA-256；trace 同时保存公共
TCP target、内部 EE task target、EE/TCP FK、两套误差、frame scale、20 关节状态和 PlaCo
velocity。Foxglove MCAP 仍只承载现有 IK 可视化合同，不承载这些诊断字段。
为保证跨 run 的逐字段确定性，trace 中除 `solve_time_ms` 外的浮点诊断统一规范化为 12 位小数。

### 可选 MCC 交互预览

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
  -DMCL_BUILD_SINGLE_ARM_STEP=ON \
  -DMCL_BUILD_STEP=ON \
  -DMCL_BUILD_TARGET=ON \
  -DMCL_BUILD_HIERARCHICAL_KINEMATICS_STEP=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/mcc-preview \
  --target mcl_single_arm_step mcl_step \
    mcl_target mcl_hierarchical_kinematics_step -j8
```

每个入口可以独立配置。只构建单臂入口：

```bash
cmake -S . -B build/single-arm \
  -DMCL_BUILD_SINGLE_ARM_STEP=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/single-arm --target mcl_single_arm_step -j8
```

只构建两个双臂入口：

```bash
cmake -S . -B build/dual-arm \
  -DMCL_BUILD_STEP=ON \
  -DMCL_BUILD_TARGET=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/dual-arm \
  --target mcl_step mcl_target -j8
```

从交互终端运行：

```bash
./build/mcc-preview/mcl_single_arm_step \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20
./build/mcc-preview/mcl_step teleop \
  --solver mcc --backend proxqp \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 \
  --mcap /new/run/path/step-mcc-proxqp.mcap
./build/mcc-preview/mcl_step teleop \
  --solver mcc --backend eiquadprog \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 \
  --mcap /new/run/path/step-mcc-eiquadprog.mcap
./build/mcc-preview/mcl_step teleop \
  --solver placo --urdf /path/to/Psi_R1_rev1.urdf --rate 20 \
  --mcap /new/run/path/step-placo.mcap
./build/mcc-preview/mcl_target \
  --solver mcc --backend proxqp \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 \
  --mcap /new/run/path/target-mcc-proxqp.mcap
./build/mcc-preview/mcl_target \
  --solver mcc --backend eiquadprog \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 \
  --mcap /new/run/path/target-mcc-eiquadprog.mcap
./build/mcc-preview/mcl_target \
  --solver placo --urdf /path/to/Psi_R1_rev1.urdf --rate 20 \
  --mcap /new/run/path/target-placo.mcap
./build/mcc-preview/mcl_hierarchical_kinematics_step \
  --profile hierarchical teleop \
  --urdf /path/to/Psi_R1_rev1.urdf \
  --ui none --viz none \
  --deadline-policy monitor
```

所有交互 app 复用 solver-neutral 的标准 IK 文档构造和同一个只消费 `TuiDocument` 的
FTXUI renderer；planned-grouped presenter 按 solver-neutral snapshot capability 增加 Joint
Planning、OTG、projection/clamp 内容，MCC diagnostics 的解释仍留在 app。
planned-grouped options 嵌入共享 `PlannedGroupedTuiConfig`，loop 只调用
`tui.handleNavigation(event)` 与 `tui.render(snapshot)`。
页面数量按快照能力为 5、6 或 7，数字键、对应的 F1–F7 和 `Tab` 动态切换；
`PageUp/PageDown/Home/End` 滚动当前页面，`h` 或 `?` 打开完整快捷键帮助。完整布局以
`155×74` 为验收尺寸，更小窗口继续使用滚动。

两个双臂入口分别定义同值的双手 Hard position/orientation task 和 Hard joint-position limits；
这种重复用于保持 app 独立，不抽取到共享 component。它们只共享 renderer、transport 与 R1
固定参数。
`--solver` 默认为 `mcc`；该选择在进程启动时确定，不提供热切换或双求解器并跑。
`--backend <proxqp|eiquadprog>` 默认为 `proxqp`，只选择 MCC 的 QP backend；当
`--solver placo` 时该参数会被解析但不生效，PlaCo 始终使用自身的 eiquadprog。MCC 与 PlaCo
都固定 floating base，并只控制 R1 配置列出的 20 个关节。position-limit margin 与数值
regularization 的编译默认分别为 `1e-3` 和 `1e-4`，非 baseline app 可以通过自己的
app-local flags 覆盖。ServoStep 每次只执行一个 QP update，`--rate` 同时定义其正
`servo_period`/PlaCo `dt`，并启用 Hard joint-velocity limits。TargetSolve 不注册 velocity limits，
增加默认权重 `1e-5` 的 initial-pose Soft joints task；默认最多迭代 10000 次，并使用
100 ms soft budget、`1e-4` Cartesian tolerance 和 `1e-8` minimum-improvement 终止规则。
这些实验参数由 `apps/target/options.*` 解析；其 `--rate` 只控制交互求解和发布频率。

各求解路径复用相同 TUI 和五个 IK/FK Viz 通道。TUI 标题及 Solver 页明确显示
`MCC/ProxQP`、`MCC/eiquadprog` 或 `PlaCo/eiquadprog`；对应的 visualization run ID 仍按
solver implementation 分别为
`interactive-preview-mcc` 与 `interactive-preview-placo`。通用 debug frame 记录总 IK 耗时、
90th/95th/99th percentile、迭代/收敛、Attempts/Accepted/Rejected 计数以及双臂 Cartesian
error，便于分别
运行两次后按同一口径观察。

hierarchical 入口使用 app-local `RedYellow` profile，要求 `red-rate > yellow-rate > 0`，默认分别为
1000 Hz 和 100 Hz。每组 period 同时是该 worker 的 deadline。默认
`--deadline-policy strict`：任一 rejected attempt、deadline miss 或 worker exception 都触发
first-writer Fault、停止并 join 两个 worker、保留最后 accepted Red state、关闭 sink 并返回非零。

macOS 等非实时开发环境可显式使用 `--deadline-policy monitor`。该模式记录 deadline miss 并继续，
同时跳过已经过期的 release，避免 worker 无间隔追赶；TUI 会显示各组累计 miss 和 skipped release。
rejected attempt 和 worker exception 在 monitor 模式下仍然触发 Fault。正式时序能力验证应使用
`strict`。

Yellow、Red 启动前会顺序预热一次；正式 run 中两者完全异步，不等待 source 的下一条
结果。TUI、Viz 和 MCAP 只在 `ui-rate` 线程运行，不进入 Red solver 路径。

`mcl_hierarchical_kinematics_step` 用必选 `--profile` 保留五条历史数据流：
`hierarchical`、`planned`、`planned-otg`、`planned-otg-nullspace`、
`planned-otg-nullspace-admittance-kinematic-sim`。前三个 profile 保留原 shared Cartesian
scale/legacy Red-Yellow topology；后两个使用严格 position > orientation > posture/link4
三级 hierarchy。planner、JointPlanner、nullspace、admittance、MuJoCo 与 telemetry 都只由
profile 能力推导，不能通过独立阶段开关组成非法数据流。

Yellow 当前使用 4 对 app-local R1 link pair 的 Soft self-collision velocity damping：minimum
distance `0.30 m`、influence distance `0.35 m`、gain `2 s^-1`、weight `100`；posture task 当前未
注册。Yellow accepted proposal 通过 weight `10` 的内部 coupling 进入 Red。self-collision 是
运动优化目标，不是硬安全屏障；
margin shortfall 不会自动拒绝 accepted solution，硬件 command authorization 仍由集成层负责。

最长 profile 在 planner 后、HKS 前执行双臂 TCP 导纳，并以 committed JointPlanner OTG 状态
驱动 MuJoCo 运动学投影；MuJoCo 不调用 `mj_step`，不进入 torque/dynamics 闭环。完整 profile、
配置、P/V/A 控制点变换、失败策略、viewer 操作和 telemetry 见
`apps/hierarchical_kinematics_step/README.md`。

`mcl_hierarchical_inverse_dynamics_torque_sim` 是独立的 ROS-free 固定基 R1 torque-driven app：
`CartesianPlanner P/V/A -> optional admittance -> HID -> qfrc_applied -> one mj_step -> measured feedback`。
其 keyboard launcher 默认对齐 planned app 的全屏分页 TUI、Foxglove WebSocket 和 MuJoCo
窗口；MuJoCo 双手 marker 与键盘共用同一个 planner target source。它不改变现有
HKS/kinematic app，也没有硬件发布路径。运行与模型合同见
`docs/hierarchical_inverse_dynamics_torque_sim.md`。

app 从 R1 URDF 所在 `robot_description/psi_r1/urdf` 布局推导 mesh package search root，并直接
交给模型加载器。调试入口不预先校验 mesh 目录或 collision diagnostics 形状；底层错误直接退出，
不新增 Foxglove schema；torque app 复用既有 state/planning/execution channels，并只在
app-local TUI 增加 Dynamics 页。

也可以设置 `MOTION_CONTROL_URDF`，省略每次运行的 `--urdf`。

有 Foxglove sink 时连接 `ws://127.0.0.1:8765`。MCAP 路径存在时会拒绝覆盖。
默认不录制 MCAP，只有显式传入 `--mcap <path>` 才会录制；`--no-mcap` 可用于显式
关闭或覆盖前面给出的 `--mcap`。
使用 `--viz none` 或配置 `MCL_ENABLE_FOXGLOVE_TRANSPORT=OFF` 时应用使用 Null sink；该选择
不改变 solver 或 canonical artifact 行为。

### App-local 启动脚本与覆盖优先级

支持 keyboard、MCAP 或 CSV 的 app 在自己的 `scripts/` 下提供对应的
`run_keyboard.sh`、`run_mcap_replay.sh`、`run_csv_replay.sh`；Cartesian planning 提供
`run_json_request.sh`。脚本默认从
`${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin` 选择 colcon 安装产物；只有显式设置
`MCL_BINARY` 才运行其他 build tree。脚本通过根目录 `scripts/launch_app.sh` 统一处理 binary、
`LD_LIBRARY_PATH`，以及可选的 `MCL_CPU_SET`/`MCL_RT_PRIORITY` 对应的 `taskset`/`chrt`，但不
解析 app 的算法语义。每个脚本把 `"$@"` 放在 preset 后面，因此优先级是：

```text
app compiled defaults < script preset/environment < trailing explicit arguments
```

`hierarchical_kinematics_step` 不使用上述 Bash wrapper，而提供可 import 的纯标准库
`launcher.py` 与三个 Python 入口。Python 未设置字段使用 `argparse.SUPPRESS`，C++ profile
defaults 是默认值唯一来源；历史 launcher preset 位于 Python preset 表，显式 argparse 参数
优先级最高。只保留 `MCL_BINARY`、`MCL_INSTALL_PREFIX`、`MCL_LD_LIBRARY_PATH`、
`MCL_CPU_SET`、`MCL_RT_PRIORITY` 五个运行环境变量，算法、robot、replay 与 UI/Viz 全部走参数。

replay artifact 记录 resolved config、原始 argv、`--launcher` 标识和输入 SHA-256/provenance。
`mcl_baseline` 接受 source/UI/Viz/output 选择，但明确拒绝 solver、backend、rate、tolerance、
iteration、regularization、task gain/weight 等算法覆盖，继续保持 production-static。

旧的 Core planning matplotlib smoke app 已迁到 Lab，并通过
`MCL_BUILD_MCC_PLOTS=ON` 显式启用；它需要当前 Python 环境包含 NumPy 和
matplotlib。

## 笛卡尔 MoveLine 规划预览

`mcl_cartesian_planning` 是独立于 IK app 的 JSON 驱动预览入口。它只调用
`motion_control::core::CartesianPlanner`，不读取 URDF、不创建 RobotModel，也不调用
FK/IK。多个 `segments` 表示同一次请求中的并行同步 frame move，不是连续路点。

先安装启用了 Foxglove 的 `motion_control_viz >= 0.3` 和 `motion_control_core`，然后单独构建：

```bash
cmake -S . -B build/cartesian-planning \
  -DMCL_BUILD_SINGLE_ARM_STEP=OFF \
  -DMCL_BUILD_STEP=OFF \
  -DMCL_BUILD_TARGET=OFF \
  -DMCL_BUILD_HIERARCHICAL_KINEMATICS_STEP=OFF \
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

Foxglove 连接 `ws://127.0.0.1:8765`，场景位于 `/mcl/cartesian/scene`，左右规划 pose 位于
`/mcl/cartesian/reference/{left,right}`。默认按规划时间循环播放至 Ctrl-C；`--once` 只播放一次，
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
adapters/data/            ROS-free source、decoder、temporal/semantic projection 与 ReplayClock
adapters/replay/          replay typed load、provenance 与 artifact mechanics
contracts/input/          MotionTargetFrame、InputStatus、SourceControl、TeleopIntent、KeyEvent
contracts/presentation/   solver-neutral AppSnapshot 与 TuiDocument
contracts/runtime/        scheduler/runtime 共享状态合同
components/               无 MCC 依赖的 scheduler、terminal、teleop、TUI、replay、Viz、R1 config 与 scaffolding
apps/cartesian_planning/  main/options/planning/loop；JSON MoveLine 规划、渲染和 Foxglove 回放
apps/plot_core_planning/  main/options/planning；可选 Core planning API matplotlib smoke app
apps/replay_plan/         main/options/loop；不运行 solver 的 timeline inspect/artifact 入口
apps/baseline/            main/options/solver/loop；PlaCo production-static teleop/replay 对照基线
apps/step/          main/options/solver/loop；普通双臂 ServoStep teleop/replay
apps/target/        main/options/solver/loop；普通双臂 TargetSolve
apps/single_arm_step/ main/options/solver/loop；单臂 ServoStep
apps/hierarchical_kinematics_step/ 五 profile Red/Yellow HKS、planning、OTG、nullspace、导纳与运动学仿真
apps/hierarchical_inverse_dynamics_torque_sim/ fixed-base R1 hierarchical ID + MuJoCo torque 闭环
contracts/                definition、manifest、metric 与 visualization 合同
data/raw/                 原始数据占位；不得静默改写
data/canonical/           规范数据占位
experiments/              E01-E04 Experiment 实例、append-only run 与 reviewed result
analyses/                 只消费已有证据的 Analysis
docs/                     愿景、通用架构和项目实现映射
third_party/placo/        仓库内直接构建和修改的 placo 源码
third_party/OpenSoT/      固定 ros2 快照；仅供隔离的 E04 external build 使用
third_party/ftxui/         固定版本、无嵌套 Git 仓库的交互 TUI 源码
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

## FTXUI 源码策略

交互 IK 控制台使用仓库内的 FTXUI `v7.0.3` 源码，来源和归档 SHA-256 记录在
[`third_party/ftxui/MOTION_CONTROL_LAB.md`](third_party/ftxui/MOTION_CONTROL_LAB.md)。
该目录不是 Git submodule，也不包含嵌套 `.git`。只有同时启用至少一个 interactive IK app 和
`MCL_ENABLE_TUI=ON` 时才构建 FTXUI；运行时 `--ui none` 不创建 renderer，且不会隐式禁用
terminal input。
