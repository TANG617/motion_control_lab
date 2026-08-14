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

仓库还提供可选的 MCC 交互预览运行时。单臂入口使用 grouped solver 的 `RedOnly` profile；
显式区分的双臂 ServoStep/TargetSolve 入口共享一个固定 topology 的 `KinematicsSolver`；grouped
双臂入口使用独立 Red/Yellow worker。它们复用 TUI、Viz 和 wall-clock pacing；算法快照统一映射为
`motion_control_viz::VisualizationFrame`。
所有 IK app 的 Foxglove topic 与 FK 一致性要求由
[Foxglove IK 可视化数据流合同](docs/foxglove_ik_visualization_contract.md)统一定义。
该路径用于开发调试，不替代由 canonical timeline 驱动的可复现实验执行器。
单臂和 grouped 交互预览 app 直接拥有各自的 MCC topology；双臂 ServoStep/TargetSolve 的
薄入口复用 `apps/common/dual_arm_ik_app.*`，确保除 solve-mode 合同外的任务、TUI 和 Viz
行为一致。可复现 replay app 则复用固定策略的单动作 engine，避免不同实验复制或漂移
solver 行为。

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

可选的 headless IK 纵向切片只依赖安装后的 `motion_control_core`，不依赖 Viz/TUI：

```bash
cmake -S . -B build/replay-ik \
  -DMCL_BUILD_SINGLE_ARM_IK=OFF \
  -DMCL_BUILD_DUAL_ARM_IK_SERVO_STEP=OFF \
  -DMCL_BUILD_DUAL_ARM_IK_TARGET_SOLVE=OFF \
  -DMCL_BUILD_GROUPED_DUAL_ARM_IK=OFF \
  -DMCL_BUILD_DUAL_ARM_REPLAY_IK=ON \
  -DCMAKE_PREFIX_PATH="/path/to/mcc-install;/path/to/eiq-install"
cmake --build build/replay-ik --target mcl_dual_arm_replay_ik -j8
```

`mcl_dual_arm_replay_ik` 支持 `batch|realtime`、`previous_solution|fixed_initial_state`
以及独立的 `--servo-period-ms` 控制 horizon。servo period 不从播放 rate 推导，并写入
manifest。MCAP 输入默认读取 `/mc/ik/joint_states` 的第一帧，按 joint name 映射为初始
position（初始 velocity 仍为零）；可用 `--initial-joint-state-stream` 选择其他 JointState
topic。R1 target stream 的输入语义是 TCP pose；runner 使用左右各自的 `0.1 m` TCP offset
转换为 end-effector target 后再构造 IK request，并在 manifest 中记录转换。macOS 上的
realtime 模式只记录 lateness/deadline miss，不声称 hard real-time。
该 runner 是 canonical evidence 路径；`mcl_dual_arm_ik_servo_step` 和
`mcl_dual_arm_ik_target_solve` 仍是 TUI/Viz 开发预览，继续使用 wall-clock interactive
scheduler，二者不共享调度语义。

不指定 `--output-dir` 时，runner 会在
`experiments/E02_dual_arm_replay_ik/runs/` 下创建新的
`<UTC timestamp>__<definition hash>` 目录。`--output-root` 可以只重定向 run 父目录，
`--run-id` 可用于可控测试；`--output-dir` 保留为一次性调试的精确目录覆盖。

需要实时可视化时，在已有构建参数上增加：

```bash
cmake -S . -B build/replay-ik-viz \
  -DMCL_BUILD_SINGLE_ARM_IK=OFF \
  -DMCL_BUILD_DUAL_ARM_IK_SERVO_STEP=OFF \
  -DMCL_BUILD_DUAL_ARM_IK_TARGET_SOLVE=OFF \
  -DMCL_BUILD_GROUPED_DUAL_ARM_IK=OFF \
  -DMCL_BUILD_DUAL_ARM_REPLAY_IK=ON \
  -DMCL_BUILD_DUAL_ARM_REPLAY_VISUALIZATION=ON \
  -DCMAKE_PREFIX_PATH="/path/to/mcc-install;/path/to/eiq-install;/path/to/mcv-install"
cmake --build build/replay-ik-viz --target mcl_dual_arm_replay_ik -j8
```

运行时加 `--visualize --execution-mode realtime`。Foxglove 连接
`ws://127.0.0.1:8765` 后，runner 会先发布 MCAP 首帧对应的 robot joint state 和双臂
输入 target，以及由初始 joint state 计算的双臂实际 FK，然后等待终端的裸空格键；只有按下空格后才建立 ReplayClock
原点。`--visualize` 默认启用该 gate，自动运行可用 `--no-wait-for-space`。如需将可视化
流一起收入 run，增加 `--record-visualization-mcap`。完整 E02 命令见
[`experiments/E02_dual_arm_replay_ik/README.md`](experiments/E02_dual_arm_replay_ik/README.md)。
五条固定 topic 见
[Foxglove IK 可视化数据流合同](docs/foxglove_ik_visualization_contract.md)。

E03 将同一个固定 replay IK engine 应用于 PSI R1 双臂动作库。默认只扫描
`/workspace/fixtures/datasets/psi_r1_dual_arm_motion_library/` 的直接 `*.mcap` 子项，按文件名
排序并在运行前固化 size/SHA-256。每个动作从自己的首帧 JointState 独立初始化，首个错误后
停止该动作，但批次继续执行后续动作；只有所有动作完成且没有 rejected solve 时批次才成功。

```bash
cmake --build /workspace/build/motion_control_lab --target mcl_e03_batch_replay_ik -j8
/workspace/build/motion_control_lab/mcl_e03_batch_replay_ik \
  --urdf /workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf
```

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
  -DMCL_BUILD_DUAL_ARM_IK_SERVO_STEP=ON \
  -DMCL_BUILD_DUAL_ARM_IK_TARGET_SOLVE=ON \
  -DMCL_BUILD_GROUPED_DUAL_ARM_IK=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/mcc-preview \
  --target mcl_single_arm_ik mcl_dual_arm_ik_servo_step \
    mcl_dual_arm_ik_target_solve mcl_grouped_dual_arm_ik -j8
```

每个入口可以独立配置。只构建单臂入口：

```bash
cmake -S . -B build/single-arm \
  -DMCL_BUILD_SINGLE_ARM_IK=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/single-arm --target mcl_single_arm_ik -j8
```

只构建两个双臂入口：

```bash
cmake -S . -B build/dual-arm \
  -DMCL_BUILD_DUAL_ARM_IK_SERVO_STEP=ON \
  -DMCL_BUILD_DUAL_ARM_IK_TARGET_SOLVE=ON \
  -DCMAKE_PREFIX_PATH="/tmp/mcc_install;/tmp/eiq_install;/tmp/mcv_install"
cmake --build build/dual-arm \
  --target mcl_dual_arm_ik_servo_step mcl_dual_arm_ik_target_solve -j8
```

从交互终端运行：

```bash
./build/mcc-preview/mcl_single_arm_ik \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20
./build/mcc-preview/mcl_dual_arm_ik_servo_step \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 --mcap /new/run/path/servo-step.mcap
./build/mcc-preview/mcl_dual_arm_ik_target_solve \
  --urdf /path/to/Psi_R1_rev1.urdf --rate 20 --mcap /new/run/path/target-solve.mcap
./build/mcc-preview/mcl_grouped_dual_arm_ik \
  --urdf /path/to/Psi_R1_rev1.urdf \
  --red-rate 1000 --yellow-rate 100 --ui-rate 20 \
  --deadline-policy monitor
```

两个双臂入口共享双手 Hard position/orientation task、Hard joint-position limits、TUI 和 Viz。
ServoStep 每次只执行一个 QP update，`--rate` 同时定义其正 `servo_period`，并启用 Hard
joint-velocity limits。TargetSolve 使用 `servo_period=0`、最多 80 次迭代且不注册 rate limits；
其 `--rate` 只控制交互求解和发布频率。两者显式使用相同的 ProxQP regularization `1e-4`。

grouped 入口固定使用 `RedYellow` profile，要求 `red-rate > yellow-rate > 0`，默认分别为
1000 Hz 和 100 Hz。每组 period 同时是该 worker 的 deadline。默认
`--deadline-policy strict`：任一 rejected attempt、deadline miss 或 worker exception 都触发
first-writer Fault、停止并 join 两个 worker、保留最后 accepted Red state、关闭 sink 并返回非零。

macOS 等非实时开发环境可显式使用 `--deadline-policy monitor`。该模式记录 deadline miss 并继续，
同时跳过已经过期的 release，避免 worker 无间隔追赶；TUI 会显示各组累计 miss 和 skipped release。
rejected attempt 和 worker exception 在 monitor 模式下仍然触发 Fault。正式时序能力验证应使用
`strict`。

Yellow、Red 启动前会顺序预热一次；正式 run 中两者完全异步，不等待 source 的下一条
结果。TUI、Viz 和 MCAP 只在 `ui-rate` 线程运行，不进入 Red solver 路径。

Red 使用双手 Scaled position/orientation task：每只手的 position/orientation 共享一个
`progress_weight=100` 的 scale，左右手则独立退化；TUI 会报告两路 scale 的 full/degraded/stuck
状态。Red 使用 `1e-6` 的 ProxQP convergence/infeasibility tolerance，并显式关闭 warm start；
joint position/velocity limits 与 Cartesian scaled equalities 共用 app 的 `1e-4` accepted-violation
合同。Yellow 的 warm-start 策略不变。Yellow 使用固定 R1
initial pose 的 Soft posture task（weight `10`），并对生产配置中筛选的 10 个 link pair 使用 Soft
self-collision velocity damping：minimum distance `0.02 m`、influence distance `0.07 m`、gain
`20 s^-1`、weight `1`。Yellow accepted proposal 通过 weight `1` 的内部 coupling 进入 Red。两组
都显式使用 Hard joint position/velocity limits，其中 joint position limit 使用 `1e-2 rad` 内部
margin。self-collision 是运动优化目标，不是硬安全屏障；
margin shortfall 不会自动拒绝 accepted solution，硬件 command authorization 仍由集成层负责。

app 从 R1 URDF 所在 `robot_description/psi_r1/urdf` 布局推导 mesh package search root，并在
启动时验证 `psi_r1/meshes`。collision diagnostics 仅在内部 warm-up 和测试中校验，不新增 TUI 或
Foxglove schema。

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
  -DMCL_BUILD_DUAL_ARM_IK_SERVO_STEP=OFF \
  -DMCL_BUILD_DUAL_ARM_IK_TARGET_SOLVE=OFF \
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
adapters/data/            ROS-free source、decoder、temporal/semantic projection 与 ReplayClock
adapters/interactive/     共享 CLI、TUI、wall-clock scheduler 和 Viz helpers
apps/common/              共享 IK 工具、R1 被动配置和双臂两模式的固定 topology/交互 runner
apps/cartesian_planning/  JSON 驱动的纯 Cartesian MoveLine 规划、渲染和 Foxglove 回放
apps/replay_plan/         不运行 solver 的 canonical timeline inspect/artifact 入口
apps/dual_arm_replay_ik/  MCC 双臂 canonical replay runner；可选 Foxglove 观察端
apps/<entry>/             独立 main、CMake 和 help test；按入口选择或直接拥有 solve topology
contracts/                definition、manifest、metric 与 visualization 合同
data/raw/                 原始数据占位；不得静默改写
data/canonical/           规范数据占位
experiments/              E01/E02 Experiment 实例、append-only run 与 reviewed result
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
