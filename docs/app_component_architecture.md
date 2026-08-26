# Motion Control Lab 应用与组件契约架构

> 更新日期：2026-08-21
>
> 状态：已实施。本文同时是现行职责合同和依赖验收基线。

本文定义 Motion Control Lab（下文简称 MCL）的最终应用架构。
目标是让 TUI、Viz、调度、teleop 和 replay 成为可独立测试、可被所有 app 复用的
Lab 内部组件，同时保持每个 app 的算法语义和实验参数独立。

## 架构决策

1. MCL 仍是一个独立 CMake package。共享能力以 package 内部 CMake target 组织，
   当前不拆成多个 package，也不向工作区其他项目导出。
2. `apps/<app>/` 之间禁止 include、link 或调用。一个 app 的新增和修改不要求同步修改
   另一个 app。
3. 不建设统一 CLI、统一 `mcl` executable 或全局 option registry。每个 app 保留自己的
   `options.*`；推荐运行入口是 app 自己的 `scripts/*.sh`。
4. 建设薄的 App Scaffolding，但它不是统一 runner、app 基类或用 mode/callback 隐藏差异的
   业务框架。它只负责公共组件的装配、生命周期和构建样板。
5. TUI 只负责展示和 TUI 导航；键盘遥操作是独立 input adapter。以后新增 gamepad、
   SpaceMouse 或网络遥操作时，不修改 TUI。
6. MCAP 和 CSV 是同一个 replay 能力的不同物理 backend，不分别复制 replay clock、
   pause/step、timeline 或 EOS 状态机。
7. solver、task、constraint、规划/OTG 语义、主循环、诊断解释和 app 专属投影继续由具体
   app 拥有并直接使用 MCC API。共享组件不依赖 MCC；不建立 solver/planner facade、controller
   或统一 runner。
8. TUI 和 Viz 都是可选输出。headless replay、batch 和测试路径不得依赖 FTXUI、Foxglove
   或网络端口。

## 总体依赖

```mermaid
flowchart TB
  subgraph Launch[每个 app 自己的启动层]
    Scripts["apps/app/scripts/*.sh"]
    Options["app-local options"]
  end

  subgraph Apps[彼此独立的 app]
    AppA["IK app: main/options/solver/loop"]
    AppB["planned IK app: main/options/solver/planning/loop"]
    AppC["planning/replay tool: only real roles"]
  end

  subgraph Shared[MCL 内部共享组件]
    Scaffold[App Scaffolding]
    Input[Input contracts]
    Teleop[Teleop]
    Replay[Replay]
    Scheduler[Scheduler]
    TUI[TUI]
    VizAdapter[Viz adapter]
    Artifacts[Run artifacts]
    Robot[R1 robot config and helpers]
  end

  MCC[motion_control_core]
  MCV[motion_control_viz]

  Scripts --> Options
  Options --> AppA
  Options --> AppB
  Options --> AppC

  AppA --> MCC
  AppB --> MCC
  AppC --> MCC

  AppA -. typed references .-> Scaffold
  AppB -. typed references .-> Scaffold
  AppC -. typed references .-> Scaffold
  Teleop --> Input
  Replay --> Input
  VizAdapter --> MCV

  AppA -. forbidden .-> AppB
  AppB -. forbidden .-> AppC
  AppC -. forbidden .-> AppA
```

依赖方向只有 `app -> shared components -> external components`。`motion_control_core`、
`motion_control_viz` 和共享组件都不得反向依赖任何具体 app。

## 输入架构

“输入”按语义分层，而不是把设备、文件格式和运行状态机揉成一个类。

```mermaid
flowchart LR
  Terminal[TerminalFrontend] --> Router[KeyRouter]
  Router -->|navigation keys| TuiNav[TUI navigation]
  Router -->|motion keys| Keyboard[KeyboardTeleop]
  Gamepad[Future GamepadTeleop] --> Intent[TeleopIntent]
  SpaceMouse[Future SpaceMouseTeleop] --> Intent
  Keyboard --> Intent
  Intent --> Cartesian[CartesianTeleop controller]

  Mcap[McapSource] --> Decode[typed decode and temporal projection]
  Csv[CsvSource] --> Decode
  Decode --> Replay[ReplaySource]

  Cartesian --> Target[MotionTargetFrame]
  Replay --> Target
  Target --> App[Concrete app]
```

### Input contracts

`motion_control_lab::input_contract` 只定义 app 消费的稳定数据：

- `MotionTargetFrame`：左右/单臂目标、logical time、revision 和来源；
- `InputStatus`：running、paused、end-of-stream、fault 等来源状态；
- `SourceControl`：pause、resume、step、stop 等通用控制意图；
- `TeleopIntent`：与设备无关的平移、旋转、选择臂和速度档位意图。

合同不包含 FTXUI event、MCAP record、CSV row、Foxglove message 或 MCC request。

### Keyboard teleop

`motion_control_lab::keyboard_teleop` 把归一化 `KeyEvent` 转成 `TeleopIntent` 或
`SourceControl`，不绘制界面、不维护 solver 状态，也不直接生成 MCC request。

`motion_control_lab::cartesian_teleop` 根据 `TeleopIntent`、当前 target 和时间增量生成
`MotionTargetFrame`。设备 adapter 与 target 积分器分开后，未来 teleop 实现只需要产生同一
`TeleopIntent`，无需复制 Cartesian target 状态机。

### Replay

`motion_control_lab::replay` 拥有一次且仅一次的 replay 状态机：timeline、clock、batch/
realtime、playback rate、pause/resume/step、EOS 和来源诊断。

- `McapSource` 只负责 MCAP 读取和 typed decode；
- `CsvSource` 只负责 CSV 读取和 typed decode；
- temporal validation、左右流 pairing、timestamp projection 和初始 JointState 映射保持为
  replay pipeline 的共享阶段；
- 两种 backend 最终都向 `ReplaySource` 提供相同 typed stream。

因此脚本可以叫 `run_mcap_replay.sh` 和 `run_csv_replay.sh`，但代码中不建立两套完整的
`mcap_replay`/`csv_replay` runner。

## 展示架构

### TUI

`motion_control_lab::tui` 接收通用 `TuiDocument`，只实现布局、页面、滚动、帮助和渲染。
它不知道 `TargetCommand`、solver backend、task handle、replay pairing 或 Foxglove。

`motion_control_lab::standard_ik_tui` 把 solver-neutral `IkDebugFrame` 格式化为标准的
Overview、Cartesian Planning、Solver and Quadratic Programming、Joint State、Runtime 和
Events 页面。它只生成 `TuiDocument`，不依赖 FTXUI、MCC 或具体 app。

每个 app 仍负责把自己的 solver/runtime 状态解释并填入 solver-neutral snapshot。共享的
planned-grouped TUI presenter 根据 snapshot capability 决定是否增加 Joint Planning、OTG、
projection 和 clamp 页面；它只格式化数据，不 include MCC、不解释 MCC diagnostics。真正的
FTXUI 渲染实现只存在一份，也不通过 callback、mode 或配置对象隐藏 app 差异。

planned-hierarchical apps 的 `options.*` 直接嵌入 `PlannedGroupedTuiConfig`，loop 侧只保留
短调用；app 先完成 MCC diagnostics 到 solver-neutral snapshot 的解释：

```cpp
PlannedGroupedTui tui(options.presentation);
tui.handleNavigation(event);
tui.render(snapshot);
```

`TerminalFrontend` 拥有 terminal session 和原始事件读取。`KeyRouter` 把导航键送到 TUI，
把运动键送到 `keyboard_teleop`。这样 `--ui none` 仍可在需要时使用键盘输入，而 TUI 也可以
在 replay 模式中只负责展示和 pause/step 控制。

### Viz

通用可视化合同和 transport 由 `motion_control_viz` 提供。MCL 只保留两类 adapter：

- 窄的公共 IK preview projector：把 solver-neutral 的 pose/FK/joint-state snapshot 转成
  `motion_control_viz::RenderBatch`；
- app-local projector：生成 planning request、OTG、collision 或其他 app 专属内容。

transport 的 WebSocket、MCAP 和 Null sink policy 可复用，但 app 不直接操作 Foxglove SDK。
headless 路径使用 Null sink，且不应因为 Viz 未构建而改变求解与 artifact 语义。

## Scheduler

`motion_control_lab::scheduler` 只提供 wall-clock/runtime 机制：

- single-rate tick；
- grouped periodic worker；
- deadline、overrun 和 skipped-release 统计；
- stop controller、mailbox 和线程 join；
- 可独立使用的 rate gate。

Scheduler 不读取键盘，不渲染 TUI，不发布 Viz，不加载 replay，也不认识 MCC group、solver
结果或 app 名称。各 app 决定有哪些 rate、哪个 tick 执行什么工作、deadline miss 是否 fatal，
并在 app-local 配置中保存这些策略。

## App Scaffolding

可以并且应该提供 scaffolding，但边界必须保持很薄。建议由两部分组成。

### Runtime scaffolding

`motion_control_lab::app_scaffold` 以 header-only template 提供 `RuntimeServices` typed reference
集合和 `RuntimeLifecycle` RAII 生命周期：

- 接收 app 已经选好的 input source、scheduler、terminal、TUI、Viz sink 和 artifact writer；
- 统一执行 start、stop、close、join 等无业务语义的资源生命周期；
- 向 app 暴露这些组件的 typed reference；
- 保留第一个基础设施错误并按原错误失败，不吞异常、不自动降级。

它明确不做以下事情：

- 不解析 `argc/argv`，不选择 keyboard/MCAP/CSV；
- 不构造 MCC/PlaCo solver、task、constraint、planner 或 OTG；
- 不拥有统一 `run()` 主循环，也不通过 virtual callback 注册 app 行为；
- 不解释 accepted/rejected、scale、collision、fault hold 等算法状态；
- 不决定 rate、deadline policy、输出 topic 或 artifact 内容。

app composition root 使用的实际 API 形状如下。solver 与 planner 由 app 直接构造；input、
scheduler、terminal、TUI、Viz 和 artifact writer 由 app 选择；scaffolding 不拥有主循环：

```cpp
int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  const auto model = loadRobotModel(options);
  SolverHandles handles;
  SolverRuntime runtime;
  configureSolver(runtime, handles, model, collision_model, robot, options);
  motion_control::core::CartesianPlanner cartesian_planner;
  return runLoop(options, robot, runtime, handles, cartesian_planner);
}
```

在需要多速率 reference 的 app 中，`SolverRuntime` 仍由该 app 自己定义：它持有单个
actuator-facing `HierarchicalKinematicsSolver`、producer solver、runtime envelope 和
`SnapshotBuffer<T>`。它不是共享的跨 app runtime facade。

### Build and directory scaffolding

提供 `mcl_add_app(...)` 一类的 CMake helper 和 app 目录模板，用来统一 target 输出目录、install、
help smoke、组件依赖检查和脚本位置。模板不生成 solver/task 实现。

```text
apps/<app>/
  CMakeLists.txt
  main.cpp                 # short composition root; exposes MCC solver/planner topology
  options.hpp / .cpp       # only this app's CLI, defaults and typed options
  solver.hpp / .cpp        # direct MCC solver/task/constraint construction
  planning.hpp / .cpp      # optional: only when the app directly uses a planner
  loop.hpp / .cpp          # workers, input/replay, presentation/Viz/artifact glue
  scripts/
    run_keyboard.sh
    run_mcap_replay.sh
    run_csv_replay.sh
  tests/
```

纯 planning/plot 工具省略 `solver.*`，单次执行且没有 runtime loop 的工具也省略 `loop.*`；
replay inspection 工具省略 `solver.*` 和 `planning.*`。这些省略反映真实职责，不建立空文件占位。

app 不复制 FTXUI renderer、Foxglove transport、MCAP/CSV decoder、replay clock 或 scheduler
机制；但 app 直接保留自己的 MCC solver、planning 和主循环代码，即使与另一个 app 局部相似。
同一 app 可以建立只供本 executable 与测试链接的 support target；它不是共享算法 component。

## CMake 组件边界

现行实现由以下 Lab 内部 targets 组成：

| Target | 唯一职责 | 禁止依赖 |
| --- | --- | --- |
| `motion_control_lab::input_contract` | target、intent、source status/control 类型 | FTXUI、MCAP、Viz、MCC |
| `motion_control_lab::scheduler` | tick、worker、deadline、mailbox、stop | input、TUI、Viz、replay、MCC |
| `motion_control_lab::terminal_frontend` | terminal session、归一化 KeyEvent | solver、replay、Viz |
| `motion_control_lab::keyboard_teleop` | KeyEvent 到 TeleopIntent/SourceControl | TUI renderer、solver、Viz |
| `motion_control_lab::cartesian_teleop` | TeleopIntent 到 MotionTargetFrame | 具体设备、TUI、solver |
| `motion_control_lab::keyboard_input` | 组合 terminal/router/keyboard/cartesian 的 typed keyboard source | solver、Viz、artifact |
| `motion_control_lab::standard_ik_tui` | solver-neutral IkDebugFrame 到标准 TuiDocument | FTXUI、MCC、具体 app |
| `motion_control_lab::planned_grouped_tui` | solver-neutral planned/optional OTG snapshot 到 TuiDocument | MCC、具体 app、solver policy |
| `motion_control_lab::tui` | TuiDocument 渲染与导航 | teleop、replay、solver、Viz |
| `motion_control_lab::data` | typed source、decoder、temporal pipeline | app、solver、TUI、Viz |
| `motion_control_lab::replay` | 单一 replay source 和 clock/state machine | app、solver、TUI、Viz |
| `motion_control_lab::run_artifacts` | manifest、status、trace 元数据和 hash | TUI、Viz、solver policy |
| `motion_control_lab::preview_projection` | 通用 IK snapshot 到 RenderBatch | transport、app-specific state |
| `motion_control_lab::preview_transport` | WebSocket/MCAP/Null sink 选择和生命周期 | solver、teleop、replay |
| `motion_control_lab::r1_robot_config` | 固定 joint/frame/TCP/default pose | app policy、solver/task config |
| `motion_control_lab::app_helpers` | 无业务语义的机械转换 | 具体 app、solver/task policy |
| `motion_control_lab::app_scaffold` | typed services 与 RAII lifecycle | CLI、solver/task、统一 run loop |
| `mcl_add_app(...)` | executable output/install/help smoke/script install | solver/task 生成、CLI 语义 |

这些 target 是 MCL 内部 implementation detail。不要让根仓、生产 SynRobot 或
`motion_control_core` 依赖它们。

现行直接依赖链为：

```text
terminal_frontend -> input_contract
keyboard_teleop   -> input_contract
cartesian_teleop  -> input_contract
keyboard_input    -> terminal_frontend + keyboard_teleop + cartesian_teleop
scheduler         -> runtime_contract + Threads
standard_ik_tui   -> presentation_contract
tui               -> input_contract + presentation_contract + [FTXUI when enabled]
data              -> mcap + Eigen + Threads
replay            -> data + input_contract + run_artifacts
app_helpers       -> cpu_affinity + presentation_contract
app_scaffold      -> runtime_contract
preview_projection -> presentation_contract + r1_robot_config + visualization_contracts
preview_transport  -> motion_control_viz::render + [motion_control_viz::foxglove when enabled]
```

方括号项受 `MCL_ENABLE_TUI` 或 `MCL_ENABLE_FOXGLOVE_TRANSPORT` 控制。`mcl_add_app(...)`
只按调用方显式列出的 `LIBRARIES` 建立 app 依赖，不注入统一 runtime。

## App 所有权与隔离

每个具体 app 必须拥有：

- solver implementation/backend 和 solve mode；
- task、constraint、gain、weight、mask、enforcement 和 hierarchy；
- planner/OTG/collision 的算法策略；
- app rate、deadline policy、fault/rejection 解释和状态转换；
- app-local `AppConfig`、默认值和 option parser；
- solver/runtime 状态到 `IkDebugFrame` 的语义投影，以及专属 TUI/Viz 页面内容；
- 本 app 的 run artifact 内容和实验解释。

运动控制 app 的主要阅读入口固定为 `solver.*`；只有实际存在 Cartesian、Joint 或其他规划
算法时才增加 `planning.*` 并作为第二个算法入口。`main.cpp` 负责显式装配，`loop.*` 可以合并
剩余的非核心 orchestration；这不是把 app 语义迁移到共享 component 的理由。纯 planning、
plot 或 replay inspection 工具只保留实际存在的职责，不为满足目录模板伪造空 solver。

共享 `r1_robot_config` 只保存固定 R1 joint、frame、TCP 和默认 pose。production-static
baseline 的冻结 solver/task/rate/profile 参数继续位于 `apps/baseline/`，不开放 runtime
override，也不因其他 app 的实验需求改变。

禁止以下依赖：

```mermaid
flowchart LR
  Servo[apps/step]
  Target[apps/target]
  Hierarchical[apps/hierarchical_step]
  Planned[apps/planned_hierarchical_step]
  OTG[apps/planned_hierarchical_step_otg]
  Nullspace[apps/planned_hierarchical_step_otg_nullspace]
  Common[Shared component targets]

  Servo --> Common
  Target --> Common
  Hierarchical --> Common
  Planned --> Common
  OTG --> Common
  Nullspace --> Common

  Servo -. no include or link .-> Target
  Target -. no include or link .-> Hierarchical
  Hierarchical -. no include or link .-> Planned
  Planned -. no include or link .-> OTG
  OTG -. no include or link .-> Nullspace
```

若一段 solver/task 逻辑只被同一个 app 的 executable 和测试使用，可以建立
`mcl_<app>_support` target，但源文件必须留在该 app 目录，其他 app 不得链接它。

## Bash 启动与 runtime override

> 本节更新日期：2026-08-26

不增加统一 CLI。用户面对的是每个 app 自己的脚本，例如：

```text
apps/planned_hierarchical_step/scripts/run_keyboard.sh
apps/planned_hierarchical_step/scripts/run_mcap_replay.sh
apps/planned_hierarchical_step/scripts/run_csv_replay.sh
```

脚本负责表达可读、可审查的实验 preset：

- 从 `${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin` 选择 workspace colcon install
  executable；只有显式的 `MCL_BINARY` 才覆盖到 standalone 或临时产物；
- URDF、input/topic 和 output root；
- `LD_LIBRARY_PATH` 等本机运行环境；
- `chrt`、`taskset`、CPU mask 和 realtime priority 等 OS policy；
- 本 app 的推荐 rate、deadline、solver 和 planner/OTG 参数；
- 把 `"$@"` 原样放在命令末尾，允许一次运行覆盖脚本默认参数。

启动脚本不执行 configure/build，也不以 `labs/motion-control-lab/build/` 作为隐式 fallback。
workspace build、install 与 standalone CMake build 是三个不同 artifact tree：标准开发闭环是
`/workspace` 下 `colcon build` 写入 `build/algorithm` 和 `install/algorithm`，随后 app 脚本运行
同一个 install prefix。standalone build 只在调用者用 `MCL_BINARY` 明确选择时参与运行，避免
同名旧 executable 掩盖刚完成的 colcon 构建。

每个 executable 只解析自己的参数。共享组件接收 typed config，不接收 `argc/argv`，也不从
全局环境隐式读取算法参数。参数优先级为：

```text
app compiled defaults < script preset/environment < script trailing explicit arguments
```

为了支持 experiment，除 production-static baseline 外，下列关键参数应能被本 app 的脚本/
本地 flags 在 runtime 覆盖：solver/backend、tolerance、iteration、regularization、task
gain/weight/enforcement/mask、collision 参数、planning/OTG limits、worker rate、deadline policy
以及 UI/Viz/replay/output 选择。固定 R1 joint/frame/TCP 和生产 profile identity 不可覆盖。

这是一组跨 app 的脚本约定，不是统一 parser。不同 app 可以拥有不同参数；脚本不得通过向
所有 binary 填充无效 flag 来伪造一致性。产生实验 artifact 的 run 应保存最终解析后的配置、
原始 argv、脚本路径或版本以及输入 hash。

## App 类型与 source 能力

当前实现保持各 app 原有能力，不为了统一外观给所有 app 强行增加所有 source：

| App 类别 | 目标 source | 说明 |
| --- | --- | --- |
| single-arm ServoStep、TargetSolve | keyboard teleop | 保留各自 ordinary solver 语义 |
| Step、Hierarchical Step、Planned Hierarchical Step、Planned Hierarchical Step OTG | keyboard teleop、MCAP replay、CSV replay | 三种输入最终进入同一 MotionTargetFrame contract |
| production-static baseline | keyboard teleop、MCAP replay、CSV replay | 算法与生产配置冻结 |
| Cartesian planning | JSON request | 不伪装成 teleop/replay app |
| replay-plan | headless MCAP/CSV | 只检查 timeline 和 artifact，不运行 solver |
| E03 batch | batch MCAP replay | 复用 ReplaySource，批处理与失败隔离留在 E03 orchestration |

E03 对应的 `experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/scripts/run_mcap_replay.sh`
表达 motion-library batch preset；它不创建第二套 MCAP runner。

## 已实施目录

```text
motion-control-lab/
  contracts/
    input/
    presentation/
    runtime/
  components/
    app_scaffold/
    scheduler/
    terminal_frontend/
    tui/
    teleop/
      keyboard/
      cartesian/
    replay/
    visualization/
    robot/r1/
  adapters/
    data/                    # physical source、decoder、temporal/semantic projection
    replay/                  # typed load、provenance、manifest mechanics
    execution/               # run artifacts 与 SHA-256
  apps/
    <app>/                    # main/options/solver/[planning]/loop + scripts/tests
  experiments/
  analyses/
```

旧 `apps/common/` 和 `adapters/interactive/` 中的聚合实现已删除；上述 component target 的
依赖合同不得退化为新的大而全聚合层。

## 已实施验收

当前实现满足：

1. CMake/静态检查证明任何 `apps/<a>` 都不 include 或 link `apps/<b>`。
2. Scheduler、keyboard teleop、replay、TUI 和 Viz adapter 各自有不启动 solver 的单元测试。
3. keyboard teleop 用 synthetic `KeyEvent` 测试；TUI 测试只验证 document render/navigation。
4. 同一 canonical 数据通过 MCAP/CSV backend 产生等价 `MotionTargetFrame` 和 replay control
   行为。
5. headless replay/batch 在未构建 FTXUI 和 Foxglove transport 时仍能构建和运行。
6. 每个 app 的 `scripts/*.sh` 可独立运行对应 binary，并允许 runtime override；没有统一 CLI
   依赖。
7. App Scaffolding 不出现 solver/task 类型、app mode enum、统一 business callback 或算法
   状态解释。
8. 删除或拆分现有过宽的 `interactive_runtime`、`app_common` 后，不再有等价的大聚合 target
   以新名字回归。
9. production-static baseline 的冻结参数仍不可覆盖，且其 resolved config/artifact 可审计。
10. Lab README、`apps/AGENTS.md`、CMake dependency tests、app scripts 与本文保持同步。
11. 通用 `components/` 不 include MCC；所有运动控制 app 的 MCC topology 直接位于各自
    `solver.*`，实际存在的 planning 位于 `planning.*`，且 app-local support target 不互相链接。

这些边界由 `tests/architecture_boundaries.py`、独立 component tests、每个 app 的 option/help
tests、TUI PTY smoke、完整 CTest，以及关闭 `MCL_ENABLE_TUI` 与
`MCL_ENABLE_FOXGLOVE_TRANSPORT` 的隔离构建共同验证。架构变更必须同时更新这些检查；不能只
修改本文状态。
