# Apps 开发约束

本文件适用于 `apps/` 及其全部子目录。这里的程序用于算法调试和行为观察；每个 app
必须保持实现边界清晰，并优先暴露真实错误。

## App 独立性

- 每个 app 的完整业务实现必须保留在自己的 `apps/<app_name>/` 目录中。
- 所有 app 都以 R1 为既定机器人，不在 app 内重复定义 R1 的 joint、frame、默认姿态、
  TCP offset 等固定参数，也不为假想的其他机器人增加抽象层。
- solver 的选择、构造和配置必须由具体 app 持有，包括 solve mode、周期、迭代次数、
  backend、regularization、joint-limit policy、约束注册和结果解释。
- task 的定义、构造和配置必须由具体 app 持有，包括 task 类型、目标、权重、gain、
  enforcement、mask、优先级和 task handle 的使用。
- app 专属的运行流程、状态更新、诊断数据解释和可视化内容组装也必须留在具体 app 中；
  solver-neutral `IkDebugFrame` 的标准 TUI 页面格式可以由共享 component 统一生成。
- MCC solver、task、constraint、`CartesianPlanner` 和 `JointPlanner` 必须由具体 app 直接
  include、构造和调用；不得在 `components/` 中增加二次 facade、controller、统一 runner 或
  mode-driven pipeline 来代替 MCC API。
- 两个或多个 app 出现相同的 solver、task 或业务流程代码时，允许并鼓励保留重复；不要
  为消除重复把这些内容提取到 `apps/common/`，也不要通过 mode、flag、callback 或配置对象
  在 common 中隐藏 app 差异。
- 一个 app 的改动不应要求理解或同步修改另一个 app 的 solver/task 配置。

## 共享 component 白名单

旧 `apps/common/` 已删除。共享实现只能进入职责窄且可独立测试的 `contracts/`、
`components/` 或 `adapters/` target：

- input/presentation/runtime typed contracts；
- terminal session、归一化 `KeyEvent` 与 key routing；
- `KeyEvent -> TeleopIntent/SourceControl` 和 Cartesian target 积分；
- 只消费 `TuiDocument` 的 renderer，以及 solver-neutral `IkDebugFrame -> TuiDocument`
  标准页面构造；共享 TUI 可以按 solver-neutral snapshot capability 增加 planning/OTG 页面，
  但不得 include MCC 或解释 MCC diagnostics；
- single/grouped scheduler、mailbox、deadline 与 stop/join；
- MCAP/CSV typed pipeline、唯一 `ReplaySource` 状态机；
- solver-neutral IK preview projection、WebSocket/MCAP/Null transport adapter；
- artifact/hash、固定 R1 joint/frame/TCP/default pose 和无业务语义机械转换；
- typed `RuntimeServices`/RAII 与 `mcl_add_app(...)` build scaffolding。

任何代码只要知道具体 solver、solve mode、task、约束、planner/OTG 实现、fault policy 或 app
名称，就必须留在具体 app。共享 TUI 只格式化 presentation contract 已提供的 solver-neutral
快照；`components/` 不得 include `motion_control_core`，也不得用 mode enum、virtual callback、
`std::function` 或大配置对象重建新的聚合 runtime。

## App 目录和启动合同

- `apps/<a>` 不得 include 或 link `apps/<b>`；app-local executable 与测试可以共享留在同目录的
  `mcl_<app>_support` target。
- 每个 app 保留 app-local `options.*`，共享组件只接收 typed config；禁止全局 CLI parser、全局
  option registry 或统一 `mcl` executable。
- 运动控制 app 按 `main.*`、`options.*`、`solver.*`、`loop.*` 组织；只有实际调用
  `CartesianPlanner`、`JointPlanner` 或其他规划算法的 app 才增加 `planning.*`：
  `main` 是短 composition root，`solver` 直接持有 MCC topology，`planning` 直接持有 MCC planning，
  `loop` 可以合并 worker、input、replay、presentation、Viz 和 artifact 等非核心胶水。不要为了
  目录看起来极简而把 solver/planning 搬到共享 component，也不要为了形式化分层继续拆分非重点代码。
- 纯 planning、plot 或 replay inspection 工具不得为满足文件形状伪造空 solver；它们保留
  `main/options/planning/loop` 中实际存在的职责，且算法入口仍使用上述简洁名称。
- 按实际 source 能力提供 `scripts/run_keyboard.sh`、`run_mcap_replay.sh`、
  `run_csv_replay.sh` 或 JSON request script。脚本最后必须原样转发 `"$@"`。
- 参数优先级是 compiled defaults、script preset/environment、trailing explicit arguments。
- 产生实验 artifact 的 app 记录 resolved config、原始 argv、launcher 标识和 input hash/
  provenance。
- production-static baseline 只允许输入、输出和 presentation/transport 选择；算法 profile、
  solver、task、rate 和数值配置保持冻结并拒绝 override。

## 调试期错误处理

- 不做防御性编程；除底层 API 的调用形式强制要求外，不主动增加输入、文件、状态、尺寸、
  enum 或结果完整性校验。
- 不用 `try/catch` 把异常转换为 TUI 状态、默认值、空结果、跳过本轮或继续运行。
- 不增加 fallback、静默降级、自动修复或“尽力运行”路径。
- solver、模型加载、可视化和调度中的错误应直接向上传播并使进程失败；保留底层错误信息，
  不用二次包装掩盖首个失败点。
- 资源清理由 RAII 和对象生命周期负责，不为捕获异常后继续运行而增加清理分支。

## 变更检查

修改或 review `apps/` 时，必须先盘点全部 `apps/<app_name>/` 目录以及直接放在 `apps/`
下的 app 源文件；不能只检查本次被点名的 app。至少确认：

1. 新增或修改的 solver/task/planner 配置是否仍位于具体 app 目录并直接使用 MCC；
2. shared component 的新增内容是否完全落在白名单内且不 include MCC；
3. `main` 是否仍清楚展示 solver 与 planning 装配，而非退化成统一 `runApp()`；
4. 是否为了去重引入了跨 app 的 solver/task 抽象；
5. 是否新增了会吞掉、改写或延迟暴露错误的校验、异常处理或 fallback。
