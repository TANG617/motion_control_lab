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
- app 专属的运行流程、状态更新、诊断数据解释和可视化内容组装也必须留在具体 app 中。
- 两个或多个 app 出现相同的 solver、task 或业务流程代码时，允许并鼓励保留重复；不要
  为消除重复把这些内容提取到 `apps/common/`，也不要通过 mode、flag、callback 或配置对象
  在 common 中隐藏 app 差异。
- 一个 app 的改动不应要求理解或同步修改另一个 app 的 solver/task 配置。

## `apps/common/` 白名单

`apps/common/` 只承载所有 app 共同依赖的先验和无业务语义的初始化代码：

- 通用 TUI 的初始化、输入绑定和显示配置；
- R1 机器人不随 app 改变的模型信息和参数，例如 joint names、默认姿态、base frame、
  end-effector frames 和模型加载所需描述；
- 调度器的通用配置解析和初始化；
- 不包含 solver/task 决策的通用展示或机械性数据转换工具。

任何代码只要知道具体 solver、solve mode、task、约束或 app 名称，就不属于 common。
`apps/common/` 是白名单边界，不因代码在多个 app 中重复而自动扩大。

当前或历史上已经位于 common 的代码不构成继续共享的先例。修改相关代码时，应按上述
边界把 solver/task 和 app 专属流程放回对应的 app 目录。

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

1. 新增或修改的 solver/task 配置是否仍位于具体 app 目录；
2. common 的新增内容是否完全落在白名单内；
3. 是否为了去重引入了跨 app 的 solver/task 抽象；
4. 是否新增了会吞掉、改写或延迟暴露错误的校验、异常处理或 fallback。
