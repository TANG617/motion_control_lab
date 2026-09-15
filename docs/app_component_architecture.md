# MCL 应用与组件架构

> 更新日期：2026-09-14

## 职责与依赖

MCL 的九个现役 app 见 [应用表](../README.md)。每个 app 直接拥有 solver/task 配置、
规划、主循环、原生结果解释及专属展示。MCC/Sim/Viz 不反向依赖 Lab。

```text
app options / composition root
  ├─ app-local solver / planning / loop → MCC、PlaCo、Sim
  ├─ contracts → typed input、runtime、presentation、execution
  ├─ components → scheduler、teleop、terminal、TUI、R1 helpers
  └─ adapters → MCAP/CSV、replay、preview transport、run artifacts
```

| 位置 | 当前责任 |
|---|---|
| apps | 独立执行结构；CLI、任务构造、solver、规划与失败语义 |
| contracts | 数据、输入、展示、运行、可视化及公开请求的版本化合同 |
| components | solver-neutral 调度、mailbox、键盘输入、目标积分、TUI 格式化、固定 R1 参数 |
| adapters | 数据解码与时间投影、唯一 ReplaySource、可视化传输、artifact/hash |
| experiments | 场景、输入引用、方法声明、串行选择与独立核验 |
| analyses | 从固定证据生成表、图、报告；不启动被测 app |
| tools/app_execution | JSON/路径/哈希、公开 app 进程、preflight 与独立核验 |
| tools/mcc_placo_study | 旧研究证据的离线读取、FK、指标与分析支持 |

共享组件不 include MCC，也不通过统一 controller、runner 或大配置对象隐藏 app 差异。
相同 solver/task 代码可以在 app 内分别保留。新增共享实现必须职责窄、有真实消费者。

## 输入与运行

MCAP/CSV → typed decoder → timestamp policy → immutable timeline → ReplaySource。
`replay_plan` 在输入阶段停止；运动 app 在自己的循环中消费同一 typed timeline。
键盘经 KeyEvent → TeleopIntent/SourceControl → Cartesian target 积分进入 app。

Scheduler 提供 single/grouped worker、mailbox、deadline、stop/join；求解模式、频率选择、
耦合和错误处理由 app 决定。暂停含义属于具体 app：例如 HKS 输入暂停与力矩仿真整链暂停不同。
不把 scheduler 计数自行解释成新的实时性保证。

## 展示与机器人配置

app 将实际算法状态解释为 solver-neutral `IkDebugFrame`/`TuiDocument`；TUI renderer 只格式化。
IK preview 投影为通用 `RenderBatch`，transport 选择 WebSocket/MCAP/Null。
专属规划、碰撞、HARP 和 telemetry 内容留在 app，topic/schema 见 [可视化合同](foxglove_mcl_telemetry_contract.md)。

HKS 的 R1 joint/frame/TCP/default pose/limits/collision/provenance 由 app-local RobotOptions 持有；
其余 app 使用共享固定 R1 配置。SDK 客户端通过自己的发布快照连接 Psi 后端，不构造 MCC solver。

## 修改和构建

运动 app 按 main/options/solver/loop 与实际需要的 planning 组织；main 展示实际装配。
replay inspection 与 SDK 客户端保留真实职责，不创建空 solver。app 不 include/link 其他 app。
`mcl_add_app` 负责 executable、安装和 help smoke；各 app 注册自己的算法测试。
共享数据/组件及多 app CLI 合同检查集中在 cmake/SharedTests.cmake。

启动器留在 app 内，选择已安装产物，不隐式构建；路径与覆盖规则见 [运行说明](build_and_run.md)。
新实验通过公开请求调用同一 app-local 执行函数，不能新增研究专用 solver 分支。
更多约束见 [Apps 规则](../apps/AGENTS.md)；证据流程见 [实验生命周期](experiment_architecture.md)。
