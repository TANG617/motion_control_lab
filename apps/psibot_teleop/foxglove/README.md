# PsiBot Teleop · Cortex 调试布局

在 Foxglove 的 Layouts → Import from file 中导入 `psibot_teleop.layout.json`，
连接 Cortex 的 `ws://127.0.0.1:8765`。Cortex Mock 与 3D 暂时统一使用
`/workspace/models/Psi_R1_visual_collision.urdf`；3D 通过
`http://127.0.0.1:8766/Psi_R1_visual_collision.urdf` 读取同一文件和相对路径的 meshes。
`psibot_teleop.layout.api.json` 是同一布局的官方 Layout API 格式。

启动配套 Mock 和模型 HTTP 服务（已有实例需先正常退出，脚本不会自动终止它）：

```zsh
source /opt/ros/jazzy/setup.zsh
source /workspace/install/setup.zsh
python3 /workspace/labs/motion-control-lab/apps/psibot_teleop/foxglove/run_model_mock.py
```

该启动器只运行 `/workspace/install/psi-cortex-mock/psi_cortex` 中的现有程序，
将配置复制到独立 `runs/psibot_teleop_model/<time>/config`，以 `mcl_initial` 使用 MCL 的
整组默认姿态，不做关节符号转换，不修改安装包、URDF 或 SDK。左臂 J2/J4 为
`-1.38/-1.4 rad`。其他 Cortex 预设姿态仍属旧模型，使用前需单独核对。
`--prefix` 可选择安装包；`--port`、`--viz-port`、`--model-port` 可选择独立端口。
退出启动器会同时结束其 Mock 和模型 HTTP 服务。桌面跨容器访问时需转发 8765、8766。

布局参考 MCL 的 goal → reference → command → actual 观测分层；话题采用 Cortex 实际
输出 `/psi_cortex/*`。布局只订阅可视化数据，控制操作仍由 `mcl_psibot_teleop` TUI 完成。

| 页 | 内容 |
|---|---|
| 01 概览 | 3D 实体 actual / 半透明 command、末端误差、服务端 session/outcome/estop、首次操作说明 |
| 02 末端跟踪 | 左右 TCP 的 XYZ：WBC goal/reference、command FK、actual FK；reference 有效性 |
| 03 关节 PVA | 左臂、右臂、腰、头四个子页；actual 实线 / command 虚线，同关节同色 |
| 04 WBC / Yellow | attempted/accepted/coupling、Red/Yellow 耗时、hard violation、task scales、原始 pass 数据 |
| 05 运行与事件 | 控制周期耗时与 deadline lateness、Part/WBC/Yellow 队列深度与丢样、事件、run/config 原始数据 |

数值曲线默认显示最近 15 秒，使用 Foxglove 的 **Log time**（`receiveTime`）。已核对 Cortex
WebSocket 消息时间戳与 `stamp.sample_time_ns` 相同，因此它仍表示服务端采样时间，不是客户端
收包完成时间。Foxglove 3.0.0 实测采用自定义 `stamp.sample_time` 时曲线空白，改用消息时间戳后
可正常显示。位置误差换算为 mm、控制周期
耗时换算为 ms。关节顺序固定为当前 R1 合同：头 0–1、腰 2–5、左臂 6–12、右臂 13–19；
可在「运行与事件 → Run / joint order」核对 `control_joint_names`。

## 数据含义

- Part 单臂模式当前不发布 SDK 编辑目标/接受目标，不能将 command FK 命名为 SDK target。
  `goal/reference` 和 WBC/Yellow 话题仅在相应服务端会话产生；未启动时页面空白是预期行为。
- reference 误差依据 `reference_valid` 和 FK 有效性过滤，PVA 按各字段有效位过滤，Red 数值
  按 `attempted=true` 过滤。切出 WBC 后保留的历史消息不能解读为新的求解结果。
- Mock actual 是 command echo，不等于硬件或动力学反馈；不画力矩占位值。
- SDK 上下电/档位和客户端 RPC 耗时仍在 TUI 查看。此布局里的 WBC/Yellow 来自 Cortex
  独立可视化输出，不代表 Psi SDK 新增了这些诊断接口。
- Cortex/SDK 源码和算法未修改；Mock 的模型路径及初始姿态由上述独立运行配置覆盖。

## 重新生成

```bash
uv run apps/psibot_teleop/foxglove/build_layout.py
# 使用其他在线模型：
uv run apps/psibot_teleop/foxglove/build_layout.py --urdf-url https://example.com/robot.urdf
```

生成依赖固定为 `foxglove-sdk==0.26.0`，不进入 app 的编译或运行依赖。
MCP 应用结构使用 `client__set_current_layout`；面板标题使用 `foxglovePanelTitle`。
认证信息不得放入布局、生成器或仓库文件。

参考：[Foxglove Layouts](https://docs.foxglove.dev/docs/visualization/layouts)、
[FoxQL](https://docs.foxglove.dev/docs/visualization/foxql)、
[MCL telemetry 合同](../../../docs/foxglove_mcl_telemetry_contract.md)。
