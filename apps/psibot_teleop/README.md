# PSI SDK Teleoperation

`mcl_psibot_teleop` 是使用 SDK **Psi 后端**的键盘控制客户端，提供关节控制、单臂笛卡尔控制与双臂 WBC。
客户端只负责输入、SDK 会话及调试展示；求解和执行仍在 `psi_cortex`。启动只观察，不自动设置档位或上电。

## 构建与依赖

从工作区根目录使用标准 algorithm colcon 配置：

```bash
source /opt/ros/jazzy/setup.bash
COLCON_DEFAULTS_FILE=/workspace/.vscode/workspace/algorithm_colcon_defaults.yaml \
  colcon build --cmake-args -DMCL_BUILD_PSIBOT_TELEOP=ON
```

SDK 不参与编译。`third_party/psi_sdk` 保存配套发布快照，可用
`-DMCL_PSIBOT_SDK_ROOT=/path/to/complete-sdk` 覆盖整个包；不单独覆盖头文件或库。
CMake 按 `CMAKE_SYSTEM_PROCESSOR` 选择 x86_64/aarch64。安装的 SDK/Inex 库位于
`lib/mcl_psibot_teleop`，不覆盖其他应用的 SDK。当前二进制依赖 ROS Jazzy 运行库（含
`librclcpp`），app 不调用 ROS API，也不订阅图像。构建/运行前需要
`source /opt/ros/jazzy/setup.bash` 提供这些库的搜索路径。SDK 的 `$ORIGIN` RUNPATH 不会
自动找到系统 ROS 库。不要将“未使用 ROS API”解读为无 ROS 动态依赖。

```bash
apps/psibot_teleop/scripts/run_keyboard.sh --address 127.0.0.1:6060
apps/psibot_teleop/scripts/run_keyboard.sh --mode wbc
apps/psibot_teleop/scripts/run_keyboard.sh --ui none --duration 5
```

脚本默认执行 `${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_psibot_teleop`。
`MCL_BINARY` 优先级最高；脚本不隐式构建。环境 preset 是 `MCL_PSIBOT_ADDRESS`、
`MCL_PSIBOT_MODE`、`MCL_UI`；末尾 CLI 参数覆盖 preset。`--help` 不连接后端。

## 操作

| 按键 | 动作 |
|---|---|
| Space | 开始/暂停发送；开始时从新读取的实际位姿初始化目标 |
| 左/右 | 选择编辑的臂；单臂切臂先暂停，需重新 Space |
| w/s、a/d、q/e | Base 系 +/-X、+/-Y、+/-Z，默认步长 5 mm |
| n、u/i | 选择 TCP 局部旋转轴、正/负旋转，默认 5° |
| 上/下、m | 步长乘/除 2；输入步长（米） |
| r | 发送状态下，将选中目标重置为新读取的实际位姿 |
| c | 控制模式：上/下选择模式，左/右选择部件，Enter 应用，Esc 取消 |
| 1–5 / F1–F5 / Tab | 概览、目标、关节、客户端运行、事件 |
| h / ? | 帮助 |
| PgUp/PgDn/Home/End | 页面滚动 |
| x / Esc / Ctrl-C | 退出；菜单内 Esc 关闭菜单 |

上表的运动键适用于 **CartesianPosition 或 WBC**。在 single 模式下，选中部件是
**JointPosition** 时，按键改为：

| 按键 | 关节控制 |
|---|---|
| 左/右 | 循环选择当前部件的关节，不切臂 |
| w/s | 选中关节目标增加/减少角度，默认 1° |
| 上/下 | 关节角度步长乘/除 2 |
| r | 从新读取的实际关节角重置编辑目标，不回预设姿态 |
| Space | 开始/暂停；每次开始从实际关节角初始化目标 |

通过 `c → Joint control → 左/右选择部件 → Enter` 选择关节控制部件，支持左右臂、腰和头；
完成后按 Space。启动参数 `--joint-step-deg 0.5` 可设置角度步长。`a/d/q/e/n/u/i/m` 在关节
控制下不提交运动。概览和目标页显示选中关节、编辑目标、最近接受目标、实际角度和 SDK 限位。
越限的按键编辑会被拒绝并显示原因，不修改目标、不裁剪到限位。SDK 发送接口为非同步 `moveJ`，
待发送目标合并、暂停清除待发目标、失败退出的规则与笛卡尔控制相同。

菜单只提供三个直接选项，不再分别选择 single/WBC 和 SDK 档位：

| 选项 | 部件 | 应用时执行 |
|---|---|---|
| Joint control | 左臂、右臂、腰、头 | 结束本应用的 WBC，设置该部件 JointPosition |
| Single-arm Cartesian | 左臂、右臂 | 结束本应用的 WBC，设置该手臂 CartesianPosition |
| Dual-arm WBC | 双臂 | 选择 WBC；下一次 Space 才启动会话 |

菜单打开即暂停发送；Esc 取消选择也保持暂停。Enter 应用后清空旧的待发/已提交目标，重新
读取实际反馈，等待 Space 开始。笛卡尔模式用方向键切臂时，也自动设置所选臂的笛卡尔档位并
暂停，避免切到另一侧后意外变成关节控制。档位调用失败时保留 SDK 原始错误并退出。

独立的 End app WBC session、Enable 和 Disable 操作已从菜单移除。WBC 清理由模式切换和
退出自动完成，不自动上电或下电。当前 Cortex 的使能是软件标志，Mock 未用它拦截运动；
使能、档位和本应用的 WBC 会话记录仍可在第 4 页 RPC 中只读查看。

Joint control 调用 `moveJ`；Single-arm Cartesian 调用 `moveEndPose`；WBC 调用
`startWbc(0)` 和 `moveWbcPose`。WBC 左右目标独立，只提交变化的侧，切换编辑侧不暂停；
另一侧继续已有轨迹，结束后保持末端目标。假设同一时刻只有一个客户端负责运动目标。

**暂停发送不等于制动**：Psi 后端未实现 `stopServoMode`；此前接受的目标可能继续执行。
退出不自动下电。RPC 返回成功仅表示接受请求，不表示运动已完成；SDK 内部 `moveEndPose` 超时
最长 30 秒，其他操作也有固有超时。一个线程串行执行 SDK 调用，等待中的模式切换显示在状态栏，RPC 活动也显示在概览及第 4 页；
暂停/退出不能取消已发出的请求。退出等待该调用结束，再完成会话清理。

UI 默认 30 Hz，反馈读取目标频率 20 Hz，目标发送上限 50 Hz。仅发送变化目标，合并尚未发送
的目标；暂停时运动编辑不积累。网络读取或命令失败使应用以非零状态退出，显示原始接口名和
SDK 错误码，不重试运动、不自动恢复旧目标。`--ui none` 是纯文本观察模式，不读取运动按键。

## 诊断含义

Foxglove 可直接导入 [Cortex 调试布局](foxglove/psibot_teleop.layout.json)，连接
`ws://127.0.0.1:8765`；3D 使用在线 R1 URDF。布局包含概览、末端跟踪、关节 PVA、
WBC/Yellow、运行与事件，配置和数据边界见 [布局说明](foxglove/README.md)。

目标页区分编辑值、最近提交成功值和 SDK 实际反馈，误差使用最近接受的目标。四元数顺序 xyzw。
关节位置/速度/加速度来自 SDK；当前 Psi 服务端力矩为零占位，界面明确标为 unavailable。
关节和位姿分别读取，不声称是同一采样周期。显示的 age 是距客户端读取完成的时间；SDK 未
暴露源时间戳，不能计算源反馈年龄。发送/读取频率为本次会话平均值；RPC 耗时不代表求解耗时。
服务端 session、急停和 HQP/Yellow 诊断未通过这份 SDK 暴露。

宽度 >120 列时并排，80×24、60 列为单列滚动；小于 48×12 提示尺寸不足。日志在新建的
`runs/psibot_teleop/<timestamp>-<pid>/`（或 `--log-dir`）中：`sdk.log` 保存原始 SDK 与操作日志，
`session.json` 保存客户端配置（认证信息脱敏）。已有日志目录不会覆盖。

## 无硬件验证

```bash
python3 tests/architecture_boundaries.py .
ctest --test-dir /path/to/test-build -R 'psibot_teleop' --output-on-failure
```

测试包含进程内 Client 替身、真实 SDK + loopback 测试协议服务端的 PTY 场景。真实 Cortex Mock
集成检查由 `tests/mock_integration.py --binary ... --prefix ...` 显式运行，启动独立端口的
`backend=mock` 实例；不会使用外部机器人地址。SDK 包检查对 ARM 仅检查文件、ELF 和符号，
不宣称 ARM 运行验证。安装后需对实际 executable 做动态依赖检查及 PTY smoke。

### 2026-09-10 本机验证记录

标准 algorithm colcon tree 编译并安装到 `/workspace/install/algorithm`；现有 SDK 和 Cortex
源码未修改。使用已安装 Cortex Mock，经安装后的 teleop 执行左右单臂、WBC、档位/上下电和
退出，全流程完成。1 mm 单臂位移的观测误差约为左 0.107 mm、右 0.109 mm。

WBC 的同样输入最终残差约为左 0.552 mm、右 0.549 mm，**未通过 0.3 mm 精度检查**。
现有 Cortex 的 `hasActiveTraj()` 在采样结束后为 false，`tick()` 随后不继续对该目标求解。
本 app 不重复发送相同目标来掩盖该行为，也没有修改 Cortex。集成脚本分别记录控制流程
`complete` 和 `precision_passed`，不能将前者解读为精度验收。完整证据位于工作区
`runs/psibot_teleop_validation/mock_installed_02/result.json`，属于本地生成数据。

### 三模式菜单验证

菜单已合并为 Joint control / Single-arm Cartesian / Dual-arm WBC。单元测试覆盖切换时
先 stopWbc 后 setProfile、等待期间不发送、档位拒绝后退出；PTY 覆盖三个直接选项、头部关节
控制、80×24 / 60×24 / 48×12 菜单可见性、日志隔离与终端恢复。共享 renderer 默认样式不变，
本应用关闭次要文字的 dim，保证浅色终端的可读性。

真实 SDK + 独立 Cortex Mock 使用统一模型和 MCL 初始姿态验证了关节步进、左右单臂、WBC
以及 WBC 切回关节控制。流程通过；WBC 约 0.55 mm 的既有残差仍未通过 0.3 mm 精度检查。
本地证据：`runs/psibot_teleop_model/mode-ui-mock/result.json`。集成脚本支持
`--motion-config <prepared-motion-config.json> --initial-pose mcl_initial` 使用已准备的模型配置。
