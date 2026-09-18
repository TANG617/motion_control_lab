# R1 joint path planning

键盘编辑 TCP 目标，调用 app 自己的 IK 求解并验收，再把活动关节名称和解传入 MCC
`JointPathPlanner`，生成避障路径、时间参数化并执行运动学动画。规划器的到达目标仍是关节配置，
不调用 IK。默认 `whole-body` 模式启用双臂加腰部共 16 个关节：目标臂到达新 TCP，
另一侧 TCP 在 `base_link` 中保持位置和姿态，腰部与保持臂关节可以协调运动。
头、腿、手指关节保持请求起点配置；完整机器人几何仍参与环境碰撞检查。

这个入口分别支持 `configs/cube.json` 的立方体绕障与 `configs/free.json` 的无障碍直达。
它需要离散关节空间搜索，因此独立于现有 CartesianTrajectoryPlanner → IK → JointTrajectoryPlanner app。
输入/TUI 复用 `hierarchical_kinematics_step` 所使用的共享组件；业务实现完全保留在本目录。

## 启动

默认使用用户指定的 **`/workspace/models/Psi_R1_visual_collision.urdf`** 及其相邻 `meshes/`。
直接加载原始 collision mesh，不生成替代模型。安装后从 `/workspace` 运行：

```bash
python3 labs/motion-control-lab/apps/joint_path_planning/scripts/run_keyboard.py
```

脚本启动已安装的 `/workspace/install/algorithm/bin/mcl_joint_path_planning`，同时启动带 CORS
的模型资源 HTTP 服务。它不会构建。`MCL_BINARY` 可显式指定其他构建产物，
`MCL_INSTALL_PREFIX` 可指定安装前缀。`--urdf`、`--config`、`--output` 可覆盖路径，
`--dry-run` 只打印启动命令。

1. Foxglove 连接 `ws://127.0.0.1:8765`，选择 Foxglove WebSocket 数据源。
2. 导入启动脚本打印的 `foxglove.layout.json`。
3. TUI 初始草稿已加载演示目标，按 **Enter** 即可完成 IK → 避障搜索 → 时间参数化 → 执行。
4. 到达后可以继续修改目标再按 Enter。`r` 把草稿重置到当前 TCP；`p` 从当前 TCP 加载配置中的偏移。

HTTP 默认 `127.0.0.1:8766`。远程 Foxglove 需转发两个端口，或显式设置
`--asset-host 0.0.0.0 --asset-url-host <可访问的主机名> --host 0.0.0.0`。
模型资源服务与 native WebSocket 的绑定地址分别设置。

## 默认场景

- `base_link` 坐标系下，20 cm 立方体中心为 `(0.600, 0.075, 1.200) m`。
- 默认左臂目标在初始 TCP 基础上平移 `(0.080, 0, 0.100) m`，保持姿态。
- 起点和 IK 终点有效；连接两者的关节直线穿过障碍物；OMPL 搜索出绕行路径。
- 初始配置沿用共享 R1 默认姿态，仅将左右 `arm_joint2` 显式设为 `-1.5/+1.5 rad`，
  22 个手指关节显式设为零。输出按完整模型的 42 关节顺序记录。
- 按当前用户设置，**忽略全部自碰撞**：两份默认配置均显式允许 34 个已建模连杆的全部 561 对。
  仍检查全部机器人几何与外部障碍物的碰撞；默认立方体场景保留 34 对环境检查，clearance 为 5 mm。
  这些规则针对当前指定的 R1 URDF，运行时不自动增加豁免。URDF 缺少碰撞几何的连杆
  在 `geometry.json` 和 Collision 页面列出，不能把这些连杆解释为已通过碰撞认证。

首版 clearance 为 5 mm；搜索连接段最大步长 0.01 rad，最终复查为 0.005 rad。
当前 app 默认原规划软预算 50 s、额外路径缩短预算 1 s、seed 42，仅接受 exact path。
Core 自身默认仍为 5+1 s。原预算包括 Core 搜索及其验证，缩短阶段单独计时；app 的 IK、
时间参数化和绘图不计入该预算。正常运行复用 Core 的直连结果与最终验证结果，
不再重复整路径检查或逐时间样本碰撞检查。性能受模型与机器负载影响，
超时保留真实拒绝结果，不改变目标或放宽约束。

Core 公开接口按 `planning/path/`、`planning/trajectory/` 和 `planning/validation/` 分层。
`JointPath` 只保存几何 waypoint；路径结果中的 `pose_constraint` 显式传给独立轨迹验证器。
本 app 的 smooth 模式默认使用独立验证器验收轨迹；可通过 `--smooth-validation none` 显式跳过二次验收。
状态中的 `linear_segments_before_reduction` / `linear_segments_after_reduction` 表示简化前后
最大单调共线段数量；它们是几何指标，实际停车信息仍由轨迹诊断提供。

## 路径缩短与质量

拿到 exact path 后，Core 用 OMPL 在连接段内部尝试 shortcut，最多 256 次，
连续 32 次无改善停止。新增连接继续检查当前碰撞规则和 held TCP 约束，随后对完整路径
加密复查。只保留归一化关节长度严格减少的候选；优化到时可以返回已验证路径。
取消、后端错误或最终复查失败仍拒绝执行。

`--simplification-budget SECONDS` 设置额外预算，默认 1，0 表示关闭；request JSON
对应 `simplification_budget_s`，也写入 `resolved.json`。显式使用 5+1 秒：

```bash
python3 apps/joint_path_planning/scripts/run_keyboard.py --budget 5 --simplification-budget 1
```

Path 页面显示归一化长度 before → after、缩短百分比、尝试数/成功数、耗时和终止原因。
Foxglove 的 Path quality 页显示前后长度曲线；`/joint_path/status` 和 MCAP 同时记录
`path_length_before_simplification`、`path_length_after_simplification`、
`path_shortening_percent`、`simplification_attempts`、`accepted_shortcuts`、
`simplification_ms`、`simplification_termination`。未得到 exact path 时不填造长度值。
三维路径和时间参数化使用最终接受的路径，共用 `/mcl/joints/*` 和 `/mcl/cartesian/*`
话题保持不变。

Accepted 表示本次检查通过；它不保证全局最短或 TCP 最短。结构字段变更后须配套重编
Core、MCV 和 App，避免混用旧二进制。

## 共线贯通与阶段计时

App 默认 `--timing-mode straight-through`：同向共线路点共享一次标量 Ruckig 运动，
内部经过点的速度、加速度不再归零。真实转角、方向反转仍停车，起终点保持静止。
共线判定只接受浮点舍入误差，时间参数化不平滑、不偏离 Core 最终接受的关节折线。
原路点精确位置仍出现在轨迹样本中；采样间隔可能不均匀，执行遵循时间戳。

`--timing-mode stop-at-waypoints` 恢复逐路点停车。请求与 resolved 字段为 `timing_mode`；
Core 公共接口默认仍为逐路点停车。两种模式均保持 V/A/J 限制与 held TCP 约束，
不支持运动中接续；jerk 有界但不承诺连续。

TUI 的 CHECK（端点和直连）、SEARCH、SIMPLIFY、VERIFY 对应 Core 实际阶段。
Path 页面及 `/joint_path/status` 显示原路点数、贯通点数、停车点数、模式与参数化耗时。
`verification_ms` 是 Core 最终验证耗时；`request_ms` 覆盖 IK、规划、参数化与预览。
`app_verification_ms`、`app_path_checks`、`app_timed_state_checks` 在正常链路均为零。
`collision_queries`、`narrowphase_calls`、`broadphase_skips`、`pose_checks` 分别计数，
快速查询未计算的全局最近距离不填造。失败碰撞对与全局最近碰撞对分开报告。

验证复用仅适用于同一不可变场景、已接受路径和保持几何的时间参数化，证据来自 Core 的区间验证，不扩展到未建模几何或变化后的场景。
独立逐时间样本复核保留在验收脚本，不放在正常执行链中。

## IK 任务配置

默认方法标识为 `whole-body-held-tcp-v1`。左右目标臂各有独立 TargetSolve 求解器，
共同使用如下 16 关节顺序：

```text
torso_pitch_joint, torso_yaw_joint,
left_arm_joint1 ... left_arm_joint7,
right_arm_joint1 ... right_arm_joint7
```

- 目标侧 PositionTask / OrientationTask 为 Scaled，共用该侧 progress group，权重 100。
- 另一侧的 PositionTask / OrientationTask 为 Hard；每次 Enter 从当前执行配置获取保持目标。
  切换手臂后重新捕获，不使用另一侧草稿，也不把保持臂关节配置锁死。
- 16 维 PostureTask 为 Soft squared-L2，权重 10，reference 固定为启动初始姿态，
  不参与收敛判定。IK 内部收敛阈值为 1e-5 m / 1e-4 rad，给路径保持留出余量；
  调用方仍显式验收 disposition、converged 和各任务误差。失败不提交给 OMPL。
- app 把 solved 关节值与保持目标分别交给路径规划接口。OMPL 只做关节空间搜索，
  路径全过程保持另一 TCP 在 1 mm / 0.01 rad 内；保持目标包括共享 R1 的 TCP offset。
- Core 对实际关节折线做自适应位姿误差上界检查，碰撞与位姿使用相同的自适应区间细分。
  时间参数化保留折线；app 复用同一场景下的 Core 最终验证。
  不对时间参数化后的配置做 IK 投影，不自动放宽容差或切换模式。

显式选择 `--planning-mode single-arm` 可运行原来的 7 关节基线；此模式其余关节固定，
不附加 TCP 保持约束，方法标识仍为 `scaled-pose-initial-posture-v1`。
例如启动脚本后追加 `--planning-mode single-arm`。默认模式不会在失败时回退到基线。
这属于全身运动学协调，不是动力学 WBC，也没有接触力或平衡控制。

Core 的 primitive TargetSolve 支持 Scaled 关节增量求解；不使用输入关节速度作为 scale 基准。
`resolved.json` 记录方法标识和权重；`attempt-N.json` / status 记录 group 名称、scale 和 posture 误差。
初始姿态偏好不能保证任意 TCP 目标可达，也不约束 OMPL 中间路点的姿态偏好。

## 交互

| 按键 | 行为 |
|---|---|
| 左/右箭头 | 选择左/右臂 |
| W/S、A/D、Q/E | 沿基座 X/Y/Z 调整 TCP 草稿 |
| 上/下箭头、M | 增减平移步长、输入步长 |
| N、I/U | 选择旋转轴、正反旋转 |
| R、P | 重置到当前 TCP、加载配置的演示目标 |
| Enter | 提交一次 IK 和规划；全部通过后自动执行 |
| Space | 冻结/恢复轨迹时钟 |
| C | 取消正在进行的规划 |
| 1–7 / F1–F7 / Tab | Overview、IK、Path、Collision、Trajectory、Joints、Runtime |
| PgUp/PgDn/Home/End、H/? | 滚动、帮助 |
| X / Esc / Ctrl+C | 退出并等待工作线程停止、恢复终端 |

规划、执行和暂停期间禁用目标编辑、手臂切换和重复提交；页面导航继续可用。
暂停是运动学展示的时钟冻结，不是动力学制动。中间路点停车，起终点静止。
本 app 不向硬件发指令，也不做 MuJoCo 动力学仿真。

## Foxglove 和证据

| 显示 | 来源与含义 |
|---|---|
| 实体机器人 | `/mcl/joints/execution` 完整关节状态 |
| 半透明机器人 | `/mcl/joints/ik` 最近一次经 IK 验收的终点候选；不代表路径已接受 |
| 橙色立方体 | `/joint_path/scene`，与碰撞世界同一配置 |
| 红线 / 灰线 | 无效 / 有效关节直连的 TCP 投影 |
| 青线路点 / 绿线 | 已接受关节路径的 TCP 投影 / 已执行轨迹 |
| Draft、Submitted、TCP 坐标轴 | 编辑目标、提交目标、当前 TCP |
| 保持目标坐标轴 | `/joint_path/held_tcp`，每次请求捕获的另一侧 TCP |
| 保持误差曲线 | `/joint_path/status.held_position_error_m` / `held_orientation_error_rad` 与容差 |
| Collision geometry 图层 | 默认隐藏，可开启查看 URDF 的碰撞几何 |
| 曲线与原始诊断 | `/joint_path/status`，位置误差、采样距离、限制比值和 nominal v/a/jerk |

共用话题直接使用与 `hierarchical_kinematics_step` 相同的 `contracts/visualization` 常量和
Foxglove schema；左右臂始终有独立话题：

- `/mcl/cartesian/input/{left,right}`、`/mcl/cartesian/goal/{left,right}`：键盘草稿 TCP。
- `/mcl/cartesian/reference/{left,right}`：提交给 IK 的 TCP 目标，仅发布本次提交的手臂。
- `/mcl/cartesian/ik/{left,right}`：已接受 IK 配置对应的双臂 FK。
- `/mcl/cartesian/execution/{left,right}`：当前完整执行配置对应的双臂 FK。
- `/mcl/joints/ik`、`/mcl/joints/execution`：完整模型关节顺序。

没有已接受 IK 解时不发布 IK 话题；新请求期间 Foxglove 保留最近一次已接受的 IK 样本。
不再发布旧的 `/joint_path/current`、`goal_joints`、`draft`、`tcp`、`submitted` 别名。
避障专属 `/joint_path/scene` 与 `/joint_path/status` 保持独立，未复用消息结构不同的共享 telemetry 话题。

TCP 曲线用于观察；完整身体是否可行取决于关节路径碰撞验证。距离曲线是离散采样证据。
暂停时 v/a/jerk 仍表示轨迹的**名义导数**，不表示实际执行速度。JointStates 只发布位置。
场景消息携带完整 `foxglove.SceneUpdate` JSON schema。障碍物、路径和停车标记仅在变化
或客户端新订阅时发送，通过实体 ID 更新/删除。蓝色球标记实际停车点；原折线仍完整显示。
执行轨迹与机器人状态持续更新。Foxglove 新增 Stops 与 Stage times 曲线。

每次启动创建新目录，已有目录报错。native 输出包括：

- `resolved.json`：参数、原始 argv、场景、URDF/config 哈希。
- `geometry.json`、`native.log`：碰撞几何覆盖与底层日志。
- `attempt-N.json`：本次 IK/路径/时间参数化诊断。
- `attempt-N-path.csv`、`attempt-N-trajectory.csv`：仅接受的路径和完整 q/v/a/jerk 样本。
- `status.json`：退出状态；`visualization.mcap`：约 30 Hz 可视化与诊断，完整时序以 CSV 为准。

launcher 另外记录 URDF 及所有引用网格的 SHA-256、资源 URL 与布局。
退出后 HTTP 服务关闭；离线打开 MCAP 时可重新启动资源服务：

```bash
python3 labs/motion-control-lab/apps/joint_path_planning/scripts/run_keyboard.py --serve-only
```

## 无界面和请求模式

```bash
/workspace/install/algorithm/bin/mcl_joint_path_planning \
  --headless --no-viz --output /workspace/build/joint_path_planning/headless-1
```

默认快速遍历轨迹样本，`--realtime` 使用轨迹时钟。`--no-record` 关闭 MCAP。
退出码：成功 0，正常 IK/规划拒绝 2，原生错误 1。`accepted` 表示运动计划获接受，`execution_complete` 单独表示执行已到达终点。
提前退出显示 STOPPED，取消规划显示 CANCELLED。错误不转换成可执行降级方案。

```json
{
  "schema_version": "joint_path_planning.request.v1",
  "config": "/workspace/install/algorithm/share/motion-control-lab/apps/joint_path_planning/configs/cube.json",
  "urdf": "/workspace/models/Psi_R1_visual_collision.urdf",
  "output": "/workspace/build/joint_path_planning/request-1",
  "side": "left",
  "goal": {"delta": [0.08, 0, 0.10]},
  "budget_s": 5,
  "simplification_budget_s": 1,
  "timing_mode": "straight-through",
  "seed": 42,
  "record": true,
  "viz": false
}
```

`--request FILE` 走相同 solver/规划/执行链，只允许额外的 `--dump-resolved-options`。
`--describe-capabilities` 不加载模型；请求中的路径按进程工作目录解析。

## 构建与验证

基础依赖为 MCC 0.5（启用 `joint_planning`，OMPL 1.7+）、MCV Foxglove、FTXUI。
开关默认关闭：`MCL_BUILD_JOINT_PATH_PLANNING=OFF`。标准 workspace 构建入口：

```bash
cd /workspace
colcon build --packages-up-to motion_control_lab \
  --cmake-args -DMOTION_CONTROL_CORE_BUILD_JOINT_PLANNING=ON -DMCL_BUILD_JOINT_PATH_PLANNING=ON
```

独立 CMake 时将 `motion_control_core_DIR` 指向启用了该组件的安装包，指定
`CMAKE_INSTALL_PREFIX`，再构建 `mcl_joint_path_planning` 和安装。基础 MCC 消费者仍不需要 OMPL。

本次工作区验收的 executable 已安装到 `/workspace/install/algorithm/bin/`，链接的 Core 为
`/workspace/build/mcc-joint-planning/install/` 中的配套安装，保留了标准前缀中原有 Core。
此开发安装需要保留该依赖目录；后续标准 colcon 构建会使用其选定的 Core 安装路径。
关闭所有交互 app 的最小构建同时设置 `MCL_ENABLE_TUI=OFF`。

```bash
ctest --test-dir <build> -R 'mcl_joint_path_planning' --output-on-failure
```

`tests/solver.cpp` 检查单臂姿态策略及左右两侧 16 关节保持求解。
`tests/contract.py` 显式使用 `single-arm` 基线和指定的 R1 URDF 检查真实绕行、无障碍直连、非活动关节、路径保持、
路点停车、导数限制、超时和 IK 拒绝、请求模式，以及真实 PTY 的目标输入和暂停/恢复。
`MCL_JOINT_PATH_TEST_URDF` 指定测试模型；模型不存在时 CMake 不注册该集成测试。
另可对录制文件运行独立 Pinocchio/Coal 网格查询与 MCAP/schema 检查：

```bash
python3 apps/joint_path_planning/tests/observability.py --run <包含 visualization.mcap 的 native 输出目录>
```

该验证脚本需要 Python mcap、jsonschema、protobuf、numpy、Pinocchio 和 Coal；
独立网格复查采用每段 9 个采样点，Core 的最终验证在配置空间步长基础上增加自适应区间上界检查。
对于 whole-body 录制，该脚本还检查每段 101 点与全部轨迹样本的保持误差、
腰部与保持臂补偿、非活动关节固定、折线保持和实际停车边界。这里只需一个固定场景，
不需要批量 seed 或全量 benchmark。
GUI 的图层外观仍需在 Foxglove 客户端目视确认；协议/schema 测试不替代这一步。

实现分布：`main` 装配；`solver` 显式 IK/FK；`planning` 碰撞场景与 MCC 规划；
`loop` 输入、工作线程、执行时钟、证据；`tui_projection` 和 `visualization` 生成展示内容。
当前只支持静态场景；区间结论依赖已加载几何及数值后端，不承诺动态避障或硬实时规划。

### 区间验证与平滑模式

默认 `--timing-mode straight-through` 保持最终关节折线，真实转角继续停车。
规划内部增加 `PRUNE` 路点删除，`VERIFY_PATH` 使用自适应区间验证。

显式指定 `--timing-mode smooth` 可启用 TOTG + Ruckig。R1 转动关节默认允许偏离原路径
0.005 rad，可用 `--smoothing-revolute-deviation` 设置；固定关节不允许变化。
`--smoothing-budget` 默认为 5 秒，覆盖平滑生成与最终曲线验证，独立于原来的 50+1 秒规划预算。
JSON request 对应 `smoothing_budget`、`smoothing_revolute_deviation`。
原有 held TCP 容差及 5 mm clearance 不随偏差选项放宽。

默认 `--smooth-validation full` 要求 `VERIFY_TRAJECTORY` 通过才执行；不通过时报告原因，不自动切换模式。
灰线显示规划参考折线，青线显示实际平滑曲线，蓝球标记静止边界。
执行与预览使用同一曲线，公共 `/mcl/joints/*`、`/mcl/cartesian/*` 话题不变。
状态新增删除点数、停车数变化、平滑缩放次数、最终曲线验证耗时和区间 clearance 下界。

## Smooth 二次验证开关与错误日志

```bash
python3 apps/joint_path_planning/scripts/run_keyboard.py --timing-mode smooth --smooth-validation none
```

`--smooth-validation full|none` 默认 `full`，request JSON 使用 `smooth_validation`。
`full` 保留独立轨迹验证器的碰撞、姿态保持及全曲线偏差验收；`none` 跳过整个二次验收，
仍要求路径规划和轨迹生成成功，保留生成器的关节限制检查、取消和预算处理。
该选项只在 smooth 模式生效，其他 timing 模式保持原行为。

`resolved.json` 和状态记录所选模式；TUI 与 `trajectory_validation_status` 明确区分
`skipped`、`passed`、`failed`、`not_run`（尚未调用）及 `not_applicable`。
跳过时二次验证耗时、碰撞查询数为零，不报告曲线 clearance 或全曲线偏差证明。

时间越界错误包含 `query_time_s`、`valid_range_s`、`query_minus_end_s`，使用足以区分相邻
浮点数的精度。致命错误同时写到终端和本次输出目录的 `native.log`，进程仍以失败退出。
此开关不改变曲线的合法时间范围，也不掩盖时间越界。
