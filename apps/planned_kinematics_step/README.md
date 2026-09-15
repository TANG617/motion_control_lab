# planned_kinematics_step

可独立执行的双臂运动回放程序，执行结构固定为：

```text
按时间采样输入目标 → CartesianPlanner → weighted MCC 或 PlaCo ServoStep
                 → JointPlanner OTG → 理想运动学提交状态 → 下一步反馈
```

此程序补齐已有 `step` 无法表达的两个规划器串联结构，可用于独立 CSV/MCAP 运动回放、
规划/IK/OTG 误差定位及多个实验的公开执行入口。HQP 由 `hierarchical_kinematics_step` 提供。
本程序不依赖实验编号；普通入口和请求入口最终调用同一个 `Solver`、`Planning` 和 `loop`。

## 独立命令

```bash
BIN=/workspace/install/mcl-app-reuse-20260914/bin/mcl_planned_kinematics_step
$BIN --describe-capabilities
$BIN --help
$BIN --input apps/planned_kinematics_step/tests/stationary.json \
  --input-format json --config apps/planned_kinematics_step/tests/config.json \
  --output-dir /tmp/planned-stationary-001
$BIN --request /absolute/request.json --dump-resolved-options
$BIN --request /absolute/request.json
```

输出目录必须不存在。`--dump-resolved-options` 只解析并输出配置，不初始化模型/求解器、
不创建输出；JSON/model、CSV/MCAP 的 robot_input 和 mapping 引用仍执行只读哈希检查。`--describe-capabilities` 不读取输入或初始化算法。请求模式只允许上述 dump
修饰符，不允许叠加算法 CLI 覆盖。程序不下载资源、不构建、不打开真实设备、不连接 UI。

## 输入与执行合同

请求使用共享 `execution_request.v1`：

```json
{
  "schema_version": "execution_request.v1",
  "app_id": "mcl_planned_kinematics_step",
  "execution_structure": "cartesian-weighted-ik-joint-otg",
  "input": {"path": "/absolute/input.json", "sha256": "<input bytes SHA256>", "format": "json"},
  "app_config": {"solver": "mcc", "dt_s": 0.01, "duration_s": 1.0},
  "execution": {"runtime_mode": "virtual"},
  "observation": {"mode": "full-raw"},
  "tracking": {"case_id": "any-opaque-id"},
  "output_dir": "/absolute/new-output"
}
```

`tracking` 只进入证据绑定，不参与算法分支。当前只提供 virtual 回放、全量原始记录。
其他 execution/observation 设置明确报错，不会声称已执行真实线程或无观测对照。

JSON 输入可以使用已有 canonical 形状，必需字段为：

- `model.locator`、`model.sha256`：URDF 文件及哈希，加载前验证。
- `joint_names`、`active_joint_names`：完整状态顺序及活动集合。
- `root_frame`、`frames.left/right`、`tcp_offsets.left/right`：目标参考系、末端 frame 和局部 TCP 偏移。
- `initial_state.q/v`：完整初态；不裁剪越界值，不替换为默认姿态。
- `limits.lower/upper/velocity`：供 PlaCo 及 JointPlanner 使用的完整约束。
- `samples[].source_time_s`、`samples[].targets.left/right.{position,rotation}`：按原窗口因果零阶保持的 TCP 目标，米和旋转矩阵。

第一项目标初始化 CartesianPlanner 的起点，后续目标从规划器当前完整 PVA 重规划。
初始规划参考与机器人的初始关节状态是两个独立输入，程序不会把第一个目标当成实测 FK。
执行 clock 将 dt/duration 四舍五入至整数纳秒，在半开窗口 `[0,duration_ns)` 内发布，
release 数由整数除法向上取整，避免 `ceil(0.07/0.01)` 的浮点额外 release；
实际规划器仍使用声明的 dt_s。最后一项输入之后保持最后目标。
解析/采样/初始化不计入 `ik_solve_call_ns`。

CSV 和 MCAP 使用现有 `replay::loadReplay` 数据适配器。`app_config.robot_input` 指定上述
机器人/初态 JSON 的 `{path,sha256}`，其 samples 被实际回放数据替代。
`app_config.replay` 可配置 `left_stream`、`right_stream`、`timestamp_source` 和
`csv_mapping: {path,sha256}`。默认左右流名为 `left`、`right`；CSV 默认时间来源为
`configured_column`，MCAP 为 `header_stamp`。配对为 exact，时间保持原序，不隐式掉帧。
目标 frame 必须与机器人 `root_frame` 一致，不隐式转换坐标系。
CSV 默认列为 `timestamp_ns,left_frame_id,left_x,...,left_qw,right_frame_id,right_x,...,right_qw`；
MCAP 使用 ROS2 `geometry_msgs/msg/PoseStamped` CDR。CSV mapping 格式与已有 replay adapter 相同。

## 应用配置与原生行为

| 字段 | 默认值 | 作用 |
|---|---:|---|
| solver | mcc | `mcc` 或 `placo` 单层 weighted |
| backend | eiquadprog | 当前唯一 backend |
| dt_s / duration_s | 0.01 / 1 | 周期和完整声明窗口 |
| servo_gain_per_s | 1 | 位置、姿态及回中姿态目标的 servo gain |
| posture_weight | 0.001 | 活动关节初态回中任务权重 |
| regularization | 1e-6 | 各原生求解器的正则化配置 |
| cartesian_velocity | 0.1 | 平移速度上限，m/s |
| cartesian_acceleration | 0.5 | 平移加速度上限，m/s² |
| cartesian_jerk | 5 | 平移 jerk 上限，m/s³ |
| joint_acceleration | 2 | 关节加速度上限，rad/s² |
| joint_jerk | 20 | 关节 jerk 上限，rad/s³ |

旋转向量速度/加速度/jerk 上限固定为 1 / 5 / 50；两个规划器使用 Time 同步。
双臂位置与姿态任务均为单层 soft，位置/姿态权重为原生单位权重；回中目标取初态。
MCC 使用 `ModelPositionAndVelocity`，由 URDF 提供原生 IK 关节限制；PlaCo 使用输入中的
位置/速度限制。JointPlanner 使用输入 limits 和上述加速度/jerk 上限。
这些约束来源和速度/增量正则化语义**不能仅凭配置同名推断两种方法方程一致**，
公平对比须另外审计实际方程和接受合同。

MCC 要求 `Status::ok` 且 disposition accepted；PlaCo 必须原生 `solve` 返回，异常直接导致
进程失败。候选未经投影或裁剪，仅原生接受后交给 JointPlanner，后者成功才提交最终状态。
JointPlanner 终点速度/加速度为零，每步从上一步实际提交 PVA 重规划。
原生拒绝写出候选、输入状态、原生状态及本次未提交，立即退出 2；后缀 releases 写为 not-run。
原生异常不被捕获/继续运行，首个异常保留在 stderr；`attempt_begin` 和预写的
`planned_schedule.json` 能定位缺失尝试及未运行后缀。初始化异常发生时只有初始化前已写出的证据。

与历史研究链相比：当前只有全串联结构、不含 HQP、不含反馈延迟模拟、不含故障注入分支、
不含显式限位投影、不在拒绝后 HOLD 继续。不能复用旧方法身份或继承旧方法的性能/科学结论。
初始越界及输入/约束不一致仍通过真实输入保留为可重现负例。

## 输出与验证

每次新输出包含请求、输入/请求哈希绑定、capabilities、最终配置、原始 argv、解析后的输入、
初始化分段计时、全计划 releases、`raw.jsonl` 和正常结束/原生拒绝的 `native_status.json`。
原始行保留参考 PVA、反馈 q/v/a、IK 候选、原生状态/disposition、weighted selected-pass 不适用
理由、单次 IK host 耗时、下游规划器状态、提交 q/v/a、逻辑时间及 wall start/finish。
墙钟耗时只是非 RT 开发观测，不是调度期限证据。

```bash
python3 apps/planned_kinematics_step/tests/contract.py \
  --binary /workspace/build/mcl-app-reuse-20260914/mcl_planned_kinematics_step \
  --artifacts /absolute/new-test-evidence
ctest --test-dir /workspace/build/mcl-app-reuse-20260914 \
  -R '^apps.planned_kinematics_step_contract$' --output-on-failure
```

测试依赖本机 R1 模型（fixture 中有哈希）和 Python `mcap` 仅用于构造回放 fixture。
被测 app 不启动 Python。正例最长逻辑窗口 0.1 秒，每个进程超时 30 秒。
覆盖 MCC/PlaCo 普通与请求入口一致、只读 dump、输出不可覆盖、坏哈希/非法覆盖/不支持
布局拒绝、真实初态越界、移动参考及 CSV/MCAP 共用解码器。
跨进程数值一致门槛为绝对 `1e-12` 的舍入误差，独立于任何算法约束/接受阈值。
