# Foxglove IK 可视化数据流合同

版本：`mcl.foxglove_ik_visualization.v1`

本合同适用于所有会发布 IK 可视化数据的 Motion Control Lab app。无论 app 是交互式、
replay 还是实验 runner，都必须使用相同 topic 和语义。非 IK app 不需要、也不得伪造这些数据。
机器可读合同位于
[`contracts/visualization/foxglove_ik.v1.json`](../contracts/visualization/foxglove_ik.v1.json)，
C++ 入口位于
[`contracts/visualization/foxglove_ik_v1.hpp`](../contracts/visualization/foxglove_ik_v1.hpp)。

`mcl_planned_grouped_servo_step` 另外发布在线规划后实际提交给 Red 的 reference；其独立扩展
合同见 [`foxglove_planned_grouped_servo_step_contract.md`](./foxglove_planned_grouped_servo_step_contract.md)。

## 必需数据流

| Topic | Foxglove schema | 语义 |
|---|---|---|
| `/mc/ik/joint_states` | `foxglove.JointStates` | 当前 sample 的 IK 输出关节状态。语义类似 ROS 2 `sensor_msgs/msg/JointState`：`name` 与 `position` 必须完整且对齐；`velocity` 可选，存在时必须使用相同顺序。 |
| `/mc/ik/target/left_pose` | `foxglove.PoseInFrame` | app 收到的左手笛卡尔输入 pose，不得用内部投影结果覆盖。 |
| `/mc/ik/target/right_pose` | `foxglove.PoseInFrame` | app 收到的右手笛卡尔输入 pose，不得用内部投影结果覆盖。 |
| `/mc/fk/pose/left_end_effector` | `foxglove.PoseInFrame` | 使用同一 sample 的 joint positions 计算出的左末端实际 FK pose。 |
| `/mc/fk/pose/right_end_effector` | `foxglove.PoseInFrame` | 使用同一 sample 的 joint positions 计算出的右末端实际 FK pose。 |

这里的 `foxglove.JointStates` 是 Foxglove 原生 schema，不携带 acceleration，也不把
URDF `effort` 冒充 acceleration。若后续需要 acceleration 或 effort，应新增有版本的数据合同，
不能改变本合同字段的含义。

## 一致性不变量

- 一次 visualization update 必须包含上述五条消息，并描述同一个逻辑 sample。
- 五条消息使用同一个 `VisualizationFrame.emit_time_ns`；source/canonical sample time 继续保存在
  Lab frame 和 run trace 中，不用它冒充当前 wall-clock emit time。
- 所有 pose 都显式携带 app 的模型 reference frame，例如 PSI R1 的 `base_link`。
- FK topic 只能发布从同帧 `/mc/ik/joint_states` 计算得到的实际末端 pose，不能复制 target。
- target topic 保留 app 外部输入语义。例如 E02 MCAP 输入是 TCP pose，runner 内部虽然会转换成
  end-effector IK task，Foxglove target topic 仍发布原始 TCP 输入；FK topic 发布实际 EE pose。
- 单臂 app 也发布左右两侧数据：未受控侧 target 固定为其初始 FK，FK 则随同帧完整机器人状态计算。

## Foxglove 使用

连接 app 输出的 WebSocket（默认 `ws://127.0.0.1:8765`）后：

1. Robot State/URDF 面板选择 `/mc/ik/joint_states`。
2. 选择两个 `/mc/ik/target/*` pose 观察输入目标。
3. 选择两个 `/mc/fk/pose/*` pose 观察关节状态对应的实际末端位置。
4. target 与 FK 的位置差才是可视化的笛卡尔跟踪误差；不要比较两个 target topic。

E02 replay 在等待空格阶段也会周期发布完整五通道初始帧，因此 Foxglove 可以在 replay
clock 启动前发现 topic 并完成面板配置。
