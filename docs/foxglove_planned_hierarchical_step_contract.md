# Planned Hierarchical Step Foxglove 扩展合同

版本：`mcl.foxglove_planned_hierarchical_step.v1`

本合同仅适用于 `mcl_planned_hierarchical_step`，并扩展基础
[`mcl.foxglove_ik_visualization.v1`](./foxglove_ik_visualization_contract.md)。机器可读合同位于
[`contracts/visualization/foxglove_planned_hierarchical_step.v1.json`](../contracts/visualization/foxglove_planned_hierarchical_step.v1.json)。
topic 常量与带 `ChannelKind` 的 channel spec 由通用 generator 在 build tree 生成，JSON 是
唯一合同来源。

## Planning request 数据流

| Topic | Foxglove schema | 语义 |
|---|---|---|
| `/mc/planning/request/left_pose` | `foxglove.PoseInFrame` | 最近一次实际提交给 Red 的左手 staged planner reference。 |
| `/mc/planning/request/right_pose` | `foxglove.PoseInFrame` | 最近一次实际提交给 Red 的右手 staged planner reference。 |

两条消息描述同一次 Red attempt。无论 Red 接受还是拒绝该 attempt，都发布实际尝试的
reference；若 planner 在产生 staged sample 之前失败，则继续发布上一次有效 request，不伪造新
sample。启动时以 warm-up reference 初始化。

这两个 topic 与基础 target topic 的语义不同：

- `/mc/ik/target/{left,right}_pose` 是与 Red attempt 对应的 source goal。
- `/mc/planning/request/{left,right}_pose` 是经过在线规划后实际送入 Red 的 reference。
- `/mc/fk/pose/{left,right}_end_effector` 是最后 accepted joint state 的实际 FK。

Planning request 随 app 的 base IK projection 追加到同一个 RenderBatch，使用同一个 timestamp 和机器人 reference
frame（PSI R1 为 `base_link`）。本合同只发布 Pose；reference twist 和 acceleration 仍保存在
planned app 的 TUI/debug/trace 数据中。
