# Planned Grouped Step OTG Foxglove 合同

版本：`mcl.foxglove_planned_grouped_step_otg.v1`

机器可读合同见
[`contracts/visualization/foxglove_planned_grouped_step_otg.v1.json`](../contracts/visualization/foxglove_planned_grouped_step_otg.v1.json)。
topic 常量与 `ChannelSpec` 由该 JSON 在 build tree 生成，不维护 checked-in C++ 合同副本。

本 app 保留基础 IK 合同，但基础 `/mc/ik/joint_states` 与 `/mc/fk/pose/*`
表示同一拍 accepted raw IK 的 P/V 和 FK。app 在一次 `RenderBatch` 中追加并发布
JointPlanner 执行扩展：

- `/mc/joint_controller/ruckig_joint_states`
- `/mc/debug/otg/fk/pose/left_end_effector`
- `/mc/debug/otg/fk/pose/right_end_effector`

`/mc/planning/request/{left,right}_pose` 仍表示实际提交给 Red 的 Cartesian
reference。一次逻辑更新只调用一次 sink `write()`，因此 raw IK、planning request、OTG
joint state 与 OTG FK 不会被 transport 拆成两个可独立观察的更新。IK 或 OTG 失败时不提交
staged sample，也不发布部分更新。

可选 visualization MCAP 仅用于开发预览，不替代正式 Lab trace、manifest 或 artifact。
