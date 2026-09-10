# E06 输入合同

状态：`generated-development`。`generated/` 中有真实 canonical 输入与 hash descriptor，正式冻结尚未执行。

输入包含 model locator/hash、mesh policy、full/active joint 映射、TCP、q/qdot、双臂 pose/twist、limits 和任务定义。生产映射来自冻结 baseline 的实际配置；受控组使用同一个物理 R1 模型。r1.cos.urdf 与 Psi_R1_visual_collision.urdf 是候选源，不因名字相近而视为同一模型。

按照 [实验矩阵](../README.md) 生成 reachable FK targets，保存 generator hash、seed、拒绝生成计数与 canonical hash。formal 前冻结模型、关节列表、数值核验容差与全部 snapshots。解析坐标变换 fixtures 属于测试，独立标记。

source time 仅用于识别输入；snapshot 无隐藏状态回灌。每个 case 明确 TargetSolve seed 或 ServoStep measurement 的语义。详见 [公共数据合同](../../../docs/mcc_placo_study/DATA_ANALYSIS.md)。
