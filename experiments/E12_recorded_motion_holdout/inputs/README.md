# E12 数据来源与待冻结清单

状态：输入准备代码已实现；正式输入/配置冻结和正式运行仍待单独启动。本目录不复制外部 MCAP，不含已冻结 holdout。

默认外部 locator 为 /mnt/mcap_dataset。候选流来自 E05 输入说明：/hal/tracker/htc/left/calib_target_pose、/hal/tracker/htc/right/calib_target_pose、/mc/ik/joint_states；代码阶段逐文件核验，不能假设所有文件 schema 和 frame 相同。

每文件登记 content hash、source/session identity、采集时间、动作区间、topic/frame、质量、重复/重叠、过去 exposure 与未知信息。来源不足时保守分组并限制结论；全部旧数据已暴露时需要新的未暴露会话才能执行真正 holdout。

正式前冻结：inventory、session map、exposure、split、转换配方、模型和初始状态政策、候选配置、settling 与阈值、required methods、重复次数。规则唯一来源为 [DATA_ANALYSIS](../../../docs/mcc_placo_study/DATA_ANALYSIS.md)。

重启或容器重建后重新确认数据可见性；不把当前挂载作为永久环境前提，不自动操作宿主挂载。原始关节输出不是唯一正确轨迹。缺失或不可访问时保留 preflight 原因，不创建空数据冒充真实输入。
