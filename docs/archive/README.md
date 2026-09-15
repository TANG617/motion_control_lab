# 历史归档

历史文档保留当时的设计、命令、方法与证据范围；现役入口见 [文档索引](../README.md)。
归档不提供旧 executable 转发，也不把历史失败改写为新方法结果。

- [原实验架构蓝图](design/experiment_architecture.md)：含未实现的通用端口、发布与可视化设计。
- [旧实现映射](design/project_mapping.md)、[旧应用架构](design/app_component_architecture.md)。
- [MCC 0.4 迁移](migrations/mcc_0_4_migration.md)。
- [公开应用迁移验收](study/APP_REUSE_ACCEPTANCE.md)、[肘部参考命名验收](study/ELBOW_REFERENCE_NAMING_ACCEPTANCE.md)。
- [历史分析验收](study/ANALYSIS_ACCEPTANCE.md)、[补证验收](study/EVIDENCE_REPAIR_ACCEPTANCE.md)。
- [PiM 评估](pim/PIM_IK_EVALUATION.md)：不能代表当前原生 HARP。
- [本地证据链接与缺项](local_evidence.md)。
- [MCL cleanup 验收](cleanup_20260914.md)。

2026-09-14 cleanup 前的完整工作树快照包含未提交源码，位于
`/workspace/runs/mcl_cleanup_20260914T104347Z/source-before.tar.gz`，逐文件 SHA256 为同目录
`source-before.json`，另存 HEAD、Git 状态及 binary diff。五个未改变的源码符号链接
另存 `source-before-symlinks.tar` 与 `source-before-symlinks.json`，恢复工作树时与主快照一起使用。旧执行器、被退役 app 和 OpenSoT
均可从该明确快照追溯；当前仓库不保留第二套可执行历史源码目录。
原始数据、Exx/Axx runs 与冻结安装仍在原位置。
