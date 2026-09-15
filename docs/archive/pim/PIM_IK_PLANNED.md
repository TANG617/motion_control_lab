# 历史 PiM 接入迁移

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

在线 PiM Python/socket 路径已退出。当前入口、模型合同与失败语义见 [HARP.md](../../../apps/hierarchical_kinematics_step/HARP.md)。

历史 PiM 原始录制仍可通过 `--elbow-reference recorded --elbow-recorded FILE` 读取；原始失败状态不变。历史 Foxglove 布局保留用于读取旧产物，实时 HARP 使用 [当前布局](../../../apps/hierarchical_kinematics_step/foxglove/harp.layout.json)，录制回放使用 [中性布局](../../../apps/hierarchical_kinematics_step/foxglove/recorded_elbow_reference.layout.json)。旧源码已随迁移前快照保留。
