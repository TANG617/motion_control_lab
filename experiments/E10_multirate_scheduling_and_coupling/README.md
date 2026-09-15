# E10：多频率调度与耦合归因

> 更新日期：2026-09-14

本实验通过 hierarchical_kinematics_step 的公开入口执行。具体单位、配置、repeat、输入与能力缺项
以 [当前声明](definition.json) 为准；当前声明仍保存原迁移来源与方法差异。
科学准入和状态统一见 [研究索引](../../docs/mcc_placo_study/README.md)。

## 准备与开发执行

从 Lab 根目录运行；prepare 验证并复制当前声明，不生成输入、不隐式迁移：

```bash
python3 experiments/E10_multirate_scheduling_and_coupling/prepare.py --output /absolute/new-E10.json
python3 tools/app_execution/run.py --definition /absolute/new-E10.json \
  --install-prefix /absolute/lab/install --dry-run
```

先查看 dry-run，再显式选择 case/repeat 和 --smoke --execute 做有界开发观测；
完整参数及进程/失败合同见 [公开执行说明](../../tools/app_execution/README.md)。
产物写入本实验 runs 的新目录；旧 runs/results、原声明和输入保持原位。

## 方法和证据范围

本次 app 配置不自动继承旧 study 方程、接受策略、扫描控制因素或性能结论。
迁移后的重复映射、退出方法、原生不支持、资源及输入缺项均保持声明身份；不自动修复或删除。
旧 verify/evidence 工具保留用于原始证据的独立读取和数值检查，不能作为新方法的自动准入。

历史设计、完整指标说明与当时验收见 [原实验记录](../../docs/archive/experiments/E10_multirate_scheduling_and_coupling.md)；
旧声明和生成器位于 declarations，仅供追溯。新方法需要对实际方程、资源和完整输入重新冻结与准入。
