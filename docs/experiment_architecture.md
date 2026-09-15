# MCL 实验与证据生命周期

> 更新日期：2026-09-14

## 当前执行链

```text
明确的 canonical input / model / hashes
  → Exx 当前 definition
  → prepare.py 验证并复制声明
  → tools/app_execution/run.py --dry-run
  → 同一入口 --execute，逐进程串行
  → Exx/runs/<run-id> 的原生产物、独立核验、manifest
  → Axx 固定来源的离线分析
```

E05 是调用 HKS 公开 launcher 的 PTY 批量回放场景；E06–E12 使用公开 app request。
E01–E04 已退役执行实现，保留身份、声明及历史证据定位。不存在现役旧 campaign 执行器。

## 所有权与合同

- Exx 持有问题、方法、case/repeat、输入和评估窗口；算法配置及执行行为属于 app。
- `experiment.v2` 配合 `execution_contract=execution_request.v1` 描述公开 app 单元。
  `--describe-capabilities` 不加载模型；`--request FILE --dump-resolved-options` 只读解析。
- 原生声明不要求迁移字段。历史声明存在 legacy_definition/migration 时仍校验完整绑定，
  原字节、方法差异、退出和能力缺项均保留；新方法不得继承旧方法的准入结论。
- 原始 MCAP、canonical 输入、模型与已有 run 只读。prepare 复制明确声明，不生成输入、替换模型、
  去重研究因素或启动 solver。输入缺失需独立准备，并显式更新新的声明。
- artifact 引用使用明确路径和 SHA256，不通过隐式 latest、改写时间戳或替换来源修复证据。

## 执行与失败

每个选定 repeat 启动独立 app 进程；执行器只管理请求、路径、进程、来源和独立检查。
保存实际命令、stdout/stderr、退出码、resolved options、原生产物、运行库身份。
新输出使用独立目录；不会覆盖历史 run。原生失败、独立核验失败与能力/输入缺项分别记录。

`--smoke` 使用明确缩短的开发窗口和至多 30 秒的单进程上限，不代替完整录制或正式计时。
当前公开执行器声明 `formal_rt=false`；资源冻结、RT 验收和研究准入见 [研究协议](mcc_placo_study/PROTOCOL.md)。
E05 的暂停/退出与统计合同在 [E05 README](../experiments/E05_real_scene_planned_mcap_batch_replay/README.md)。

## 离线分析与历史追溯

A01/A02 仅读固定来源及其哈希，分别管理原批次与补证分母，保留失败、缺项和原 validation。
重新分析或重绘产生新 run。旧 manifest v1/v2、analysis v1/v2 与已有录制读取继续支持。
缺少历史 plan/index/artifact 时明确失败；cleanup 不重建或伪造来源。

本地 runs/results 仍为原来的证据目录，忽略规则不代表可以删除。研究结果不自动晋升或发布。
目前未实现的通用 publisher/框架端口蓝图仅保留在 [历史设计](archive/design/experiment_architecture.md)。
当前操作见 [公开执行接口](../tools/app_execution/README.md)，进度见 [研究索引](mcc_placo_study/README.md)。
