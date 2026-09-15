# 历史研究证据与离线分析支持

本目录保留 A01/A02 及独立核验实际消费的能力：

- evidence / check_inventory：manifest v1/v2、来源与 artifact 哈希读取；
- inputs：独立 XML FK、canonical 输入和已归档 descriptor 的验证；
- metrics / verify / progress：原指标、逐单元核验及核验进度；
- analysis：固定来源的分析、表与重绘；
- repair：旧补证来源绑定、计时准入和几何核验，原批次与补证分母分开；
- freeze：E12 输入访问所需的冻结前置条件核验；
- relabel_metrics：从旧值生成独立的离线角色修正 artifact。

旧 study、campaign、continuation、补证执行器及一次性迁移器不再维护。
新实验使用 [公开 app 执行](../app_execution/README.md)。
旧数据仍在原位置；来源缺失明确失败，不替换 run 或重写旧 validation。

```bash
python3 -m unittest discover -s tools/mcc_placo_study/tests -v
python3 tools/mcc_placo_study/check_inventory.py /absolute/archived/run/inventory.json
```

分析使用 [A01](../../analyses/A01_priority_and_robustness/README.md) 和
[A02](../../analyses/A02_runtime_and_scheduling/README.md) 的固定声明与窄范围测试命令。
完整源码冻结和旧执行指引见 [历史归档](../../docs/archive/README.md)。
