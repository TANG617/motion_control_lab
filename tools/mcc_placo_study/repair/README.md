# 历史补证的离线读取与核验

保留 evidence-repair.v1 的原始合同：A01/A02 原批次与 supplemental_units 分开计数，
308 个补证单元的历史身份、失败和来源哈希不改写。

common 读取固定 manifest/index、检查声明/输入/运行绑定，并为旧计时证据核对原审计准入。
E09 的 audit_gate 由实验目录 verify_evidence.py 持有；kinematics 保留独立几何 FK/Jacobian。
这些模块不运行 solver。旧补证准备、运行、重现及 smoke 入口已退役。

```bash
python3 -m pytest -q tools/mcc_placo_study/repair/tests
```

旧科学范围与验收见 [历史研究记录](../../../docs/archive/README.md)。
