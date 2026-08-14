# E01：PlaCo C++ R1 Smoke

E01 使用 vendored PlaCo C++ 核心和固定模型
`/workspace/models/r1.cos.urdf`，验证 R1 左臂位置 IK 与实验 evidence 链路。
它和 E04 使用完全相同的按名称映射初态、目标态、`left_arm_ee_link`
位置目标、100 次迭代上限与 `1e-5 m` 收敛阈值；目标位置由声明的目标关节状态
做 FK 得到。

R1 URDF 含 HTTPS mesh。E01 以
`RobotWrapper::IGNORE_COLLISIONS | RobotWrapper::IGNORE_GEOMETRY` 加载模型，
因此 smoke 不下载几何资源，也不改变 PlaCo 原有 `IGNORE_COLLISIONS` 的含义。

研究问题、输入、arm、控制条件、指标、guardrail 和 artifact contract 均固定在
[`definition.json`](definition.json) 中。运行会保存原始 URDF 副本、模型哈希、
依赖源码指纹、逐迭代 trace、status、tidy metrics、report 和带 artifact 哈希的
manifest。

PlaCo 固定为 `v0.9.23` 对应提交
`e6c288604639d67b979a16cb2ad26913413c8e3a`，源码是
`third_party/placo/` 下不含嵌套 Git 的普通文件。完整来源和 Lab 补丁见
[`MOTION_CONTROL_LAB.md`](../../third_party/placo/MOTION_CONTROL_LAB.md)。

从仓库根目录构建和运行：

```bash
cmake --preset dev
cmake --build --preset dev --target e01_placo_smoke -j8
./build/dev/e01_placo_smoke
```

默认 evidence 目录为：

```text
runs/<run-id>/
  manifest.json
  definition/resolved.json
  inputs/psi_r1_cos/{canonical_copy.urdf,metadata.json}
  arms/placo_v0_9_23/psi_r1_cos/{trace.csv,status.json}
  evaluation/{metrics.csv,report.md}
```

自动验证：

```bash
ctest --preset dev --output-on-failure -R 'contracts.e01_definition|contracts.r1_smoke_pair|experiments.e01_placo_smoke'
```

只有求解收敛、关节限位和有限值 guardrail 通过，且 evidence bundle 满足
definition 的 artifact contract 时测试才通过。耗时数据只验证采集链，不构成性能结论。
