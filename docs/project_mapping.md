# Motion Control Lab 实现映射

本文将
[《可复用实验架构：从零创建、分析与发布》](./experiment_architecture.md)
中的抽象角色映射到当前仓库。这里只记录已经存在的实现；尚未落地的能力明确标记为
planned。

| 抽象角色 | 当前实现 |
|---|---|
| Raw data | `data/raw/`；当前无业务 MCAP |
| Canonical data | `data/canonical/`；MCAP→CSV schema 与 normalizer planned |
| Definition format | `contracts/definitions/experiment.v1.schema.json` + 每个实验的 `definition.json` |
| Definition validator | `tests/validate_contracts.py definition` |
| Experiment executor | 当前为 `e01_placo_smoke`；通用调度 CLI planned |
| Execution adapter | `adapters/execution/` 中的 append-only artifact store 与 manifest writer |
| Solver source | `third_party/placo/` 中的普通 vendored 源码；直接参与主工程构建 |
| Metric evaluator | E01 执行器内的最小 metric evaluator；领域公共 evaluator planned |
| Manifest contract | `contracts/manifests/run_manifest.v1.schema.json` |
| Metric row contract | `contracts/metrics/metric_row.v1.schema.json` |
| Artifact root | `experiments/<experiment>/runs/<run-id>/` |
| Result promotion | 人工复核后写入 `experiments/<experiment>/results/`；promotion command planned |
| Analysis collector | planned |
| Static renderer | planned；后续只读取落盘 artifact |
| Publisher / Release index | planned |

## 当前数据流

```text
E01 definition + synthetic URDF
            |
            v
 vendored placo C++ source + source fingerprint
            |
            v
 append-only experiment run
   manifest + input copy + trace + status + metrics + report
            |
            v
 contract and hash validation
```

E01 使用合成模型，只用于验证依赖、求解器 API 和证据链。真实 MCAP、
canonical CSV 和业务机器人模型接入后，应建立新的输入合同和实验，而不是把
E01 的 fixture 扩展成业务 benchmark。
