# Experiment template

复制本目录建立新的 Experiment，并至少提供：

- `definition.json`：符合 `contracts/definitions/experiment.v1.schema.json`；
- `README.md`：说明运行方式、输入边界和人工复核项；
- `runs/`：本地 append-only 执行证据；
- `results/`：仅保存人工复核后晋升的结果。

Experiment 负责重新运行被测系统并产生证据。只消费既有 result 的工作应建立为
Analysis。

