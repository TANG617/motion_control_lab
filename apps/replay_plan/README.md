# Replay plan inspection

`mcl_replay_plan` 通过共享 MCAP/CSV typed pipeline 加载输入、配对双臂 pose 并生成
canonical timeline trace 与 manifest；输出 matched/unmatched 帧计数，不运行 solver 或发送机器人命令。

```bash
${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_replay_plan --help
```

按 help 指定 --input、输入格式和独立 --output-dir 目录。输入须满足声明的 topic/mapping 与时间策略；
原始录制只读。运行说明见 [构建与运行](../../docs/build_and_run.md)。
