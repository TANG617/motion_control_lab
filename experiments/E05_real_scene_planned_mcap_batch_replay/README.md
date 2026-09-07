# E05：真实场景 planned MCAP 批量回放

从 Lab 根目录运行：

```bash
python3 experiments/E05_real_scene_planned_mcap_batch_replay/run_batch.py
python3 experiments/E05_real_scene_planned_mcap_batch_replay/run_batch.py --dry-run
python3 experiments/E05_real_scene_planned_mcap_batch_replay/run_batch.py --limit 2
python3 experiments/E05_real_scene_planned_mcap_batch_replay/run_batch.py --dataset-dir /another/dataset --recursive \
  --include '*20260731*' --exclude '*broken*' -- --playback-rate 2 --port 8766
```

默认扫描 `/mnt/mcap_dataset` 的直属 `*.mcap`，按相对路径排序，启动时固定清单。
`--include` 和 `--exclude` 可重复，按相对路径匹配；`--limit` 在筛选排序之后应用。
空目录或空选择报错。`--dry-run` 只打印文件与实际命令，不写目录、不运行 app。
`--output-root` 默认是本实验的 `runs/`；`--run-id` 必须是单个目录名，已有批次不能覆盖。

每个文件使用独立进程调用
`apps/hierarchical_kinematics_step/scripts/profiles/planned/run_mcap_interactive.py`。
不导入私有 recipe、不复制 solver/planner 配置。首帧 JointState 初始化、配对、10 ms
时间投影和算法参数遵循原 launcher；native manifest 的 resolved options 是实际配置证据。
默认保留实时 1x、TUI 和 Foxglove，每段自动开始。连接 `ws://127.0.0.1:8765`，
切换文件时服务重启，客户端需要重连。

工具通过 PTY 保留空格暂停/继续、`.` 单步和原有页面导航。原生退出操作（`q` / `x` / Escape）
停止整个批次；Ctrl+C 请求当前 app 收尾，并停止后续文件。若 app 尚在加载或未响应，
10 秒后终止进程组，再过 2 秒强制清理；此时可能没有完整原生产物，批次明确记录中断。
非交互调用也为子进程分配 PTY，但没有键盘输入；可将控制台输出重定向到文件。

`--` 后的参数直接传给 launcher；完整参数见原脚本 `--help`。输入、输入格式、
输出目录、run ID、MCAP 路径和 dump 模式由调度器管理，不能透传覆盖。
自动开始、CSV trace 和故障退出始终由批量工具开启。
其他参数可显式覆盖，例如 `--ui none --viz none --no-terminal-input`，回放输出 MCAP 仍会录制。
若显式指定 `--duration` 提前停止回放，原生 `stopped` 将停止批次并返回 130，不算完成。

## 环境与构建

需要 Linux、Python 3、已安装的 app、URDF/mesh 与其匹配的动态库。启动器不隐式构建。
默认使用 `/workspace/install/algorithm/bin/mcl_hierarchical_kinematics_step`；继续支持
`MCL_BINARY`、`MCL_INSTALL_PREFIX`、`MCL_LD_LIBRARY_PATH`、`MCL_CPU_SET` 和
`MCL_RT_PRIORITY`。默认绑核/调度继承现有 launcher。改变播放速度或录制开关以外的
运行参数，会改变实验条件，必须结合 resolved options 解读统计。

本工具依赖新增的 `--replay-exit-on-fault` 参数，更新后需重建并安装 app。
该参数默认关闭，E05 显式开启；TUI 遇到致命错误时写完产物并退出，单独运行原脚本
仍默认保留 FAULT HOLD 供检查。算法错误判定不变。

## 产物与结果

```text
runs/<run-id>/
  manifest.json
  definition/{resolved.json,batch_options.json}
  inputs/inventory.json
  items/001_<filename-stem>/
    command.json
    console.log
    result.json
    replay/{manifest.json,status.json,trace.csv,visualization.mcap}
  evaluation/{action_summary.csv,failures.json,report.md}
```

外围目录由批量工具创建，`replay/` 由 app 创建。原始数据只读；可视化 MCAP 是 app
生成的机器人回放输出，不包含原始相机视频。`console.log` 保留原始 ANSI 终端字节。
启动失败时保留命令、日志和批量结果，不伪造 native manifest、trace 或输入哈希。

只有退出码为 0、原生状态为 `succeeded` 且所需产物存在才记为完成；失败后继续。
`stopped`、`missing_result`、`interrupted` 与 `not_run` 分别记录。
全部完成返回 0，有失败返回 1，用户停止/中断返回 130。
摘要 CSV 汇总 native 帧数、accepted/rejected、tracking、Red timing；详细嵌套统计保留在
`result.json`。Red P95/P99 是原生 1 us 直方图桶上界，最大值是实测值。
完成状态不代表无丢帧、无 deadline miss 或跟踪精度通过额外验收。

新一轮执行创建新批次；用 `--include` 指定需要重跑的文件。没有自动重试、数据修复或
结果晋升。批次记录 Lab revision/dirty 状态、launcher 和 binary 哈希、显式环境及命令，
完成时对产物建立哈希清单。`results/` 仅用于人工复核后晋升的证据。

## 验证

```bash
python3 -m unittest discover -s experiments/E05_real_scene_planned_mcap_batch_replay/tests -v
python3 tests/validate_contracts.py definition experiments/E05_real_scene_planned_mcap_batch_replay/definition.json
python3 tests/validate_contracts.py manifest experiments/E05_real_scene_planned_mcap_batch_replay/runs/<run-id>/manifest.json
```

目录从 `E05` 重命名后，已有 run 中的原始 argv 和绝对路径继续保留执行时的值，
不改写历史产物及其哈希。历史报告中的相对产物链接仍可使用。
