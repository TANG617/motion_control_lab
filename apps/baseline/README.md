# Production-static baseline

`mcl_baseline` 是独立 PlaCo-only 对照，冻结 production revision
`42ed3ce3a19f5a7346874a31ec659c0298751137` 的任务、权重、姿态、joint mask、限位、
周期与 TargetSolve 终止条件。只允许输入输出和展示选择；不读取生产配置、不允许算法 override。

从 Lab 根目录查看已安装入口：

```bash
${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_baseline --help
${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_baseline replay --help
```

支持键盘及 canonical CSV/MCAP replay，输出原生 trace、manifest 和所选可视化产物。
需要匹配的 PlaCo/Pinocchio 与 R1 模型。研究执行器仅机械转换输入并调用冻结 CLI，
不为 baseline 增加第二套 solver 或通用 request 配置。
构建与路径覆盖见 [运行说明](../../docs/build_and_run.md)。
