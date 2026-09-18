# 公开 app 实验执行

Exx 持有场景、选择、repeat 和来源；app 持有任务、solver、接受策略和规划。
本目录处理 JSON、路径、哈希、逐进程串行执行和独立证据核对。

## 支持的执行结构

| app | 结构 |
|---|---|
| baseline | 冻结公开 replay，外围机械 CSV 转换 |
| step / target | 实际双臂 hard-pose ServoStep / TargetSolve，MCC/PlaCo |
| hierarchical_kinematics_step | 原 Red/Yellow、命名布局、真实调度和规划 replay |
| optimization_problem | 显式矩阵 weighted QP / HQP |
| planned_kinematics_step | CartesianTrajectoryPlanner → weighted IK → JointTrajectoryPlanner |

weighted 不是历史 study soft-task 方程的别名；新方法需要独立准入。

## 声明与请求

现役声明为 experiment.v2，带 `execution_contract=execution_request.v1`。
每个 unit 提供 case_id、arm_id、method_id、app、execution_structure、input descriptor、
config、execution、observation、required；可声明 repeats 和 available/unavailable_reason。
input descriptor 引用具有明确绝对路径和 SHA256 的 canonical JSON。

原生声明无需 legacy_definition 或 migration；原生配置不合法记为 configuration_invalid。旧迁移声明保留这两个成对字段及完整来源绑定；
读取时校验来源哈希和逐单元绑定，retired_research_method/native_unsupported 不会变成 ready。
`prepare.py --output FILE` 默认验证并逐字节复制本实验当前声明；
`--definition FILE` 显式选择另一个现役声明。已有输出拒绝覆盖，不转换旧研究方程、不生成输入。

app 请求保持 execution_request.v1。`--describe-capabilities` 不初始化模型；
`--request FILE --dump-resolved-options` 不创建运行输出，请求模式拒绝额外算法 CLI 覆盖。
tracking 只用于来源追踪，不参与算法分支。baseline 继续使用冻结公开 CLI。

## 操作

从 Lab 根目录；下列输出路径需尚不存在，安装前缀需匹配所选源码和依赖：

```bash
python3 experiments/E06_solver_semantic_parity/prepare.py --output /absolute/new-E06.json
python3 tools/app_execution/run.py --definition /absolute/new-E06.json \
  --install-prefix /absolute/lab/install --dry-run
python3 tools/app_execution/run.py --definition /absolute/new-E06.json \
  --install-prefix /absolute/lab/install --case servo_snapshot_velocity_0.0 --repeat 0 --smoke --execute
```

完整 dry-run 检查声明、输入哈希、实际能力和配置；临时请求只写系统临时目录，不启动 solver。
未指定 definition 时选择当前 E06–E12 公开声明。来源缺失、碰撞 URI 不支持、CPU 未冻结均保留实际原因。
不下载或替换碰撞模型，不关闭约束。

执行写入对应 Exx/runs 的新目录，保留声明、请求、命令、stdout/stderr、原生退出码、
动态库、原生产物和独立核验。原生失败不因独立核验通过而改变。
--smoke 为开发窗口，单进程上限 30 秒；所有被测 app 串行。当前执行不宣称正式 RT 条件成立。

```bash
python3 -m unittest discover -s tools/app_execution/tests -v
```

研究方法与状态见 [研究索引](../../docs/mcc_placo_study/README.md)。
