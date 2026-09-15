# 可复用应用与原生 HARP 开发验收（2026-09-14）

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

本轮完成执行接口和目录重构，未启动全量新实验。新方法直接调用实际应用的任务构造、
solver 和规划链。旧 A01/A02 结论继续只适用于原方法与原数据；目录迁移、接口一致性和
HARP 数值一致性都不能替代新的跨方法方程及接受合同准入。

证据根目录：[mcl_app_reuse_20260914T054601Z](/workspace/runs/mcl_app_reuse_20260914T054601Z)。
精确命令和每次失败均保留；本文件中的通过指开发合同验收，不指所有候选求解成功。
最终索引见 [acceptance-index.json](/workspace/runs/mcl_app_reuse_20260914T054601Z/acceptance-index.json)，
源码、独立安装及动态依赖冻结见 [freeze-final.json](/workspace/runs/mcl_app_reuse_20260914T054601Z/freeze-final.json)
和 [source_final.tar.gz](/workspace/runs/mcl_app_reuse_20260914T054601Z/source_final.tar.gz)。

## 应用归属与合同

| app | 实际执行结构 | 使用范围 |
|---|---|---|
| baseline | 冻结公开 CSV/MCAP replay，外围只做机械输入转换 | production-static |
| step | 同一 MCC/PlaCo ServoSolver、单层双臂 hard-pose | E06–E09 |
| target | 同一 MCC/PlaCo TargetSolver、双臂 hard-pose | E06、E09 |
| hierarchical_kinematics_step | 同一 Red/Yellow solver；有限命名布局、同步/分频/异步 batch；原规划/OTG replay | E07–E12 |
| optimization_problem | 显式矩阵 weighted QP/HQP，无机器人控制循环 | E07 解析、E06 方程调试、独立数值调试 |
| planned_kinematics_step | CartesianPlanner → weighted MCC/PlaCo → JointPlanner | E11/E12、独立 JSON/CSV/MCAP |

接口说明：[公共入口](../../../tools/app_execution/README.md)、
[Step](../../../apps/step/README.md)、[Target](../../../apps/target/README.md)、
[HQP](../../../apps/hierarchical_kinematics_step/README.md)、
[显式矩阵](../../../apps/optimization_problem/README.md)、
[规划 weighted](../../../apps/planned_kinematics_step/README.md)、
[HARP](../../../apps/hierarchical_kinematics_step/HARP.md)。

可配置 app 的 `--describe-capabilities` 不初始化模型或 solver；
`--request FILE --dump-resolved-options` 只解析显式输入/配置。请求不接受额外算法 CLI 覆盖。
正常 batch 与 request 入口复用 app-local 构造和执行函数，tracking 不参与算法分支。
新合同是独立 `execution_request.v1`，experiment manifest v1/v2 和旧 analysis 读取保留。

## 方法迁移的科学边界

每个现役 Exx/definition.json 的每个 unit 都保存旧声明 SHA256、旧方法/arm/config、
新 app/config/execution、具体差异、迁移状态与能力摘要。旧声明和生成器原字节存放于
Exx/declarations；旧运行和分析目录未覆盖。七个 study app 已从现役源码/CMake/新安装退出，
无旧 executable 转发别名；历史安装保留用于旧批次来源追溯。

- Step/Target 的实际 hard-pose 方程替代旧 study soft-pose/posture/workload 方程，旧权重与
  workload 扫描不能冒充保留了原控制因素。旧 arm 只用于追溯，重复映射须在新批次设计中去重。
- HQP 使用实际 app 的 RobotOptions、Red/Yellow 布局、耦合与接受策略；例如原生状态、
  selected pass 和实际提交分别记录，不能把旧 study 的 MAX_ITER 策略套到现有应用上。
- 两/三层、primary-only 和公开任务权重为 app-local 布局/选项。历史交换优先级、模拟延迟、
  故障注入或非原生投影无法对应现有执行链时明确退出，原始失败不会改写。
- 原生 backend/HQP/warm-start 缺项保持缺项。新 planned app 的固定 eiquadprog 与完整规划链
  明确列入差异，不以旧 backend 或 pipeline 名称宣称对照成立。
- 新核验只判定可观测的边界、候选/提交区分与声明分母；旧 E06 方程准入及 A01/A02 耗时
  优势不继承。独立数值测试验证目标与可行性，多解不要求关节向量唯一。

## 构建和测试

独立 HARP ON build/install 分别为 `/workspace/build/mcl-app-reuse-20260914`、
`/workspace/install/mcl-app-reuse-20260914`；依赖快照为
`/workspace/install/mcl-app-reuse-deps-20260914`。未替换旧批次安装依赖。
另有 HARP OFF、CPU affinity OFF 开发构建 `/workspace/build/mcl-app-reuse-noharp-20260914`。

```bash
cmake --build /workspace/build/mcl-app-reuse-20260914 -j4
cmake --install /workspace/build/mcl-app-reuse-20260914
ctest --test-dir /workspace/build/mcl-app-reuse-20260914 -j1 --output-on-failure
python3 -m unittest discover -s tools/app_execution/tests -v
python3 -m unittest discover -s tools/mcc_placo_study/tests -q
python3 tests/architecture_boundaries.py /workspace/labs/motion-control-lab
```

- 完整 CTest 105 项：首次 5 项旧测试预期失败，逐项修复测试后复验通过；加上新增的 replay accounting 单测，合计 106 项：102 项通过、
  4 项环境跳过；[按测试名汇总](/workspace/runs/mcl_app_reuse_20260914T054601Z/ctest-consolidated.json)。原日志保留在 [final-ctest.log](/workspace/runs/mcl_app_reuse_20260914T054601Z/final-ctest.log)，
  修复复验见 [contract-regressions-recheck.log](/workspace/runs/mcl_app_reuse_20260914T054601Z/contract-regressions-recheck.log)
  和 [ctest_hks_options_repaired.log](/workspace/runs/mcl_app_reuse_20260914T054601Z/ctest_hks_options_repaired.log)。
- 4 项 Step PTY 跳过原因是冻结默认 CPU 31 不在当前进程可用核 0–23 中。关闭绑核的开发构建
  Step/Target 键盘 8/8 通过：[日志](/workspace/runs/mcl_app_reuse_20260914T054601Z/noaffinity-keyboard-regression.log)。
  未为通过测试修改产品默认绑核。
- 公共 Python 合同/核验 10 项通过，包含损坏哈希、声明内及跨选择重复身份、非法跨声明绑定、旧 manifest、
  NaN、完整分母、初始越界/候选/实际提交分离、HTTPS 碰撞资源不可静默替换，以及超时终止外围适配器的子进程。
- 历史工具 unittest 58 项通过；E06/E07/E08/E11/E12 独立核验 pytest 81 项通过。
- Step/Target 的 solver/backend/观测配置正常入口与 request 数值一致；minimal 也保留原生
  拒绝候选。显式矩阵覆盖多解、退化、2/3 层、hard infeasible 和 weighted。
- Planned app 覆盖 MCC/PlaCo、CSV/MCAP、整数 release 边界、初始越界和
  IK 已接受但 JointPlanner 拒绝；正常/request 误差至多 1.67e-27。
- HQP 覆盖命名布局、同配置正常/request 一致、TCP/frame 非零 offset、初始越界、
  同核/双核真实线程、capacity=8 的保留失败、cpp-new/minimal 观测。
  直接开发测试使用显式本地碰撞模型，不把它冒充旧声明的相同模型。

所有被测 app 逐进程串行；运动 smoke 单例上限 30 秒。失败 app 退出码和 stderr 与独立核验
状态分别保存；核验通过不把原生失败改为成功。

## 全矩阵与外围 smoke

最终完整只读 dry-run：**7,303 个开发 repeat 单元全部有结论，0 个迁移实现错误**。其中 E06–E10 为 6,472 个；这些是迁移状态，不是旧批次的求解成功率。
7,303 按本轮归档的现有声明展开；由于原 campaign 索引缺失，不能用该数复原或替代历史批次总量。

[完整逐单元矩阵](/workspace/runs/mcl_app_reuse_20260914T054601Z/full-matrix-delivery.json) 与 [进度日志](/workspace/runs/mcl_app_reuse_20260914T054601Z/full-matrix-delivery.log)。

| 实验 | 单元 | ready | 原生不支持 | 研究方法退出 | 碰撞资源缺项 | CPU 未冻结 | 输入缺失 | 外围 smoke 完成/失败 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| E06 | 21 | 21 | 0 | 0 | 0 | 0 | 0 | 5/0 |
| E07 | 565 | 304 | 0 | 0 | 261 | 0 | 0 | 4/2 |
| E08 | 293 | 131 | 15 | 2 | 145 | 0 | 0 | 2/6 |
| E09 | 5400 | 936 | 3969 | 0 | 495 | 0 | 0 | 3/2 |
| E10 | 193 | 0 | 8 | 112 | 33 | 40 | 0 | 0/2 |
| E11 | 372 | 90 | 0 | 242 | 40 | 0 | 0 | 3/1 |
| E12 | 459 | 6 | 0 | 0 | 3 | 0 | 450 | 3/1 |

合计 ready 1,488；原生不支持 3,992；研究方法退出 356；碰撞资源缺项 977；CPU 未冻结 40；输入缺失 450。40 个 CPU 未冻结单元也保留了碰撞资源审计字段，不表示其模型资源已经就绪。

外围 smoke 共 34 个单元：20 个进程完成，14 个失败。11 个在初始化碰撞网格时失败；其旧 URDF 使用 HTTPS URI，当前原生几何加载器不支持。发现后已加入只读 preflight 检查。另 3 个保留原生拒绝：E08 两个初始越界（MCC/PlaCo），E09 一个 PlaCo hard-QP infeasible。未改算法、未降低约束或把失败改为通过。baseline 的 solver 级独立准入为 unavailable，原生轨迹和退出状态完整保存。

HQP 正常布局、真实调度、故意溢出和规划正/负例使用独立声明的本地碰撞模型完成，见 hks_execution_smoke05、hks_observation_acceptance、hks_planned_smoke04；它们不替代上述旧输入的失败。外围 run 全部位于各 Exx/runs：

- [E06 smoke](/workspace/labs/motion-control-lab/experiments/E06_solver_semantic_parity/runs/20260914T063208.670443Z-2cc4a8ae8e)
- [E07 smoke](/workspace/labs/motion-control-lab/experiments/E07_hqp_priority_and_redundancy/runs/20260914T063210.589866Z-c7d4614de4)
- [E08 smoke](/workspace/labs/motion-control-lab/experiments/E08_constraints_scaling_and_degeneracy/runs/20260914T063214.436517Z-b47b67fa7a)
- [E09 smoke](/workspace/labs/motion-control-lab/experiments/E09_solver_cost_and_scalability/runs/20260914T063217.745440Z-dc12f287c4)
- [E10 smoke](/workspace/labs/motion-control-lab/experiments/E10_multirate_scheduling_and_coupling/runs/20260914T063220.831741Z-3766cd1ab8)
- [E11 smoke](/workspace/labs/motion-control-lab/experiments/E11_planning_ik_otg_error_propagation/runs/20260914T063223.492558Z-b4f28fbffe)
- [E12 smoke](/workspace/labs/motion-control-lab/experiments/E12_recorded_motion_holdout/runs/20260914T063226.805171Z-c1c3a67d75)

可复现命令：

```bash
python3 tools/app_execution/run.py --install-prefix /workspace/install/mcl-app-reuse-20260914 --dry-run
python3 /workspace/runs/mcl_app_reuse_20260914T054601Z/run_outer_smoke.py
```

第二条仅重做已明确选定的有界开发 smoke；精确选择、每进程 argv/timeout 及原始 log 见 [命令清单](/workspace/runs/mcl_app_reuse_20260914T054601Z/outer-smoke-commands.json)。所有输出使用新目录。

## 原生 HARP

在线来源只保留 manual/recorded/harp。删除旧 PiM Python 模型服务、socket 通信与在线
launcher；旧参数明确报迁移错误，历史记录仍可读取。MCL 不再维护第二套模型实现。
MCC Predictor 在普通线程初始化、预热和推理；Red 不调用 Torch、不等待结果。
完整 30×9 FP32、100 Hz、启动填充、generation/sequence、暂停/重启和 100 ms 年龄合同保留。

CPU/CUDA 64 个完整窗口对原参考最大绝对误差分别为 6.259e-7、2.980e-7；CPU/CUDA
相互最大差 6.259e-7，低于既有 1e-4 门槛：[原始命令与结果](/workspace/runs/mcl_app_reuse_20260914T054601Z/harp-parity)。
生命周期测试覆盖完整窗口、跳过请求、暂停、过期、重启、错误传播，strace 证明未启动 Python
推理进程：[进程证据](/workspace/runs/mcl_app_reuse_20260914T054601Z/harp_no_python_process.json)。
HARP OFF 的 HQP 二进制不链接 Torch/CUDA/posture_reference。

HQP 回放新增证据的最终正/负例见 [计数核验](/workspace/runs/mcl_app_reuse_20260914T054601Z/hks_replay_accounting_smoke02/result.json)。独立 Python 复核分别完成 154、54 项边界/候选/来源计数检查，负例仍保留原生 exit 1。

## 历史保全与来源缺项

[保全结果](/workspace/runs/mcl_app_reuse_20260914T054601Z/preservation-final.json)：
112 个原 manifest、117,865 项历史 run artifact 引用全部符合原 SHA256；baseline 11 个生产
源码/启动文件与迁移前一致。唯一 baseline 改动是共享 TUI 当前合同的测试预期。
七个退役 app 的 64 个源码文件逐个核对快照 SHA256 后移除，恢复依据为
[source_before.tar.gz](/workspace/runs/mcl_app_reuse_20260914T054601Z/source_before.tar.gz)
及配套 source_before.sha256.json/retired_app_sources.json。

原 campaign execution_index、plan 和部分补证归档在本轮开始前已缺失，现有分析声明中共
18 个缺失引用，登记其预期哈希，未拼造替代索引：
[缺项表](/workspace/runs/mcl_app_reuse_20260914T054601Z/historical_missing_references.json)。
这个缺项与仍存在且哈希通过的 Exx/Axx runs 是两件事。

## 后续启动条件

1. 先选择有科学意义的新 app 配置、去除旧扫描因素坍缩造成的重复映射，再建立新方法矩阵。
2. 恢复缺失原始来源；为 HQP 显式准备本地碰撞资源。任何 URDF/资源定位变换必须生成新的
   输入/模型身份及哈希并验证几何对应，不能关闭碰撞约束或静默换模型。
3. 冻结实际 CPU/线程/调度、输入、模型/网格、配置、capabilities、源码、安装动态库。
   真实异步的 40 个未指定 CPU 单元不能自动分配核后冒充冻结对照。
4. 重新核对实际任务矩阵、单位/坐标系、正则化、约束、原生接受和提交合同，完成新准入。
5. 使用独立安装前缀重新 dry-run/smoke 后，另行授权并串行启动新批次。普通 replay 的
   settling/暂停使控制 release 总数不预定；其 planned/not-run 标为 unavailable，不从轨迹行数
   推算 deadline 分母。固定计划的 batch 单独保存 planned/skipped/not-run。

本轮不宣称正式 RT 就绪、正式实验完成或科学证据充分；不运行新 A01/A02 分析、A03、
跨运行统计或论文。没有重启、设备操作、提交或推送。
