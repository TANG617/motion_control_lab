# E10：多频率调度与耦合归因

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

当前执行入口已迁移到可复用 app：见 [公开执行与映射](../../../tools/app_execution/README.md)。
`definition.json` 绑定新 app/方法；原声明及输入生成器保存在 `declarations/`。
新准备入口：`python3 prepare.py --output /absolute/new-definition.json`；
执行与 dry-run 使用 `tools/app_execution/run.py`，不再启动按实验编号命名的程序。
旧 run、validation、分析及失败不变；下面保留的研究设计/历史验收描述适用于原方法，
不能作为新 app 方程或耗时结论。完整旧新配置差异见每单元的 `migration.differences`。


状态：`implemented-development-accepted`；正式运行就绪与正式实验完成均未声明。公共定义：[PROTOCOL](../../mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](../../../experiments/E10_multirate_scheduling_and_coupling/inputs/README.md)。消费：[A02](../../../analyses/A02_runtime_and_scheduling/README.md)。对应 G07/G09/G10/G13；C4；F01/F07/F08、T03。

## 问题与假设

分频和异步耦合能否在保持声明质量的条件下降低主环的 release-to-finish 成本和 miss，且不会因旧参考损害轨迹？减少次级计算、增加核心和解耦线程分别是独立解释。

主要比较在 MCC 内完成；placo-controlled 能保持同任务/约束语义接入同样 proposal/runtime 时加入对应控制组。不能为让 placo 参加而在其外部重写一套 HQP，再仍称为原生 placo。缺失能力进入 T01，不归因于调度性能差。

## 必需 scheduler arms

| scheduler_id | 主环 / 次级计算 | 线程与资源 | 对照用途 |
|---|---|---|---|
| synchronous-full | 1000/1000 Hz，次级先算，主环消费本 tick 结果 | 单线程、一个指定核心 | 全频成本基准 |
| synchronous-divided | 1000/100 Hz，每十个 release 更新次级 | 单线程、同一核心 | 仅减少次级计算 |
| asynchronous-shared-core | 1000/100 Hz，独立线程/latest-value | 两 worker 固定在同一核心 | 隔离异步结构与增加核心 |
| asynchronous-two-core | 1000/100 Hz，独立线程/latest-value | 两 worker 各一个同类型核心 | 额外核心的资源收益 |

单线程分频规定在 release index 可整除 10 时先执行次级；确定性虚拟 schedule 也使用这一顺序。真实异步不强制同步新参考，记录实际年龄。初始化和 warm-up 独立窗口，不能把双方不同的初始 proposal 隐去。

次级 rate 20/50/100/200 Hz 只在分频 arms 扫描；反馈采样默认 1 kHz，另以 100/250 Hz 作敏感性分析。输入目标 100 Hz、主求解和输出 1 kHz。输入/反馈/求解/输出四个时钟分别记录。

## 耦合与干扰矩阵

- coupling-enabled 与 proposal-disabled 成对运行，用于量化收益和负担。关闭 proposal 时也保留声明的计算负载，不混成“少做任务”的伪消融。
- 延迟固定为 0/5/20/50 ms；分别对 proposal 的发布和 measured state 的交付施加，两个实验不同时改变。注入使用 run-relative schedule，不 sleep 阻塞主环。
- 在 5 s 时暂停次级计算 100 ms，恢复后继续；包含 proposal 不可用、发布失败、旧版本和重复消费。保留原 app policy，不自行增加 age expiry 或 fallback。不同政策作为将来的独立候选，不在测量中隐式切换。
- 可重复计算负载分别为无额外负载、同核 CPU 工作负载、异核 CPU 工作负载；记录实际 CPU time。不要把固定 sleep 当成等价 CPU 计算负载。

开发 smoke 每个代表条件 2 s，其中暂停注入移至 0.5 s 并在 smoke 声明中记录；pilot trace 10 s 与 formal 使用上述 5 s 注入时刻。formal 每 required 条件 30 s、10 次独立进程重复，采用平衡 arm 顺序。全量矩阵是上述 family 的配对扫描，不对所有因素做无解释的全组合。严格模式若提前失败，按 DATA_ANALYSIS 保留后缀 not-run 和 time-to-failure；monitor 压力图另标诊断。

## 指标与窗口

Primary：release_to_finish P99、deadline_miss_rate。Guardrail：hard_violation_excess、主跟踪误差、完整任务质量与完整窗口完成率。Secondary：consecutive_misses、次级收益与总 cpu_time。Diagnostic：proposal_age/state_age、source/attempt/revision、worker budget、queue drops、skipped releases。

位置或朝向的 secondary 条件与 E07 一致；若次级用 collision，最终控制状态精确距离独立测量，低频 collision proposal 不构成主环硬约束。过时 Jacobian/投影不能假称由当前反馈重算。

## 实现与产物

app-owned runtime 消费标准时间/状态记录，scheduler 机械层不 include MCC。保存每次计划 release、实际执行区间、所消费 proposal、proposal 所用 state time、提交结果和负载事件。F07 展示完整预声明暂停窗口，不挑特定漂亮 tick。

输出 worker_timeline.csv、coupling_trace.csv、injection_schedule.json、resource_summary.csv、tracking/quality 表、完整平台 manifest。正式部署预检仅检查，不能改内核、宿主调度或重启机器。

## 验收

手工虚拟 schedule 验证每个 release、分频次数、age、重复消费、skipped denominator；真实线程 smoke 核验停止/join 和非阻塞消费。虚拟 replay 验证数值语义，不能当作实时证据。满计时缓冲要产生可见缺样状态。

只有同资源/同语义对照才能归因于调度结构；多核心结果同时报告资源成本。论文不得把有限时长零 miss 写成 hard-real-time guarantee 或稳定性证明。

## 实现入口与开发验收合同（2026-09-10）

`apps/study_e10/` 是独立 app，primary 直接使用 MCC 两层 HKS，secondary 使用普通
MCC posture QP；每个 worker 拥有独立 solver。同步/full、同步/divided、异步同核、
异步双核全部有真实线程运行路径；独立虚拟模式也实际调用 MCC，其人为工作时长明确
标为 virtual semantics，不能作为真实计时证据。

`prepare.py` 生成 hold+secondary、低幅可达正弦、有界困难窗口三类 canonical 输入
和 **192 科学 cells，184 可执行、8 native capability 不可用** 的成对扫描矩阵。
另有一个仅 development 可选的 `fixture-observation-overflow` cell；开发矩阵总计 **193**，
正式矩阵仍为 **192 cells / 1920 repeats**。该 fixture 使用真实 app、20 ms virtual
窗口和 8 条 observation buffer，预期 native exit 2 / 缺样核验失败；不能记成科研结果。
分频 rate 20/50/100/200 Hz、反馈 100/250/1000 Hz、两类分别扫描的 0/5/20/50 ms
延迟、pause/publication-failure/old-version 和三类 CPU load 全部显式声明。
PlaCo 没有相同原生两层 topology，保留 unavailable cell，不在外部重写 HQP。

运行记录分别保留 target、feedback-capture、primary、output 和 secondary 的所有计划
release。`planned_releases.csv` 在 worker/native 调用前落盘，即使异常退出仍保留完整
分母；完成后实际 `worker_timeline.jsonl/.csv` 与 planned 表对应。原始 attempt 在
开始及完成时 flush；proposal/state capture、revision、重复消费、旧版本、注入、
候选与实际输出均可追踪。固定容量 observation buffer 溢出具有显式计数并使验收非零。
`coupling_trace.csv` 为真实逐 attempt 字段投影，`resource_summary.json/.csv`
记录线程 CPU 与 wall duration。full telemetry 属于开发观测条件，不宣称严格截止期。

反馈模型是明确的 ideal post-command kinematic state；没有动力学或硬件。
原生 rejected 候选不被消费；执行位置 HOLD 与 solver rejection 分字段保存。
proposal-disabled 保持 secondary 计算，只关闭 primary posture 使用；failed latest
proposal 关闭 coupling，不增加 age expiry。延迟通过 delivery queue 实现，不用 sleep
阻塞主环模拟延迟。CPU 干扰运行真实算术运算，sleep 仅用于 release 等待。

真实线程根据显式 `cpu_ids` pin；development 空列表从实际允许 cpuset 选择，不把这些
编号当作正式冻结的同类型 physical cores。正式运行须经公共 RT/核心类型预检，明确
冻结 CPU IDs。代码不改 scheduler policy、内核或宿主机配置。

```bash
python3 experiments/E10_multirate_scheduling_and_coupling/prepare.py --model /workspace/models/r1.cos.urdf
python3 experiments/E10_multirate_scheduling_and_coupling/test_schedule.py
python3 tools/mcc_placo_study/study.py --experiment E10 --dry-run --inventory-output runs/e10-matrix.json
python3 tools/mcc_placo_study/study.py --experiment E10 --case hold-secondary-baseline --arm synchronous-divided-coupled --repeat 0 --phase development --smoke --timeout 60 --output-root runs/e10-smoke
```

每个 smoke 声明 2 s，pause/failure 注入移到 0.5 s；没有自动晋升正式结果。
`verify.py` 用独立公式核对每个计划分母、skipped/miss、版本年龄及 overflow。
`test_schedule.py` 的手工 fixture 检查 secondary-first/division、重复消费、暂停恢复、
初始不可用、exact deadline、skipped denominator 和缺样。实际进程正常退出验证 stop/join；
超时/崩溃由公共 orchestrator 保留。正式 pilot 为 10 s；confirmatory 为每 cell 30 s
和 10 个新进程重复，注入时刻仍为 5 s，必须串行启动。正式实验、A02、科研统计、
图表与论文均 deferred。

## 本次已完成的开发验证记录

2026-09-10，匹配安装产物 `/workspace/install/algorithm/bin/mcl_study_e10`
在如下证据目录完成 **24/24 单元并通过独立核验**：

`runs/mcc_placo_study/development_acceptance/smoke/20260910T110029.012135Z-b9689101ce/`

选择为 baseline/pause/virtual-schedule 三类 × 四 scheduler × coupling 开/关。
其中 16 个真实时间单元各声明 2 s；8 个虚拟单元各有 2 s logical window 并调用真实 MCC。
每个 `command.json` 保存精确安装路径、argv、cwd、20 s timeout；运行 CLI 选择如下：

```bash
python3 tools/mcc_placo_study/study.py --experiment E10 \
  --case hold-secondary-baseline --case hold-secondary-pause \
  --case hold-secondary-virtual-schedule \
  --repeat 0 --phase development --smoke --timeout 20 \
  --output-root runs/mcc_placo_study/development_acceptance/smoke
```

已核验实际异步 worker 为两个线程，secondary thread CPU clock 有正值并正常 stop/join；
同核/双核分别记录本次 development 实际 CPU 0/0 与 0/1。这些编号仅为这次观测值，
没有冒称已确认的 RT 同类型 physical cores。虚拟模式记录实际一个线程、effective CPU null。

所有单元保留 target/feedback/primary/secondary/output 的完整计划分母及状态连接。
100 Hz secondary pause 保留 10 个计划 skipped releases，1 kHz full 保留 100 个；
暂停期间重复消费既有 proposal、版本/状态年龄、恢复后新发布都有原始字段。
每个真实时间单元即使 primary skipped/HOLD，仍保留 2000 个 primary/output 计划 releases；
`output` 的候选来源、committed sequence、HOLD q 保持、反馈捕获状态连接由独立 verifier 检查。
实际漏截止期与 skipped 没有被隐藏，不由本次检查推导性能、稳定性或硬实时结论。

纯 schedule 测试 **9/9** 通过，涵盖虚拟手工时序、重置、失败 proposal、缺失身份、
observability overflow 和成对矩阵合同。首轮
`20260910T105505.491986Z-b9689101ce` 的 **24 个 constructor crash** 原样保留；
同 E09，工程原因是误读 `input.model.path`，改为合同 `model.locator` 后重跑。
该修复不改变数值策略或失败接受政策。

额外真实 app overflow 验证使用以下 development-only fixture（预期非零并保留失败状态）：

```bash
python3 tools/mcc_placo_study/study.py --experiment E10 \
  --case fixture-observation-overflow --arm real-mcc-virtual-buffer8 \
  --repeat 0 --phase development --smoke --timeout 20 \
  --output-root runs/mcc_placo_study/development_acceptance/smoke
```

代码与上述开发子集已验收；正式仍需要 RT/核心资源及配置/输入/阈值冻结。
泛用 `resource_requirements` 阻止空 CPU ID 的正式运行。正式 192-cell 矩阵未执行；
A02、跨运行科研统计、图表、论文全部 deferred。

实际 overflow fixture 证据：
`runs/mcc_placo_study/development_acceptance/smoke/20260910T111212.489628Z-517b25c18c/`。
同一安装 app 完成 **20 primary + 2 secondary 真实 MCC 调用，native rejection 0**；
8 条 buffer 保留 8 events，`timing_buffer_overflow=58`。进程 **exit 2**、独立 verifier
**exit 1** 为本 fixture 的预期可见失败，状态保留 `missing-samples` 与
`observation-overflow-tail-unavailable`。完整预写分母为 primary/feedback/output 各 20，
secondary/target 各 2；已保留 timeline 分别为 2/2/1、1/1，缺样没有当成正常零 miss 证据。
raw attempts 与计划表仍可读取。该单元仅用于观测设施开发验收，不进入科学矩阵/正式运行。
此前空 `smoke_config` 的 launch 拒绝也由 parent receipt `e10overflow-20260910T111004`
保留；显式填写相同 `duration_s=0.02` 后执行，未改变算法或输入。

补充 same-core/other-core load worker 两单元已实际 completed/validated，load thread CPU 均大于零：`20260910T112021.726829Z-d6770a96d3`。完整命令与追加证据见 [最终开发验收](../study/DEVELOPMENT_ACCEPTANCE.md)。
