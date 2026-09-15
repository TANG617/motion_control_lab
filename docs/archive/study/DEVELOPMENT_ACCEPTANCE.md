# E06–E12 开发验收记录

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

> 历史 study 实现合同/验收记录，保留原命令和结论范围。现役程序与声明已迁移，使用 [公开 app 执行接口](../../../tools/app_execution/README.md)，不要直接执行下述历史构建/启动命令。

日期：2026-09-10。范围：真实实验 app、公共合同、矩阵与输入、原始证据、独立核验及有界开发 smoke。此文件是工程验收记录，不是科研分析报告。正式 campaign、A01–A03、跨 run 科研统计、科学图表、分析报告、paper 工具与论文撰写均未执行，标记 **deferred**。C1–C6/D1–D3 未因代码完成而被验证。

## 实现与边界

主 agent 在公共接口 Wave0 验证后分配三个 owner：A 独占 E06–E08 app/experiment，B 独占 E09–E10，C 独占 E11–E12。主 agent 独占公共工具、schema、顶层构建、架构登记与最终集成。交叉审阅先报问题再由 owner 修改。七个 app 均有独立 main/options/solver/loop/support target；规划保留在 E11/E12 app 内。没有共享 solver facade、跨 app include/link 或 production 算法改动。

公共入口 `tools/mcc_placo_study/study.py` 支持选择、development/pilot/confirmatory、完整 dry-run、只读预检、独立进程、串行执行锁、超时/信号/失败保留。公共 schema 为 canonical_input.v1、input_descriptor.v1、study_request.v1、experiment.v2、run_manifest.v2、metric_row.v2；旧 E01–E05/v1 读取兼容。独立 NumPy/XML URDF FK 和 app-owned oracle 在 native timing 外执行。指标只计算逐单元结果，保留未来配对身份与哈希。

每单元保留 request、resolved declaration、实际 command、environment、raw、stdout/stderr、原生错误、独立检查、状态及全部递归 artifact 哈希。进程完成、原生 full/partial/rejected、独立检查通过分别解释。旧失败目录不覆盖、不筛除；修复后的复验建立新 run。源树快照包括 dirty/untracked 内容，运行物记录实际安装 ELF 和 ldd 库。

## 全矩阵 dry-run

无 selection，全部 required 声明。development 使用各声明重复数（E09 为3，其余1）；confirmatory 使用冻结研究计划的重复数（E12 为3，其余10），development-only fixture 排除在正式矩阵之外。

| 实验 | 声明 cell | development 单元 | development ready | confirmatory 单元 |
|---|---:|---:|---:|---:|
| E06 | 21 | 21 | 20 | 210 |
| E07 | 565 | 565 | 565 | 5640 |
| E08 | 293 | 293 | 258 | 2920 |
| E09 | 1800 | 5400 | 1431 | 18000 |
| E10 | 193 | 193 | 185 | 1920 |
| E11 | 372 | 372 | 372 | 3700 |
| E12 | 459 | 459 | 5 | 1350 |
| 合计 | 3703 | 7303 | 2836 | 33740 |

逐单元清单见 `runs/mcc_placo_study/development_acceptance/matrix-development.json` 和 `matrix-confirmatory.json`。正式 ready=0；只读 `formal-preflight.json` 返回非零。ready 仅代表当前 development 可启动，不能解释为正式就绪或所有方法成功。

E06 有1个 PlaCo shared-scaled 能力缺项；E08 有35个原生策略/缩放/精确碰撞几何缺项；E09 的3969个重复单元保留原生 mode/backend/warm-start/layer 不支持状态；E10 有8个明确 native-capability 缺项。E12 的450个真实 cell 尚未转换/冻结，另4个 synthetic-confirmatory 负向准入 fixture 不可用，5个 fixture 可执行。

## 构建与运行物

使用当前标准 colcon 构建目录 `/workspace/build/algorithm/motion_control_lab` 和安装前缀 `/workspace/install/algorithm`，RelWithDebInfo，MCL_BUILD_STUDY_EXPERIMENTS=ON。初次 colcon 命令的实际 CMake 子命令记录在 `/workspace/log/algorithm/build_2026-09-10_10-19-42/motion_control_lab/command.log`。后续按修改的 app target 增量构建并安装。

```bash
cmake --build /workspace/build/algorithm/motion_control_lab --target mcl_study_e06 mcl_study_e07 mcl_study_e08 mcl_study_e09 mcl_study_e10 mcl_study_e11 mcl_study_e12 -j 3
cmake --install /workspace/build/algorithm/motion_control_lab
ctest --test-dir /workspace/build/algorithm/motion_control_lab --output-on-failure -R '^(study\.|architecture\.boundaries|apps\.mcl_study)'
python3 tools/mcc_placo_study/study.py --dry-run --inventory-output runs/mcc_placo_study/development_acceptance/matrix-development.json
python3 tools/mcc_placo_study/study.py --phase confirmatory --dry-run --inventory-output runs/mcc_placo_study/development_acceptance/matrix-confirmatory.json
python3 tools/mcc_placo_study/study.py --phase confirmatory --preflight
```

七个 build/install ELF BuildID 一致，见 `installed_identity-final.json`；安装修改 RPATH 导致文件 SHA 不同，两个哈希均记录。默认 launcher 选择安装产物，未选旧 standalone。Core source clean、安装头文件与当前源码匹配；`baseline-freeze-verification.json` 证实所有冻结 baseline 源文件哈希与任务开始一致。没有内核/宿主调度改动、重启、硬件访问、commit/push/publish 或 results promotion。

## 正式阶段的外部前置条件

当前内核 `6.8.0-139-generic` 为 PREEMPT_DYNAMIC，RT flag 不可用。本任务不会重启/换内核。正式阶段需要另行确认当前 boot 的 RT、同类型物理核、cpuset/频率/调度政策、显式 CPU ID、benchmark 串行及环境冻结。

E12 仅对当前可见50个 MCAP 做 hash-only inventory，未解码真实 payload；外部原始数据不复制、不修改。来源/会话信息不足且存在历史 E05 曝光，保守归为 development；需补充来源、session、exposure、重复/重叠和新的未曝光会话。当前数据不能建立真正 holdout。转换设施用显式标记的真实 ROS2 MCAP fixture 验证原始时间配对、retime、完整初始化、坏流/帧/时间拒绝、raw 哈希不变；不将历史 solver 输出视为正确轨迹。

后续单独启动：补齐 metadata/session/exposure → 按 recipe 转换允许的 development/tuning 数据 → 等预算候选计划/执行/选择 → 冻结候选、输入/模型/资源/阈值来源/完整 required 矩阵 → 当前 boot RT attestation → 只读预检全部通过 → 单独串行启动正式 campaign。`candidates.py plan/execute/select/freeze` 和 `freeze_campaign.py` 提供设施，未在本任务启动真实调参 campaign。

```bash
# 以下为未来步骤模板，本次没有执行。
python3 tools/mcc_placo_study/freeze_campaign.py --definition <definition.json> --host-attestation <host.json> --inputs <input-inventory.json> --models <model-inventory.json> --candidates <candidate-freeze.json> --preregistration <threshold-policy.json> --resources <resource-policy.json> --exposure <exposure.json> --split <split.json> --output <new-freeze-dir>
python3 tools/mcc_placo_study/study.py --definition <new-freeze-dir>/definition.frozen.json --phase confirmatory --preflight
python3 tools/mcc_placo_study/study.py --definition <new-freeze-dir>/definition.frozen.json --phase confirmatory --output-root <new-campaign-root>
```

Formal-executed=false 对全部七项实验成立。代码验收、正式就绪、正式执行、分析完成和论文证据完整分别登记；缺失与不利结果不会被伪装为成功。

## 已知失败与修复分类

首轮 smoke 保留了真实触发的工程错误：A 的 analytic HQP 在注册层之前注册 shared bounds，Core 直接拒绝；B 使用不存在的 model.path 导致空 URDF。修复仅改 app 的 API 注册顺序和 model.locator 字段。没有改目标、约束、容差、迭代预算或 Core 接受政策。

测量修复单独记录：NotRun 层零初始化 residual 不再作为 optimum；begin-only 日志不会通过结果完整性核验；单帧/尚未到 withdrawal 的 recovery 保留 not-observed；压缩 fixture 用声明的 challenge 起点计算方向；恒定非二进制浮点信号在每个相关窗口先检查精确零方差，不再产生虚假 lag。旧 checker 输出保留，新核验建立独立目录或新 smoke run。

原生负向证据不修复：E07 tertiary 硬可行性拒绝并返回已完成的较高层；E08 Secondary infeasible、partial、初始越界、PlaCo QPError，以及压缩恢复窗口短于100ms dwell导致的 censored。E06 admission 保留 native 接受/模式语义不一致，不能把任务矩阵对齐自动升级成公平因果比较。

baseline 相关检查31/32总闸门中唯一失败为 apps.mcl_baseline_tui_projection（研究24项当时全部通过）。只读定位：测试默认 requirements_page_enabled=false，却期待仅在可选 Requirements 页显示的 frame_position scale，并同时要求四页。相关 test/renderer/contracts 均未被本任务修改；原生 baseline solver、配置、确定性 replay 和 PTY 检查通过。为冻结 production-static 保留此独立展示测试失败，未修改 baseline。

## 最终逐实验开发验收

各项均 implemented=true、tested=true、smoke-executed=true，formal-ready=false、formal-executed=false。下表计数为声明的开发 smoke 单元状态，不是成功率/方法排名。预期负向单元仍返回非零，完整 raw/error/validator 证据保留。

| 实验 | 专项测试 | 最新主要 smoke run | 开发结果 |
|---|---:|---|---|
| E06 | 7 passed | 20260910T104933.419463Z-93358841be | 20 completed/validated，1 native capability unavailable |
| E07 | 7 passed | 20260910T105533.501826Z-861f19d76c | 53 completed/validated，原生 partial hierarchy 保留 |
| E08 | 8 passed | 20260910T105602.548978Z-32af7bd026 | 19 completed/validated；2 unavailable；3 初态越界检查失败；1 原生 PlaCo QPError 崩溃 |
| E09 | 5 passed + allocation CTest | 20260910T110024.558252Z-f60fca063b | 14 completed/validated，2 native capability unavailable |
| E10 | 9 passed | 20260910T110029.012135Z-b9689101ce | 24 completed/validated（16 真实调度，8 虚拟时序） |
| E11 | 4 passed | 20260910T110546.488837Z-f7861e12e2 | 25 completed，1 预期 OTG 拒绝；26 独立核验通过 |
| E12 | 17 passed | 20260910T110605.051726Z-af76d83891 | 5 completed/validated，4 synthetic-confirmatory 负向 fixture unavailable |

E10 额外真实 overflow fixture 为 `20260910T111212.489628Z-517b25c18c`：20 primary + 2 secondary native MCC calls，buffer8 保留8/丢弃58事件，native exit2 / validator exit1。原始 planned denominator 保留，tail 明确 unavailable。首次空 smoke_config 启动在 native 前失败，该不完整目录另有 orchestration_failure.json，未伪造成 native 成功。

E09 每单元11次真实调用，覆盖 TargetSolve、两层/三层及 minimal/full/cpp-new；MCC 观测三种模式的 q/status/quality 完全一致。分配只测 app-local C++ new/new[]/aligned-new，不声称捕获 malloc/Eigen 的全部分配。PlaCo TargetSolve 的 feasible-suboptimal/no-progress 原样记录。E10 保留完整 release 分母、真实线程 CPU、stop/join、延迟/暂停/重复消费及 HOLD 状态；这些是开发测量链验证，不提供 RT 或性能优势结论。

E11/E12 的阶段原子记录和独立核验覆盖 goal/reference/rawIK/projection/execution、20ms 反馈延迟、P/V/A/J 来源和明确 HOLD。E11 零运动的12个单元 lag 已正确为 not-applicable/zero variance。E12 fixture 的 source coverage=0.6 与 settling not-reached 保留，不能解释为真实录制完成或 holdout 通过。

## 检查证据和复现命令

工程证据根目录为 [development_acceptance](../../../runs/mcc_placo_study/development_acceptance)，每个上表 run 位于其 `smoke/` 子目录。总入口 [acceptance_inventory.json](../../../runs/mcc_placo_study/development_acceptance/acceptance_inventory.json) 包含全部旧/新17个 manifest run 和1个启动前失败目录。原始 native 状态、各版本 checker 及前后修复结果均可追溯。外层完整文件哈希清单为 `artifact_inventory.json`。

- `ctest-final.log`：29/29，通过研究24项及既有 E01–E04/配对合同5项；E05 旧合同另见 `legacy-e05-contract.log`。
- `public-tests-final.log`：14/14，覆盖独立 FK/已知旋转、种子及可变对象隔离、哈希篡改、旧版本、子进程失败/超时/中断、缺失/空结果保留、指标窗口/零分母及 metric role-only 修正。
- `ctest-presmoke.log`：原32项广义检查31通过，保留上述冻结 baseline TUI 展示失败。
- `audit-evidence.log`：全部现存 manifest 哈希/required-actual 身份及274956行 metric_row schema 校验；这是格式/完整性检查，不把旧版本的错误测量判断升级为正确。
- `installed_identity-final.json`、`core-runtime-audit.json`、`baseline-freeze-verification.json`：7个 app BuildID匹配，Core104个对象不早于源文件、2个 build/install 库 BuildID匹配，冻结 baseline 全部源哈希未变。mtime 仅是新鲜度证据，不等价于可复现构建证明。
- E12 新准备层17测试日志位于 `experiments/E12_recorded_motion_holdout/runs/development_preparation_validation_20260910/tests.log`。

从 Lab 根目录实际使用以下串行 smoke 入口。脚本只运行明确给出的选择，每次保存完整 argv、cwd、exit/stdout/stderr receipt，每个 native process timeout20s；独立核验不计入 native timing。需要逐单元精确命令时查看对应 `command.json`。

```bash
python3 runs/mcc_placo_study/development_acceptance/run_smoke.py e06 e07 e08
python3 runs/mcc_placo_study/development_acceptance/run_smoke.py e09 e10 e11 e12 e07 e08
python3 runs/mcc_placo_study/development_acceptance/run_smoke.py e09 e10 e11 e12
python3 runs/mcc_placo_study/development_acceptance/run_smoke.py e10overflow
python3 tools/mcc_placo_study/check_inventory.py <run-dir>/inventory.json
python3 runs/mcc_placo_study/development_acceptance/audit_evidence.py
```

后续消费者使用 role 修正目录 `revalidation/declared_metric_roles/{E06,E11,E12}/revalidation.json` 中的 corrected metrics：只将声明要求的 position/orientation 角色改为 primary，共2916行；values、identity 和 raw source locator 与原文件逐行相等。对应命令记录在同目录 `commands.json`。E07/E08 压缩方向重核验位于 `revalidation/a_directional_window_20260910T105954.035461Z/`，原 raw 哈希不变。E06 的 `e06-pair-admission.json` 明确保留不满足 native acceptance/模式语义条件的配对，未生成效果估计。

公共输入工厂修复返回列表共享问题后，旧工厂源码归档及定位见 `input-generator-relocation.json`，既有输入未覆写。E10 添加 overflow fixture 时另建输入目录；`e10-input-value-equivalence.json` 证实原192科学 cell 的输入值、config 和 smoke_config 不变，仅 generator/study_generator provenance 更新。更早只排除一个 provenance 字段的检查结果也保留。

最后补充此前未覆盖的 native 执行分支，全部 completed/validated：E09 Eiquadprog warm0 的 weighted/HQP3 两单元（`20260910T112019.371547Z-a59f0f09c4`）；E11 historical production-static adapter 一单元（`20260910T112020.700612Z-07be1f3d89`）；E10 same-core/other-core load worker 两单元（`20260910T112021.726829Z-d6770a96d3`）。两种负载都有正的 load_thread_cpu_s，证明负载线程实际运行；不比较其性能。实际命令：

```bash
python3 runs/mcc_placo_study/development_acceptance/run_smoke.py e09backend e11baseline e10load
```

包括补充分支后，E09 smoke 可执行16单元均通过（另2 unavailable）；E10常规调度/负载26单元均通过（另1预期overflow失败）；E11普通/基线26 completed，1预期OTG失败，27独立核验通过。主要表仍保留各固定选择的原始计数，避免混淆不同 smoke selection。
