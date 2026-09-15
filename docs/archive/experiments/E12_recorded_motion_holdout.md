# E12：真实录制动作与独立保留验证

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

当前执行入口已迁移到可复用 app：见 [公开执行与映射](../../../tools/app_execution/README.md)。
`definition.json` 绑定新 app/方法；原声明及输入生成器保存在 `declarations/`。
新准备入口：`python3 prepare.py --output /absolute/new-definition.json`；
执行与 dry-run 使用 `tools/app_execution/run.py`，不再启动按实验编号命名的程序。
旧 run、validation、分析及失败不变；下面保留的研究设计/历史验收描述适用于原方法，
不能作为新 app 方程或耗时结论。完整旧新配置差异见每单元的 `migration.differences`。


状态：`implemented`；开发测试与有界 smoke 已执行，正式实验未执行。公共定义：[PROTOCOL](../../mcc_placo_study/PROTOCOL.md)、[DATA_ANALYSIS](../../mcc_placo_study/DATA_ANALYSIS.md)。输入：[inputs](../../../experiments/E12_recorded_motion_holdout/inputs/README.md)。消费：[A03](../../../analyses/A03_end_to_end_and_recorded_motion/README.md)。对应 G02/G06/G08/G09/G10/G12/G14；C6；F10、T02/T04。

## 问题与假设

冻结的候选方法在真实动作中的效果是否与机制实验一致？在未用于开发和调参的独立采集会话中，改善能覆盖多少动作，失败和代价是什么？

假设以预声明执行精度/valid completion 指标和 guardrails 判断，不能仅凭回放正常退出宣称成立。没有合格未暴露 holdout 时，只进行 recorded case study，C6 的泛化部分保持 evidence-insufficient。

## 数据与方法

外部默认 locator 为 /mnt/mcap_dataset；2026-09-10 盘点为 50 个直属文件。真实身份、内容 hashes、session 关系、重复/重叠和 exposure 尚未冻结。过去已用于问题定位或参数选择的数据不因重划 split 变成未见数据。

| evaluation stratum | methods | 解释 |
|---|---|---|
| historical migration | production-static、冻结后的完整 MCC 链路 | 记录原始任务/mask/period 的全部差异，衡量整体迁移 |
| controlled comparison | placo-controlled、mcc-weighted、选定 mcc-hqp | 相同输入/约束/下游/反馈模型；引用 E06 parity |
| confirmatory holdout | 上述已在开发/调参阶段冻结的方法 | 只运行一次冻结协议下的既定重复，不再挑选配置 |

model、TCP、初始 state、timeline transform、source availability、plant model、settling 条件在 stratum 内固定。输入取 recording 的目标流；旧 solver 输出只作初始状态或明示参考，不能当 ground truth 或每 tick 强迫候选跟随旧 q。

## 分组与评价窗口

采用 DATA_ANALYSIS 的 session 分组、exposure 和 split 规则。动作标签按采集信息、运动幅度/速度/工作空间等与候选结果无关的特征登记。未观测到的奇异/极限条件不宣称被真实数据覆盖。

主窗口为完整有效 source interval；尾部 settling 单独窗口，所有方法使用同一上限和阈值，不把额外收敛时间悄悄并入某一方法。输入无法配对/初始化时保留 input-invalid 单元及原因，不从 dataset inventory 删除。

录制评价的统计单位是动作/采集 session，重复运行不创造新的独立会话。formal 每方法每 recording 初始规定 3 个新进程重复，用于异步运行变异；source/配置一致但结果可存在调度差异。该重复数与 split 在正式前冻结。

## 指标

Primary：goal→execution position_error/orientation_error、valid_completion_rate。Guardrail：硬约束违反、deadline_miss_rate、完整层级质量、source_coverage 和完整窗口完成。Secondary：lag、joint_vaj、次级任务收益、time-to-failure。Diagnostic：每类失败、raw IK 与 downstream 差异、posture/proposal age。

主要效果按动作配对，再按 session 聚合；同时展示每动作散点和所有 required 单元失败矩阵。只有完整配对上的效果必须标 conditional-on-completion，不代表所有输入。不得把 E05 完成计数或 output MCAP CRC 当成精度/实时性通过。

## 执行与产出

复用 E05 的每文件独立进程与失败继续机制，但使用本实验冻结的 stratum/arm/输入合同，不导入其私有 recipe 或继承随时间变化的默认参数。单元异常后继续下一单元，批次记录失败；用户中断停止余下单元。正式 timing/holdout 仅在已确认 RT 工作站另行执行。

生成 dataset_inventory.csv、session_split.json、exposure_register.csv、resolved_candidate_configs、recording_summary.csv、paired_coverage.csv 和 failures.json。每单元保留 trace/状态/必要的 telemetry。A03 固定 source inventory 后生成 F10 和 T02/T04。

## 验收

代码测试覆盖重复/重叠分组、已暴露数据不能进 holdout、session 不跨 split、坏文件与缺流、子进程失败/中断、空选择、尾部未收敛和完整配对规则。小 fixture smoke 检查流程，不作真实效果证据。

formal preflight 必须检查 RT、输入与模型 hashes、候选冻结、阈值、曝光和分组；任一关键项缺失就保留 formal-not-ready。若只有单会话/已暴露数据，代码仍可完成并给描述性案例报告，但不伪造 CI 或泛化结论。

## Implemented development interface (2026-09-10)

`apps/study_e12` is an independent real MCC/PlaCo/CartesianPlanner/JointPlanner app.
Its duplicate business code remains app-local; it neither includes nor links E11 or any
production app. Its selected-method provenance remains a development candidate until a
separate E07–E10/candidate campaign freezes that selection.

The hash-only inventory under `inputs/inventory-development-20260910` contains the 50
visible raw MCAP files. No real payload was decoded in this implementation task. All
currently known files have unknown/historical E05 exposure and insufficient session
provenance, so they conservatively form one development group: **C6 evidence-insufficient**.
Raw bytes remain at their external locators. Each unavailable real canonical descriptor
exists, carries raw/model hashes and an explicit missing reason, and has `canonical=null`.
It never masquerades as converted data. The 450 real recording/stratum/method cells remain
listed, plus 9 development-only fixture cells; formal repetition is three per real cell.
Confirmatory rows remain unavailable even after historical exposed files are converted.

`data.py inventory` hashes and groups without decoding. `data.py convert` preserves raw
bytes, requires matching hashes, checks full causal initialization, stream/schema/frame and
strict timestamps, pairs original source times before optional one-to-one retiming, and
refuses every unmatched record instead of silently dropping it. Header/log/publish times
are preserved separately. No interpolation, clipping, frame relabelling or forced tracking
of old solver joints is performed. Default conversion rejects holdout before opening raw
bytes. The converter is tested with an explicitly labelled **real ROS2 MCAP fixture**.

`candidates.py` provides explicit equal-budget plan/execute/select/freeze. Every registered
candidate runs the same input/repeat budget in separate serial processes. The plan is
hashed, at most 30 candidates per method are accepted, holdout is rejected, and no directed
retry is performed. Execute reads actual nested unit raw data and independent metric/check
files; it records metric source hashes, complete quality coverage and preregistered guardrail
status. Selection orders guardrail eligibility, then full-quality coverage, then the declared
primary score. Missing policy/quality/evidence stays unavailable; no eligible candidate
remains a failed selection. Candidate freeze rechecks the complete trial ledger, ordering,
configs and source evidence hashes and refuses overwrite. It does not authorize holdout or
replace the parent's complete `campaign_freeze.v1` prerequisite contract.

```bash
python3 experiments/E12_recorded_motion_holdout/data.py inventory --locator /mnt/mcap_dataset --output /tmp/e12-inventory-new
python3 experiments/E12_recorded_motion_holdout/prepare.py --inventory /tmp/e12-inventory-new/dataset_inventory.json
python3 tools/mcc_placo_study/study.py --experiment E12 --dry-run --inventory-output /tmp/e12-matrix.json
python3 tools/mcc_placo_study/study.py --experiment E12 --case recording-fixture-zero-controlled --repeat 0 --smoke --timeout 60
python3 -m unittest discover -s experiments/E12_recorded_motion_holdout/tests -v
python3 experiments/E12_recorded_motion_holdout/candidates.py --help
```

Subsequent separately authorized recorded development uses explicit metadata/session/exposure,
`data.py convert --inventory ... --split ... --recording ... --recipe inputs/conversion_recipe.json
--model ... --output ...`, followed by declaration regeneration. Candidate planning requires
JSON candidates, development/tuning descriptor records, and a preregistered policy containing
`metric`, `direction`, `maximum_position_violation_rad`, `minimum_source_coverage`, and
`maximum_execution_holds`. Formal readiness additionally requires new unexposed sessions,
immutable dataset/split/config/candidate/threshold freeze and RT/core/frequency attestation.
Actual formal benchmark processes run serially in a later task. Source and settling windows
are separate; absent settling thresholds never become valid completion. Analysis and paper
remain **deferred**, and no real recording campaign or holdout experiment has been completed.

### Development acceptance evidence

The final fixture-only native smoke is
[runs/mcc_placo_study/development_acceptance/smoke/20260910T110605.051726Z-af76d83891](../../../runs/mcc_placo_study/development_acceptance/smoke/20260910T110605.051726Z-af76d83891).
Five positive cells completed and passed independent checks: three controlled methods,
the historical MCC chain and the unchanged production-static executable. Four separate
confirmatory fixture cells remain intentionally **unavailable**, because synthetic input
cannot supply holdout evidence. Those negative capability cells are development-only.
The 25-tick controlled smoke consumes 3/5 fixture source samples (`source_coverage=0.6`),
never reaches settling, and keeps valid completion unavailable. The baseline processes
three frozen 100 Hz source samples. No real recording has been decoded or executed.
The earlier smoke `20260910T105530.276084Z-af76d83891` is retained alongside the final
remeasurement following the independently corrected zero-variance lag check.

Seventeen E12 tests pass, including actual ROS2 MCAP fixture decode/CRC, raw immutability,
malformed MCAP, wrong frames, unpaired/nonmonotonic times, invalid initialization,
duplicate/overlap/session/exposure rules, equal budget/max-30 policy, quality-before-score
selection, immutable evidence-backed freeze and a mocked-metadata RT/holdout gate.
The preparation layer rejects nonfinite, ambiguous and out-of-model-bounds initial joints
without clipping or passing them to a candidate. The focused final test log is
[runs/development_preparation_validation_20260910/tests.log](../../../experiments/E12_recorded_motion_holdout/runs/development_preparation_validation_20260910/tests.log).

Future holdout conversion is implemented but was **not invoked**: `data.py convert --phase
confirmatory --frozen-definition ...` first validates the parent's complete campaign and RT
prerequisites, exact frozen raw/split/exposure/model/recipe identities and selected candidate
freeze, before opening raw bytes. Exposed/unknown records cannot pass. After authorized
conversion, `prepare.py --candidate-freeze ...` may enable only genuine unexposed holdout
cells; the execution declaration/canonical input freeze must then be finalized before
serial formal execution. The present 50-recording dataset still has 450 unavailable real
stratum/method cells and no eligible holdout. Code: implemented/tested/fixture-smoke-executed.
Formal readiness: blocked by explicit data/exposure/freeze/threshold/platform prerequisites.
Formal execution: none. Analysis and paper: deferred.
