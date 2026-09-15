> 历史评估归档：以下结果与命令属于旧 PiM 环境，不是当前 HARP 启动说明；原有结论保持不变。

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

# PiM-IK 接入运行评估

本机工程链路已跑通：v2 网络代码加载旧 Mamba 权重、真实 CUDA 推理、完整位姿 Primary、
Secondary 消费学习肘部目标，以及录制参考时序复现。下面保留成功和失败结果；这些测量
不构成拟人效果改善、严格 1 kHz 实时或避碰保证。

## 后续：planned 直接 IK 输出验证

可直接运行的命令见 [planned 入口](PIM_IK_PLANNED.md)。新增 `planned + pose-primary`
以同一 Mamba 参考执行完整位姿 Primary 和学习肘部 Secondary，joint OTG 关闭。
保留 planned 自身硬约束及数值默认值，故不将这些结果与下面旧 profile 表格作单变量比较。
最新 18 项 app CTest 全部通过，新增测试覆盖 planned 可选拓扑、默认兼容及拒绝 joint OTG
参数；四 Primary/两 scale/Secondary 保留验证同时在两种 profile 上执行。

| 实际运行 | 小幅 CSV `planned_small` | MCAP 32 秒片段 `planned_mcap` |
| --- | ---: | ---: |
| 退出码 | 0 | 0 |
| 接受 ticks | 31,171 | 31,938 |
| Secondary 最终选中 ticks | 31,157 | 31,934 |
| 位置保留漂移 max (m/s) | 0.000263 | 0.000270 |
| 姿态保留漂移 max (rad/s) | 0.000270 | 0.000270 |
| Scale 保留漂移 max | 0.000050 | 0.000070 |
| raw IK / 输出关节位置最大差 (rad) | 0 | 0 |
| Red execution P95 / P99 (ms) | 0.344 / 0.376 | 0.648 / 0.706 |
| Deadline misses | 6 | 10 |
| 参考年龄 max (ms) | 82 | 87 |

两次 manifest 都明确记录 `joint_otg.enabled=false`，没有调用 JointPlanner。
raw/output 差为零只证明直接输出路径一致，不证明对笛卡尔参考的跟踪误差为零。
均使用 monitor；长尾/参考年龄仍受本机调度影响，不宣称严格实时或所有输入都可行。
`planned_recorded` 以最新安装入口复现全部 31,172 条小幅记录（含初始化）：时间、序号、
目标版本、source index 完全一致，cos/sin 最大差为 `2.22e-16`。

交互入口首次启动因本机 psi_cortex 已占用 8765 端口退出，记录保留在
`planned_keyboard/terminal.log`；未停止该服务。新 planned recipes 使用独立 8775 端口。
随后 `planned_keyboard_8775` 通过真实 PTY 启动 TUI/Foxglove 和 GPU 模型，运行 3 秒后
正常退出（退出码 0）。

## 环境、方法与产物

环境和冻结 revision/hash 见 [接入说明](PIM_IK.md)。使用既有 algorithm build/install，
已检查安装二进制解析到同一 algorithm overlay 的 MCC、Ruckig、sim/viz 库，未改 MCC。
运行均为软件实验，未接硬件。产物在 MCL 忽略的 `runs/pim_ik_assets/`，不进入版本管理。

小幅输入 `reachable_30s.csv` 由 `scripts/pim_ik/make_reachable_motion.py` 从 R1 初始
关节状态生成：双臂 joint1 各加 `0.005*(1-cos(2πt/30)) rad`，以 FK 输出含 TCP offset
的双臂位姿，100 Hz、30 秒。A/B/C 顺序执行，均 14 活动关节、Red 1000 Hz、Yellow
100 Hz、10 ms 输入周期、32 秒运行上限、`deadline-policy monitor`；硬限制、OTG 和
其余数值参数一致。记录的是每个已接受 tick，不把失败迭代当作执行结果。

| 配置与产物目录 | 退出码 | 接受 ticks | 结果 |
| --- | ---: | ---: | --- |
| A `A_position_manual`：position-first + manual | 1 | 13 | 第 14 次 JointPlanner target infeasible |
| B `B_pose_manual`：pose-primary + manual | 1 | 380 | 第 381 次同类 JointPlanner 故障 |
| C `C_pose_mamba`：pose-primary + Mamba | 0 | 31,146 | 完成约 31.146 秒活动参考时间 |

A/B 原始错误均为 `JointPlanner plan failed: joint trajectory target is infeasible with the
configured limits`。未通过放宽硬限制或调参掩盖失败；两组的短前缀不能与 C 全程做有效
性能/质量比较。因此当前对照不能证明 PiM 优于人工参考，A/B 的 OTG 失败仍待独立诊断。
各组 `metrics.json` 保留双臂 raw/executed TCP、关节、肘部和运行统计，`native/` 保留
resolved 配置、trace、manifest，`console.log` 和 `exit_code.txt` 保留退出证据。

## 在线结果

另用既有 MCAP `fixtures/raw/sliced-RW1AZHYCSEFT5_RW1AZHYCSEFT5260310002_20260813164525_0.mcap`
执行 32 秒片段，使用原 left/right calib_target_pose 和初始 IK joint_states streams。
该 recipe 的 Yellow 为 500 Hz，因此它是独立回放验证，不是上述 A/B/C 的受控比较。
产物目录为 `live_mcap_isolated`；没有宣称播完整个 MCAP。

| 指标 | 小幅 C | 既有 MCAP 片段 |
| --- | ---: | ---: |
| 接受 ticks | 31,146 | 31,768 |
| Secondary 成功并最终选中 | 31,011 | 31,765 |
| 仅 Primary 接受（不计学习完成） | 135 | 3 |
| Primary 位置保留漂移 max (m/s) | 0.000262 | 0.000270 |
| Primary 姿态保留漂移 max (rad/s) | 0.000270 | 0.000270 |
| Scale 保留漂移 max | 0.000050 | 0.000070 |
| 最终选中关节硬约束违例 max | 1.974e-5 | 1.995e-5 |
| 参考年龄 P95 / P99 / max (ms) | 12 / 13 / 21 | 14 / 15 / 30 |
| 推理 P95 / P99 (ms) | 2.182 / 4.479 | 2.712 / 4.030 |
| Red execution P95 / P99 (ms) | 0.372 / 0.418 | 0.668 / 0.793 |
| Red execution max (ms) | 40.342 | 38.696 |
| Deadline misses / skipped releases | 8 / 88 | 74 / 315 |

保留漂移均在位置 `5e-4 m/s`、姿态 `5e-4 rad/s`、scale `1e-4` 的配置容差内。
最终选中关节约束违例也在现有验收容差 `1e-3` 内；这里是 Core 原生约束诊断值，不把
不同单位的关节约束强行解释为米或弧度。历史 aggregate hard violation 包含被丢弃的
失败下层证据，C/MCAP 分别达到 0.394006/1.325234，不能当作最终接受关节解的违例。
新版 trace/record 另有 `selected_shared_hard_violation`，评估从 manifest 的
`tracking.maximum_accepted_joint_violation` 读取最终接受值。

推理分位数按实际消费的唯一预测计算，排除启动推理（C 141.94 ms、MCAP 137.51 ms，
均在启动周期控制之前）。Red 数字来自原有 worker execution 统计，分位数是 0.001 ms
直方图上界，**不包含 post-iteration observer**；release-to-finish 和 solver 子区间
也保存在 metrics。存在长尾和 misses，故没有达到严格 1 ms 截止期验收。

| 跟踪/连续性指标 | C raw IK / OTG 执行 | MCAP raw IK / OTG 执行 |
| --- | ---: | ---: |
| 左 TCP 位置误差 max (mm) | 0.0441 / 14.5350 | 35.7934 / 67.2074 |
| 右 TCP 位置误差 max (mm) | 0.0462 / 0.0463 | 51.1316 / 89.2646 |
| 左 TCP 姿态误差 max (rad) | 0.0000487 / 0.0355841 | 0.0849264 / 0.1594481 |
| 右 TCP 姿态误差 max (rad) | 0.0000435 / 0.0000440 | 0.2361052 / 0.3775612 |
| 左肘单 tick 位移 max (mm) | 0.2968 / 0.2716 | 0.4985 / 0.5435 |
| 关节总变化量 (rad) | 1.2241 / 1.2661 | 118.9247 / 117.8445 |
| 配置碰撞 pairs 最小距离 (m) | 0.125710 / 0.125710 | 0.091943 / 0.091847 |

TCP 误差相对该 tick 笛卡尔参考，包含配置 TCP offset；不能把 HQP 下层保留速度残差
等同于全链路零跟踪误差。C 的左 TCP 执行位置 P99 为 0.1170 mm，但峰值达 14.535 mm，
需要单独评估初始化学习目标和 OTG 执行的影响。MCAP 更有明显原始与执行跟踪误差。
碰撞距离每 10 个接受 tick 离线采样配置 pairs；MCAP 的最小距离低于 0.1 m 软目标，
没有硬避碰保证，也没有做所有 link pairs 的连续碰撞认证。

此前 `live_mcap` 尝试与另一个 native 回放/GPU 离线作业并行，约 15.78 秒因参考年龄
超过 100 ms 退出。保留该失败产物；随后停止并行负载，单独执行上述 `live_mcap_isolated`。
独立重复成功不抹去资源竞争下的年龄故障。

## 模型、复现和针对性验证

- 最新 18 项 `hierarchical_kinematics_step` CTest 全部通过。覆盖四个 scaled Primary
  任务、两个共享组、14 活动关节、姿态瓶颈同臂共用 scale、另一臂独立、Secondary 扰动
  的任务/scale 保留、R1 FK/臂角往返、固定腕目标肘部运动、±π 连续性及退化错误。
- 确定性假推理进程测试完整 30 帧窗口、在途请求跳过、活动年龄、暂停/恢复、延迟超时、
  断连和进程退出；普通 CTest 不依赖 GPU 环境。
- `gpu_smoke.json` 和安装后 `installed_final/parity.json` 检查真实 CUDA 扩展、权重
  加载及 v2/旧 Mamba 前向一致性。`installed_final` 为最新安装入口 3 秒在线复查，
  记录原始 Python `predictions.jsonl` 并用同一窗口离线比较，301 个窗口的最大差为 **0**，
  v2/旧网络前向最大差也为 **0**，结果存入 parity.json。
- `recorded_full` 复现 C 的全部 **31,147 条记录（含启动记录）**：消费时间、预测序号、
  目标版本和 source index 一致；cos/sin 最大差分别为 `2.22e-16` / `1.11e-16`，
  左右目标分量最大差为 `3.33e-16` / `4.44e-16`。Secondary 最终选中仍为 31,011 次。
  不声称 Yellow 调度或关节执行轨迹逐位一致。
- 最新安装版本在 `recorded_latest` 再次完成同样的 31,147 条记录复现，匹配结果保存为
  `comparison.json`；此版本也验证了独立的最终硬约束字段和录制输入覆盖保护。
- 早期完整在线窗口与离线预测比较最大差 `1.061e-7`，其中 C++ 会重新归一化 cos/sin。
  最新 Python 原始响应比较可排除这一边界差异。

可复用命令、依赖锁定和录制复现入口见 [接入说明](PIM_IK.md)。本轮交付了在线参考与
任务装配；拟人性、对照组故障、OTG 跟踪和严格实时预算仍是有明确测量证据的后续评估项。

## 2026-09-12：键盘 TCP 镜像输入验收

新增 `--mirror-tcp-input`，默认关闭，仅在 app 输入侧镜像左主控 TCP 的相对运动。
未更改 HQP、硬约束、OTG、模型或 Yellow 参数。右臂保留 Yellow posture，镜像开启时
清除右臂人工 link4 目标；不复制左臂关节角或学习肘部目标。

- 完成 algorithm build/install 和 20/20 项 app CTest。针对性用例覆盖非对称初始姿态、
  不同 TCP 偏移的平移/旋转、SO(3) 合法性、暂停开关不改目标、重新捕获基准、双 FK 重设、
  左主控/右 link4 互斥、故障时拒绝开关、CLI 默认/禁用/不支持来源，以及旧录制字段兼容。
- 已安装的 planned keyboard 入口真实 Mamba 运行 20 秒，native 与模型正常退出。
  平移步长 5 mm、旋转步长 1°；覆盖平移、旋转、运行时拒绝切换、暂停切换和双 FK 重设。
  8775 有既有用户会话，本次使用 8777，不停止既有进程。
- `runs/pim_ik_assets/mirror_live/`：20,011 条消费记录（含启动记录），19,992 条为
  Secondary 成功且最终选中。镜像 ON→OFF→ON 发生在活动时间 6.535 s / 9.039 s，
  两次切换均保留目标及 revision=5。源 TCP 目标最大镜像位置误差 `2.55e-16 m`，
  旋转矩阵 Frobenius 误差 `1.40e-15`，属于浮点舍入量级；它们不是执行跟踪误差。
- WebSocket 解码 1,603 组角度/场景消息。在线 MCAP 的 1,884 组消息均与消费记录匹配，
  含模式标签；其中 3 组为 Primary-only，未标成学习目标完成。参考年龄 P99 为 14 ms。
- `runs/pim_ik_assets/mirror_recorded_final/`：使用 `mirror_live/initial.csv` 提供键盘初态，
  recorded 模式重放消费记录中的双臂目标和臂角，正常退出。20,011 条记录的消费时刻、
  预测序号、采样时刻、模式标签、目标 revision 一致；目标分量误差最大 `6.67e-16`，
  cos/sin 误差最大 `2.23e-16`（重新归一化）。回放 MCAP 的 1,981 组消息匹配消费记录。

上述结果验证输入变换、交互和记录发布，不构成模型效果、严格执行对称性或避碰改善结论。
运行驱动、resolved options、模式/目标检查、WebSocket/MCAP 检查均保留在上述 run 目录。
