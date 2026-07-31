# E01：PlaCo C++ 基础 Smoke Test

## 目的

E01 验证以下最小链路：

1. 从固定源码版本构建 placo C++ 核心；
2. 读取仓库内的二连杆 URDF；
3. 固定 floating base 并创建末端位置 task；
4. 运行 IK，将末端位置误差降到声明阈值以内；
5. 保存完整 run manifest、输入副本、逐步 trace、tidy metrics 和报告；
6. 重新计算 artifact SHA-256，验证证据包完整。

它是依赖和实验框架的 qualification，不回答 task 取舍、业务机器人效果或 placo
性能优劣。

## 固定声明

研究问题、输入、arm、控制条件、指标和失败策略均在
[`definition.json`](definition.json) 中。执行器只接受这份固定声明；修改声明或
URDF 后必须重新运行 CMake，使构建中记录的输入 hash 同步更新。

placo 当前固定为 `v0.9.23` 对应提交
`e6c288604639d67b979a16cb2ad26913413c8e3a`。
源码以普通文件保存在仓库根目录的 `third_party/placo/`，不是 submodule。
为兼容当前 Homebrew Pinocchio 4 / Eigen 5，vendored 源码包含只涉及上游 API
重命名的最小修改，不改变 IK 求解逻辑。每次 run 还会记录实际参与构建的 placo
C++ 源码内容指纹，修改源码后不会继续伪装成未修改的上游版本。

## 运行

从仓库根目录执行：

```bash
cmake --preset dev
cmake --build --preset dev --target e01_placo_smoke -j8
./build/dev/e01_placo_smoke
```

运行目录遵循：

```text
runs/<run-id>/
  manifest.json
  definition/resolved.json
  inputs/synthetic_two_link/
    canonical_copy.urdf
    metadata.json
  arms/placo_v0_9_23/synthetic_two_link/
    trace.csv
    status.json
  evaluation/
    metrics.csv
    report.md
```

自动验证：

```bash
ctest --preset dev --output-on-failure
```

只有 manifest 状态为 `completed`、末端误差达到声明阈值、关节限位 guardrail
通过且全部 artifact 哈希一致时，测试才通过。Smoke test 产生的耗时只用于确认
采集链有效，不构成性能 Benchmark。
