# Motion Control Lab

Motion Control Lab 是面向机器人遥操作 whole-body IK 的可复现实验仓库。研究愿景见
[docs/experiments.md](docs/experiments.md)，实验与证据生命周期见
[docs/experiment_architecture.md](docs/experiment_architecture.md)。

当前仓库提供第一条端到端纵向切片：

- 仓库内可直接修改的固定版本 placo C++ 源码；
- Experiment、run manifest 与 tidy metric 的首版合同；
- append-only run artifact store；
- `E01_placo_smoke` 实验及其合成二连杆 URDF；
- 对实验定义、run manifest、artifact 哈希和 placo 求解结果的自动检查。

E01 是基础设施 smoke test，只证明 placo C++ 求解链与证据落盘链能够跑通，不是
算法性能结论。

## 环境要求

当前开发环境以 Apple Silicon macOS 为基线，不需要 ROS2 或容器。

```bash
brew install cmake pinocchio
```

placo 源码已经保存在 `third_party/placo/`，配置时不下载 placo。首次配置仍需
联网获取固定版本的 eiquadprog 和 jsoncpp；Pinocchio 使用本机安装，这两个小型
依赖下载到 CMake build tree。

## 构建与测试

```bash
cmake --preset dev
cmake --build --preset dev --target e01_placo_smoke -j8
ctest --preset dev
```

手动运行 E01，并把证据写入 build tree：

```bash
./build/dev/e01_placo_smoke --output-root ./build/manual-runs
```

不指定 `--output-root` 时，run 会写入
`experiments/E01_placo_smoke/runs/`。每次执行创建新的
`<UTC timestamp>__<definition hash>` 目录，拒绝覆盖已有 run。

## 目录

```text
adapters/execution/       通用 artifact store、manifest 与 SHA-256
contracts/                definition、manifest、metric 合同
data/raw/                 原始数据占位；不得静默改写
data/canonical/           规范数据占位
experiments/              Experiment 实例、run 与 reviewed result
analyses/                 只消费已有证据的 Analysis
docs/                     愿景、通用架构和项目实现映射
third_party/placo/        仓库内直接构建和修改的 placo 源码
tests/                    合同与端到端检查
```

具体实现与通用架构角色的对应关系见
[docs/project_mapping.md](docs/project_mapping.md)。

## placo 源码策略

placo 固定到 `v0.9.23` 对应提交
`e6c288604639d67b979a16cb2ad26913413c8e3a`，源码作为普通文件保存在
[`third_party/placo/`](third_party/placo/)；它不是 Git submodule，也不包含
嵌套 `.git` 目录。

为适配 Lab，CMake 关闭 Python bindings，源码中还包含当前 Pinocchio 4 /
Eigen 5 所需的 API 兼容修改。完整来源和修改清单见
[`third_party/placo/MOTION_CONTROL_LAB.md`](third_party/placo/MOTION_CONTROL_LAB.md)。

可以直接编辑 `third_party/placo/src/` 做算法实验。CMake 会直接重新编译这份
源码，并将 C++ build inputs 的内容指纹写入 run manifest 的
`environment.placo_revision`。升级上游版本或修改算法后必须创建新的 run，
不能复用旧证据。
