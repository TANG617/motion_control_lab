# 构建与运行

> 更新日期：2026-09-14

## 环境与依赖

原生算法 app 使用 C++17、CMake 3.28+、Python 3，支持原生 macOS/Linux。
需要 Eigen、Pinocchio、已安装的 MCC 0.4、Sim 0.1、Viz 0.5。
PlaCo/FTXUI 源码固定在 third_party；jsoncpp、MCAP 和必要的压缩/QP fallback 固定版本由 CMake 管理。
测试及研究工具另需 jsonschema、numpy、scipy、pytest、matplotlib；HKS telemetry 使用 Protobuf。

`MCL_BUILD_HARP=ON` 需要带 posture_reference 的 MCC 安装及匹配的模型/runtime。
`MCL_BUILD_PSIBOT_TELEOP=ON` 启用 Linux SDK 客户端，其 app-local 发布快照需要 ROS Jazzy 运行库。
SDK 依赖仅属于该客户端。机器人模型通过 app 参数或 launcher 配置提供；配置阶段不再读取 E01 模型。

## Workspace

从 `/workspace` 构建，使用当前 algorithm defaults：

```bash
COLCON_DEFAULTS_FILE=/workspace/.vscode/workspace/algorithm_colcon_defaults.yaml \
  colcon build --packages-up-to motion_control_lab
```

标准 build/install 是 `/workspace/build/algorithm` 与 `/workspace/install/algorithm`。
`COLCON_PREFIX_PATH` 用于依赖发现，不决定本次 build/install 或 launcher 选择。

## 独立 CMake

从 Lab 根目录：

```bash
cmake --preset dev -DCMAKE_PREFIX_PATH=/path/to/native/install -DCMAKE_INSTALL_PREFIX="$PWD/out/dev"
cmake --build --preset dev -j4
cmake --install build/dev
ctest --preset dev
```

无 TUI/Foxglove transport 的构建使用 `headless` preset；保持算法及公开 request 不变：

```bash
cmake --preset headless -DCMAKE_PREFIX_PATH=/path/to/native/install -DCMAKE_INSTALL_PREFIX="$PWD/out/headless"
cmake --build --preset headless -j4
ctest --preset headless
```

按需用现役 app 对应的 `MCL_BUILD_*` 开关选择目标。HARP、SDK 客户端默认关闭。
Linux 默认编译 CPU affinity；开发环境不满足默认 CPU 配置时可在独立构建设
`MCL_ENABLE_CPU_AFFINITY=OFF`，并把该构建明确标记为开发验证。
`MCL_TEST_URDF` 只供运行时回归测试使用，默认 `/workspace/models/r1.cos.urdf`；
其他模型/窗口测试的前置条件以各 app README 和 CTest 命令为准。

## 启动与验证

app-local 脚本默认选择 `${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/mcl_<app>`。
`MCL_BINARY` 优先级最高，可显式选择独立构建；launcher 不隐式构建。
配置顺序为 compiled defaults → script/environment → trailing explicit arguments。
公开 request 模式拒绝额外算法 CLI 覆盖；baseline 的算法配置冻结。

```bash
python3 tests/architecture_boundaries.py "$PWD"
python3 -m unittest discover -s tools/app_execution/tests -v
python3 -m unittest discover -s tools/mcc_placo_study/tests -v
```

共享组件和 retained app 测试由 CTest 注册。离线分析使用研究 README 的窄范围命令，
不要递归收集 runs 中的历史源码快照。运行证据与命令必须对应同一 build/install 和动态库。
可选 GUI、CPU、模型或 SDK 环境缺项应明确记录；不作为算法通过的证据。
