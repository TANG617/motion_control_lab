# 肘部参考命名与历史兼容整理验收

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

本轮只整理 MCL 的代码命名、显示和历史资料，未修改 MCC Predictor、模型、solver/task/约束或控制时序。未提交或推送，未重跑完整实验矩阵。

## 迁移结果

| 原名称/行为 | 当前名称/行为 |
|---|---|
| pim_visualization.*、PimAngleTracker、PimVisualizationSnapshot | elbow_reference_visualization.*、ElbowReferenceAngleTracker、ElbowReferenceVisualizationSnapshot |
| makePimVisualizationSnapshot / appendPimVisualization | makeElbowReferenceVisualizationSnapshot / appendElbowReferenceVisualization |
| 镜像界面固定 L PiM | 按明确配置显示 L HARP / L recorded / L manual，保留 R Yellow |
| 实时 HARP | 保持 /mcl/harp/left/angle、scene；mcl.harp.Angle；harp_* marker |
| recorded 回放使用 PiM topic | /mcl/elbow_reference/left/angle、scene；mcl.elbow_reference.Angle；elbow_reference_* marker |

现役不再发布或双发 PiM topic。录制来源仅标 recorded，不根据文件名、数据值或控制权推断模型身份；旧记录格式、数值、时间与原始结果不重写。

新增录制布局为 apps/hierarchical_kinematics_step/foxglove/recorded_elbow_reference.layout.json，实时 HARP 布局原字节不变。
四份历史文档、两个旧布局、三个历史辅助工具移至 app 内 legacy/pim/{docs,foxglove,tools}；安装时同样使用明确的 legacy 路径。
旧入口只有拒绝提示，没有兼容转发。历史评估增加归档说明，原结论保留。

## 验证结果

- HARP ON 独立构建/安装成功，针对性 CTest **11/11 通过**。
- HARP OFF 独立构建/安装成功，针对性 CTest **10/10 通过**；ldd 确认不依赖 Torch/CUDA/posture_reference。
- 几何、双向角度展开、尺度/跟踪误差、精确 schema 与全部 marker 身份通过；manual 不发布，harp 与 recorded 分别使用各自的单一 topic。
- 三种来源标签、镜像/暂停、原生拒绝、公开正常/request 数值一致和错误参数拒绝均通过。
- CUDA 生命周期由 ON CTest 覆盖；CPU 生命周期使用 CPU 模型目录复验通过。首次额外 CPU 检查误传 CUDA 模型目录，原生拒绝 deployment contract；失败日志完整保留，未改模型或放宽检查。
- 旧格式兼容 fixture 与现有 HARP reachable/consumed.jsonl 的读取、暂停、输入耗尽失败通过。旧格式 fixture 是明确生成的兼容测试数据，不冒充历史 PiM 科研录制；真实 HARP 来源 SHA256 在测试前后相同。
- 公共 app smoke 串行执行，每次子进程 timeout 30 秒；两套构建的原始请求、结果和异常分别归档。预期失败仍保留非零退出码。
- 架构检查、历史工具/布局安装位置及剩余字符串审计通过；旧冻结安装二进制和依赖库 SHA256 均未改变。

## 产物与命令

证据根目录：[本轮证据](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z)。

- [实际构建/安装命令](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/commands.json)
- [测试、模型和录制读取命令](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/test-commands.json)
- [全部测试状态，含首次错误模型目录失败](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/test-results.json)
- [ON 公共 app smoke](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/on-public-smoke/result.json)；[OFF 公共 app smoke](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/off-public-smoke/result.json)
- [源码/归档/冻结安装审计](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/audit.json)
- [逐行剩余 PiM 字符串分类](/workspace/runs/mcl_elbow_cleanup_20260914T092745Z/remaining-pim.json)

独立安装：`/workspace/install/mcl-elbow-cleanup-on` 和 `/workspace/install/mcl-elbow-cleanup-off`；旧 `/workspace/install/mcl-app-reuse-20260914` 保持冻结。

可复验入口：

```bash
cmake --build /workspace/build/mcl-elbow-cleanup-on -j6
cmake --install /workspace/build/mcl-elbow-cleanup-on
ctest --test-dir /workspace/build/mcl-elbow-cleanup-on -j1 --output-on-failure --timeout 30 \
  -R 'architecture.boundaries|apps.mcl_hierarchical_kinematics_step_(elbow_reference_visualization|tcp_mirror|elbow_geometry|harp_lifecycle|execution_contract|options|control|tui_projection|replay_pipeline_gate|rejection_policy)$'
python3 tests/architecture_boundaries.py /workspace/labs/motion-control-lab
git diff --check
```

名称与展示整理完成，不扩大既有科研结论。正式 RT、全量新实验、A03、分析与论文工作不属于本轮。
