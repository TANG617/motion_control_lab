# 实验输入同步

`experiments/*/inputs/` 纳入 motion-control-lab 子仓的普通 Git：包含 canonical JSON、descriptor、inventory、转换配方、required units 和 superseded 历史快照。`generated` 表示来源由脚本生成，不表示可随运行覆盖。提交输入保留其原有 development、fixture、unavailable 或未冻结状态。

每次执行产物仍写入被忽略的 `runs/`；后续输入修改使用明确的新版本，保留历史输入与 hash。生成器 locator/hash 是生成时的来源记录，旧脚本可能已归档或变更；不要用当前脚本 hash 覆盖历史值。使用已保存的 canonical 输入和从生成器重建输入是不同的复现路径。

E11 的 `superseded/e339e9c97026a9fb/descriptor-original.json` 保留被替换前的原位置；读取该历史输入应使用同目录的 `descriptor-relocated.json`，它指向 hash 匹配的归档 `canonical.json`。

## 固定 devcontainer 路径

| 资源 | 容器内路径 |
| --- | --- |
| Lab checkout | `/workspace/labs/motion-control-lab` |
| R1 模型 | `/workspace/models/r1.cos.urdf`，必须匹配输入记录的 SHA-256 |
| E05 batch / E12 新 inventory 数据目录 | `/workspace/fixtures/raw/batch` |

保留绝对路径。另一台电脑需将 workspace、模型和数据放到相同位置，并安装匹配的依赖和 app。Lab 的 Git 不包含目录外的 MCAP、模型或安装产物；E02/E03 的外部数据引用仍按各自合同提供。

2026-09-15 的 batch 含 37 个 `-slice.mcap`，内容清单见 [E12 batch inventory](E12_recorded_motion_holdout/inputs/inventory-batch-20260915/dataset_inventory.json)。这批切片与旧 E12 的 50 个原始 MCAP 是不同输入；旧清单及 descriptor 保留原路径和 hash。新清单只记录字节身份和保守会话分组，不说明源流、坐标系或实验准入已经验证。

## 跨电脑检查

从 Lab 根目录运行 E05 文件发现检查：

```bash
python3 experiments/E05_real_scene_planned_mcap_batch_replay/run_batch.py --dry-run
```

核验 batch 快照的文件完整性（只读文件，不解码或运行 app）：

```bash
python3 - <<'PY'
import json
import sys
from pathlib import Path
sys.path.insert(0, 'tools/mcc_placo_study')
from evidence import stable_hash, verify_artifact
inventory = json.loads(Path('experiments/E12_recorded_motion_holdout/inputs/inventory-batch-20260915/dataset_inventory.json').read_text())
if inventory['inventory_hash'] != stable_hash(inventory['records']):
    raise SystemExit('inventory hash mismatch')
failed = [row['raw']['locator'] for row in inventory['records'] if not verify_artifact(row['raw'])]
if failed:
    raise SystemExit('missing or changed inputs:\n' + '\n'.join(failed))
print(f"verified {len(inventory['records'])} batch inputs")
PY
```

E06–E12 的声明准备与执行见各实验 README。E12 旧声明不会自动切换到新 batch；需要完成新输入转换及准入后再显式建立对应实验声明。
