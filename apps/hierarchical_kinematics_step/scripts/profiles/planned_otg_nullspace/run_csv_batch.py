#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from _launcher.command import run_recipe  # noqa: E402
from _launcher.recipe import Recipe  # noqa: E402

_RECIPE = Recipe(
    profile='planned-otg-nullspace',
    name='csv_batch',
    source='csv',
    options={
        'urdf': '/workspace/models/Psi_R1_visual_collision.urdf',
        'input-format': 'csv',
        'left-stream': 'left',
        'right-stream': 'right',
        'timestamp-source': 'csv_timestamp',
        'target-period-ms': 1,
        'execution-mode': 'batch',
        'red-rate': 1000,
        'yellow-rate': 100,
        'deadline-policy': 'strict',
        'ui': 'none',
        'terminal-input': False,
        'replay-elbow-teleop': False,
        'viz': 'none',
        'mcap': None,
        'output-root': str(Path(__file__).resolve().parents[5] / 'runs' / 'hierarchical_kinematics_step'),
        'joint-target-mode': 'future-o1-pv',
    },
)


def main() -> None:
    run_recipe(_RECIPE, sys.argv[1:], __file__)


if __name__ == "__main__":
    main()
