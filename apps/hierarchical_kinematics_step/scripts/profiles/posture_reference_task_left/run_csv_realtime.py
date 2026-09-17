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
    profile='posture-reference-task-left',
    name='harp_csv_realtime',
    source='csv',
    options={
        'hqp-layout': 'pose-primary',
        'elbow-reference': 'harp',
        'harp-device': 'cuda',
        'inactive-joints': 'head_yaw_joint,head_pitch_joint,torso_yaw_joint,torso_pitch_joint,knee_pitch_joint,ankle_pitch_joint',
        'urdf': '/workspace/models/Psi_R1_visual_collision.urdf',
        'input-format': 'csv',
        'left-stream': 'left',
        'right-stream': 'right',
        'timestamp-source': 'csv_timestamp',
        'target-period-ms': 10,
        'execution-mode': 'realtime',
        'red-rate': 1000,
        'yellow-rate': 100,
        'deadline-policy': 'monitor',
        'ui': 'none',
        'terminal-input': False,
        'replay-elbow-teleop': False,
        'viz': 'none',
        'mcap': None,
        'output-root': '/workspace/runs/mcl_posture_reference',
    },
)


def main() -> None:
    run_recipe(_RECIPE, sys.argv[1:], __file__)


if __name__ == "__main__":
    main()
