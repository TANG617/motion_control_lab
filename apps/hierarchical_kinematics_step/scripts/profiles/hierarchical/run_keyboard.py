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
    profile='hierarchical',
    name='keyboard',
    source='teleop',
    options={
        'urdf': '/workspace/models/Psi_R1_visual_collision.urdf',
        'red-rate': 1000,
        'yellow-rate': 100,
        'deadline-policy': 'strict',
        'ui': 'tui',
        'viz': 'foxglove',
        'host': '127.0.0.1',
        'port': 8765,
        'mcap': None,
    },
)


def main() -> None:
    run_recipe(_RECIPE, sys.argv[1:], __file__)


if __name__ == "__main__":
    main()
