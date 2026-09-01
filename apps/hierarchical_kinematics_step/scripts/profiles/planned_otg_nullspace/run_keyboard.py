#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from _launcher.entrypoint import bind  # noqa: E402
from profiles.planned_otg_nullspace.config import RECIPES  # noqa: E402

_BOUND = bind(RECIPES["keyboard"], __file__)
build_command = _BOUND.build_command
run = _BOUND.run


def main() -> None:
    run(sys.argv[1:])


if __name__ == "__main__":
    main()

