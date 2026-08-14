#!/usr/bin/env python3
"""Validate that E01 and E04 declare the same R1 position task."""

from __future__ import annotations

import json
import pathlib
import sys
from typing import Any


SHARED_FIELDS = {
    "robot_model",
    "base_frame",
    "end_effector_frame",
    "controlled_joint_names",
    "initial_joints_rad",
    "goal_joints_rad",
    "maximum_iterations",
    "position_tolerance_m",
    "unlisted_joints",
}


def load(path: pathlib.Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} E01_DEFINITION E04_DEFINITION", file=sys.stderr)
        return 2

    e01 = load(pathlib.Path(sys.argv[1]))
    e04 = load(pathlib.Path(sys.argv[2]))
    e01_factors = e01["controlled_factors"]
    e04_factors = e04["controlled_factors"]
    for field in sorted(SHARED_FIELDS):
        if e01_factors.get(field) != e04_factors.get(field):
            raise ValueError(f"E01/E04 controlled factor differs: {field}")

    expected_model = "/workspace/models/r1.cos.urdf"
    if e01_factors["robot_model"] != expected_model:
        raise ValueError(f"unexpected shared model: {e01_factors['robot_model']}")
    if e01["inputs"][0]["artifact"] != expected_model:
        raise ValueError("E01 input does not use the hard-coded R1 model")
    if e04["inputs"][0]["artifact"] != expected_model:
        raise ValueError("E04 input does not use the hard-coded R1 model")

    print("E01 and E04 declare the same hard-coded R1 position task")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"R1 smoke pair validation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
