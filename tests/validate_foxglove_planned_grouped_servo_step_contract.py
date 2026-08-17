#!/usr/bin/env python3

import json
import pathlib
import sys


EXPECTED_CHANNELS = [
    (
        "/mc/planning/request/left_pose",
        "foxglove.PoseInFrame",
        "left_planning_request",
        "left_planning_request",
    ),
    (
        "/mc/planning/request/right_pose",
        "foxglove.PoseInFrame",
        "right_planning_request",
        "right_planning_request",
    ),
]


def main() -> int:
    contract_path = pathlib.Path(sys.argv[1])
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    assert contract["schema_version"] == "mcl.foxglove_planned_grouped_servo_step.v1"
    assert contract["extends"] == "mcl.foxglove_ik_visualization.v1"
    channels = contract["channels"]
    actual = [
        (
            entry["topic"],
            entry["foxglove_schema"],
            entry["role"],
            entry["entity_id"],
        )
        for entry in channels
    ]
    assert actual == EXPECTED_CHANNELS
    assert all(entry["required"] is True for entry in channels)
    assert "Red attempt" in contract["coherence"]["attempt"]
    assert "retain" in contract["coherence"]["retention"]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
