#!/usr/bin/env python3

import json
import pathlib
import sys


EXPECTED_CHANNELS = [
    ("/mc/ik/joint_states", "foxglove.JointStates", "ik_output_joint_state"),
    ("/mc/ik/target/left_pose", "foxglove.PoseInFrame", "left_input_target"),
    ("/mc/ik/target/right_pose", "foxglove.PoseInFrame", "right_input_target"),
    ("/mc/fk/pose/left_end_effector", "foxglove.PoseInFrame", "left_fk_output"),
    ("/mc/fk/pose/right_end_effector", "foxglove.PoseInFrame", "right_fk_output"),
]


def main() -> int:
    contract_path = pathlib.Path(sys.argv[1])
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    assert contract["schema_version"] == "mcl.foxglove_ik_visualization.v1"
    channels = contract["channels"]
    actual = [
        (entry["topic"], entry["foxglove_schema"], entry["role"])
        for entry in channels
    ]
    assert actual == EXPECTED_CHANNELS
    assert all(entry["required"] is True for entry in channels)
    assert "same sample" in contract["coherence"]["fk_invariant"]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
