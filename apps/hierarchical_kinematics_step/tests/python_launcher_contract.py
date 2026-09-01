#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_DIR / "scripts"))
import launcher  # noqa: E402


def native_command(source: str, arguments: list[str], binary: str) -> list[str]:
    previous = {name: os.environ.get(name) for name in (
        "MCL_BINARY", "MCL_CPU_SET", "MCL_RT_PRIORITY")}
    os.environ["MCL_BINARY"] = binary
    os.environ["MCL_CPU_SET"] = ""
    os.environ["MCL_RT_PRIORITY"] = ""
    try:
        return launcher.build_command(source, arguments)
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def resolved(source: str, arguments: list[str], binary: str) -> dict[str, object]:
    command = native_command(source, [*arguments, "--dump-resolved-options"], binary)
    completed = subprocess.run(command, check=True, text=True, capture_output=True)
    return json.loads(completed.stdout)


def main() -> int:
    binary = sys.argv[1]
    urdf = sys.argv[2]
    mujoco = sys.argv[3]

    python_flags = set().union(
        launcher._BOOL_FLAGS,
        launcher._VALUE_FLAGS,
        launcher._REPEAT_FLAGS,
        launcher._REPLAY_VALUE_FLAGS,
        launcher._REPLAY_BOOL_FLAGS,
        {"profile", "dump-resolved-options", "no-mcap"},
    )
    cpp_flags = set(re.findall(r"--([a-z][a-z0-9-]+)",
                               (APP_DIR / "options.cpp").read_text()))
    cpp_flags = {flag[3:] if flag.startswith("no-") else flag
                 for flag in cpp_flags if not flag.endswith("-")}
    python_flags = {flag[3:] if flag.startswith("no-") else flag
                    for flag in python_flags}
    assert cpp_flags - {"help", "profile", "launcher", "launcher-argv-json"} == \
        python_flags - {"profile"}

    for profile in launcher.PROFILES:
        arguments = ["--profile", profile, "--urdf", urdf, "--ui", "none",
                     "--viz", "none"]
        if profile.endswith("admittance-kinematic-sim"):
            arguments.extend(("--mujoco-model", mujoco, "--no-mujoco-viewer"))
        value = resolved("teleop", arguments, binary)
        assert value["profile"] == profile
        assert value["robot"]["profile_provenance"] == profile

    value = resolved(
        "teleop",
        ["--profile", "planned-otg-nullspace-admittance-kinematic-sim",
         "--urdf", urdf, "--mujoco-model", mujoco, "--ui", "none",
         "--viz", "none", "--red-rate", "700", "--yellow-rate", "70",
         "--joint-algorithm", "jerk-limited",
         "--joint-position-braking-velocity-envelope",
         "--red-joint-acceleration-limits", "--left-tcp-offset",
         "0.1,0.2,0.3,0,0,0,1", "--inactive-joints", "knee_pitch_joint",
         "--self-collision-pair", "left_arm_link4:body_link4",
         "--yellow-task-posture-preference-joint-weight-multipliers",
         "left_arm_joint6=2.5"],
        binary,
    )
    assert value["runtime"]["red_rate_hz"] == 700
    assert value["solver"]["joint_position_braking_velocity_envelope_enabled"]
    assert value["solver"]["red_joint_acceleration_limits_enabled"]
    assert value["robot"]["left_tcp_offset"]["translation"] == [0.1, 0.2, 0.3]
    assert value["robot"]["inactive_joint_names"] == ["knee_pitch_joint"]
    assert value["robot"]["self_collision_link_pairs"] == [
        {"first_link": "left_arm_link4", "second_link": "body_link4"}
    ]
    assert json.loads(value["launcher_argv_json"])["launcher"]

    teleop = native_command(
        "teleop", ["--profile", "hierarchical", "--mcap", "/tmp/view.mcap"],
        binary,
    )
    assert "/workspace/models/r1.cos.urdf" in teleop
    assert teleop[teleop.index("--mcap") + 1] == "/tmp/view.mcap"

    csv = native_command(
        "csv", ["--profile", "planned-otg", "--input", "/tmp/input.csv"], binary
    )
    assert csv[:4] == [binary, "--profile", "planned-otg", "replay"]
    assert "--input-format" in csv and "csv" in csv
    assert "--no-terminal-input" in csv
    assert "--terminal-input" not in csv
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
