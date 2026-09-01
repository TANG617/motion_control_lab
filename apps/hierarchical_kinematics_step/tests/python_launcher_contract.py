#!/usr/bin/env python3
from __future__ import annotations

import contextlib
import importlib
import io
import json
import os
import re
import subprocess
import sys
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = APP_DIR / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from _launcher import arguments  # noqa: E402


PROFILES = {
    "hierarchical": "hierarchical",
    "planned": "planned",
    "planned_otg": "planned-otg",
    "planned_otg_nullspace": "planned-otg-nullspace",
    "planned_otg_nullspace_admittance_kinematic_sim":
        "planned-otg-nullspace-admittance-kinematic-sim",
}
RECIPES = {
    "keyboard": "teleop",
    "mcap_interactive": "mcap",
    "mcap_headless": "mcap",
    "csv_batch": "csv",
}
DEFAULT_URDF = "/workspace/models/Psi_R1_visual_collision.urdf"


def load_recipe(profile_module: str, recipe: str):
    return importlib.import_module(f"profiles.{profile_module}.run_{recipe}")


def native_command(module, arguments_: list[str], binary: str) -> list[str]:
    previous = {
        name: os.environ.get(name)
        for name in ("MCL_BINARY", "MCL_CPU_SET", "MCL_RT_PRIORITY")
    }
    os.environ["MCL_BINARY"] = binary
    os.environ["MCL_CPU_SET"] = ""
    os.environ["MCL_RT_PRIORITY"] = ""
    try:
        return module.build_command(arguments_)
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def resolved(module, arguments_: list[str], binary: str) -> dict[str, object]:
    command = native_command(module, [*arguments_, "--dump-resolved-options"], binary)
    completed = subprocess.run(command, check=True, text=True, capture_output=True)
    return json.loads(completed.stdout)


def required_arguments(
    profile: str,
    recipe: str,
    urdf: str,
    mujoco: str,
) -> list[str]:
    result = ["--urdf", urdf]
    if recipe.startswith("mcap_"):
        result.extend(("--input", "/tmp/recipe-contract.mcap"))
    elif recipe == "csv_batch":
        result.extend(("--input", "/tmp/recipe-contract.csv"))
    if profile.endswith("admittance_kinematic_sim"):
        result.extend(("--mujoco-model", mujoco, "--no-mujoco-viewer"))
    return result


def assert_recipe_tree() -> None:
    profile_root = SCRIPTS_DIR / "profiles"
    actual_profiles = {
        path.name for path in profile_root.iterdir()
        if path.is_dir() and not path.name.startswith("__")
    }
    assert actual_profiles == set(PROFILES)
    expected_files = {f"run_{name}.py" for name in RECIPES}
    for profile in PROFILES:
        actual_files = {
            path.name for path in (profile_root / profile).glob("run_*.py")
        }
        assert actual_files == expected_files


def assert_flag_contract() -> None:
    python_flags = set().union(
        arguments.BOOL_FLAGS,
        arguments.VALUE_FLAGS,
        arguments.REPEAT_FLAGS,
        arguments.REPLAY_VALUE_FLAGS,
        arguments.REPLAY_BOOL_FLAGS,
        {"dump-resolved-options", "no-mcap"},
    )
    cpp_source = (APP_DIR / "options.cpp").read_text()
    cpp_source = re.sub(r'"\s*"', "", cpp_source)
    cpp_flags = set(re.findall(
        r"--([a-z][a-z0-9-]+)",
        cpp_source,
    ))
    cpp_flags = {
        flag[3:] if flag.startswith("no-") else flag
        for flag in cpp_flags if not flag.endswith("-")
    }
    python_flags = {
        flag[3:] if flag.startswith("no-") else flag for flag in python_flags
    }
    assert cpp_flags - {"help", "profile", "launcher", "launcher-argv-json"} == \
        python_flags


def assert_fixed_recipe_commands(binary: str, urdf: str, mujoco: str) -> None:
    for profile_module, native_profile in PROFILES.items():
        for recipe, source in RECIPES.items():
            module = load_recipe(profile_module, recipe)
            recipe_arguments = required_arguments(
                profile_module, recipe, urdf, mujoco,
            )
            command = native_command(module, recipe_arguments, binary)
            assert command[:4] == [
                binary,
                "--profile",
                native_profile,
                "teleop" if source == "teleop" else "replay",
            ]
            assert command[command.index("--urdf") + 1] == urdf
            try:
                with contextlib.redirect_stderr(io.StringIO()):
                    native_command(module, ["--profile", native_profile], binary)
            except SystemExit as error:
                assert error.code == 2
            else:
                raise AssertionError("fixed recipe accepted --profile")

            value = resolved(module, recipe_arguments, binary)
            assert value["profile"] == native_profile
            assert value["robot"]["profile_provenance"] == native_profile
            provenance = json.loads(value["launcher_argv_json"])
            assert provenance["profile"] == native_profile
            assert provenance["recipe"] == recipe
            assert provenance["launcher"].endswith(f"run_{recipe}.py")


def assert_recipe_semantics(binary: str) -> None:
    for profile_module in PROFILES:
        command = native_command(
            load_recipe(profile_module, "keyboard"), [], binary,
        )
        assert command[command.index("--urdf") + 1] == DEFAULT_URDF

    interactive = native_command(
        load_recipe("planned_otg_nullspace", "mcap_interactive"), [], binary,
    )
    assert interactive[interactive.index("--execution-mode") + 1] == "realtime"
    assert interactive[interactive.index("--ui") + 1] == "tui"
    assert interactive[interactive.index("--viz") + 1] == "foxglove"
    assert "--terminal-input" in interactive
    assert "--start-paused" in interactive
    assert "--replay-elbow-teleop" in interactive

    headless = native_command(
        load_recipe("planned_otg_nullspace", "mcap_headless"), [], binary,
    )
    assert headless[headless.index("--execution-mode") + 1] == "batch"
    assert headless[headless.index("--ui") + 1] == "none"
    assert headless[headless.index("--viz") + 1] == "none"
    assert "--no-terminal-input" in headless
    assert "--no-start-paused" in headless
    assert "--no-replay-elbow-teleop" in headless

    maximum_headless = native_command(
        load_recipe(
            "planned_otg_nullspace_admittance_kinematic_sim", "mcap_headless",
        ),
        [],
        binary,
    )
    assert "--no-mujoco-viewer" in maximum_headless

    hierarchical = load_recipe("hierarchical", "mcap_headless")
    try:
        native_command(hierarchical, [], binary)
    except SystemExit as error:
        assert "--input is required" in str(error)
    else:
        raise AssertionError("hierarchical MCAP recipe accepted missing input")

    exact = native_command(
        load_recipe("planned", "mcap_headless"),
        ["--pairing-policy", "exact"],
        binary,
    )
    assert "--nearest-tolerance-ms" not in exact

    previous_prefix = os.environ.get("MCL_INSTALL_PREFIX")
    os.environ["MCL_INSTALL_PREFIX"] = "/tmp/mcl-recipe-prefix"
    try:
        maximum = native_command(
            load_recipe(
                "planned_otg_nullspace_admittance_kinematic_sim", "keyboard",
            ),
            [],
            binary,
        )
    finally:
        if previous_prefix is None:
            os.environ.pop("MCL_INSTALL_PREFIX", None)
        else:
            os.environ["MCL_INSTALL_PREFIX"] = previous_prefix
    assert maximum[maximum.index("--urdf") + 1] == DEFAULT_URDF
    assert maximum[maximum.index("--mujoco-model") + 1].startswith(
        "/tmp/mcl-recipe-prefix/",
    )

    previous_cpu = os.environ.get("MCL_CPU_SET")
    previous_rt = os.environ.get("MCL_RT_PRIORITY")
    os.environ["MCL_CPU_SET"] = "5-7"
    os.environ["MCL_RT_PRIORITY"] = "60"
    try:
        prefixed = load_recipe("planned", "keyboard").build_command([])
    finally:
        if previous_cpu is None:
            os.environ.pop("MCL_CPU_SET", None)
        else:
            os.environ["MCL_CPU_SET"] = previous_cpu
        if previous_rt is None:
            os.environ.pop("MCL_RT_PRIORITY", None)
        else:
            os.environ["MCL_RT_PRIORITY"] = previous_rt
    assert prefixed[:6] == [
        "taskset", "--cpu-list", "5-7", "chrt", "--fifo", "60",
    ]


def assert_explicit_overrides(binary: str, urdf: str, mujoco: str) -> None:
    module = load_recipe(
        "planned_otg_nullspace_admittance_kinematic_sim", "keyboard",
    )
    value = resolved(
        module,
        [
            "--urdf", urdf,
            "--mujoco-model", mujoco,
            "--no-mujoco-viewer",
            "--ui", "none",
            "--viz", "none",
            "--red-rate", "700",
            "--yellow-rate", "70",
            "--joint-algorithm", "jerk-limited",
            "--joint-position-braking-velocity-envelope",
            "--red-joint-acceleration-limits",
            "--left-tcp-offset", "0.1,0.2,0.3,0,0,0,1",
            "--inactive-joints", "knee_pitch_joint",
            "--self-collision-pair", "left_arm_link4:body_link4",
            "--yellow-task-posture-preference-joint-weight-multipliers",
            "left_arm_joint6=2.5",
        ],
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


def main() -> int:
    binary = sys.argv[1]
    urdf = sys.argv[2]
    mujoco = sys.argv[3]
    assert_recipe_tree()
    assert_flag_contract()
    assert_fixed_recipe_commands(binary, urdf, mujoco)
    assert_recipe_semantics(binary)
    assert_explicit_overrides(binary, urdf, mujoco)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
