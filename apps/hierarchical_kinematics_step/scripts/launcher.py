#!/usr/bin/env python3
"""Strict, importable launcher for hierarchical_kinematics_step experiments."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Sequence

PROFILES = (
    "hierarchical",
    "planned",
    "planned-otg",
    "planned-otg-nullspace",
    "planned-otg-nullspace-admittance-kinematic-sim",
)

_BOOL_FLAGS = (
    "mujoco-viewer",
    "angular-admittance",
    "joint-position-braking-velocity-envelope",
    "red-joint-acceleration-limits",
    "red-proxqp-warm-start",
)

_VALUE_FLAGS = (
    "side", "urdf", "mujoco-model", "host", "port", "red-rate",
    "yellow-rate", "ui-rate", "ui", "viz", "deadline-policy", "duration",
    "step-m", "min-step-m", "max-step-m", "rotation-step-deg",
    "environment-stiffness", "environment-damping", "maximum-force",
    "rotation-environment-stiffness", "rotation-environment-damping",
    "maximum-torque", "wrench-filter-alpha", "regularization",
    "position-tolerance-m", "orientation-tolerance-rad",
    "minimum-position-improvement-m", "minimum-orientation-improvement-rad",
    "maximum-hard-violation", "joint-position-margin-rad",
    "legacy-cartesian-progress-weight",
    "legacy-cartesian-preservation-tolerance",
    "legacy-scale-preservation-tolerance",
    "legacy-posture-preservation-tolerance", "legacy-yellow-posture-weight",
    "legacy-yellow-to-red-coupling-weight",
    "legacy-minimum-collision-distance-m",
    "legacy-collision-influence-distance-m",
    "legacy-collision-damping-gain-per-s", "legacy-collision-weight",
    "red-primary-task-tcp-position-progress-weight",
    "red-primary-task-tcp-position-preservation-tolerance-mps",
    "red-primary-task-tcp-position-progress-preservation-tolerance",
    "red-secondary-task-tcp-orientation-progress-weight",
    "red-secondary-task-tcp-orientation-preservation-tolerance-radps",
    "red-secondary-task-tcp-orientation-progress-preservation-tolerance",
    "red-tertiary-task-yellow-posture-coupling-preservation-tolerance",
    "red-tertiary-task-yellow-posture-coupling-weight",
    "red-tertiary-task-yellow-posture-coupling-servo-gain-per-s",
    "red-tertiary-task-link4-position-weight",
    "red-tertiary-task-link4-position-servo-gain-per-s",
    "red-tertiary-task-link4-position-preservation-tolerance-mps",
    "yellow-maximum-iterations", "red-proxqp-maximum-iterations",
    "red-proxqp-absolute-tolerance",
    "red-proxqp-primal-infeasibility-tolerance",
    "yellow-task-posture-preference-weight",
    "yellow-task-posture-preference-servo-gain-per-s",
    "yellow-constraints-self-collision-avoidance-minimum-distance-m",
    "yellow-constraints-self-collision-avoidance-influence-distance-m",
    "yellow-constraints-self-collision-avoidance-damping-gain-per-s",
    "yellow-constraints-self-collision-avoidance-weight", "base-frame",
    "left-end-effector-frame", "right-end-effector-frame", "left-link4-frame",
    "right-link4-frame", "left-tcp-offset", "right-tcp-offset", "joint-names",
    "default-joint-positions", "left-arm-joint-indices",
    "right-arm-joint-indices", "effort-limits", "inactive-joints",
    "collision-mesh-search-paths", "joint-stream-source-revision",
    "joint-stream-source-path", "joint-stream-source-sha256",
    "joint-stream-jerk-override-reason", "joint-stream-joint-names",
    "joint-stream-position-lower-rad", "joint-stream-position-upper-rad",
    "joint-stream-max-velocity-rad-per-s",
    "joint-stream-max-acceleration-rad-per-s2",
    "joint-stream-max-jerk-rad-per-s3", "max-linear-velocity-mps",
    "max-linear-acceleration-mps2", "max-linear-jerk-mps3",
    "max-angular-velocity-rps", "max-angular-acceleration-rps2",
    "max-angular-jerk-rps3", "cartesian-synchronization",
    "joint-synchronization", "joint-algorithm", "joint-target-mode",
    "future-o1-velocity-deadband-rad-per-s", "settling-fk-position-m",
    "settling-fk-orientation-rad", "settling-velocity-rad-per-s",
    "settling-acceleration-rad-per-s2", "settling-required-cycles",
)

_REPEAT_FLAGS = (
    "self-collision-pair",
    "yellow-task-posture-preference-joint-weight-multipliers",
    "red-tertiary-task-yellow-posture-coupling-joint-weight-multipliers",
)

_REPLAY_VALUE_FLAGS = (
    "input", "input-format", "left-stream", "right-stream",
    "initial-joint-state-stream", "csv-mapping", "timestamp-source",
    "target-period-ms", "pairing-policy", "nearest-tolerance-ms",
    "unmatched-policy", "execution-mode", "playback-rate", "output-dir",
    "output-root", "run-id", "viz-host", "viz-port",
)

_REPLAY_BOOL_FLAGS = (
    "start-paused", "replay-trace", "replay-elbow-teleop", "terminal-input",
)


def _dest(flag: str) -> str:
    return flag.replace("-", "_")


def create_parser(source: str) -> argparse.ArgumentParser:
    if source not in {"teleop", "mcap", "csv"}:
        raise ValueError("source must be teleop, mcap, or csv")
    parser = argparse.ArgumentParser(
        prog=f"run_{'keyboard' if source == 'teleop' else source + '_replay'}.py",
        argument_default=argparse.SUPPRESS,
    )
    parser.add_argument("--profile", required=True, choices=PROFILES)
    for flag in _BOOL_FLAGS:
        parser.add_argument(
            f"--{flag}", dest=_dest(flag), action=argparse.BooleanOptionalAction,
        )
    for flag in _VALUE_FLAGS:
        parser.add_argument(f"--{flag}")
    for flag in _REPEAT_FLAGS:
        parser.add_argument(f"--{flag}", action="append")
    parser.add_argument("--mcap")
    parser.add_argument("--no-mcap", dest="mcap", action="store_const",
                        const=None, default=argparse.SUPPRESS)
    parser.add_argument("--dump-resolved-options", action="store_true")
    if source != "teleop":
        for flag in _REPLAY_VALUE_FLAGS:
            parser.add_argument(f"--{flag}")
        for flag in _REPLAY_BOOL_FLAGS:
            parser.add_argument(
                f"--{flag}", dest=_dest(flag),
                action=argparse.BooleanOptionalAction,
            )
    return parser


def _binary_path() -> str:
    explicit = os.environ.get("MCL_BINARY")
    if explicit:
        return explicit
    prefix = Path(os.environ.get("MCL_INSTALL_PREFIX", "/workspace/install/algorithm"))
    installed = prefix / "bin" / "mcl_hierarchical_kinematics_step"
    if installed.exists():
        return str(installed)
    found = shutil.which("mcl_hierarchical_kinematics_step")
    return found or str(installed)


def _asset_paths(profile: str) -> tuple[str, str]:
    prefix = Path(os.environ.get("MCL_INSTALL_PREFIX", "/workspace/install/algorithm"))
    root = prefix / "share" / "motion-control-lab" / "robots" / "r1" / "mujoco"
    if profile == "hierarchical":
        urdf = Path("/workspace/models/r1.cos.urdf")
    elif profile == "planned":
        urdf = Path("/workspace/models/Psi_R1_visual_collision.urdf")
    elif profile.endswith("admittance-kinematic-sim"):
        urdf = root / "urdf" / "r1.urdf"
    else:
        urdf = Path(
            "/workspace/products/synrobot/modules/common/robot_description/"
            "psi_r1/urdf/Psi_R1_rev1.urdf"
        )
    return str(urdf), str(root / "mjcf" / "r1.xml")


def _presets(source: str, profile: str) -> dict[str, object]:
    urdf, mujoco = _asset_paths(profile)
    values: dict[str, object] = {"urdf": urdf}
    planned = profile != "hierarchical"
    otg = profile.startswith("planned-otg")
    nullspace = "nullspace" in profile
    maximum = profile.endswith("admittance-kinematic-sim")
    if source == "teleop":
        values.update({"red-rate": "1000", "yellow-rate": "100",
                       "deadline-policy": "strict" if profile == "hierarchical" else "monitor",
                       "ui": "tui", "viz": "foxglove", "host": "127.0.0.1",
                       "port": "8765", "mcap": None})
        if planned:
            values.update({
                "max-linear-velocity-mps": "0.9",
                "max-linear-acceleration-mps2": "5.0",
                "max-linear-jerk-mps3": "80.0",
                "max-angular-velocity-rps": "3.0",
                "max-angular-acceleration-rps2": "20.0",
                "max-angular-jerk-rps3": "150.0" if profile == "planned" else "200.0",
            })
    elif source == "mcap":
        values.update({
            "input-format": "mcap",
            "input": "/workspace/fixtures/raw/sliced-RW1AZHYCSEFT5_RW1AZHYCSEFT5260310002_20260813164525_0.mcap",
            "left-stream": "/hal/tracker/htc/left/calib_target_pose",
            "right-stream": "/hal/tracker/htc/right/calib_target_pose",
            "initial-joint-state-stream": "/mc/ik/joint_states",
            "timestamp-source": "header_stamp", "target-period-ms": "10",
            "pairing-policy": "nearest", "nearest-tolerance-ms": "5",
            "unmatched-policy": "drop_with_diagnostics",
            "execution-mode": "batch" if profile == "hierarchical" else "realtime",
            "playback-rate": "1", "red-rate": "1000",
            "yellow-rate": "500" if otg else "100",
            "deadline-policy": "strict" if profile == "hierarchical" else "monitor",
            "ui": "none" if profile == "hierarchical" else "tui",
            "terminal-input": profile != "hierarchical", "replay-trace": True,
            "viz": "none" if profile == "hierarchical" else "foxglove",
            "host": "127.0.0.1", "port": "8765", "mcap": None,
            "output-root": "/workspace/runs/mcl_hierarchical_kinematics_step",
            "start-paused": True,
        })
        if profile == "hierarchical":
            values.pop("input", None)
            values.pop("initial-joint-state-stream", None)
            values.pop("nearest-tolerance-ms", None)
            values.update({"left-stream": "/mc/ik/left_target",
                           "right-stream": "/mc/ik/right_target",
                           "target-period-ms": "1", "pairing-policy": "exact",
                           "start-paused": False})
        if planned:
            if profile == "planned":
                values.update({"max-linear-velocity-mps": "0.9",
                               "max-linear-acceleration-mps2": "5.0",
                               "max-linear-jerk-mps3": "80.0",
                               "max-angular-velocity-rps": "3.0",
                               "max-angular-acceleration-rps2": "20.0",
                               "max-angular-jerk-rps3": "150.0"})
            elif profile == "planned-otg":
                values.update({"max-linear-velocity-mps": "0.8",
                               "max-linear-acceleration-mps2": "4.0",
                               "max-linear-jerk-mps3": "60.0",
                               "max-angular-velocity-rps": "3.5",
                               "max-angular-acceleration-rps2": "15.0",
                               "max-angular-jerk-rps3": "180.0"})
            else:
                values.update({"max-linear-velocity-mps": "3.0",
                               "max-linear-acceleration-mps2": "20.0",
                               "max-linear-jerk-mps3": "400.0",
                               "max-angular-velocity-rps": "3.0",
                               "max-angular-acceleration-rps2": "20.0",
                               "max-angular-jerk-rps3": "300.0"})
        if nullspace:
            values["replay-elbow-teleop"] = True
    else:
        values.update({
            "input-format": "csv", "left-stream": "left", "right-stream": "right",
            "timestamp-source": "csv_timestamp",
            "target-period-ms": "10" if profile == "planned" else "1",
            "execution-mode": "batch", "red-rate": "1000", "yellow-rate": "100",
            "deadline-policy": "monitor" if profile == "planned" else "strict", "ui": "none",
            "terminal-input": False, "viz": "none", "mcap": None,
            "output-root": str(Path(__file__).resolve().parents[3] / "runs" /
                               "hierarchical_kinematics_step"),
        })
        if profile == "planned":
            values.update({"max-linear-velocity-mps": "0.9",
                           "max-linear-acceleration-mps2": "5.0",
                           "max-linear-jerk-mps3": "80.0",
                           "max-angular-velocity-rps": "3.0",
                           "max-angular-acceleration-rps2": "20.0",
                           "max-angular-jerk-rps3": "150.0"})
    if otg:
        values["joint-target-mode"] = "future-o1-pv"
    if maximum:
        values.update({"mujoco-model": mujoco, "mujoco-viewer": source != "csv"})
    return values


def _append_flag(command: list[str], flag: str, value: object) -> None:
    if flag in _BOOL_FLAGS or flag in {"start-paused", "replay-trace",
                                      "replay-elbow-teleop", "terminal-input"}:
        command.append(f"--{flag}" if bool(value) else f"--no-{flag}")
    elif flag == "mcap" and value is None:
        command.append("--no-mcap")
    elif flag in _REPEAT_FLAGS:
        for item in value if isinstance(value, list) else [value]:
            command.extend((f"--{flag}", str(item)))
    else:
        command.extend((f"--{flag}", str(value)))


def build_command(source: str, argv: Sequence[str]) -> list[str]:
    """Parse Python options and return the exact native command."""
    namespace = vars(create_parser(source).parse_args(list(argv)))
    profile = str(namespace.pop("profile"))
    resolved = _presets(source, profile)
    for name, value in namespace.items():
        resolved[name.replace("_", "-")] = value
    if "output_dir" in namespace:
        resolved.pop("output-root", None)
    if "output_root" in namespace:
        resolved.pop("output-dir", None)
    if namespace.get("pairing_policy") == "exact":
        resolved.pop("nearest-tolerance-ms", None)
    if source != "teleop" and not resolved.get("input"):
        raise SystemExit(f"run_{source}_replay.py: error: --input is required")
    native_source = "teleop" if source == "teleop" else "replay"
    command = [_binary_path(), "--profile", profile, native_source]
    dump = bool(resolved.pop("dump-resolved-options", False))
    resolved["launcher-argv-json"] = json.dumps(
        {"launcher": Path(sys.argv[0]).name, "argv": list(argv)},
        separators=(",", ":"),
    )
    if native_source == "replay":
        resolved["launcher"] = str(Path(sys.argv[0]).resolve())
    for flag, value in resolved.items():
        _append_flag(command, flag, value)
    if dump:
        command.append("--dump-resolved-options")
    cpu_set = os.environ.get("MCL_CPU_SET", "5-7")
    rt_priority = os.environ.get("MCL_RT_PRIORITY", "60")
    if rt_priority:
        command = ["chrt", "--fifo", rt_priority, *command]
    if cpu_set:
        command = ["taskset", "--cpu-list", cpu_set, *command]
    return command


def run(source: str, argv: Sequence[str]) -> None:
    """Replace this process with the configured native application."""
    command = build_command(source, argv)
    environment = os.environ.copy()
    allowed_mcl_environment = {
        "MCL_BINARY", "MCL_INSTALL_PREFIX", "MCL_LD_LIBRARY_PATH",
        "MCL_CPU_SET", "MCL_RT_PRIORITY",
    }
    for name in tuple(environment):
        if name.startswith("MCL_") and name not in allowed_mcl_environment:
            environment.pop(name)
    extra_library_path = environment.get("MCL_LD_LIBRARY_PATH")
    if extra_library_path:
        current = environment.get("LD_LIBRARY_PATH")
        environment["LD_LIBRARY_PATH"] = (
            extra_library_path if not current else f"{extra_library_path}:{current}"
        )
    os.execvpe(command[0], command, environment)


def main(argv: Sequence[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    source_parser = argparse.ArgumentParser()
    source_parser.add_argument("source", choices=("teleop", "mcap", "csv"))
    source, remaining = source_parser.parse_known_args(args)
    run(source.source, remaining)


if __name__ == "__main__":
    main()
