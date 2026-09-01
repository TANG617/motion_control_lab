"""Strict argparse definitions shared only by this app's recipes."""

from __future__ import annotations

import argparse


BOOL_FLAGS = (
    "mujoco-viewer",
    "angular-admittance",
    "joint-position-braking-velocity-envelope",
    "red-joint-acceleration-limits",
    "red-proxqp-warm-start",
)

VALUE_FLAGS = (
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
    "red-primary-task-tcp-cartesian-progress-weight",
    "red-primary-task-tcp-cartesian-progress-preservation-tolerance",
    "red-primary-task-tcp-position-preservation-tolerance-mps",
    "red-primary-task-tcp-orientation-preservation-tolerance-radps",
    "red-secondary-task-yellow-posture-coupling-preservation-tolerance",
    "red-secondary-task-yellow-posture-coupling-weight",
    "red-secondary-task-yellow-posture-coupling-servo-gain-per-s",
    "red-secondary-task-link4-position-weight",
    "red-secondary-task-link4-position-servo-gain-per-s",
    "red-secondary-task-link4-position-preservation-tolerance-mps",
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

REPEAT_FLAGS = (
    "self-collision-pair",
    "yellow-task-posture-preference-joint-weight-multipliers",
    "red-secondary-task-yellow-posture-coupling-joint-weight-multipliers",
)

REPLAY_VALUE_FLAGS = (
    "input", "input-format", "left-stream", "right-stream",
    "initial-joint-state-stream", "csv-mapping", "timestamp-source",
    "target-period-ms", "pairing-policy", "nearest-tolerance-ms",
    "unmatched-policy", "execution-mode", "playback-rate", "output-dir",
    "output-root", "run-id", "viz-host", "viz-port",
)

REPLAY_BOOL_FLAGS = (
    "start-paused", "replay-trace", "replay-elbow-teleop", "terminal-input",
)


def _dest(flag: str) -> str:
    return flag.replace("-", "_")


def create_parser(source: str, program: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=program,
        argument_default=argparse.SUPPRESS,
    )
    for flag in BOOL_FLAGS:
        parser.add_argument(
            f"--{flag}", dest=_dest(flag), action=argparse.BooleanOptionalAction,
        )
    for flag in VALUE_FLAGS:
        parser.add_argument(f"--{flag}")
    for flag in REPEAT_FLAGS:
        parser.add_argument(f"--{flag}", action="append")
    parser.add_argument("--mcap")
    parser.add_argument(
        "--no-mcap", dest="mcap", action="store_const",
        const=None, default=argparse.SUPPRESS,
    )
    parser.add_argument("--dump-resolved-options", action="store_true")
    if source != "teleop":
        for flag in REPLAY_VALUE_FLAGS:
            parser.add_argument(f"--{flag}")
        for flag in REPLAY_BOOL_FLAGS:
            parser.add_argument(
                f"--{flag}", dest=_dest(flag),
                action=argparse.BooleanOptionalAction,
            )
    return parser
