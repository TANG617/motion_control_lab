#!/usr/bin/env python3
"""Structural and determinism checks for the generic contract generator."""

from __future__ import annotations

import argparse
import importlib.util
import json
import tempfile
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("generator", type=Path)
    parser.add_argument("contracts", nargs="+", type=Path)
    args = parser.parse_args()

    spec = importlib.util.spec_from_file_location("mcl_contract_generator", args.generator)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {args.generator}")
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)

    for path in args.contracts:
        document = generator.load_contract(path)
        first = generator.contract_header(document, "motion_control_lab::contracts::test")
        second = generator.contract_header(document, "motion_control_lab::contracts::test")
        if first != second:
            raise AssertionError(f"non-deterministic output for {path}")

    actual_topics: dict[str, str] = {}
    for path in args.contracts:
        document = generator.load_contract(path)
        for channel in document["channels"]:
            topic = channel["topic"]
            if topic in actual_topics:
                raise AssertionError(f"duplicate topic {topic}")
            actual_topics[topic] = channel["foxglove_schema"]
    expected_topics = {
        "/mcl/run/info": "mcl.telemetry.v1.RunInfo",
        "/mcl/events": "foxglove.Log",
        "/mcl/cartesian/input/left": "foxglove.PoseInFrame",
        "/mcl/cartesian/input/right": "foxglove.PoseInFrame",
        "/mcl/cartesian/nominal/left": "foxglove.PoseInFrame",
        "/mcl/cartesian/nominal/right": "foxglove.PoseInFrame",
        "/mcl/cartesian/goal/left": "foxglove.PoseInFrame",
        "/mcl/cartesian/goal/right": "foxglove.PoseInFrame",
        "/mcl/cartesian/reference/left": "foxglove.PoseInFrame",
        "/mcl/cartesian/reference/right": "foxglove.PoseInFrame",
        "/mcl/cartesian/ik/left": "foxglove.PoseInFrame",
        "/mcl/cartesian/ik/right": "foxglove.PoseInFrame",
        "/mcl/cartesian/execution/left": "foxglove.PoseInFrame",
        "/mcl/cartesian/execution/right": "foxglove.PoseInFrame",
        "/mcl/cartesian/scene": "foxglove.SceneUpdate",
        "/mcl/nullspace/elbow/scene": "foxglove.SceneUpdate",
        "/mcl/joints/ik": "foxglove.JointStates",
        "/mcl/joints/execution": "foxglove.JointStates",
        "/mcl/telemetry/tracking/cartesian": "mcl.telemetry.v1.CartesianTracking",
        "/mcl/telemetry/tracking/joints": "mcl.telemetry.v1.JointTracking",
        "/mcl/telemetry/solver/ik": "mcl.telemetry.v1.SolverTelemetry",
        "/mcl/telemetry/solver/avoidance": "mcl.telemetry.v1.SolverTelemetry",
        "/mcl/telemetry/planner/cartesian": "mcl.telemetry.v1.PlannerTelemetry",
        "/mcl/telemetry/planner/joint": "mcl.telemetry.v1.PlannerTelemetry",
        "/mcl/telemetry/safety/collision": "mcl.telemetry.v1.CollisionTelemetry",
        "/mcl/telemetry/coupling/avoidance_to_ik": "mcl.telemetry.v1.CouplingTelemetry",
        "/mcl/telemetry/compliance/cartesian": "mcl.telemetry.v1.CartesianCompliance",
        "/mcl/telemetry/worker/control": "mcl.telemetry.v1.WorkerTelemetry",
        "/mcl/telemetry/worker/avoidance": "mcl.telemetry.v1.WorkerTelemetry",
        "/mcl/telemetry/transport": "mcl.telemetry.v1.TransportTelemetry",
        "/mcl/telemetry/replay": "mcl.telemetry.v1.ReplayTelemetry",
    }
    if actual_topics != expected_topics:
        missing = sorted(expected_topics.keys() - actual_topics.keys())
        unexpected = sorted(actual_topics.keys() - expected_topics.keys())
        mismatched = sorted(
            topic for topic in expected_topics.keys() & actual_topics.keys()
            if expected_topics[topic] != actual_topics[topic]
        )
        raise AssertionError(
            f"topic contract mismatch missing={missing} unexpected={unexpected} "
            f"types={mismatched}"
        )
    if any(topic.startswith("/mc/") for topic in actual_topics):
        raise AssertionError("legacy /mc output topic remains in visualization contracts")

    invalid = json.loads(args.contracts[0].read_text(encoding="utf-8"))
    invalid["channels"][0]["foxglove_schema"] = "foxglove.JointState"
    with tempfile.TemporaryDirectory() as directory:
        invalid_path = Path(directory) / "invalid.json"
        invalid_path.write_text(json.dumps(invalid), encoding="utf-8")
        try:
            generator.load_contract(invalid_path)
        except ValueError as error:
            if "unsupported" not in str(error):
                raise
        else:
            raise AssertionError("unknown Foxglove schema was accepted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
