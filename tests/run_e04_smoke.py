#!/usr/bin/env python3
"""Run E04 with its private ROS/OpenSoT environment and inspect the evidence bundle."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import pathlib
import subprocess
import tempfile

from validate_contracts import validate_definition, validate_manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True, type=pathlib.Path)
    parser.add_argument("--definition", required=True, type=pathlib.Path)
    parser.add_argument("--work-root", required=True, type=pathlib.Path)
    parser.add_argument("--ros-setup", required=True, type=pathlib.Path)
    parser.add_argument("--library-path", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    definition = validate_definition(args.definition.resolve())
    args.work_root.mkdir(parents=True, exist_ok=True)

    environment = os.environ.copy()
    current_library_path = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = (
        args.library_path
        if not current_library_path
        else f"{args.library_path}:{current_library_path}"
    )

    with tempfile.TemporaryDirectory(prefix="e04-", dir=args.work_root) as temporary:
        output_root = pathlib.Path(temporary) / "runs"
        command = [
            "/bin/bash",
            "-c",
            'source "$1"; shift; exec "$@"',
            "run-e04",
            str(args.ros_setup.resolve()),
            str(args.executable.resolve()),
            "--output-root",
            str(output_root),
            "--run-id",
            "ctest-e04",
        ]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                "E04 executable failed\n"
                f"stdout:\n{completed.stdout}\n"
                f"stderr:\n{completed.stderr}"
            )

        run_root = output_root / "ctest-e04"
        manifest = validate_manifest(run_root / "manifest.json")
        if manifest["status"] != "completed":
            raise ValueError(f"unexpected run status: {manifest['status']}")
        if manifest["failures"] != {"required": 0, "total": 0}:
            raise ValueError(f"unexpected run failures: {manifest['failures']}")
        dependencies = manifest["environment"].get("dependencies", {})
        if "opensot" not in dependencies:
            raise ValueError("run manifest does not record the OpenSoT dependency")

        expected_model = definition["controlled_factors"]["robot_model"]
        expected_model_sha256 = hashlib.sha256(
            pathlib.Path(expected_model).read_bytes()
        ).hexdigest()
        expected_input = {
            "id": "psi_r1_cos",
            "locator": expected_model,
            "sha256": expected_model_sha256,
        }
        if manifest["inputs"] != [expected_input]:
            raise ValueError(f"unexpected recorded model input: {manifest['inputs']}")
        resolved_factors = manifest["definition"]["resolved"]["controlled_factors"]
        for field in (
            "robot_model",
            "controlled_joint_names",
            "initial_joints_rad",
            "goal_joints_rad",
        ):
            if resolved_factors[field] != definition["controlled_factors"][field]:
                raise ValueError(f"resolved definition changed shared field: {field}")

        expected = set(definition["artifact_requirements"])
        missing = [relative for relative in expected if not (run_root / relative).is_file()]
        if missing:
            raise ValueError(f"missing required E04 artifacts: {missing}")

        trace_path = (
            run_root
            / "arms"
            / "opensot_v4_0_1_ros2_osqp"
            / "psi_r1_cos"
            / "trace.csv"
        )
        with trace_path.open(newline="", encoding="utf-8") as stream:
            trace = list(csv.DictReader(stream))
        if len(trace) < 2:
            raise ValueError("E04 trace does not contain a solver step")
        for row in trace:
            for field, value in row.items():
                if not math.isfinite(float(value)):
                    raise ValueError(f"non-finite E04 trace value in {field}")

        initial_error = float(trace[0]["position_error_m"])
        final_error = float(trace[-1]["position_error_m"])
        tolerance = definition["controlled_factors"]["position_tolerance_m"]
        maximum_iterations = definition["controlled_factors"]["maximum_iterations"]
        if not final_error < initial_error:
            raise ValueError("OpenSoT did not reduce the task error")
        if not final_error <= tolerance:
            raise ValueError(
                f"final error {final_error} exceeds declared tolerance {tolerance}"
            )
        if int(trace[-1]["iteration"]) > maximum_iterations:
            raise ValueError("OpenSoT exceeded the declared iteration limit")

        with (run_root / "evaluation" / "metrics.csv").open(
            newline="", encoding="utf-8"
        ) as stream:
            metrics = list(csv.DictReader(stream))
        metric_ids = {row["metric_id"] for row in metrics}
        declared_metrics = {
            metric
            for values in definition["metrics"].values()
            for metric in values
        }
        if metric_ids != declared_metrics:
            raise ValueError(
                f"metric mismatch: produced={sorted(metric_ids)}, "
                f"declared={sorted(declared_metrics)}"
            )
        for metric in metrics:
            if not math.isfinite(float(metric["value"])):
                raise ValueError(f"non-finite E04 metric: {metric['metric_id']}")
        guardrail = next(
            row for row in metrics if row["metric_id"] == "joint_limit_violation_count"
        )
        if float(guardrail["value"]) != 0.0:
            raise ValueError("OpenSoT joint-limit guardrail failed")

    print("E04 OpenSoT smoke test and evidence bundle are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
