#!/usr/bin/env python3
"""Run E01 in a temporary build workspace and inspect its evidence bundle."""

from __future__ import annotations

import argparse
import csv
import pathlib
import subprocess
import tempfile

from validate_contracts import validate_definition, validate_manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True, type=pathlib.Path)
    parser.add_argument("--definition", required=True, type=pathlib.Path)
    parser.add_argument("--work-root", required=True, type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    definition = validate_definition(args.definition.resolve())
    args.work_root.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="e01-", dir=args.work_root) as temporary:
        output_root = pathlib.Path(temporary) / "runs"
        completed = subprocess.run(
            [
                str(args.executable.resolve()),
                "--output-root",
                str(output_root),
                "--run-id",
                "ctest-e01",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                "E01 executable failed\n"
                f"stdout:\n{completed.stdout}\n"
                f"stderr:\n{completed.stderr}"
            )

        run_root = output_root / "ctest-e01"
        manifest = validate_manifest(run_root / "manifest.json")
        if manifest["status"] != "completed":
            raise ValueError(f"unexpected run status: {manifest['status']}")
        dependencies = manifest["environment"].get("dependencies", {})
        if "placo" not in dependencies:
            raise ValueError("run manifest does not record the PlaCo dependency")

        expected = set(definition["artifact_requirements"])
        missing = [relative for relative in expected if not (run_root / relative).is_file()]
        if missing:
            raise ValueError(f"missing required E01 artifacts: {missing}")

        trace_path = (
            run_root
            / "arms"
            / "placo_v0_9_23"
            / "synthetic_two_link"
            / "trace.csv"
        )
        with trace_path.open(newline="", encoding="utf-8") as stream:
            trace = list(csv.DictReader(stream))
        if len(trace) < 2:
            raise ValueError("E01 trace does not contain a solver step")

        initial_error = float(trace[0]["position_error_m"])
        final_error = float(trace[-1]["position_error_m"])
        tolerance = definition["controlled_factors"]["position_tolerance_m"]
        if not final_error < initial_error:
            raise ValueError("PlaCo did not reduce the task error")
        if not final_error <= tolerance:
            raise ValueError(
                f"final error {final_error} exceeds declared tolerance {tolerance}"
            )

        with (run_root / "evaluation" / "metrics.csv").open(
            newline="", encoding="utf-8"
        ) as stream:
            metric_ids = {row["metric_id"] for row in csv.DictReader(stream)}
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

    print("E01 PlaCo smoke test and evidence bundle are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
