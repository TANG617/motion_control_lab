#!/usr/bin/env python3

import csv
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind(("127.0.0.1", 0))
        return int(server.getsockname()[1])


def run_once(executable: str, urdf: str, fixture: str, output: Path) -> None:
    subprocess.run(
        [
            executable,
            "replay",
            "--urdf",
            urdf,
            "--input",
            fixture,
            "--input-format",
            "csv",
            "--left-stream",
            "left",
            "--right-stream",
            "right",
            "--timestamp-source",
            "csv_timestamp",
            "--target-period-ms",
            "10",
            "--execution-mode",
            "batch",
            "--ui",
            "none",
            "--port",
            str(free_port()),
            "--no-mcap",
            "--output-dir",
            str(output),
        ],
        check=True,
        env=os.environ.copy(),
    )


def load_trace(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 2:
        raise RuntimeError(f"expected two trace rows, got {len(rows)}")
    return rows


def semantic_differences(
    first_rows: list[dict[str, str]], second_rows: list[dict[str, str]]
) -> list[str]:
    differences = []
    for row_index, (first_row, second_row) in enumerate(zip(first_rows, second_rows)):
        for key, first_value in first_row.items():
            if key == "solve_time_ms":
                continue
            second_value = second_row[key]
            if first_value != second_value:
                differences.append(
                    f"row {row_index} {key}: {first_value} != {second_value}"
                )
    return differences


def validate_artifacts(output: Path) -> tuple[list[dict[str, str]], str]:
    required = {
        "baseline_config.json",
        "manifest.json",
        "status.json",
        "trace.csv",
    }
    missing = [name for name in required if not (output / name).is_file()]
    if missing:
        raise RuntimeError(f"missing baseline artifacts: {missing}")

    config_bytes = (output / "baseline_config.json").read_bytes()
    config_sha256 = hashlib.sha256(config_bytes).hexdigest()
    config = json.loads(config_bytes)
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    status = json.loads((output / "status.json").read_text(encoding="utf-8"))
    if config["schema_version"] != "mcl.placo_baseline_config.v1":
        raise RuntimeError("unexpected baseline config schema")
    if len(config["tasks"]) != 18 or len(config["constraints"]) != 1:
        raise RuntimeError("baseline config registry is incomplete")
    if manifest["baseline"]["config_sha256"] != config_sha256:
        raise RuntimeError("manifest baseline config SHA-256 mismatch")
    if manifest["artifacts"]["baseline_config.json"]["sha256"] != config_sha256:
        raise RuntimeError("artifact baseline config SHA-256 mismatch")
    if status["state"] != "succeeded" or status["accepted_count"] != 2:
        raise RuntimeError("baseline replay did not succeed with two accepted solves")
    rows = load_trace(output / "trace.csv")
    for row in rows:
        if row["accepted"] != "true":
            raise RuntimeError("trace contains a rejected solve")
        if row["solver_status"] not in {"Converged", "Saturated", "BestEffort"}:
            raise RuntimeError("trace contains an unknown solver status")
        if len(row["positions"].split(";")) != 20:
            raise RuntimeError("trace does not contain the full 20-joint state")
    return rows, config_sha256


def main() -> int:
    if len(sys.argv) != 4:
        raise RuntimeError("usage: replay_determinism.py <app> <urdf> <fixture>")
    executable, urdf, fixture = sys.argv[1:]
    with tempfile.TemporaryDirectory(prefix="mcl-baseline-replay-") as temporary:
        root = Path(temporary)
        first = root / "first"
        second = root / "second"
        run_once(executable, urdf, fixture, first)
        run_once(executable, urdf, fixture, second)
        first_rows, first_sha = validate_artifacts(first)
        second_rows, second_sha = validate_artifacts(second)
        if first_sha != second_sha:
            raise RuntimeError("baseline config is not deterministic")
        differences = semantic_differences(first_rows, second_rows)
        if differences:
            raise RuntimeError(
                "accepted state/error/status trace is not deterministic: "
                + "; ".join(differences)
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
