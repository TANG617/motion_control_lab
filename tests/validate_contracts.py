#!/usr/bin/env python3
"""Dependency-free checks for the lab's first JSON contracts."""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys
from typing import Any


EXPERIMENT_REQUIRED = {
    "schema_version",
    "experiment_id",
    "title",
    "question",
    "hypothesis",
    "independent_factors",
    "controlled_factors",
    "allowed_differences",
    "inputs",
    "arms",
    "evaluation_windows",
    "metrics",
    "comparisons",
    "failure_policy",
    "artifact_requirements",
}

MANIFEST_REQUIRED = {
    "schema_version",
    "run_id",
    "run_kind",
    "experiment_id",
    "status",
    "definition",
    "source_control",
    "environment",
    "inputs",
    "units",
    "failures",
    "outputs",
}

METRIC_ROLES = {"primary", "secondary", "guardrail", "diagnostic"}


def _load_json(path: pathlib.Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    _require(isinstance(value, dict), f"{path}: root must be an object")
    return value


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(64 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_child(root: pathlib.Path, relative: str) -> pathlib.Path:
    relative_path = pathlib.PurePosixPath(relative)
    _require(not relative_path.is_absolute(), f"absolute artifact path: {relative}")
    _require(".." not in relative_path.parts, f"escaping artifact path: {relative}")
    candidate = (root / pathlib.Path(*relative_path.parts)).resolve()
    _require(candidate.is_relative_to(root.resolve()), f"artifact escapes run: {relative}")
    return candidate


def validate_definition(path: pathlib.Path) -> dict[str, Any]:
    definition = _load_json(path)
    missing = EXPERIMENT_REQUIRED - definition.keys()
    _require(not missing, f"{path}: missing fields: {sorted(missing)}")
    _require(
        definition["schema_version"] == "experiment.v1",
        f"{path}: unsupported schema_version",
    )
    experiment_id = definition["experiment_id"]
    _require(
        isinstance(experiment_id, str)
        and len(experiment_id) == 3
        and experiment_id.startswith("E")
        and experiment_id[1:].isdigit(),
        f"{path}: invalid experiment_id",
    )

    for field in ("title", "question", "hypothesis"):
        _require(
            isinstance(definition[field], str) and definition[field].strip(),
            f"{path}: {field} must be non-empty",
        )

    inputs = definition["inputs"]
    arms = definition["arms"]
    _require(isinstance(inputs, list) and inputs, f"{path}: inputs must be non-empty")
    _require(isinstance(arms, list) and arms, f"{path}: arms must be non-empty")
    _require(
        len({item["input_id"] for item in inputs}) == len(inputs),
        f"{path}: duplicate input_id",
    )
    _require(
        len({item["arm_id"] for item in arms}) == len(arms),
        f"{path}: duplicate arm_id",
    )

    for item in inputs:
        _require(
            {"input_id", "artifact", "required"} <= item.keys(),
            f"{path}: incomplete input declaration",
        )
        artifact = _safe_child(path.parent, item["artifact"])
        _require(artifact.is_file(), f"{path}: missing input artifact {item['artifact']}")

    for arm in arms:
        _require(
            {"arm_id", "method", "required"} <= arm.keys(),
            f"{path}: incomplete arm declaration",
        )

    metrics = definition["metrics"]
    _require(isinstance(metrics, dict), f"{path}: metrics must be an object")
    _require(
        METRIC_ROLES <= metrics.keys(),
        f"{path}: all metric roles must be declared",
    )
    for role in METRIC_ROLES:
        _require(isinstance(metrics[role], list), f"{path}: metrics.{role} must be a list")

    requirements = definition["artifact_requirements"]
    _require(
        isinstance(requirements, list) and requirements,
        f"{path}: artifact_requirements must be non-empty",
    )
    for relative in requirements:
        _require(
            isinstance(relative, str)
            and relative
            and not pathlib.PurePosixPath(relative).is_absolute()
            and ".." not in pathlib.PurePosixPath(relative).parts,
            f"{path}: unsafe required artifact path",
        )
    return definition


def validate_manifest(path: pathlib.Path) -> dict[str, Any]:
    manifest = _load_json(path)
    missing = MANIFEST_REQUIRED - manifest.keys()
    _require(not missing, f"{path}: missing fields: {sorted(missing)}")
    _require(
        manifest["schema_version"] == "run_manifest.v1",
        f"{path}: unsupported schema_version",
    )
    _require(
        manifest["run_kind"] == "experiment",
        f"{path}: E01 must produce an experiment run",
    )
    _require(
        manifest["status"] in {"running", "completed", "failed"},
        f"{path}: invalid run status",
    )

    definition = manifest["definition"]
    _require(
        isinstance(definition, dict)
        and {"locator", "sha256", "resolved"} <= definition.keys(),
        f"{path}: incomplete definition provenance",
    )
    _require(
        len(definition["sha256"]) == 64,
        f"{path}: definition hash is not SHA-256",
    )
    _require(manifest["inputs"], f"{path}: no inputs")
    _require(manifest["units"], f"{path}: no execution units")

    run_root = path.parent
    outputs = manifest["outputs"]
    _require(isinstance(outputs, dict) and outputs, f"{path}: no output inventory")
    for relative, record in outputs.items():
        artifact = _safe_child(run_root, relative)
        _require(artifact.is_file(), f"{path}: missing output {relative}")
        _require(
            record.get("size_bytes") == artifact.stat().st_size,
            f"{path}: size mismatch for {relative}",
        )
        _require(
            record.get("sha256") == _sha256(artifact),
            f"{path}: hash mismatch for {relative}",
        )
    return manifest


def main(argv: list[str]) -> int:
    if len(argv) != 3 or argv[1] not in {"definition", "manifest"}:
        print(f"Usage: {argv[0]} definition|manifest PATH", file=sys.stderr)
        return 2

    path = pathlib.Path(argv[2]).resolve()
    if argv[1] == "definition":
        validate_definition(path)
    else:
        validate_manifest(path)
    print(f"valid {argv[1]}: {path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"contract validation failed: {error}", file=sys.stderr)
        raise SystemExit(1)

