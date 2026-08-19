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
