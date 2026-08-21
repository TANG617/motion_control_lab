#!/usr/bin/env python3
"""Validate that E03 records an empty motion library as a complete failed run."""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import tempfile

LAB_ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(LAB_ROOT / "tests"))

from validate_contracts import validate_manifest  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True, type=pathlib.Path)
    parser.add_argument("--urdf", required=True, type=pathlib.Path)
    parser.add_argument("--work-root", required=True, type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.work_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="e03-empty-", dir=args.work_root) as temporary:
        root = pathlib.Path(temporary)
        library = root / "library"
        output = root / "runs"
        library.mkdir()
        completed = subprocess.run(
            [
                str(args.executable.resolve()),
                "--urdf",
                str(args.urdf.resolve()),
                "--library-dir",
                str(library),
                "--output-root",
                str(output),
                "--run-id",
                "empty-library-ctest",
                "--launcher",
                "ctest-empty-library",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode == 0:
            raise ValueError("empty motion library unexpectedly succeeded")

        run_root = output / "empty-library-ctest"
        manifest = validate_manifest(run_root / "manifest.json")
        if manifest["status"] != "failed":
            raise ValueError(f"unexpected run status: {manifest['status']}")
        if manifest.get("failure", {}).get("code") != "empty_motion_library":
            raise ValueError("empty motion library failure classification is missing")
        if manifest.get("invocation", {}).get("launcher") != "ctest-empty-library":
            raise ValueError("E03 launcher identity is missing")
        if "--library-dir" not in manifest.get("invocation", {}).get("argv", []):
            raise ValueError("E03 original argv is missing")
        resolved = manifest.get("resolved_config", {})
        if resolved.get("library_directory") != str(library.resolve()):
            raise ValueError("E03 resolved library directory is missing")
        if resolved.get("visualization_enabled") is not False:
            raise ValueError("E03 resolved visualization mode is missing")
        inventory = json.loads((run_root / "inputs" / "inventory.json").read_text())
        if inventory["actions"]:
            raise ValueError("empty library inventory contains actions")

    print("E03 empty-library failed run and manifest are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
