#!/usr/bin/env python3
"""Static dependency checks for package-internal component and app boundaries."""

from __future__ import annotations

import pathlib
import re
import sys


ROOT = pathlib.Path(sys.argv[1]).resolve()
APP_ROOT = ROOT / "apps"
APP_NAMES = {
    path.name
    for path in APP_ROOT.iterdir()
    if path.is_dir() and path.name not in {"common"}
}
FORBIDDEN_OLD = re.compile(
    "|".join(
        (
            "mcl_" + "interactive_runtime",
            "motion_control_lab::" + "interactive_runtime",
            "mcl_" + "app_common",
            "motion_control_lab::" + "app_common",
            "TuiConsole::" + "command",
            "Tui" + "SourceControls",
        )
    )
)
TEXT_SUFFIXES = {".cpp", ".hpp", ".h", ".cmake", ".txt"}


def fail(message: str) -> None:
    print(message, file=sys.stderr)
    raise SystemExit(1)


for app in sorted(APP_NAMES):
    app_dir = APP_ROOT / app
    for path in app_dir.rglob("*"):
        if not path.is_file() or (path.suffix not in TEXT_SUFFIXES and path.name != "CMakeLists.txt"):
            continue
        text = path.read_text(errors="replace")
        for other in APP_NAMES - {app}:
            if f"apps/{other}/" in text or f"apps/{other}" in text:
                fail(f"cross-app dependency: {path.relative_to(ROOT)} -> apps/{other}")

scaffold_text = (ROOT / "components/app_scaffold/app_scaffold.hpp").read_text()
for forbidden in ("motion_control_core", "solver", "task", "argc", "argv", "std::function", "virtual"):
    if forbidden in scaffold_text:
        fail(f"app scaffolding contains forbidden dependency/abstraction: {forbidden}")

for relative in ("CMakeLists.txt", "cmake", "adapters", "apps", "contracts", "components", "tests"):
    path = ROOT / relative
    candidates = [path] if path.is_file() else path.rglob("*")
    for candidate in candidates:
        if candidate.resolve() == pathlib.Path(__file__).resolve():
            continue
        if candidate.is_file() and FORBIDDEN_OLD.search(candidate.read_text(errors="replace")):
            fail(f"legacy boundary reference: {candidate.relative_to(ROOT)}")

print("app/component dependency boundaries are clean")
