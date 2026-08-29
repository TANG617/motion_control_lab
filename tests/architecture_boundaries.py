#!/usr/bin/env python3
"""Static dependency checks for package-internal component and app boundaries."""

from __future__ import annotations

import pathlib
import re
import sys


ROOT = pathlib.Path(sys.argv[1]).resolve()
APP_ROOT = ROOT / "apps"
COMPONENT_ROOT = ROOT / "components"
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
MCC_COMPONENT_PATTERNS = (
    re.compile(r"#\s*include\s*[<\"]motion_control_core/"),
    re.compile(r"\bmotion_control::core::(?:KinematicsSolverBuilder|HierarchicalKinematicsSolver|CartesianPlanner|JointPlanner)\b"),
    re.compile(r"\bmotion_control_core(?:::[A-Za-z0-9_]+)?\b"),
)
APP_SOURCE_FILES = {
    "baseline": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "loop.cpp", "loop.hpp"},
    "cartesian_planning": {"main.cpp", "options.cpp", "options.hpp", "planning.cpp", "planning.hpp", "loop.cpp", "loop.hpp"},
    "hierarchical_inverse_dynamics_torque_sim": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "loop.cpp", "loop.hpp"},
    "hierarchical_step": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "loop.cpp", "loop.hpp"},
    "planned_hierarchical_step": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "planning.cpp", "planning.hpp", "loop.cpp", "loop.hpp"},
    "planned_hierarchical_step_otg": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "planning.cpp", "planning.hpp", "telemetry.cpp", "telemetry.hpp", "loop.cpp", "loop.hpp"},
    "planned_hierarchical_step_otg_nullspace": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "nullspace.cpp", "nullspace.hpp", "planning.cpp", "planning.hpp", "rejection_policy.cpp", "rejection_policy.hpp", "telemetry.cpp", "telemetry.hpp", "loop.cpp", "loop.hpp"},
    "planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim": {"main.cpp", "options.cpp", "options.hpp", "admittance.cpp", "admittance.hpp", "admittance_options.hpp", "solver.cpp", "solver.hpp", "nullspace.cpp", "nullspace.hpp", "planning.cpp", "planning.hpp", "rejection_policy.cpp", "rejection_policy.hpp", "telemetry.cpp", "telemetry.hpp", "loop.cpp", "loop.hpp"},
    "plot_core_planning": {"main.cpp", "options.cpp", "options.hpp", "planning.cpp", "planning.hpp"},
    "replay_plan": {"main.cpp", "options.cpp", "options.hpp", "loop.cpp", "loop.hpp"},
    "step": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "loop.cpp", "loop.hpp"},
    "single_arm_step": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "loop.cpp", "loop.hpp"},
    "target": {"main.cpp", "options.cpp", "options.hpp", "solver.cpp", "solver.hpp", "loop.cpp", "loop.hpp"},
}
APP_MAIN_REQUIREMENTS = {
    "baseline": ("parseTeleopOptions", "parseReplayOptions", "BaselineSolver", "runLoop", "runReplayLoop"),
    "cartesian_planning": ("parseOptions", "CartesianPlanner", "planner.generate", "playTrajectory"),
    "hierarchical_inverse_dynamics_torque_sim": ("parseOptions", "configureSolver", "runLoop"),
    "hierarchical_step": ("parseOptions", "SolverRuntime", "configureSolver", "runLoop"),
    "planned_hierarchical_step": ("parseOptions", "SolverRuntime", "configureSolver", "CartesianPlanner", "runLoop"),
    "planned_hierarchical_step_otg": ("parseOptions", "SolverRuntime", "configureSolver", "CartesianPlanner", "JointPlanner", "runLoop"),
    "planned_hierarchical_step_otg_nullspace": ("parseOptions", "SolverRuntime", "configureSolver", "CartesianPlanner", "JointPlanner", "runLoop"),
    "planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim": ("parseOptions", "SolverRuntime", "configureSolver", "CartesianPlanner", "JointPlanner", "runLoop"),
    "plot_core_planning": ("parseOptions", "CartesianPlanner", "JointPlanner", "cartesian_planner.generate", "joint_planner.generate"),
    "replay_plan": ("parseOptions", "runLoop"),
    "step": ("parseOptions", "parseReplayOptions", "MccServoSolver", "PlacoServoSolver", "runLoop", "runReplayLoop"),
    "single_arm_step": ("parseOptions", "KinematicsSolverBuilder", "KinematicsSolver", "configureSolver", "builder.finalize", "runLoop"),
    "target": ("parseOptions", "MccTargetSolver", "PlacoTargetSolver", "runLoop"),
}


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

for path in COMPONENT_ROOT.rglob("*"):
    if not path.is_file() or (path.suffix not in TEXT_SUFFIXES and path.name != "CMakeLists.txt"):
        continue
    text = path.read_text(errors="replace")
    for pattern in MCC_COMPONENT_PATTERNS:
        if pattern.search(text):
            fail(f"shared component depends on MCC: {path.relative_to(ROOT)}")

if APP_NAMES != APP_SOURCE_FILES.keys():
    missing = sorted(APP_NAMES - APP_SOURCE_FILES.keys())
    stale = sorted(APP_SOURCE_FILES.keys() - APP_NAMES)
    fail(f"app source-shape registry mismatch: missing={missing}, stale={stale}")

for app, expected_sources in APP_SOURCE_FILES.items():
    app_dir = APP_ROOT / app
    actual_sources = {
        path.name
        for path in app_dir.iterdir()
        if path.is_file() and path.suffix in {".cpp", ".hpp"}
    }
    if actual_sources != expected_sources:
        missing = sorted(expected_sources - actual_sources)
        extra = sorted(actual_sources - expected_sources)
        fail(f"app source shape mismatch: {app}: missing={missing}, extra={extra}")

    main_text = (app_dir / "main.cpp").read_text(errors="replace")
    main_lines = len(main_text.splitlines())
    if not 15 <= main_lines <= 200:
        fail(f"app main is not a short composition root: {app}: {main_lines} lines")
    for required in APP_MAIN_REQUIREMENTS[app]:
        if required not in main_text:
            fail(f"app main hides required composition: {app}: {required}")
    for forbidden in (
        "struct ",
        "TuiDocument",
        "ReplayExecutionMetadata",
        "while (",
        "PlanningRequestVisualization",
    ):
        if forbidden in main_text:
            fail(f"app main contains runtime implementation: {app}: {forbidden.strip()}")

    cmake_text = (app_dir / "CMakeLists.txt").read_text(errors="replace")
    if f"mcl_{app}_support" not in cmake_text:
        fail(f"app-local support target is missing: {app}")
    for other in APP_NAMES - {app}:
        if f"mcl_{other}_support" in cmake_text or f"motion_control_lab::{other}_support" in cmake_text:
            fail(f"app support target links another app: {app} -> {other}")

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
