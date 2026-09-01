"""Build and execute native commands for one fixed profile recipe."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import NoReturn, Sequence

from .arguments import BOOL_FLAGS, REPEAT_FLAGS, REPLAY_BOOL_FLAGS, create_parser
from .environment import binary_path, command_prefix, execution_environment
from .recipe import Recipe


def _append_flag(command: list[str], flag: str, value: object) -> None:
    if flag in BOOL_FLAGS or flag in REPLAY_BOOL_FLAGS:
        command.append(f"--{flag}" if bool(value) else f"--no-{flag}")
    elif flag == "mcap" and value is None:
        command.append("--no-mcap")
    elif flag in REPEAT_FLAGS:
        for item in value if isinstance(value, list) else [value]:
            command.extend((f"--{flag}", str(item)))
    else:
        command.extend((f"--{flag}", str(value)))


def build_recipe_command(
    recipe: Recipe,
    argv: Sequence[str],
    launcher_path: str | Path,
) -> list[str]:
    """Return the exact native command for a fixed profile/source recipe."""
    launcher = Path(launcher_path).resolve()
    namespace = vars(create_parser(recipe.source, launcher.name).parse_args(list(argv)))
    resolved = dict(recipe.options)
    for name, value in namespace.items():
        resolved[name.replace("_", "-")] = value
    if "output_dir" in namespace:
        resolved.pop("output-root", None)
    if "output_root" in namespace:
        resolved.pop("output-dir", None)
    if namespace.get("pairing_policy") == "exact":
        resolved.pop("nearest-tolerance-ms", None)
    if recipe.source != "teleop" and not resolved.get("input"):
        raise SystemExit(f"{launcher.name}: error: --input is required")

    native_source = "teleop" if recipe.source == "teleop" else "replay"
    command = [binary_path(), "--profile", recipe.profile, native_source]
    dump = bool(resolved.pop("dump-resolved-options", False))
    resolved["launcher-argv-json"] = json.dumps(
        {
            "recipe": recipe.name,
            "profile": recipe.profile,
            "launcher": str(launcher),
            "argv": list(argv),
        },
        separators=(",", ":"),
    )
    if native_source == "replay":
        resolved["launcher"] = str(launcher)
    for flag, value in resolved.items():
        _append_flag(command, flag, value)
    if dump:
        command.append("--dump-resolved-options")
    return [*command_prefix(), *command]


def run_recipe(
    recipe: Recipe,
    argv: Sequence[str],
    launcher_path: str | Path,
) -> NoReturn:
    command = build_recipe_command(recipe, argv, launcher_path)
    os.execvpe(command[0], command, execution_environment())
