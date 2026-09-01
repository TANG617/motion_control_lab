"""Binary, install asset, environment, and execution-policy resolution."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shutil


ALLOWED_MCL_ENVIRONMENT = {
    "MCL_BINARY",
    "MCL_INSTALL_PREFIX",
    "MCL_LD_LIBRARY_PATH",
    "MCL_CPU_SET",
    "MCL_RT_PRIORITY",
}


def install_prefix() -> Path:
    return Path(os.environ.get("MCL_INSTALL_PREFIX", "/workspace/install/algorithm"))


@dataclass(frozen=True)
class InstallPath:
    relative_path: Path

    def __str__(self) -> str:
        return str(install_prefix() / self.relative_path)


def binary_path() -> str:
    explicit = os.environ.get("MCL_BINARY")
    if explicit:
        return explicit
    installed = install_prefix() / "bin" / "mcl_hierarchical_kinematics_step"
    if installed.exists():
        return str(installed)
    found = shutil.which("mcl_hierarchical_kinematics_step")
    return found or str(installed)


def mujoco_model_path() -> InstallPath:
    root = Path("share") / "motion-control-lab" / "robots" / "r1" / "mujoco"
    return InstallPath(root / "mjcf" / "r1.xml")


def command_prefix() -> list[str]:
    prefix: list[str] = []
    cpu_set = os.environ.get("MCL_CPU_SET", "5-7")
    rt_priority = os.environ.get("MCL_RT_PRIORITY", "60")
    if rt_priority:
        prefix.extend(("chrt", "--fifo", rt_priority))
    if cpu_set:
        prefix = ["taskset", "--cpu-list", cpu_set, *prefix]
    return prefix


def execution_environment() -> dict[str, str]:
    environment = os.environ.copy()
    for name in tuple(environment):
        if name.startswith("MCL_") and name not in ALLOWED_MCL_ENVIRONMENT:
            environment.pop(name)
    extra_library_path = environment.get("MCL_LD_LIBRARY_PATH")
    if extra_library_path:
        current = environment.get("LD_LIBRARY_PATH")
        environment["LD_LIBRARY_PATH"] = (
            extra_library_path if not current else f"{extra_library_path}:{current}"
        )
    return environment
