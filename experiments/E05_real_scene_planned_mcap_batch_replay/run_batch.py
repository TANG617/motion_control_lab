#!/usr/bin/env python3
"""Sequential planned MCAP replay, with a PTY and append-only batch evidence."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import csv
from datetime import datetime, timezone
import errno
import fnmatch
import hashlib
import json
import os
from pathlib import Path
import pty
import select
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import termios
import time
import tty
import uuid

ROOT = Path(__file__).resolve().parent
LAB = ROOT.parents[1]
LAUNCHER = LAB / "apps/hierarchical_kinematics_step/scripts/profiles/planned/run_mcap_interactive.py"
OWNED = (
    "--input", "--input-format", "--output-dir", "--output-root", "--run-id",
    "--mcap", "--no-mcap", "--dump-resolved-options",
)
FIELDS = (
    "index", "input", "state", "native_state", "returncode", "elapsed_s",
    "source_frame_count", "consumed_frame_count", "dropped_frame_count",
    "accepted_count", "rejected_count", "deadline_miss_count",
    "maximum_accepted_joint_violation", "maximum_accepted_scaled_residual_l2",
    "final_reference_position_error_left_m", "final_reference_position_error_right_m",
    "final_reference_orientation_error_left_rad", "final_reference_orientation_error_right_rad",
    "red_solver_p95_ms", "red_solver_p99_ms", "red_solver_max_ms",
    "red_deadline_miss_count", "red_skipped_release_count", "input_sha256",
    "artifact_dir", "error",
)


def write_json(path: Path, value: object) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    split = argv.index("--") if "--" in argv else len(argv)
    extra = argv[split + 1:]
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--dataset-dir", type=Path, default=Path("/workspace/fixtures/raw/batch"))
    parser.add_argument("--include", action="append", default=[])
    parser.add_argument("--exclude", action="append", default=[])
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--output-root", type=Path, default=ROOT / "runs")
    parser.add_argument("--run-id")
    parser.add_argument("--dry-run", action="store_true")
    options = parser.parse_args(argv[:split])
    if options.limit is not None and options.limit < 1:
        parser.error("--limit must be positive")
    if options.run_id and (Path(options.run_id).name != options.run_id or options.run_id in (".", "..")):
        parser.error("--run-id must be one directory name")
    # The native argparse launcher accepts abbreviations, so reserve those too.
    for argument in extra:
        key = argument.split("=", 1)[0]
        if key.startswith("--") and any(flag.startswith(key) for flag in OWNED):
            parser.error(f"{key} is managed by the batch tool")
    options.dataset_dir = options.dataset_dir.resolve()
    options.output_root = options.output_root.resolve()
    return options, extra


def discover(options: argparse.Namespace) -> list[Path]:
    if not options.dataset_dir.is_dir():
        raise ValueError(f"dataset directory does not exist: {options.dataset_dir}")
    candidates = options.dataset_dir.rglob("*.mcap") if options.recursive else options.dataset_dir.glob("*.mcap")
    includes = options.include or ["*.mcap"]
    paths = sorted(
        (path for path in candidates if path.is_file()
         and any(fnmatch.fnmatchcase(path.relative_to(options.dataset_dir).as_posix(), pattern) for pattern in includes)
         and not any(fnmatch.fnmatchcase(path.relative_to(options.dataset_dir).as_posix(), pattern) for pattern in options.exclude)),
        key=lambda path: path.relative_to(options.dataset_dir).as_posix(),
    )
    paths = paths[:options.limit]
    if not paths:
        raise ValueError("no MCAP files selected")
    return paths


def command_for(source: Path, item_dir: Path, extra: list[str]) -> list[str]:
    return [sys.executable, str(LAUNCHER), *extra,
            "--input", str(source), "--input-format", "mcap",
            "--output-dir", str(item_dir / "replay"),
            "--mcap", str(item_dir / "replay/visualization.mcap"),
            "--no-start-paused", "--replay-trace", "--replay-exit-on-fault"]


class TerminalRunner:
    """Own the child process group; preserve terminal state on every exit path."""

    def __init__(self) -> None:
        self.interrupted = False
        self.process: subprocess.Popen | None = None

    def interrupt(self, _signal: int, _frame: object) -> None:
        self.interrupted = True

    def run(self, command: list[str], log: Path) -> tuple[int, float]:
        started = time.monotonic()
        master, slave = pty.openpty()
        interactive = sys.stdin.isatty() and sys.stdout.isatty()
        saved = termios.tcgetattr(sys.stdin.fileno()) if interactive else None
        old_handlers = {sig: signal.signal(sig, self.interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        cancel_at = None
        terminating = False
        previous_size = None
        try:
            size = (os.get_terminal_size(sys.stdout.fileno()) if interactive else os.terminal_size((160, 48)))
            fcntl_size(slave, size)
            environment = os.environ.copy()
            environment.setdefault("TERM", "xterm-256color")
            self.process = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave,
                                            start_new_session=True, close_fds=True, env=environment)
            os.close(slave)
            slave = -1
            if interactive:
                tty.setraw(sys.stdin.fileno())
            with log.open("wb") as output:
                while True:
                    now = time.monotonic()
                    if self.interrupted and cancel_at is None:
                        # Native replay's x key exits through artifact finalization.
                        os.write(master, b"x")
                        cancel_at = now
                    if cancel_at is not None and self.process.poll() is None:
                        if now - cancel_at > 12:
                            os.killpg(self.process.pid, signal.SIGKILL)
                        elif now - cancel_at > 10 and not terminating:
                            os.killpg(self.process.pid, signal.SIGTERM)
                            terminating = True
                    if interactive:
                        size = os.get_terminal_size(sys.stdout.fileno())
                        if size != previous_size:
                            fcntl_size(master, size)
                            previous_size = size
                    readers = [master]
                    if interactive and not self.interrupted:
                        readers.append(sys.stdin.fileno())
                    ready, _, _ = select.select(readers, [], [], 0.05)
                    if interactive and sys.stdin.fileno() in ready:
                        data = os.read(sys.stdin.fileno(), 4096)
                        if not data or b"\x03" in data:
                            self.interrupted = True
                        else:
                            os.write(master, data)
                    if master in ready:
                        try:
                            data = os.read(master, 65536)
                        except OSError as error:
                            if error.errno != errno.EIO:
                                raise
                            data = b""
                        if not data:
                            break
                        output.write(data)
                        output.flush()
                        sys.stdout.buffer.write(data)
                        sys.stdout.buffer.flush()
                    elif self.process.poll() is not None:
                        break
            return self.process.wait(), time.monotonic() - started
        finally:
            if self.process is not None and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
            self.process = None
            os.close(master)
            if slave >= 0:
                os.close(slave)
            if saved is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, saved)
                sys.stdout.write("\x1b[?25h\x1b[?1049l")
                sys.stdout.flush()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)


def fcntl_size(fd: int, size: os.terminal_size) -> None:
    import fcntl
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", size.lines, size.columns, 0, 0))


@contextmanager
def batch_signals(runner: TerminalRunner):
    previous = {sig: signal.signal(sig, runner.interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def read_native(path: Path) -> tuple[dict, str]:
    if not path.is_file():
        return {}, f"missing {path.name}"
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        return {}, f"cannot read {path.name}: {error}"
    if not isinstance(value, dict):
        return {}, f"invalid {path.name}: expected object"
    return value, ""


def collect(item: dict, item_dir: Path, returncode: int, elapsed: float) -> dict:
    status, status_error = read_native(item_dir / "replay/status.json")
    native, manifest_error = read_native(item_dir / "replay/manifest.json")
    errors = [error for error in (status_error, manifest_error, status.get("error", "")) if error]
    for name in ("trace.csv", "visualization.mcap"):
        path = item_dir / "replay" / name
        if not path.is_file() or path.stat().st_size == 0:
            errors.append(f"missing or empty {name}")
    native_state = status.get("state")
    state = "succeeded" if returncode == 0 and native_state == "succeeded" and not errors else "failed"
    if native_state == "stopped":
        state = "stopped"
    elif not status and status_error.startswith("missing"):
        state = "missing_result"
    if returncode:
        errors.append(f"process exit code {returncode}; see console.log for original error")
    row = {field: None for field in FIELDS}
    row.update({"index": item["index"], "input": item["input"], "state": state,
                "native_state": native_state, "returncode": returncode,
                "elapsed_s": round(elapsed, 6), "artifact_dir": item["artifact_dir"],
                "error": "; ".join(errors), "input_sha256": native.get("input", {}).get("sha256")})
    for key in ("source_frame_count", "consumed_frame_count", "dropped_frame_count",
                "accepted_count", "rejected_count", "deadline_miss_count"):
        if key in status:
            row[key] = status[key]
    for key in FIELDS:
        if key in native.get("tracking", {}):
            row[key] = native["tracking"][key]
    # Keep native nested statistics verbatim as well as the comparable CSV columns.
    row["tracking"] = native.get("tracking")
    row["red_timing"] = native.get("red_timing")
    timing = native.get("red_timing", {})
    for column, key in (("p95_ms", "p95_ms_upper_bound"), ("p99_ms", "p99_ms_upper_bound"), ("max_ms", "maximum_ms")):
        row["red_solver_" + column] = timing.get("solver", {}).get(key)
    for key in ("deadline_miss_count", "skipped_release_count"):
        row["red_" + key] = timing.get(key)
    return row


def source_control() -> dict:
    return {"revision": subprocess.check_output(["git", "-C", str(LAB), "rev-parse", "HEAD"], text=True).strip(),
            "dirty": bool(subprocess.check_output(["git", "-C", str(LAB), "status", "--porcelain"], text=True))}


def save_batch(batch: Path, manifest: dict, rows: list[dict]) -> None:
    failures = [row for row in rows if row["state"] not in ("succeeded", "not_run", "running")]
    manifest["units"] = rows
    manifest["failures"] = {"total": len(failures), "required": len(failures)}
    write_json(batch / "manifest.json", manifest)
    write_json(batch / "evaluation/failures.json", failures)
    with (batch / "evaluation/action_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    counts = {state: sum(row["state"] == state for row in rows)
              for state in ("succeeded", "failed", "missing_result", "stopped", "interrupted", "not_run", "running")}
    report = ["# E05 planned MCAP batch replay", "", f"Batch: `{batch.name}`; status: **{manifest['status']}**", "",
              ", ".join(f"{key}: {value}" for key, value in counts.items()), "",
              "Success follows native replay status; it does not certify tracking accuracy or realtime deadlines.", "",
              "| Input | State | Exit | Artifacts |", "|---|---|---:|---|"]
    for row in rows:
        label = row["input"].replace("|", "\\|")
        report.append(f"| {label} | {row['state']} | {row.get('returncode', '')} | [{row['index']}]({ '../' + row['artifact_dir'] }) |")
    (batch / "evaluation/report.md").write_text("\n".join(report) + "\n")


def execute_items(batch: Path, manifest: dict, items: list[dict], rows: list[dict], runner: TerminalRunner) -> None:
    for position, item in enumerate(items):
        if runner.interrupted:
            break
        item_dir = batch / item["artifact_dir"]
        item_dir.mkdir()
        write_json(item_dir / "command.json", item["command"])
        rows[position]["state"] = "running"
        save_batch(batch, manifest, rows)
        print(f"\n[{position + 1}/{len(items)}] {item['input']}\nArtifacts: {item_dir}", flush=True)
        returncode, elapsed = runner.run(item["command"], item_dir / "console.log")
        row = collect(item, item_dir, returncode, elapsed)
        if runner.interrupted:
            row["state"] = "interrupted"
        rows[position] = row
        write_json(item_dir / "result.json", row)
        save_batch(batch, manifest, rows)
        print(f"\n[{position + 1}/{len(items)}] {row['state']} (exit {returncode}, {elapsed:.2f}s)", flush=True)
        if runner.interrupted or row["state"] == "stopped":
            runner.interrupted = True
            break


def main(argv: list[str] | None = None) -> int:
    options, extra = parse_args(sys.argv[1:] if argv is None else argv)
    paths = discover(options)
    run_id = options.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:8]
    batch = options.output_root / run_id
    items = []
    for index, source in enumerate(paths, 1):
        relative = source.relative_to(options.dataset_dir).as_posix()
        artifact_dir = f"items/{index:03d}_{source.stem}"
        info = source.stat()
        items.append({"index": index, "input": relative, "path": str(source),
                      "size_bytes": info.st_size, "mtime_ns": info.st_mtime_ns,
                      "artifact_dir": artifact_dir,
                      "command": command_for(source, batch / artifact_dir, extra)})
    if options.dry_run:
        for item in items:
            print(f"[{item['index']}/{len(items)}] {item['input']}\n{shlex.join(item['command'])}")
        return 0
    batch.mkdir(parents=True, exist_ok=False)
    for name in ("definition", "inputs", "evaluation", "items"):
        (batch / name).mkdir()
    definition = json.loads((ROOT / "definition.json").read_text())
    installed = Path(os.environ.get("MCL_INSTALL_PREFIX", "/workspace/install/algorithm")) / "bin/mcl_hierarchical_kinematics_step"
    executable = Path(os.environ.get("MCL_BINARY") or (str(installed) if installed.exists() else
                      shutil.which("mcl_hierarchical_kinematics_step") or str(installed)))
    config = {key: str(value) if isinstance(value, Path) else value for key, value in vars(options).items()}
    config.update({"launcher": str(LAUNCHER), "launcher_sha256": sha256(LAUNCHER),
                   "tool_sha256": sha256(Path(__file__)), "extra_argv": extra,
                   "managed_arguments_take_precedence": True})
    write_json(batch / "definition/resolved.json", definition)
    write_json(batch / "definition/batch_options.json", config)
    write_json(batch / "inputs/inventory.json", items)
    manifest = {"schema_version": "run_manifest.v1", "run_id": run_id, "run_kind": "experiment",
                "experiment_id": "E05", "status": "running",
                "definition": {"locator": str(ROOT / "definition.json"), "sha256": sha256(ROOT / "definition.json"), "resolved": definition},
                "source_control": source_control(),
                "environment": {"runtime": sys.version, "platform": sys.platform,
                                "binary": str(executable), "binary_sha256": sha256(executable) if executable.is_file() else None,
                                "mcl": {key: os.environ[key] for key in ("MCL_BINARY", "MCL_INSTALL_PREFIX", "MCL_LD_LIBRARY_PATH", "MCL_CPU_SET", "MCL_RT_PRIORITY") if key in os.environ}},
                "inputs": items, "units": [], "failures": {"total": 0, "required": 0},
                "outputs": {}, "started_at": datetime.now(timezone.utc).isoformat()}
    rows = [{key: item[key] for key in ("index", "input", "artifact_dir")} | {"state": "not_run"} for item in items]
    runner = TerminalRunner()
    save_batch(batch, manifest, rows)
    with batch_signals(runner):
        execute_items(batch, manifest, items, rows, runner)
    result = 130 if runner.interrupted else (0 if all(row["state"] == "succeeded" for row in rows) else 1)
    manifest["status"] = "completed" if result == 0 else "failed"
    manifest["interrupted"] = runner.interrupted
    manifest["returncode"] = result
    manifest["finished_at"] = datetime.now(timezone.utc).isoformat()
    save_batch(batch, manifest, rows)
    # Hash all evidence except the self-referential batch manifest.
    manifest["outputs"] = {path.relative_to(batch).as_posix(): {"sha256": sha256(path), "size_bytes": path.stat().st_size}
                           for path in sorted(batch.rglob("*")) if path.is_file() and path != batch / "manifest.json"}
    write_json(batch / "manifest.json", manifest)
    print(f"Batch exit {result}. Report: {batch / 'evaluation/report.md'}", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
