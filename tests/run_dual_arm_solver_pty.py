#!/usr/bin/env python3

import fcntl
import os
import pty
import select
import socket
import struct
import subprocess
import sys
import termios
import time


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind(("127.0.0.1", 0))
        return int(server.getsockname()[1])


def main() -> int:
    if len(sys.argv) != 5:
        raise RuntimeError(
            "usage: run_dual_arm_solver_pty.py <app> <default|mcc|placo> "
            "<default|proxqp|eiquadprog> <urdf>"
        )

    executable, requested_solver, requested_backend, urdf = sys.argv[1:]
    if requested_solver not in ("default", "mcc", "placo"):
        raise RuntimeError(f"unsupported solver: {requested_solver}")
    if requested_backend not in ("default", "proxqp", "eiquadprog"):
        raise RuntimeError(f"unsupported backend: {requested_backend}")
    expected_solver = "mcc" if requested_solver == "default" else requested_solver
    effective_backend = (
        "proxqp"
        if expected_solver == "mcc" and requested_backend == "default"
        else requested_backend
    )
    if expected_solver == "placo":
        effective_backend = "eiquadprog"
    expected_label = (
        b"PlaCo/eiquadprog"
        if expected_solver == "placo"
        else (
            b"MCC/ProxQP"
            if effective_backend == "proxqp"
            else b"MCC/eiquadprog"
        )
    )
    expected_qp = (
        b"eiquadprog/solved"
        if expected_solver == "placo"
        else f"{effective_backend}/optimal".encode()
    )

    command = [
        executable,
        "--urdf",
        urdf,
        "--rate",
        "20",
        "--port",
        str(free_port()),
        "--no-mcap",
    ]
    if requested_solver != "default":
        command[1:1] = ["--solver", requested_solver]
    if requested_backend != "default":
        command[1:1] = ["--backend", requested_backend]

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(slave_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 45, 180, 0, 0))
    environment = os.environ.copy()
    environment["TERM"] = "xterm-256color"
    environment.setdefault("LC_ALL", "C.UTF-8")
    process = subprocess.Popen(
        command,
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        close_fds=True,
        env=environment,
        start_new_session=True,
    )

    output = bytearray()
    ui_ready_at = None
    action_index = 0
    deadline = time.monotonic() + 12.0
    try:
        while process.poll() is None and time.monotonic() < deadline:
            if ui_ready_at is None and expected_label in output and b"Cartesian" in output:
                ui_ready_at = time.monotonic()
            ready_elapsed = (
                time.monotonic() - ui_ready_at if ui_ready_at is not None else 0.0
            )
            if action_index == 0 and ready_elapsed >= 0.15:
                os.write(master_fd, b"w")
                action_index += 1
            elif action_index == 1 and ready_elapsed >= 0.55:
                os.write(master_fd, b"2")
                action_index += 1
            elif action_index == 2 and ready_elapsed >= 0.85:
                os.write(master_fd, b"4")
                action_index += 1
            elif action_index == 3 and ready_elapsed >= 1.15:
                os.write(master_fd, b"r1x")
                action_index += 1

            readable, _, _ = select.select([master_fd], [], [], 0.05)
            if readable:
                try:
                    output.extend(os.read(master_fd, 65536))
                except OSError:
                    break

        if process.poll() is None:
            process.kill()
            process.wait()
            sys.stderr.buffer.write(output)
            raise RuntimeError("dual-arm solver TUI PTY test timed out")

        return_code = process.wait()
        while True:
            readable, _, _ = select.select([master_fd], [], [], 0.05)
            if not readable:
                break
            try:
                output.extend(os.read(master_fd, 65536))
            except OSError:
                break

        local_flags = termios.tcgetattr(slave_fd)[3]
        terminal_restored = bool(local_flags & termios.ICANON) and bool(
            local_flags & termios.ECHO
        )
        expected_markers = (
            expected_label,
            expected_qp,
            b"IK solve-time percentiles",
            b"P90 [ms]",
            b"P95 [ms]",
            b"P99 [ms]",
            b"attempts/accepted/rejected",
            b"IK accepted",
        )
        missing = [marker for marker in expected_markers if marker not in output]
        if return_code != 0 or not terminal_restored or missing:
            sys.stderr.buffer.write(output)
            sys.stderr.write(
                "dual-arm solver PTY validation failed: "
                + ", ".join(marker.decode() for marker in missing)
                + f" (exit={return_code}, restored={terminal_restored})\n"
            )
            return 1
        return 0
    finally:
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == "__main__":
    raise SystemExit(main())
