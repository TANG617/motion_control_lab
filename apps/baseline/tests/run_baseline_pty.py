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
    if len(sys.argv) != 3:
        raise RuntimeError("usage: run_baseline_pty.py <app> <urdf>")
    executable, urdf = sys.argv[1:]
    command = [
        executable,
        "teleop",
        "--urdf",
        urdf,
        "--port",
        str(free_port()),
        "--no-mcap",
    ]

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(slave_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 48, 200, 0, 0))
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
    ready_at = None
    solver_page_selected = False
    exit_requested = False
    deadline = time.monotonic() + 12.0
    try:
        while process.poll() is None and time.monotonic() < deadline:
            if ready_at is None and b"PlaCo Production-Static Baseline" in output:
                ready_at = time.monotonic()
            if ready_at is not None:
                elapsed = time.monotonic() - ready_at
                if not solver_page_selected and elapsed >= 0.25:
                    os.write(master_fd, b"2")
                    solver_page_selected = True
                elif solver_page_selected and not exit_requested and elapsed >= 1.0:
                    os.write(master_fd, b"x")
                    exit_requested = True

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
            raise RuntimeError("baseline TUI PTY test timed out")

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
        markers = (
            b"PlaCo Production-Static Baseline",
            b"source=42ed3ce3a19",
            b"Cartesian",
            b"IK status",
        )
        missing = [marker for marker in markers if marker not in output]
        if return_code != 0 or not terminal_restored or missing:
            sys.stderr.buffer.write(output)
            sys.stderr.write(
                "baseline PTY validation failed: "
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
