#!/usr/bin/env python3

import fcntl
import os
import pty
import select
import struct
import subprocess
import sys
import termios
import time


def main() -> int:
    if len(sys.argv) not in (4, 6):
        raise RuntimeError(
            "usage: run_planned_tui_pages_pty.py <app> <5|6> <urdf> "
            "[--profile <profile>]"
        )
    executable, page_count_text, urdf = sys.argv[1:4]
    profile_arguments = sys.argv[4:]
    page_count = int(page_count_text)
    if page_count not in (5, 6):
        raise RuntimeError(f"unsupported page count: {page_count}")

    command = [
        executable,
        *profile_arguments,
        "teleop",
        "--urdf",
        urdf,
        "--red-rate",
        "100",
        "--yellow-rate",
        "20",
        "--ui-rate",
        "10",
        "--deadline-policy",
        "monitor",
        "--ui",
        "tui",
        "--viz",
        "none",
        "--no-mcap",
    ]

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(
        slave_fd,
        termios.TIOCSWINSZ,
        struct.pack("HHHH", 72, 160, 0, 0),
    )
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
    action_index = 0
    actions = tuple(("input", str(page).encode()) for page in range(1, page_count + 1)) + (
        ("input", b"\t"),
        ("input", b"\x1b[Z"),
    )
    if page_count == 6:
        actions += (
            ("input", b"6"),
            ("input", b"\x1b"),
        )
    else:
        actions += (
            ("input", b"1"),
            ("input", b"2"),
            ("input", b"x"),
        )
    deadline = time.monotonic() + 15.0
    try:
        while process.poll() is None and time.monotonic() < deadline:
            if ready_at is None and b"TCP tracking" in output:
                ready_at = time.monotonic()
            if ready_at is not None and action_index < len(actions):
                elapsed = time.monotonic() - ready_at
                if elapsed >= 0.15 + 0.20 * action_index:
                    action, payload = actions[action_index]
                    if action == "input":
                        os.write(master_fd, payload)
                    else:
                        rows, columns = payload
                        fcntl.ioctl(
                            master_fd,
                            termios.TIOCSWINSZ,
                            struct.pack("HHHH", rows, columns, 0, 0),
                        )
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
            raise RuntimeError("planned TUI PTY test timed out")

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
        expected_markers = [
            b"Monitor",
            b"Motion",
            b"Solver",
            b"Joints",
            b"System",
            b"TCP tracking",
            b"Tracking error",
            b"Worker runtime",
            b"Safety and hold",
            b"Cartesian states",
            b"Passes",
            b"Last-iterate violations",
            b"Processor affinity",
            b"Collision pairs",
        ]
        if page_count == 6:
            expected_markers.extend(
                [
                    b"Null-space",
                    b"Control",
                    b"Tertiary objectives",
                    b"Tertiary",
                    b"Terminal",
                    b"held",
                ]
            )
        missing = [marker for marker in expected_markers if marker not in output]
        actions_completed = action_index == len(actions)
        if return_code != 0 or not terminal_restored or missing or not actions_completed:
            sys.stderr.buffer.write(output)
            sys.stderr.write(
                "planned TUI PTY validation failed: "
                + ", ".join(marker.decode() for marker in missing)
                + f" (exit={return_code}, restored={terminal_restored}, "
                + f"actions={action_index}/{len(actions)})\n"
            )
            return 1
        if "┌".encode() in output:
            sys.stderr.buffer.write(output)
            sys.stderr.write("compact tables rendered a nested square outer border\n")
            return 1
        return 0
    finally:
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == "__main__":
    raise SystemExit(main())
