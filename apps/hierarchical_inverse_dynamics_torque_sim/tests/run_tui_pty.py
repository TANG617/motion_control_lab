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
    if len(sys.argv) != 4:
        raise RuntimeError("usage: run_tui_pty.py <binary> <urdf> <mjcf>")

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(slave_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 80, 0, 0))
    environment = os.environ.copy()
    environment["TERM"] = "xterm-256color"
    environment.setdefault("LC_ALL", "C.UTF-8")
    command = [
        sys.argv[1],
        "teleop",
        "--duration",
        "0",
        "--retarget-x",
        "0",
        "--ui",
        "tui",
        "--viz",
        "none",
        "--no-mujoco-viewer",
        "--urdf",
        sys.argv[2],
        "--mujoco-model",
        sys.argv[3],
    ]
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
    actions = (b"1", b"2", b"3", b"4", b"5", b"6", b"7", b"?", b"?", b"x")
    deadline = time.monotonic() + 15.0
    try:
        while process.poll() is None and time.monotonic() < deadline:
            if ready_at is None and b"1 Monitor" in output:
                ready_at = time.monotonic()
            if ready_at is not None and action_index < len(actions):
                if time.monotonic() - ready_at >= 0.12 + 0.10 * action_index:
                    os.write(master_fd, actions[action_index])
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
            raise RuntimeError("torque-sim TUI PTY test timed out")

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
        expected = (
            b"Hierarchical Inverse Dynamics Torque",
            b"Monitor",
            b"Motion",
            b"Solver",
            b"Requirements",
            b"Joints",
            b"System",
            b"Dynamics",
            b"Keyboard help",
            b"HID",
            b"tertiary-posture",
            b"Torque",
            b"ticks=",
            b"\x1b[?1049h",
            b"\x1b[?1049l",
        )
        missing = [marker for marker in expected if marker not in output]
        forbidden = (b"terminal-minimum-motion",)
        unexpected = [marker for marker in forbidden if marker in output]
        if return_code != 0 or not terminal_restored or missing or unexpected:
            sys.stderr.buffer.write(output)
            sys.stderr.write(
                "torque-sim TUI contract failed: "
                + ", ".join(marker.decode(errors="replace") for marker in missing)
                + (
                    "; unexpected: "
                    + ", ".join(
                        marker.decode(errors="replace") for marker in unexpected
                    )
                    if unexpected
                    else ""
                )
                + f" (exit={return_code}, restored={terminal_restored})\n"
            )
            return 1
        return 0
    finally:
        os.close(master_fd)
        os.close(slave_fd)


if __name__ == "__main__":
    raise SystemExit(main())
