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
    if len(sys.argv) not in (2, 3):
        raise RuntimeError(
            "usage: run_tui_pty.py <test-executable> "
            "[--expect-exception|--fault-hold]"
        )
    expect_exception = len(sys.argv) == 3 and sys.argv[2] == "--expect-exception"
    fault_hold = len(sys.argv) == 3 and sys.argv[2] == "--fault-hold"
    if len(sys.argv) == 3 and not (expect_exception or fault_hold):
        raise RuntimeError(f"unknown argument: {sys.argv[2]}")

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(slave_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 45, 180, 0, 0))
    environment = os.environ.copy()
    environment["TERM"] = "xterm-256color"
    environment.setdefault("LC_ALL", "C.UTF-8")
    command = [sys.argv[1]]
    if expect_exception:
        command.append("--throw-after-render")
    elif fault_hold:
        command.append("--fault-hold")
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
    started_at = time.monotonic()
    ui_ready_at = None
    action_index = 0
    deadline = started_at + 10.0
    try:
        while process.poll() is None and time.monotonic() < deadline:
            ready_marker = b"FAULT HOLD" if fault_hold else b"Cartesian"
            if not expect_exception and ui_ready_at is None and ready_marker in output:
                ui_ready_at = time.monotonic()
            ready_elapsed = (
                time.monotonic() - ui_ready_at if ui_ready_at is not None else 0.0
            )
            if fault_hold and action_index == 0 and ready_elapsed >= 0.12:
                # Motion/reset/pause/step controls must be ignored. Arm selection,
                # Runtime page navigation, and exit remain available.
                os.write(master_fd, b"w\x1b[C\x1b[Am0.020\rr i4")
                action_index += 1
            elif fault_hold and action_index == 1 and ready_elapsed >= 0.42:
                os.write(master_fd, b"2")
                action_index += 1
            elif fault_hold and action_index == 2 and ready_elapsed >= 0.72:
                os.write(master_fd, b"x")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 0 and ready_elapsed >= 0.12:
                os.write(master_fd, b"2")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 1 and ready_elapsed >= 0.32:
                fcntl.ioctl(
                    master_fd,
                    termios.TIOCSWINSZ,
                    struct.pack("HHHH", 24, 100, 0, 0),
                )
                os.write(master_fd, b"3")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 2 and ready_elapsed >= 0.52:
                fcntl.ioctl(
                    master_fd,
                    termios.TIOCSWINSZ,
                    struct.pack("HHHH", 18, 60, 0, 0),
                )
                os.write(master_fd, b"4")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 3 and ready_elapsed >= 0.72:
                os.write(master_fd, b"\x1b[6~")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 4 and ready_elapsed >= 0.92:
                os.write(master_fd, b"\x1b[6~")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 5 and ready_elapsed >= 1.12:
                os.write(master_fd, b"5")
                action_index += 1
            elif not expect_exception and not fault_hold and action_index == 6 and ready_elapsed >= 1.32:
                # Move left +x, select right, double the step, move right -y,
                # enter a manual 0.020 m step, move right +z, pause, return to
                # Overview, and exit.
                os.write(master_fd, b"w\x1b[C\x1b[Adm0.020\rq 1x")
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
            raise RuntimeError("TUI PTY test timed out")

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
        if not terminal_restored:
            sys.stderr.write("TUI did not restore canonical echo mode\n")
            return_code = return_code or 1

        if expect_exception:
            expected_message = b"test_tui_console: intentional TUI exception"
            restore_sequence = b"\x1b[?1049l"
            restore_index = output.rfind(restore_sequence)
            message_index = output.rfind(expected_message)
            passed = (
                return_code == 1
                and terminal_restored
                and restore_index >= 0
                and message_index > restore_index
            )
            if not passed:
                sys.stderr.buffer.write(output)
                sys.stderr.write(
                    "expected failure text after alternate-screen restoration "
                    f"(exit={return_code}, restore={restore_index}, "
                    f"message={message_index})\n"
                )
                return 1
            return 0

        if fault_hold:
            expected_markers = (
                b"TARGET REJECTED",
                b"FAULT HOLD",
                b"rejected target revision",
                b"recoverable rejects",
                b"maximum hard violation",
            )
            missing_markers = [
                marker for marker in expected_markers if marker not in output
            ]
            if missing_markers or return_code != 0:
                sys.stderr.buffer.write(output)
                sys.stderr.write(
                    "fault-hold TUI validation failed: "
                    + ", ".join(marker.decode() for marker in missing_markers)
                    + f" (exit={return_code})\n"
                )
                return 1
            return 0

        expected_markers = (
            b"Cartesian",
            b"maximum hard violation",
            b"All joint state",
            b"head_yaw",
            b"J7",
            b"CPU affinity",
            b"bound",
            b"disabled",
            b"4101",
            b"requested CPUs",
            b"effective CPUs",
            b"IK solve-time percentiles",
            b"P90 [ms]",
            b"P95 [ms]",
            b"P99 [ms]",
            b"0.160",
            b"Periodic workers",
            b"sched delay",
            b"IK non-QP",
            b"attempts/accepted/rejected",
            b"Recent state changes",
        )
        missing_markers = [marker for marker in expected_markers if marker not in output]
        if missing_markers:
            sys.stderr.buffer.write(output)
            sys.stderr.write(
                "TUI did not render all data-driven pages: "
                + ", ".join(marker.decode() for marker in missing_markers)
                + "\n"
            )
            return 1

        if return_code != 0:
            sys.stderr.buffer.write(output)
        return return_code
    finally:
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == "__main__":
    sys.exit(main())
