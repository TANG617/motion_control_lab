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
            "[--expect-exception|--fault-hold|--replay|--replay-start-paused]"
        )
    expect_exception = len(sys.argv) == 3 and sys.argv[2] == "--expect-exception"
    fault_hold = len(sys.argv) == 3 and sys.argv[2] == "--fault-hold"
    replay_controls = len(sys.argv) == 3 and sys.argv[2] in (
        "--replay",
        "--replay-start-paused",
    )
    replay_start_paused = (
        len(sys.argv) == 3 and sys.argv[2] == "--replay-start-paused"
    )
    if len(sys.argv) == 3 and not (expect_exception or fault_hold or replay_controls):
        raise RuntimeError(f"unknown argument: {sys.argv[2]}")

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(slave_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 74, 155, 0, 0))
    environment = os.environ.copy()
    environment["TERM"] = "xterm-256color"
    environment.pop("NO_COLOR", None)
    environment.setdefault("LC_ALL", "C.UTF-8")
    command = [sys.argv[1]]
    if expect_exception:
        command.append("--throw-after-render")
    elif fault_hold:
        command.append("--fault-hold")
    elif replay_controls:
        command.append(
            "--replay-start-paused" if replay_start_paused else "--replay"
        )
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
            if replay_controls and action_index == 0 and ready_elapsed >= 0.12:
                # Cartesian edit is disabled; arm selection and pause remain active.
                os.write(master_fd, b"w\x1b[C ")
                action_index += 1
            elif replay_controls and action_index == 1 and ready_elapsed >= 0.32:
                os.write(master_fd, b".")
                action_index += 1
            elif replay_controls and action_index == 2 and ready_elapsed >= 0.52:
                os.write(master_fd, b"2")
                action_index += 1
            elif replay_controls and action_index == 3 and ready_elapsed >= 0.72:
                os.write(master_fd, b"q")
                action_index += 1
            elif fault_hold and action_index == 0 and ready_elapsed >= 0.12:
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
            elif not expect_exception and not fault_hold and not replay_controls:
                actions = (
                    b"1", b"2", b"3", b"4", b"5", b"6", b"7",
                    b"\x1bOP", b"\x1bOQ", b"\x1bOR", b"\x1bOS",
                    b"\x1b[15~", b"\x1b[17~", b"\x1b[18~",
                    b"\t", b"\x1b[Z", b"?", b"?", b"3", b"5", b"\x1b[6~",
                    b"\x1b[6~", b"w\x1b[C\x1b[Adm0.020\rq 1x",
                )
                action_time = 0.12 + 0.10 * action_index
                if action_index < len(actions) and ready_elapsed >= action_time:
                    if action_index == 18:
                        fcntl.ioctl(
                            master_fd,
                            termios.TIOCSWINSZ,
                            struct.pack("HHHH", 24, 100, 0, 0),
                        )
                    elif action_index == 19:
                        fcntl.ioctl(
                            master_fd,
                            termios.TIOCSWINSZ,
                            struct.pack("HHHH", 18, 60, 0, 0),
                        )
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

        if replay_controls:
            expected_markers = (
                b"Replay timeline paused",
                b"Replay single-frame",
                b"publish sequence=7  replay frame=128/1732",
                b"publish sequence=7  replay frame=129/1732",
            )
            missing_markers = [
                marker for marker in expected_markers if marker not in output
            ]
            if missing_markers or return_code != 0:
                sys.stderr.buffer.write(output)
                sys.stderr.write(
                    "replay-control TUI validation failed: "
                    + ", ".join(marker.decode() for marker in missing_markers)
                    + f" (exit={return_code})\n"
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
            red_markers = [
                marker
                for marker in (
                    b"TARGET REJECTED",
                    b"FAULT HOLD",
                    b"rejected target revision",
                )
                if b"\x1b[31m\x1b[49m" + marker not in output
            ]
            if red_markers:
                sys.stderr.buffer.write(output)
                sys.stderr.write(
                    "fault states were not rendered in red: "
                    + ", ".join(marker.decode() for marker in red_markers)
                    + "\n"
                )
                return 1
            return 0

        expected_markers = (
            b"Overview",
            b"Cartesian Planning",
            b"Joint Planning",
            b"Solver and Quadratic Programming",
            b"Joint State",
            b"Runtime",
            b"Events",
            b"overview-bottom-marker",
            b"cartesian-bottom-marker",
            b"joint-plan-bottom-marker",
            b"solver-bottom-marker",
            b"joints-bottom-marker",
            b"runtime-bottom-marker",
            b"events-bottom-marker",
            b"Keyboard help",
            b"maximum hard violation",
            b"Executed joint state",
            b"head_yaw",
            b"left_arm_joint7",
            b"Processor affinity",
            b"bound",
            b"disabled",
            b"4101",
            b"requested processors",
            b"effective processors",
            b"IK calculation percentiles",
            b"90th percentile [ms]",
            b"95th percentile [ms]",
            b"99th percentile [ms]",
            b"0.160",
            b"Worker timing",
            b"release lateness",
            b"Non-Quadratic Programming",
            b"Attempts Accepted Rejected",
            b"Current state",
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

        if b"replay frame=" in output:
            sys.stderr.buffer.write(output)
            sys.stderr.write("teleop TUI unexpectedly rendered replay progress\n")
            return 1

        if "┌".encode() in output:
            sys.stderr.buffer.write(output)
            sys.stderr.write("compact tables rendered a nested square outer border\n")
            return 1

        if b"\x1b[36m\x1b[49m System " not in output:
            sys.stderr.buffer.write(output)
            sys.stderr.write("section title was not rendered in the accent color\n")
            return 1

        if return_code != 0:
            sys.stderr.buffer.write(output)
        return return_code
    finally:
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == "__main__":
    sys.exit(main())
