#!/usr/bin/env python3
"""Controllable native executable for process/PTY contract tests."""
import hashlib
import json
import os
from pathlib import Path
import select
import sys
import termios


def value(flag):
    return sys.argv[sys.argv.index(flag) + 1]


source = Path(value("--input"))
output = Path(value("--output-dir"))
assert os.isatty(0) and os.isatty(1), "expected PTY"
assert "--no-start-paused" in sys.argv
assert "--replay-exit-on-fault" in sys.argv
assert "--replay-trace" in sys.argv
if "--red-rate" in sys.argv:
    assert float(value("--red-rate")) > 0
output.mkdir(parents=True, exist_ok=False)
(output.parent / "child.pid").write_text(str(os.getpid()))
if "missing" in source.stem:
    print("ORIGINAL STARTUP FAILURE", flush=True)
    sys.exit(7)
state = "succeeded"
if "failed" in source.stem:
    state = "failed"
if "stopped" in source.stem:
    state = "stopped"
if "wait" in source.stem:
    original = termios.tcgetattr(0)
    raw = termios.tcgetattr(0)
    raw[3] &= ~(termios.ICANON | termios.ECHO)
    termios.tcsetattr(0, termios.TCSANOW, raw)
    print("READY FOR STOP", flush=True)
    while True:
        readable, _, _ = select.select([0], [], [], 20)
        if not readable:
            raise RuntimeError("test did not stop child")
        keys = os.read(0, 128)
        if b"w" in keys:
            (output.parent / "window.json").write_text(json.dumps(list(termios.tcgetwinsize(0))))
        if b"x" in keys:
            break
    termios.tcsetattr(0, termios.TCSANOW, original)
    state = "stopped"
status = {"state": state, "source_frame_count": 2, "consumed_frame_count": 2,
          "dropped_frame_count": 0, "accepted_count": 2, "rejected_count": 0}
if state == "failed":
    status["error"] = "ORIGINAL SOLVER FAILURE"
(output / "status.json").write_text(json.dumps(status))
(output / "manifest.json").write_text(json.dumps({
    "input": {"sha256": hashlib.sha256(source.read_bytes()).hexdigest()},
    "tracking": {"maximum_accepted_joint_violation": 0.0},
    "red_timing": {"solver": {"p95_ms_upper_bound": 0.2, "p99_ms_upper_bound": 0.4, "maximum_ms": 0.5}}
}))
(output / "trace.csv").write_text("frame,accepted\n1,true\n")
if "noartifact" not in source.stem:
    Path(value("--mcap")).write_bytes(b"test-output")
print("NATIVE FINAL STATE", state, flush=True)
sys.exit(1 if state == "failed" else 0)
