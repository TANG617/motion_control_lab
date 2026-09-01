"""Complete Python launcher overrides for the hierarchical profile."""

from pathlib import Path

from _launcher import Recipe


PROFILE = "hierarchical"
MCAP_OUTPUT_ROOT = "/workspace/runs/mcl_hierarchical_kinematics_step"
CSV_OUTPUT_ROOT = str(
    Path(__file__).resolve().parents[5] / "runs" / "hierarchical_kinematics_step"
)
URDF = "/workspace/models/Psi_R1_visual_collision.urdf"

RECIPES = {
    "keyboard": Recipe(
        profile=PROFILE,
        name="keyboard",
        source="teleop",
        options={
            "urdf": URDF,
            "red-rate": 1000,
            "yellow-rate": 100,
            "deadline-policy": "strict",
            "ui": "tui",
            "viz": "foxglove",
            "host": "127.0.0.1",
            "port": 8765,
            "mcap": None,
        },
    ),
    "mcap_interactive": Recipe(
        profile=PROFILE,
        name="mcap_interactive",
        source="mcap",
        options={
            "urdf": URDF,
            "input-format": "mcap",
            "left-stream": "/mc/ik/left_target",
            "right-stream": "/mc/ik/right_target",
            "timestamp-source": "header_stamp",
            "target-period-ms": 1,
            "pairing-policy": "exact",
            "unmatched-policy": "drop_with_diagnostics",
            "execution-mode": "realtime",
            "playback-rate": 1,
            "red-rate": 1000,
            "yellow-rate": 100,
            "deadline-policy": "strict",
            "ui": "tui",
            "terminal-input": True,
            "replay-trace": True,
            "viz": "foxglove",
            "host": "127.0.0.1",
            "port": 8765,
            "mcap": None,
            "output-root": MCAP_OUTPUT_ROOT,
            "start-paused": True,
        },
    ),
    "mcap_headless": Recipe(
        profile=PROFILE,
        name="mcap_headless",
        source="mcap",
        options={
            "urdf": URDF,
            "input-format": "mcap",
            "left-stream": "/mc/ik/left_target",
            "right-stream": "/mc/ik/right_target",
            "timestamp-source": "header_stamp",
            "target-period-ms": 1,
            "pairing-policy": "exact",
            "unmatched-policy": "drop_with_diagnostics",
            "execution-mode": "batch",
            "playback-rate": 1,
            "red-rate": 1000,
            "yellow-rate": 100,
            "deadline-policy": "strict",
            "ui": "none",
            "terminal-input": False,
            "replay-trace": True,
            "viz": "none",
            "host": "127.0.0.1",
            "port": 8765,
            "mcap": None,
            "output-root": MCAP_OUTPUT_ROOT,
            "start-paused": False,
        },
    ),
    "csv_batch": Recipe(
        profile=PROFILE,
        name="csv_batch",
        source="csv",
        options={
            "urdf": URDF,
            "input-format": "csv",
            "left-stream": "left",
            "right-stream": "right",
            "timestamp-source": "csv_timestamp",
            "target-period-ms": 1,
            "execution-mode": "batch",
            "red-rate": 1000,
            "yellow-rate": 100,
            "deadline-policy": "strict",
            "ui": "none",
            "terminal-input": False,
            "viz": "none",
            "mcap": None,
            "output-root": CSV_OUTPUT_ROOT,
        },
    ),
}
