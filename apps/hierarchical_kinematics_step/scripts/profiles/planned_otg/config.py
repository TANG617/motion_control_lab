"""Complete Python launcher overrides for the planned-otg profile."""

from pathlib import Path

from _launcher import Recipe


PROFILE = "planned-otg"
MCAP_OUTPUT_ROOT = "/workspace/runs/mcl_hierarchical_kinematics_step"
CSV_OUTPUT_ROOT = str(
    Path(__file__).resolve().parents[5] / "runs" / "hierarchical_kinematics_step"
)
URDF = "/workspace/models/Psi_R1_visual_collision.urdf"
DEFAULT_MCAP = (
    "/workspace/fixtures/raw/"
    "sliced-RW1AZHYCSEFT5_RW1AZHYCSEFT5260310002_20260813164525_0.mcap"
)

RECIPES = {
    "keyboard": Recipe(
        profile=PROFILE,
        name="keyboard",
        source="teleop",
        options={
            "urdf": URDF,
            "red-rate": 1000,
            "yellow-rate": 100,
            "deadline-policy": "monitor",
            "ui": "tui",
            "viz": "foxglove",
            "host": "127.0.0.1",
            "port": 8765,
            "mcap": None,
            "max-linear-velocity-mps": 0.9,
            "max-linear-acceleration-mps2": 5.0,
            "max-linear-jerk-mps3": 80.0,
            "max-angular-velocity-rps": 3.0,
            "max-angular-acceleration-rps2": 20.0,
            "max-angular-jerk-rps3": 200.0,
            "joint-target-mode": "future-o1-pv",
        },
    ),
    "mcap_interactive": Recipe(
        profile=PROFILE,
        name="mcap_interactive",
        source="mcap",
        options={
            "urdf": URDF,
            "input-format": "mcap",
            "input": DEFAULT_MCAP,
            "left-stream": "/hal/tracker/htc/left/calib_target_pose",
            "right-stream": "/hal/tracker/htc/right/calib_target_pose",
            "initial-joint-state-stream": "/mc/ik/joint_states",
            "timestamp-source": "header_stamp",
            "target-period-ms": 10,
            "pairing-policy": "nearest",
            "nearest-tolerance-ms": 5,
            "unmatched-policy": "drop_with_diagnostics",
            "execution-mode": "realtime",
            "playback-rate": 1,
            "red-rate": 1000,
            "yellow-rate": 500,
            "deadline-policy": "monitor",
            "ui": "tui",
            "terminal-input": True,
            "replay-trace": True,
            "viz": "foxglove",
            "host": "127.0.0.1",
            "port": 8765,
            "mcap": None,
            "output-root": MCAP_OUTPUT_ROOT,
            "start-paused": True,
            "max-linear-velocity-mps": 0.8,
            "max-linear-acceleration-mps2": 4.0,
            "max-linear-jerk-mps3": 60.0,
            "max-angular-velocity-rps": 3.5,
            "max-angular-acceleration-rps2": 15.0,
            "max-angular-jerk-rps3": 180.0,
            "joint-target-mode": "future-o1-pv",
        },
    ),
    "mcap_headless": Recipe(
        profile=PROFILE,
        name="mcap_headless",
        source="mcap",
        options={
            "urdf": URDF,
            "input-format": "mcap",
            "input": DEFAULT_MCAP,
            "left-stream": "/hal/tracker/htc/left/calib_target_pose",
            "right-stream": "/hal/tracker/htc/right/calib_target_pose",
            "initial-joint-state-stream": "/mc/ik/joint_states",
            "timestamp-source": "header_stamp",
            "target-period-ms": 10,
            "pairing-policy": "nearest",
            "nearest-tolerance-ms": 5,
            "unmatched-policy": "drop_with_diagnostics",
            "execution-mode": "batch",
            "playback-rate": 1,
            "red-rate": 1000,
            "yellow-rate": 500,
            "deadline-policy": "monitor",
            "ui": "none",
            "terminal-input": False,
            "replay-trace": True,
            "viz": "none",
            "host": "127.0.0.1",
            "port": 8765,
            "mcap": None,
            "output-root": MCAP_OUTPUT_ROOT,
            "start-paused": False,
            "max-linear-velocity-mps": 0.8,
            "max-linear-acceleration-mps2": 4.0,
            "max-linear-jerk-mps3": 60.0,
            "max-angular-velocity-rps": 3.5,
            "max-angular-acceleration-rps2": 15.0,
            "max-angular-jerk-rps3": 180.0,
            "joint-target-mode": "future-o1-pv",
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
            "joint-target-mode": "future-o1-pv",
        },
    ),
}
