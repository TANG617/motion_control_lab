#!/usr/bin/env python3
"""CoM display option parity across native CLI, requests and app launchers."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("--binary", type=Path, required=True)
args = parser.parse_args()
app = Path(__file__).resolve().parents[1]
profiles = (
    "hierarchical", "planned", "planned-otg", "planned-otg-nullspace",
    "planned-otg-nullspace-admittance-kinematic-sim",
    "posture-reference-task", "posture-reference-task-left",
    "posture-reference-task-right",
)
environment = dict(os.environ, MCL_BINARY=str(args.binary), MCL_CPU_SET="",
                   MCL_RT_PRIORITY="")


def read(command):
    result = subprocess.run(command, env=environment, text=True,
                            capture_output=True, timeout=20)
    assert result.returncode == 0, (command, result.returncode, result.stderr)
    return json.loads(result.stdout)


def show_com(command):
    return read([*command, "--dump-resolved-options"])["runtime"]["visualization"]["show_com"]


with tempfile.TemporaryDirectory(prefix="mcl-com-options-") as temporary:
    root = Path(temporary)
    source = root / "input.json"
    source.write_text('{"samples":[{}]}')
    for profile in profiles:
        # Dumping options must not load the inference package.
        uses_harp = profile.startswith("posture-reference-task")
        harp_model = str(root / "unloaded-model")
        harp_options = {"harp-model": harp_model} if uses_harp else {}
        harp_arguments = ["--harp-model", harp_model] if uses_harp else []
        for mode in ("teleop", "replay"):
            native = [str(args.binary), "--profile", profile, mode,
                      "--urdf", "/workspace/models/Psi_R1_visual_collision.urdf", *harp_arguments]
            if mode == "replay":
                native += ["--input", str(root / "input.mcap"),
                           "--left-stream", "/left", "--right-stream", "/right",
                           "--target-period-ms", "10"]
                if uses_harp:
                    native += ["--execution-mode", "realtime"]
            assert show_com(native) is False
            assert show_com([*native, "--show-com"]) is True
            assert show_com([*native, "--show-com", "--no-show-com"]) is False
            resolved = read([*native, "--show-com", "--viz", "none", "--dump-resolved-options"])
            assert resolved["runtime"]["visualization"]["enabled"] is False
            support = resolved["robot"]["support_visualization"]
            assert support["reference_frame"] == "base_link"
            assert support["ground_height_m"] == 0.0
            assert support["wheel_frames"] == [
                "wheel_front_left_link", "wheel_front_right_link",
                "wheel_back_right_link", "wheel_back_left_link"]
            assert support["geometry"] == "wheel_center_projections"
            assert support["assumption"] == "fixed_level_base_four_wheels_in_contact"

            script = "run_keyboard.py" if mode == "teleop" else "run_mcap_headless.py"
            launcher = app / "scripts" / "profiles" / profile.replace("-", "_") / script
            command = [sys.executable, str(launcher), *harp_arguments]
            if mode == "replay":
                command += ["--input", str(root / "input.mcap")]
            assert show_com([*command, "--show-com"]) is True
            assert show_com([*command, "--show-com", "--no-show-com"]) is False

        request = {
            "schema_version": "execution_request.v1",
            "app_id": "mcl_hierarchical_kinematics_step",
            "execution_structure": "replay" if profile != "hierarchical" else "snapshot",
            "input": {"path": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                      "format": "json"},
            "app_config": {"profile": profile, "options": {
                "show-com": True, "urdf": "/workspace/models/Psi_R1_visual_collision.urdf", **harp_options}},
            "execution": {}, "observation": {}, "tracking": {},
            "output_dir": str(root / profile),
        }
        if uses_harp:
            request["app_config"]["options"]["execution-mode"] = "realtime"
        request_path = root / "request.json"
        request_path.write_text(json.dumps(request))
        assert show_com([str(args.binary), "--request", str(request_path)]) is True
        request["app_config"]["options"]["show-com"] = False
        request_path.write_text(json.dumps(request))
        assert show_com([str(args.binary), "--request", str(request_path)]) is False

capabilities = read([str(args.binary), "--describe-capabilities"])
assert capabilities["visualization"]["show_com"]["default"] is False
assert capabilities["visualization"]["show_com"]["state_source"] == "committed_execution"
assert capabilities["visualization"]["show_com"]["entities"] == [
    "whole_robot_com", "four_wheel_support", "whole_robot_com_projection",
    "whole_robot_com_projection_line"]
assert capabilities["visualization"]["show_com"]["projection_color"] == \
    "green_strictly_inside_red_boundary_or_outside"
print(f"CoM options: {len(profiles)} profiles, CLI, launchers, requests and capabilities passed")
