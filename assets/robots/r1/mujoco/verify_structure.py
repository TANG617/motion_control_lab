#!/usr/bin/env python3
"""Validate the checked-in R1 model contract without loading a display."""

from __future__ import annotations

import sys
from pathlib import Path
import xml.etree.ElementTree as ET

from prepare_asset import ACTIVE_JOINTS


def main() -> int:
    if len(sys.argv) != 2:
        raise RuntimeError("expected <mjcf>")
    root = ET.parse(Path(sys.argv[1])).getroot()
    joints = [joint.get("name") for joint in root.findall(".//joint")]
    motors = root.findall(".//actuator/motor")
    sites = {site.get("name") for site in root.findall(".//site")}
    free_joints = [joint.get("name") for joint in root.findall(".//freejoint")]
    if set(joints) != set(ACTIVE_JOINTS) or len(joints) != len(ACTIVE_JOINTS):
        raise RuntimeError("MJCF controlled-joint contract does not match R1")
    if free_joints != ["floating_base"]:
        raise RuntimeError("MJCF must contain exactly the floating_base free joint")
    if len(motors) != len(ACTIVE_JOINTS):
        raise RuntimeError("MJCF must contain one future motor per controlled joint")
    if {motor.get("joint") for motor in motors} != set(ACTIVE_JOINTS):
        raise RuntimeError("MJCF motor-to-joint mapping is incomplete")
    if not {"left_tcp_site", "right_tcp_site"}.issubset(sites):
        raise RuntimeError("MJCF is missing a TCP site")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
