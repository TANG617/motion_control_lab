#!/usr/bin/env python3
"""Generate small reachable dual-arm TCP input from the resolved R1 FK."""
import argparse
import csv
import json
from pathlib import Path
import math
import numpy as np
import pinocchio as pin

parser = argparse.ArgumentParser()
parser.add_argument("--resolved-options", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--duration-s", type=float, default=30)
parser.add_argument("--motion", choices=("static", "symmetric", "asymmetric"), default="symmetric")
args = parser.parse_args()
options = json.loads(args.resolved_options.read_text())
robot = options["robot"]
model = pin.buildModelFromUrdf(options["runtime"]["urdf_path"])
data = model.createData()
q0 = pin.neutral(model)
for name, value in zip(robot["joint_names"], robot["default_positions"]):
    joint = model.joints[model.getJointId(name)]
    q0[joint.idx_q] = value
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open("w") as output:
    writer = csv.writer(output)
    writer.writerow(["timestamp_ns", "left_frame_id", "right_frame_id"] + [side+"_"+key for side in ("left", "right") for key in ("x", "y", "z", "qx", "qy", "qz", "qw")])
    for tick in range(round(args.duration_s * 100) + 1):
        time_s = tick / 100
        q = q0.copy()
        delta = (0. if args.motion == "static" else .005) * (1-math.cos(2*math.pi*time_s / args.duration_s))
        for side, sign in (("left", 1), ("right", -1)):
            q[model.joints[model.getJointId(side+"_arm_joint1")].idx_q] += sign*delta
        if args.motion == "asymmetric":
            q[model.joints[model.getJointId('left_arm_joint2')].idx_q] += .015 * (1-math.cos(2*math.pi*time_s / args.duration_s))
            q[model.joints[model.getJointId('right_arm_joint3')].idx_q] += .025 * (1-math.cos(4*math.pi*time_s / args.duration_s))
        pin.framesForwardKinematics(model, data, q)
        row = [tick*10000000, robot["base_frame"], robot["base_frame"]]
        base = data.oMf[model.getFrameId(robot["base_frame"])].inverse()
        for side in ("left", "right"):
            pose = base * data.oMf[model.getFrameId(robot[side+"_end_effector_frame"])]
            offset = robot[side+"_tcp_offset"]
            x,y,z,w = offset["quaternion_xyzw"]
            pose = pose * pin.SE3(pin.Quaternion(w,x,y,z).matrix(), np.array(offset["translation"]))
            row += pose.translation.tolist() + pin.Quaternion(pose.rotation).coeffs().tolist()
        writer.writerow(row)
