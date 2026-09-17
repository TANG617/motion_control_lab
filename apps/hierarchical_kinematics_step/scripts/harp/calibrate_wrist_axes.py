#!/usr/bin/env python3
"""Offline fixed R1 zero-pose / SMPL-H neutral wrist-axis calibration (no inference)."""
import argparse
import hashlib
import io
import json
import tarfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def calibrate(urdf, archive):
    with tarfile.open(archive, 'r:xz') as tar:
        member = next(m for m in tar if m.name.endswith('neutral/model.npz'))
        data = tar.extractfile(member).read()
    with np.load(io.BytesIO(data), allow_pickle=False) as model:
        rest = model['J_regressor'][:22] @ model['v_template']
    left = rest[16] - rest[17]
    left /= np.linalg.norm(left)
    up = rest[12] - rest[9]
    up -= up.dot(left) * left
    up /= np.linalg.norm(up)
    human = np.column_stack((np.cross(left, up), left, up)).T
    poses = {'body_link4': np.eye(4)}
    joints = list(ET.parse(urdf).getroot().findall('joint'))
    while joints:
        ready = [j for j in joints if j.find('parent').get('link') in poses]
        if not ready:
            break
        for joint in ready:
            origin = joint.find('origin')
            t = np.eye(4)
            if origin is not None:
                t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            poses[joint.find('child').get('link')] = poses[joint.find('parent').get('link')] @ t
            joints.remove(joint)
    axes = np.diag([-1., -1., 1.])
    robot = [axes @ poses[side + '_arm_ee_link'][:3, :3] for side in ('left', 'right')]
    return dict(schema_version='r1-zero_smplh-neutral-zero-beta.v1',
                urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
                neutral_model_sha256=hashlib.sha256(data).hexdigest(),
                method='C_side = R_robot_EE_zero.T @ R_human_wrist_zero; beta=0; no startup calibration',
                human_wrist_zero=human.tolist(), robot_ee_zero=[r.tolist() for r in robot],
                reference_from_torso_axes=axes.tolist(), wrist_in_ee=[0., 0., -.097],
                ee_to_model_wrist_axes=[(r.T @ human).tolist() for r in robot])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--smplh', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.output.open('x') as output:
        json.dump(calibrate(args.urdf, args.smplh), output, indent=2)
        output.write('\n')
