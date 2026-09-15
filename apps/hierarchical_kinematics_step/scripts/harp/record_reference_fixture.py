#!/usr/bin/env python3
"""Record 30 s of a successful native run's accepted EE references as TCP MCAP.

This is a derived integration fixture, not a hardware recording. All original
source timestamps remain in the source trace. Generated MCAP uses its own
explicit synthetic epoch and is stored with source hashes outside version control.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from mcap_ros2.writer import Writer

POSE_SCHEMA = """std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
"""
JOINT_SCHEMA = """std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--run",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    manifest=json.loads((a.run/'manifest.json').read_text())
    status=json.loads((a.run/'status.json').read_text())
    if status['state']!='succeeded': raise ValueError('requires a successful source run')
    with (a.run/'trace.csv').open() as f:
        rows=[r for r in csv.DictReader(f) if r['accepted']=='true']
    if len(rows)<30001: raise ValueError('source has less than 30 seconds of accepted control samples')
    robot=manifest['resolved_options']['robot']
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('wb') as f:
        writer=Writer(f);schema=writer.register_msgdef('geometry_msgs/msg/PoseStamped',POSE_SCHEMA)
        joint_schema=writer.register_msgdef('sensor_msgs/msg/JointState',JOINT_SCHEMA)
        initial=dict(header=dict(stamp=dict(sec=1,nanosec=0),frame_id=robot['base_frame']),
                     name=robot['joint_names'],position=robot['default_positions'],
                     velocity=[0.0]*len(robot['joint_names']),effort=[])
        writer.write_message('/mc/ik/joint_states',joint_schema,initial,
                             log_time=1_000_000_000,publish_time=1_000_000_000)
        for i,row in enumerate(rows[:30001:10]):
            ns=1_000_000_000+i*10_000_000
            for side in ('left','right'):
                v=[float(x) for x in row[f'reference_{side}_pose'].split(';')]
                x,y,z,w=v[3:]
                rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                offset=robot[f'{side}_tcp_offset']
                if offset['quaternion_xyzw']!=[0.0,0.0,0.0,1.0]: raise ValueError('fixture requires the R1 translation-only TCP offset')
                xyz=np.array(v[:3])+rotation@np.array(offset['translation'])
                message=dict(header=dict(stamp=dict(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000),frame_id=robot['base_frame']),
                    pose=dict(position=dict(zip(('x','y','z'),xyz)),orientation=dict(zip(('x','y','z','w'),v[3:]))))
                writer.write_message(f'/harp/reference/{side}_tcp',schema,message,log_time=ns,publish_time=ns,sequence=i)
        writer.finish()
    metadata=dict(kind='derived-native-accepted-reference-fixture',duration_s=30,sample_hz=100,
        source_run=str(a.run.resolve()),source_trace_sha256=hashlib.sha256((a.run/'trace.csv').read_bytes()).hexdigest(),
        source_manifest_sha256=hashlib.sha256((a.run/'manifest.json').read_bytes()).hexdigest(),
        synthetic_epoch_ns=1_000_000_000,sha256=hashlib.sha256(a.output.read_bytes()).hexdigest())
    a.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+"\n")


if __name__=='__main__': main()
