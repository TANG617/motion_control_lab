#!/usr/bin/env python3
"""MCAP schemas + protobuf joint order + independent sampled R1 mesh checks.

Run after contract.py. Requires mcap, jsonschema, protobuf, numpy, Pinocchio and Coal.
These are verification dependencies only, not app runtime dependencies.
"""
import argparse
import csv
import json
from pathlib import Path
from mcap.reader import make_reader
import jsonschema
from google.protobuf import descriptor_pb2, message_factory
import coal
import numpy as np
import pinocchio as pin

p = argparse.ArgumentParser()
p.add_argument('--run', required=True, type=Path)
p.add_argument('--urdf', default='/workspace/models/Psi_R1_visual_collision.urdf', type=Path)
a = p.parse_args()
config = json.loads((a.run/'resolved.json').read_text())['scene']
status = json.loads((a.run/'status.json').read_text())
assert status['accepted'] and not status['direct_valid']
with (a.run/'attempt-1-path.csv').open() as f:
    rows = list(csv.reader(f)); names = rows[0]; waypoints = np.array(rows[1:], float)
counts = {}; validators = {}; classes = {}; last_scene = None; entities = {}; entity_updates = {}; last_q = None; poses = {}; quality_messages = 0
with (a.run/'visualization.mcap').open('rb') as f:
    for schema, channel, message in make_reader(f, validate_crcs=True).iter_messages():
        counts[channel.topic] = counts.get(channel.topic, 0) + 1
        if channel.message_encoding == 'json':
            if schema.id not in validators:
                validators[schema.id] = jsonschema.Draft7Validator(json.loads(schema.data))
            data = json.loads(message.data); validators[schema.id].validate(data)
            if channel.topic == '/joint_path/scene':
                last_scene = data
                for deletion in data['deletions']:
                    assert deletion['type'] == 0
                    entities.pop(deletion['id'], None)
                for entity in data['entities']:
                    entities[entity['id']] = entity
                    entity_updates[entity['id']] = entity_updates.get(entity['id'], 0) + 1
            if channel.topic == '/joint_path/status' and data.get('accepted'):
                assert data['path_length_after_simplification'] <= data['path_length_before_simplification']
                assert data['simplification_attempts'] <= 256
                assert 'simplification_termination' in data
                quality_messages += 1
        elif channel.message_encoding == 'protobuf':
            if schema.id not in classes:
                descriptors = descriptor_pb2.FileDescriptorSet.FromString(schema.data)
                classes[schema.id] = message_factory.GetMessages(descriptors.file)[schema.name]
            data = classes[schema.id].FromString(message.data)
            if schema.name == 'foxglove.JointStates':
                assert [j.name for j in data.joints] == names
                if channel.topic == '/mcl/joints/execution': last_q = [j.position for j in data.joints]
            elif schema.name == 'foxglove.PoseInFrame':
                assert data.frame_id == 'base_link'
                poses[channel.topic] = data.pose
shared_root = Path(__file__).resolve().parents[3] / 'contracts/visualization'
expected = set()
for contract in ['mcl_state.v1.json', 'mcl_execution.v1.json']:
    expected.update(x['topic'] for x in json.loads((shared_root/contract).read_text())['channels'])
assert quality_messages > 0
assert expected <= counts.keys(), expected - counts.keys()
assert '/mcl/cartesian/reference/left' in counts
assert not any(t in counts for t in ['/joint_path/current', '/joint_path/goal_joints',
                                    '/joint_path/draft', '/joint_path/tcp', '/joint_path/submitted'])
assert np.max(np.abs(np.array(last_q) - waypoints[-1])) < 1e-10
assert entity_updates['planned-path'] == 1
assert len([e for e in entities if e.startswith('waypoint-')]) == status['stop_waypoint_count']
for box in config['obstacles']:
    assert entity_updates[box['id']] == 1
    cube = entities[box['id']]['cubes'][0]
    assert [cube['pose']['position'][k] for k in 'xyz'] == box['xyz']
    assert [cube['size'][k] for k in 'xyz'] == box['size']
assert {'joint-direct', 'planned-path', 'executed-trace'} <= entities.keys()

# Independent narrowphase oracle: no MCC broadphase or cached distances.
model = pin.buildModelFromUrdf(str(a.urdf))
geometry = pin.buildGeomFromUrdf(model, str(a.urdf), pin.GeometryType.COLLISION,
                                 package_dirs=[str(a.urdf.parent)])
data = model.createData(); gd = pin.GeometryData(geometry)
links = [model.frames[g.parentFrame].name for g in geometry.geometryObjects]
allowed = {frozenset(x['links']) for x in config['allowed_collisions']}
world = [(coal.Box(*x['size']), coal.Transform3s(np.eye(3), np.array(x['xyz']))) for x in config['obstacles']]
indices = [model.joints[model.getJointId(n)].idx_q for n in names]
final_q = pin.neutral(model); final_q[indices] = waypoints[-1]
pin.forwardKinematics(model, data, final_q); pin.updateFramePlacements(model, data)
for side in ['left', 'right']:
    ee = data.oMf[model.getFrameId(side + '_arm_ee_link')]
    expected_tcp = ee.translation + ee.rotation @ np.array([0, 0, .1])
    for kind in ['execution', 'ik']:
        actual = poses['/mcl/cartesian/' + kind + '/' + side].position
        assert np.linalg.norm(np.array([actual.x, actual.y, actual.z]) - expected_tcp) < 1e-9

def distance(values):
    q = pin.neutral(model); q[indices] = values
    pin.updateGeometryPlacements(model, data, geometry, gd, q)
    poses = [coal.Transform3s(t.rotation, t.translation) for t in gd.oMg]
    best = float('inf')
    def pair(ga, ta, gb, tb):
        cr = coal.CollisionResult()
        coal.collide(ga, ta, gb, tb, coal.CollisionRequest(), cr)
        if cr.isCollision(): return 0.0
        dr = coal.DistanceResult(); request = coal.DistanceRequest(); request.enable_signed_distance = False
        coal.distance(ga, ta, gb, tb, request, dr)
        return dr.min_distance
    for i, ga in enumerate(geometry.geometryObjects):
        for j in range(i+1, len(poses)):
            if links[i] == links[j] or frozenset([links[i], links[j]]) in allowed: continue
            best = min(best, pair(ga.geometry, poses[i], geometry.geometryObjects[j].geometry, poses[j]))
        for gb, tb in world: best = min(best, pair(ga.geometry, poses[i], gb, tb))
    return best
minimum = float('inf'); count = 0
for x, y in zip(waypoints, waypoints[1:]):
    for alpha in np.linspace(0, 1, 9):
        d = distance(x + alpha*(y-x)); assert d >= 0.005 - 1e-9, (alpha, d)
        minimum = min(minimum, d); count += 1
assert any(distance(waypoints[0]+t*(waypoints[-1]-waypoints[0])) < 0.005 for t in [.25,.5,.75])
print('PASS MCAP schemas/joints/scene and independent mesh oracle:', counts, 'states', count, 'minimum', minimum)

if status.get('planning_mode') == 'whole-body':
    assert status['active_dof'] == 16
    active = set(status['active_joint_names'])
    assert len(active) == 16
    held_frame = status['held_frame']
    def tcp_at(values):
        q = pin.neutral(model); q[indices] = values
        pin.forwardKinematics(model, data, q); pin.updateFramePlacements(model, data)
        ee = data.oMf[model.getFrameId(held_frame)]
        return ee.translation + ee.rotation @ np.array([0, 0, .1]), ee.rotation.copy()
    anchor_p, anchor_r = tcp_at(waypoints[0])
    maximum_position = 0.; maximum_orientation = 0.
    # Independently sample the actual polyline, not projected manifold states.
    for start, end in zip(waypoints, waypoints[1:]):
        for alpha in np.linspace(0, 1, 101):
            pos, rot = tcp_at(start + alpha*(end-start))
            maximum_position = max(maximum_position, np.linalg.norm(pos-anchor_p))
            maximum_orientation = max(maximum_orientation, np.linalg.norm(pin.log3(anchor_r.T @ rot)))
    assert maximum_position <= status['held_path_position_error_bound_m'] <= .001
    assert maximum_orientation <= status['held_path_orientation_error_bound_rad'] <= .01
    for j, name in enumerate(names):
        if name not in active: assert np.all(waypoints[:, j] == waypoints[0, j]), name
    for name in ['torso_pitch_joint', 'torso_yaw_joint']:
        assert np.ptp(waypoints[:, names.index(name)]) > 1e-5, name
    held_prefix = 'right' if status['side'] == 'left' else 'left'
    assert max(np.ptp(waypoints[:, names.index(held_prefix + '_arm_joint' + str(i))]) for i in range(1, 8)) > 1e-4
    with (a.run/'attempt-1-trajectory.csv').open() as f:
        timed = list(csv.DictReader(f))
    last_time = -1.
    trajectory_q = []
    for sample in timed:
        t = float(sample['time_s']); assert t > last_time; last_time = t
        values = np.array([float(sample[n + '_q']) for n in names]); trajectory_q.append(values)
        pos, rot = tcp_at(values)
        assert np.linalg.norm(pos-anchor_p) <= .001
        assert np.linalg.norm(pin.log3(anchor_r.T @ rot)) <= .01
        assert any(np.linalg.norm(values-(x+np.clip(np.dot(values-x,y-x)/np.dot(y-x,y-x),0,1)*(y-x))) < 1e-9
                   for x,y in zip(waypoints,waypoints[1:]))
    for waypoint in waypoints[status["stop_waypoint_indices"]]:
        k = next(k for k,q in enumerate(trajectory_q) if np.max(np.abs(q-waypoint)) < 1e-10)
        assert max(abs(float(timed[k][n+'_v'])) for n in names) < 1e-10
        assert max(abs(float(timed[k][n+'_a'])) for n in names) < 1e-10
    assert '/joint_path/held_tcp' in counts
    pose = poses['/joint_path/held_tcp']
    assert np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z])-anchor_p) < 1e-10
    print('PASS 16-DOF hold, waist/arm compensation, fixed other joints and timing:',
          'maximum hold error', maximum_position, maximum_orientation, 'samples', len(timed))
