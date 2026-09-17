#!/usr/bin/env python3
"""Installed/native CLI, exact R1 detour, rejection, timing and terminal lifecycle."""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import tempfile
import termios
import time

p = argparse.ArgumentParser()
p.add_argument('--binary', required=True)
p.add_argument('--urdf', required=True)
p.add_argument('--app', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
root = Path(tempfile.mkdtemp(prefix='contract-', dir=a.output))
base = [a.binary, '--urdf', a.urdf, '--no-viz', '--no-record', '--planning-mode', 'single-arm', '--budget', '5']

def run(name, config='cube', extra=(), code=0):
    out = root / name
    command = base + ['--config', str(a.app / 'configs' / (config + '.json')),
                      '--output', str(out), '--headless', *extra]
    result = subprocess.run(command, capture_output=True, text=True, timeout=45)
    assert result.returncode == code, (command, result.returncode, result.stderr)
    return out, json.loads((out / 'status.json').read_text())

cap = subprocess.run([a.binary, '--describe-capabilities', '--urdf', '/missing'],
                     check=True, capture_output=True, text=True)
assert json.loads(cap.stdout)['hardware'] is False
cube, status = run('cube')
assert status['accepted'] and not status['direct_valid'], status
assert status['backend_status'] == 'Exact solution' and status['waypoints'] >= 3
assert status['path_length_after_simplification'] <= status['path_length_before_simplification']
assert 0 <= status['path_shortening_percent'] <= 100
assert 0 <= status['accepted_shortcuts'] <= status['simplification_attempts'] <= 256
assert status['simplification_ms'] <= status['planning_ms']
assert status['simplification_termination'] in ['completed', 'stalled', 'attempt_limit', 'budget_exhausted']
assert json.loads((cube/'resolved.json').read_text())['simplification_budget_s'] == 1
assert status['app_path_checks'] == status['app_timed_state_checks'] == 0
assert status['timing_mode'] == 'straight-through'
assert status['collision_queries'] > 0 and status['broadphase_skips'] > 0
geometry = json.loads((cube/'geometry.json').read_text())
assert geometry['initial_checked_pairs'] == geometry['geometry_count'] == 34

assert status['remaining_position_error_m'] < 0.001
assert status['final_path_sampled_distance_m'] >= 0.005
for key in ['velocity_limit_ratio', 'acceleration_limit_ratio', 'jerk_limit_ratio']:
    assert status[key] <= 1.000001, (key, status[key])
with (cube / 'attempt-1-path.csv').open() as f:
    rows = list(csv.reader(f)); names = rows[0]; waypoints = [list(map(float, r)) for r in rows[1:]]
with (cube / 'attempt-1-trajectory.csv').open() as f:
    samples = list(csv.DictReader(f))
assert len(names) == 42 and float(samples[0]['time_s']) == 0
assert all(float(b['time_s']) > float(x['time_s']) for x, b in zip(samples, samples[1:]))
for waypoint_index, waypoint in enumerate(waypoints):
    hits = [s for s in samples if max(abs(float(s[n + '_q']) - q) for n, q in zip(names, waypoint)) < 1e-10]
    # A nearby periodic sample can enter the position tolerance before the exact
    # boundary. Require an actual rest sample, not the first tolerance match.
    assert hits
    if waypoint_index in status['stop_waypoint_indices']:
        assert any(all(float(h[n + '_v']) == 0 and float(h[n + '_a']) == 0 for n in names) for h in hits)
for s in samples:
    q = [float(s[n + '_q']) for n in names]
    for j, n in enumerate(names):
        if not n.startswith('left_arm_joint'):
            assert q[j] == waypoints[0][j], n
    distances = []
    for x, y in zip(waypoints, waypoints[1:]):
        delta = [v-u for u,v in zip(x,y)]
        alpha = sum((v-u)*d for v,u,d in zip(q,x,delta)) / sum(d*d for d in delta)
        distances.append(max(abs(v-u-max(0,min(1,alpha))*d) for v,u,d in zip(q,x,delta)))
    assert min(distances) < 1e-9
free_dir, free = run('free', 'free')
assert json.loads((free_dir/'geometry.json').read_text())['initial_checked_pairs'] == 0
assert free['accepted'] and free['direct_valid'] and free['waypoints'] == 2
_, old_timing = run('stop-mode', extra=['--timing-mode', 'stop-at-waypoints'])
assert old_timing['accepted'] and old_timing['through_waypoint_count'] == 0
assert old_timing['stop_waypoint_count'] == old_timing['waypoints']
_, disabled = run('disabled', extra=['--simplification-budget', '0'])
assert disabled['accepted'] and disabled['simplification_termination'] == 'disabled'
assert disabled['simplification_attempts'] == disabled['accepted_shortcuts'] == 0
assert disabled['path_length_before_simplification'] == disabled['path_length_after_simplification']
assert free['simplification_termination'] == 'not_run'
rejected, timeout = run('timeout' , extra=['--budget', '0.000001'], code=2)
assert not timeout['accepted'] and not list(rejected.glob('*trajectory.csv'))
_, ik = run('unreachable', extra=['--goal', '{"xyz":[5,5,5]}'], code=2)
assert not ik['ik_accepted'] and ik['trajectory_time_s'] == 0
request = root / 'request.json'
request.write_text(json.dumps(dict(schema_version='joint_path_planning.request.v1', planning_mode='single-arm',
    config=str(a.app/'configs/free.json'), urdf=a.urdf, output=str(root/'request-run'),
    record=False, goal={'delta':[0,0,0]}, seed=42, budget_s=5, simplification_budget_s=0, timing_mode='stop-at-waypoints')))
result = subprocess.run([a.binary, '--request', str(request)], capture_output=True, text=True, timeout=30)
assert result.returncode == 0, result.stderr
assert json.loads((root/'request-run/status.json').read_text())['accepted']
assert json.loads((root/'request-run/resolved.json').read_text())['simplification_budget_s'] == 0
assert json.loads((root/'request-run/resolved.json').read_text())['timing_mode'] == 'stop-at-waypoints'
result = subprocess.run([a.binary, '--request', str(request), '--budget', '10'], capture_output=True)
assert result.returncode == 1

# Real PTY: target entry, page navigation, no resubmission while paused, clock resume,
# normal exit and terminal attribute restoration. Native logs must not corrupt the UI.
master, slave = pty.openpty()
fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 26, 90, 0, 0))
before = termios.tcgetattr(slave)
process = subprocess.Popen(base + ['--config', str(a.app/'configs/free.json'), '--output', str(root/'pty')],
                           stdin=slave, stdout=slave, stderr=slave)
buffer = bytearray()
def wait_text(text, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if text.encode() in buffer:
            buffer.clear(); return
        if select.select([master], [], [], 0.05)[0]:
            buffer.extend(os.read(master, 65536))
        assert process.poll() is None, bytes(buffer)[-2000:]
    raise AssertionError((text, bytes(buffer)[-2000:]))
try:
    wait_text('READY')
    os.write(master, b'm0.02\r31p\r')
    wait_text('EXECUTING')
    os.write(master, b' ')
    wait_text('PAUSED')
    os.write(master, b'\r')
    time.sleep(0.2)
    os.write(master, b' ')
    wait_text('Reached goal')
    os.write(master, b'3')
    wait_text('Normalized length')
    os.write(master, b'x')
    process.wait(timeout=10)
    assert process.returncode == 0
    assert termios.tcgetattr(slave) == before
finally:
    if process.poll() is None:
        process.terminate(); process.wait(timeout=10)
    os.close(master); os.close(slave)
status = json.loads((root/'pty/status.json').read_text())
assert status['request_id'] == 1 and status['message'] == 'Reached goal', status
assert 'Execution clock paused' in status['events'] and 'Execution clock resumed' in status['events']
print('PASS: capabilities, cube detour, free path, timing/polyline, inactive joints, timeout, IK rejection, request, PTY pause/resume:', root)

# Cancellation stops the pending request and never publishes an executable CSV.
master, slave = pty.openpty()
fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 24, 60, 0, 0))
before = termios.tcgetattr(slave)
process = subprocess.Popen(base + ['--config', str(a.app/'configs/cube.json'), '--output', str(root/'cancel')],
                           stdin=slave, stdout=slave, stderr=slave)
buffer.clear()
try:
    wait_text('READY')
    os.write(master, b'\r')
    # Search can finish within one UI frame; final verification is cancellable too.
    wait_text('VERIFY')
    os.write(master, b'c')
    wait_text('CANCELLED')
    process.send_signal(2)
    process.wait(timeout=10)
    assert process.returncode == 0
    assert termios.tcgetattr(slave) == before
finally:
    if process.poll() is None:
        process.terminate(); process.wait(timeout=10)
    os.close(master); os.close(slave)
status = json.loads((root/'cancel/status.json').read_text())
assert status['state'] == 'CANCELLED' and not status['accepted'] and not status['execution_complete']
assert not list((root/'cancel').glob('*trajectory.csv'))
print('PASS cancellation and Ctrl+C terminal restoration at 60 columns')
