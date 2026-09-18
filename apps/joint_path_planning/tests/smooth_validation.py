#!/usr/bin/env python3
"""Exercise explicit smooth acceptance policy and persistent fatal diagnostics."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--binary', required=True)
parser.add_argument('--urdf', required=True)
parser.add_argument('--app', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
a = parser.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
root = Path(tempfile.mkdtemp(prefix='smooth-validation-', dir=a.output))
base = [a.binary, '--urdf', a.urdf, '--config', str(a.app / 'configs/free.json'),
        '--headless', '--no-viz', '--no-record', '--planning-mode', 'single-arm',
        '--goal', '{"delta":[0.02,0,0]}']

def run(name, args):
    output = root / name
    completed = subprocess.run(base + ['--output', str(output)] + args,
                               capture_output=True, text=True, timeout=30)
    assert completed.returncode == 0, (completed.stderr, output)
    return json.loads((output / 'status.json').read_text()), json.loads((output / 'resolved.json').read_text())

full, resolved = run('full', ['--timing-mode', 'smooth'])
assert resolved['smooth_validation'] == 'full'
assert full['accepted'] and full['trajectory_validation_status'] == 'passed', full
none, resolved = run('none', ['--timing-mode', 'smooth', '--smooth-validation', 'none'])
assert resolved['smooth_validation'] == 'none'
assert resolved['trajectory_policy'] == 'totg-ruckig-generated-curve-v1'
assert none['accepted'] and none['trajectory_validation_status'] == 'skipped', none
assert none['trajectory_collision_queries'] == none['trajectory_interval_count'] == 0
assert none['trajectory_verification_ms'] == 0
assert 'trajectory_clearance_lower_bound_m' not in none
assert 'maximum_deviation_bound' not in none
assert 'skipped' in none['reason']
plain, _ = run('plain', ['--smooth-validation', 'none'])
assert plain['accepted'] and plain['trajectory_validation_status'] == 'not_applicable'
invalid = subprocess.run(base + ['--smooth-validation', 'invalid'], capture_output=True, text=True)
assert invalid.returncode == 1 and 'full or none' in invalid.stderr
request = root / 'request.json'
request.write_text(json.dumps(dict(schema_version='joint_path_planning.request.v1',
    planning_mode='single-arm', config=str(a.app / 'configs/free.json'), urdf=a.urdf,
    output=str(root / 'request-run'), record=False, goal={'delta': [0.02, 0, 0]},
    timing_mode='smooth', smooth_validation='none')))
subprocess.run([a.binary, '--request', str(request)], check=True, capture_output=True, timeout=30)
assert json.loads((root / 'request-run/status.json').read_text())['trajectory_validation_status'] == 'skipped'
cap = subprocess.run([a.binary, '--describe-capabilities'], check=True, capture_output=True, text=True)
assert json.loads(cap.stdout)['smooth_validation_modes'] == ['full', 'none']
# Fatal errors after run creation are persisted as well as printed.
failed_dir = root / 'failed'
failed = subprocess.run(base + ['--output', str(failed_dir), '--urdf', str(root / 'missing.urdf')],
                        capture_output=True, text=True)
assert failed.returncode == 1 and failed.stderr.strip()
assert failed.stderr.strip() in (failed_dir / 'native.log').read_text()
