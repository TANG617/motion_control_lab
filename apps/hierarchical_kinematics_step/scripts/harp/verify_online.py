#!/usr/bin/env python3
"""Independently verify dual-arm runtime sampling/consumption; no Python inference."""
import argparse
import csv
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def stats(values):
    x = np.asarray(values, dtype=float)
    if not x.size:
        return None
    return dict(mean=float(x.mean()), median=float(np.median(x)),
                p95=float(np.percentile(x, 95)), max=float(x.max()))


def transform(pose):
    t = np.eye(4)
    t[:3, 3] = pose[:3]
    t[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    return t


def features(row, robot):
    torso = np.array(row['base_from_torso']).reshape(4, 4)
    axes = np.eye(4)
    axes[:3, :3] = robot['reference_from_torso_axes']
    ref = axes @ np.linalg.inv(torso)
    result = []
    for side in range(2):
        ee = transform(row['ee_reference'][side])
        wrist = (ref @ ee @ np.r_[robot['wrist_in_ee'], 1])[:3]
        rotation = ref[:3, :3] @ ee[:3, :3] @ robot['ee_to_model_wrist_axes'][side]
        result.extend(np.r_[wrist, rotation[:, :2].reshape(-1)])
    return np.array(result, dtype=np.float32)


def verify(consumed, run, output, executable=None, model=None, device='cpu'):
    rows = [json.loads(x) for x in consumed.read_text().splitlines()]
    raw = [json.loads(x) for x in Path(str(consumed)+'.windows.jsonl').read_text().splitlines()]
    resolved = json.loads((run/'manifest.json').read_text())['resolved_options']
    samples = {round(r['consume_time_s'], 9): r for r in rows}
    windows = {(r['generation'], r['sequence']): r for r in raw}
    maximum_error = 0.
    previous = None
    for r in windows.values():
        assert r['schema_version'] == 'dual-arm-elbow-window.v1'
        x = np.array(r['window'], dtype=np.float32).reshape(30, 18)
        assert np.isfinite(x).all()
        assert abs(r['sample_time_s']-r['sequence']*.01) < 1e-9
        # Check every frame against the original accepted EE references, including startup fill.
        for i in range(30):
            time = round(max(0., r['sample_time_s']-(29-i)*.01), 9)
            expected = features(samples[time], resolved['robot'])
            maximum_error = max(maximum_error, float(np.abs(x[i]-expected).max()))
        if previous and 0 < r['sequence']-previous['sequence'] < 30:
            gap = r['sequence']-previous['sequence']
            old = np.array(previous['window'], dtype=np.float32).reshape(30, 18)
            assert np.array_equal(old[gap:], x[:-gap]), 'history overlap'
        previous = r
    assert maximum_error < 1e-6, 'runtime torso/wrist mapping mismatch'
    for row in rows:
        assert row['schema_version'] == 'dual-arm-elbow-consumption.v1'
        prediction = windows[(row['generation'], row['sequence'])]
        assert np.array_equal(row['arm_angles'], prediction['arm_angles']), 'consumed pair mismatch'
        assert 0 <= row['age_ms'] <= 100+1e-6
        assert [a['enabled'] for a in row['arms']] == resolved['elbow_reference']['enabled']
    # Independent circle reconstruction in the training torso basis.
    contract = json.loads(Path(str(consumed)+'.model.json').read_text())['contract']
    maximum_elbow_error = 0.
    for row in rows:
        axes = np.eye(4)
        axes[:3,:3] = resolved['robot']['reference_from_torso_axes']
        ref = axes @ np.linalg.inv(np.array(row['base_from_torso']).reshape(4,4))
        for a in range(2):
            if not row['arms'][a]['enabled']:
                continue
            shoulder = np.array(contract['profile']['shoulders_m'][a])
            ee = transform(row['ee_reference'][a])
            wrist = (ref @ ee @ np.r_[resolved['robot']['wrist_in_ee'],1])[:3]
            upper, lower = contract['profile']['lengths_m'][a]
            d = np.linalg.norm(wrist-shoulder)
            n = (wrist-shoulder)/d
            along = (upper**2-lower**2+d*d)/(2*d)
            u = np.array([-1.,0.,0.]); u -= n*u.dot(n); u /= np.linalg.norm(u)
            radius = np.sqrt(upper**2-along**2)
            cosine,sine = row['arm_angles'][a]
            norm = np.hypot(cosine,sine)
            elbow = shoulder+along*n+radius*(cosine*u+sine*np.cross(n,u))/norm
            expected = (np.linalg.inv(ref) @ np.r_[elbow,1])[:3]
            maximum_elbow_error = max(maximum_elbow_error,float(np.linalg.norm(expected-row['arms'][a]['target_elbow'])))
    assert maximum_elbow_error < 1e-8, 'elbow training-basis reconstruction mismatch'
    with (run/'trace.csv').open() as stream:
        trace = [r for r in csv.DictReader(stream) if r['accepted']=='true']
    assert len(trace)==len(rows)-1, 'consumption vs accepted control count'
    for row,tick in zip(rows[1:],trace):
        for a,side in enumerate(('left','right')):
            assert np.max(np.abs(np.array(row['ee_reference'][a])-np.fromstring(tick['reference_'+side+'_pose'],sep=';'))) < 1e-6
    positions = np.array([np.fromstring(t['otg_positions'],sep=';') for t in trace])
    report = dict(profile=resolved['profile'], consumed_ticks=len(rows)-1,
                  max_elbow_reconstruction_error_m=maximum_elbow_error,
                  joint_step_rad=stats(np.abs(np.diff(positions,axis=0)).reshape(-1)),
                  max_input_feature_error=maximum_error,
                  inference_windows=len(windows),
                  inference_ms=stats([w['inference_ms'] for w in windows.values() if w['sequence']]),
                  reference_age_ms=stats([r['age_ms'] for r in rows[1:]]),
                  selected_priority_counts={str(p): sum(r['selected_priority']==p for r in rows[1:]) for p in (1,2,3)},
                  maximum_hard_violation=stats([r['hard_violation'] for r in rows[1:]]),
                  preservation={k:stats([r[k] for r in rows[1:]]) for k in ('primary_position_drift','primary_orientation_drift','scale_drift')},
                  arms=[])
    for a in range(2):
        predictions = []
        previous_identity = None
        for row in rows:
            identity = (row['generation'], row['sequence'])
            if identity != previous_identity:
                predictions.append(row)
                previous_identity = identity
        angles = np.unwrap([np.arctan2(r['arm_angles'][a][1], r['arm_angles'][a][0]) for r in predictions])
        values = [r['arms'][a] for r in rows[1:]]
        report['arms'].append(dict(side=('left','right')[a], task_enabled=resolved['elbow_reference']['enabled'][a],
            tcp_position_error_m=stats([v['executed_position_error_m'] for v in values]),
            tcp_orientation_error_rad=stats([v['executed_orientation_error_rad'] for v in values]),
            elbow_error_m=stats([np.linalg.norm(np.array(v['target_elbow'])-v['executed_elbow']) for v in values if v['enabled']]),
            prediction_increment_deg=stats(np.abs(np.diff(angles))*180/np.pi),
            prediction_sample_gap_ms=stats(np.diff([r['sample_time_s'] for r in predictions])*1000)))
    if executable:
        input_file=output.with_suffix('.windows.txt')
        np.savetxt(input_file,np.array([r['window'] for r in windows.values()]),fmt='%.9g')
        result=subprocess.run([str(executable),str(model),device,str(input_file)],capture_output=True,text=True,check=True)
        output.with_suffix('.cpp.txt').write_text(result.stdout)
        predicted=np.loadtxt(output.with_suffix('.cpp.txt'),ndmin=2)[:,:4]
        expected=np.array([r['arm_angles'] for r in windows.values()]).reshape(-1,4)
        error=float(np.max(np.abs(predicted-expected)))
        assert error<=1e-5, 'independent MCC consumer mismatch'
        report['cpp_consumer_max_absolute_error']=error
    for name in ('status.json','manifest.json'):
        path=run/name
        if path.exists(): report[name]=json.loads(path.read_text())
    report['verification']='passed'
    output.write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('consumed','run','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--cpp-executable',type=Path)
    p.add_argument('--model',type=Path)
    p.add_argument('--device',choices=('cpu','cuda'),default='cpu')
    a=p.parse_args()
    report=verify(a.consumed,a.run,a.output,a.cpp_executable,a.model,a.device)
    print(json.dumps({k:report[k] for k in ('profile','verification','consumed_ticks','max_input_feature_error')}))
