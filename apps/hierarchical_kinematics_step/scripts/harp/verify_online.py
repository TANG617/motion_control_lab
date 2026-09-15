#!/usr/bin/env python3
"""Join native HARP windows and actual Secondary consumptions; retain failed-run status."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(zip(("p50", "p95", "p99", "max"), np.percentile(values, (50,95,99,100)).tolist()))


def feature(goal):
    p = np.array(goal[:3]); x,y,z,w = goal[3:]
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z)],
                  [2*(x*z-y*w), 2*(y*z+x*w)]])
    return np.r_[p,r.flatten()].astype(np.float32)


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--consumed",type=Path,required=True)
    p.add_argument("--run",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    rows=[json.loads(x) for x in a.consumed.read_text().splitlines()]
    raw=[json.loads(x) for x in Path(str(a.consumed)+".windows.jsonl").read_text().splitlines()]
    windows={(r['generation'],r['sequence']):r for r in raw}
    # left_goal is the replay target; reference_left_pose is the accepted
    # Cartesian planner EE reference actually sampled by HARP.
    with (a.run/'trace.csv').open() as f:
        trace=[r for r in csv.DictReader(f) if r['accepted']=='true']
    assert len(trace)==len(rows)-1, 'consumption/accepted trace count mismatch'
    samples={0.0:rows[0]['left_goal']}
    for row, entry in zip(rows[1:],trace):
        samples[round(row['consume_time_s'],9)]=[float(x) for x in entry['reference_left_pose'].split(';')]
    maximum_feature_error=0.0
    previous=None
    for key,r in windows.items():
        x=np.array(r['window'],dtype=np.float32).reshape(30,9)
        if r['sequence']==0:
            assert np.array_equal(x,np.repeat(x[-1:],30,axis=0)), 'initial history not filled'
        assert abs(r['sample_time_s']-r['sequence']*.01)<1e-9, 'sampling interval changed'
        if previous and 0<r['sequence']-previous['sequence']<30:
            gap=r['sequence']-previous['sequence']
            assert np.array_equal(np.array(previous['window'],dtype=np.float32).reshape(30,9)[gap:],x[:-gap]), 'history changed across skipped requests'
        previous=r
        sample=samples.get(round(r['sample_time_s'],9))
        if sample is not None:
            maximum_feature_error=max(maximum_feature_error,float(np.abs(feature(sample)-x[-1]).max()))
    for row in rows:
        prediction=windows[(row['generation'],row['sequence'])]
        assert abs(row['cos']-prediction['cos'])<1e-12 and abs(row['sin']-prediction['sin'])<1e-12, 'consumed output differs'
        assert 0<=row['age_ms']<=100+1e-6, 'expired prediction consumed'
    assert maximum_feature_error<1e-6, 'EE/TCP feature mismatch'
    selected=[r for r in rows[1:] if r['secondary_succeeded'] and r['selected_priority']==2]
    online=[r for r in windows.values() if r['sequence']>0]
    manifest=json.loads((a.run/'manifest.json').read_text())
    status=json.loads((a.run/'status.json').read_text())
    report=dict(run=str(a.run), native_status=status, statistics=manifest['statistics'],
        control_timing=manifest['red_timing'], tracking=manifest['tracking'],
        consumed_ticks=len(rows)-1, active_duration_s=rows[-1]['consume_time_s'],
        secondary_selected=len(selected), primary_only=sum(r['selected_priority']==1 for r in rows[1:]),
        secondary_succeeded=sum(r['secondary_succeeded'] for r in rows[1:]),
        inference_windows=len(online), skipped_requests=max(w['sequence'] for w in windows.values())-len(online),
        inference_ms=stats([r['inference_ms'] for r in online]), reference_age_ms=stats([r['age_ms'] for r in rows[1:]]),
        max_input_feature_error=maximum_feature_error,
        selected_shared_hard_violation=stats([r['selected_shared_hard_violation'] for r in rows[1:]]),
        selected_secondary_preservation={k:stats([r[k] for r in selected]) for k in ('primary_position_drift','primary_orientation_drift','scale_drift')} if selected else {})
    a.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
