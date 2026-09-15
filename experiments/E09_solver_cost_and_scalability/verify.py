#!/usr/bin/env python3
"""Independent unit-level retention and timing boundary checks; no cross-run statistics."""
import argparse,csv,json,pathlib,sys
LAB=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(LAB/'tools/mcc_placo_study'))
from evidence import write_json
from metrics import nearest_rank
from progress import SampleProgress

def check(rows,config,progress=None):
    failures=[];expected=1+config['warmup_calls']+config['measured_calls']
    if len(rows)!=expected:failures.append('incomplete-attempt-denominator')
    for i,r in enumerate(rows):
        if r['attempt_sequence']!=i:failures.append('sequence-gap')
        expected_window='cold' if i==0 else 'warmup' if i<=config['warmup_calls'] else 'steady'
        if r['window_id']!=expected_window:failures.append('window-boundary')
        if r['finish_ns']<r['start_ns']:failures.append('negative-call-time')
        if abs(r['ik_call_time_ms']-(r['finish_ns']-r['start_ns'])*1e-6)>1e-12:failures.append('duration-mismatch')
        if config.get('observation')=='cpp-new':
            if r.get('allocation_count') is None or r.get('allocation_bytes') is None:failures.append('missing-cpp-allocation-observation')
        elif r.get('allocation_count') is not None:failures.append('unexpected-allocation-observation')
        if progress is not None:progress.update(i+1)
    return dict(schema_version='e09.validation.v1',passed=not failures,failures=failures,expected_attempts=expected,observed_attempts=len(rows),
      rejected_attempts=sum(r['solution_quality']=='rejected' for r in rows),tail_estimation='unavailable' if failures else 'all raw attempt samples retained')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--request',required=True);a=ap.parse_args();req=json.load(open(a.request));out=pathlib.Path(req['output_dir'])
    rows=[r for l in (out/'raw.jsonl').read_text().splitlines() if (r:=json.loads(l)).get('record_type')=='attempt'];result=check(rows,req['config'],SampleProgress(out,'app-validation',len(rows)));write_json(out/'timing_validation.json',result);sys.exit(not result['passed'])
