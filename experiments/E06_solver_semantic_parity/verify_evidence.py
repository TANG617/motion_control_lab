#!/usr/bin/env python3
"""Independent actual-equation audit for the new common ServoStep identities only."""
from pathlib import Path
import argparse
import sys
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from analysis import common as C
from repair.common import COMMON_ACCEPTANCE_HASH
from repair.kinematics import GeometricFk

def expected_tasks(data,c,row,fk):
    names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']];q=np.asarray(row['input_state']['q']);result={}
    for side in ('left','right'):
        frame=data['frames'][side];current=fk.pose(names,q,frame,tcp=(0,0,0));target=row['targets'][side]
        goal=np.asarray(target['position'])-np.asarray(target['rotation'])@data['tcp_offsets'][side]
        result[side+'-position']=(fk.jacobian(names,q,frame)[:,active],c['gain_per_s']*(goal-current['position']),1.,c['enforcement'])
        result[side+'-orientation']=(fk.angular_jacobian(names,q,frame)[:,active],c['gain_per_s']*Rotation.from_matrix(np.asarray(target['rotation'])@np.asarray(current['rotation']).T).as_rotvec(),c['orientation_weight'],'soft')
    return result

def audit_row(data,c,row,fk):
    issues=[];checks=[];n=len(data['active_joint_names']);dt=c['period_s'];tol=c['preservation_tolerance'];placo='native_candidate_delta' in row
    expected=expected_tasks(data,c,row,fk);native={r['name']:r for r in row.get('native_task_rows',[])}
    if set(native)!=set(expected):issues.append('actual task set differs or absent')
    for name,(A,b,w,enforcement) in expected.items():
        if name not in native:continue
        r=native[name];actual_A=np.asarray(r['A'],float)
        if placo:actual_A=actual_A[:,row['active_variable_columns']];actual_b=np.asarray(r['b'])/dt
        else:actual_b=np.asarray(r['lower'])-np.asarray(r['offset'])
        gapA=float(np.max(np.abs(A-actual_A)));gapb=float(np.max(np.abs(b-actual_b)))
        normalization=np.asarray(r.get('normalization',np.ones(len(b))))
        passed=bool(max(gapA,gapb)<=tol and r['enforcement']==enforcement and (enforcement=='hard' or (r['weight']==w and np.all(normalization==1))))
        checks.append(dict(task=name,max_A_gap=gapA,max_b_velocity_gap=gapb,tolerance=tol,passed=passed,native_A=actual_A.tolist(),native_b_velocity=actual_b.tolist(),expected_A=A.tolist(),expected_b_velocity=b.tolist(),enforcement=enforcement,weight=w))
        if not passed:issues.append('task equation or enforcement mismatch: '+name)
    active=[data['joint_names'].index(n) for n in data['active_joint_names']];q=np.asarray(row['input_state']['q'])[active]
    lo=np.maximum((np.asarray(data['limits']['lower'])[active]-q)/dt,-np.asarray(data['limits']['velocity'])[active])
    hi=np.minimum((np.asarray(data['limits']['upper'])[active]-q)/dt,np.asarray(data['limits']['velocity'])[active])
    actual_lo=np.full(n,-np.inf);actual_hi=np.full(n,np.inf)
    if placo:
        for constraint in row.get('native_problem_constraints',[]):
            if constraint['type']!=1 or constraint['priority']!=1:continue
            mat=np.asarray(constraint['A'])[:,row['active_variable_columns']];offset=np.asarray(constraint['offset'])
            for a,bound in zip(mat,offset):
                cols=np.flatnonzero(a)
                if len(cols)!=1:continue
                j=cols[0];value=-bound/(a[j]*dt)
                if a[j]>0:actual_lo[j]=max(actual_lo[j],value)
                else:actual_hi[j]=min(actual_hi[j],value)
    elif row.get('native_linearization_available'):
        actual_lo=np.asarray(row['native_joint_lower']);actual_hi=np.asarray(row['native_joint_upper'])
    bounds_ok=len(actual_lo)==n and np.all(np.isfinite(actual_lo)) and np.all(np.isfinite(actual_hi)) and max(np.max(np.abs(actual_lo-lo)),np.max(np.abs(actual_hi-hi)))<=tol
    if not bounds_ok:issues.append('actual hard velocity bounds missing or different')
    if row.get('native_regularization')!=c['regularization']:issues.append('actual regularization differs')
    if row.get('acceptance_contract_hash')!=COMMON_ACCEPTANCE_HASH:issues.append('common contract not applied')
    if row.get('committed') and not(row.get('native_accepted') and row.get('common_quality_passed') and row.get('candidate_available')):issues.append('commit violates native/common conjunction')
    candidate=np.asarray(row.get('native_candidate_delta',[]),float)[row['active_variable_columns']]/dt if placo else np.asarray(row.get('native_candidate_values',[]),float)
    available=bool(row.get('candidate_available')) and len(candidate)==n and np.all(np.isfinite(candidate));quality=False;violation=None
    if available:
        violation=float(max(0.,np.max(lo-candidate),np.max(candidate-hi)))
        for A,b,w,enforcement in expected.values():
            if enforcement=='hard':violation=max(violation,float(np.max(np.abs(A@candidate-b))))
        quality=violation<=c.get('hard_tolerance',tol)
    if row.get('common_quality_passed')!=bool(quality):issues.append('common gate differs from independent candidate quality')

    return dict(attempt_sequence=row['attempt_sequence'],design_comparable=not issues,reasons=issues,tasks=checks,
                declared_lower=lo.tolist(),declared_upper=hi.tolist(),actual_lower=[float(x) if np.isfinite(x) else None for x in actual_lo],actual_upper=[float(x) if np.isfinite(x) else None for x in actual_hi],bounds_passed=bool(bounds_ok),independent_candidate_available=bool(available),independent_common_quality=bool(quality),independent_hard_violation=violation,
                native_accepted=row.get('native_accepted'),candidate_available=row.get('candidate_available'),common_quality_passed=row.get('common_quality_passed'),committed=row.get('committed'))

def verify(request):
    request=Path(request);o=C.read_json(request);out=Path(o['output_dir']);raw=out/'raw.jsonl';checks=[]
    if raw.exists():
        fk=GeometricFk(o['input']['model']['locator'])
        for line,r in enumerate(C.jsonl_rows(raw),1):
            if r.get('record_type')=='attempt':checks.append(dict(source_line=line,**audit_row(o['input'],o['config'],r,fk)))
    complete=len(checks)==o['config']['steps'];reasons=sorted({x for r in checks for x in r['reasons']})
    if not complete:reasons.append('native observation incomplete')
    comparable=bool(checks) and all(r['design_comparable'] for r in checks)
    C.write_json(out/'common_servo_checks.json',checks)
    audit=dict(schema_version='common_servo_audit.v1',acceptance_contract_hash=COMMON_ACCEPTANCE_HASH,
               design_comparable=comparable,observation_complete=complete,claim_eligible=comparable and complete,reasons=reasons,
               source_artifacts=[C.artifact(p) for p in (request,raw,out/'common_servo_checks.json') if p.exists()],
               claim_scope='complete named method under common external contract, not isolated backend',native_failures_preserved=True)
    C.write_json(out/'common_servo_audit.json',audit)
    C.write_json(out/'app_validation.json',dict(status='completed' if complete and comparable else 'failed',audit=audit,original_thresholds_unchanged=True))
    return int(not(complete and comparable))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--request',required=True);raise SystemExit(verify(p.parse_args().request))
