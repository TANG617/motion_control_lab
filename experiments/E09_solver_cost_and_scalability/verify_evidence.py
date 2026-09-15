#!/usr/bin/env python3
"""Independent common ServoStep native-math and external acceptance audit, no native solve."""
import argparse
import importlib.util
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from analysis import common as C
from repair.common import COMMON_ACCEPTANCE_HASH
from repair.kinematics import GeometricFk
spec=importlib.util.spec_from_file_location('e09_timing_verify',Path(__file__).with_name('verify.py'))
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)


def expected_problem(data,c,sample,fk):
    names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']]
    q=np.asarray(sample['q']);tasks={};gain=c['servo_gain'];n=len(active)
    for side in ('left','right'):
        frame=data['frames'][side];pose=fk.pose(names,q,frame,tcp=(0,0,0));target=sample['targets'][side]
        goal=np.asarray(target['position'])-np.asarray(target['rotation'])@data['tcp_offsets'][side]
        tasks[side+'-position']=(fk.jacobian(names,q,frame)[:,active],gain*(goal-np.asarray(pose['position'])),1.)
        if c['workload']>=1:
            tasks[side+'-orientation']=(fk.angular_jacobian(names,q,frame)[:,active],gain*Rotation.from_matrix(np.asarray(target['rotation'])@np.asarray(pose['rotation']).T).as_rotvec(),1.)
        if c['workload']>=3:
            frame=side+'_arm_link4';p=fk.pose(names,q,frame,tcp=(0,0,0))
            tasks[side+'-link4']=(fk.jacobian(names,q,frame)[:,active],gain*(np.asarray(sample['link4_targets'][side]['position'])-p['position']),1.)
    if c['workload']>=2:
        tasks['posture']=(np.eye(n),gain*(np.asarray(data['posture_target'])[active]-q[active]),c['posture_weight'])
    lo=np.maximum(-np.asarray(data['limits']['velocity'])[active],(np.asarray(data['limits']['lower'])[active]-q[active])/c['dt_s'])
    hi=np.minimum(np.asarray(data['limits']['velocity'])[active],(np.asarray(data['limits']['upper'])[active]-q[active])/c['dt_s'])
    return tasks,lo,hi


def external_check(data,c,sample,row):
    reasons=[];accepted=row.get('native_accepted') is True;available=row.get('candidate_available') is True
    q=np.asarray(row.get('candidate_q',[]),float);v=np.asarray(row.get('candidate_v',[]),float)
    names=data['joint_names'];ids=[names.index(n) for n in data['active_joint_names']];dt=c['dt_s'];tol=c['hard_tolerance']
    complete=available and q.shape==(len(names),) and v.shape==(len(names),) and np.isfinite(q).all() and np.isfinite(v).all()
    violation=None
    if complete:
        lo=np.maximum(-np.asarray(data['limits']['velocity'])[ids],(np.asarray(data['limits']['lower'])[ids]-np.asarray(sample['q'])[ids])/dt)
        hi=np.minimum(np.asarray(data['limits']['velocity'])[ids],(np.asarray(data['limits']['upper'])[ids]-np.asarray(sample['q'])[ids])/dt)
        violation=float(max(0,np.max(lo-v[ids]),np.max(v[ids]-hi)))
        integrated=np.max(np.abs((q[ids]-np.asarray(sample['q'])[ids])/dt-v[ids]))
        if integrated>tol:reasons.append('candidate-q-v-integration-disagreement')
    quality=complete and violation<=tol
    if row.get('acceptance_contract_hash')!=COMMON_ACCEPTANCE_HASH:reasons.append('external-contract-hash-mismatch')
    if row.get('common_quality_passed')!=(accepted and quality):reasons.append('external-quality-label-mismatch')
    if row.get('committed')!=(accepted and quality):reasons.append('commit-native-common-conjunction-mismatch')
    if accepted and not quality:reasons.append('candidate-rejected-by-external-contract')
    return dict(native_accepted=accepted,candidate_available=available,common_quality_passed=bool(quality and accepted),
        hard_violation_velocity_units=violation,external_consistent=not reasons,reasons=reasons)


def audit_row(data,c,sample,row,fk):
    actual=row.get('native_audit',{});tasks,lo,hi=expected_problem(data,c,sample,fk);tol=c['hard_tolerance'];n=len(lo)
    problems=[];checks=[];native={t['name']:t for t in actual.get('tasks',[])}
    if not actual.get('available') or set(native)!=set(tasks):problems.append('missing-or-different-native-task-set')
    if len(native)!=len(actual.get('tasks',[])):problems.append('duplicate-native-task-name')
    delta=actual.get('variable_unit')=='rad-increment';scale=c['dt_s'] if delta else 1.
    H=np.eye(n)*c['regularization'];g=np.zeros(n);canonical=[]
    for name,(A,b,weight) in sorted(tasks.items()):
        H+=weight*A.T@A;g-=weight*A.T@b
        canonical.append(dict(name=name,A=A.tolist(),b=b.tolist(),weight=weight))
        if name not in native:continue
        t=native[name];na=np.asarray(t['A']);nb=(np.asarray(t['lower'])-np.asarray(t['offset']))/scale
        norm=np.asarray(t.get('normalization',[]));valid=na.shape==A.shape and nb.shape==b.shape and norm.shape==b.shape
        gap=None
        if valid and np.isfinite(na).all() and np.isfinite(nb).all() and np.all(norm>0):
            alpha=float(t['weight'])/norm**2
            # Equivalent row permutations are legitimate (PlaCo posture is joint-name sorted).
            hgap=float(np.max(np.abs(na.T@(alpha[:,None]*na)-weight*A.T@A)))
            ggap=float(np.max(np.abs(na.T@(alpha*nb)-weight*A.T@b)))
            gap=max(hgap,ggap);valid=gap<=tol and t.get('enforcement')=='soft' and t.get('enabled') is True
        else:valid=False
        checks.append(dict(task=name,passed=bool(valid),objective_max_gap=gap,tolerance=tol,expected_rows=A.shape[0],native_rows=na.shape[0] if na.ndim else None))
        if not valid:problems.append('native-task-objective-mismatch:'+name)
    native_lo=np.asarray(actual.get('joint_lower',[]))/scale;native_hi=np.asarray(actual.get('joint_upper',[]))/scale
    if delta:
        native_lo=np.full(n,-np.inf);native_hi=np.full(n,np.inf)
        columns=actual.get('active_variable_columns',[])
        if len(columns)!=n:problems.append('missing-native-active-variable-map')
        else:
            for constraint in actual.get('problem_constraints',[]):
                if constraint['type']!=1 or constraint['priority']!=1:continue
                matrix=np.asarray(constraint['A'])[:,columns];offset=np.asarray(constraint['offset'])
                for coeff,bound in zip(matrix,offset):
                    idx=np.flatnonzero(coeff)
                    if len(idx)!=1:continue
                    j=idx[0];value=-bound/(coeff[j]*scale)
                    if coeff[j]>0:native_lo[j]=max(native_lo[j],value)
                    else:native_hi[j]=min(native_hi[j],value)
    bounds=native_lo.shape==lo.shape and native_hi.shape==hi.shape and np.isfinite(native_lo).all() and np.isfinite(native_hi).all()
    if bounds:bounds=max(np.max(np.abs(native_lo-lo)),np.max(np.abs(native_hi-hi)))<=tol
    if not bounds:problems.append('native-bounds-missing-or-mismatch')
    if actual.get('regularization')!=c['regularization']:problems.append('native-regularization-mismatch')
    external=external_check(data,c,sample,row);problems+=external['reasons']
    if not external['native_accepted']:problems.append('native-rejection-preserved')
    canonical=dict(tasks=canonical,lower=lo.tolist(),upper=hi.tolist(),regularization=c['regularization'])
    return dict(attempt_sequence=row['attempt_sequence'],input_sequence=row['input_sequence'],design_comparable=not problems,
        checks=checks,bounds_passed=bool(bounds),reasons=problems,external=external,canonical_problem=canonical)


def verify(request):
    request=Path(request);req=C.read_json(request);out=Path(req['output_dir']);c=req['config'];data=req['input'];raw=out/'raw.jsonl'
    rows=[r for r in C.jsonl_rows(raw) if r.get('record_type')=='attempt'] if raw.exists() else []
    timing=original.check(rows,c);C.write_json(out/'timing_validation.json',timing)
    audit=c.get('repair_stage')=='E09-audit';checks=[];reasons=list(timing['failures']);gate=[]
    if c.get('common_acceptance_contract_hash')!=COMMON_ACCEPTANCE_HASH:reasons.append('wrong-common-acceptance-contract')
    if audit:
        fk=GeometricFk(data['model']['locator'])
        checks=[audit_row(data,c,data['samples'][r['input_sequence']],r,fk) for r in rows]
        if len(rows)!=20 or {r['input_sequence'] for r in rows}!=set(range(20)):reasons.append('audit-does-not-cover-all-20-frozen-snapshots')
    else:
        checks=[dict(attempt_sequence=r['attempt_sequence'],**external_check(data,c,data['samples'][r['input_sequence']],r)) for r in rows]
        gate=c.get('native_audit_evidence',[])
        if len(gate)!=2:reasons.append('matching-native-audit-pair-unavailable')
        for source in gate:
            C.verify(source);prior=C.read_json(source['locator'])
            if prior.get('input_hash')!=req['identity']['input_hash'] or prior.get('workload')!=c['workload'] or prior.get('acceptance_contract_hash')!=COMMON_ACCEPTANCE_HASH or not prior.get('claim_eligible'):reasons.append('native-audit-gate-mismatch')
    for check in checks:reasons.extend(check.get('reasons',[]))
    complete=timing['passed'] and (not audit or len(rows)==20)
    comparable=bool(checks) and not reasons
    C.write_json(out/'common_servo_checks.json',checks)
    report=dict(schema_version='common_servo_audit.v1',acceptance_contract_hash=COMMON_ACCEPTANCE_HASH,
        method_id=req['identity']['method_id'],input_hash=req['identity']['input_hash'],workload=c['workload'],
        design_comparable=comparable,observation_complete=complete,claim_eligible=comparable and complete,
        reasons=sorted(set(reasons)),canonical_problem_hash=C.stable_hash([r['canonical_problem'] for r in checks]) if audit else None,
        source_artifacts=[C.artifact(p) for p in (request,raw,out/'common_servo_checks.json') if p.exists()]+gate,
        claim_scope='complete named method under common external contract; not isolated backend',native_failures_preserved=True)
    C.write_json(out/'common_servo_audit.json',report)
    C.write_json(out/'app_validation.json',dict(passed=comparable and complete,status='completed' if comparable and complete else 'failed',failures=report['reasons']))
    return int(not(comparable and complete))

from analysis.common import read_json, unit_json, verify
from repair.common import COMMON_ACCEPTANCE
METHODS = {'mcc': 'mcc-weighted-common-servo-v1', 'placo': 'placo-common-servo-v1'}

def audit_gate(audit_run):
    """Both named native methods need complete matching actual-math evidence per design."""
    run=Path(audit_run);inventory=read_json(run/'inventory.json');grouped={}
    for u in inventory['actual_units']:
        u=dict(u);u['directory']=str(run/'units'/u['experiment_id']/u['case_id']/u['arm_id']/str(u['repeat_id']))
        request=unit_json(u,'request.json',{});key=(u['input_hash'],request.get('config',{}).get('workload'))
        records=[a for a in u['artifacts'] if Path(a['locator']).name=='common_servo_audit.json']
        evidence=None;report={}
        if len(records)==1:verify(records[0]);evidence=records[0];report=read_json(evidence['locator'])
        grouped.setdefault(key,[]).append((u,report,evidence))
    result={}
    for key,rows in grouped.items():
        reasons=[];ids={u['method_id'] for u,_,_ in rows}
        if ids!=set(METHODS.values()) or len(rows)!=2:reasons.append('missing-or-duplicate-native-method')
        for u,a,e in rows:
            if u['status']!='completed' or u.get('validation',{}).get('status')!='passed':reasons.append(u['method_id']+':native-or-validation-not-complete')
            if a.get('method_id')!=u['method_id'] or a.get('input_hash')!=u['input_hash'] or a.get('workload')!=key[1]:reasons.append('audit-unit-identity-mismatch')
            if a.get('acceptance_contract_hash')!=COMMON_ACCEPTANCE_HASH:reasons.append('acceptance-contract-hash-mismatch')
            if not all(a.get(k) is True for k in ('design_comparable','observation_complete','claim_eligible')):reasons.append(u['method_id']+':audit-not-eligible')
        # Audit canonical digest explicitly binds all samples and physical rows to each other.
        hashes={a.get('canonical_problem_hash') for _,a,_ in rows}
        if None in hashes or len(hashes)!=1:reasons.append('cross-native-canonical-problem-mismatch')
        result[key]=dict(admitted=not reasons,verdict='admitted' if not reasons else 'unavailable',reasons=sorted(set(reasons)),
            evidence_artifacts=[e for _,_,e in rows if e],acceptance_contract_hash=COMMON_ACCEPTANCE_HASH,
            method_ids=sorted(ids),claim_scope=COMMON_ACCEPTANCE['claim_scope'])
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--request',required=True);raise SystemExit(verify(parser.parse_args().request))
