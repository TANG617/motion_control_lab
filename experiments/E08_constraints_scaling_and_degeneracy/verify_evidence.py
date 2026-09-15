#!/usr/bin/env python3
"""Independent geometric and fixed-step derivative evidence; never changes native status."""
from __future__ import annotations
import argparse
import collections
import json
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from analysis import common as C
from repair.kinematics import GeometricFk
from progress import SampleProgress

FD_EPS=(1e-5,1e-6,1e-7)


def maximum(value):
    value=np.asarray(value,dtype=float)
    return float(np.max(np.abs(value))) if value.size else None


def violation(value,lower,upper):
    x=np.asarray(value,dtype=float);lo=np.asarray(lower,dtype=float);hi=np.asarray(upper,dtype=float)
    if x.shape!=lo.shape or x.shape!=hi.shape or not np.all(np.isfinite(x)):return None
    return float(np.max(np.maximum(np.maximum(lo-x,x-hi),0))) if x.size else None


def residual(A,b,baseline,candidate,scale):
    A=np.asarray(A,dtype=float);b=np.asarray(b,dtype=float)
    if candidate is None or scale is None:return None
    x=np.asarray(candidate,dtype=float);ref=np.asarray(baseline,dtype=float)
    if A.ndim!=2 or x.shape!=(A.shape[1],) or ref.shape!=x.shape or b.shape!=(A.shape[0],):return None
    if not all(np.all(np.isfinite(v)) for v in (A,b,x,ref)) or not np.isfinite(scale):return None
    return A@(x-ref)-scale*(b-A@ref)


def residual_record(A,b,reference,candidate,scale,tolerance):
    r=residual(A,b,reference,candidate,scale)
    return {'residual':r.tolist() if r is not None else None,'maximum':maximum(r) if r is not None else None,
            'within_original_tolerance':bool(np.all(np.abs(r)<=tolerance)) if r is not None else None}


def state_evidence(row,data,config):
    names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']]
    limits=data['limits'];lo=np.asarray(limits['lower']);hi=np.asarray(limits['upper']);vmax=np.asarray(limits['velocity'])
    state=row['input_state'];q0=np.asarray(state['q']);v0=np.asarray(state['v']);q=np.asarray(row['q']);v=np.asarray(row['v'])
    lin=row.get('evidence_linearization',{});available=bool(lin.get('candidate_available'))
    candidate=lin.get('candidate_values') if available else None
    raw_q=row.get('candidate_active_q',[]);raw_v=row.get('candidate_active_v',[])
    committed=row.get('committed') is True
    tol=float(config['hard_tolerance']);dt=float(config['period_s'])
    native_candidate_violation=violation(candidate,lin.get('joint_bounds',{}).get('lower',[]),lin.get('joint_bounds',{}).get('upper',[])) if available else None
    return dict(attempt_sequence=row['attempt_sequence'],source_time_s=row.get('source_time_s'),
        native_disposition=row.get('disposition'),committed=committed,
        input_position_violation_rad=violation(q0,lo,hi),input_velocity_violation_rad_s=violation(v0,-vmax,vmax),
        returned_position_violation_rad=violation(q,lo,hi),returned_velocity_violation_rad_s=violation(v,-vmax,vmax),
        returned_solution_position_violation_rad=violation(raw_q,lo[active],hi[active]),
        returned_solution_velocity_violation_rad_s=violation(raw_v,-vmax[active],vmax[active]),
        native_selected_candidate_available=available,native_selected_candidate_bound_violation_rad_s=native_candidate_violation,
        rejected_input_state_held=bool(np.array_equal(q,q0) and np.array_equal(v,v0)) if not committed else None,
        committed_position_violation_rad=violation(q,lo,hi) if committed else None,
        committed_position_within_original_tolerance=bool(violation(q,lo,hi)<=tol*dt) if committed else None,
        committed_velocity_within_original_tolerance=bool(violation(v,-vmax,vmax)<=tol) if committed else None,
        original_velocity_hard_tolerance=tol,original_integrated_position_tolerance_rad=tol*dt,
        selected_priority=row.get('selected_priority'),completed_passes=row.get('completed_passes'),
        interpretation='input / returned state / native selected candidate / commit are separate; a rejected placeholder is not a candidate')


def position_tasks(row,config,active):
    """Normalize recorded row units, retaining native matrices and increment convention."""
    lin=row.get('evidence_linearization',{})
    if lin.get('available'):
        for task in lin.get('tasks',[]):
            if task.get('name') not in ('left-position','right-position') or not task.get('enabled'):continue
            result=dict(task);result['b']=(np.asarray(task['lower'])-np.asarray(task['offset'])).tolist()
            result['native_row_source']='Core assembled LinearRequirement before optimization'
            result['native_candidate']=lin.get('candidate_values') if lin.get('candidate_available') else None
            yield result
    elif 'placo' in config['evidence_original_method']:
        for task in row.get('native_task_rows',[]):
            A=np.asarray(task['A'])[:,row['active_variable_columns']]
            yield dict(name=task['name'],A=A.tolist(),b=(np.asarray(task['b'])/config['period_s']).tolist(),
                       enabled=True,enforcement=config['enforcement'],priority=None,
                       native_candidate=np.asarray(row['v'])[active].tolist() if row.get('committed') else None,
                       native_row_source='PlaCo position A and increment b; b divided by original dt for velocity units')


def audit(request):
    request_path=Path(request);req=C.read_json(request_path);out=Path(req['output_dir']);data=req['input'];config=req['config']
    if tuple(config['evidence_fd_eps'])!=FD_EPS:raise ValueError('finite difference steps are frozen; no tuning')
    C.verify(data['model']);fk=GeometricFk(data['model']['locator']);names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']]
    expected=int(config['steps']);counter=collections.Counter();fd_cache={};previous_revision=None
    raw=out/'raw.jsonl';sources=[C.artifact(request_path),C.artifact(raw)] if raw.exists() else [C.artifact(request_path)]
    progress=SampleProgress(out,'e08-evidence',expected)
    table_names=('e08_native_geometry','e08_fixed_fd','e08_state_evidence','e08_pass_evidence')
    streams={name:(out/(name+'.jsonl')).open('w') for name in table_names}
    def emit(name,row):streams[name].write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n')
    try:
        if raw.exists():
            for line,row in enumerate(C.jsonl_rows(raw),1):
                if row.get('record_type')=='attempt_started':counter['started']+=1;continue
                if row.get('record_type')!='attempt':continue
                counter['attempts']+=1;seq=row['attempt_sequence'];counter['committed']+=bool(row.get('committed'))
                state=state_evidence(row,data,config);state['source_line']=line;emit('e08_state_evidence',state)
                if not row.get('committed') and not state['rejected_input_state_held']:counter['rejected_state_changed']+=1
                if state['input_position_violation_rad'] and not row.get('committed'):counter['initial_invalid_rejected']+=1
                if row.get('committed') and state['committed_position_within_original_tolerance'] is False:counter['committed_position_exceedances']+=1
                if row.get('committed') and state['committed_velocity_within_original_tolerance'] is False:counter['committed_velocity_exceedances']+=1
                if row.get('committed') and state['native_selected_candidate_bound_violation_rad_s'] is not None and state['native_selected_candidate_bound_violation_rad_s']>config['hard_tolerance']:counter['committed_native_bound_exceedances']+=1
                for p in row.get('passes',[]):
                    emit('e08_pass_evidence',dict(source_line=line,attempt_sequence=seq,selected_priority=row.get('selected_priority'),
                        committed=row.get('committed'),**p))
                q=np.asarray(row['input_state']['q']);baseline=np.asarray(row.get('task_scale_reference',[0.]*len(active)))
                if config['enforcement']!='scaled':baseline=np.zeros(len(active))
                task_rows=list(position_tasks(row,config,active))
                # Early native infeasibility can leave no assembled rows. This absence is
                # explicit evidence, not a synthesized empty feasible task.
                if not task_rows:counter['attempts_without_native_rows']+=1
                for task in task_rows:
                    counter['native_task_rows']+=1;side=task['name'].split('-')[0];A=np.asarray(task['A']);b=np.asarray(task['b'])
                    frame=data['frames'][side];target=row['targets'][side]
                    pose=fk.pose(names,q,frame,tcp=(0,0,0),root=data['root_frame'])
                    geo=fk.jacobian(names,q,frame,tcp=(0,0,0))[:,active]
                    goal=np.asarray(target['position'])-np.asarray(target['rotation'])@np.asarray(data['tcp_offsets'][side])
                    desired=config['gain_per_s']*(goal-np.asarray(pose['position']))
                    scale=1.
                    if task['enforcement']=='scaled':
                        handle=task.get('scale_group_handle')
                        value=next((s for s in row.get('scales',[]) if s.get('handle')==handle and s.get('evaluated')),None)
                        scale=value['value'] if value is not None else None
                    candidate=task['native_candidate'];tol=config['hard_tolerance']
                    if task['enforcement']=='scaled' and candidate is not None and scale is None:counter['native_candidate_scale_unavailable']+=1
                    native=residual_record(A,b,baseline,candidate,scale,tol)
                    geometric=residual_record(geo,desired,baseline,candidate,scale,tol)
                    legacy=residual_record(geo,desired,baseline,row.get('candidate_active_v'),scale,tol)
                    disagreement=native['within_original_tolerance'] is not None and geometric['within_original_tolerance'] is not None and native['within_original_tolerance']!=geometric['within_original_tolerance']
                    info=dict(source_line=line,attempt_sequence=seq,source_time_s=row.get('source_time_s'),task=task['name'],side=side,
                        native_disposition=row.get('disposition'),committed=row.get('committed'),selected_priority=row.get('selected_priority'),
                        native_configuration=row.get('evidence_linearization',{}).get('configuration'),input_linearization_q=q.tolist(),
                        native_row_source=task['native_row_source'],native_A=A.tolist(),native_b=b.tolist(),geometric_A=geo.tolist(),geometric_b=desired.tolist(),
                        max_A_difference=maximum(A-geo),max_b_difference=maximum(b-desired),baseline_velocity=baseline.tolist(),scale=scale,
                        native_selected_candidate_available=candidate is not None,native_candidate=candidate,native_residual=native,geometric_residual=geometric,
                        legacy_returned_solution_residual=legacy,legacy_residual_role='diagnostic of old verifier operands; rejected returned state is not a native candidate',
                        original_hard_tolerance=tol,tolerance_disagreement=disagreement,task_enforcement=task['enforcement'])
                    emit('e08_native_geometry',info)
                    if disagreement:counter['native_geometric_tolerance_disagreements']+=1
                    if row.get('committed') and task['enforcement']=='scaled' and native['within_original_tolerance'] is False:counter['committed_native_scaled_exceedances']+=1
                    if row.get('committed') and task['enforcement']=='scaled' and geometric['within_original_tolerance'] is False:counter['committed_geometric_scaled_exceedances']+=1
                    if not row.get('committed') and legacy['within_original_tolerance'] is False:counter['rejected_returned_state_equation_exceedances']+=1
                    # Cross-check every distinct state where native and geometric threshold
                    # verdicts disagree, plus declared boundary observations. No epsilon scan
                    # is used to choose a favorable derivative or alter tolerance.
                    boundary=seq in (0,expected-1) or row.get('task_revision')!=previous_revision
                    if boundary or disagreement:
                        key=(side,q.tobytes())
                        if key not in fd_cache:
                            fd_cache[key]=[(eps,fk.finite_difference(names,q,frame,tcp=(0,0,0),eps=eps)[:,active]) for eps in FD_EPS]
                        for eps,fd in fd_cache[key]:
                            fdr=residual_record(fd,desired,baseline,candidate,scale,tol)
                            emit('e08_fixed_fd',dict(source_line=line,attempt_sequence=seq,source_time_s=row.get('source_time_s'),task=task['name'],
                                epsilon=eps,selection='native-geometric-tolerance-disagreement' if disagreement else 'first,last,task-revision-boundary',
                                native_disposition=row.get('disposition'),committed=row.get('committed'),max_fd_geometric_A_difference=maximum(fd-geo),
                                max_fd_native_A_difference=maximum(fd-A),fixed_fd_A=fd.tolist(),fixed_fd_residual=fdr,
                                native_residual=native,geometric_residual=geometric,original_hard_tolerance=tol))
                            counter['fixed_fd_rows']+=1
                previous_revision=row.get('task_revision');progress.update(counter['attempts'])
    finally:
        for stream in streams.values():stream.close()
    complete=counter['attempts']==expected and not counter['rejected_state_changed']
    expected_crash=config.get('evidence_class')=='initial-infeasible-reproduction' and 'placo' in config['evidence_original_method']
    summary=dict(schema_version='e08_evidence_audit.v1',observation_complete=complete,expected_attempts=expected,
        counts=dict(counter),expected_failure_case=expected_crash,native_failure_preserved=True,original_validation_unchanged=True,
        classification='native-crash-with-begin-only-evidence' if counter['attempts']==0 and counter['started'] else 'measured' if complete else 'incomplete',
        finite_difference_eps=list(FD_EPS),sources=sources,
        claim_eligible=bool(complete and counter['committed'] and not counter['initial_invalid_rejected'] and not counter['committed_position_exceedances'] and not counter['committed_velocity_exceedances'] and not counter['committed_native_bound_exceedances'] and not counter['committed_native_scaled_exceedances'] and not counter['committed_geometric_scaled_exceedances']),
        native_algorithm_fix=False,threshold_changes=False)
    if (counter['attempts_without_native_rows'] and config['enforcement']=='scaled') or counter['native_candidate_scale_unavailable']:
        summary['observation_complete']=False;summary['claim_eligible']=False
    failed=bool(counter['rejected_state_changed'] or counter['committed_position_exceedances'] or counter['committed_velocity_exceedances'] or counter['committed_native_bound_exceedances'] or counter['committed_native_scaled_exceedances'] or counter['committed_geometric_scaled_exceedances'])
    summary['validation_failed']=failed
    C.write_json(out/'e08_evidence_summary.json',summary)
    C.write_json(out/'app_validation.json',dict(status='failed' if failed else 'completed' if summary['observation_complete'] else 'unavailable',
        purpose='geometric/native hard equations at original thresholds; evidence completeness and candidate status separate',
        observed_anomalies=dict(counter),claim_eligible=summary['claim_eligible']))
    return 1 if failed else 0 if summary['observation_complete'] else 2


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--request',required=True)
    args=parser.parse_args();raise SystemExit(audit(args.request))
