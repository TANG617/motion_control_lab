"""Independent evidence checks for public app runs; never rewrites native outcome.
Checks observable feasibility and accounting, not an undeclared performance ranking.
"""
from __future__ import annotations
import math,pathlib,xml.etree.ElementTree as ET
import numpy as np
from contracts import read,digest
TOLERANCE=1e-7

def nearest_rank(values,p):
    if not values:return None
    if not 0<p<=1:raise ValueError('percentile outside (0,1]')
    return sorted(values)[math.ceil(p*len(values))-1]

def check_bounds(q,lower,upper,tolerance=TOLERANCE):
    if q is None or len(q)==0:return {'status':'unavailable','reason':'candidate absent'}
    if len(q)!=len(lower) or len(q)!=len(upper):return {'status':'unavailable','reason':'joint mapping length mismatch'}
    if any(not math.isfinite(x) for x in q):return {'status':'failed','reason':'non-finite joint value','tolerance_rad':tolerance}
    v=max([0.]+[max(lo-x,x-hi,0.) for x,lo,hi in zip(q,lower,upper)])
    return {'status':'passed' if math.isfinite(v) and v<=tolerance else 'failed','maximum_violation_rad':v,'tolerance_rad':tolerance}

def model_bounds(data):
    tree=ET.parse(data['model']['locator']);joints={j.attrib['name']:j for j in tree.getroot().findall('joint')}
    lower=[];upper=[]
    for name in data['joint_names']:
        limit=joints[name].find('limit');lower.append(float(limit.attrib.get('lower','-inf')));upper.append(float(limit.attrib.get('upper','inf')))
    return lower,upper

def matrix_check(problem,result):
    x=result.get('candidate');lo=problem['lower'];hi=problem['upper'];bounds=check_bounds(x,lo,hi);residuals=[]
    if x:
        for i,t in enumerate(problem['tasks']):
            r=np.asarray(t['A'])@x-np.asarray(t['b']);residuals.append({'task_index':i,'enabled':t.get('enabled',True),'hard':t.get('hard',False),'raw_squared_cost':float(r@r),'weighted_squared_cost':float(t.get('weight',1.)*(r@r)),'max_residual':float(np.max(np.abs(r),initial=0.))})
    failed=result.get('accepted') and (bounds['status']=='failed' or any(t['enabled'] and t['hard'] and t['max_residual']>TOLERANCE for t in residuals))
    return {'status':'failed' if failed else 'passed','bounds':bounds,'tasks':residuals,'optimality':'not inferred from feasibility; standalone numerical-reference tests are separate','multi_solution_policy':'objective and feasibility, never joint-vector uniqueness'}

def release_accounting(planned,executed,skipped,notrun):
    return {'status':'passed' if planned==executed+skipped+notrun and min(planned,executed,skipped,notrun)>=0 else 'failed','planned':planned,'executed':executed,'skipped':skipped,'not_run':notrun,'deadline_denominator':planned,'not_run_is_not_success':True}

def verify_output(out,request,exit_code):
    out=pathlib.Path(out);issues=[];checks=[]
    if not out.is_dir():return {'status':'unavailable','native_exit_code':exit_code,'issues':['no app evidence directory; preserve process stderr']}
    if request['app_id']=='mcl_baseline':return {'status':'unavailable','native_exit_code':exit_code,'issues':['frozen baseline native traces retained; no new solver-level admission inferred']}
    if request['app_id']=='mcl_optimization_problem':
        if not (out/'result.json').is_file():return {'status':'unavailable','native_exit_code':exit_code,'issues':['native exception; no returned candidate']}
        result=read(out/'result.json');data=read(request['input']['path']);v=matrix_check(data.get('analytic',data),result);v['native_exit_code']=exit_code;return v
    raw=out/'raw.jsonl'
    replay_journal=False
    if not raw.is_file() and (out/'native_calls.jsonl').is_file():
        raw=out/'native_calls.jsonl';replay_journal=True
    if not raw.is_file():return {'status':'unavailable','native_exit_code':exit_code,'issues':['raw missing; initialization/replay-native artifacts and stderr retained']}
    import json
    rows=[json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
    source=read(request['input']['path']) if request['input']['format']=='json' else None
    source=source or {}
    mapping=read(out/'model_mapping.json') if (out/'model_mapping.json').is_file() else {}
    model_path=source.get('model',{}).get('locator') or request['app_config'].get('urdf_path') or request['app_config'].get('options',{}).get('urdf')
    names=mapping.get('joint_names',source.get('joint_names',[]))
    if model_path and names:
        lo,hi=model_bounds({'model':{'locator':model_path},'joint_names':names})
        limits=mapping.get('limits',{})
        if limits.get('lower') and limits.get('upper'):lo,hi=limits['lower'],limits['upper']
        elif mapping.get('joint_limits'):lo=[x['minimum_position'] for x in mapping['joint_limits']];hi=[x['maximum_position'] for x in mapping['joint_limits']]
        if request['app_id']=='mcl_planned_kinematics_step' and request['app_config'].get('solver')=='placo':lo,hi=source['limits']['lower'],source['limits']['upper']
        if source.get('initial_state'):checks.append({'kind':'input_feasibility','check':check_bounds(source['initial_state']['q'],lo,hi),'affects_native_status':False})
        if replay_journal and (out/'replay/trace.csv').is_file():
            import csv
            trace_path=out/'replay/trace.csv'
            trace=list(csv.DictReader(trace_path.open()))
            for i,row in enumerate(trace):
                if row.get('accepted')=='true':
                    q=[float(x) for x in row.get('otg_positions','').split(';') if x]
                    check=check_bounds(q,lo,hi)
                    checks.append({'kind':'replay_committed_position_bounds','trace_line':i+2,'check':check})
            checks.append({'kind':'native_replay_trace','check':{'status':'passed'},'sha256':digest(trace_path),'rows':len(trace),'accepted_rows':sum(r.get('accepted')=='true' for r in trace),'denominator_scope':'observed pipeline attempts; planned releases are unavailable in this legacy replay trace, not inferred from rows'})
        for i,row in enumerate(rows):
            if row.get('candidate_available') and row.get('candidate_q'):
                q=row['candidate_q'];ids=mapping.get('active_indices',[])
                boundlo,boundhi=([lo[j] for j in ids],[hi[j] for j in ids]) if len(q)!=len(lo) and len(q)==len(ids) else (lo,hi)
                checks.append({'kind':'candidate_position_bounds','raw_line':i+1,'check':check_bounds(q,boundlo,boundhi),'affects_native_status':False})
            if row.get('committed') and row.get('q'):
                q=row['q'];active=mapping.get('active_joint_names',[])
                boundlo,boundhi=lo,hi
                if len(q)!=len(lo) and (active or mapping.get('active_indices')):
                    ids=mapping.get('active_indices') or [names.index(n) for n in active];boundlo=[lo[j] for j in ids];boundhi=[hi[j] for j in ids]
                check=check_bounds(q,boundlo,boundhi);checks.append({'kind':'committed_position_bounds','raw_line':i+1,'check':check})
                if check['status']=='failed':issues.append('committed position outside declared actual limits at raw line '+str(i+1))
            if row.get('committed') and row.get('disposition')=='rejected':issues.append('rejected candidate committed at raw line '+str(i+1))
    for plan_name in ('planned_schedule.json','call_plan.json'):
        if (out/plan_name).is_file():
            plan=read(out/plan_name)
            if plan_name=='call_plan.json':
                planned=plan['planned_calls'];begun={r['call_index'] for r in rows if r.get('record_type')=='attempt_begin'}
                checks.append({'kind':'calls','check':release_accounting(planned,len(begun),0,max(0,planned-len(begun))),'unfinished_attempts':[r['call_index'] for r in rows if r.get('record_type')=='attempt_begin' and not any(x.get('call_index')==r['call_index'] and x.get('record_type')=='result' for x in rows)]})
            else:
                attempts=sum(r.get('record_type')=='attempt' for r in rows);notrun=sum(r.get('execution_state')=='not-run' for r in rows)
                checks.append({'kind':'releases','check':release_accounting(plan['planned_releases'],attempts,0,notrun)})
    if (out/'summary.json').is_file():
        summary=read(out/'summary.json')
        if 'planned_releases' in summary:checks.append({'kind':'releases','check':release_accounting(summary['planned_releases'],summary['completed_calls'],summary['skipped_releases'],summary['not_run_releases'])})
        for ref in summary.get('native_evidence',[]):
            if digest(ref['path'])!=ref['sha256']:issues.append('native evidence hash mismatch: '+ref['path'])
    if (out/'replay/release_counts.json').is_file():
        counts=read(out/'replay/release_counts.json');source_counts=counts['source']
        keys=('planned_frame_count','native_selected_frame_count','native_dropped_frame_count','not_selected_suffix_count')
        if all(source_counts.get(k) is not None for k in keys):
            total,selected,dropped,suffix=(source_counts[k] for k in keys)
            checks.append({'kind':'replay_source_coverage','check':{'status':'passed' if total==selected+dropped+suffix and min(total,selected,dropped,suffix)>=0 else 'failed'},'counts':source_counts,'scope':'input frames, not a worker deadline denominator'})
        checks.append({'kind':'replay_worker_plan','check':{'status':'unavailable' if counts['planned_worker_releases'] is None else 'passed'},'counts':counts,'scope':'preserve native callback/skip semantics; never infer an exact denominator from observed rows'})
    if any(c.get('affects_native_status',True) and c['check']['status']=='failed' for c in checks):issues.append('observable feasibility/accounting check failed')
    return {'schema_version':'app_validation.v1','status':'failed' if issues else ('passed' if checks else 'unavailable'),'native_exit_code':exit_code,'native_status_rewritten':False,'raw_sha256':digest(raw),'raw_record_count':len(rows),'checks':checks,'issues':issues,'scope':'independent bounds, native rejection/commit separation and full denominator where a finite schedule is declared; no inherited equation parity or scientific superiority','unavailable_checks':['new cross-method task-equation admission; formal RT timing']}
