"""A01 v2 E08 supplemental diagnostics; immutable original and new counts stay separate."""
from __future__ import annotations
import collections
import json
from pathlib import Path
import math
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/mcc_placo_study'))
from analysis import common as C


def records(unit,name):
    path=C.unit_path(unit,name)
    if path is not None:yield from C.jsonl_rows(path)


def value_max(old,new):
    if new is None:return old
    return new if old is None else max(old,new)


def old_issues(units):
    """Original failure meanings are preserved, never overwritten by supplemental data."""
    for unit in units:
        if unit['experiment_id']!='E08' or unit['status']=='unavailable':continue
        if unit['status']=='completed' and unit['validation']['status']=='passed':continue
        counter=collections.Counter();largest=None;first=None
        scale=C.unit_path(unit,'scale_equation_checks.json')
        if scale:
            for check in C.array_rows(scale):
                if check.get('status')!='failed':continue
                counter[check.get('native_disposition','unknown')]+=1
                maximum=max((abs(x) for x in check.get('equation_residual',[])),default=None)
                largest=value_max(largest,maximum)
                if first is None:first=check.get('source_line')
        kind='scale-equation' if counter else 'native-crash' if unit['status']=='crashed' else 'initial-infeasible-state' if unit['case_id']=='initial_infeasible' else 'unresolved'
        yield dict(C.base_row(unit),issue_kind=kind,original_failure_unchanged=True,
            original_scale_failure_dispositions=dict(counter),original_maximum_scale_residual=largest,
            first_original_failed_raw_line=first,original_validation_locator=str(C.unit_path(unit,'validation.json')),
            original_scale_checks_locator=str(scale) if scale else None)


FINAL_SCALE_VERSION='e08-final-scale-reanalysis.v1'
FINAL_FIELDS=('metric_version','unit_id','case_id','method_id','original_method','task','attempt_sequence','source_time_s',
    'source_line','geometry_line','native_disposition','committed','selected_priority','selected_pass_evidence',
    'native_selected_candidate_available','primary_optimum_scale','final_selected_scale','scale_source','status','reason',
    'original_hard_tolerance','old_verifier_native_maximum','old_verifier_geometric_maximum','old_returned_state_maximum',
    'corrected_native_residual','corrected_geometric_residual','corrected_native_maximum','corrected_geometric_maximum',
    'corrected_native_exceeds','corrected_geometric_exceeds','old_optimum_false_positive','native_geometric_disagreement',
    'native_constraint_value','native_constraint_maximum_violation','raw_locator','raw_sha256','geometry_locator','geometry_sha256')


def exact_residual(A,b,baseline,candidate,scale):
    import numpy as np
    A=np.asarray(A,dtype=float);b=np.asarray(b,dtype=float);baseline=np.asarray(baseline,dtype=float);candidate=np.asarray(candidate,dtype=float)
    if A.ndim!=2 or candidate.shape!=(A.shape[1],) or baseline.shape!=candidate.shape or b.shape!=(A.shape[0],):return None
    if not all(np.all(np.isfinite(a)) for a in (A,b,baseline,candidate)) or not math.isfinite(scale):return None
    return (A@(candidate-baseline)-scale*(b-A@baseline)).tolist()


def correct_final_scale(raw,geometry,method):
    """Use exact same-tick selected scale; never infer sign from absolute drift."""
    row=dict(metric_version=FINAL_SCALE_VERSION,task=geometry['task'],attempt_sequence=raw['attempt_sequence'],
        source_time_s=raw.get('source_time_s'),native_disposition=raw.get('disposition'),committed=raw.get('committed') is True,
        selected_priority=raw.get('selected_priority'),native_selected_candidate_available=raw.get('evidence_linearization',{}).get('candidate_available',False),
        primary_optimum_scale=geometry.get('scale'),old_verifier_native_maximum=geometry['native_residual']['maximum'],
        old_verifier_geometric_maximum=geometry['geometric_residual']['maximum'],old_returned_state_maximum=geometry['legacy_returned_solution_residual']['maximum'],
        original_hard_tolerance=geometry['original_hard_tolerance'],status='insufficient',reason=None)
    lin=raw.get('evidence_linearization',{})
    if not lin.get('candidate_available'):
        row.update(status='native-rejection-no-candidate' if not row['committed'] else 'insufficient',
                   reason='returned solution is not a native optimization candidate' if not row['committed'] else 'committed candidate capture missing')
        return row
    tasks=[t for t in lin.get('tasks',[]) if t['name']==geometry['task'] and t.get('enabled') and t.get('enforcement')=='scaled']
    if len(tasks)!=1:row['reason']='matching scaled native task missing or duplicate';return row
    task=tasks[0]
    scales=[s for s in raw.get('scales',[]) if s.get('handle')==task['scale_group_handle'] and s.get('evaluated')]
    if len(scales)!=1:row['reason']='scale identity missing or duplicate';return row
    scale=scales[0]
    if 'hqp' in method:
        passes=[p for p in raw.get('passes',[]) if p.get('pass')==row['selected_priority'] and p.get('attempted') and p.get('succeeded')]
        if row['selected_priority'] is None or len(passes)!=1:row['reason']='selected completed native pass unavailable';return row
        row['selected_pass_evidence']={k:passes[0].get(k) for k in ('pass','attempted','succeeded','native_qp_status','backend_status')}
        final=[c for c in raw.get('native_constraints',[]) if c.get('name')==scale['name'] and c.get('component_name')=='scale']
        if len(final)!=1:row['reason']='exact final selected scale unavailable; absolute drift cannot reconstruct it';return row
        final_scale=final[0]['value'];row['scale_source']='native_constraints exact selected-candidate scale value'
    else:
        if raw.get('completed_passes')!=1:row['reason']='weighted final successful solve unavailable';return row
        final_scale=scale['value'];row['scale_source']='weighted final OptimizationDiagnostics.task_scales.scale'
        row['selected_pass_evidence']={'pass':'weighted-only','native_qp_status':raw.get('native_qp_status')}
    row['final_selected_scale']=final_scale
    native_candidate=lin['candidate_values']
    if geometry.get('native_candidate')!=native_candidate:row['reason']='same-tick candidate does not match frozen geometric evidence';return row
    # Native rows use their own offset and target; independent rows use independent
    # geometric target terms. Both use the same native candidate and final scale.
    import numpy as np
    native_b=(np.asarray(task['lower'])-np.asarray(task['offset'])).tolist()
    native=exact_residual(task['A'],native_b,raw['task_scale_reference'],native_candidate,final_scale)
    geometric=exact_residual(geometry['geometric_A'],geometry['geometric_b'],raw['task_scale_reference'],native_candidate,final_scale)
    if native is None or geometric is None:row['reason']='nonfinite or incomplete selected-candidate equation';return row
    tol=row['original_hard_tolerance'];nmax=max(map(abs,native));gmax=max(map(abs,geometric))
    row.update(status='measured',corrected_native_residual=native,corrected_geometric_residual=geometric,
        corrected_native_maximum=nmax,corrected_geometric_maximum=gmax,corrected_native_exceeds=nmax>tol,
        corrected_geometric_exceeds=gmax>tol,native_geometric_disagreement=(nmax>tol)!=(gmax>tol),
        old_optimum_false_positive=bool(row['old_verifier_native_maximum'] is not None and row['old_verifier_native_maximum']>tol and nmax<=tol),reason=None)
    constraint=[c for c in raw.get('native_constraints',[]) if c.get('name')==task['name']]
    if len(constraint)==1:
        row['native_constraint_value']=constraint[0].get('value');row['native_constraint_maximum_violation']=constraint[0].get('maximum_violation')
    return row


def final_scale_reanalysis(ctx):
    """Stream all rows with source hashes; preserve all original verifier artifacts."""
    aggregates={};events=[];fixed=[]
    def generate():
        for unit in sorted(ctx.supplemental_units,key=C.unit_id):
            if unit['experiment_id']!='E08' or unit['config'].get('enforcement')!='scaled':continue
            uid=C.unit_id(unit);raw_path=C.unit_path(unit,'raw.jsonl');geom_path=C.unit_path(unit,'e08_native_geometry.jsonl')
            if raw_path is None or geom_path is None:continue
            art={a['locator']:a for a in unit['artifacts']}
            fd_path=C.unit_path(unit,'e08_fixed_fd.jsonl');fd_lookup=collections.defaultdict(list)
            if fd_path:
                for number,fd in enumerate(C.jsonl_rows(fd_path),1):fd_lookup[(fd['source_line'],fd['task'])].append((number,fd))
            geo_iter=iter(enumerate(C.jsonl_rows(geom_path),1));next_geo=next(geo_iter,None)
            for source_line,raw in enumerate(C.jsonl_rows(raw_path),1):
                if raw.get('record_type')!='attempt':continue
                while next_geo is not None and next_geo[1]['source_line']<source_line:raise ValueError('unmatched earlier geometric evidence')
                while next_geo is not None and next_geo[1]['source_line']==source_line:
                    geometry_line,geometry=next_geo;next_geo=next(geo_iter,None)
                    if geometry['attempt_sequence']!=raw['attempt_sequence']:raise ValueError('cross-attempt geometric evidence')
                    row=correct_final_scale(raw,geometry,unit['config']['evidence_original_method'])
                    row.update(unit_id=uid,case_id=unit['case_id'],method_id=unit['method_id'],original_method=unit['config']['evidence_original_method'],
                        source_line=source_line,geometry_line=geometry_line,raw_locator=str(raw_path),raw_sha256=art[str(raw_path)]['sha256'],
                        geometry_locator=str(geom_path),geometry_sha256=art[str(geom_path)]['sha256'])
                    key=(uid,row['task']);a=aggregates.setdefault(key,dict(unit_id=uid,case_id=unit['case_id'],method_id=unit['method_id'],
                        original_method=row['original_method'],task=row['task'],metric_version=FINAL_SCALE_VERSION,rows=0,
                        measured_rows=0,insufficient_rows=0,rejected_without_candidate_rows=0,committed_final_native_exceedances=0,
                        committed_final_geometric_exceedances=0,old_optimum_false_positives=0,native_geometric_disagreements=0,
                        final_native_maximum=None,final_geometric_maximum=None,old_optimum_native_maximum=None,original_hard_tolerance=row['original_hard_tolerance']))
                    a['rows']+=1;a['measured_rows']+=row['status']=='measured';a['insufficient_rows']+=row['status']=='insufficient'
                    a['rejected_without_candidate_rows']+=row['status']=='native-rejection-no-candidate'
                    a['committed_final_native_exceedances']+=bool(row['committed'] and row.get('corrected_native_exceeds'))
                    a['committed_final_geometric_exceedances']+=bool(row['committed'] and row.get('corrected_geometric_exceeds'))
                    a['old_optimum_false_positives']+=bool(row.get('old_optimum_false_positive'))
                    a['native_geometric_disagreements']+=bool(row.get('native_geometric_disagreement'))
                    for target,source in [('final_native_maximum','corrected_native_maximum'),('final_geometric_maximum','corrected_geometric_maximum'),('old_optimum_native_maximum','old_verifier_native_maximum')]:a[target]=value_max(a[target],row.get(source))
                    if row['status']=='insufficient' or row.get('corrected_native_exceeds') or row.get('corrected_geometric_exceeds') or row.get('old_optimum_false_positive'):
                        events.append(row)
                    for fd_line,fd in fd_lookup.get((source_line,geometry['task']),[]):
                        corrected=None
                        if row['status']=='measured':corrected=exact_residual(fd['fixed_fd_A'],geometry['geometric_b'],raw['task_scale_reference'],raw['evidence_linearization']['candidate_values'],row['final_selected_scale'])
                        fixed.append(dict(unit_id=uid,case_id=unit['case_id'],method_id=unit['method_id'],task=row['task'],metric_version=FINAL_SCALE_VERSION,
                            source_line=source_line,attempt_sequence=raw['attempt_sequence'],source_time_s=raw.get('source_time_s'),epsilon=fd['epsilon'],
                            primary_optimum_scale=row['primary_optimum_scale'],final_selected_scale=row.get('final_selected_scale'),
                            old_fd_residual=fd['fixed_fd_residual'],corrected_fd_residual=corrected,
                            corrected_fd_maximum=max(map(abs,corrected)) if corrected is not None else None,
                            corrected_fd_exceeds=max(map(abs,corrected))>row['original_hard_tolerance'] if corrected is not None else None,
                            final_native_maximum=row.get('corrected_native_maximum'),final_geometric_maximum=row.get('corrected_geometric_maximum'),
                            original_hard_tolerance=row['original_hard_tolerance'],source_fd_line=fd_line,
                            source_fd_locator=str(fd_path),source_fd_sha256=art[str(fd_path)]['sha256'],raw_locator=str(raw_path),raw_sha256=art[str(raw_path)]['sha256']))
                    yield {k:row.get(k) for k in FINAL_FIELDS}
            if next_geo is not None:raise ValueError('unmatched remaining geometric evidence')
    C.write_csv(ctx.output/'evaluation/e08_final_scale_rows.csv',generate(),FINAL_FIELDS)
    rows=[aggregates[key] for key in sorted(aggregates)]
    ctx.table('e08_final_scale_summary',rows);ctx.table('e08_final_scale_events',events);ctx.table('e08_final_scale_fixed_fd',fixed)
    return rows


def original_final_row(raw,check,config):
    """Reassess a failed ORIGINAL check using only that original attempt's operands."""
    row=dict(metric_version=FINAL_SCALE_VERSION,source_layer='original-failed-check',task=check['side']+'-position',
        source_line=check['source_line'],attempt_sequence=raw['attempt_sequence'],source_time_s=raw.get('source_time_s'),
        committed=raw.get('committed') is True,native_disposition=raw.get('disposition'),native_status=raw.get('status'),
        native_qp_status=raw.get('native_qp_status'),selected_priority=raw.get('selected_priority'),
        primary_optimum_scale=check.get('scale'),original_hard_tolerance=check['hard_tolerance'],
        old_verifier_residual=check['equation_residual'],old_verifier_maximum=max(map(abs,check['equation_residual'])),
        native_matrix_status='unavailable-original-trace',status='insufficient',reason=None,
        returned_state_held=raw.get('q')==raw.get('input_state',{}).get('q') and raw.get('v')==raw.get('input_state',{}).get('v'),
        returned_solution_is_zero=bool(raw.get('candidate_active_v')) and all(v==0 for v in raw['candidate_active_v']))
    if check.get('native_disposition')!=raw.get('disposition'):raise ValueError('original check disposition differs from exact raw attempt')
    if not row['committed']:
        row.update(status='native-rejection-no-committed-candidate',reason='old trace has no native failed iterate; returned vector is not evidence of optimizer candidate')
        return row
    selected=raw.get('selected_priority')
    passes=[p for p in raw.get('passes',[]) if selected is not None and p.get('pass')==selected and p.get('attempted') and p.get('succeeded')]
    if len(passes)!=1:row['reason']='selected completed original pass unavailable';return row
    row['selected_pass_evidence']=passes[0]
    group=0 if config['scale_group']=='shared' or check['side']=='left' else 1
    name='progress-'+str(group)
    optimum=[s for s in raw.get('scales',[]) if s.get('name')==name and s.get('evaluated')]
    final=[s for s in raw.get('native_constraints',[]) if s.get('name')==name and s.get('component_name')=='scale']
    if len(optimum)!=1 or len(final)!=1:row['reason']='original exact final scale absent or duplicate';return row
    if optimum[0]['value']!=check['scale']:raise ValueError('original check primary scale differs from exact raw attempt')
    if raw.get('task_scale_reference')!=check['baseline_velocity']:raise ValueError('original check baseline differs from exact raw attempt')
    scale=final[0]['value'];row['final_selected_scale']=scale;row['scale_source']='original raw native_constraints '+name
    r=exact_residual(check['independent_J'],check['desired_velocity'],check['baseline_velocity'],raw['candidate_active_v'],scale)
    if r is None:row['reason']='original selected candidate equation incomplete or nonfinite';return row
    row.update(status='measured-original-fd',corrected_original_fd_residual=r,corrected_original_fd_maximum=max(map(abs,r)),
        corrected_original_fd_exceeds=max(map(abs,r))>check['hard_tolerance'])
    constraints=[c for c in raw.get('native_constraints',[]) if c.get('name')==row['task']]
    if len(constraints)==1:
        c=constraints[0];row.update(recorded_native_worst_component=c['component_name'],recorded_native_value=c['value'],
            recorded_native_maximum_violation=c['maximum_violation'],recorded_native_state=c['state'],
            recorded_native_exceeds=abs(c['value'])>check['hard_tolerance'],recorded_native_maximum=abs(c['value']))
    return row


def original_final_scale_reanalysis(ctx):
    """The 12 old units are evaluated independently of any supplemental trajectory."""
    from repair.kinematics import GeometricFk
    import numpy as np
    summaries=[];rows=[];fds=[]
    for unit in sorted(ctx.units,key=C.unit_id):
        if unit['experiment_id']!='E08' or unit['status']=='unavailable' or unit['validation']['status']=='passed':continue
        path=C.unit_path(unit,'scale_equation_checks.json');rawpath=C.unit_path(unit,'raw.jsonl')
        if path is None or rawpath is None:continue
        # Failed checks are bounded (12 units); retain scalar evidence for each row,
        # stream the large raw trajectory, and never mix a new run's candidate.
        failed=collections.defaultdict(list)
        for number,check in enumerate(C.array_rows(path),1):
            if check.get('status')=='failed':failed[check['source_line']].append((number,check))
        if not failed:continue
        art={a['locator']:a for a in unit['artifacts']};request=C.unit_json(unit,'request.json');fk=None
        summary=dict(unit_id=C.unit_id(unit),case_id=unit['case_id'],method_id=unit['method_id'],source_layer='original-run',
            metric_version=FINAL_SCALE_VERSION,failed_check_rows=sum(map(len,failed.values())),committed_final_native_exceedances=0,
            committed_final_geometric_exceedances=0,old_optimum_false_positives=0,rejected_without_candidate_rows=0,
            rejected_hold_rows=0,insufficient_rows=0,final_native_maximum=None,final_geometric_maximum=None,
            original_hard_tolerance=unit['config']['hard_tolerance'],native_matrix_status='unavailable-original-trace')
        for line,raw in enumerate(C.jsonl_rows(rawpath),1):
            if line not in failed:continue
            if raw.get('record_type')!='attempt':raise ValueError('original scale check does not address an attempt')
            for check_number,check in failed.pop(line):
                row=original_final_row(raw,check,unit['config'])
                row.update(unit_id=C.unit_id(unit),case_id=unit['case_id'],method_id=unit['method_id'],
                    original_check_array_index_1based=check_number,raw_locator=str(rawpath),raw_sha256=art[str(rawpath)]['sha256'],
                    old_checks_locator=str(path),old_checks_sha256=art[str(path)]['sha256'])
                if row['status']=='measured-original-fd' and request:
                    data=request['input'];names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']]
                    if fk is None:C.verify(data['model']);fk=GeometricFk(data['model']['locator'])
                    side=check['side'];q=np.asarray(raw['input_state']['q']);frame=data['frames'][side]
                    pose=fk.pose(names,q,frame,tcp=(0,0,0),root=data['root_frame']);target=raw['targets'][side]
                    A=fk.jacobian(names,q,frame,tcp=(0,0,0))[:,active]
                    b=unit['config']['gain_per_s']*(np.asarray(target['position'])-np.asarray(target['rotation'])@np.asarray(data['tcp_offsets'][side])-np.asarray(pose['position']))
                    r=exact_residual(A,b,raw['task_scale_reference'],raw['candidate_active_v'],row['final_selected_scale'])
                    primary_r=exact_residual(A,b,raw['task_scale_reference'],raw['candidate_active_v'],row['primary_optimum_scale'])
                    row.update(primary_scale_geometric_maximum=max(map(abs,primary_r)),
                        primary_scale_geometric_exceeds=max(map(abs,primary_r))>row['original_hard_tolerance'],corrected_geometric_residual=r,corrected_geometric_maximum=max(map(abs,r)),
                        corrected_geometric_exceeds=max(map(abs,r))>row['original_hard_tolerance'],
                        model_locator=data['model']['locator'],model_sha256=data['model']['sha256'],
                        request_locator=str(C.unit_path(unit,'request.json')),request_sha256=art[str(C.unit_path(unit,'request.json'))]['sha256'])
                    # Every original committed anomaly receives all three fixed steps.
                    for eps in (1e-5,1e-6,1e-7):
                        fd=fk.finite_difference(names,q,frame,tcp=(0,0,0),eps=eps)[:,active]
                        rr=exact_residual(fd,b,raw['task_scale_reference'],raw['candidate_active_v'],row['final_selected_scale'])
                        fds.append(dict(unit_id=C.unit_id(unit),case_id=unit['case_id'],method_id=unit['method_id'],
                            task=row['task'],source_line=line,attempt_sequence=raw['attempt_sequence'],epsilon=eps,
                            final_selected_scale=row['final_selected_scale'],corrected_fd_residual=rr,
                            corrected_fd_maximum=max(map(abs,rr)),corrected_fd_exceeds=max(map(abs,rr))>row['original_hard_tolerance'],
                            original_hard_tolerance=row['original_hard_tolerance'],metric_version=FINAL_SCALE_VERSION,
                            raw_locator=str(rawpath),raw_sha256=art[str(rawpath)]['sha256']))
                if row['status']=='native-rejection-no-committed-candidate':
                    summary['rejected_without_candidate_rows']+=1;summary['rejected_hold_rows']+=row['returned_state_held']
                elif row['status']=='insufficient' or row.get('recorded_native_exceeds') is None or row.get('corrected_geometric_exceeds') is None:
                    summary['insufficient_rows']+=1
                else:
                    summary['committed_final_native_exceedances']+=row['recorded_native_exceeds']
                    summary['committed_final_geometric_exceedances']+=row['corrected_geometric_exceeds']
                    row['old_optimum_false_positive']=row['primary_scale_geometric_exceeds'] and not row['recorded_native_exceeds'] and not row['corrected_geometric_exceeds']
                    summary['old_optimum_false_positives']+=row['old_optimum_false_positive']
                    summary['final_native_maximum']=value_max(summary['final_native_maximum'],row.get('recorded_native_maximum'))
                    summary['final_geometric_maximum']=value_max(summary['final_geometric_maximum'],row.get('corrected_geometric_maximum'))
                rows.append(row)
            if not failed:break
        if failed:raise ValueError('original failed check raw line unavailable')
        summaries.append(summary)
    ctx.table('e08_original_final_scale_rows',rows);ctx.table('e08_original_final_scale_summary',summaries);ctx.table('e08_original_final_scale_fixed_fd',fds)
    return summaries


def analyze(ctx):
    old=list(old_issues(ctx.units));units=[];geometry=[];states=[];fd=[];exceedances=[];links=[]
    grouped=collections.defaultdict(list)
    for unit in sorted(ctx.supplemental_units,key=C.unit_id):
        if unit['experiment_id']!='E08':continue
        config=unit['config'];base=C.base_row(unit)
        base.update(repair_stage=unit['repair_stage'],evidence_class=config.get('evidence_class'),
            original_case=config.get('evidence_original_case'),original_method=config.get('evidence_original_method'),
            source_layer='supplemental-new-run',original_validation_not_replaced=True)
        audit=C.unit_json(unit,'e08_evidence_summary.json',{})
        units.append(dict(base,observation_complete=audit.get('observation_complete',False),
            candidate_claim_eligible=audit.get('claim_eligible',False),audit_classification=audit.get('classification','unavailable'),
            counts=audit.get('counts',{}),audit_locator=str(C.unit_path(unit,'e08_evidence_summary.json'))))
        grouped[(base['original_case'],base['original_method'])].append(units[-1])
        aggregates={}
        for row in records(unit,'e08_native_geometry.jsonl'):
            task=row['task'];agg=aggregates.setdefault(task,dict(base,task=task,rows=0,candidate_unavailable=0,
                native_max_residual=None,geometric_max_residual=None,legacy_returned_max_residual=None,
                max_A_difference=None,max_b_difference=None,native_geometric_disagreements=0,
                committed_native_exceedances=0,committed_geometric_exceedances=0,rejected_returned_exceedances=0,
                original_hard_tolerance=row['original_hard_tolerance'],task_enforcement=row['task_enforcement']))
            agg['rows']+=1;agg['candidate_unavailable']+=not row['native_selected_candidate_available']
            for target,source in [('native_max_residual','native_residual'),('geometric_max_residual','geometric_residual'),('legacy_returned_max_residual','legacy_returned_solution_residual')]:
                agg[target]=value_max(agg[target],row[source]['maximum'])
            for key in ('max_A_difference','max_b_difference'):agg[key]=value_max(agg[key],row[key])
            nfail=row['native_residual']['within_original_tolerance'] is False
            gfail=row['geometric_residual']['within_original_tolerance'] is False
            scaled=row['task_enforcement']=='scaled'
            agg['native_geometric_disagreements']+=row['tolerance_disagreement']
            agg['committed_native_exceedances']+=bool(row['committed'] and nfail and scaled)
            agg['committed_geometric_exceedances']+=bool(row['committed'] and gfail and scaled)
            agg['rejected_returned_exceedances']+=bool(not row['committed'] and row['legacy_returned_solution_residual']['within_original_tolerance'] is False)
            if row['tolerance_disagreement'] or (row['committed'] and (nfail or gfail) and scaled):
                exceedances.append(dict(base,task=task,attempt_sequence=row['attempt_sequence'],source_line=row['source_line'],
                    source_time_s=row['source_time_s'],committed=row['committed'],native_disposition=row['native_disposition'],
                    native_residual=row['native_residual'],geometric_residual=row['geometric_residual'],
                    original_hard_tolerance=row['original_hard_tolerance'],native_geometric_disagreement=row['tolerance_disagreement'],
                    evidence_locator=str(C.unit_path(unit,'e08_native_geometry.jsonl')),
                    raw_locator=str(C.unit_path(unit,'raw.jsonl'))+'#line='+str(row['source_line'])))
        geometry.extend(aggregates.values())
        st=dict(base,attempts=0,committed_attempts=0,rejected_attempts=0,rejected_state_changed=0,
                input_position_max_rad=None,returned_position_max_rad=None,committed_position_max_rad=None,
                native_selected_candidate_available=0,first_initial_violation_raw_line=None)
        for row in records(unit,'e08_state_evidence.jsonl'):
            st['attempts']+=1;st['committed_attempts']+=row['committed'];st['rejected_attempts']+=not row['committed']
            st['native_selected_candidate_available']+=row['native_selected_candidate_available']
            st['rejected_state_changed']+=row['rejected_input_state_held'] is False
            for a,b in [('input_position_max_rad','input_position_violation_rad'),('returned_position_max_rad','returned_position_violation_rad'),('committed_position_max_rad','committed_position_violation_rad')]:st[a]=value_max(st[a],row[b])
            if row['input_position_violation_rad'] and st['first_initial_violation_raw_line'] is None:st['first_initial_violation_raw_line']=row['source_line']
        states.append(st)
        for row in records(unit,'e08_fixed_fd.jsonl'):
            fd.append(dict(base,**{k:v for k,v in row.items() if k!='fixed_fd_A'},
                evidence_locator=str(C.unit_path(unit,'e08_fixed_fd.jsonl')),
                interpretation='all three preregistered eps shown; no favorable derivative selected'))
    original_summaries=original_final_scale_reanalysis(ctx)
    original_by_case={(r["case_id"],r["method_id"]):r for r in original_summaries}
    final_summaries=final_scale_reanalysis(ctx)
    final_by_case=collections.defaultdict(list)
    for item in final_summaries:final_by_case[(item['case_id'],item['original_method'])].append(item)
    for issue in old:
        related=grouped.get((issue['case_id'],issue['method_id']),[])
        reproduction=[r for r in related if r['evidence_class']!='feasible-initial-positive-control']
        control=[r for r in related if r['evidence_class']=='feasible-initial-positive-control']
        counts=collections.Counter()
        for r in reproduction:counts.update(r['counts'])
        if issue['issue_kind']=='native-crash':
            diagnosis='原始 PlaCo 无完整候选结果；新补证仅检验同输入异常是否复现，原生异常直接传播。可行 Q0 是另立输入的正控制，不能用于修复旧失败。'
        elif issue['issue_kind']=='initial-infeasible-state':
            diagnosis='原始输入已越界。分别检查 input、返回状态、原生候选与 committed；拒绝并保持输入不代表提交新的越界候选。旧公共核验失败保留。'
        else:
            diagnosis=('先比较同一次补证调用的原生组装行与几何 Jacobian，再列固定差分三步长。原生失败后的返回占位向量不具候选身份；其缩放方程超差单列。'
                       '两种矩阵均在已提交候选上超差时记录约束行为；仅一方超差时保留导数/组装解释未决，不能自动归因算法。')
        supplemental_corrected=final_by_case.get((issue['case_id'],issue['method_id']),[])
        original_corrected=original_by_case.get((issue['case_id'],issue['method_id']))
        corrected=[original_corrected] if original_corrected else []
        final_violations=sum(r['committed_final_native_exceedances'] for r in corrected)
        false_positives=sum(r['old_optimum_false_positives'] for r in corrected)
        unavailable=sum(r['insufficient_rows'] for r in corrected)
        if issue['issue_kind']=='scale-equation':
            if final_violations:
                attribution='committed-final-scaled-violation'
                diagnosis=f'直接重算原批次同次调用最终 scale 仍有 {final_violations} 条已提交原生方程超过原阈值；另有 {false_positives} 条旧检查混用 primary optimum scale 的假阳性。候选算法与核验操作数问题分开保留。'
            elif corrected and not unavailable and sum(r['rejected_without_candidate_rows'] for r in corrected):
                attribution='native-rejection-and-returned-placeholder-mischeck'
                diagnosis='原生优化拒绝并 HOLD；拒绝后的返回占位向量不是候选，旧缩放检查误将其与原生 scale 配对。原生 hard-feasibility rejection 仍是真实失败行为，不能改写为求解成功。'
            elif corrected and not unavailable:
                attribution='primary-versus-final-scale-operand-mismatch'
                diagnosis=f'最终 scale 方程未观察到超差，旧检查存在 {false_positives} 条 primary-versus-final 操作数假阳性；原始失败状态不改写。'
            else:attribution='insufficient-final-scale-evidence'
        else:attribution=issue['issue_kind']
        links.append(dict(issue,attribution=attribution,final_scale_metric_version=FINAL_SCALE_VERSION,corrected_native_exceedances=final_violations,
            old_optimum_false_positives=false_positives,insufficient_final_scale_rows=unavailable,
            original_reanalysis_evidence='evaluation/e08_original_final_scale_rows.csv',
            supplemental_corrected_native_exceedances=sum(r['committed_final_native_exceedances'] for r in supplemental_corrected),
            supplemental_old_optimum_false_positives=sum(r['old_optimum_false_positives'] for r in supplemental_corrected),diagnosis=diagnosis,supplemental_reproduction_units=[r['unit_id'] for r in reproduction],
            supplemental_control_units=[r['unit_id'] for r in control],new_observation_counts=dict(counts),
            evidence_complete=bool(reproduction) and all(r['observation_complete'] for r in reproduction),
            causal_comparison='new runtime diagnostic identity; no samplewise equality assumed with old run'))
    for name,rows in [('original_issues',old),('units',units),('geometry_summary',geometry),('state_summary',states),('fixed_fd',fd),('exceedances',exceedances),('issues',links)]:
        ctx.table('e08_evidence_'+name,rows)
    C.write_json(ctx.output/'evaluation/e08_evidence_validation.json',dict(status='completed',original_issue_count=len(old),
        supplemental_unit_count=len(units),expected_original_issues=16,expected_supplemental_units=56,
        coverage_matches=len(old)==16 and len(units)==56,original_validation_unchanged=True,
        scientific_claim='diagnostic-only; no automatic candidate repair or original-status promotion'))
    counts=collections.Counter()
    for r in units:counts.update(r['counts'])
    original_counts=collections.Counter()
    for r in original_summaries:
        for k in ('failed_check_rows','committed_final_native_exceedances','committed_final_geometric_exceedances','old_optimum_false_positives','rejected_without_candidate_rows','rejected_hold_rows','insufficient_rows'):original_counts[k]+=r[k]
    final_counts=collections.Counter()
    for r in final_summaries:
        for k in ('committed_final_native_exceedances','committed_final_geometric_exceedances','old_optimum_false_positives','insufficient_rows','native_geometric_disagreements'):final_counts[k]+=r[k]
    lines=['# E08 原始异常与定向补证','',
        f'原始失败单元 {len(old)} 个与新增补证单元 {len(units)} 个分别记账。新运行采用独立诊断方法身份，旧输入、约束和阈值保持；四个可行初始控制使用另立的 canonical Q0，不裁剪旧输入。','',
        '## 原批次逐条重算','',
        f'旧 12 个缩放异常单元的 {original_counts["failed_check_rows"]} 条原失败检查：最终已提交原生/几何超差 {original_counts["committed_final_native_exceedances"]}/{original_counts["committed_final_geometric_exceedances"]} 条；primary/final 操作数假阳性 {original_counts["old_optimum_false_positives"]} 条；拒绝而无可采信原生候选 {original_counts["rejected_without_candidate_rows"]} 条，其中 HOLD {original_counts["rejected_hold_rows"]} 条；归因证据不足 {original_counts["insufficient_rows"]} 条。旧失败状态全部保留。','',
        '## 新证据观察','',
        f'完整 attempt {counts["attempts"]}，committed {counts["committed"]}。冻结补证核验中使用 primary optimum scale 的原生/几何超差计数分别为 {counts["committed_native_scaled_exceedances"]}/{counts["committed_geometric_scaled_exceedances"]}，不能直接解释为最终候选方程超差。',
        f'新增分析指标 `{FINAL_SCALE_VERSION}` 使用同次调用 selected candidate 与 exact final scale：已提交原生/几何方程超差分别 {final_counts["committed_final_native_exceedances"]}/{final_counts["committed_final_geometric_exceedances"]} 条；识别旧操作数假阳性 {final_counts["old_optimum_false_positives"]} 条，证据不足 {final_counts["insufficient_rows"]} 条。这些是行级计数，不是独立试验数。',
        f'原生与几何阈值判定分歧 {counts["native_geometric_tolerance_disagreements"]} 条；固定差分核对 {counts["fixed_fd_rows"]} 条。每个核对状态显示 1e-5、1e-6、1e-7 全部结果，不择优。',
        f'拒绝后返回状态方程超差 {counts["rejected_returned_state_equation_exceedances"]} 条，该返回向量不能充当原生优化候选。拒绝后状态改变 {counts["rejected_state_changed"]} 次。','',
        '## 原始问题与补证定位','', '| 原始 case / 方法 | 原始问题及来源 | 新补证 | 说明 |','|---|---|---|---|']
    for row in links:lines.append(f'| {row["case_id"]} / {row["method_id"]} | {row["issue_kind"]} · [原验证]({row["original_validation_locator"]}) · [原 raw]({row["source_directory"]}/raw.jsonl#L{row["first_original_failed_raw_line"] or 1}) | {len(row["supplemental_reproduction_units"])} 个复现、{len(row["supplemental_control_units"])} 个独立控制 | {row["diagnosis"]} |')
    lines+=['','## 数值解释与限制','',
        '原生与几何残差均使用各自完整组装目标项和共同可行速度基线，不能只替换 Jacobian 而保留不同目标。固定差分仅用于交叉核对。native selected candidate、返回状态、选中 priority 和提交状态分别保留。',
        '只读诊断发现 HKS 的 task_scales.weighted_progress_scale 是 primary optimum；final selected scale 保存在同次 native_constraints 的 progress scale 值。绝对 drift 无法恢复方向，禁止据此猜测最终 scale。两套原始核验均保留，分析单独更正操作数。',
        '原始核验文件不变；新增数据缺失、失败和部分观察保持显式。原生与几何均超差只能证明记录的已提交方程行为，不能在未追踪算法内部原因时声称已定位全部根因。未完成的正控制不能宣称修复成功。',
        '旧 12 项逐条归因仅使用原批次 raw、原始检查矩阵和独立几何重算；旧轨迹缺少原生完整矩阵，保留 unavailable-original-trace，原生最坏分量取自旧 native_constraints。新 56 项轨迹另表记账，不能替代旧轨迹。',
        '后续 verifier 应发布新版本：缩放任务使用选中成功 pass 对应的精确 final scale；拒绝后的返回占位向量不得充当原生候选。不得重写旧 validation；修正操作数不等于放宽原阈值。',
        '最小复现：从 e08_original_final_scale_rows.csv 或 e08_final_scale_events.csv 取得 raw_locator、source_line、raw_sha256；只读取这一 attempt 的 selected pass、native_constraints progress scale、基线和候选，对照三个固定差分步长表。真实已提交超差行仍列为候选约束行为问题；原生 task diagnostics 报告超差而提交的门控问题需未来独立修复验证。',
        'PlaCo 最小复现沿原始 initial_infeasible 单元 request.json 启动原身份，保留 stderr 的 QPError 与 attempt_started；可行 Q0 是另立正控制，不能充当原输入修复。',
        '正式实验、RT 结论、统计推断和论文继续 deferred。','', '## 可追溯表','']
    for name in ('original_issues','units','geometry_summary','state_summary','fixed_fd','exceedances','issues'):
        lines.append(f'- [e08_evidence_{name}.csv](evaluation/e08_evidence_{name}.csv)')
    for name in ('rows','summary','fixed_fd'):lines.append(f'- [e08_original_final_scale_{name}.csv](evaluation/e08_original_final_scale_{name}.csv)')
    for name in ('rows','summary','events','fixed_fd'):lines.append(f'- [e08_final_scale_{name}.csv](evaluation/e08_final_scale_{name}.csv)')
    (ctx.output/'report_E08_evidence.md').write_text('\n'.join(lines)+'\n')


def numeric(row,key):
    try:value=float(row.get(key,''))
    except (TypeError,ValueError):return None
    return value if math.isfinite(value) else None


def render(ctx):
    plt=C.plotting()
    geometry=list(C.csv_rows(ctx.output/'evaluation/e08_final_scale_summary.csv'))
    state=list(C.csv_rows(ctx.output/'evaluation/e08_evidence_state_summary.csv'))
    original=list(C.csv_rows(ctx.output/'evaluation/e08_original_final_scale_summary.csv'))
    from matplotlib.ticker import MaxNLocator, ScalarFormatter
    fig,axes=plt.subplots(1,3,figsize=(15,4.8),constrained_layout=True)
    fig.set_constrained_layout_pads(w_pad=.08,h_pad=.08,wspace=.10,hspace=.08)
    for method in sorted({r['original_method'] for r in geometry}):
        selected=[r for r in geometry if r['original_method']==method and numeric(r,'final_native_maximum') is not None]
        axes[0].scatter([numeric(r,'final_native_maximum') for r in selected],[numeric(r,'final_geometric_maximum') for r in selected],s=12,label=method)
    axes[0].set(xlabel='Native residual with FINAL scale (m/s)',
        ylabel='Geometric residual with FINAL scale (m/s)',title='Supplemental scaled trajectories')
    tolerances=sorted({numeric(r,'original_hard_tolerance') for r in geometry if numeric(r,'original_hard_tolerance') is not None})
    for tolerance in tolerances:
        axes[0].axvline(tolerance,color='black',ls=':',lw=.7);axes[0].axhline(tolerance,color='black',ls=':',lw=.7)
    for method in sorted({r['method_id'] for r in original}):
        selected_old=[r for r in original if r['method_id']==method and numeric(r,'final_native_maximum') is not None]
        axes[1].scatter([numeric(r,'final_native_maximum') for r in selected_old],[numeric(r,'final_geometric_maximum') for r in selected_old],s=20,label=method)
    axes[1].set(xlabel='Original recorded native worst residual (m/s)',
        ylabel='Original independently recomputed\ngeometric residual (m/s)',
        title='Old failed checks only; exact final scale')
    # One exponent per axis keeps small residual tick labels compact. This only
    # changes presentation; all points and original tolerance lines are retained.
    for ax in axes[:2]:
        for axis in (ax.xaxis,ax.yaxis):
            axis.set_major_locator(MaxNLocator(nbins=4))
            formatter=ScalarFormatter(useOffset=False,useMathText=True)
            formatter.set_powerlimits((-3,3))
            axis.set_major_formatter(formatter)
    for tolerance in tolerances:
        axes[1].axvline(tolerance,color='black',ls=':',lw=.7);axes[1].axhline(tolerance,color='black',ls=':',lw=.7)
    selected=[r for r in state if r['original_case']=='initial_infeasible']
    for i,r in enumerate(selected):
        inp=numeric(r,'input_position_max_rad');committed=numeric(r,'committed_position_max_rad')
        if inp is not None:axes[2].scatter([i],[inp],color='#D55E00',marker='x')
        if committed is not None:axes[2].scatter([i],[committed],color='#0072B2',marker='o')
    axes[2].set(xlabel='Separate method / original-invalid\nand Q0-control unit',ylabel='Position bound violation (rad)',
        title='x=input; circle=committed only\nAbsent is not zero')
    axes[0].legend(fontsize=6);fig.suptitle('F04 E08 supplemental diagnostics — original failures retained')
    C.save_figure(ctx,fig,'F04','e08-native-geometric-state',
        ['evaluation/e08_final_scale_summary.csv','evaluation/e08_original_final_scale_summary.csv','evaluation/e08_evidence_state_summary.csv'],
        'All supplemental scaled units and original failed checks shown in separate panels; final scale only; rejected candidates unavailable',
        fields=['final_native_maximum','final_geometric_maximum','input_position_max_rad','committed_position_max_rad'],
        selection={'metric_version':FINAL_SCALE_VERSION,'source_layers':'original failed checks / supplemental full scaled trajectories; separate panels'})
