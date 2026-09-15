"""Targeted diagnostic evidence tests; no solver or experiment launches."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import pytest
HERE=Path(__file__).resolve().parents[1]

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from analysis import common as C
V=load('verify_e08_evidence',HERE/'verify_evidence.py')
A=load('analyze_e08_evidence',HERE.parents[1]/'analyses/A01_priority_and_robustness/e08_evidence.py')


def test_three_jacobians_do_not_choose_favorable_result():
    assert V.FD_EPS==(1e-5,1e-6,1e-7)
    ref=[.2];x=[.35];b=[.5];scale=.5
    assert V.residual([[2]],b,ref,x,scale)==pytest.approx([.25])
    assert V.residual_record([[2]],b,ref,x,scale,1e-7)['within_original_tolerance'] is False


def test_rejected_placeholder_is_not_native_candidate():
    row={'evidence_linearization':{'available':True,'candidate_available':False,'candidate_values':[],
           'tasks':[dict(name='left-position',enabled=True,A=[[1]],offset=[0],lower=[1],upper=[1],enforcement='scaled')]},
         'candidate_active_v':[0]}
    task=next(V.position_tasks(row,{'evidence_original_method':'mcc-weighted'},[0]))
    assert task['native_candidate'] is None
    assert V.residual_record([[1]],[1],[0],task['native_candidate'],1,1e-7)['maximum'] is None
    assert V.residual_record([[1]],[1],[0],row['candidate_active_v'],1,1e-7)['maximum']==1


def test_input_returned_candidate_and_commit_are_separate():
    data={'joint_names':['j'],'active_joint_names':['j'],'limits':{'lower':[-1],'upper':[1],'velocity':[1]}}
    row=dict(attempt_sequence=0,input_state={'q':[1.01],'v':[0]},q=[1.01],v=[0],candidate_active_q=[0],candidate_active_v=[0],committed=False)
    value=V.state_evidence(row,data,{'period_s':.001,'hard_tolerance':1e-7})
    assert value['input_position_violation_rad']==pytest.approx(.01)
    assert value['rejected_input_state_held'] is True
    assert value['committed_position_violation_rad'] is None
    assert value['native_selected_candidate_available'] is False
    assert value['returned_solution_position_violation_rad']==0






def fixture(tmp_path,committed=True):
    model=tmp_path/'model.urdf';model.write_text('''<robot name="test"><link name="base_link"/><link name="left"/><link name="right"/><joint name="j" type="revolute"><parent link="base_link"/><child link="left"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint><joint name="fixed" type="fixed"><parent link="left"/><child link="right"/><origin xyz="1 0 0"/></joint></robot>''')
    out=tmp_path/'out';out.mkdir()
    data=dict(model=C.artifact(model),joint_names=['j'],active_joint_names=['j'],root_frame='base_link',frames={'left':'left','right':'right'},tcp_offsets={'left':[0,0,0],'right':[0,0,0]},limits={'lower':[-1],'upper':[1],'velocity':[1]})
    tasks=[dict(name='left-position',unit='m/s',source='left-position',enabled=True,A=[[0],[0],[0]],offset=[0,0,0],lower=[0,0,0],upper=[0,0,0],enforcement='scaled',scale_group_handle=0),
           dict(name='right-position',unit='m/s',source='right-position',enabled=True,A=[[0],[1],[0]],offset=[0,0,0],lower=[0,0,0],upper=[0,0,0],enforcement='scaled',scale_group_handle=0)]
    row=dict(record_type='attempt',attempt_sequence=0,source_time_s=0,input_state={'q':[0],'v':[0]},q=[0],v=[0],candidate_active_q=[0],candidate_active_v=[0],task_revision=0,committed=committed,disposition='accepted' if committed else 'rejected',task_scale_reference=[0],scales=[dict(handle=0,evaluated=True,value=1)],targets={'left':dict(position=[0,0,0],rotation=np.eye(3).tolist()),'right':dict(position=[1,0,0],rotation=np.eye(3).tolist())},evidence_linearization=dict(available=True,configuration=[0],candidate_available=committed,candidate_values=[0] if committed else [],tasks=tasks,joint_bounds={'lower':[-1],'upper':[1]}))
    (out/'raw.jsonl').write_text(json.dumps({'record_type':'attempt_started'})+'\n'+json.dumps(row)+'\n')
    request=tmp_path/'request.json';C.write_json(request,dict(output_dir=str(out),input=data,config=dict(steps=1,evidence_fd_eps=list(V.FD_EPS),period_s=.001,gain_per_s=1.,hard_tolerance=1e-7,enforcement='scaled',evidence_original_method='mcc-weighted',evidence_class='scaled-diagnostic')))
    return request,out


def test_complete_geometric_and_three_fixed_fd_evidence(tmp_path):
    request,out=fixture(tmp_path);before=C.digest(out/'raw.jsonl')
    assert V.audit(request)==0
    summary=C.read_json(out/'e08_evidence_summary.json')
    assert summary['observation_complete'] and summary['counts']['fixed_fd_rows']==6
    assert summary['counts'].get('native_geometric_tolerance_disagreements',0)==0
    assert C.digest(out/'raw.jsonl')==before
    assert all(x['max_fd_geometric_A_difference']<1e-7 for x in C.jsonl_rows(out/'e08_fixed_fd.jsonl'))


def test_rejected_native_candidate_absent_still_records_geometry(tmp_path):
    request,out=fixture(tmp_path,committed=False)
    assert V.audit(request)==0
    rows=list(C.jsonl_rows(out/'e08_native_geometry.jsonl'))
    assert all(not row['native_selected_candidate_available'] and row['native_residual']['maximum'] is None for row in rows)
    assert C.read_json(out/'e08_evidence_summary.json')['claim_eligible'] is False


def test_crash_begin_only_remains_incomplete(tmp_path):
    request,out=fixture(tmp_path);(out/'raw.jsonl').write_text('{"record_type":"attempt_started"}\n')
    assert V.audit(request)==2
    assert C.read_json(out/'e08_evidence_summary.json')['classification']=='native-crash-with-begin-only-evidence'


def test_changed_epsilon_rejected(tmp_path):
    request,out=fixture(tmp_path);req=C.read_json(request);req['config']['evidence_fd_eps']=[1e-4];C.write_json(request,req)
    with pytest.raises(ValueError,match='frozen'):V.audit(request)


def test_original_failure_never_promoted_by_analysis(tmp_path):
    p=tmp_path/'scale_equation_checks.json';C.write_json(p,[dict(status='failed',native_disposition='rejected',source_line=2,equation_residual=[.2])])
    unit=dict(experiment_id='E08',case_id='case',method_id='mcc-weighted',arm_id='a',repeat_id=0,run_id='old',status='completed',validation={'status':'failed'},directory=str(tmp_path),artifacts=[C.artifact(p)])
    issue=next(A.old_issues([unit]));assert issue['validation_status']=='failed' and issue['original_failure_unchanged']
    assert issue['original_scale_failure_dispositions']=={'rejected':1}


def test_committed_scaled_violation_remains_failed_at_original_tolerance(tmp_path):
    request,out=fixture(tmp_path)
    lines=list(C.jsonl_rows(out/'raw.jsonl'));row=lines[-1]
    row['q']=[.0001];row['v']=[.1];row['candidate_active_q']=[.0001];row['candidate_active_v']=[.1]
    row['evidence_linearization']['candidate_values']=[.1]
    (out/'raw.jsonl').write_text('\n'.join(json.dumps(x) for x in lines)+'\n')
    assert V.audit(request)==1
    summary=C.read_json(out/'e08_evidence_summary.json')
    assert summary['observation_complete'] and summary['validation_failed'] and not summary['claim_eligible']
    assert summary['counts']['committed_native_scaled_exceedances']==1
    assert C.read_json(out/'app_validation.json')['status']=='failed'


def test_supplement_hook_keeps_old_and_new_counts_and_table_only_render(tmp_path):
    import types
    request,out=fixture(tmp_path);assert V.audit(request)==0
    req=C.read_json(request)
    unit=dict(experiment_id='E08',case_id='new-scaled',method_id='mcc-weighted-e08-evidence-v1',arm_id='diagnostic',repeat_id=0,
              run_id='new',status='completed',validation={'status':'passed'},directory=str(out),
              artifacts=[C.artifact(p) for p in out.iterdir() if p.is_file()],repair_stage='E08',config=req['config'])
    unit['config'].update(evidence_original_case='old-scaled',evidence_original_method='mcc-weighted')
    output=tmp_path/'analysis';output.mkdir()
    ctx=types.SimpleNamespace(units=[],supplemental_units=[unit],output=output,figures=[],definition={'display':{},'analysis_id':'A01'})
    ctx.table=lambda name,rows:C.write_csv(output/'evaluation'/(name+'.csv'),rows)
    A.analyze(ctx)
    validation=C.read_json(output/'evaluation/e08_evidence_validation.json')
    assert validation['original_issue_count']==0 and validation['supplemental_unit_count']==1
    assert validation['coverage_matches'] is False
    before={p.name:C.digest(p) for p in (output/'evaluation').iterdir()}
    # Renderer is deliberately detached from all source readers and consumes only tables.
    ctx.supplemental_units=None;ctx.units=None
    A.render(ctx)
    assert (output/'figures/F04-e08-native-geometric-state.png').is_file()
    assert before=={p.name:C.digest(p) for p in (output/'evaluation').iterdir()}


def test_committed_velocity_excess_not_hidden_by_position_feasibility(tmp_path):
    request,out=fixture(tmp_path)
    lines=list(C.jsonl_rows(out/'raw.jsonl'));row=lines[-1]
    row['v']=[1.01];row['q']=[.00101];row['candidate_active_v']=[1.01];row['candidate_active_q']=[.00101]
    row['evidence_linearization']['candidate_values']=[1.01]
    (out/'raw.jsonl').write_text('\n'.join(json.dumps(x) for x in lines)+'\n')
    assert V.audit(request)==1
    summary=C.read_json(out/'e08_evidence_summary.json')
    assert summary['counts']['committed_velocity_exceedances']==1
    assert summary['counts']['committed_native_bound_exceedances']==1
    assert summary['counts'].get('committed_position_exceedances',0)==0
