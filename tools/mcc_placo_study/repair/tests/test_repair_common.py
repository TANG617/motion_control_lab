import copy,sys
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from analysis import common as C
from repair.common import evidence_class,STAGE_COUNTS,COMMON_ACCEPTANCE_HASH
from repair.kinematics import GeometricFk


def test_v2_requires_explicit_supplement():
    d=C.read_json(ROOT/'analyses/A01_priority_and_robustness/definition.json');C.validate_definition(d)
    d['schema_version']='analysis.v2'
    with pytest.raises(Exception):C.validate_definition(d)
    a={'locator':'/explicit/manifest.json','sha256':'0'*64,'size_bytes':0}
    d['supplement']={'declaration':a,'execution_index':a};C.validate_definition(d)
    d['campaign']['plan']['locator']='/runs/latest/plan.json'
    with pytest.raises(ValueError,match='latest'):C.validate_definition(d)


def test_expected_fixture_preserves_failure():
    u={'case_id':'fixture-observation-overflow','status':'failed','validation':{'status':'failed'},'request':{'input':{'fixture':True}}}
    assert evidence_class(u)=='expected-failure-fixture'
    assert u['status']=='failed'
    u['request']['input']['fixture']=False
    assert evidence_class(u)=='observed-anomaly'


def test_matrix_identity():
    assert sum(STAGE_COUNTS.values())==308
    assert len(COMMON_ACCEPTANCE_HASH)==64


def test_controlled_pair_cannot_silently_cross_runs():
    u={k:'same' for k in C.PAIR_KEYS};u.update(run_id='first',method_id='mcc-weighted',status='completed',validation={'status':'passed'},config={})
    v={**u,'run_id':'second'}
    assert 'cross-run-controlled-comparison-not-declared' in C.compatible(u,v)


def test_geometric_jacobian_including_tcp_and_prismatic(tmp_path):
    model=tmp_path/'robot.urdf'
    model.write_text('''<robot name="two"><link name="base_link"/><link name="arm"/><link name="tip"/>
    <joint name="turn" type="revolute"><parent link="base_link"/><child link="arm"/><origin xyz=".2 .1 0" rpy=".1 .2 .3"/><axis xyz="0 0 1"/></joint>
    <joint name="slide" type="prismatic"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/><axis xyz="1 0 0"/></joint></robot>''')
    fk=GeometricFk(model);names=['turn','slide'];q=[.4,.2]
    for eps in [1e-5,1e-6,1e-7]:
        np.testing.assert_allclose(fk.jacobian(names,q,'tip',(.1,.2,.3)),fk.finite_difference(names,q,'tip',(.1,.2,.3),eps),atol=1e-8,rtol=0)
    assert np.linalg.norm(fk.angular_jacobian(names,q,'tip')[:,0])==pytest.approx(1.)
    np.testing.assert_array_equal(fk.angular_jacobian(names,q,'tip')[:,1],0)


def test_new_name_alone_does_not_admit():
    def unit(name):return {'method_id':name,'config':{'acceptance_contract_hash':COMMON_ACCEPTANCE_HASH},'artifacts':[]}
    assert C.method_policy_reasons(unit('mcc-common'),unit('placo-common'))==['common-native-audit-missing']


def test_native_audit_must_be_hash_bound(tmp_path):
    units=[]
    for method in ('mcc-common','placo-common'):
        p=tmp_path/method/'common_servo_audit.json'
        C.write_json(p,{'acceptance_contract_hash':COMMON_ACCEPTANCE_HASH,'design_comparable':True,'observation_complete':True})
        units.append({'method_id':method,'config':{'acceptance_contract_hash':COMMON_ACCEPTANCE_HASH},'artifacts':[C.artifact(p)]})
    assert C.method_policy_reasons(*units)==[]
    p.write_text('{}')
    with pytest.raises(ValueError,match='hash'):C.method_policy_reasons(*units)


def test_timing_gate_missing_never_changes_math(tmp_path):
    from repair.common import bind_timing
    descriptor=tmp_path/'input.json';C.write_json(descriptor,{'canonical':{'sha256':'0'*64}})
    source={'units':[{'case_id':'case','arm_id':'arm','input':str(descriptor),'config':{'workload':0,'hard_tolerance':1e-7}}]}
    resolved,blocked=bind_timing(source,{})
    assert blocked
    assert resolved['units'][0]['config']['hard_tolerance']==source['units'][0]['config']['hard_tolerance']
    assert 'native_audit_gate_verdict' not in source['units'][0]['config']


def binding_fixture(tmp_path):
    from types import SimpleNamespace
    model=tmp_path/'robot.urdf';model.write_text('<robot/>')
    data={'model':C.artifact(model),'q':[1,2]};canonical=tmp_path/'input.json';C.write_json(canonical,data)
    descriptor={'canonical':C.artifact(canonical)};path=tmp_path/'input.descriptor.json';C.write_json(path,descriptor)
    declared={'config':{'hard_tolerance':1e-7},'input':str(path),'method_id':'mcc-test'}
    u={'config':copy.deepcopy(declared['config']),'config_hash':C.stable_hash(declared['config']),
       'method_id':declared['method_id'],'input_hash':descriptor['canonical']['sha256'],'model_hash':data['model']['sha256']}
    u['request']={'identity':{k:v for k,v in u.items() if k!='config'},'config':u['config'],
                  'input':data,'input_descriptor':descriptor}
    return SimpleNamespace(check_artifact=C.verify),u,declared,{a['locator']:a for a in [C.artifact(path),C.artifact(canonical),C.artifact(model)]}


@pytest.mark.parametrize('mutation',['config','method','input_hash','request_identity','request_input','descriptor','canonical','model'])
def test_frozen_input_and_config_binding(tmp_path,mutation):
    from repair.common import verify_unit_binding
    ctx,u,d,inputs=binding_fixture(tmp_path)
    verify_unit_binding(ctx,u,d,tmp_path/'definition.json',inputs)
    if mutation=='config':
        u['config']={'hard_tolerance':.1};u['config_hash']=C.stable_hash(u['config'])
    elif mutation=='method':u['method_id']='different'
    elif mutation=='input_hash':u['input_hash']='1'*64
    elif mutation=='request_identity':u['request']['identity']['method_id']='different'
    elif mutation=='request_input':u['request']['input']={**u['request']['input'],'q':[7,8]}
    else:
        name={'descriptor':'input.descriptor.json','canonical':'input.json','model':'robot.urdf'}[mutation]
        (tmp_path/name).write_text('{}')
    with pytest.raises(ValueError):verify_unit_binding(ctx,u,d,tmp_path/'definition.json',inputs)


def test_nested_git_source_inventory_uses_repository_relative_paths(tmp_path):
    import subprocess
    from evidence import source_fingerprint,digest
    subprocess.run(['git','init','-q',str(tmp_path)],check=True)
    source=tmp_path/'component';source.mkdir();(source/'solver.cpp').write_text('native source')
    (tmp_path/'solver.cpp').write_text('unrelated root file')
    subprocess.run(['git','-C',str(tmp_path),'add','.'],check=True)
    subprocess.run(['git','-C',str(tmp_path),'-c','user.name=fixture','-c','user.email=fixture@example.test','commit','-qm','fixture'],check=True)
    result=source_fingerprint(source)
    assert result['files']==[{'path':'component/solver.cpp','sha256':digest(source/'solver.cpp')}]




@pytest.mark.parametrize('mutation',['base_campaign','declaration','duplicate_stage'])
def test_supplement_rejects_cross_batch_or_duplicate_stage(tmp_path,mutation):
    from repair.common import load_supplement
    from types import SimpleNamespace
    declaration={'schema_version':'evidence_repair.v1','base_campaign':{'id':'old'},'stage_counts':STAGE_COUNTS,'inputs':[]}
    dp=tmp_path/'declaration.json';C.write_json(dp,declaration)
    index={'schema_version':'repair_execution_index.v1','declaration':C.artifact(dp),'status':'closed',
           'runs':[{'stage':'E06'}]*4}
    ip=tmp_path/'execution_index.json';C.write_json(ip,index)
    d={'analysis_id':'A01','campaign':{'id':'old'},'supplement':{'declaration':C.artifact(dp),'execution_index':C.artifact(ip)}}
    if mutation=='base_campaign':d['campaign']={'id':'another'}
    elif mutation=='declaration':
        index['declaration']={**index['declaration'],'sha256':'0'*64};C.write_json(ip,index);d['supplement']['execution_index']=C.artifact(ip)
    ctx=SimpleNamespace(definition=d,check_artifact=C.verify)
    with pytest.raises(ValueError,match='different|duplicate'):load_supplement(ctx)
