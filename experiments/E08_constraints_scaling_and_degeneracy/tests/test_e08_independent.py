import importlib.util,pathlib,numpy as np
p=pathlib.Path(__file__).parents[1]/'verify.py';s=importlib.util.spec_from_file_location('local_verifier',p);v=importlib.util.module_from_spec(s);s.loader.exec_module(v)
def test_compatible_reference():
 p={'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,0]],'b':[1]},{'A':[[0,1]],'b':[1]}]}
 r=v.analytic_reference(p,True,regularization=0);assert np.allclose(r['x'],[1,1])
def test_conflict_swapped_and_active_bound():
 p={'lower':[-2,0],'upper':[2,0],'tasks':[{'A':[[1,0]],'b':[1]},{'A':[[1,1]],'b':[0]}]}
 assert np.allclose(v.analytic_reference(p,True,regularization=0)['x'],[1,0]);p['tasks'].reverse();assert np.allclose(v.analytic_reference(p,True,regularization=0)['x'],[0,0])
 assert np.allclose(v.box_qp([[1,0]],[3],[-2,-2],[2,2])[1][0],2)
def test_nonunique_objective_not_joint_vector():
 r=v.box_qp([[1,1],[2,2]],[1,2],[-2,-2],[2,2]);assert abs(r[0])<1e-12
 for x in [[1,0],[0,1],[-1,2]]:assert np.linalg.norm(np.array([[1,1],[2,2]])@x-[1,2])==0
 assert v.box_qp([[1,0]],[1],[2,0],[1,0]) is None
def test_corrupt_reference_missing_and_zero_denominator():
 assert v.check_preservation([0,0],[0,.01],1e-7)['status']=='failed'
 assert v.check_preservation(None,[0],1e-7)['status']=='unavailable'
 assert v.check_preservation([0],[0],1e-7)['preservation_ratio']==0

def test_semantic_counterexamples():
 base={'joint_names':['a','b'],'tcp_offsets':[0,0,.1],'mode':'ServoStep','scale_group':'shared'}
 for key,value in [('joint_names',['b','a']),('tcp_offsets',[0,0,.2]),('mode','TargetSolve'),('scale_group','per-arm')]:
  other={**base,key:value};assert any(x['status']=='semantic_mismatch' and x['field']==key for x in v.semantic_audit(base,other))
def test_declared_matrix_and_event_fixture():
 import json
 root=pathlib.Path(__file__).parents[1];d=json.loads((root/'declarations/study_legacy.v2.json').read_text());units=d['units']
 assert len([u for u in units if u['case_id'].startswith('joint_limit_')])==3*2*3*4
 assert len([u for u in units if u['case_id'].startswith('moving_state_')])==5*3*2*4
 assert len([u for u in units if u['case_id'].startswith('unreachable_one_arm_')])==2*4*2*4
 u=next(u for u in units if u['case_id']=='fixture_compressed_events');descriptor=json.loads(pathlib.Path(u['input']).read_text());data=json.loads(pathlib.Path(descriptor['canonical']['locator']).read_text())
 assert data['fixture'] and u['smoke_config']['steps']==18
 assert [r['source_time_s'] for r in data['samples']]==[0,.003,.012]
 bad=json.loads((root/'inputs/generated/initial_infeasible.json').read_text());assert bad['initial_state']['q'][6]>bad['limits']['upper'][6]
def test_recovery_snapshot_and_unobserved_release_are_missing():
 config={'period_s':.001,'recovery_release_s':4.}
 assert v.recovery_observation([0],[0],[0],[True],config,'snapshot')['status']=='not-observed'
 result=v.recovery_observation([0,.001],[1,1],[0,0],[True,True],config,'evolving')
 assert result['status']=='not-observed' and result['value'] is None
def test_directional_window_uses_declared_compressed_events():
 assert v.declared_challenge_start({'event_windows':{'challenge':[.003,.012]}})==.003
 assert v.declared_challenge_start({})==1.
