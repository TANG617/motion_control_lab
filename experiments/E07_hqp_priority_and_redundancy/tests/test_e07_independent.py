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
def test_declared_priority_matrix_and_source_events():
 import json
 root=pathlib.Path(__file__).parents[1];d=json.loads((root/'declarations/study_legacy.v2.json').read_text());units=d['units']
 assert len([u for u in units if u['case_id'].startswith('r1_hold_tcp_')])==2*3*4*2*4*2
 for method in ['placo-controlled','mcc-weighted','mcc-hqp-2','mcc-hqp-3']:
  for family in ['analytic_redundant','analytic_conflict','analytic_rank_deficient','analytic_active_bound']:
   assert any(u['method_id']==method and u['case_id']==family and not u.get('available') is False for u in units)
 for u in units:
  if u['case_id']=='r1_task_transition_both':
   descriptor=json.loads(pathlib.Path(u['input']).read_text());data=json.loads(pathlib.Path(descriptor['canonical']['locator']).read_text());assert [s['source_time_s'] for s in data['samples']]==[0,1.,3.]
def test_native_notrun_zero_buffer_is_not_optimum():
 assert v.native_reference({'state':0,'enabled':True,'residual_at_level':[0,0,0]}) is None
 assert v.native_reference({'state':2,'enabled':True,'residual_at_level':[0,0,0]})==[0,0,0]
 assert v.native_reference({'state':2,'enabled':False,'residual_at_level':[0]}) is None
