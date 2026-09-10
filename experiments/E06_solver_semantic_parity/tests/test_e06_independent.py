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
def test_missing_pair_is_retained():
 p=pathlib.Path(__file__).parents[1]/'admit_pairs.py';s=importlib.util.spec_from_file_location('pair_admission',p);module=importlib.util.module_from_spec(s);s.loader.exec_module(module)
 result=module.admit(None,None);assert result['status']=='unavailable';assert result['reasons']
def test_begin_only_raw_is_not_success(tmp_path):
 import json
 (tmp_path/'raw.jsonl').write_text(json.dumps({'record_type':'attempt_started','attempt_sequence':0})+'\n')
 request=tmp_path/'request.json';request.write_text(json.dumps({'output_dir':str(tmp_path),'input':{},'config':{}}))
 assert v.verify(request)!=0
 assert json.loads((tmp_path/'app_validation.json').read_text())['status']=='unavailable'
