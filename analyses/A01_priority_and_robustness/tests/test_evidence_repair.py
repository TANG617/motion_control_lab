import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'));sys.path.insert(0,str(ROOT/'analyses/A01_priority_and_robustness'))
import e07_evidence as E
R=E.R
class RecomputeTests(unittest.TestCase):
 def test_disabled_secondary_is_measured(self):
  row=dict(record_type='analytic',candidate=[1.,0.],accepted=True,problem={'tasks':[{'A':[[1,0]],'b':[1]},{'A':[[0,1]],'b':[2],'weight':3}]})
  tasks=R.task_rows({},dict(primary_only=True),row,None);cost=R.costs(tasks,R.output_vector({},row))[1]
  self.assertFalse(cost['solve_enabled']);self.assertEqual(cost['raw_cost'],2);self.assertEqual(cost['weighted_cost'],6)
 def test_rejected_analytic_not_candidate_output(self):
  self.assertIsNone(R.output_vector({},dict(record_type='analytic',candidate=[1,2],accepted=False)))
 def test_multisolution_reference_objective(self):
  tasks=[dict(name='x',A=np.array([[1.,0.]]),b=np.array([1.]),weight=1.,enabled=True,level=0)]
  r=R.sequential_reference(tasks,np.array([-2.,-2.]),np.array([2.,2.]),1e-8,1e-7,1e-7,'mcc-hqp-2')
  self.assertEqual(r['status'],'certified');self.assertLess(r['levels'][0]['objective'],1e-12)
 def test_nonconvergence_retained(self):
  tasks=[dict(name='x',A=np.eye(2),b=np.ones(2),weight=1.,enabled=True,level=0)]
  r=R.sequential_reference(tasks,np.array([-2.,-2.]),np.array([2.,2.]),1e-8,1e-7,1e-7,'mcc-hqp-2',maxiter=0)
  self.assertEqual(r['status'],'unavailable')
 def test_initial_empty_set_no_relaxation(self):
  r=R.sequential_reference([],np.array([1.]),np.array([0.]),1e-8,1e-7,1e-7,'mcc-hqp-2');self.assertEqual(r['reason'],'empty original hard set')
 def test_declared_switch_selection(self):
  d=dict(samples=[dict(source_time_s=0.,task_revision=0,secondary_enabled=False),dict(source_time_s=1.,task_revision=1,secondary_enabled=True)])
  self.assertTrue(R.reference_selected(d,dict(state_mode='evolving',period_s=.001),dict(source_time_s=1.)))
  self.assertFalse(R.reference_selected(d,dict(state_mode='evolving',period_s=.001),dict(source_time_s=.999)))
  self.assertFalse(R.reference_selected(dict(d,fixture=True),dict(state_mode='snapshot'),{}))
 def test_zero_gain_and_missing_reference(self):
  from metrics import secondary_gain
  self.assertIsNone(secondary_gain(0,0)['relative'])
  self.assertEqual(E.paired_costs([],[]),[])
 def test_geometry_frame_origin_no_default_tcp(self):
  class Fk:
   def pose(self,names,q,frame,tcp):
    self.assert_tcp=tcp
    assert tcp==(0,0,0)
    return {'position':np.zeros(3),'rotation':np.eye(3)}
   def jacobian(self,*a):return np.zeros((3,1))
   def angular_jacobian(self,*a):return np.zeros((3,1))
  d=dict(joint_names=['q'],active_joint_names=['q'],frames={'left':'l','right':'r'},tcp_offsets={'left':[0,0,0],'right':[0,0,0]})
  row=dict(record_type='attempt',input_state={'q':[0]},targets={s:dict(position=[0,0,0],rotation=np.eye(3)) for s in ('left','right')})
  self.assertEqual(len(R.task_rows(d,dict(gain_per_s=1,orientation_weight=1),row,Fk())),4)

class HardReferenceTests(unittest.TestCase):
 def test_hard_task_not_softened(self):
  tasks=[dict(name='hard',A=np.array([[1.,0.]]),b=np.array([1.]),weight=1.,enabled=True,level=0,enforcement='hard'),dict(name='soft',A=np.eye(2),b=np.array([-1.,1.]),weight=1.,enabled=True,level=0)]
  r=R.sequential_reference(tasks,np.array([-2.,-2.]),np.array([2.,2.]),1e-8,1e-7,1e-7,'mcc-weighted')
  self.assertEqual(r['status'],'certified');self.assertAlmostEqual(r['levels'][0]['x'][0],1.,places=7)

class PairStateTests(unittest.TestCase):
 def fixture(self,mode='snapshot'):
  a={k:'identity' for k in E.C.PAIR_KEYS};a.update(experiment_id='E07',case_id='case',method_id='mcc-hqp-2',arm_id='full',repeat_id=0,run_id='run',directory='/unused',status='completed',validation={'status':'passed'},config={'primary_only':False,'state_mode':mode})
  b=dict(a,arm_id='primary',config={'primary_only':True,'state_mode':mode})
  rows=[dict(unit_id=E.C.unit_id(u),task='posture',secondary=True,weighted_cost_mean=cost,raw_cost_mean=cost,complete=True,measured_count=1,linearization_sequence_hash=str(i)) for i,(u,cost) in enumerate([(a,1.),(b,2.)])]
  return a,b,rows
 def test_snapshot_different_state_rejected(self):
  a,b,rows=self.fixture();r=E.paired_costs([a,b],rows);self.assertEqual(r[0]['status'],'unavailable');self.assertIn('snapshot-linearization-sequence-mismatch',r[0]['reasons'])
 def test_evolving_own_states_retained(self):
  a,b,rows=self.fixture('evolving');r=E.paired_costs([a,b],rows);self.assertEqual(r[0]['status'],'ok');self.assertEqual(r[0]['absolute_gain'],1.);self.assertIn('own evolving states',r[0]['interpretation'])
 def test_missing_reference_retained(self):
  a,b,rows=self.fixture();r=E.paired_costs([a],rows[:1]);self.assertEqual(r[0]['status'],'unavailable');self.assertIn('missing-or-duplicate-primary-only-reference',r[0]['reasons'])
 def test_zero_reference_keeps_absolute(self):
  a,b,rows=self.fixture('evolving');rows[1].update(weighted_cost_mean=0.,raw_cost_mean=0.);r=E.paired_costs([a,b],rows);self.assertEqual(r[0]['status'],'not-applicable');self.assertEqual(r[0]['absolute_gain'],-1.);self.assertIsNone(r[0]['relative_gain'])

class ApplicabilityAndKktTests(unittest.TestCase):
 def tasks(self,link4,posture,primary):
  class Fk:
   def pose(self,*a,**k):return {'position':np.zeros(3),'rotation':np.eye(3)}
   def jacobian(self,*a):return np.ones((3,1))
   def angular_jacobian(self,*a):return np.ones((3,1))
  data=dict(joint_names=['q'],active_joint_names=['q'],frames=dict(left='l',right='r'),tcp_offsets=dict(left=[0,0,0],right=[0,0,0]))
  row=dict(record_type='attempt',input_state={'q':[0]},posture_target=[1],elbow_targets=dict(left=[1,0,0],right=[1,0,0]),targets={s:dict(position=[0,0,0],rotation=np.eye(3)) for s in ('left','right')})
  return R.task_rows(data,dict(gain_per_s=1,orientation_weight=1,secondary_weight=1,secondary_tasks=True,secondary_link4=link4,secondary_posture=posture,primary_only=primary),row,Fk())
 def test_link4_only_primary_evaluation(self):
  cost={r['task']:r for r in R.costs(self.tasks(True,False,True),np.array([0.]))}
  self.assertEqual(cost['left-elbow']['raw_cost'],.5);self.assertFalse(cost['left-elbow']['solve_enabled'])
  self.assertFalse(cost['posture']['declared']);self.assertIsNone(cost['posture']['raw_cost'])
 def test_posture_only_full_evaluation(self):
  cost={r['task']:r for r in R.costs(self.tasks(False,True,False),np.array([0.]))}
  self.assertEqual(cost['posture']['raw_cost'],.5);self.assertTrue(cost['posture']['solve_enabled'])
  self.assertFalse(cost['left-elbow']['declared']);self.assertIsNone(cost['left-elbow']['raw_cost'])
 def test_kkt_reports_complementarity_and_original_band_fraction(self):
  tasks=[dict(name='x',A=np.eye(2),b=np.array([2.,1.]),weight=1.,enabled=True,level=0)]
  r=R.sequential_reference(tasks,np.array([-1.,-1.]),np.array([1.,1.]),1e-8,1e-7,1e-7,'mcc-hqp-2')
  self.assertEqual(r['status'],'certified');l=r['levels'][0]
  self.assertLessEqual(l['kkt_complementarity_max'],1e-9);self.assertEqual(l['preservation_half_width'],.5e-7)
 def test_stationary_near_bound_without_complementarity_is_unavailable(self):
  from unittest.mock import patch
  from types import SimpleNamespace
  tasks=[dict(name='x',A=np.eye(1),b=np.array([2.]),weight=1.,enabled=True,level=0)]
  fake=SimpleNamespace(x=np.array([1.-1e-8]),success=True,message='fixture stationary but noncomplementary',fun=.5)
  with patch.object(R,'minimize',return_value=fake):
   r=R.sequential_reference(tasks,np.array([-1.]),np.array([1.]),1e-8,1e-7,1e-7,'mcc-weighted')
  self.assertEqual(r['status'],'unavailable');self.assertGreater(r['levels'][0]['kkt_complementarity_max'],1e-9);self.assertLess(r['levels'][0]['kkt_stationarity_inf'],1e-6)

class E06SummaryTests(unittest.TestCase):
 def test_native_candidate_quality_and_commit_counts_remain_separate(self):
  import tempfile
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);unit=dict(experiment_id='E06',case_id='fixture',arm_id='common',method_id='mcc-weighted-common-servo-v1',repeat_id=0,run_id='run',directory=str(p),status='completed',validation={'status':'passed'},config={'steps':2},artifacts=[])
   rows=[dict(record_type='attempt_started'),dict(record_type='attempt',native_accepted=True,candidate_available=True,common_quality_passed=False,committed=False),dict(record_type='attempt',native_accepted=False,candidate_available=True,common_quality_passed=True,committed=False)]
   (p/'raw.jsonl').write_text('\n'.join(__import__('json').dumps(r) for r in rows)+'\n')
   E.C.write_json(p/'common_servo_checks.json',[dict(bounds_passed=True,tasks=[dict(max_A_gap=1e-9,max_b_velocity_gap=2e-9,tolerance=1e-7)])])
   E.C.write_json(p/'common_servo_audit.json',dict(design_comparable=True,observation_complete=True,claim_eligible=True,reasons=[]))
   unit['artifacts']=[E.C.artifact(x) for x in p.iterdir()]
   r=E.e06_summary(unit)
   self.assertEqual((r['native_accepted_count'],r['candidate_available_count'],r['common_quality_passed_count'],r['committed_count']),(1,2,1,0))
   self.assertEqual(r['observed_attempts'],2);self.assertEqual(r['audit_attempts'],1);self.assertEqual(r['max_target_velocity_gap'],2e-9);self.assertEqual(len(r['source_artifacts']),3)

class RepairRenderingTests(unittest.TestCase):
 def test_e06_and_e07_figures_render_together_without_changing_tables(self):
  import tempfile
  from types import SimpleNamespace
  with tempfile.TemporaryDirectory() as d:
   out=Path(d);(out/'report.md').write_text('Original base report\n')
   for name in ['e07_actual_task_costs','e07_independent_references','e07_repair_coverage','e06_common_admission','e06_common_pairs']:
    E.C.write_csv(out/'evaluation'/(name+'.csv'),[])
   pair=dict(status='ok',cost_definition='weighted',method_id='mcc-hqp-2',interpretation='same linearization snapshot sequence',absolute_gain=.5,task='posture')
   E.C.write_csv(out/'evaluation/e07_actual_primary_only_effects.csv',[pair])
   row=dict(case_id='fixture',method_id='mcc-weighted-common-servo-v1',design_comparable=True,observation_complete=True,claim_eligible=True,observed_attempts=1,native_accepted_count=1,candidate_available_count=1,common_quality_passed_count=1,committed_count=1,max_A_audit_ratio=1e-8,max_target_audit_ratio=1e-6)
   E.C.write_csv(out/'evaluation/e06_common_numeric_summary.csv',[row]);E.C.write_csv(out/'evaluation/evidence_classification.csv',[{'origin':'base','experiment_id':'E07'},{'origin':'E06','experiment_id':'E06'}]);E.C.write_json(out/'evaluation/e07_source_policy_recovery.json',{'files':[]})
   ctx=SimpleNamespace(output=out,definition={'analysis_id':'A01','display':{}},figures=[])
   before={p.name:E.C.digest(p) for p in (out/'evaluation').glob('*.csv')};E.render(ctx)
   self.assertEqual({r['figure_id'] for r in ctx.figures},{'F02','F03'})
   self.assertEqual(before,{p.name:E.C.digest(p) for p in (out/'evaluation').glob('*.csv')})
   self.assertIn('恢复 0/2',(out/'report_E06_E07_evidence.md').read_text())
   report=(out/'report.md').read_text();self.assertTrue(report.startswith('# A01 v2：定向补证入口'));self.assertIn('旧批次 **1 个单元**与新增补证 **1 个单元**',report)
   self.assertLess(report.index('report_E06_E07_evidence.md'),report.index('Original base report'))
   self.assertIn('仅适用于旧方法身份',report);E.render(ctx)
   self.assertEqual(report,(out/'report.md').read_text());self.assertEqual((out/'report.md').read_text().count('# A01 v2：定向补证入口'),1)
   self.assertEqual(before,{p.name:E.C.digest(p) for p in (out/'evaluation').glob('*.csv')})
 def test_worker_records_actual_blas_versions_inside_limit(self):
  from unittest.mock import patch
  with patch.object(E,'_reduce_unit',return_value=([],[],[{}])):
   result=E.reduce_unit({})
  libraries=result[2][0]['numerical_libraries'];self.assertTrue(libraries)
  for library in libraries:
   self.assertEqual(library['num_threads'],1);self.assertIn('version',library)

class HardOnlyReducerTests(unittest.TestCase):
 def test_empty_soft_rows_retain_hard_reference_and_both_objective_gaps(self):
  import tempfile,json
  from unittest.mock import patch
  with tempfile.TemporaryDirectory() as d:
   directory=Path(d)/'run/units/E07/hard/primary/0';directory.mkdir(parents=True)
   config=dict(primary_only=True,state_mode='snapshot',steps=1,period_s=.001,regularization=1e-8,preservation_tolerance=1e-7,hard_tolerance=1e-7)
   data=dict(joint_names=['a','b'],active_joint_names=['a','b'],model={'locator':'unused-fixture'},limits={'lower':[-2,-2],'upper':[2,2],'velocity':[2,2]})
   raw=dict(record_type='attempt',input_state={'q':[0,0],'v':[0,0]},v=[1,0],committed=True,attempt_sequence=0,source_time_s=0)
   request={'input':data,'config':config};E.C.write_json(directory/'request.json',request);(directory/'raw.jsonl').write_text(json.dumps(raw)+'\n')
   unit=dict(experiment_id='E07',case_id='hard',arm_id='primary',method_id='mcc-weighted',repeat_id=0,run_id='run',directory=str(directory),status='completed',validation={'status':'passed'},config=config,request=request,artifacts=[E.C.artifact(p) for p in directory.iterdir()])
   task=dict(name='hard-position',A=np.array([[1.,0.]]),b=np.array([1.]),weight=1.,enabled=True,declared=True,secondary=False,level=0,enforcement='hard')
   with patch.object(R,'CachedGeometry'),patch.object(R,'task_rows',return_value=[task]):
    costs,refs,coverage=E._reduce_unit(unit)
   self.assertEqual(refs[0]['reference']['levels'][0]['A'],[])
   gap=refs[0]['candidate_objective_gaps'][0];self.assertEqual(gap['soft_objective_gap'],0.)
   self.assertAlmostEqual(gap['regularized_objective_gap'],0.,places=14)
   self.assertEqual(refs[0]['reference_status'],'certified');self.assertTrue(coverage[0]['complete'])
