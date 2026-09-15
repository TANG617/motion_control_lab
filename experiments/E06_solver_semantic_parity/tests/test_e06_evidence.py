import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
V=module('verify_e06_repair',ROOT/'experiments/E06_solver_semantic_parity/verify_evidence.py')
from repair.common import COMMON_ACCEPTANCE_HASH
class Fk:
 def pose(self,names,q,frame,tcp):return {'position':np.zeros(3),'rotation':np.eye(3)}
 def jacobian(self,names,q,frame):return np.eye(3)
 def angular_jacobian(self,names,q,frame):return np.eye(3)
class EvidenceTests(unittest.TestCase):
 def fixture(self):
  d=dict(joint_names=['a','b','c'],active_joint_names=['a','b','c'],frames=dict(left='l',right='r'),tcp_offsets=dict(left=[0,0,0],right=[0,0,0]),limits=dict(lower=[-1]*3,upper=[1]*3,velocity=[2]*3))
  c=dict(period_s=.001,gain_per_s=1.,regularization=1e-8,preservation_tolerance=1e-7,orientation_weight=1.,enforcement='soft')
  row=dict(attempt_sequence=0,input_state={'q':[0]*3},targets={s:dict(position=[.1,0,0],rotation=np.eye(3).tolist()) for s in ('left','right')},native_candidate_values=[0,0,0],native_linearization_available=True,native_joint_lower=[-2]*3,native_joint_upper=[2]*3,native_regularization=1e-8,acceptance_contract_hash=COMMON_ACCEPTANCE_HASH,committed=True,native_accepted=True,candidate_available=True,common_quality_passed=True,native_task_rows=[])
  for name,(A,b,w,e) in V.expected_tasks(d,c,row,Fk()).items():row['native_task_rows'].append(dict(name=name,A=A.tolist(),lower=b.tolist(),offset=[0]*3,enforcement=e,weight=w,normalization=[1]*3))
  return d,c,row
 def test_actual_equations_and_acceptance(self):
  d,c,r=self.fixture();import json;json.dumps(V.audit_row(d,c,r,Fk()),allow_nan=False);self.assertTrue(V.audit_row(d,c,r,Fk())['design_comparable']);r['native_accepted']=False;self.assertFalse(V.audit_row(d,c,r,Fk())['design_comparable'])
 def test_corrupt_normalization_blocks(self):
  d,c,r=self.fixture();r['native_task_rows'][0]['normalization']=[2]*3;self.assertFalse(V.audit_row(d,c,r,Fk())['design_comparable'])
 def test_missing_actual_bounds_blocks(self):
  d,c,r=self.fixture();r['native_linearization_available']=False;self.assertFalse(V.audit_row(d,c,r,Fk())['design_comparable'])
