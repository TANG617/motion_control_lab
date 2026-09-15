import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import numpy as np
HERE=Path(__file__).resolve().parent

def load(name,path):
    s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
v=load('e09_verify_evidence',HERE/'verify_evidence.py');p=v


class Gate(unittest.TestCase):
    def test_audit_gate_needs_both_methods_same_hash_complete_and_identity(self):
        from analysis import common as C
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);units=[]
            for method in p.METHODS.values():
                directory=root/'units/E09/case'/method/'0';directory.mkdir(parents=True)
                C.write_json(directory/'request.json',{'config':{'workload':0}})
                C.write_json(directory/'common_servo_audit.json',dict(method_id=method,input_hash='input',workload=0,
                    acceptance_contract_hash=p.COMMON_ACCEPTANCE_HASH,design_comparable=True,observation_complete=True,claim_eligible=True,canonical_problem_hash='math'))
                units.append(dict(experiment_id='E09',case_id='case',arm_id=method,method_id=method,repeat_id=0,input_hash='input',status='completed',validation={'status':'passed'},artifacts=[C.artifact(directory/n) for n in ('request.json','common_servo_audit.json')]))
            C.write_json(root/'inventory.json',{'actual_units':units})
            self.assertTrue(p.audit_gate(root)[('input',0)]['admitted'])
            units[1]['status']='failed';C.write_json(root/'inventory.json',{'actual_units':units})
            self.assertFalse(p.audit_gate(root)[('input',0)]['admitted'])
            C.write_json(root/'inventory.json',{'actual_units':units[:1]})
            self.assertIn('missing-or-duplicate-native-method',p.audit_gate(root)[('input',0)]['reasons'])

class Audit(unittest.TestCase):
    def setUp(self):
        self.data={'joint_names':['j'],'active_joint_names':['j'],'limits':{'lower':[-1.],'upper':[1.],'velocity':[2.]}}
        self.c={'dt_s':.001,'hard_tolerance':1e-7,'regularization':1e-8}
        self.sample={'q':[0.]}
        self.row={'attempt_sequence':0,'input_sequence':0,'native_accepted':True,'candidate_available':True,'candidate_q':[.001],'candidate_v':[1.],
            'committed':True,'common_quality_passed':True,'acceptance_contract_hash':v.COMMON_ACCEPTANCE_HASH,
            'native_audit':{'available':True,'variable_unit':'rad/s','joint_lower':[-2.],'joint_upper':[2.],'regularization':1e-8,
                'tasks':[{'name':'task','A':[[1.]],'offset':[0.],'lower':[1.],'upper':[1.],'enabled':True,'enforcement':'soft','weight':1.,'normalization':[1.]}]}}
    def audit(self,row):
        with mock.patch.object(v,'expected_problem',return_value=({'task':(np.ones((1,1)),np.ones(1),1.)},np.array([-2.]),np.array([2.]))):
            return v.audit_row(self.data,self.c,self.sample,row,None)
    def test_actual_matrix_weight_bounds_are_not_assumed(self):
        self.assertTrue(self.audit(self.row)['design_comparable'])
        for field,value in [('A',[[2.]]),('weight',2.),('normalization',[2.])]:
            r=copy.deepcopy(self.row);r['native_audit']['tasks'][0][field]=value;self.assertFalse(self.audit(r)['design_comparable'])
        r=copy.deepcopy(self.row);r['native_audit']['joint_upper']=[3.];self.assertFalse(self.audit(r)['bounds_passed'])
    def test_native_rejection_never_externally_accepted(self):
        r=copy.deepcopy(self.row);r.update(native_accepted=False,candidate_available=False,committed=False,common_quality_passed=False)
        self.assertIn('native-rejection-preserved',self.audit(r)['reasons'])
    def test_velocity_tolerance_not_position_tolerance(self):
        r=copy.deepcopy(self.row);r['candidate_v']=[2.000001];r['candidate_q']=[.002000001]
        self.assertFalse(v.external_check(self.data,self.c,self.sample,r)['common_quality_passed'])
    def test_placo_actual_constraint_bounds_required(self):
        r=copy.deepcopy(self.row);n=r['native_audit'];n.update(variable_unit='rad-increment',active_variable_columns=[0],joint_lower=[-.002],joint_upper=[.002]);n['tasks'][0]['lower']=[.001];n['tasks'][0]['upper']=[.001]
        self.assertFalse(self.audit(r)['bounds_passed'])
        n['problem_constraints']=[dict(type=1,priority=1,A=[[1.],[-1.]],offset=[.002,.002])]
        self.assertTrue(self.audit(r)['design_comparable'])

if __name__=='__main__':unittest.main()
