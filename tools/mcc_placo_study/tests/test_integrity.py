import os,pathlib,signal,subprocess,sys,tempfile,time,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from evidence import *
from inputs import *
from freeze import frozen_prerequisites
from verify import verify_unit
from check_inventory import check
class Integrity(unittest.TestCase):
    def test_fault_injected_fk_mapping_and_nan(self):
        with tempfile.TemporaryDirectory() as t:
            t=pathlib.Path(t);c=canonical_snapshot('/workspace/models/r1.cos.urdf',fixture=True)
            (t/'raw.jsonl').write_text(json.dumps({'q':[100.]*20,'joint_names':c['joint_names'],'attempt_sequence':0})+'\n')
            self.assertEqual(verify_unit(t,c)['status'],'failed')
            (t/'raw.jsonl').write_text(json.dumps({'q':Q0,'joint_names':list(reversed(JOINTS))})+'\n')
            self.assertEqual(verify_unit(t,c)['status'],'failed')
            (t/'raw.jsonl').write_text('{partial')
            self.assertEqual(verify_unit(t,c)['status'],'failed')
    def test_formal_gate_not_boolean_freeze(self):
        self.assertTrue(frozen_prerequisites({'freeze':True},{'boot_id':'x','affinity':[]}))
if __name__=='__main__':unittest.main()

class LagMeasurement(unittest.TestCase):
    def test_fixed_delay_and_censoring(self):
        import numpy as np
        from metrics import correlation_lag,event_response_lag
        rng=np.random.default_rng(42);reference=rng.normal(size=200);actual=np.r_[np.zeros(3),reference[:-3]]
        self.assertAlmostEqual(correlation_lag(reference,actual,.001,.010)['value'],.003)
        self.assertEqual(correlation_lag(np.zeros(10),np.zeros(10),.001,.003)['status'],'not-applicable')
        self.assertEqual(correlation_lag(np.full(25,.35005080167994268),np.full(25,.35005080167994268),.001,.010)['status'],'not-applicable')
        self.assertAlmostEqual(event_response_lag([0,.1,.2,.3],[0,0,.6,1],.1,0,1)['value'],.1)
        self.assertEqual(event_response_lag([0,.1],[0,0],0,0,1)['reason'],'right-censored')

class ProgressMeasurement(unittest.TestCase):
    def test_progress_stall_recovery(self):
        from metrics import directional_progress,stall_duration,recovery_time
        self.assertAlmostEqual(directional_progress([0,0,0],[1,0,0],[0,0,0],[.2,.8,0])['value'],.2)
        self.assertEqual(directional_progress([0,0,0],[0,0,0],[0,0,0],[0,0,0])['status'],'not-applicable')
        self.assertEqual(stall_duration([0,1,2,3],[0,0,0,1],.01)['value'],2.)
        r=recovery_time([0,1,2,3],[1,.1,.01,.01],[1,.1,.01,.01],[True]*4,1,.02,.02,.5,4)
        self.assertEqual(r['value'],1.);self.assertFalse(r['censored'])
        r=recovery_time([0,1],[1,1],[1,1],[True,False],.5,.1,.1,.5,2)
        self.assertTrue(r['censored']);self.assertEqual(r['observed_after_release_s'],1.5)

class ResultRecordCoverage(unittest.TestCase):
    def test_begin_only_is_not_analytic_result(self):
        from verify import verify_unit
        import tempfile,pathlib,json
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory)
            (root/'raw.jsonl').write_text(json.dumps({'record_type':'attempt_started'})+'\n')
            check=verify_unit(root,{})
            self.assertEqual(check['status'],'unavailable')
            self.assertEqual(check['result_rows'],0)
            (root/'raw.jsonl').write_text(json.dumps({'record_type':'analytic','candidate':[1.,2.]})+'\n')
            check=verify_unit(root,{})
            self.assertEqual(check['status'],'unavailable')
            self.assertEqual(check['result_rows'],1)

class InputOwnership(unittest.TestCase):
    def test_snapshot_mutation_does_not_change_seed_or_initial_defaults(self):
        before=canonical_snapshot('/workspace/models/r1.cos.urdf',fixture=True)
        expected=stable_hash(before)
        before['initial_state']['q'][0]=999
        before['samples'][0]['q'][1]=999
        before['joint_names'][0]='corrupt'
        before['active_joint_names'].clear()
        after=canonical_snapshot('/workspace/models/r1.cos.urdf',fixture=True)
        self.assertEqual(stable_hash(after),expected)
        after['initial_state']['q'][0]=123
        self.assertNotEqual(after['samples'][0]['q'][0],123)

class MetricRoles(unittest.TestCase):
    def test_role_relabel_keeps_values_and_source_bytes(self):
        from relabel_metrics import relabel
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);unit=root/'run/units/E06/case/method/0';unit.mkdir(parents=True)
            source=unit/'metrics.jsonl';row={'metric_id':'position_error','role':'guardrail','value':.125,'source_locator':'original#1'}
            source.write_text(json.dumps(row)+'\n');original=artifact(source)
            write_json(unit/'request.json',{'identity':{'experiment_id':'E06'}})
            definition=root/'definition.json';write_json(definition,{'experiment_id':'E06','metric_roles':{'position_error':'primary'}})
            result=relabel(definition,root/'run',root/'corrected')
            corrected=read_json(result['records'][0]['corrected']['locator'])
            self.assertEqual(corrected,{**row,'role':'primary'});self.assertTrue(verify_artifact(original))
            with self.assertRaises(FileExistsError):relabel(definition,root/'run',root/'corrected')
