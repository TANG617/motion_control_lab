"""Labelled hand fixtures for E09 timing collection; never experiment results."""
import importlib.util,pathlib,unittest
HERE=pathlib.Path(__file__).parent
spec=importlib.util.spec_from_file_location('e09verify',HERE/'verify.py');verify=importlib.util.module_from_spec(spec);spec.loader.exec_module(verify)
class Measurements(unittest.TestCase):
    def rows(self):
        return [dict(attempt_sequence=i,start_ns=1000*i,finish_ns=1000*i+100,ik_call_time_ms=.0001,window_id=['cold','warmup','steady'][i],allocation_count=None,solution_quality='rejected' if i==1 else 'full') for i in range(3)]
    def test_cold_failure_and_complete_denominator(self):
        r=verify.check(self.rows(),{'warmup_calls':1,'measured_calls':1});self.assertTrue(r['passed']);self.assertEqual(r['rejected_attempts'],1)
    def test_missing_sample_invalidates_tail(self):
        r=verify.check(self.rows()[:2],{'warmup_calls':1,'measured_calls':1});self.assertFalse(r['passed']);self.assertEqual(r['tail_estimation'],'unavailable')
    def test_window_and_timing_corruption(self):
        rows=self.rows();rows[0]['window_id']='steady';rows[1]['ik_call_time_ms']=1
        self.assertEqual(set(verify.check(rows,{'warmup_calls':1,'measured_calls':1})['failures']),{'window-boundary','duration-mismatch'})
    def test_nearest_rank_includes_failed_attempt(self):
        self.assertEqual(verify.nearest_rank([1.,2.,100.],.99),100.)
    def test_full_matrix_has_unique_units(self):
        import json
        d=json.load(open(HERE/'definition.json'));keys=[(u['case_id'],u['arm_id']) for u in d['units']];self.assertEqual(len(keys),1800);self.assertEqual(len(keys),len(set(keys)))
        self.assertEqual({u['config']['mode'] for u in d['units']},{'ServoStep','TargetSolve'})
        self.assertTrue(any('supports ServoStep only' in (u.get('unavailable_reason') or '') for u in d['units']))
if __name__=='__main__':unittest.main()
