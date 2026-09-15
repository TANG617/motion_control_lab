import importlib.util
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

spec=importlib.util.spec_from_file_location('a02_run',Path(__file__).with_name('run.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

class Statistics(unittest.TestCase):
    def test_nearest_rank_not_interpolation(self):
        self.assertEqual(a.quantiles([8,1,2,3])['p50'],2)
        self.assertEqual(a.quantiles([1,2,3,8])['p95'],8)
        self.assertIsNone(a.quantiles([])['max']);self.assertIsNone(a.ratio(0,0))

    def test_warmup_boundaries_rejected_remain(self):
        rows=[dict(attempt_sequence=i,window_id=w,start_ns=0,finish_ns=n*1000000,ik_call_time_ms=n,solution_quality=q)
              for i,(w,n,q) in enumerate([('cold',10,'full'),('warmup',9,'full'),('steady',1,'full'),('steady',99,'rejected')])]
        result=a.timing_summary(rows,dict(warmup_calls=1,measured_calls=2))
        self.assertEqual(result[2]['p99_ms'],99);self.assertEqual(result[2]['native_full_quality_coverage'],.5)
        self.assertEqual(result[0]['p50_ms'],10)
        rows[1]['window_id']='steady';self.assertIsNone(a.timing_summary(rows,dict(warmup_calls=1,measured_calls=2))[2]['p99_ms'])

    def test_nonfinite_timing_is_unavailable(self):
        row=dict(attempt_sequence=0,window_id='cold',start_ns=0,finish_ns=1,ik_call_time_ms=float('nan'))
        result=a.timing_summary([row],dict(warmup_calls=0,measured_calls=0))
        self.assertIsNone(result[0]['p99_ms']);self.assertIn('missing-or-nonfinite-timing',result[0]['problems'])
    def test_missing_calls_null_tail(self):
        row=dict(attempt_sequence=0,window_id='cold',start_ns=0,finish_ns=1,ik_call_time_ms=1e-6)
        result=a.timing_summary([row],dict(warmup_calls=0,measured_calls=1))
        self.assertIsNone(result[0]['p99_ms']);self.assertIn('incomplete-attempt-denominator',result[0]['problems'])

class Quality(unittest.TestCase):
    def test_all_native_categories_and_independent_status_distinct(self):
        rows=[dict(solution_quality=k,independent_status='passed') for k in ('full','partial-hierarchy','feasible-suboptimal','rejected','future-label',None)]
        r=a.quality_summary(rows,10)
        self.assertEqual(sum(r['native_quality_counts'].values()),6)
        self.assertEqual(r['native_full_quality'],1);self.assertEqual(r['native_full_quality_coverage'],.1)
        self.assertEqual(r['native_feasible_suboptimal'],1);self.assertEqual(r['native_partial_hierarchy'],1)
        self.assertEqual(r['independent_guardrail_checks_passed'],6)
        self.assertIsNone(r['independent_full_task_quality'])
    def test_invalid_ecdf_excluded(self):
        self.assertEqual(a.ecdf_summary([{'ik_call_time_ms':float('nan')}],[{'all_calls_retained':False}]),[])
    def test_ecdf_uses_nearest_rank(self):
        rows=[dict(window_id='steady',ik_call_time_ms=v) for v in [1,2,3,9]]
        result=a.ecdf_summary(rows,[{'all_calls_retained':True}])
        self.assertEqual(result[50]['ik_call_time_ms'],2)
        self.assertEqual(result[99]['ik_call_time_ms'],9)

class Pairing(unittest.TestCase):
    def unit(self,method,backend):
        return dict(experiment_id='E09',case_id='c',method_id=method,arm_id=method+'-'+backend,repeat_id=0,
            session_id='s',split_id='development',window_id='all',component_id='both',input_hash='i',model_hash='m',
            status='completed',validation={'status':'passed'},directory='/fixture',run_id='r',config={'backend':backend})
    def test_acceptance_policy_blocks_cross_method_ratio(self):
        l=self.unit('mcc-controlled','a');r=self.unit('placo-controlled','b')
        ctx=types.SimpleNamespace(units=[l,r],definition={'pairing':{'allowed_factors':{'backend':['backend']}}})
        summaries={a.base_row(u)['unit_id']:{'p50_ms':1} for u in (l,r)}
        rows=a.pairing(ctx,summaries,{})
        self.assertFalse(rows[0]['admitted']);self.assertIsNone(rows[0]['median_time_ratio'])
    def test_pinned_a01_policy_consumed(self):
        l=self.unit('mcc-controlled','a');r=self.unit('mcc-controlled','b')
        ctx=types.SimpleNamespace(units=[l,r],definition={'pairing':{'allowed_factors':{'backend':['backend']}}})
        summaries={a.base_row(u)['unit_id']:{'p50_ms':1} for u in (l,r)}
        policy=[{'scope':'method-policy','left_method_id':'mcc-controlled','right_method_id':'mcc-controlled','admitted':'False','reasons':'fixture denial'}]
        self.assertFalse(a.pairing(ctx,summaries,{},policy)[0]['admitted'])

class Schedule(unittest.TestCase):
    def setUp(self):
        self.config=dict(duration_s=5e-3,**{rate:1000 for rate in a.WORKERS.values()})
        self.events=[dict(worker='primary',sequence=i,release_ns=i*1000000,start_ns=i*1000000,
            deadline_ns=(i+1)*1000000,finish_ns=(i+1)*1000000+(1 if i in (1,2) else 0),skipped=i==1) for i in range(5)]
    def primary(self,events=None,**native):
        return a.release_summary(self.events if events is None else events,self.config,native)[0]
    def test_skipped_is_miss_full_denominator(self):
        row=self.primary(operation='completed');self.assertEqual(row['planned_releases'],5)
        self.assertEqual(row['observed_deadline_misses'],2);self.assertEqual(row['deadline_miss_fraction'],.4)
        self.assertEqual(row['longest_observed_miss_run'],2)
    def test_failure_suffix_not_run(self):
        row=self.primary(self.events[:3],operation='failed');self.assertEqual(row['not_run_releases'],2)
        self.assertIsNone(row['deadline_miss_fraction']);self.assertEqual(row['deadline_miss_fraction_status'],'lower-bound-only')
        self.assertEqual(row['deadline_miss_fraction_lower_bound'],.4)
        self.assertEqual(row['unknown_releases'],0);self.assertIsNone(row['duration_p99_ms'])
    def test_overflow_missing_not_notrun(self):
        row=self.primary(self.events[:3],operation='completed',timing_buffer_overflow=10)
        self.assertEqual(row['unknown_releases'],2);self.assertEqual(row['not_run_releases'],0)
        self.assertEqual(row['longest_miss_status'],'lower-bound')
    def test_interior_gap_breaks_known_streak(self):
        row=self.primary(self.events[:2]+self.events[3:],operation='failed')
        self.assertEqual(row['unknown_releases'],1);self.assertEqual(row['not_run_releases'],0)
    def test_release_to_finish_hand_values_and_incomplete_mask(self):
        events=[dict(r) for r in self.events]
        for r in events:r['start_ns']=r['release_ns']+200000
        row=self.primary(events,operation='completed')
        self.assertEqual(row['release_to_finish_p50_ms'],1)
        self.assertAlmostEqual(row['release_to_finish_p99_ms'],1.000001)
        self.assertAlmostEqual(row['duration_p50_ms'],.8)
        row=self.primary(events[:3],operation='failed')
        self.assertIsNone(row['release_to_finish_p99_ms']);self.assertIsNone(row['release_to_finish_max_ms'])
    def test_declared_plan_extra_event_and_deadline_change(self):
        plan=[dict(worker='primary',sequence=i,release_ns=i*1000000,deadline_ns=(i+1)*1000000) for i in range(5)]
        events=self.events+[dict(self.events[0],sequence=99)]
        result=a.release_summary(events,self.config,{'operation':'completed'},plan)[0]
        self.assertGreater(result['grid_errors'],0);self.assertFalse(result['window_completed'])
        events=[dict(r) for r in self.events];events[0]['deadline_ns']+=1
        self.assertGreater(a.release_summary(events,self.config,{'operation':'completed'},plan)[0]['grid_errors'],0)
    def test_missing_or_negative_execution_interval_blocks_tail(self):
        for finish in (None,-1):
            events=[dict(r) for r in self.events];events[0]['finish_ns']=finish
            row=self.primary(events,operation='completed')
            self.assertEqual(row['invalid_intervals'],1);self.assertIsNone(row['duration_p99_ms'])
    def test_actual_resources_do_not_use_arm_name(self):
        r=a.resource_identity({'scheduler':'asynchronous-two-core','schedule_mode':'realtime'},
            {'effective_primary_cpu':3,'effective_secondary_cpu':3,'thread_count':2},{'platform':{'boot_id':'old'}})
        self.assertEqual(r['effective_cpu_count'],1);self.assertFalse(r['rt_claim'])
        self.assertEqual(r['boot_id'],'old')
        self.assertEqual(a.resource_identity({'schedule_mode':'virtual'},{},{})['timing_interpretation'],'virtual-semantics')
    def test_display_keeps_extrema_and_events(self):
        rows=[{'value':i%5,'proposal_revision':i//2} for i in range(100)]
        rows[55]['value']=1000;result=a.compact_display(rows,['value'],bins=5)
        self.assertIn(rows[55],result);self.assertGreater(len(result),40)

class Rendering(unittest.TestCase):
    def test_render_reads_only_tables(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'evaluation').mkdir()
            for name in ('timing_ecdf','timing_quantiles','problem_dimensions','resource_tradeoffs','representative_timeline','T03_platform'):
                (p/'evaluation'/f'{name}.csv').write_text('status\n')
            ctx=types.SimpleNamespace(output=p,definition={'display':{}},figures=[])
            with mock.patch.object(a,'unit_path',side_effect=AssertionError('raw accessed')),mock.patch.object(a,'unit_json',side_effect=AssertionError('raw accessed')),mock.patch.object(a,'save_figure') as save:
                a.render(ctx)
            self.assertEqual([c.args[2] for c in save.call_args_list],['F05','F06','F08','T03'])

if __name__=='__main__':unittest.main()
