import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'analyses/A01_priority_and_robustness'))
import e08_evidence as E


def fixture(candidate=.9,final=.9):
    raw=dict(attempt_sequence=4,source_time_s=.004,committed=True,disposition='accepted',selected_priority=2,
        completed_passes=3,passes=[dict(pass_=2)],task_scale_reference=[0.],
        evidence_linearization=dict(candidate_available=True,candidate_values=[candidate],tasks=[dict(name='left-position',
            enabled=True,enforcement='scaled',scale_group_handle=0,A=[[1.]],lower=[1.],offset=[0.])]),
        scales=[dict(name='progress-0',handle=0,evaluated=True,value=1.)],
        native_constraints=[dict(name='progress-0',component_name='scale',value=final),
            dict(name='left-position',component_name='x',state=2,value=candidate-final,maximum_violation=max(0,abs(candidate-final)-1e-7))])
    raw['passes']=[{'pass':2,'attempted':True,'succeeded':True,'native_qp_status':'PROXQP_SOLVED'}]
    geometry=dict(task='left-position',scale=1.,native_residual={'maximum':abs(candidate-1)},
        geometric_residual={'maximum':abs(candidate-1)},legacy_returned_solution_residual={'maximum':abs(candidate-1)},
        original_hard_tolerance=1e-7,native_candidate=[candidate],geometric_A=[[1.]],geometric_b=[1.])
    return raw,geometry


class FinalScaleTests(unittest.TestCase):
    def test_primary_optimum_false_positive(self):
        r,g=fixture();v=E.correct_final_scale(r,g,'mcc-hqp-3')
        self.assertEqual(v['status'],'measured');self.assertEqual(v['corrected_native_maximum'],0)
        self.assertTrue(v['old_optimum_false_positive']);self.assertEqual(v['primary_optimum_scale'],1)
        self.assertEqual(v['final_selected_scale'],.9)
    def test_true_committed_violation_survives(self):
        r,g=fixture(candidate=.9+1.0397834428882424e-7)
        v=E.correct_final_scale(r,g,'mcc-hqp-3');self.assertTrue(v['corrected_native_exceeds'])
        self.assertTrue(v['committed']);self.assertEqual(v['original_hard_tolerance'],1e-7)
    def test_boundary_false_positive_regression(self):
        r,g=fixture(candidate=.9+9.951406023528397e-8)
        v=E.correct_final_scale(r,g,'mcc-hqp-3');self.assertFalse(v['corrected_native_exceeds'])
    def test_exact_final_missing_no_drift_inference(self):
        r,g=fixture();r['native_constraints']=[];r['scales'][0]['drift']=.1
        self.assertEqual(E.correct_final_scale(r,g,'mcc-hqp-3')['status'],'insufficient')
    def test_missing_selected_pass(self):
        r,g=fixture();r['selected_priority']=None
        self.assertEqual(E.correct_final_scale(r,g,'mcc-hqp-3')['status'],'insufficient')
    def test_cross_candidate_join(self):
        r,g=fixture();g['native_candidate']=[0.]
        self.assertEqual(E.correct_final_scale(r,g,'mcc-hqp-3')['status'],'insufficient')
    def test_rejected_placeholder_never_native_candidate(self):
        r,g=fixture();r.update(committed=False,disposition='rejected');r['evidence_linearization'].update(candidate_available=False,candidate_values=[])
        v=E.correct_final_scale(r,g,'mcc-weighted');self.assertEqual(v['status'],'native-rejection-no-candidate')
        self.assertNotIn('corrected_native_maximum',v)
    def test_nonfinite_operand_unavailable(self):
        self.assertIsNone(E.exact_residual([[1.]],[1.],[0.],[float('nan')],1.))
    def test_weighted_scale_is_final_only_after_completed_solve(self):
        r,g=fixture();r['completed_passes']=1
        self.assertEqual(E.correct_final_scale(r,g,'mcc-weighted')['final_selected_scale'],1.)
        r['completed_passes']=0;self.assertEqual(E.correct_final_scale(r,g,'mcc-weighted')['status'],'insufficient')


class OriginalSourceTests(unittest.TestCase):
    def old_fixture(self):
        raw,g=fixture();raw.update(candidate_active_v=[.9],input_state={'q':[0],'v':[0]},q=[.9],v=[.9])
        check=dict(side='left',source_line=7,native_disposition='accepted',scale=1.,hard_tolerance=1e-7,
            equation_residual=[-.1],independent_J=[[1.]],desired_velocity=[1.],baseline_velocity=[0.])
        return raw,check
    def test_original_native_matrix_absence_and_exact_constraint(self):
        r,c=self.old_fixture();v=E.original_final_row(r,c,{'scale_group':'shared'})
        self.assertEqual(v['native_matrix_status'],'unavailable-original-trace')
        self.assertEqual(v['corrected_original_fd_maximum'],0);self.assertFalse(v['recorded_native_exceeds'])
    def test_original_rejected_hold_distinct(self):
        r,c=self.old_fixture();r.update(committed=False,disposition='rejected',q=[0],v=[0],candidate_active_v=[0]);c['native_disposition']='rejected'
        v=E.original_final_row(r,c,{'scale_group':'shared'})
        self.assertTrue(v['returned_state_held']);self.assertTrue(v['returned_solution_is_zero'])
        self.assertEqual(v['status'],'native-rejection-no-committed-candidate');self.assertNotIn('final_selected_scale',v)
    def test_original_does_not_consume_supplemental_candidate(self):
        r,c=self.old_fixture();old=E.original_final_row(r,c,{'scale_group':'shared'})
        new,g=fixture(candidate=.95);new_result=E.correct_final_scale(new,g,'mcc-hqp-3')
        self.assertFalse(old['recorded_native_exceeds']);self.assertTrue(new_result['corrected_native_exceeds'])
    def test_original_scale_join_mismatch_rejected(self):
        r,c=self.old_fixture();c['scale']=.5
        with self.assertRaisesRegex(ValueError,'primary scale'):E.original_final_row(r,c,{'scale_group':'shared'})
    def test_original_missing_selected_scale_unavailable(self):
        r,c=self.old_fixture();r['native_constraints']=[]
        self.assertEqual(E.original_final_row(r,c,{'scale_group':'shared'})['status'],'insufficient')


class StreamingJoinTests(unittest.TestCase):
    def run_context(self,directory,wrong_attempt=False):
        from types import SimpleNamespace
        raw,geo=fixture();raw['record_type']='attempt';geo.update(source_line=1,attempt_sequence=99 if wrong_attempt else 4)
        directory.mkdir(parents=True,exist_ok=True)
        for name,row in [('raw.jsonl',raw),('e08_native_geometry.jsonl',geo)]:
            (directory/name).write_text(json.dumps(row)+'\n')
        u=dict(experiment_id='E08',case_id='case',arm_id='mcc-hqp-3-evidence-full',repeat_id=0,method_id='mcc-hqp-3-evidence',
            directory=str(directory),config={'enforcement':'scaled','evidence_original_method':'mcc-hqp-3'},
            artifacts=[E.C.artifact(directory/n) for n in ('raw.jsonl','e08_native_geometry.jsonl')])
        ctx=SimpleNamespace(output=directory/'analysis',supplemental_units=[u])
        ctx.table=lambda name,rows:E.C.write_csv(ctx.output/'evaluation'/(name+'.csv'),rows)
        return ctx
    def test_cross_attempt_join_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctx=self.run_context(Path(tmp),True)
            with self.assertRaisesRegex(ValueError,'cross-attempt'):E.final_scale_reanalysis(ctx)
    def test_repeatable_tables_and_immutable_raw(self):
        with tempfile.TemporaryDirectory() as tmp:
            ctx=self.run_context(Path(tmp));before=E.C.artifact(Path(tmp)/'raw.jsonl')
            E.final_scale_reanalysis(ctx);first=(ctx.output/'evaluation/e08_final_scale_rows.csv').read_bytes()
            E.final_scale_reanalysis(ctx)
            self.assertEqual(first,(ctx.output/'evaluation/e08_final_scale_rows.csv').read_bytes())
            self.assertEqual(before,E.C.artifact(Path(tmp)/'raw.jsonl'))

if __name__=='__main__':unittest.main()
