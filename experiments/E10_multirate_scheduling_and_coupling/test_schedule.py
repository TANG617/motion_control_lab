"""Hand-authored deterministic scheduling fixtures, not real-time evidence."""
import importlib.util,pathlib,unittest,json,tempfile,copy
HERE=pathlib.Path(__file__).parent
spec=importlib.util.spec_from_file_location('e10verify',HERE/'verify.py');v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
class Schedule(unittest.TestCase):
    def join_fixture(self):
        config=dict(duration_s=.001,solver_hz=1000,secondary_hz=1000,target_hz=1000,feedback_hz=1000,output_hz=1000,coupling_enabled=True,schedule_mode='virtual')
        events=[dict(worker=w,sequence=0,release_ns=0,start_ns=0,finish_ns=100,deadline_ns=1000000,skipped=False,deadline_miss=False) for w in v.RELEASE_WORKERS]
        events[3].update(capture_ns=0,q=[.2],captured_committed_sequence=5)
        events[4].update(q=[.2],committed=False)
        row=dict(worker='primary',attempt_sequence=0,state_sequence=0,proposal_revision=2,proposal_created_ns=100,proposal_age_ns=100,proposal_state_age_ns=200,proposal_state_capture_ns=0,start_ns=200,measured_state_age_ns=200,measured_state_capture_ns=0,coupling_enabled=True,measured_q=[.2],measured_state_committed_sequence=5)
        return config,events,row
    def test_publication_index_keeps_last_eligible_input_order(self):
        config,events,row=self.join_fixture()
        events.extend([dict(worker='proposal-publication',revision=2,delivery_ns=190,accepted=False),
                       dict(worker='proposal-publication',revision=2,delivery_ns=100,accepted=True),
                       dict(worker='proposal-publication',revision=2,delivery_ns=300,accepted=False),
                       dict(worker='proposal-publication',revision=9)])
        self.assertTrue(v.check(events,[row],config,{'timing_buffer_overflow':0})['passed'])
        events[5],events[6]=events[6],events[5]
        self.assertIn('failed-proposal-consumed',v.check(events,[row],config,{'timing_buffer_overflow':0})['failures'])
    def test_capture_index_keeps_last_duplicate_and_missing_behavior(self):
        config,events,row=self.join_fixture()
        events.append(dict(worker='proposal-publication',revision=2,delivery_ns=100,accepted=True))
        duplicate=copy.deepcopy(events[3]);duplicate['q']=[.3];events.append(duplicate)
        self.assertIn('feedback-state-join',v.check(events,[row],config,{'timing_buffer_overflow':0})['failures'])
        events[3],events[-1]=events[-1],events[3]
        self.assertNotIn('feedback-state-join',v.check(events,[row],config,{'timing_buffer_overflow':0})['failures'])
        row.update(measured_state_capture_ns=1,measured_state_age_ns=199)
        self.assertIn('feedback-capture-missing',v.check(events,[row],config,{'timing_buffer_overflow':0})['failures'])
        row['proposal_revision']=3
        self.assertIn('consumed-publication-missing',v.check(events,[row],config,{'timing_buffer_overflow':0})['failures'])
    def test_progress_finishes_with_invalid_attempt_and_skipped_output(self):
        config,events,row=self.join_fixture()
        del row['attempt_sequence']
        events[-1].update(skipped=True,deadline_miss=True)
        attempts=[row,{'worker':'secondary'}]
        with tempfile.TemporaryDirectory() as root:
            out=pathlib.Path(root);total=v.progress_total(events,attempts)
            self.assertEqual(total,6)
            result=v.check(events,attempts,config,{'timing_buffer_overflow':0},progress=v.SampleProgress(out,'app-validation',total))
            self.assertEqual(result,v.check(events,attempts,config,{'timing_buffer_overflow':0}))
            state=json.loads((out/'sample_progress.json').read_text())
            self.assertEqual((state['current'],state['total']),(total,total))
    def test_secondary_first_divided_and_repeated_consumption(self):
        rows=v.virtual_reference(5000,1000,2000,100,200)
        self.assertEqual([r['start_ns'] for r in rows],[200,1000,2200,3000,4200])
        self.assertEqual([r['proposal_revision'] for r in rows],[1,1,2,2,3])
        self.assertEqual([r['proposal_age_ns'] for r in rows],[0,800,0,800,0])
        self.assertEqual([r['state_age_ns'] for r in rows],[200,1000,200,1000,200])
    def test_pause_preserves_old_proposal_until_recovery(self):
        rows=v.virtual_reference(5000,1000,2000,100,200,pause=(2000,4000))
        self.assertEqual([r['proposal_revision'] for r in rows],[1,1,1,1,2])
        self.assertEqual(rows[3]['proposal_age_ns'],2800)
    def test_skipped_release_is_deadline_denominator(self):
        rows=v.virtual_reference(3000,1000,1000,100,1200)
        self.assertEqual(len(rows),3);self.assertTrue(all(r['skipped'] and r['deadline_miss'] for r in rows))
    def test_exact_deadline_is_success(self):
        row=v.virtual_reference(1000,1000,1000,800,200)[0];self.assertFalse(row['deadline_miss']);self.assertEqual(row['finish_ns'],1000)
    def test_initial_proposal_unavailable(self):
        row=v.virtual_reference(1000,1000,1000,100,200,pause=(0,1000))[0];self.assertEqual(row['proposal_revision'],0);self.assertIsNone(row['proposal_age_ns'])
    def test_clock_coverage_and_overflow(self):
        c=dict(duration_s=.001,solver_hz=1000,secondary_hz=1000,target_hz=1000,feedback_hz=1000,output_hz=1000,coupling_enabled=False,schedule_mode='virtual')
        events=[dict(worker=w,sequence=0,release_ns=0,start_ns=0,finish_ns=100,deadline_ns=1000000,skipped=False,deadline_miss=False) for w in ['primary','secondary','target','feedback-capture','output']]
        self.assertTrue(v.check(events,[],c,{'timing_buffer_overflow':0})['passed'])
        self.assertIn('observation-overflow-tail-unavailable',v.check(events,[],c,{'timing_buffer_overflow':1})['failures'])
        self.assertFalse(v.check(events[:-1],[],c,{'timing_buffer_overflow':0})['passed'])
    def test_new_run_resets_state_and_proposal_versions(self):
        first=v.virtual_reference(5000,1000,2000,100,200)
        reset=v.virtual_reference(1000,1000,1000,100,200,pause=(0,1000))
        self.assertGreater(first[-1]['proposal_revision'],1)
        self.assertEqual(reset[0]['proposal_revision'],0)
        self.assertIsNone(reset[0]['state_age_ns'])
        self.assertEqual(first,v.virtual_reference(5000,1000,2000,100,200))
    def test_failed_publication_cannot_be_consumed(self):
        c=dict(duration_s=0.,solver_hz=1000,secondary_hz=100,target_hz=100,feedback_hz=1000,output_hz=1000,coupling_enabled=True,schedule_mode='virtual')
        events=[dict(worker='proposal-publication',revision=2,delivery_ns=100,accepted=False)]
        row=dict(worker='primary',attempt_sequence=0,input_sequence=0,state_sequence=0,proposal_revision=2,proposal_created_ns=100,proposal_age_ns=100,proposal_state_age_ns=200,proposal_state_capture_ns=0,start_ns=200,measured_state_age_ns=200,measured_state_capture_ns=0,coupling_enabled=True)
        self.assertIn('failed-proposal-consumed',v.check(events,[row],c,{'timing_buffer_overflow':0})['failures'])
        row['coupling_enabled']=False
        self.assertTrue(v.check(events,[row],c,{'timing_buffer_overflow':0})['passed'])
        del row['attempt_sequence']
        self.assertIn('primary-missing-attempt_sequence',v.check(events,[row],c,{'timing_buffer_overflow':0})['failures'])
    def test_matrix_families_and_pairing(self):
        d=json.load(open(HERE/'declarations/study_legacy.v2.json'));all_units=d['units'];self.assertEqual(len(all_units),193);units=[u for u in all_units if u['case_id']!='fixture-observation-overflow'];self.assertEqual(len(units),192)
        fixture=next(u for u in all_units if u['case_id']=='fixture-observation-overflow');self.assertEqual(fixture['phases'],['development']);self.assertEqual(fixture['config']['buffer_capacity'],8)
        keys=[(u['case_id'],u['arm_id']) for u in units];self.assertEqual(len(keys),len(set(keys)))
        self.assertEqual({u['config']['secondary_hz'] for u in units},{20.,50.,100.,200.,1000.})
        for u in units:
            counterpart=u['arm_id'].replace('proposal-disabled','coupled') if not u['config']['coupling_enabled'] else u['arm_id'].replace('coupled','proposal-disabled')
            self.assertIn((u['case_id'],counterpart),keys)
if __name__=='__main__':unittest.main()
