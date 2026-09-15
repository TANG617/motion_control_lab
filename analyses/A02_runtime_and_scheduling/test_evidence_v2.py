import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
s=importlib.util.spec_from_file_location('a02_evidence_v2',Path(__file__).with_name('evidence_v2.py'));v=importlib.util.module_from_spec(s);s.loader.exec_module(v)
from analysis import common as C

class EvidenceV2(unittest.TestCase):
    def unit(self,i):
        return dict(experiment_id='E10',case_id='case-'+str(i),method_id='m',arm_id='a',repeat_id=0,run_id='r',
            status='completed',validation={'status':'passed'},directory='/never/raw',artifacts=[],config={},request={'input':{}})
    def test_base_and_supplement_never_merge_expected_fixture_stays_failed(self):
        units=[self.unit(i) for i in range(5593)];units[0].update(case_id='fixture-observation-overflow',status='failed',validation={'status':'failed'},request={'input':{'fixture':True}})
        units[1]['request']['input']['fixture']=True;units[2]['status']='unavailable'
        supplement=[dict(self.unit(i),experiment_id='E09',repair_stage='E09-audit' if i<24 else 'E09-timing',status='unavailable') for i in range(240)]
        with tempfile.TemporaryDirectory() as tmp:
            ctx=types.SimpleNamespace(units=units,supplemental_units=supplement,output=Path(tmp));tables={}
            ctx.table=lambda name,rows:tables.update({name:list(rows)})
            v.analyze(ctx,lambda ctx:(ctx.output/'report.md').write_text('base report'),None)
            self.assertEqual(len(ctx.units),5593);self.assertEqual(len(tables['supplemental_audit']),240)
            rows=tables['base_evidence_classification'];self.assertEqual(rows[0]['native_status'],'failed')
            self.assertEqual(rows[0]['evidence_class'],'expected-failure-fixture');self.assertFalse(rows[0]['scientific_timing_eligible'])
            self.assertEqual(rows[1]['evidence_class'],'development-fixture');self.assertEqual(rows[2]['evidence_class'],'capability-unavailable')
            self.assertEqual(rows[3]['evidence_class'],'scientific-case-nonRT')
            report=(ctx.output/'report.md').read_text()
            self.assertTrue(report.startswith('# A02 v2'))
            self.assertLess(report.index('旧批次 5593'),report.index('base report'))
            self.assertLess(report.index('新增 E09 240'),report.index('base report'))
            self.assertLess(report.index('预期失败 fixture'),report.index('base report'))
            self.assertLess(report.index('不继承旧方法身份'),report.index('base report'))
    def test_pair_ratio_retains_explicit_numerator_denominator_and_direction(self):
        units=[self.unit(i) for i in range(5593)]
        supplement=[dict(self.unit(i),experiment_id='E09',repair_stage='E09-audit' if i<24 else 'E09-timing',status='unavailable') for i in range(240)]
        for u,method,p50 in zip(supplement[24:26],v.METHOD_COLORS,(6.,3.)):
            u.update(status='completed',method_id=method,input_hash='same',model_hash='same',config={'workload':0,'observation':'minimal','warmup_calls':0,'measured_calls':0,'fixture_p50':p50},raw_fixture=True)
        report=dict(acceptance_contract_hash=v.COMMON_ACCEPTANCE_HASH,design_comparable=True,observation_complete=True,claim_eligible=True)
        helpers=types.SimpleNamespace(DIMENSIONS=(),check_errors=lambda u:{},quality_summary=lambda rows,n:{},ratio=lambda l,r:l/r if r else None,
            timing_summary=lambda rows,c:[dict(window='steady',p50_ms=c['fixture_p50'],p99_ms=c['fixture_p50'],all_calls_retained=True)])
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);raw=out/'fixture.jsonl';raw.write_text('{"record_type":"attempt","attempt_sequence":0}\n');tables={}
            ctx=types.SimpleNamespace(units=units,supplemental_units=supplement,output=out,table=lambda name,rows:tables.update({name:list(rows)}))
            with mock.patch.object(C,'unit_json',side_effect=lambda u,n,d:report if u.get('raw_fixture') else d),mock.patch.object(C,'unit_path',side_effect=lambda u,n:raw if u.get('raw_fixture') else None):
                v.analyze(ctx,lambda ctx:(out/'report.md').write_text('base'),helpers)
            pair=tables['supplemental_method_pairs'][0]
            self.assertTrue(pair['admitted']);self.assertEqual(pair['left_method_id'],'mcc-weighted-common-servo-v1')
            self.assertEqual(pair['right_method_id'],'placo-common-servo-v1');self.assertEqual(pair['left_p50_ms'],6.)
            self.assertEqual(pair['right_p50_ms'],3.);self.assertEqual(pair['complete_method_median_time_ratio'],2.)
            self.assertIn('>1 means left method slower',pair['ratio_interpretation'])
            self.assertIn('left_method_id',pair['ratio_numerator']);self.assertIn('right_method_id',pair['ratio_denominator'])
            self.assertIn('大于 1 表示左侧方法更慢',(out/'report.md').read_text())
    def test_wrong_base_count_fails_before_any_analysis(self):
        with self.assertRaisesRegex(ValueError,'5593'):v.analyze(types.SimpleNamespace(units=[]),mock.Mock(),None)
    def test_supplement_empty_plot_reads_only_analysis_tables(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);(out/'evaluation').mkdir();(out/'evaluation/supplemental_timing.csv').write_text('window,claim_eligible,p99_ms\n')
            ctx=types.SimpleNamespace(output=out)
            (out/'report.md').write_text('frozen v2 report')
            before_csv=(out/'evaluation/supplemental_timing.csv').read_bytes();before_report=(out/'report.md').read_bytes()
            with mock.patch.object(C,'save_figure') as save,mock.patch.object(C,'unit_json',side_effect=AssertionError('raw read')):
                v.render(ctx);v.render(ctx)
            self.assertEqual(save.call_args.args[2],'F05')
            self.assertEqual((out/'evaluation/supplemental_timing.csv').read_bytes(),before_csv)
            self.assertEqual((out/'report.md').read_bytes(),before_report)
    def test_supplement_plot_facets_observation_and_retains_every_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);(out/'evaluation').mkdir();rows=[]
            for observation in v.OBSERVATIONS:
                for family in v.FAMILY_MARKERS:
                    for method in v.METHOD_COLORS:
                        for repeat in range(3):
                            rows.append(dict(unit_id='/'.join((observation,family,method,str(repeat))),window='steady',claim_eligible=True,
                                p50_ms=1+repeat,p99_ms=4+repeat,observation=observation,scene_family=family,method_id=method,
                                workload=0,input_hash='hash-'+family,model_hash='model',repeat_id=repeat))
            C.write_csv(out/'evaluation/supplemental_timing.csv',rows)
            ctx=types.SimpleNamespace(output=out)
            with mock.patch.object(C,'save_figure') as save,mock.patch.object(C,'unit_json',side_effect=AssertionError('raw read')):
                v.render(ctx)
            fig=save.call_args.args[1];selection=save.call_args.args[7]
            self.assertEqual(len(fig.axes),6)
            self.assertEqual(selection['admitted_process_count'],54)
            self.assertEqual(len(selection['selected_processes']),54)
            self.assertEqual(sum(len(c.get_offsets()) for ax in fig.axes for c in ax.collections),108)
            for column,observation in enumerate(v.OBSERVATIONS):
                self.assertEqual(fig.axes[column].get_title(),'Observation: '+observation)
                self.assertEqual(sum(len(c.get_offsets()) for c in fig.axes[column].collections),18)
            self.assertIn('repeat_id',save.call_args.args[6]);self.assertIn('input_hash',save.call_args.args[6])
            C.plotting().close(fig)
if __name__=='__main__':unittest.main()
