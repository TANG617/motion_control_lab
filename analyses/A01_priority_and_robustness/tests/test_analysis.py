import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('a01_analysis',HERE/'run.py')
A=importlib.util.module_from_spec(spec);spec.loader.exec_module(A)


class Context:
    def __init__(self,root,units):
        self.output=root;self.units=units;self.figures=[]
        self.definition={'analysis_id':'A01','display':{'downsampling':'fixed extrema and event edges'}}
    def table(self,name,rows,fields=None):A.C.write_csv(self.output/'evaluation'/(name+'.csv'),rows,fields)
    def progress(self,*args):pass


def fixture(root,method='mcc-hqp-2',primary=False,case='r1_hold_tcp_fixture',failed=False):
    arm=method+('-primary-only' if primary else '-full');p=root/arm;p.mkdir()
    config={'primary_only':primary,'secondary_tasks':not primary,'state_mode':'snapshot','preservation_tolerance':1e-7,'hard_tolerance':1e-7,'secondary_weight':1}
    files={'request.json':{'config':config,'input':{'initial_state':{'q':[0]},'limits':{'lower':[-1],'upper':[1]},'active_joint_names':['j']}},
           'method_semantics.json':{'method_id':method},'priority_checks.json':[{'source_line':2,'task':'left-position','status':'pass','drift':[5e-8],'preservation_ratio':.5,'independent_final_residual':[0.2]}],
           'secondary_effects.json':[{'task':'left-elbow','candidate_secondary_cost':4 if primary else 2,'primary_only_secondary_cost':5,'absolute':1 if primary else 3,'relative':.2 if primary else .6}],
           'progress_recovery.json':[{'side':'left','source_time_s':0.,'attempt_sequence':0,'native_committed':True,'position_error_m':.01,'orientation_error_rad':.001}],
           'progress_recovery_metrics.json':[{'side':'left','directional_progress':[{'status':'not-applicable','value':None}], 'recovery':{'status':'unavailable','censored':True},'stall':{'status':'unavailable'}}]}
    for f,data in files.items():(p/f).write_text(json.dumps(data))
    (p/'raw.jsonl').write_text(json.dumps({'record_type':'attempt_started','attempt_sequence':0})+'\n'+json.dumps({'record_type':'result','attempt_sequence':0,'source_time_s':0.,'committed':True,'disposition':'accepted','completed_passes':2,'requested_passes':2})+'\n')
    u=dict(experiment_id='E07',case_id=case,arm_id=arm,method_id=method,repeat_id=0,session_id='s',split_id='development',window_id='all',component_id='app',input_hash='i',model_hash='m',run_id='frozen',status='completed',validation={'status':'failed' if failed else 'passed','failures':[]},directory=str(p),config=config)
    u['artifacts']=[A.C.artifact(f) for f in p.iterdir()];return u


class AnalysisTests(unittest.TestCase):
    def test_zero_denominator(self):
        self.assertEqual(A.gain(0,2),(-2,None,'not-applicable-zero-reference'))
        self.assertEqual(A.gain(None,2),(None,None,'unavailable'))
    def test_multisolution_is_objective_based(self):
        self.assertEqual(A.gain(3,3),(0,0,'ok'))
        # No joint-vector difference enters the metric; equal objective is no effect.
    def test_extrema_and_discrete_edges_are_preserved(self):
        points=[{'y':i%7,'event':i>=503} for i in range(1000)];points[275]['y']=999
        shown=A.select_points(points,['y'],['event'],bins=10)
        for i in (0,999,275,502,503):self.assertIn(points[i],shown)
        self.assertEqual(max(p['y'] for p in shown),999)
    def test_aggregate_missing_is_not_zero(self):
        agg=A.new_aggregate();A.add(agg,None);A.add(agg,4)
        self.assertEqual(A.summary(agg,'x')['x_mean'],4)
        self.assertEqual(A.summary(agg,'x')['x_samples'],2)
    def test_initial_infeasible_not_attributed_to_candidate(self):
        self.assertEqual(A.initial_violation({'input':{'initial_state':{'q':[2]},'limits':{'lower':[-1],'upper':[1]}}}),1)
        kind,text=A.anomaly_kind({'status':'completed','validation':{'failures':[{'reason':'hard'}]}},[],1,{})
        self.assertEqual(kind,'initial-hard-violation');self.assertIn('拒绝',text)
    def test_missing_reference_and_acceptance_gate(self):
        with tempfile.TemporaryDirectory() as d:
            u=fixture(Path(d));rs=A.paired_secondary([u],[])
            self.assertEqual(rs[0]['status'],'unavailable')
            policies=[r for r in A.pair_rows([u]) if r['scope']=='method-policy' and 'placo' in r['left_method_id']]
            self.assertTrue(all(not r['admitted'] for r in policies))
    def test_same_case_pair_and_wrong_hash(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);u=fixture(root);p=fixture(root,primary=True)
            rows=list(A.pair_rows([u,p]));self.assertTrue(rows[0]['admitted'])
            p['input_hash']='changed';self.assertFalse(list(A.pair_rows([u,p]))[0]['admitted'])
    def test_failed_source_cannot_support_effect(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);u=fixture(root,failed=True);p=fixture(root,primary=True)
            rows=list(A.pair_rows([u,p]));self.assertFalse(rows[0]['admitted'])
    def test_generated_weight_family_is_one_family(self):
        self.assertEqual(A.family({'case_id':'r1_weight_sweep_0.01_snapshot'}),'r1_weight_sweep')
        self.assertEqual(A.family({'case_id':'r1_weight_sweep_100.0_snapshot'}),'r1_weight_sweep')

    def test_raw_exact_line_join_and_states(self):
        with tempfile.TemporaryDirectory() as d:
            u=fixture(Path(d));summary,byline,points,events=A.trace_summary(u,{},True)
            self.assertEqual(summary['started_rows'],1);self.assertTrue(byline[2]['committed']);self.assertEqual(byline[2]['attempt_sequence'],0)
    def test_reproducible_analysis_and_render_only_tables(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);sources=root/'sources';sources.mkdir();u=fixture(sources);p=fixture(sources,primary=True)
            before={f:str(A.C.digest(f)) for f in sources.rglob('*') if f.is_file()}
            one=root/'one';two=root/'two';one.mkdir();two.mkdir()
            c1=Context(one,[u,p]);c2=Context(two,[u,p]);A.analyze(c1);A.analyze(c2)
            for file in (one/'evaluation').glob('*.csv'):self.assertEqual(file.read_bytes(),(two/'evaluation'/file.name).read_bytes())
            effect=list(A.C.csv_rows(one/'evaluation/paired_effects.csv'))[0]
            self.assertEqual(float(effect['absolute_gain']),2)
            recovery=list(A.C.csv_rows(one/'evaluation/recovery.csv'))[0]
            self.assertTrue(json.loads(recovery['recovery'])['censored'])
            # Delete in-memory source units: renderer must use generated CSVs only.
            c1.units=[];A.render(c1)
            self.assertTrue((one/'figures/F02-conditions-and-coverage.svg').exists())
            self.assertEqual(before,{f:str(A.C.digest(f)) for f in sources.rglob('*') if f.is_file()})


if __name__=='__main__':unittest.main()


class ReportScopeTests(unittest.TestCase):
    def test_report_counts_use_experiment_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);ctx=Context(root,[])
            ctx.table('priority_checks',[{'experiment_id':'E07'},{'experiment_id':'E08'}])
            ctx.table('recovery',[{'experiment_id':'E06'},{'experiment_id':'E07'},{'experiment_id':'E08'}])
            (root/'report.md').write_text('保留 2 个逐单元逐任务的保持检查汇总；恢复记录共 3 行。')
            before={p.name:A.C.digest(p) for p in (root/'evaluation').glob('*.csv')}
            A.refresh_report_counts(ctx)
            self.assertIn('保留 1 个',(root/'report.md').read_text())
            self.assertIn('恢复记录共 1 行',(root/'report.md').read_text())
            self.assertEqual(before,{p.name:A.C.digest(p) for p in (root/'evaluation').glob('*.csv')})
