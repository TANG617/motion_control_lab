import json,pathlib,sys,tempfile,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from contracts import *
class Contracts(unittest.TestCase):
    def test_timeout_stops_adapter_child(self):
        import subprocess
        sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
        from run import bounded_process
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t);script=p/'spawn.py'
            script.write_text("import subprocess,pathlib,time\nc=subprocess.Popen(['sleep','30'])\npathlib.Path('child.pid').write_text(str(c.pid))\ntime.sleep(30)\n")
            code=bounded_process([sys.executable,str(script)],p,subprocess.DEVNULL,subprocess.DEVNULL,1.)
            self.assertEqual(code,124)
            child=int((p/'child.pid').read_text());state=pathlib.Path(f'/proc/{child}/status')
            if state.exists():self.assertIn('Z (zombie)',state.read_text())
    def test_request_corruption_and_explicit_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=pathlib.Path(tmp)/'input.json';p.write_text('{}')
            r=dict(schema_version='execution_request.v1',app_id='mcl_step',execution_structure='servo_step',input=dict(path=str(p),sha256=digest(p),format='json'),app_config={},execution={},observation={},tracking={},output_dir=tmp+'/out')
            validate_request(r,'mcl_step');self.assertFalse(pathlib.Path(r['output_dir']).exists())
            with self.assertRaises(ValueError):validate_request(r,'wrong')
            p.write_text('{"changed":true}')
            with self.assertRaises(ValueError):validate_request(r)
            r['input']['path']='latest'
            with self.assertRaises(jsonschema.ValidationError):validate_request(r)
    def test_duplicate_json(self):
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t)/'x';p.write_text('{"id":1,"id":2}')
            with self.assertRaises(ValueError):read(p)
    def test_remote_collision_resources_not_silently_localized(self):
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t)/'robot.urdf'
            p.write_text('<robot><link><collision><geometry><mesh filename="https://example.invalid/mesh.obj"/></geometry></collision></link></robot>')
            before=digest(p);audit=collision_resources(p)
            self.assertEqual(audit['status'],'resource_missing');self.assertEqual(digest(p),before)
    def test_old_manifest_readers(self):
        sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
        from evidence import load_manifest
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t)/'m'
            for v in (1,2):
                d=dict(schema_version=f'run_manifest.v{v}',run_kind='experiment',experiment_id='E06',status='failed')
                write(p,d);self.assertEqual(load_manifest(p),d)

class NumericalAndAccounting(unittest.TestCase):
    def test_replay_candidate_is_not_pipeline_commit(self):
        sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
        from verify import verify_output
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t);(p/'replay').mkdir()
            model=p/'robot.urdf';model.write_text('<robot><joint name="j"><limit lower="-1" upper="1"/></joint></robot>')
            write(p/'input.json',{'initial_state':{'q':[2]}})
            write(p/'model_mapping.json',{'joint_names':['j']})
            (p/'native_calls.jsonl').write_text(json.dumps({'kind':'native_solver_result','candidate_available':True,'candidate_q':[2],'accepted':False,'committed':False})+'\n')
            (p/'replay/trace.csv').write_text('accepted,otg_positions\nfalse,2\ntrue,0.1\n')
            r={'app_id':'mcl_hierarchical_kinematics_step','app_config':{'options':{'urdf':str(model)}},'input':{'path':str(p/'input.json'),'format':'json'}}
            v=verify_output(p,r,1);self.assertEqual(v['status'],'passed');self.assertEqual(v['native_exit_code'],1);self.assertFalse(v['native_status_rewritten'])
            (p/'replay/trace.csv').write_text('accepted,otg_positions\ntrue,2\n')
            self.assertEqual(verify_output(p,r,0)['status'],'failed')
    def test_rank_denominator_and_limits(self):
        sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
        from verify import nearest_rank,release_accounting,check_bounds,matrix_check
        self.assertIsNone(nearest_rank([], .99));self.assertEqual(nearest_rank([9,1,4,2],.5),2);self.assertEqual(nearest_rank([9,1,4,2],.99),9)
        self.assertEqual(release_accounting(10,4,2,4)['deadline_denominator'],10)
        self.assertEqual(release_accounting(10,4,2,0)['status'],'failed')
        self.assertEqual(check_bounds([3],[-1],[1])['status'],'failed')
        self.assertEqual(check_bounds([float('nan')],[-1],[1])['status'],'failed')
        problem={'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,0]],'b':[1]},{'A':[[0,1]],'b':[2],'enabled':False}]}
        v=matrix_check(problem,{'candidate':[1,0],'accepted':True});self.assertEqual(v['tasks'][1]['raw_squared_cost'],4)
        self.assertEqual(v['status'],'passed')

class Migration(unittest.TestCase):
    def test_duplicate_selection_before_any_app_starts(self):
        import argparse
        sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
        from run import select
        p=next(ROOT.glob('experiments/E06*/definition.json'))
        args=argparse.Namespace(experiment=None,case=None,app=None,repeat=[0])
        with self.assertRaisesRegex(ValueError,'duplicate selected execution identity'):select([p,p],args)
    def test_duplicate_and_cross_source(self):
        p=next(ROOT.glob('experiments/E06*/definition.json'))
        d=read(p);self.assertEqual(len(d['units']),21)
        d['units'].append(d['units'][0])
        with self.assertRaisesRegex(ValueError,'duplicate'):validate_definition(d)
        d=read(p);d['units'][0]['migration']['legacy_definition']=dict(d['legacy_definition'],sha256='0'*64)
        with self.assertRaisesRegex(ValueError,'cross-declaration'):validate_definition(d)
    def test_retired_fixture_not_laundered(self):
        d=read(next(ROOT.glob('experiments/E11*/definition.json')))
        retired=[u for u in d['units'] if u.get('migration',{}).get('status')=='retired_research_method']
        self.assertTrue(retired)
        for u in retired:
            self.assertFalse(u['available'])
        validate_definition(d)
if __name__=='__main__':unittest.main()
