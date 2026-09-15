import copy
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from analysis import common as c


def definition():
    return c.read_json(c.ROOT/'analyses/A01_priority_and_robustness/definition.json')


def test_definitions():
    for directory in ('A01_priority_and_robustness','A02_runtime_and_scheduling'):
        c.validate_definition(c.read_json(c.ROOT/'analyses'/directory/'definition.json'))


def test_latest_rejected():
    d=definition();d['campaign']['plan']['locator']='/some/latest/plan.json'
    with pytest.raises(ValueError,match='latest'):c.validate_definition(d)


def test_wrong_selection_rejected():
    d=definition();d['sources']=d['sources'][:1]
    with pytest.raises(ValueError,match='selection'):c.validate_definition(d)


def test_no_bootstrap():
    d=definition();d['statistics']['bootstrap']=True
    with pytest.raises(ValueError,match='descriptive'):c.validate_definition(d)


def test_hash_change(tmp_path):
    p=tmp_path/'x';p.write_text('original');a=c.artifact(p);c.verify(a)
    p.write_text('changed')
    with pytest.raises(ValueError,match='hash'):c.verify(a)


def test_mutation_after_read(tmp_path):
    p=tmp_path/'x';p.write_text('original');ctx=c.Context(definition());ctx.check_artifact(c.artifact(p))
    p.write_text('changed')
    with pytest.raises(ValueError,match='changed'):ctx.unchanged()


def test_stream_large_array(tmp_path):
    rows=[{'n':i,'payload':'hello'*70} for i in range(10000)]
    p=tmp_path/'x';c.write_json(p,rows)
    assert list(c.array_rows(p))==rows


def test_truncated_array(tmp_path):
    p=tmp_path/'x';p.write_text('[{"x": 2}')
    with pytest.raises(ValueError,match='truncated'):list(c.array_rows(p))


def test_csv_roundtrip(tmp_path):
    p=tmp_path/'x.csv';rows=[{'a':None,'b':{'a':2}},{'a':1,'b':{'a':3}}]
    c.write_csv(p,rows);v=list(c.csv_rows(p))
    assert v[0]['a']=='' and json.loads(v[1]['b'])=={'a':3}


def pair():
    u={k:'same' for k in c.PAIR_KEYS};u.update(method_id='mcc-weighted',status='completed',validation={'status':'passed'},config={'backend':'proxqp'})
    return u,copy.deepcopy(u)


def test_different_input_cannot_pair():
    l,r=pair();r['input_hash']='different';assert 'identity:input_hash' in c.compatible(l,r)


def test_acceptance_gate():
    l,r=pair();r['method_id']='placo-controlled';assert 'E06-native-acceptance-mismatch' in c.compatible(l,r)


def test_failed_pair_retained_unavailable():
    l,r=pair();r['status']='failed';assert c.compatible(l,r)


def test_only_declared_factor_may_change():
    l,r=pair();r['config']['backend']='eiquadprog'
    assert c.compatible(l,r)
    assert not c.compatible(l,r,allowed=['backend'])


def test_declared_artifacts_only(tmp_path):
    (tmp_path/'hidden.json').write_text('{}')
    assert c.unit_path({'directory':str(tmp_path),'artifacts':[]},'hidden.json') is None


def test_manifest_v3_analysis_identity():
    import jsonschema
    v={'schema_version':'run_manifest.v3','run_kind':'analysis','run_id':'x','status':'running','phase':'development','metric_version':'study-metrics.v1','outputs':{},'analysis_id':'A01','definition_hash':'hash','analysis_version':c.VERSION,'source_policy':{},'code':[],'environment':{}}
    schema=c.read_json(c.ROOT/'contracts/manifests/run_manifest.v3.schema.json')
    jsonschema.validate(v,schema)
    v['experiment_id']='E06'
    with pytest.raises(jsonschema.ValidationError):jsonschema.validate(v,schema)


def source_fixture(tmp_path, aid='A01'):
    d=definition() if aid=='A01' else c.read_json(c.ROOT/'analyses/A02_runtime_and_scheduling/definition.json')
    exps=('E06','E07','E08') if aid=='A01' else ('E09','E10')
    d['expected_units']=len(exps);d['sources']=[];index=[]
    for exp in exps:
        root=tmp_path/exp;root.mkdir()
        u={k:'fixture' for k in c.IDENTITY}
        u.update(experiment_id=exp,case_id='fixture',arm_id='mcc-weighted',method_id='mcc-weighted',repeat_id=0,
                 run_id='fixture-run',status='completed',required=True,validation={'status':'passed'},artifacts=[])
        m=root/(exp+'.manifest.json');c.write_json(m,{'schema_version':'run_manifest.v2','experiment_id':exp,'status':'completed','units':[u]})
        inv=root/'inventory.json';c.write_json(inv,{'phase':'development','smoke':False,'required_units':[u], 'actual_units':[u],'artifacts':[]})
        s={'experiment_id':exp,'run_id':'fixture-run','run_dir':str(root),'manifest':c.artifact(m),'inventory':c.artifact(inv)}
        d['sources'].append(s);index.append(s)
    for key,data in [('execution_index',{'runs':index}),('status',{'all_experiment_waves_processed':True}),('plan',{})]:
        p=tmp_path/(key+'.json');c.write_json(p,data);d['campaign'][key]=c.artifact(p)
    return d


def test_preflight_small_closed_fixture(tmp_path):
    ctx=c.Context(source_fixture(tmp_path));assert ctx.preflight()['units']==3;ctx.unchanged()


def test_duplicate_inventory_rejected(tmp_path):
    d=source_fixture(tmp_path);s=d['sources'][0];p=Path(s['inventory']['locator']);v=c.read_json(p)
    v['actual_units']*=2;c.write_json(p,v);s['inventory']=c.artifact(p)
    c.write_json(d['campaign']['execution_index']['locator'],{'runs':d['sources']})
    d['campaign']['execution_index']=c.artifact(d['campaign']['execution_index']['locator'])
    with pytest.raises(ValueError,match='coverage'):c.Context(d).preflight()


def test_source_not_in_index_rejected(tmp_path):
    d=source_fixture(tmp_path);d['sources'][0]['run_id']='other'
    with pytest.raises(ValueError,match='pinned'):c.Context(d).preflight()


def test_closed_failure_is_valid_analysis_input(tmp_path):
    d=source_fixture(tmp_path)
    for s in d['sources']:
        p=Path(s['manifest']['locator']);v=c.read_json(p);v['status']='failed';c.write_json(p,v);s['manifest']=c.artifact(p)
    c.write_json(d['campaign']['execution_index']['locator'],{'runs':d['sources']})
    d['campaign']['execution_index']=c.artifact(d['campaign']['execution_index']['locator'])
    assert c.Context(d).preflight()['status']=='passed'


def test_a02_needs_a01():
    d=c.read_json(c.ROOT/'analyses/A02_runtime_and_scheduling/definition.json');d['admission_source']=None
    with pytest.raises(ValueError,match='A01'):c.Context(d).preflight()


def test_manifest_validator_legacy_and_v3(tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location('public_validator',c.ROOT/'tests/validate_contracts.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    p=tmp_path/'manifest.json'
    c.write_json(p,{'schema_version':'run_manifest.v3','run_kind':'analysis','analysis_id':'A01',
                    'run_id':'unit','status':'completed','phase':'development','metric_version':'study-metrics.v1',
                    'outputs':{},'definition_hash':'x','analysis_version':c.VERSION,'source_policy':{},'code':[],'environment':{}})
    assert module.validate_manifest(p)['analysis_id']=='A01'
    # New dispatch must not change the frozen v1/v2 reader's accepted identity boundary.
    import evidence
    for version in ('run_manifest.v1','run_manifest.v2'):
        c.write_json(p,{'schema_version':version,'run_kind':'experiment','experiment_id':'E01'})
        assert evidence.load_manifest(p)['experiment_id']=='E01'


def test_renderer_uses_only_tables_and_creates_new_run(tmp_path, monkeypatch):
    import types
    d=definition()
    src=tmp_path/'original';src.mkdir()
    c.write_json(src/'definition.json',d)
    c.write_csv(src/'evaluation/fixture.csv',[{'x':1,'y':2}])
    c.write_json(src/'sources/inventory.json',{'no_experiment_access':'required'})
    (src/'anomalies.md').write_text('original diagnostic page')
    out=tmp_path/'rendered'
    def create(_):out.mkdir();return out
    monkeypatch.setattr(c,'create_output',create)
    outputs={str(p.relative_to(src)):{k:v for k,v in c.artifact(p).items() if k!='locator'} for p in src.rglob('*') if p.is_file()}
    c.write_json(src/'manifest.json',{'status':'completed','outputs':outputs})
    original={str(p):c.digest(p) for p in src.rglob('*') if p.is_file()}
    def render(ctx):
        assert list(c.csv_rows(ctx.output/'evaluation/fixture.csv'))==[{'x':'1','y':'2'}]
        plt=c.plotting();fig,ax=plt.subplots();ax.plot([1],[2])
        c.save_figure(ctx,fig,'F02','fixture',['evaluation/fixture.csv'],'test plot',fields=['x','y'])
    actual=c.render_existing(types.SimpleNamespace(render=render),src)
    assert actual==out and c.read_json(out/'manifest.json')['status']=='completed'
    assert original=={str(p):c.digest(p) for p in src.rglob('*') if p.is_file()}
    assert (out/'figures/F02-fixture.svg').is_file()
    assert (out/'anomalies.md').read_text()=='original diagnostic page'
    assert c.read_json(out/'figures/F02-fixture.json')['analysis_run_id']=='rendered'


def with_admission(tmp_path, mismatch=False):
    d=source_fixture(tmp_path, 'A02')
    root=tmp_path/'A01';root.mkdir()
    prior=definition();prior['campaign']=copy.deepcopy(d['campaign'])
    if mismatch:prior['campaign']['plan']['sha256']='0'*64
    c.write_json(root/'definition.json',prior)
    for name in ('parity_table','pair_admission'):c.write_csv(root/'evaluation'/(name+'.csv'),[{'status':'unavailable'}])
    outputs={str(p.relative_to(root)):{k:v for k,v in c.artifact(p).items() if k!='locator'} for p in root.rglob('*') if p.is_file()}
    c.write_json(root/'manifest.json',{'run_kind':'analysis','analysis_id':'A01','status':'completed','definition_hash':c.stable_hash(prior),'outputs':outputs})
    d['admission_source']={'manifest':c.artifact(root/'manifest.json'),**{k:c.artifact(root/'evaluation'/(k+'.csv')) for k in ('parity_table','pair_admission')}}
    return d


def test_a02_same_campaign_admission(tmp_path):
    assert c.Context(with_admission(tmp_path)).preflight()['units']==2


def test_a02_other_campaign_admission_rejected(tmp_path):
    with pytest.raises(ValueError,match='different campaign'):c.Context(with_admission(tmp_path,True)).preflight()


def test_cli_readonly_dryrun_and_no_native_launch(tmp_path,monkeypatch):
    import types
    fixture=tmp_path/'fixture';fixture.mkdir();d=source_fixture(fixture)
    definition_path=tmp_path/'definition.json';c.write_json(definition_path,d)
    monkeypatch.setattr(sys,'argv',['fixture-analysis','--definition',str(definition_path),'--dry-run'])
    def reject(*a,**kw):raise AssertionError('dry run must not create output or launch a subprocess')
    monkeypatch.setattr(c,'create_output',reject)
    monkeypatch.setattr(c.subprocess,'Popen',reject)
    before={str(p):c.digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    assert c.cli(types.SimpleNamespace(__doc__='fixture'),definition_path)==0
    assert before=={str(p):c.digest(p) for p in tmp_path.rglob('*') if p.is_file()}


def test_cli_two_runs_same_tables_and_source_bytes(tmp_path,monkeypatch):
    import types
    fixture=tmp_path/'fixture';fixture.mkdir();d=source_fixture(fixture)
    definition_path=tmp_path/'definition.json';c.write_json(definition_path,d)
    monkeypatch.setattr(sys,'argv',['fixture-analysis','--definition',str(definition_path)])
    runs=[]
    def create(_):
        p=tmp_path/('result'+str(len(runs)));p.mkdir();runs.append(p);return p
    monkeypatch.setattr(c,'create_output',create)
    original_popen=c.subprocess.Popen
    calls=[]
    def safe_popen(argv,*a,**kw):
        assert argv[0]=='git','analysis must never start an experiment'
        calls.append(argv);return original_popen(argv,*a,**kw)
    monkeypatch.setattr(c.subprocess,'Popen',safe_popen)
    def analyze(ctx):ctx.table('values',[{'unit_id':c.unit_id(u),'value':1.25} for u in ctx.units])
    module=types.SimpleNamespace(__doc__='fixture',analyze=analyze,render=lambda ctx:None)
    before={str(p):c.digest(p) for p in fixture.rglob('*') if p.is_file()}
    for _ in range(2):assert c.cli(module,definition_path)==0
    assert calls and runs[0]!=runs[1]
    for name in ['evaluation/values.csv','evaluation/coverage.csv','combined/units.csv']:
        assert (runs[0]/name).read_bytes()==(runs[1]/name).read_bytes()
    assert before=={str(p):c.digest(p) for p in fixture.rglob('*') if p.is_file()}


@pytest.mark.parametrize('text',['[1 2]','[1,]','[] true'])
def test_bad_array_delimiters(tmp_path,text):
    p=tmp_path/'x';p.write_text(text)
    with pytest.raises(ValueError):list(c.array_rows(p))
