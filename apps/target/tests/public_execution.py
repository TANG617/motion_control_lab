#!/usr/bin/env python3
"""Bounded actual-app normal/request equivalence, windows and failure preservation."""
import argparse,copy,hashlib,json,pathlib,subprocess,tempfile

def run(binary,source,root):
    def write(name,value):
        p=root/name;p.write_text(json.dumps(value));return p
    counter=0
    def call(args,ok=True):
        nonlocal counter
        counter+=1
        p=subprocess.run([binary,*map(str,args)],capture_output=True,text=True,timeout=30)
        (root/f'command-{counter:03}.json').write_text(json.dumps({'argv':[binary,*map(str,args)],'exit_code':p.returncode}))
        (root/f'command-{counter:03}.stdout').write_text(p.stdout)
        (root/f'command-{counter:03}.stderr').write_text(p.stderr)
        assert (p.returncode==0)==ok,(p.returncode,p.stdout,p.stderr,args)
        return p
    def rows(path):return [json.loads(x) for x in (path/'raw.jsonl').read_text().splitlines()]
    if source.suffix=='.urdf':
        canonical=json.loads((pathlib.Path(__file__).parent/'hold_input.json').read_text());canonical['model']={'locator':str(source.resolve()),'sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    else:canonical=json.loads(source.read_text())
    canonical['samples']=[{'source_time_s':0},{'source_time_s':.01}]
    ip=write('input.json',canonical)
    cap=json.loads(call(['--describe-capabilities']).stdout);assert cap['app_id']=='mcl_target'
    for solver,backend in [('mcc','eiquadprog'),('mcc','proxqp'),('placo','eiquadprog')]:
      for mode in ['minimal','full','cpp-new']:
        stem=solver+'-'+backend+'-'+mode
        config={'urdf_path':canonical['model']['locator'],'solver':solver,'backend':backend,'target_space':'frame'}
        execution={'cold_calls':1,'warmup_calls':1,'measured_calls':2,'state_mode':'snapshot'}
        cp=write(stem+'-config.json',config);ep=write(stem+'-execution.json',execution)
        normal=root/(stem+'-normal');reqout=root/(stem+'-request')
        # Normal batch defaults full; matching full makes deterministic parity check.
        call(['batch','--input',ip,'--config',cp,'--execution',ep,'--output',normal])
        req={'schema_version':'execution_request.v1','app_id':'mcl_target','execution_structure':'target_solve','input':{'path':str(ip),'sha256':hashlib.sha256(ip.read_bytes()).hexdigest(),'format':'json'},'app_config':config,'execution':execution,'observation':{'mode':mode},'tracking':{'opaque':'not-an-algorithm-selector'},'output_dir':str(reqout)}
        rp=write(stem+'-request.json',req)
        call(['--request',rp,'--dump-resolved-options']);assert not reqout.exists()
        call(['--request',rp]);a=[x for x in rows(normal) if x['record_type']=='result'];b=[x for x in rows(reqout) if x['record_type']=='result']
        assert [x['q'] for x in a]==[x['q'] for x in b]
        assert [x['phase'] for x in b]==['cold','warmup','steady','steady']
        assert all(x['committed'] for x in b)
        native=[x for x in rows(reqout) if x['record_type']=='native_result'];assert len(native)>=4
        assert all('candidate_q' in x and 'candidate_v' in x and 'commit_eligible' in x for x in native)
        if mode!='minimal':assert all(x['native_task_rows'] for x in native)
        if mode=='cpp-new':assert all(x['cpp_new_count']>=0 for x in native)
        call(['--request',rp],False);call(['--request',rp,'--backend','proxqp'],False)
        for mutate in [lambda r:r['app_config'].update({'hidden_algorithm':True}),lambda r:r['execution'].update({'state_mode':'guess'}),lambda r:r['observation'].update({'mode':'guess'}),lambda r:r['input'].update({'sha256':'0'*64})]:
            bad=copy.deepcopy(req);mutate(bad);bp=write(stem+'-bad.json',bad);call(['--request',bp,'--dump-resolved-options'],False)
    # Force physically impossible pose; preserve attempt + native rejection or throw, no implicit hold success.
    bad=copy.deepcopy(canonical);bad['samples'][0]['targets']={'left':{'position':[100,100,100],'rotation':[[1,0,0],[0,1,0],[0,0,1]]}}
    ip=write('unreachable.json',bad);cp=write('negative-config.json',{'urdf_path':canonical['model']['locator'],'solver':'mcc','backend':'eiquadprog'});ep=write('negative-execution.json',{'cold_calls':1,'warmup_calls':0,'measured_calls':0,'state_mode':'snapshot'})
    out=root/'negative';call(['batch','--input',ip,'--config',cp,'--execution',ep,'--output',out],False)
    records=rows(out);assert records[0]['record_type']=='attempt_begin';assert any(x['record_type']=='native_result' and not x['commit_eligible'] for x in records)
    assert not any(x.get('committed') for x in records)
    request={'schema_version':'execution_request.v1','app_id':'mcl_target','execution_structure':'target_solve','input':{'path':str(ip),'sha256':hashlib.sha256(ip.read_bytes()).hexdigest(),'format':'json'},'app_config':json.loads(cp.read_text()),'execution':json.loads(ep.read_text()),'observation':{'mode':'minimal'},'tracking':{},'output_dir':str(root/'negative-minimal')}
    rp=write('negative-minimal-request.json',request);call(['--request',rp],False)
    rejected=[x for x in rows(root/'negative-minimal') if x['record_type']=='native_result']
    assert rejected and all('candidate_q' in x and 'candidate_v' in x and not x['commit_eligible'] for x in rejected)

    print('mcl_target: 9 normal/request/observation pairs, windows, integrity rejection and native failure PASS')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('binary');p.add_argument('canonical',type=pathlib.Path);p.add_argument('--output',type=pathlib.Path);a=p.parse_args()
    if a.output:a.output.mkdir(parents=True,exist_ok=False);run(a.binary,a.canonical,a.output.resolve())
    else:
        with tempfile.TemporaryDirectory() as d:run(a.binary,a.canonical,pathlib.Path(d))
