#!/usr/bin/env python3
"""Independent small-matrix references and public entry parity; no robot apps."""
import argparse, hashlib, json, pathlib, subprocess, tempfile

def run(binary, root):
    def write(name,value):
        p=root/name;p.write_text(json.dumps(value));return p
    counter=0
    def invoke(args,ok=True):
        nonlocal counter
        counter+=1
        r=subprocess.run([binary,*map(str,args)],capture_output=True,text=True,timeout=30)
        (root/f'command-{counter:03}.json').write_text(json.dumps({'argv':[binary,*map(str,args)],'exit_code':r.returncode}))
        (root/f'command-{counter:03}.stdout').write_text(r.stdout)
        (root/f'command-{counter:03}.stderr').write_text(r.stderr)
        assert (r.returncode==0)==ok,(r.returncode,r.stdout,r.stderr,args)
        return r
    assert json.loads(invoke(['--describe-capabilities']).stdout)['app_id']=='mcl_optimization_problem'
    cases=[
        ('multi',{'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,1]],'b':[1]}]},lambda x:abs(sum(x)-1)<1e-6),
        ('degenerate',{'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,1],[2,2]],'b':[1,2]}]},lambda x:abs(sum(x)-1)<1e-6),
        ('two',{'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,1]],'b':[1]},{'A':[[1,0]],'b':[.25]}]},lambda x:abs(x[0]-.25)<1e-6 and abs(sum(x)-1)<1e-6),
        ('three',{'lower':[-2,-2,-2],'upper':[2,2,2],'tasks':[{'A':[[1,1,1]],'b':[1]},{'A':[[1,0,0]],'b':[.25]},{'A':[[0,1,0]],'b':[.5]}]},lambda x:max(abs(a-b) for a,b in zip(x,[.25,.5,.25]))<1e-6),
        ('hard_infeasible',{'lower':[-1,-1],'upper':[1,1],'tasks':[{'A':[[1,1]],'b':[3],'hard':True}]},None),
    ]
    for backend in ['eiquadprog','proxqp']:
        config={'solver':'mcc','mode':'hqp','backend':backend,'regularization':1e-8,'preservation_tolerance':1e-7}
        cp=write(backend+'.json',config)
        for name,problem,check in cases:
            stem=backend+'-'+name;ip=write(stem+'.json',problem);normal=root/(stem+'-normal');request_output=root/(stem+'-request')
            invoke(['--input',ip,'--config',cp,'--output',normal],check is not None)
            request={'schema_version':'execution_request.v1','app_id':'mcl_optimization_problem','execution_structure':'explicit_matrix','input':{'path':str(ip),'sha256':hashlib.sha256(ip.read_bytes()).hexdigest(),'format':'json'},'app_config':config,'execution':{},'observation':{},'tracking':{'uninterpreted':'E-not-a-selector'},'output_dir':str(request_output)}
            rp=write(stem+'-request.json',request)
            invoke(['--request',rp,'--dump-resolved-options']);assert not request_output.exists()
            invoke(['--request',rp],check is not None)
            a=json.loads((normal/'result.json').read_text());b=json.loads((request_output/'result.json').read_text())
            assert a['accepted']==b['accepted'];assert a['candidate']==b['candidate']
            if check:assert check(a['candidate']),(stem,a)
            else:assert not a['accepted']
            invoke(['--request',rp],False)
            invoke(['--request',rp,'--backend','proxqp'],False)
    weighted={'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,0]],'b':[0],'weight':1},{'A':[[1,0]],'b':[1],'weight':3},{'A':[[0,1]],'b':[.2],'hard':True}]}
    ip=write('weighted-input.json',weighted)
    for solver,backend in [('mcc','eiquadprog'),('mcc','proxqp'),('placo','eiquadprog')]:
        stem='weighted-'+solver+'-'+backend
        config={'solver':solver,'mode':'weighted','backend':backend,'regularization':1e-8}
        cp=write(stem+'-config.json',config);out=root/stem
        invoke(['--input',ip,'--config',cp,'--output',out])
        result=json.loads((out/'result.json').read_text());x=result['candidate'];assert abs(x[0]-.75)<1e-6 and abs(x[1]-.2)<1e-6,(stem,result)
    print('optimization_problem: 10 hierarchy native/reference + 10 normal/request pairs + 3 weighted references PASS')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('binary');p.add_argument('--output',type=pathlib.Path);a=p.parse_args()
    if a.output:a.output.mkdir(parents=True,exist_ok=False);run(a.binary,a.output.resolve())
    else:
        with tempfile.TemporaryDirectory() as d:run(a.binary,pathlib.Path(d))
