#!/usr/bin/env python3
"""Serial public app execution; dry-run never starts a solver or mutates sources."""
from __future__ import annotations
import argparse,collections,copy,datetime,json,os,pathlib,subprocess,sys,tempfile,time
from contracts import *

def bounded_process(command,cwd,stdout,stderr,timeout):
    """Bound the owned process group, including baseline's mechanical adapter child."""
    import signal
    process=subprocess.Popen(command,cwd=cwd,stdout=stdout,stderr=stderr,start_new_session=True)
    try:return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.wait()
        return 124

def select(paths,args):
    result=[];seen=set()
    for path in paths:
        d=validate_definition(read(path))
        if args.experiment and d['experiment_id'] not in args.experiment:continue
        for u in d['units']:
            if args.case and u['case_id'] not in args.case:continue
            if args.app and u['app'] not in args.app:continue
            n=u.get('repeats',d.get('repeats',1));n=n.get('development',1) if isinstance(n,dict) else n
            reps=range(n) if args.repeat is None else args.repeat
            for rep in reps:
                if not 0<=rep<n:raise ValueError('repeat outside declaration')
                identity=(d['experiment_id'],u['case_id'],u['arm_id'],rep)
                if identity in seen:raise ValueError('duplicate selected execution identity: '+repr(identity))
                seen.add(identity)
                result.append((path,d,u,rep))
    if not result:raise ValueError('empty declaration selection')
    return result

def request_for(d,u,repeat,out,smoke=False):
    ref,data,descriptor=descriptor_input(u['input'])
    cfg=copy.deepcopy(u['config']);execution=copy.deepcopy(u['execution'])
    if smoke:
        if u['app'] in ('mcl_step','mcl_target'):execution.update(cold_calls=1,warmup_calls=1,measured_calls=1)
        elif u['app']=='mcl_hierarchical_kinematics_step':
            if u['execution_structure']!='replay':execution.update(planned_releases=u['smoke_config']['maximum_calls'],warmup_calls=1)
            else:cfg['options']['duration']=.03
        elif u['app']=='mcl_planned_kinematics_step':cfg['duration_s']=min(cfg['duration_s'],.03)
    tracking={'experiment_id':d['experiment_id'],'case_id':u['case_id'],'method_id':u['method_id'],'arm_id':u['arm_id'],'repeat_id':repeat,'smoke':smoke}
    if 'legacy_definition' in d:
        tracking['legacy_definition_sha256']=d['legacy_definition']['sha256']
    request=dict(schema_version='execution_request.v1',app_id=u['app'],execution_structure=u['execution_structure'],input=ref,
                 app_config=cfg,execution=execution,observation=u['observation'],output_dir=str(out),tracking=tracking)
    return validate_request(request),data,descriptor

def preflight(selection,prefix,smoke=False,probe=True):
    rows=[];caps={};cache={};input_cache={};definitions={}
    for path,d,_,_ in selection:
        if str(path) not in definitions:definitions[str(path)]={'source':artifact(path),'resolved_sha256':stable_hash(d)}
    for position,(path,d,u,rep) in enumerate(selection):
        if position%100==0:print(f'preflight {position}/{len(selection)}',file=sys.stderr,flush=True)
        configuration_error='migration_defect' if 'migration' in u else 'configuration_invalid'
        key=(d['experiment_id'],u['case_id'],u['arm_id'],rep)
        row={'experiment_id':key[0],'case_id':key[1],'arm_id':key[2],'repeat_id':rep,'method_id':u['method_id'],'app':u['app'],'execution_structure':u['execution_structure'],'status':'ready','reasons':[],'required':u['required']}
        if 'migration' in u:
            row['migration_status']=u['migration']['status']
            if u['migration']['status']!='mapped':row.update(status=u['migration']['status'],reasons=[u['migration']['reason']])
        if u.get('available') is False and row['status']=='ready':
            row.update(status='declared_unavailable',reasons=[u.get('unavailable_reason','declaration marks this unit unavailable')])
        binary=prefix/'bin'/u['app'];row['binary']=str(binary)
        if not binary.is_file():row.update(status='environment_missing');row['reasons'].append('executable missing: '+str(binary))
        try:
            request,data,descriptor=request_for(d,u,rep,pathlib.Path('/tmp/mcl-readonly-preflight-output'),smoke)
            row['input']=request['input'];row['config_hash']=stable_hash(request['app_config']);row['fixture']=data.get('fixture',False)
            if descriptor.get('split_id')=='holdout':row.update(status='source_ineligible');row['reasons'].append('holdout not authorized in development')
        except (ValueError,KeyError,OSError) as e:
            row['source_status']='missing_or_invalid';row['reasons'].append(str(e))
            if row['status']=='ready':row['status']='source_missing'
            rows.append(row);continue
        if row['status']!='ready':rows.append(row);continue
        if u['app']=='mcl_hierarchical_kinematics_step' and data.get('model'):
            model_path=data['model']['locator']
            if model_path not in input_cache:input_cache[model_path]=collision_resources(model_path)
            row['collision_resources']=input_cache[model_path]
            if row['collision_resources']['status']!='passed':
                row['reasons'].append('declared model collision resources unavailable to the native loader; no model replacement or collision bypass')
        schedule=request['execution'].get('schedule','sync')
        if request['execution'].get('timing_mode')=='real' and schedule in ('async_same_core','async_dual_core'):
            e=request['execution'];a=e.get('red_cpu');b=e.get('yellow_cpu')
            if a is None or b is None:
                row.update(status='resource_unfrozen');row['reasons'].append('real asynchronous comparison requires explicitly frozen CPU identities; no automatic CPU selection')
            elif a not in os.sched_getaffinity(0) or b not in os.sched_getaffinity(0):
                row.update(status='resource_unavailable');row['reasons'].append('declared CPU not in effective affinity')
            elif (schedule=='async_same_core') != (a==b):
                row.update(status=configuration_error);row['reasons'].append('declared CPUs do not match scheduling topology')
        if row['status']!='ready':rows.append(row);continue
        if u['app']=='mcl_baseline':
            row['capability_policy']='frozen public replay; no algorithm configuration overrides'
            rows.append(row);continue
        if u['app'] not in caps:caps[u['app']]=capabilities(binary)
        binding=u.get('capability_binding',{})
        if binding.get('sha256') and binding['sha256']!=caps[u['app']]['sha256']:raise ValueError('frozen app capabilities changed: '+u['app'])
        row['capabilities_sha256']=caps[u['app']]['sha256']
        if request['execution_structure'] not in caps[u['app']]['document']['execution_structures']:
            row.update(status=configuration_error);row['reasons'].append('declared execution structure absent from actual app capabilities')
        elif probe:
            probe_key=stable_hash({k:v for k,v in request.items() if k not in ('tracking','output_dir')})
            if probe_key not in cache:
                with tempfile.TemporaryDirectory(prefix='mcl-readonly-config-') as t:
                    p=pathlib.Path(t)/'request.json';write(p,request)
                    proc=subprocess.run([str(binary),'--request',str(p),'--dump-resolved-options'],capture_output=True,text=True,timeout=15)
                    cache[probe_key]={'exit_code':proc.returncode,'stderr':proc.stderr,'resolved':json.loads(proc.stdout) if proc.returncode==0 else None}
            resolved=cache[probe_key]
            if resolved['exit_code']:
                row.update(status=configuration_error);row['reasons'].append(resolved['stderr'].strip())
            else:row['resolved_options_hash']=stable_hash(resolved['resolved'])
        if row['status']=='ready' and row.get('collision_resources',{}).get('status')=='resource_missing':row['status']='resource_missing'
        rows.append(row)
    for path,record in definitions.items():
        if digest(path)!=record['source']['sha256'] or stable_hash(read(path))!=record['resolved_sha256']:raise ValueError('declaration changed during preflight: '+path)
    count=collections.Counter(r['status'] for r in rows)
    return {'schema_version':'app_matrix_preflight.v1','unit_count':len(rows),'counts':dict(count),'status':'failed' if count['migration_defect'] or count['configuration_invalid'] or count['environment_missing'] else 'passed','smoke':smoke,'formal_rt':False,'units':rows,'capabilities':caps,'config_probe_count':len(cache),'definitions':definitions}

def execute(selection,check,args):
    overall_failed=False
    # One process at a time. Failed or unavailable sources retain their declaration row.
    for experiment in sorted({d['experiment_id'] for _,d,_,_ in selection}):
        pairs=[(x,row) for x,row in zip(selection,check['units']) if x[1]['experiment_id']==experiment]
        directory=next(ROOT.glob('experiments/'+experiment+'_*'))/'runs'
        stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        out=directory/(stamp+'-'+stable_hash([x[0][2] for x in pairs])[:10]);out.mkdir(parents=True,exist_ok=False)
        write(out/'preflight.json',check);write(out/'declaration.json',pairs[0][0][1]);write(out/'source_inventory.json',{'declaration':artifact(pairs[0][0][0]),'capabilities':check['capabilities'],'runtimes':{name:runtime_inventory(args.install_prefix/'bin'/name) for name in sorted({x[0][2]['app'] for x in pairs}) if (args.install_prefix/'bin'/name).is_file()}})
        states=[]
        for (path,d,u,rep),pre in pairs:
            state=copy.deepcopy(pre);states.append(state)
            if pre['status']!='ready':
                state['preflight_status']=pre['status'];state['status']='unavailable';continue
            target=out/'units'/u['case_id']/u['arm_id']/str(rep);target.mkdir(parents=True)
            request,_,_=request_for(d,u,rep,target/'app',args.smoke);write(target/'request.json',request)
            command=[pre['binary'],'--request',str(target/'request.json')]
            if u['app']=='mcl_baseline':command=[sys.executable,str(ROOT/'tools/app_execution/baseline.py'),'--request',str(target/'request.json'),'--binary',pre['binary']]
            write(target/'command.json',{'argv':command,'timeout_s':args.timeout,'cwd':str(ROOT),'serial_native':True})
            began=time.monotonic()
            with (target/'stdout.log').open('w') as stdout,(target/'stderr.log').open('w') as stderr:
                code=bounded_process(command,ROOT,stdout,stderr,min(args.timeout,30) if args.smoke else args.timeout)
            state.update(exit_code=code,status='completed' if code==0 else 'failed',elapsed_s=time.monotonic()-began,evidence=str(target))
            from verify import verify_output
            try:validation=verify_output(target/'app',request,code)
            except Exception as error:
                validation={'status':'failed','native_exit_code':code,'native_status_rewritten':False,'issues':['independent verifier exception: '+repr(error)]}
            write(target/'validation.json',validation);state['validation']=validation['status']
            state['artifacts']=[{'locator':a['path'],'sha256':a['sha256'],'size_bytes':a['size_bytes']} for p in sorted(target.rglob('*')) if p.is_file() for a in [artifact(p)]]
            print(json.dumps({k:state[k] for k in ('experiment_id','case_id','app','status','exit_code','validation')}),flush=True)
        source=artifact(out/'source_inventory.json')
        manifest={'schema_version':'run_manifest.v2','run_id':out.name,'run_kind':'experiment','experiment_id':experiment,'status':'failed' if any(s['status']=='failed' or s.get('validation')=='failed' for s in states) else 'completed','execution_contract':'execution_request.v1','units':states,'source_inventory':{'locator':source['path'],'sha256':source['sha256'],'size_bytes':source['size_bytes']},'metric_version':'app-execution-evidence.v1','formal_rt':False,'smoke':args.smoke}
        write(out/(experiment+'.manifest.json'),manifest);print(json.dumps({'run':str(out),'status':manifest['status']}),flush=True)
        overall_failed |= manifest['status']=='failed'
    return int(overall_failed)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--definition',action='append',type=pathlib.Path);p.add_argument('--install-prefix',type=pathlib.Path,default=pathlib.Path(os.environ.get('MCL_INSTALL_PREFIX','/workspace/install/algorithm')))
    p.add_argument('--experiment',action='append');p.add_argument('--case',action='append');p.add_argument('--app',action='append');p.add_argument('--repeat',type=int,action='append');p.add_argument('--smoke',action='store_true');p.add_argument('--timeout',type=float,default=30)
    mode=p.add_mutually_exclusive_group(required=True);mode.add_argument('--dry-run',action='store_true');mode.add_argument('--execute',action='store_true');a=p.parse_args()
    paths=a.definition or sorted(ROOT.glob('experiments/E*/definition.json'));paths=[p for p in paths if read(p).get('execution_contract')=='execution_request.v1']
    selection=select(paths,a);check=preflight(selection,a.install_prefix,a.smoke)
    if a.dry_run:print(json.dumps(check,indent=2));return int(check['status']=='failed')
    if check['status']=='failed':raise ValueError('matrix has invalid configuration or missing runtime; fix before execution')
    return execute(selection,check,a)
if __name__=='__main__':raise SystemExit(main())
