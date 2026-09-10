#!/usr/bin/env python3
"""Study-local process orchestration; experiment apps own all algorithm semantics."""
from __future__ import annotations
import argparse, datetime, json, os, pathlib, signal, subprocess, sys, time, shutil
from evidence import ROOT, artifact, digest, environment, read_json, runtime_fingerprint, source_fingerprint, stable_hash, validate_definition, verify_artifact, write_json
from verify import verify_unit
from freeze import frozen_prerequisites

def resolve_input(ref,base):
    path=pathlib.Path(ref)
    if not path.is_absolute(): path=base/path
    descriptor=read_json(path)
    if descriptor['schema_version']!='input_descriptor.v1': raise ValueError('input descriptor version')
    if descriptor.get('canonical') is None: raise ValueError(descriptor.get('reason','canonical input unavailable; preparation required'))
    if not verify_artifact(descriptor['canonical']): raise ValueError('canonical hash mismatch or missing')
    value=read_json(descriptor['canonical']['locator'])
    if value.get('model') and not verify_artifact(value['model']): raise ValueError('model hash mismatch or missing')
    return descriptor,value

def executable(unit):
    return os.environ.get('MCL_BINARY',str(pathlib.Path(os.environ.get('MCL_INSTALL_PREFIX','/workspace/install/algorithm'))/'bin'/unit.get('binary','missing')))

def select(definitions,args):
    result=[]
    for path in definitions:
        d=validate_definition(read_json(path))
        if args.experiment and d['experiment_id'] not in args.experiment: continue
        for u in d['units']:
            if args.phase not in u.get('phases',['development','pilot','confirmatory']): continue
            if args.method and u['method_id'] not in args.method and u['arm_id'] not in args.method: continue
            if args.case and u['case_id'] not in args.case: continue
            count=u.get('repeats',d.get('repeats',{'development':3,'pilot':3,'confirmatory':10}))
            count=count[args.phase] if isinstance(count,dict) else count
            repeats=args.repeat if args.repeat is not None else list(range(count))
            for repeat in repeats:
                if repeat<0 or repeat>=count: raise ValueError('repeat outside declared count')
                result.append({'definition_path':str(path),'definition':d,'unit':u,'repeat_id':repeat})
    if args.experiment:
        missing=set(args.experiment)-{x['definition']['experiment_id'] for x in result}
        if missing:raise ValueError('requested experiment has no matching declaration/units: '+','.join(sorted(missing)))
    if not result: raise ValueError('empty selection')
    return result

def resolved_config(unit,phase,smoke=False):
    config=dict(unit['config']);config.update(unit.get('phase_config',{}).get(phase,{}))
    if smoke:config.update(unit.get('smoke_config',{}))
    return config

def preflight(selection,phase,smoke=False):
    env=environment(); rows=[];input_cache={}
    for item in selection:
        u=item['unit']; missing=[]; descriptor=value=None
        if smoke and not u.get('smoke_config'):missing.append('explicit bounded smoke configuration missing')
        try:
            cache_key=(u['input'],str(pathlib.Path(item['definition_path']).parent))
            if cache_key not in input_cache:input_cache[cache_key]=resolve_input(u['input'],pathlib.Path(item['definition_path']).parent)
            descriptor,value=input_cache[cache_key]
        except (OSError,ValueError,KeyError) as e: missing.append('input: '+str(e))
        binary=executable(u) if 'command' not in u else (shutil.which(u['command'][0]) or u['command'][0])
        if not pathlib.Path(binary).is_file(): missing.append('executable missing: '+binary)
        if 'command' in u and 'binary' in u and not pathlib.Path(executable(u)).is_file(): missing.append('native executable missing: '+executable(u))
        if u.get('available') is False: missing.append(u.get('unavailable_reason','declared unavailable'))
        if phase!='development':
            missing.extend(frozen_prerequisites(item['definition'],env))
            resources=u.get('resource_requirements',{})
            ids=u['config'].get('cpu_ids',[])
            if resources.get('explicit_cpu_ids') and len(ids)<resources.get('minimum_cpu_count',1): missing.append('explicit frozen CPU IDs missing/insufficient')
            if ids and not set(ids)<=set(env['affinity']):missing.append('frozen CPU IDs unavailable in effective affinity')
            if not env['realtime_flag']=='1': missing.append('verified RT kernel unavailable')
            if value and value.get('fixture'): missing.append('fixture prohibited for formal evidence')
        if value and value.get('split_id')=='holdout' and phase!='confirmatory': missing.append('holdout access forbidden in development/pilot')
        rows.append({'experiment_id':item['definition']['experiment_id'],'case_id':u['case_id'],'method_id':u['method_id'],
                     'arm_id':u['arm_id'],'repeat_id':item['repeat_id'],'required':u['required'],'missing':missing,
                     'session_id':descriptor.get('session_id','unknown') if descriptor else 'unknown','split_id':descriptor.get('split_id','unknown') if descriptor else 'unknown',
                     'window_id':'all','component_id':'app','config_hash':stable_hash(resolved_config(u,phase,smoke)),
                     'input_descriptor':u['input'],
                     'status':'unavailable' if missing else 'ready','binary':binary,
                     'input_hash':descriptor['canonical']['sha256'] if descriptor else None,
                     'model_hash':value.get('model',{}).get('sha256') if value else None})
    return {'schema_version':'study_preflight.v1','phase':phase,'units':rows,'unit_count':len(rows),
            'smoke':smoke,'ready_count':sum(r['status']=='ready' for r in rows),'status':'failed' if any(r['missing'] and r['required'] for r in rows) else 'passed','environment':env}

def execute(selection,check,args,on_event=None,unit_gate=None):
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    run_id=stamp+'-'+stable_hash([{'unit':x['unit'],'repeat':x['repeat_id']} for x in selection])[:10]
    root=pathlib.Path(args.output_root).resolve()/run_id;root.mkdir(parents=True,exist_ok=False)
    write_json(root/'preflight.json',check)
    source=[source_fingerprint(ROOT),source_fingerprint(ROOT.parents[1]/'components/motion_control_core'),source_fingerprint(ROOT/'third_party/placo')]
    build_root=ROOT.parents[1]/'build/algorithm'
    build_files=[artifact(p) for package in ('motion_control_core','motion_control_lab') for name in ('CMakeCache.txt','compile_commands.json','colcon_command_prefix_build.sh') if (p:=build_root/package/name).is_file()]
    write_json(root/'build_inventory.json',build_files)
    write_json(root/'source_inventory.json',source)
    units=[]; interrupted=False;runtime_cache={}; active_process=None
    def request_stop(signum,frame):
        nonlocal interrupted
        interrupted=True
        if active_process is not None and active_process.poll() is None:
            os.killpg(active_process.pid,signal.SIGINT)
    old_sigint=signal.signal(signal.SIGINT,request_stop)
    old_sigterm=signal.signal(signal.SIGTERM,request_stop)
    def finished(out,state):
        write_json(out/'status.json',state)
        state['artifacts']=[artifact(p) for p in sorted(out.rglob('*')) if p.is_file()]
        if on_event:on_event('finished',state)
    for item,pre in zip(selection,check['units']):
        u=item['unit']; exp=item['definition']['experiment_id']; repeat=item['repeat_id']
        out=root/'units'/exp/u['case_id']/u['arm_id']/str(repeat);out.mkdir(parents=True,exist_ok=False)
        write_json(out/'resolved_declaration.json',{'definition':item['definition'],'unit':u,'resolved_config':resolved_config(u,args.phase,args.smoke),'preflight':pre})
        state={**pre,'run_id':run_id,'status':'not-run','reason':'pending','exit_code':None,'validation':{'status':'unavailable'}};units.append(state)
        if on_event:on_event('started',state)
        if interrupted:
            state['reason']='not started after user interruption';finished(out,state);continue
        if pre['missing']:
            state['status']='unavailable';state['reason']='; '.join(pre['missing']);finished(out,state);continue
        blocked=unit_gate(item) if unit_gate else None
        if blocked:
            state.update(status='unavailable',reason=blocked);finished(out,state);continue
        descriptor,canonical=resolve_input(u['input'],pathlib.Path(item['definition_path']).parent)
        config=resolved_config(u,args.phase,args.smoke)
        if args.smoke:
            config.update(u.get('smoke_config',{}))
            if not u.get('smoke_config'): raise ValueError('smoke requires explicit per-unit smoke_config')
        identity={'run_id':run_id,'experiment_id':exp,'case_id':u['case_id'],'method_id':u['method_id'],'arm_id':u['arm_id'],
                  'repeat_id':repeat,'session_id':descriptor.get('session_id','unknown'),'split_id':descriptor.get('split_id','unknown'),
                  'window_id':'all','component_id':'app','input_hash':pre['input_hash'],'model_hash':pre['model_hash'],'config_hash':stable_hash(config)}
        request={'schema_version':'study_request.v1','identity':identity,'phase':args.phase,'smoke':args.smoke,'config':config,
                 'input':canonical,'input_descriptor':descriptor,'metric_roles':item['definition'].get('metric_roles',{}),'output_dir':str(out)}
        write_json(out/'request.json',request);write_json(out/'resolved_declaration.json',{'definition':item['definition'],'unit':u,'request':request})
        request_path=str(out/'request.json')
        command=[t.replace('{request}',request_path).replace('{output_dir}',str(out)).replace('{input_path}',descriptor['canonical']['locator']).replace('{binary}',executable(u)) for t in u['command']] if 'command' in u else [executable(u),'--request',request_path]
        command[0]=shutil.which(command[0]) or command[0]
        write_json(out/'command.json',{'argv':command,'cwd':str(ROOT),'launcher':'study.py','timeout_s':args.timeout})
        if command[0] not in runtime_cache:runtime_cache[command[0]]=runtime_fingerprint(command[0])
        write_json(out/'environment.json',{'runtime':runtime_cache[command[0]],'native_runtime':runtime_fingerprint(executable(u)) if 'binary' in u and 'command' in u else runtime_cache[command[0]],'platform':check['environment']})
        state.update(identity);state.update(status='running',reason=None);write_json(out/'status.json',state)
        start=time.monotonic_ns()
        with (out/'stdout.log').open('wb') as stdout,(out/'stderr.log').open('wb') as stderr:
            try:
                proc=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=stderr,start_new_session=True)
                active_process=proc
                try: code=proc.wait(timeout=args.timeout)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    code=proc.returncode;state.update(status='interrupted',reason='bounded timeout')
                except KeyboardInterrupt:
                    interrupted=True;os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    code=proc.returncode;state.update(status='interrupted',reason='user interruption')
                state['exit_code']=code
                active_process=None
                if interrupted: state.update(status='interrupted',reason='user interruption')
                if state['status']=='running': state.update(status='completed' if code==0 else ('crashed' if code<0 else 'failed'),reason=None if code==0 else 'native subprocess failure; see stderr.log')
            except OSError as e:state.update(status='unavailable',reason=str(e))
        state['wall_duration_ns']=time.monotonic_ns()-start
        try: state['validation']=verify_unit(out,canonical,identity,config,item['definition'].get('metric_roles',{}))
        except Exception as e:state['validation']={'status':'failed','reason':str(e)} # boundary only; raw/native untouched
        if u.get('validation_command') and not interrupted:
            validator=[t.replace('{request}',request_path).replace('{output_dir}',str(out)).replace('{input_path}',descriptor['canonical']['locator']) for t in u['validation_command']]
            write_json(out/'validation_command.json',{'argv':validator})
            with (out/'validator.stdout.log').open('wb') as stdout,(out/'validator.stderr.log').open('wb') as stderr:
                try:
                    result=subprocess.run(validator,cwd=ROOT,stdout=stdout,stderr=stderr,timeout=args.timeout)
                    validation_code=result.returncode;validation_reason=None
                except (subprocess.TimeoutExpired,OSError) as e:
                    validation_code=None;validation_reason=str(e)
            state['validation']['independent_fk_status']=state['validation']['status']
            if validation_code==0 and state['validation']['status']=='unavailable' and state['validation'].get('result_rows',state['validation'].get('rows',0))>0:state['validation']['status']='passed'
            state['validation']['experiment_validator']={'exit_code':validation_code,'status':'passed' if validation_code==0 else 'failed','reason':validation_reason}
            if validation_code!=0:state['validation']['status']='failed'
        write_json(out/'validation.json',state['validation']);write_json(out/'status.json',state)
        finished(out,state)
    failed=any(u['required'] and (u['status']!='completed' or u['validation']['status']!='passed') for u in units)
    status='interrupted' if interrupted else ('failed' if failed else 'completed')
    for exp in sorted({x['definition']['experiment_id'] for x in selection}):
        own=[u for u in units if u['experiment_id']==exp]
        manifest={'schema_version':'run_manifest.v2','run_kind':'experiment','experiment_id':exp,'run_id':run_id,'phase':args.phase,
                  'smoke':args.smoke,'status':status,'units':own,'definition_hashes':sorted({stable_hash(x['definition']) for x in selection if x['definition']['experiment_id']==exp}),
                  'source_inventory':artifact(root/'source_inventory.json'),'metric_version':'study-metrics.v1','formal_executed':args.phase!='development'}
        write_json(root/(exp+'.manifest.json'),manifest)
    inventory={'schema_version':'evidence_inventory.v1','run_id':run_id,'status':status,'phase':args.phase,'smoke':args.smoke,
      'required_units':check['units'],'actual_units':units,'source_inventory':artifact(root/'source_inventory.json'),'build_inventory':artifact(root/'build_inventory.json'),
      'artifacts':[artifact(p) for p in sorted(root.glob('*.manifest.json'))],
      'identity_fields':list(identity) if 'identity' in locals() else [],'metric_version':'study-metrics.v1',
      'analysis':'deferred','paper':'deferred'}
    write_json(root/'inventory.json',inventory)
    print(json.dumps({'run_dir':str(root),'status':status,'units':len(units)}))
    signal.signal(signal.SIGINT,old_sigint);signal.signal(signal.SIGTERM,old_sigterm)
    return 130 if interrupted else (1 if failed else 0)

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--definition',action='append',type=pathlib.Path)
    p.add_argument('--experiment',action='append');p.add_argument('--method','--arm',action='append');p.add_argument('--case',action='append')
    p.add_argument('--repeat',action='append',type=int);p.add_argument('--phase',choices=['development','pilot','confirmatory'],default='development')
    p.add_argument('--output-root',default=str(ROOT/'runs/mcc_placo_study'));p.add_argument('--dry-run',action='store_true');p.add_argument('--preflight',action='store_true')
    p.add_argument('--smoke',action='store_true');p.add_argument('--timeout',type=float,default=60);p.add_argument('--inventory-output',type=pathlib.Path)
    return p

def main():
    args=parser().parse_args()
    if args.smoke and args.phase!='development':raise ValueError('bounded smoke is development-only; formal campaign requires its frozen full configuration')
    definitions=args.definition or sorted(ROOT.glob('experiments/E*/definition.json'))
    if not args.definition:definitions=[p for p in definitions if p.parent.name[:3] in [f'E{i:02}' for i in range(6,13)]]
    selection=select(definitions,args); check=preflight(selection,args.phase,args.smoke)
    if args.inventory_output:write_json(args.inventory_output,check)
    if args.dry_run or args.preflight:
        print(json.dumps(check,indent=2));return 0 if args.dry_run else (0 if check['status']=='passed' else 1)
    if args.phase!='development' and check['status']!='passed':print(json.dumps(check,indent=2));return 1
    # Process-level lock covers execution and validation, including all child units.
    import fcntl
    lock_path=ROOT/'runs/mcc_placo_study/.execution.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    with lock_path.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return execute(selection,check,args)
if __name__=='__main__':sys.exit(main())
