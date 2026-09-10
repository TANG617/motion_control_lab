#!/usr/bin/env python3
"""Serial non-RT development campaign, with finite sanitized progress records.
No scientific aggregation, candidate search, holdout, or solver overrides.
"""
from __future__ import annotations
import argparse,collections,copy,datetime,fcntl,json,os,pathlib,signal,subprocess,sys,traceback
import study
from evidence import ROOT,artifact,read_json,write_json,verify_artifact,stable_hash,runtime_fingerprint
from check_inventory import check

EXPERIMENTS=[f'E{i:02}' for i in range(6,13)]

def prepare(output):
    root=pathlib.Path(output).resolve();root.mkdir(parents=True,exist_ok=False)
    definitions=[];changes=[]
    for experiment in EXPERIMENTS:
        source=next(ROOT.glob('experiments/'+experiment+'*/definition.json'))
        definition=read_json(source)
        definition['campaign_context']={'phase':'development','realtime_evaluation':False,'full_windows':True,'scientific_analysis':'deferred','candidate_search':'not-requested','production_static':'unchanged'}
        if experiment=='E10':
            for unit in definition['units']:
                if 'development' in unit.get('phases',[]) and unit['case_id'].startswith('fixture-'):continue
                if unit.get('available',True):
                    old=copy.deepcopy(unit['config'])
                    unit['config']['duration_s']=10.
                    unit['config']['windows']={'initialization':[0.,.1],'steady':[.1,10.],'injection':[5.,5.1]}
                    changes.append({'experiment_id':experiment,'case_id':unit['case_id'],'arm_id':unit['arm_id'],'old_duration_s':old['duration_s'],'new_duration_s':10.,'reason':'cover declared 5s injection and recovery, full non-RT development window','solver_configuration_changed':False})
        target=root/'definitions'/f'{experiment}.json';write_json(target,definition)
        definitions.append({'experiment_id':experiment,'source':artifact(source),'snapshot':artifact(target)})
    e12=ROOT/'experiments/E12_recorded_motion_holdout'
    inv=e12/'inputs/inventory-development-20260910/dataset_inventory.json'
    split=e12/'inputs/inventory-development-20260910/session_split.json'
    recipe=e12/'inputs/conversion_recipe.json'
    for name,path in [('dataset_inventory.json',inv),('session_split.json',split),('conversion_recipe.json',recipe)]:write_json(root/'inputs'/name,read_json(path))
    args=study.parser().parse_args(['--phase','development'])
    selected=study.select([pathlib.Path(d['snapshot']['locator']) for d in definitions],args)
    preflight=study.preflight(selected,'development');write_json(root/'preflight-before-conversion.json',preflight)
    models={u['model_hash'] for u in preflight['units'] if u.get('model_hash')}
    model=ROOT.parents[1]/'models/r1.cos.urdf'
    if len(models)!=1 or artifact(model)['sha256'] not in models:raise ValueError('current matrix model identity mismatch')
    records=read_json(inv)['records'];assignments={r['recording_id']:r for r in read_json(split)['records']}
    if any(assignments[r['recording_id']]['split_id']=='holdout' for r in records):raise ValueError('non-RT campaign cannot open holdout')
    runtimes={name:runtime_fingerprint(study.executable({'binary':name})) for name in [f'mcl_study_e{i:02}' for i in range(6,13)]+['mcl_baseline']}
    write_json(root/'runtime-inventory.json',runtimes)
    plan={'schema_version':'nonrt_development_campaign.v1','root':str(root),'definitions':definitions,'config_changes':changes,
          'dataset_inventory':artifact(root/'inputs/dataset_inventory.json'),'session_split':artifact(root/'inputs/session_split.json'),
          'recipe':artifact(root/'inputs/conversion_recipe.json'),'model':artifact(model),'runtime_inventory':artifact(root/'runtime-inventory.json'),
          'conversion_count':len(records),'unit_count':len(selected),'verification_count':len(EXPERIMENTS),'progress_total':len(records)+len(selected)+len(EXPERIMENTS),
          'unit_timeout_s':3600,'conversion_timeout_s':600,'minimum_free_bytes':16*1024**3,
          'repeat_policy':'declared development repeats: E09=3, others=1; no automatic retries',
          'phase':'development','smoke':False,'formal_executed':False,'holdout_access':False,'analysis':'deferred','paper':'deferred',
          'progress_definition':'terminally classified conversion jobs + experiment units (including failed/unavailable) + inventory checks; not percentage of elapsed time or successful solves'}
    write_json(root/'plan.json',plan)
    print(json.dumps({'plan':str(root/'plan.json'),'unit_count':len(selected),'currently_ready':preflight['ready_count'],'progress_total':plan['progress_total']}))
    return plan

class Coordinator:
    def __init__(self,plan):
        self.plan=plan;self.root=pathlib.Path(plan['root']);self.current=0;self.counts=collections.Counter();self.stop=False;self.child=None;self.waves=[];self.conversions=[]
    def progress(self,stage,active=None):
        write_json(self.root/'progress.json',{'current':self.current,'total':self.plan['progress_total'],'stage':stage,'active_unit':active,'terminal_status_counts':dict(self.counts),'definition':self.plan['progress_definition'],'updated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()})
        print(f'PROGRESS: {self.current}/{self.plan["progress_total"]} stage={stage}',flush=True)
    def signal(self,signum,frame):
        self.stop=True
        if self.child is not None and self.child.poll() is None:os.killpg(self.child.pid,signal.SIGINT)
    def process(self,command,output,timeout):
        output.mkdir(parents=True,exist_ok=True);write_json(output/'command.json',{'argv':command,'cwd':str(ROOT),'timeout_s':timeout})
        with (output/'stdout.log').open('w') as stdout,(output/'stderr.log').open('w') as stderr:
            self.child=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=stderr,start_new_session=True)
            try:code=self.child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid,signal.SIGTERM)
                try:self.child.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(self.child.pid,signal.SIGKILL);self.child.wait()
                code=124
            finally:self.child=None
        return code
    def storage_gate(self,item):
        free=__import__('shutil').disk_usage(self.root).free
        if free<self.plan['minimum_free_bytes']:return 'campaign storage guard: below 16 GiB reserve; evidence preserved, no automatic deletion'
        return None
    def event(self,kind,state):
        active={k:state.get(k) for k in ('experiment_id','case_id','arm_id','repeat_id')}
        if kind=='finished':
            self.current+=1;self.counts[state['status']+'/'+state['validation']['status']]+=1
        self.progress(state['experiment_id'],active)
    def convert_inputs(self):
        inv=read_json(self.plan['dataset_inventory']['locator']);splits={r['recording_id']:r for r in read_json(self.plan['session_split']['locator'])['records']}
        definition=read_json(self.root/'definitions/E12.json');converter=ROOT/'experiments/E12_recorded_motion_holdout/data.py'
        for record in inv['records']:
            if self.stop:break
            key=record['recording_id'];out=self.root/'conversion'/key;out.mkdir(parents=True,exist_ok=False);self.progress('prepare')
            canonical=out/'canonical.json';command=[sys.executable,str(converter),'convert','--inventory',self.plan['dataset_inventory']['locator'],'--split',self.plan['session_split']['locator'],'--recording',key,'--recipe',self.plan['recipe']['locator'],'--model',self.plan['model']['locator'],'--output',str(canonical),'--phase','development']
            reason=self.storage_gate(None)
            try:
                if splits[key]['split_id']=='holdout':raise ValueError('holdout denied before raw access')
                code=self.process(command,out,self.plan['conversion_timeout_s']) if not reason else None
                success=code==0
                if success:
                    descriptor=canonical.with_suffix('.descriptor.json');data=read_json(canonical)
                    if data['split_id']=='holdout':raise ValueError('unexpected holdout conversion')
                else:reason=reason or 'canonical conversion failed; original stderr retained; no repair, interpolation or dropped targets'
            except Exception as error:
                success=False;code=None;reason=str(error);(out/'exception.log').write_text(traceback.format_exc())
            for unit in definition['units']:
                if unit['case_id'] not in {key+'-historical',key+'-controlled',key+'-confirmatory'}:continue
                if success:
                    unit['input']=str(descriptor)
                    if unit['config'].get('stratum')!='confirmatory':
                        unit['available']=True;unit.pop('unavailable_reason',None)
                        if unit['method_id']!='production-static':unit['config']['duration_s']=data['samples'][-1]['source_time_s']+unit['config'].get('settling_duration_s',2.)
                else:
                    missing=out/'unavailable.descriptor.json';write_json(missing,{'schema_version':'input_descriptor.v1','input_id':key,'canonical':None,'reason':reason,'raw_source':record['raw'],'session_id':splits[key]['session_id'],'split_id':splits[key]['split_id']})
                    unit['input']=str(missing);unit['available']=False;unit['unavailable_reason']=reason
            status={'recording_id':key,'status':'converted' if success else 'input-invalid','exit_code':code,'reason':reason,'raw_source':record['raw'],'original_raw_untouched':True}
            write_json(out/'status.json',status);self.conversions.append(status);self.current+=1;self.counts[status['status']]+=1;self.progress('prepare')
        resolved=self.root/'definitions/E12.converted.json';write_json(resolved,definition);write_json(self.root/'conversion_inventory.json',{'records':self.conversions,'required_recordings':len(inv['records']),'holdout_access':False})
        return resolved
    def run(self):
        marker=self.root/'execution-started.json'
        with marker.open('x') as f:json.dump({'pid':os.getpid(),'utc':datetime.datetime.now(datetime.timezone.utc).isoformat()},f)
        for key in ('dataset_inventory','session_split','recipe','model','runtime_inventory'):
            if not verify_artifact(self.plan[key]):raise ValueError('campaign preparation artifact changed: '+key)
        for d in self.plan['definitions']:
            if not verify_artifact(d['snapshot']):raise ValueError('definition snapshot changed')
        for record in self.plan.get('orchestrator_artifacts',[]):
            if not verify_artifact(record):raise ValueError('campaign tooling changed after validation')
        for runtime in read_json(self.plan['runtime_inventory']['locator']).values():
            for record in [runtime['executable']]+runtime['libraries']:
                if not verify_artifact(record):raise ValueError('installed runtime changed after preparation')
        signal.signal(signal.SIGINT,self.signal);signal.signal(signal.SIGTERM,self.signal)
        self.progress('prepare');e12=self.convert_inputs()
        resolved_paths=[pathlib.Path(d['snapshot']['locator']) if d['experiment_id']!='E12' else e12 for d in self.plan['definitions']]
        args=study.parser().parse_args(['--phase','development','--timeout',str(self.plan['unit_timeout_s'])]);full=study.select(resolved_paths,args)
        write_json(self.root/'preflight-after-conversion.json',study.preflight(full,'development'))
        for path in resolved_paths:
            if self.stop:break
            experiment=read_json(path)['experiment_id'];args.output_root=str(self.root/'execution'/experiment)
            selection=study.select([path],args);preflight=study.preflight(selection,'development');self.progress(experiment)
            try:
                code=study.execute(selection,preflight,args,on_event=self.event,unit_gate=self.storage_gate)
                inventory=next(pathlib.Path(args.output_root).glob('*/inventory.json'));validation=check(inventory)
                self.current+=1;self.progress('verify');write_json(self.root/'checks'/f'{experiment}.json',validation)
                self.waves.append({'experiment_id':experiment,'exit_code':code,'inventory':artifact(inventory),'integrity':validation})
                if experiment=='E06':
                    cmd=[sys.executable,str(ROOT/'experiments/E06_solver_semantic_parity/admit_pairs.py'),'--definition',str(path),'--evidence-root',str(inventory.parent),'--output',str(self.root/'e06-pair-admission.json')]
                    admission_code=self.process(cmd,self.root/'admission',600)
                    if admission_code:self.counts['admission-failed']+=1
                if code==130:self.stop=True
            except Exception as error:
                failure=self.root/'checks'/f'{experiment}.exception.json';write_json(failure,{'reason':str(error),'traceback':traceback.format_exc(),'required_units':preflight['units'],'execution_directory':args.output_root,'status':'infrastructure-failed; any existing partial output retained'})
                self.waves.append({'experiment_id':experiment,'exit_code':1,'failure':artifact(failure)});self.counts['infrastructure-failed']+=1
            finally:
                signal.signal(signal.SIGINT,self.signal);signal.signal(signal.SIGTERM,self.signal)
            write_json(self.root/'campaign_status.json',{'state':'running','waves':self.waves,'terminal_status_counts':dict(self.counts),'formal_executed':False})
        failed=any(r['status']!='converted' for r in self.conversions) or any(r['exit_code']!=0 or r.get('integrity',{}).get('status')!='passed' for r in self.waves) or bool(self.counts['admission-failed'])
        state='interrupted' if self.stop else 'finished-with-failures' if failed else 'completed'
        write_json(self.root/'campaign_status.json',{'state':state,'waves':self.waves,'terminal_status_counts':dict(self.counts),'processed':self.current,'total':self.plan['progress_total'],'all_experiment_waves_processed':len(self.waves)==7,'conversion_count':len(self.conversions),'formal_executed':False,'analysis':'deferred','paper':'deferred','remaining_experiments':[e for e in EXPERIMENTS if e not in [w['experiment_id'] for w in self.waves]]})
        print('Campaign finished: '+state,flush=True)
        return 130 if self.stop else 1 if failed else 0

def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('prepare');p.add_argument('--output',required=True)
    p=sub.add_parser('run');p.add_argument('--plan',required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.output);return 0
    plan=read_json(args.plan);lock=ROOT/'runs/mcc_placo_study/.execution.lock';lock.parent.mkdir(parents=True,exist_ok=True)
    with lock.open('a') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:return Coordinator(plan).run()
        except Exception as error:
            write_json(pathlib.Path(plan['root'])/'campaign-fatal.json',{'state':'infrastructure-failed','reason':str(error),'traceback':traceback.format_exc(),'formal_executed':False,'partial_evidence_preserved':True})
            raise
if __name__=='__main__':raise SystemExit(main())
