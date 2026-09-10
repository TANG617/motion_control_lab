#!/usr/bin/env python3
"""Equal-budget search orchestration and immutable selected-candidate configuration.
This prepares/executes explicit development units only; it does not tune implicitly.
"""
import argparse,json,pathlib,subprocess,sys,math
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from evidence import artifact,stable_hash,write_json,verify_artifact

def search_plan(candidates,recordings,budget,repeats=1,policy=None):
 if budget<=0 or repeats<=0 or not candidates or not recordings:raise ValueError('nonempty candidates/recordings and positive equal budget required')
 if any(r['split_id'] not in ('development','tuning') for r in recordings):raise ValueError('holdout is never a search input')
 if len({c['candidate_id'] for c in candidates})!=len(candidates):raise ValueError('duplicate candidate identity')
 methods={c['method_id'] for c in candidates};counts={m:sum(c['method_id']==m for c in candidates) for m in methods}
 if any(n>30 for n in counts.values()):raise ValueError('maximum 30 preregistered candidates per method; new exploration version required')
 if len(set(counts.values()))!=1:raise ValueError('unequal candidate search counts across methods')
 trials=[]
 for candidate in candidates:
  for index in range(budget):
   recording=recordings[index%len(recordings)]
   for repeat in range(repeats):trials.append(dict(trial_id=f"{candidate['candidate_id']}-{index}-{repeat}",candidate_id=candidate['candidate_id'],method_id=candidate['method_id'],config=candidate['config'],config_hash=stable_hash(candidate['config']),recording=recording,repeat_id=repeat,status='not-run'))
 return dict(schema_version='candidate_search_plan.v1',candidate_count_by_method=counts,budget_per_candidate=budget,repeats=repeats,trials=trials,preregistration=policy,selection_policy='guardrail then full-quality coverage then predeclared score and guardrails; failed and missing retain status; no automatic retries',plan_hash=stable_hash(trials))

def measure_trial(directory,trial,policy):
 """Read actual independently checked nested unit evidence outside timers."""
 raws=list((pathlib.Path(directory)/'runs').glob('**/raw.jsonl'))
 if len(raws)!=1:return dict(guardrails_passed=None,measurement_status='missing-or-ambiguous-unit',selection_metrics={},full_quality_coverage=None)
 unit=raws[0].parent
 raw=[json.loads(line) for line in raws[0].read_text().splitlines() if json.loads(line).get('record_type')=='attempt']
 metrics_path=unit/'metrics.jsonl';validation_path=unit/'validation.json';checks_path=unit/'stage_checks.jsonl'
 if not metrics_path.exists() or not validation_path.exists() or not checks_path.exists():return dict(guardrails_passed=None,measurement_status='independent-evidence-missing',selection_metrics={},full_quality_coverage=None)
 metrics=[json.loads(line) for line in metrics_path.read_text().splitlines()];validation=json.loads(validation_path.read_text());checks=[json.loads(line) for line in checks_path.read_text().splitlines()]
 values={name:[r['value'] for r in metrics if r['metric_id']==name and r['status']=='ok' and r['window_id']=='source-full'] for name in ('position_error','orientation_error')}
 measured={name:sum(v)/len(v) if v else None for name,v in values.items()}
 expected=math.ceil(trial['config']['duration_s']/trial['config']['dt_s']);full=sum(bool(r.get('ik_committed')) and r.get('completed_passes')==r.get('requested_passes') for r in raw)/expected if expected else None
 descriptor=json.loads(pathlib.Path(trial['recording']['descriptor']).read_text());canonical=json.loads(pathlib.Path(descriptor['canonical']['locator']).read_text());source_coverage=len({r['input_sequence'] for r in raw})/len(canonical['samples']);holds=sum(r['execution_state']=='HOLD' for r in raw)
 violation=max([c['executed_position_violation_rad'] for c in checks]+[c['raw_ik_position_violation_rad'] for c in checks if c['raw_ik_position_violation_rad'] is not None],default=None)
 required=('maximum_position_violation_rad','minimum_source_coverage','maximum_execution_holds')
 guardrails=None
 if policy and all(k in policy for k in required):guardrails=validation['status']=='passed' and violation is not None and violation<=policy['maximum_position_violation_rad'] and source_coverage>=policy['minimum_source_coverage'] and holds<=policy['maximum_execution_holds'] and len(raw)==expected
 return dict(guardrails_passed=guardrails,measurement_status='measured' if guardrails is not None else 'measured-preregistered-guardrails-missing',selection_metrics=measured,full_quality_coverage=full,source_coverage=source_coverage,execution_holds=holds,maximum_position_violation_rad=violation,evidence=[artifact(x) for x in [raws[0],metrics_path,validation_path,checks_path]],reduction='per-trial source-full arithmetic mean over both arms; selection only, no scientific cross-run result')

def execute(plan,output):
 if plan['plan_hash']!=stable_hash(plan['trials']):raise ValueError('registered search plan changed')
 output=pathlib.Path(output);output.mkdir(parents=True,exist_ok=False);statuses=[]
 try:
  for trial in plan['trials']:
   row=dict(trial);command=trial.get('command')
   if command is None and trial['recording'].get('descriptor'):
    directory=output/trial['trial_id'];directory.mkdir()
    definition={'schema_version':'experiment.v2','experiment_id':'E12','question':'Explicit equal-budget development selection','units':[{'case_id':trial['trial_id'],'method_id':trial['method_id'],'arm_id':trial['candidate_id'],'input':trial['recording']['descriptor'],'config':trial['config'],'required':True,'binary':'mcl_study_e12','validation_command':['python3',str(pathlib.Path(__file__).with_name('verify.py')),'--request','{request}']}],'failure_policy':'continue','metrics':[],'evaluation_windows':['source-full'],'controlled_factors':['candidate','recording','repeat']}
    write_json(directory/'definition.json',definition)
    command=[sys.executable,str(ROOT/'tools/mcc_placo_study/study.py'),'--definition',str(directory/'definition.json'),'--phase','development','--repeat','0','--timeout','120','--output-root',str(directory/'runs')]
   if not command:row.update(status='unavailable',reason='explicit frozen per-trial command required')
   else:
    directory=output/trial['trial_id'];directory.mkdir(exist_ok=True);write_json(directory/'command.json',command)
    with (directory/'stdout.log').open('w') as out,(directory/'stderr.log').open('w') as err:
     proc=subprocess.run(command,stdout=out,stderr=err);row.update(status='completed' if proc.returncode==0 else 'failed',exit_code=proc.returncode)
   if command and trial['recording'].get('descriptor'):row.update(measure_trial(directory,trial,plan.get('preregistration')))
   statuses.append(row);write_json(output/'search_status.json',statuses)
 except KeyboardInterrupt:
  attempted={r['trial_id'] for r in statuses};statuses.extend(dict(t,status='interrupted' if t['trial_id']==trial['trial_id'] else 'not-run') for t in plan['trials'] if t['trial_id'] not in attempted);write_json(output/'search_status.json',statuses);raise
 return statuses

def select(plan, results, policy):
 """Apply a predeclared scalar search objective, retaining all trial statuses."""
 if plan.get('preregistration') and plan['preregistration']!=policy:raise ValueError('selection policy changed after search registration')
 if policy.get('direction') not in ('minimize','maximize') or not policy.get('metric'):
  raise ValueError('explicit metric and direction required')
 if {r['trial_id'] for r in results}!={t['trial_id'] for t in plan['trials']}:
  raise ValueError('required search trials missing')
 byid={r['trial_id']:r for r in results};ranked={}
 for candidate_id in sorted({t['candidate_id'] for t in plan['trials']}):
  trials=[t for t in plan['trials'] if t['candidate_id']==candidate_id];rows=[byid[t['trial_id']] for t in trials]
  eligible=all(r['status']=='completed' and r.get('guardrails_passed') is True and isinstance(r.get('full_quality_coverage'),(int,float)) and isinstance(r.get('selection_metrics',{}).get(policy['metric']),(int,float)) for r in rows)
  score=sum(r['selection_metrics'][policy['metric']] for r in rows)/len(rows) if eligible else None
  ranked.setdefault(trials[0]['method_id'],[]).append(dict(candidate_id=candidate_id,config=trials[0]['config'],score=score,full_quality_coverage=sum(r['full_quality_coverage'] for r in rows)/len(rows) if eligible else None,eligible=eligible,trials=len(rows)))
 selected=[]
 for method,rows in ranked.items():
  choices=[r for r in rows if r['eligible']]
  if not choices:continue
  chosen=sorted(choices,key=lambda r:(-r['full_quality_coverage'],(r['score'] if policy['direction']=='minimize' else -r['score']),r['candidate_id']))[0]
  selected.append(dict(method_id=method,**chosen))
 return dict(schema_version='candidate_selection.v1',status='selected' if len(selected)==len(ranked) else 'no-eligible-candidate',missing_methods=sorted(set(ranked)-{s['method_id'] for s in selected}),policy=policy,selected=selected,all_candidates=ranked,search_plan_hash=plan['plan_hash'])

def freeze(selected,plan_artifact,result_artifact,preregistration,out):
 for item in (plan_artifact,result_artifact,preregistration):
  if not verify_artifact(item):raise ValueError('freeze source hash mismatch')
 plan=json.loads(pathlib.Path(plan_artifact['locator']).read_text());results=json.loads(pathlib.Path(result_artifact['locator']).read_text())
 if set(r['trial_id'] for r in results)!=set(r['trial_id'] for r in plan['trials']):raise ValueError('incomplete required trial status inventory')
 if any(r['status'] in ('not-run','running','interrupted','unavailable') for r in results):raise ValueError('search budget not completed; cannot freeze selection')
 byid={t['candidate_id']:t for t in plan['trials']}
 if any(s['candidate_id'] not in byid or stable_hash(s['config'])!=byid[s['candidate_id']]['config_hash'] for s in selected):raise ValueError('selected config differs from searched candidate')
 registered=json.loads(pathlib.Path(preregistration['locator']).read_text())
 actual=select(plan,results,registered)
 if actual['status']!='selected':raise ValueError('no eligible candidate for every method; cannot freeze')
 if {s['candidate_id'] for s in selected}!={s['candidate_id'] for s in actual['selected']}:raise ValueError('selection disagrees with preregistered ordering')
 for result in results:
  for ref in result.get('evidence',[]):
   if not verify_artifact(ref):raise ValueError('search evidence changed before freeze')
 if pathlib.Path(out).exists():raise FileExistsError('candidate freeze immutable')
 value=dict(schema_version='candidate_freeze.v1',selected=selected,search_plan=plan_artifact,search_status=result_artifact,preregistration=preregistration,holdout_access='not-authorized-by-candidate-freeze',candidate_hash=stable_hash(selected));write_json(out,value);return value
if __name__=='__main__':
 p=argparse.ArgumentParser();s=p.add_subparsers(dest='operation',required=True);q=s.add_parser('plan');q.add_argument('--candidates',required=True);q.add_argument('--recordings',required=True);q.add_argument('--budget',type=int,required=True);q.add_argument('--repeats',type=int,default=1);q.add_argument('--policy',required=True);q.add_argument('--output',required=True);e=s.add_parser('execute');e.add_argument('--plan',required=True);e.add_argument('--output',required=True);sel=s.add_parser('select');sel.add_argument('--plan',required=True);sel.add_argument('--results',required=True);sel.add_argument('--policy',required=True);sel.add_argument('--output',required=True);f=s.add_parser('freeze');f.add_argument('--selected',required=True);f.add_argument('--plan',required=True);f.add_argument('--results',required=True);f.add_argument('--preregistration',required=True);f.add_argument('--output',required=True);a=p.parse_args();read=lambda p:json.loads(pathlib.Path(p).read_text())
 if a.operation=='plan':
  if pathlib.Path(a.output).exists():raise FileExistsError(a.output)
  write_json(a.output,search_plan(read(a.candidates),read(a.recordings),a.budget,a.repeats,read(a.policy)))
 elif a.operation=='execute':sys.exit(0 if all(t['status']=='completed' for t in execute(read(a.plan),a.output)) else 1)
 elif a.operation=='select':
  if pathlib.Path(a.output).exists():raise FileExistsError(a.output)
  write_json(a.output,select(read(a.plan),read(a.results),read(a.policy)))
 else:freeze((read(a.selected).get('selected') if isinstance(read(a.selected),dict) else read(a.selected)),artifact(a.plan),artifact(a.results),artifact(a.preregistration),a.output)
