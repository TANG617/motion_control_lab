#!/usr/bin/env python3
"""Hash-bound E06 semantic pairing admission. No effects, ranking or statistics."""
import argparse,collections,importlib.util,json,pathlib,sys
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from evidence import artifact,stable_hash,write_json,verify_artifact
spec=importlib.util.spec_from_file_location('e06_audit',HERE/'verify.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
PAIR_KEYS=['experiment_id','case_id','repeat_id','session_id','split_id','window_id','component_id','input_hash','model_hash']
def admit(left,right):
 reasons=[];fields=[];artifacts=[]
 if left is None or right is None:return {'status':'unavailable','reasons':['missing baseline unit' if left is None else 'missing candidate unit']}
 for key in PAIR_KEYS:
  a=left['request']['identity'].get(key);b=right['request']['identity'].get(key)
  if a!=b:reasons.append('pair identity mismatch: '+key)
 for label,unit in [('baseline',left),('candidate',right)]:
  directory=unit['directory']
  if unit['status'].get('status')!='completed':reasons.append(label+' native unit '+str(unit['status'].get('status')))
  validation=unit.get('validation',{})
  if validation.get('status') not in ('passed','completed'):reasons.append(label+' independent validation '+str(validation.get('status','missing')))
  for name in ['request.json','status.json','method_semantics.json','model_mapping.json','raw.jsonl','validation.json']:
   path=directory/name
   if path.is_file():artifacts.append(artifact(path))
   elif name in ['method_semantics.json','model_mapping.json','raw.jsonl']:reasons.append(label+' missing '+name)
 l={**left['request']['config'],**left.get('mapping',{})};r={**right['request']['config'],**right.get('mapping',{})};fields=audit.semantic_audit(l,r)
 reasons += ['semantic mismatch: '+x['field'] for x in fields if x['status']!='pass']
 lm=left['request']['identity']['method_id'];rm=right['request']['identity']['method_id']
 if 'production-static' in (lm,rm):reasons.append('production-static keeps historical task, acceptance, termination and state policies; not controlled parity')
 if l.get('mode')=='TargetSolve' and lm!=rm:reasons.append('TargetSolve native convergence/budget/iteration acceptance differ; separate descriptive outcomes')
 if l.get('enforcement')=='scaled' and ('placo' in lm or 'placo' in rm):reasons.append('PlaCo shared feasible-reference scale equation unavailable')
 # Same physical/task model may pass while native acceptance differs. Retain that precise boundary.
 if ('placo' in lm)!=('placo' in rm):reasons.append('native acceptance mismatch: PlaCo throw-on-failure versus MCC explicit post-solve hard gate; causal admission requires frozen reviewed common acceptance contract')
 return {'status':'admitted' if not reasons else 'unavailable','reasons':reasons,'field_checks':fields,'source_artifacts':artifacts,'interpretation':'admission only; no effect or superiority estimate'}
def collect(roots):
 units=[]
 for root in roots:
  for p in pathlib.Path(root).rglob('request.json'):
   request=json.loads(p.read_text())
   if request.get('identity',{}).get('experiment_id')!='E06':continue
   u={'directory':p.parent,'request':request}
   for key,name in [('status','status.json'),('semantics','method_semantics.json'),('mapping','model_mapping.json'),('validation','validation.json')]:u[key]=json.loads((p.parent/name).read_text()) if (p.parent/name).exists() else {}
   units.append(u)
 return units

def main():
 p=argparse.ArgumentParser();p.add_argument('--definition',default=str(HERE/'definition.json'));p.add_argument('--evidence-root',action='append',required=True);p.add_argument('--output',required=True);a=p.parse_args();definition=json.loads(pathlib.Path(a.definition).read_text());units=collect(a.evidence_root);rows=[]
 for case in sorted(set(u['case_id'] for u in definition['units'])):
  declared=[u for u in definition['units'] if u['case_id']==case];methods=[u['method_id'] for u in declared]
  for candidate in [m for m in methods if m!='placo-controlled']:
   reps=sorted(set(u['request']['identity']['repeat_id'] for u in units if u['request']['identity']['case_id']==case)) or [0]
   for rep in reps:
    matches=lambda method:[u for u in units if all(u['request']['identity'].get(k)==v for k,v in [('case_id',case),('method_id',method),('repeat_id',rep)])]
    baseline=matches('placo-controlled');candidates=matches(candidate)
    result=admit(baseline[0] if len(baseline)==1 else None,candidates[0] if len(candidates)==1 else None)
    if len(baseline)>1 or len(candidates)>1:result['reasons'].append('ambiguous multiple matching runs; select explicit evidence roots')
    rows.append({'case_id':case,'baseline':'placo-controlled','candidate':candidate,'repeat_id':rep,**result})
 result={'schema_version':'e06_pair_admission.v1','definition':artifact(a.definition),'pair_keys':PAIR_KEYS,'required_pairs':rows,'analysis':'deferred','input_roots':[str(pathlib.Path(p).resolve()) for p in a.evidence_root]};result['content_hash']=stable_hash(result);write_json(a.output,result);return 0
if __name__=='__main__':raise SystemExit(main())
