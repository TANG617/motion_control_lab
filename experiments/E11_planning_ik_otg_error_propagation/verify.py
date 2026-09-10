#!/usr/bin/env python3
"""Per-unit independent XML FK stage checks; no cross-run inference or plots."""
import argparse,csv,json,math,pathlib,sys
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from inputs import UrdfFk
from metrics import position_error,orientation_error,hard_violation,correlation_lag,event_response_lag
from evidence import write_json

def verify(request):
 inp=request['input'];cfg=request['config'];out=pathlib.Path(request['output_dir']);fk=UrdfFk(inp['model']['locator']);rows=[];joints=[];events=[];count=0;holds=0;issues=[];trace=[];lagseries={s:{'goal':[],'execution':[]} for s in ('left','right')};times=[];checks=[]
 for line in (out/'raw.jsonl').read_text().splitlines():
  row=json.loads(line)
  if row.get('record_type')!='attempt':continue
  trace.append(row);count+=1;holds+=row['execution_state']=='HOLD';times.append(row['source_time_s'])
  check={'attempt_sequence':row['attempt_sequence'],'executed_position_violation_rad':float(np.max(hard_violation(row['q'],inp['limits']['lower'],inp['limits']['upper']))),'raw_ik_position_violation_rad':float(np.max(hard_violation(row['raw_ik_q'],inp['limits']['lower'],inp['limits']['upper']))) if row.get('raw_ik_q') else None,'ik_disposition':row.get('ik_disposition'),'execution_state':row['execution_state']};checks.append(check)
  if row.get('raw_ik_v'):
   velocity_excess=max(max(abs(v)-limit,0.) for v,limit in zip(row['raw_ik_v'],inp['limits']['velocity']))
   check['raw_ik_velocity_violation_rad_s']=velocity_excess
   if cfg['method'].startswith('mcc'):
    tolerance=cfg['maximum_accepted_hard_violation'];check['raw_ik_position_tolerance_rad']=tolerance*cfg['dt_s'];check['raw_ik_velocity_tolerance_rad_s']=tolerance
    if row.get('ik_committed') and (check['raw_ik_position_violation_rad']>tolerance*cfg['dt_s'] or velocity_excess>tolerance):issues.append(f'accepted raw IK hard violation beyond native velocity-unit tolerance at {count-1}')
   else:check['raw_ik_acceptance_tolerance_reason']='PlaCo does not expose native feasibility acceptance tolerance; physical violations retained without invented epsilon'
  if check['executed_position_violation_rad']>0:issues.append(f'executed physical position violation at {count-1}')
  if not all(math.isfinite(x) for key in ('q','v','a') for x in row[key]):issues.append(f'nonfinite executed state at {count-1}')
  if row['state_sequence']!=max(0,row['attempt_sequence']-round(cfg['feedback_delay_s']/cfg['dt_s'])):issues.append(f'feedback sequence mismatch at {count-1}')
  captured=row['state_sequence'];expected_feedback=inp['initial_state']['q'] if captured==0 else trace[captured-1]['q']
  if row['feedback_q']!=expected_feedback:issues.append(f'feedback state does not match captured committed sequence at {count-1}')
  if row['execution_state']=='HOLD' and count>1 and row['q']!=trace[-2]['q']:issues.append(f'HOLD changed position at {count-1}')
  for side in ('left','right'):
   def forward(q):return fk.pose(inp['joint_names'],q,inp['frames'][side],inp['tcp_offsets'][side],inp['root_frame']) if q else None
   execution=forward(row['q']);ik=forward(row.get('raw_ik_q'));projected=forward(row.get('projected_target_q'));reference=row['reference'][side] if row.get('reference') else None
   pairs=[('goal-reference',row['goal'][side],reference),('reference-ik',reference,ik),('ik-execution',ik,execution),('raw-ik-projected-target',ik,projected),('goal-execution',row['goal'][side],execution)]
   for stage,a,b in pairs:
    rows.append(dict(attempt_sequence=row['attempt_sequence'],source_time_s=row['source_time_s'],side=side,stage=stage,position_error_m=position_error(a['position'],b['position']) if a and b else None,orientation_error_rad=orientation_error(a['rotation'],b['rotation']) if a and b else None,status='available' if a and b else 'missing-native-stage',window=row['window_id']))
   lagseries[side]['goal'].append(row['goal'][side]['position'][0]);lagseries[side]['execution'].append(execution['position'][0])
  for i,name in enumerate(inp['joint_names']):
   jerk=(row['a'][i]-trace[-2]['a'][i])/cfg['dt_s'] if count>1 else None
   joints.append(dict(attempt_sequence=row['attempt_sequence'],joint=name,q=row['q'][i],v=row['v'][i],a=row['a'][i],jerk=jerk,jerk_source='backward-difference acceleration; first sample unavailable',acceleration_source=row.get('acceleration_source','HOLD-zero'),execution_state=row['execution_state']))
  for e in row.get('projection_events',[]):events.append(dict(attempt_sequence=row['attempt_sequence'],**e))
 expected=math.ceil(cfg['duration_s']/cfg['dt_s']);coverage=count/expected if expected else None
 if count!=expected:issues.append(f'partial tick coverage {count}/{expected}')
 lag={s:correlation_lag(v['goal'],v['execution'],cfg['dt_s'],cfg.get('lag_max_s',.1)) for s,v in lagseries.items()}
 for s in lag:lag[s].update(axis='root-frame-x',window='full-executed-development-window',policy='predeclared diagnostic correlation; primary errors unshifted')
 for name,data,fields in [('stage_error_trace.csv',rows,['attempt_sequence','source_time_s','side','stage','position_error_m','orientation_error_rad','status','window']),('joint_stream.csv',joints,['attempt_sequence','joint','q','v','a','jerk','jerk_source','acceleration_source','execution_state']),('projection_events.csv',events,['attempt_sequence','joint_index','before','after'])]:
  with (out/name).open('w') as f:w=csv.DictWriter(f,fields);w.writeheader();w.writerows(data)
 with (out/'stage_checks.jsonl').open('w') as f:
  for c in checks:f.write(json.dumps(c)+'\n')
 with (out/'pipeline_summary.csv').open('w') as f:w=csv.DictWriter(f,['attempts','expected_attempts','tick_coverage','holds','plant_model']);w.writeheader();w.writerow(dict(attempts=count,expected_attempts=expected,tick_coverage=coverage,holds=holds,plant_model='ideal-kinematic-committed-state'))
 write_json(out/'stage_validation.json',dict(status='failed' if issues else 'passed',issues=issues,attempts=count,expected_attempts=expected,tick_coverage=coverage,holds=holds,raw_ik_and_execution_constraints='separate in stage_checks.jsonl; no invented acceptance epsilon',fk='independent-URDF-XML',lag=lag,business_acceptance='not-evaluated: formal thresholds not preregistered'))
 return 1 if issues else 0
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--request',required=True);a=p.parse_args();sys.exit(verify(json.loads(pathlib.Path(a.request).read_text())))
