#!/usr/bin/env python3
"""Independent unit correctness checks; no cross-run inference or statistics."""
import argparse,csv,itertools,json,pathlib,sys
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from inputs import UrdfFk
from evidence import write_json
from metrics import position_error,orientation_error,hard_violation,preservation_ratio,secondary_gain,directional_progress,error_reduction,stall_duration,recovery_time

def box_qp(A,b,lower,upper,regularization=0.,E=None,f=None):
 """Enumerate box active sets and solve equality-constrained KKT by SVD.
 No candidate library, pseudoinverse handles non-unique semantic solutions.
 """
 A=np.asarray(A,float);b=np.asarray(b,float);lower=np.asarray(lower,float);upper=np.asarray(upper,float);n=len(lower)
 E=np.empty((0,n)) if E is None else np.asarray(E,float);f=np.empty(0) if f is None else np.asarray(f,float)
 H=A.T@A+regularization*np.eye(n);g=-A.T@b;best=None
 for active in itertools.product([-1,0,1],repeat=n):
  rows=[E];values=[f]
  for i,s in enumerate(active):
   if s:rows.append(np.eye(n)[[i]]);values.append(np.array([lower[i] if s<0 else upper[i]]))
  C=np.concatenate(rows);d=np.concatenate(values);K=np.block([[H,C.T],[C,np.zeros((len(d),len(d)))]])
  z=np.linalg.lstsq(K,np.r_[-g,d],rcond=None)[0];x=z[:n]
  if np.max(np.abs(C@x-d),initial=0)>1e-9 or np.any(x<lower-1e-9) or np.any(x>upper+1e-9):continue
  cost=.5*np.sum((A@x-b)**2)+.5*regularization*np.sum(x*x)
  if best is None or cost<best[0]:best=(float(cost),x)
 return best

def analytic_reference(problem,hierarchy,primary_only=False,regularization=1e-8):
 tasks=[t for i,t in enumerate(problem['tasks']) if t.get('enabled',True) and (not primary_only or i==0)]
 if not tasks:return {'status':'unavailable','reason':'no active objective'}
 lo=problem['lower'];hi=problem['upper'];E=np.empty((0,2));f=np.empty(0);results=[]
 if not hierarchy:tasks=[{'A':np.concatenate([np.asarray(t['A'])*np.sqrt(t.get('weight',1)) for t in tasks]),'b':np.concatenate([np.asarray(t['b'])*np.sqrt(t.get('weight',1)) for t in tasks])}]
 for t in tasks:
  A=np.asarray(t['A'])*np.sqrt(t.get('weight',1));b=np.asarray(t['b'])*np.sqrt(t.get('weight',1));result=box_qp(A,b,lo,hi,regularization,E,f)
  if result is None:return {'status':'infeasible'}
  cost,x=result;results.append({'objective':float(.5*np.sum((A@x-b)**2)),'regularized_objective':cost,'x':x.tolist(),'A':A.tolist(),'b':b.tolist()});E=np.concatenate([E,A]);f=np.r_[f,A@x]
 return {'status':'solved','levels':results,'x':x.tolist(),'regularization':regularization,'preservation_reference':'exact affine optimum; candidate tolerance bands independently accounted'}

def semantic_audit(left,right):
 fields=['joint_names','active_joint_names','root_frame','frames','tcp_offsets','mode','scale_group','enforcement','normalization','gain_per_s','period_s','regularization','limits','units']
 return [{'field':field,'status':'pass' if left.get(field)==right.get(field) else 'semantic_mismatch','source':left.get(field),'target':right.get(field),'reason':'' if left.get(field)==right.get(field) else 'declared mathematical identity differs'} for field in fields]

def check_preservation(reference,final,tolerance):
 if reference is None or final is None:return {'status':'unavailable','reason':'missing highest-level or final residual'}
 r=np.asarray(reference);f=np.asarray(final);drift=np.abs(f-r);return {'status':'pass' if np.all(drift<=tolerance) else 'failed','drift':drift.tolist(),'preservation_ratio':preservation_ratio(r,f,tolerance)}

def native_reference(task):
 # Core NotRun initializes a dimensioned zero vector; it is NOT an optimum.
 if not task or task.get('state')==0 or not task.get('enabled',True):return None
 return task.get('residual_at_level') or None

def declared_challenge_start(data):
 return data.get('event_windows',{}).get('challenge',[1.])[0]

def recovery_observation(times,errors,angles,accepted,config,state_mode):
 release=config.get('recovery_release_s',4.)
 if state_mode!='evolving':return {'value':None,'status':'not-observed','reason':'snapshot has no recovery observation interval'}
 end=times[-1]+config['period_s']
 if end<=release:return {'value':None,'status':'not-observed','reason':'withdrawal event lies outside executed window','observation_end_s':end,'declared_release_s':release}
 return recovery_time(times,errors,angles,accepted,release,config['recovery_position_threshold_m'],config['recovery_orientation_threshold_rad'],config['recovery_dwell_s'],end)

def verify(request):
 o=json.loads(pathlib.Path(request).read_text());out=pathlib.Path(o['output_dir']);data=o['input'];c=o['config'];raw=out/'raw.jsonl';checks=[];oracle=[];priority=[];constraints=[];progress=[];semantics=[];secondary=[];scale_checks=[];native_rows=[]
 if not raw.exists():write_json(out/'app_validation.json',{'status':'unavailable','reason':'native process produced no raw.jsonl'});return 1
 actual_records=[json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
 if not any(row.get('record_type') in ['attempt','analytic'] for row in actual_records):
  write_json(out/'app_validation.json',{'status':'unavailable','reason':'no completed native attempt or analytic result; begin-only trace is not validation evidence','native_records':0});return 2
 fk=UrdfFk(data['model']['locator']);names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']]
 if HERE.name.startswith('E06'):
  expected={**c,**{key:data[key] for key in ['joint_names','active_joint_names','root_frame','frames','tcp_offsets','limits','units']}}
  mapping=json.loads((out/'model_mapping.json').read_text()) if (out/'model_mapping.json').exists() else {}
  actual={**c,**mapping};semantics=semantic_audit(expected,actual)
  semantics += [{'field':'TCP task versus evaluated TCP','status':'semantic_mismatch','source':'frame-origin position task p_tcp-R_goal*offset','target':'true TCP Jacobian','reason':'surrogate task row differs from true TCP residual when orientation changes'}, {'field':'cross-method causal pairing','status':'unavailable','reason':'this unit audit is insufficient; compare frozen method_semantics artifacts and accepted common subset'}]
 for line,text in enumerate(raw.read_text().splitlines(),1):
  row=json.loads(text)
  if row.get('record_type')=='analytic':
   ref=analytic_reference(row['problem'],'hqp' in o['identity']['method_id'],c['primary_only'],c['regularization']);r={'source_line':line,'reference':ref,'native_status':row['status']}
   x=np.asarray(row.get('candidate',[]));p=row['problem'];r['status']='unavailable'
   if len(x)==2 and ref['status']=='solved':
    hard=float(max(np.max(np.asarray(p['lower'])-x),np.max(x-np.asarray(p['upper'])),0));gaps=[]
    for l in ref['levels']:
     A=np.asarray(l['A']);b=np.asarray(l['b']);cost=.5*np.sum((A@x-b)**2);gaps.append(float(cost-l['objective']))
    r.update(hard_violation=hard,objective_gaps=gaps,status='pass' if hard<=c['hard_tolerance'] and all(abs(g)<1e-5 for g in gaps) else 'failed',acceptance_basis='objective and hard feasibility; joint-vector equality not required; numerical oracle tolerance 1e-5')
   oracle.append(r);continue
  if row.get('record_type')!='attempt':continue
  q=np.asarray(row['q']);state=row.get('input_state',{'q':q.tolist(),'v':[0.]*len(q)});q0=np.asarray(state['q']);v=np.asarray(row['v']);v0=np.asarray(state['v']);lo=np.asarray(data['limits']['lower']);hi=np.asarray(data['limits']['upper']);vmax=np.asarray(data['limits']['velocity']);position_excess=hard_violation(q,lo,hi);velocity_excess=hard_violation(v,-vmax,vmax)
  entry={'source_line':line,'attempt_sequence':row['attempt_sequence'],'position_raw_violation':position_excess.tolist(),'velocity_raw_violation':velocity_excess.tolist(),'initial_position_feasible':bool(np.all((q0>=lo)&(q0<=hi))),'native_disposition':row.get('disposition'),'independent_scope':'URDF XML FK and scalar hard sets; no candidate FK'}
  if c.get('native_acceleration'):
   a=c['acceleration_limit'];braking_margin=np.where(v0>=0,hi-q0,q0-lo)-v0*v0/(2*a);entry['initial_braking_margin']=braking_margin.tolist();entry['initial_braking_feasible']=bool(np.all(braking_margin[active]>=0));entry['acceleration_violation']=np.maximum(np.abs(v-v0)/c['period_s']-a,0).tolist()
  constraints.append(entry)
  for side in ['left','right']:
   target=row['targets'][side];pose=fk.pose(names,q,data['frames'][side],data['tcp_offsets'][side],data['root_frame']);error=position_error(target['position'],pose['position']);angle_error=orientation_error(target['rotation'],pose['rotation']);progress.append({'attempt_sequence':row['attempt_sequence'],'source_time_s':row.get('source_time_s'),'side':side,'position_error_m':error,'orientation_error_rad':angle_error,'actual_position':pose['position'],'reference_position':target['position'],'native_committed':row.get('committed'),'recovery_status':'censored-until-declared-window-complete'})
   if 'hqp' in o['identity']['method_id']:
    J=fk.jacobian(names,q0,data['frames'][side],tcp=[0,0,0])[:,active];frame=fk.pose(names,q0,data['frames'][side],tcp=[0,0,0]);R=np.asarray(target['rotation']);goal=np.asarray(target['position'])-R@np.asarray(data['tcp_offsets'][side]);desired=c['gain_per_s']*(goal-np.asarray(frame['position']));res=J@v[active]-desired
    if c.get('enforcement')=='scaled':
     scales=[s for s in row.get('scales',[]) if s['evaluated']];scale=next((s['value'] for s in scales if s['name']==('progress-0' if side=='left' or c['scale_group']=='shared' else 'progress-1')),None);baseline=np.asarray(row.get('task_scale_reference',[]))
     if scale is not None and len(baseline)==len(active):res=J@(v[active]-baseline)-scale*(desired-J@baseline)
     else:res=None
    task=next((t for t in row.get('tasks',[]) if t['name']==side+'-position'),None);reference=native_reference(task)
    if not reference:reference=None
    result=check_preservation(reference,None if res is None else res.tolist(),c['preservation_tolerance']);result.update(source_line=line,task=side+'-position',reference_provenance='native same-tick residual optimum; independent final matrix residual; oracle optimum unavailable for R1',linearization_q=q0.tolist(),independent_A=J.tolist(),desired=desired.tolist(),independent_final_residual=None if res is None else res.tolist(),requested_passes=row.get('requested_passes'),completed_passes=row.get('completed_passes'));priority.append(result)
  if row.get('native_task_rows') and row.get('active_variable_columns'):
   columns=row['active_variable_columns'];linearization=np.asarray(row['native_task_linearization_q'])
   for native in row['native_task_rows']:
    side=native['name'].split('-')[0];J=fk.jacobian(names,linearization,data['frames'][side],tcp=[0,0,0])[:,active];actual_A=np.asarray(native['A'])[:,columns];target=row['targets'][side];goal=np.asarray(target['position'])-np.asarray(target['rotation'])@np.asarray(data['tcp_offsets'][side]);current=fk.pose(names,linearization,data['frames'][side],tcp=[0,0,0]);desired=(c['gain_per_s']*c['period_s'] if c['mode']=='ServoStep' else 1.)*(goal-np.asarray(current['position']));A_gap=float(np.max(np.abs(actual_A-J)));b_gap=float(np.max(np.abs(np.asarray(native['b'])-desired)))
    native_rows.append({'source_line':line,'task':native['name'],'active_variable_columns':columns,'native_dimensions':row['native_dimensions'],'native_A':actual_A.tolist(),'independent_A':J.tolist(),'native_b':native['b'],'independent_b':desired.tolist(),'max_A_difference':A_gap,'max_b_difference':b_gap,'audit_tolerance':c['preservation_tolerance'],'audit_tolerance_role':'independent row reconstruction; does not relax physical hard sets','status':'pass' if max(A_gap,b_gap)<=c['preservation_tolerance'] else 'semantic_mismatch'})
  if c.get('enforcement')=='scaled':
   baseline=np.asarray(row.get('task_scale_reference',[]));candidate=np.asarray(row.get('candidate_active_v',[]))
   for side in ['left','right']:
    name='progress-0' if side=='left' or c['scale_group']=='shared' else 'progress-1';scale=next((x['value'] for x in row.get('scales',[]) if x['name']==name and x['evaluated']),None);check={'source_line':line,'side':side,'equation':'J*(v-v_baseline)-s*(gain*error-J*v_baseline)','status':'unavailable','native_disposition':row.get('disposition')}
    if scale is not None and len(baseline)==len(active) and len(candidate)==len(active):
     J=fk.jacobian(names,q0,data['frames'][side],tcp=[0,0,0])[:,active];current=fk.pose(names,q0,data['frames'][side],tcp=[0,0,0]);target=row['targets'][side];goal=np.asarray(target['position'])-np.asarray(target['rotation'])@np.asarray(data['tcp_offsets'][side]);desired=c['gain_per_s']*(goal-np.asarray(current['position']));residual=J@(candidate-baseline)-scale*(desired-J@baseline);check.update(status='pass' if np.all(np.abs(residual)<=c['hard_tolerance']) else 'failed',scale=scale,baseline_velocity=baseline.tolist(),desired_velocity=desired.tolist(),independent_J=J.tolist(),equation_residual=residual.tolist(),hard_tolerance=c['hard_tolerance'])
    else:check['reason']='native candidate/scale/reference absent; rejected state is not a candidate'
    scale_checks.append(check)
  if 'hqp' in o['identity']['method_id']:
   for task in row.get('tasks',[]):
    name=task['name']
    if name in ['left-position','right-position']:continue
    if not task.get('enabled'):
     priority.append({'source_line':line,'task':name,'status':'unavailable','reason':'explicitly disabled task; no semantic residual optimum expected'});continue
    reference=native_reference(task);A=None;desired=None
    if name=='posture':
     A=np.eye(len(active));desired=c['gain_per_s']*(np.asarray(row['posture_target'])[active]-q0[active])
    elif name.endswith('-elbow'):
     side=name.split('-')[0];A=fk.jacobian(names,q0,side+'_arm_link4',tcp=[0,0,0])[:,active];current=fk.pose(names,q0,side+'_arm_link4',tcp=[0,0,0]);desired=c['gain_per_s']*(np.asarray(row['elbow_targets'][side])-np.asarray(current['position']))
    elif name.endswith('-orientation'):
     side=name.split('-')[0];current=np.asarray(fk.pose(names,q0,data['frames'][side],tcp=[0,0,0])['rotation']);target=np.asarray(row['targets'][side]['rotation']);desired=c['gain_per_s']*Rotation.from_matrix(target@current.T).as_rotvec();columns=[];eps=1e-6
     for index in active:
      plus=q0.copy();minus=q0.copy();plus[index]+=eps;minus[index]-=eps;Rp=np.asarray(fk.pose(names,plus,data['frames'][side],tcp=[0,0,0])['rotation']);Rm=np.asarray(fk.pose(names,minus,data['frames'][side],tcp=[0,0,0])['rotation']);columns.append(Rotation.from_matrix(Rp@Rm.T).as_rotvec()/(2*eps))
     A=np.asarray(columns).T
    final=None if A is None else A@v[active]-desired;result=check_preservation(reference,None if final is None else final.tolist(),c['preservation_tolerance']);result.update(source_line=line,task=name,reference_provenance='native same-tick per-level residual; independent XML kinematics or identity posture final matrix',independent_A=None if A is None else A.tolist(),desired=None if desired is None else desired.tolist(),independent_final_residual=None if final is None else final.tolist());priority.append(result)
  if c.get('enforcement')=='soft' and c.get('mode')=='ServoStep':
   Js=[];bs=[]
   for side in ['left','right']:
    target=row['targets'][side];J=fk.jacobian(names,q0,data['frames'][side],tcp=[0,0,0])[:,active];frame=fk.pose(names,q0,data['frames'][side],tcp=[0,0,0]);goal=np.asarray(target['position'])-np.asarray(target['rotation'])@np.asarray(data['tcp_offsets'][side]);Js.append(J);bs.append(c['gain_per_s']*(goal-np.asarray(frame['position'])))
   A=np.concatenate(Js);b=np.concatenate(bs);lower=np.maximum((lo[active]-q0[active])/c['period_s'],-vmax[active]);upper=np.minimum((hi[active]-q0[active])/c['period_s'],vmax[active])
   if c.get('native_acceleration'):
    acceleration=c['acceleration_limit'];lower=np.maximum(lower,v0[active]-acceleration*c['period_s']);upper=np.minimum(upper,v0[active]+acceleration*c['period_s']);lower=np.maximum(lower,-np.sqrt(np.maximum(2*acceleration*(q0[active]-lo[active]),0)));upper=np.minimum(upper,np.sqrt(np.maximum(2*acceleration*(hi[active]-q0[active]),0)))
   if np.all(lower<upper):
    reference=lsq_linear(np.r_[A,np.sqrt(c['regularization'])*np.eye(len(active))],np.r_[b,np.zeros(len(active))],bounds=(lower,upper),tol=1e-12)
    objective=.5*np.sum((A@reference.x-b)**2);cost=.5*np.sum((A@v[active]-b)**2)
    for taskcheck in [x for x in priority if x.get('source_line')==line and x.get('independent_A') is not None and x.get('task') not in ['left-position','right-position']]:
     task_A=np.asarray(taskcheck['independent_A']);task_b=np.asarray(taskcheck['desired']);weight=c['orientation_weight'] if taskcheck['task'].endswith('-orientation') else c['secondary_weight'];primary_cost=.5*weight*np.sum((task_A@reference.x-task_b)**2);candidate_cost=.5*weight*np.sum((task_A@v[active]-task_b)**2)
     secondary.append({'source_line':line,'task':taskcheck['task'],'primary_only_reference':'independent position-primary bounded-LS at same frozen linearization','primary_only_secondary_cost':float(primary_cost),'candidate_secondary_cost':float(candidate_cost),'primary_semantic_objective_gap':float(cost-objective),**secondary_gain(float(primary_cost),float(candidate_cost))})
    oracle.append({'source_line':line,'status':'measured' if reference.success else 'unavailable','reference_kind':'independent scipy bounded least squares at frozen state; no candidate solver','reference_scope':'position-primary soft objective','independent_A':A.tolist(),'independent_b':b.tolist(),'lower':lower.tolist(),'upper':upper.tolist(),'reference_velocity':reference.x.tolist(),'reference_semantic_objective':float(objective),'candidate_semantic_objective':float(cost),'reference_objective_gap':float(cost-objective),'reference_optimum_acceptance':'diagnostic gap; same-tick preservation uses exact declared residual tolerance'})
   else:oracle.append({'source_line':line,'status':'unavailable','reason':'initial hard velocity set empty or contains fixed coordinates; retained without optimizer relaxation'})
  checks.append({'source_line':line,'status':'pass' if np.all(np.isfinite(q)) else 'failed','nonlinear_errors_separate_from_linear_preservation':True})
 def table(name,rows):
  write_json(out/(name+'.json'),rows)
  if rows:
   keys=sorted(set().union(*(r.keys() for r in rows)))
   with (out/(name+'.csv')).open('w') as f:
    writer=csv.DictWriter(f,keys);writer.writeheader();writer.writerows({k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in r.items()} for r in rows)
 table('parity_checks',semantics);table('native_task_row_checks',native_rows);table('oracle_checks',oracle);table('secondary_effects',secondary);table('scale_equation_checks',scale_checks);table('priority_checks',priority);table('constraint_trace',constraints);table('progress_recovery',progress);table('quality_events',checks);table('task_transition_trace',[{'attempt_sequence':r['attempt_sequence'],'time_s':r['source_time_s'],'side':r['side'],'position_error_m':r['position_error_m']} for r in progress])
 temporal=[]
 for side in ['left','right']:
  rows=[r for r in progress if r['side']==side]
  if not rows:continue
  times=[r['source_time_s'] for r in rows];errors=[r['position_error_m'] for r in rows];angles=[r['orientation_error_rad'] for r in rows];accepted=[bool(r['native_committed']) for r in rows]
  origin=rows[0]['actual_position'];start=rows[0]['reference_position'];goal=next((r['reference_position'] for r in rows if r['source_time_s']>=declared_challenge_start(data)),start)
  directions=[directional_progress(start,goal,origin,r['actual_position']) for r in rows];values=[d['value'] for d in directions]
  entry={'side':side,'error_reduction_m':[error_reduction(errors[0],v) for v in errors],'directional_progress':directions,'source_time_s':times,'thresholds_provisional_development':True}
  if len(times)>1 and all(t2>t1 for t1,t2 in zip(times,times[1:])) and all(v is not None for v in values):entry['stall']=stall_duration(times,values,c['stall_threshold_mps'])
  else:entry['stall']={'status':'unavailable','reason':'zero direction or insufficient monotonic trajectory'}
  if HERE.name.startswith('E08'):
   end=(times[-1]+c['period_s']) if c.get('state_mode')=='evolving' else times[-1]
   entry['recovery']=recovery_observation(times,errors,angles,accepted,c,c.get('state_mode'))
  temporal.append(entry)
 table('progress_recovery_metrics',temporal)
 applicability=[]
 for field in ['qp_iterations','qp_absolute_tolerance','qp_relative_tolerance','warm_start','target_budget_ms','target_iterations','target_min_position_improvement_m','target_min_orientation_improvement_rad','target_min_posture_improvement_rad']:
  placo='placo' in o['identity']['method_id'];ignored=placo and field!='target_iterations';applicability.append({'field':field,'resolved_value':c.get(field),'applied':not ignored,'reason':'PlaCo native backend or fixed-iteration TargetSolve loop has no corresponding MCC policy; causal mismatch retained' if ignored else 'native MCC policy or explicit fixed PlaCo loop; execution mode applicability in resolved config'})
 table('configuration_applicability',applicability)
 if (out/'method_semantics.json').exists():table('method_semantics',[json.loads((out/'method_semantics.json').read_text())])
 failed=any(r.get('status')=='failed' for r in oracle+checks+priority+scale_checks);write_json(out/'app_validation.json',{'status':'failed' if failed else 'completed','oracle_rows':len(oracle),'priority_rows':len(priority),'raw_native_status_unchanged':True,'R1_reference_optimum':'independent bounded least squares position-primary; complete same-tick task residual verification additionally retained','formal_recovery_threshold_status':'provisional-development; formal freeze required'});return int(failed)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--request',required=True);a=p.parse_args();raise SystemExit(verify(a.request))
