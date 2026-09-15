#!/usr/bin/env python3
"""Generate a complete declared family scan; never execute candidate methods."""
import argparse,copy,json,pathlib,sys
import numpy as np
HERE=pathlib.Path(__file__).resolve().parent
LAB=HERE.parents[1]
sys.path.insert(0,str(LAB/'tools/mcc_placo_study'))
from inputs import canonical_snapshot,save_input,UrdfFk,JOINTS,ACTIVE,rotation
from evidence import write_json,artifact,validate_definition
EXPERIMENT=HERE.name[:3]
METHODS=['placo-controlled','mcc-weighted','mcc-hqp-2','mcc-hqp-3']
def config(method,**kwargs):
 c=dict(period_s=.001,gain_per_s=1.,mode='ServoStep',state_mode='snapshot',steps=1,target_iterations=20,target_budget_ms=3.,qp_iterations=100,qp_absolute_tolerance=1e-7,qp_relative_tolerance=0.,warm_start=True,regularization=1e-8,hard_tolerance=1e-7,preservation_tolerance=1e-7,orientation_weight=1.,secondary_weight=1.,secondary_tasks=EXPERIMENT!='E06',secondary_link4=True,secondary_posture=True,primary_only=False,enforcement='soft',scale_group='shared',scale_progress_weight=1.,native_acceleration=False,acceleration_limit=4.,swap_priorities=False,position_tolerance_m=.001,orientation_tolerance_rad=.01,target_min_position_improvement_m=1e-5,target_min_orientation_improvement_rad=1e-4,target_min_posture_improvement_rad=1e-5,terminal_policy='disabled',maximum_iterations_policy='reject',normalization='identity',geometry_policy='model-only',feedback_model='ideal-kinematic-accepted-state; rejected attempt freezes measured q and v',rejection_policy='freeze-measured-q-and-v; no failed iterate consumed',rejected_derivative_limit='frozen measured velocity is not a physical zero-velocity stopped plant',derivative_policy='solver-commanded-velocity; TargetSolve derivative unavailable',hard_position_margin=0.,placo_use_sparsity=False,placo_rewrite_equalities=True,scale_equation='J*(v-v_baseline)=s*(gain*error-J*v_baseline)',causal_parity_artifact=None,causal_parity_status='unavailable-until-E06-audit',stall_threshold_mps=.0001,recovery_position_threshold_m=.001,recovery_orientation_threshold_rad=.01,recovery_dwell_s=.1,recovery_release_s=4.)
 c.update(kwargs);return c

def generate(model):
 fk=UrdfFk(model);units=[];inventory=[]
 def fresh(perturb=0.):
  data=copy.deepcopy(canonical_snapshot(model,perturbation=perturb));data['generator']=artifact(__file__);data['generator_parameters']['experiment_id']=EXPERIMENT
  for s in data['samples']:
   s['elbows']={side:fk.pose(JOINTS,s['q'],side+'_arm_link4',tcp=[0,0,0])['position'] for side in ['left','right']};s['posture']=list(s['q']);s['secondary_enabled']=True;s['task_revision']=0
  return data
 def emit(case,data,methods=METHODS,variants=None,unavailable=None,**kw):
  data=copy.deepcopy(data);data['input_id']=EXPERIMENT+'-'+case;descriptor=save_input(HERE/'inputs/generated'/(case+'.json'),data)
  for method in methods:
   arms=variants or [('full',{})]
   for arm,options in arms:
    c=config(method,**kw);c.update(options)
    if 'analytic' in data:c.update(qp_iterations=1000,qp_iterations_provenance='native QpSolverConfig ProxQpOptions default; generic analytic facade')
    u=dict(case_id=case,method_id=method,arm_id=method+'-'+arm,input=descriptor,config=c,required=True,binary='mcl_study_'+EXPERIMENT.lower(),smoke_config={'steps':min(c['steps'],8)},validation_command=[sys.executable,str(HERE/'verify.py'),'--request','{request}'],repeats={'development':1,'pilot':3,'confirmatory':10})
    reason=unavailable
    if method=='placo-controlled' and c['enforcement']=='scaled':reason='PlaCo kinematics scaled rows do not expose common feasible-velocity baseline/shared versus per-arm scale contract'
    if method=='placo-controlled' and c['native_acceleration']:reason='PlaCo native joint limits lack MCC acceleration/braking feasible-set equivalence'
    if reason:u.update(available=False,unavailable_reason=reason)
    if method=='production-static':u['binary']='mcl_baseline';u['command']=[sys.executable,str(HERE/'production_adapter.py'),'--request','{request}','--binary','{binary}'];u['config']={'profile':'production-static','algorithm_overrides':'forbidden','steps':1,'mode':'historical-TargetSolve','state_mode':'snapshot'};u['smoke_config']={'maximum_input_samples':1}
    units.append(u)
  inventory.append({'case_id':case,'descriptor':descriptor,'shared_hard_set_feasibility':data.get('initial_feasibility','position-and-velocity-box-feasible'),'candidate_independent_generation':True})
 if EXPERIMENT=='E06':
  for fam in ['model_mapping','representation']:emit(fam,fresh(),['production-static','placo-controlled','mcc-weighted'] if fam=='model_mapping' else ['placo-controlled','mcc-weighted'])
  for p,name in [(0,'zero'),(.01,'small'),(.05,'medium')]:emit('target_pose_'+name,fresh(p),METHODS[:2],mode='TargetSolve')
  for vel in [0.,.25]:
   d=fresh(.01);d['samples'][0]['v']=[vel*v if j in ACTIVE else 0. for j,v in zip(JOINTS,d['limits']['velocity'])];d['initial_state']['v']=d['samples'][0]['v'];emit('servo_snapshot_velocity_'+str(vel),d,METHODS[:2])
  for e in ['hard','soft','scaled']:emit('enforcement_'+e,fresh(.01),METHODS[:2],enforcement=e)
 elif EXPERIMENT=='E07':
  variants=[('full',{}),('primary-only',{'primary_only':True})]
  problems={'analytic_redundant':{'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,0]],'b':[1]},{'A':[[0,1]],'b':[1]}]},'analytic_conflict':{'lower':[-2,0],'upper':[2,0],'tasks':[{'A':[[1,0]],'b':[1]},{'A':[[1,1]],'b':[0]}]},'analytic_rank_deficient':{'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,1],[2,2]],'b':[1,2]},{'A':[[0,1]],'b':[1],'enabled':False}]}}
  problems['analytic_active_bound']={'lower':[-2,-2],'upper':[2,2],'tasks':[{'A':[[1,0]],'b':[3]},{'A':[[0,1]],'b':[1]}]}
  for name,problem in problems.items():
   for swap in ([False,True] if 'conflict' in name else [False]):
    d=fresh();d['analytic']=copy.deepcopy(problem)
    if swap:d['analytic']['tasks'].reverse()
    emit(name+('_swapped' if swap else ''),d,METHODS[1:],variants=variants)
    emit(name+('_swapped' if swap else ''),d,['placo-controlled'],variants=variants)
  def temporal(d,duration=4.):
   start=copy.deepcopy(d['samples'][0]);start['secondary_enabled']=False
   active=copy.deepcopy(d['samples'][0]);active['source_time_s']=1.;active['sequence']=1;active['task_revision']=1
   end=copy.deepcopy(start);end['source_time_s']=3.;end['sequence']=2;end['task_revision']=2
   d['samples']=[start,active,end];return d
  for side in ['left','right']:
   for axis in range(3):
    for delta in [-.05,-.02,.02,.05]:
     for mode in ['snapshot','evolving']:
      d=fresh();d['samples'][0]['elbows'][side][axis]+=delta
      if mode=='evolving':d=temporal(d)
      emit(f'r1_hold_tcp_{side}_{axis}_{delta}_{mode}',d,variants=variants,state_mode=mode,steps=4000 if mode=='evolving' else 1)
  for angle in [.1,.3]:
   for mode in ['snapshot','evolving']:
    d=fresh()
    for side in ['left','right']:d['samples'][0]['targets'][side]['rotation']=(rotation([0,0,1],angle)@np.array(d['samples'][0]['targets'][side]['rotation'])).tolist()
    if mode=='evolving':d=temporal(d)
    emit(f'r1_orientation_conflict_{angle}_{mode}',d,variants=variants,state_mode=mode,steps=4000 if mode=='evolving' else 1)
  for weight in [.01,.1,1.,10.,100.]:
   for mode in ['snapshot','evolving']:
    d=fresh();d['samples'][0]['elbows']['left'][0]+=.05
    if mode=='evolving':d=temporal(d)
    emit(f'r1_weight_sweep_{weight}_{mode}',d,variants=variants,secondary_weight=weight,state_mode=mode,steps=4000 if mode=='evolving' else 1)
  for task in ['link4','posture','both']:
   d=temporal(fresh());d['samples'][1]['posture'][6]+=.1;d['samples'][1]['elbows']['left'][0]+=.02;d['transition_task']=task
   emit('r1_task_transition_'+task,d,variants=variants,state_mode='evolving',steps=4000,secondary_link4=task in ['link4','both'],secondary_posture=task in ['posture','both'])
  d=fresh();d['samples'][0]['elbows']['left'][0]+=.02;emit('hard_primary_positive_control',d,['placo-controlled','mcc-weighted'],variants=variants,enforcement='hard')
 else:
  def dynamic(d,kind='continue'):
   difficult=copy.deepcopy(d['samples'][0]);base=copy.deepcopy(difficult);base['targets']={side:fk.pose(JOINTS,base['q'],side+'_arm_ee_link') for side in ['left','right']};base['source_time_s']=0.;base['sequence']=0;base['task_revision']=0
   difficult['source_time_s']=1.;difficult['sequence']=1;difficult['task_revision']=1
   restore=copy.deepcopy(base);restore['source_time_s']=4.;restore['sequence']=2;restore['task_revision']=2
   d['samples']=[base,difficult,restore];return d
  for joint in ['left_arm_joint4','right_arm_joint4','torso_yaw_joint']:
   ix=JOINTS.index(joint)
   for bound in ['lower','upper']:
    for margin in [.005,.02,.1]:
     d=fresh();q=d['samples'][0]['q'].copy();lo=d['limits']['lower'][ix];hi=d['limits']['upper'][ix];q[ix]=lo+margin*(hi-lo) if bound=='lower' else hi-margin*(hi-lo);d['samples'][0]['q']=q;d['initial_state']['q']=q
     d['samples'][0]['targets']={side:fk.pose(JOINTS,q,side+'_arm_ee_link') for side in ['left','right']};emit(f'joint_limit_{joint}_{bound}_{margin}',dynamic(d),state_mode='evolving',steps=6000)
  for vel in [0.,-.25,.25,-.75,.75]:
   for target in ['continue','reverse','stop']:
    for policy in ['common','native-policy']:
     d=fresh();ix=6;d['initial_state']['v'][ix]=vel*d['limits']['velocity'][ix];d['samples'][0]['v']=d['initial_state']['v'].copy();q=d['initial_state']['q'].copy();q[ix]+=(.1 if target=='continue' else -.1 if target=='reverse' else 0.)*(1 if vel>=0 else -1);d['samples'][0]['targets']={side:fk.pose(JOINTS,q,side+'_arm_ee_link') for side in ['left','right']};d['initial_feasibility']='feasible-position-velocity-and-braking-under-4-rad-s2';emit(f'moving_state_{vel}_{target}_{policy}',dynamic(d),state_mode='evolving',steps=6000,native_acceleration=policy=='native-policy')
  for side in ['left','right']:
   for offset in [0.,.05,.15,.3]:
    for group in ['shared','per-arm']:
     d=fresh();d['samples'][0]['targets'][side]['position'][0]+=offset;d['reachability']='known-FK-base; perturbed target global feasibility unknown';emit(f'unreachable_one_arm_{side}_{offset}_{group}',dynamic(d),state_mode='evolving',steps=6000,enforcement='scaled',scale_group=group)
  candidates=[]
  for seed in range(12):
   d=canonical_snapshot(model,seed=seed,perturbation=.01);q=d['initial_state']['q'].copy();rng=np.random.default_rng(seed)
   for ix,j in enumerate(JOINTS):
    if j in ACTIVE:q[ix]=float(rng.uniform(d['limits']['lower'][ix],d['limits']['upper'][ix]))
   J=np.concatenate([fk.jacobian(JOINTS,q,side+'_arm_ee_link')[:,[JOINTS.index(j) for j in ACTIVE]] for side in ['left','right']]);sv=np.linalg.svd(J,compute_uv=False);candidates.append({'seed':seed,'q':q,'singular_values':sv.tolist()})
  candidates.sort(key=lambda x:x['singular_values'][-1]);write_json(HERE/'inputs/generated/singular_candidates.json',candidates)
  for rank,label in [(0,'low'),(5,'middle'),(11,'high')]:
   d=fresh();item=candidates[rank];d['initial_state']['q']=item['q'];d['samples'][0]['q']=item['q'];d['singularity']=dict(bucket=label,selected_rank=rank,candidate_count=12,singular_values=item['singular_values'],scope='independent finite-difference stacked position Jacobian');d['samples'][0]['targets']={side:fk.pose(JOINTS,item['q'],side+'_arm_ee_link') for side in ['left','right']};emit('near_singular_'+label,d)
  for swap in [False,True]:
   d=fresh();d['samples'][0]['targets']['left']['rotation']=(rotation([0,0,1],.3)@np.array(d['samples'][0]['targets']['left']['rotation'])).tolist();d['samples'][0]['posture'][6]+=.1;emit('conflicting_tasks_'+str(swap),dynamic(d),swap_priorities=swap,state_mode='evolving',steps=6000)
  d=fresh();d['initial_state']['q'][6]=d['limits']['upper'][6]+.01;d['samples'][0]['q']=d['initial_state']['q'];d['initial_feasibility']='infeasible-position-box-before-solve';emit('initial_infeasible',d)
  for distance in [.05,.3]:
   d=fresh();d['samples'][0]['targets']['left']['position'][0]+=distance;emit('recovery_'+str(distance),dynamic(d),state_mode='evolving',steps=6000)
  emit('collision_diagnostic_approach_exit',fresh(),unavailable='no frozen mesh hashes, geometry pair list and independent exact-distance provider; soft collision only')
 if EXPERIMENT in ['E07','E08']:
  d=fresh();d['fixture']=True;d['fixture_label']='time-compressed-development-event-coverage; not formal 4s/6s evidence';base=copy.deepcopy(d['samples'][0]);base['secondary_enabled']=False
  challenge=copy.deepcopy(d['samples'][0]);challenge['source_time_s']=.003;challenge['sequence']=1;challenge['task_revision']=1;challenge['elbows']['left'][0]+=.02;challenge['posture'][6]+=.1
  if EXPERIMENT=='E08':challenge['targets']['left']['position'][0]+=.3
  end=copy.deepcopy(base);end['source_time_s']=.012;end['sequence']=2;end['task_revision']=2;d['samples']=[base,challenge,end];d['event_windows']={'baseline':[0,.003],'challenge':[.003,.012],'recovery':[.012,.018]}
  emit('fixture_compressed_events',d,['mcc-hqp-3'],state_mode='evolving',steps=18)
  units[-1]['smoke_config']={'steps':18};units[-1]['config']['recovery_release_s']=.012;units[-1]['phases']=['development']
 definition={'schema_version':'experiment.v2','experiment_id':EXPERIMENT,'question':{'E06':'Which native mathematical semantics are comparable?','E07':'Do declared lexicographic levels preserve their residual optimum?','E08':'How do explicit hard sets, scaling and recovery behave?'}[EXPERIMENT],'units':units,'failure_policy':'continue units; retain native failure and independent checks; failed required units make batch nonzero','metrics':['position_error','orientation_error','hard_violation','hard_violation_excess','preservation_drift','preservation_ratio','secondary_gain','reference_objective_gap','task_scale','progress','stall_duration','recovery_time','full_quality_rate','joint_vaj','source_coverage'],'evaluation_windows':{'snapshot':'one frozen common state','E07':['0-1 baseline','1-3 secondary','3-4 withdrawal'],'E08':['0-1 baseline','1-4 difficulty','4-6 recovery']},'controlled_factors':{'family_scan':'README discrete required factors','seed':20260910,'formal_status':'threshold provenance, parity and input/config freeze pending'},'repeats':{'development':1,'pilot':3,'confirmatory':10}}
 definition['metric_roles']={'position_error':'primary' if EXPERIMENT=='E06' else 'guardrail','orientation_error':'primary' if EXPERIMENT=='E06' else 'guardrail'}
 validate_definition(definition);write_json(HERE/'definition.json',definition);write_json(HERE/'inputs/generated/case_inventory.json',inventory);return len(units)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--model',default='/workspace/models/r1.cos.urdf');a=p.parse_args();print(EXPERIMENT,generate(a.model),'declared units')
