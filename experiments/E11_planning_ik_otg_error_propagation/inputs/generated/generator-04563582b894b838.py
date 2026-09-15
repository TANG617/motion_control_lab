#!/usr/bin/env python3
"""Freeze deterministic reachable R1 trajectories and the entire E11 factor matrix."""
import argparse,copy,math,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from inputs import canonical_snapshot,UrdfFk,save_input
from evidence import write_json,artifact,validate_definition
METHODS=('placo-controlled','mcc-weighted','mcc-hqp-2')
PIPELINES=('direct','cartesian','joint','cartesian-joint')
def config(method,pipeline,delay=0.):
 return dict(method=method,pipeline=pipeline,dt_s=.001,duration_s=10.,source_period_s=.01,feedback_delay_s=delay,feedback_model='ideal-committed-state',initial_acceleration='zero',servo_gain_per_s=10.,posture_weight=.01,regularization=1e-8,backend='placo-native' if method=='placo-controlled' else 'eiquadprog',joint_limit_policy='model-position-and-velocity',maximum_accepted_hard_violation=1e-7,maximum_iterations_policy='reject',task_position_weight=1.,task_orientation_weight=1.,hqp_preservation_tolerance=1e-8,cartesian_velocity=.3,cartesian_acceleration=2.,cartesian_jerk=20.,cartesian_angular_velocity=1.,cartesian_angular_acceleration=5.,cartesian_angular_jerk=50.,joint_acceleration=10.,joint_jerk=100.,joint_target_derivatives='explicit-zero-stationary-endpoint',synchronization='time',lag_max_s=.1,lag_axis='root-frame-x',lag_estimator='fixed-grid-correlation-full-window',projection_policy='none',rejection_policy='HOLD-position-zero-velocity-continue',derivative_policy='native-velocity; joint-native-acceleration or backward-difference dt=.001; no lookahead')
def trajectory(model,amplitude,frequency,family,fixture=False):
 data=canonical_snapshot(model,fixture=fixture);fk=UrdfFk(model);q0=data['initial_state']['q'];names=data['joint_names'];data['input_id']=f'{family}-a{amplitude:g}-f{frequency:g}';data['generator']=artifact(__file__);data['samples']=[]
 indexes=[names.index(s+'_arm_joint7') for s in ('left','right')];radii=[float((fk.jacobian(names,q0,s+'_arm_ee_link')[:,j]**2).sum()**.5) for s,j in zip(('left','right'),indexes)]
 angles=[2*math.asin(amplitude/(2*r)) if amplitude else 0 for r in radii]
 for seq in range(1001):
  t=seq*.01;event='motion'
  if family=='step':wave=0. if t<2 else 1.;event='step' if 2<=t<2.01 else 'motion'
  elif family=='stop-hold-continue':
   phase=t if t<3 else 3 if t<6 else t-3;wave=math.sin(2*math.pi*frequency*phase);event='hold' if 3<=t<6 else 'transition' if abs(t-3)<.011 or abs(t-6)<.011 else 'motion'
  elif family=='piecewise-smooth':
   u=(t%4)/4;wave=(10*u**3-15*u**4+6*u**5)*(1 if int(t/4)%2==0 else -1);event='transition' if t%4<.01 else 'motion'
  else:wave=math.sin(2*math.pi*frequency*t)
  q=list(q0)
  for j,a in zip(indexes,angles):q[j]+=a*wave
  if any(x<lo or x>hi for x,lo,hi in zip(q,data['limits']['lower'],data['limits']['upper'])):raise ValueError('independent reachable generator rejected joint envelope')
  data['samples'].append(dict(sequence=seq,source_time_s=t,q=q,v=[0.]*len(q),targets={s:fk.pose(names,q,s+'_arm_ee_link') for s in ('left','right')},event=event,derivative_source='target-PVA-not-supplied; q is reachability witness only'))
 data['generator_parameters']=dict(amplitude_m=amplitude,frequency_hz=frequency,family=family,period_s=.01,duration_s=10.,joint7_circle_radii_m=radii,joint7_excursion_rad=angles,screening='independent URDF FK known-joint witness; all sample position limits',rejected_candidates=0)
 data['derivative_policy']='causal zero-order source pose; witnesses are not feedback or ground truth';return data

def prepare(model,out):
 out=pathlib.Path(out);out.mkdir(parents=True,exist_ok=True);units=[]
 specs=[('sine',a,f) for a in (.01,.03) for f in (.2,.5,1.)]+[('piecewise-smooth',.03,.2),('stop-hold-continue',.03,.5),('step',.03,.5),('zero',0.,.2)]
 for family,a,f in specs:
  data=trajectory(model,a,f,family);descriptor=save_input(out/(data['input_id']+'.json'),data)
  for delay in (0.,.005,.02):
   for method in METHODS:
    for pipeline in PIPELINES:
     units.append(dict(case_id=data['input_id']+f'-delay{int(delay*1000)}',method_id=method,arm_id=method+'-'+pipeline,input=descriptor,config=config(method,pipeline,delay),required=True,binary='mcl_study_e11',smoke_config={'duration_s':.025},validation_command=['python3',str(pathlib.Path(__file__).with_name('verify.py')),'--request','{request}']))
  units.append(dict(case_id=data['input_id']+'-historical',method_id='production-static',arm_id='production-static',input=descriptor,config={'identity':'production-static','control_rate_hz':100,'comparison':'historical-overall; original frozen mask/task/rate'},required=True,binary='mcl_baseline',command=['python3',str(pathlib.Path(__file__).with_name('baseline_adapter.py')),'--request','{request}','--binary','{binary}'],smoke_config={'maximum_samples':3}))
 fixture=canonical_snapshot(model,fixture=True);fixture['input_id']='downstream-fault-fixture';fixture_descriptor=save_input(out/'downstream-fault-fixture.json',fixture)
 for policy,label in [('explicit-position-bounds','projection'),('none','otg-rejection')]:
  cfg=config('mcc-hqp-2','joint');cfg.update(fixture_downstream_outside_limit=True,projection_policy=policy,duration_s=.003)
  units.append(dict(case_id='fixture-'+label,method_id='mcc-hqp-2',arm_id='mcc-hqp-2-joint',input=fixture_descriptor,config=cfg,required=True,binary='mcl_study_e11',smoke_config={'duration_s':.003},validation_command=['python3',str(pathlib.Path(__file__).with_name('verify.py')),'--request','{request}']))
 d=dict(schema_version='experiment.v2',experiment_id='E11',question='Which IK differences survive Cartesian and joint planning under fixed ideal feedback?',units=units,failure_policy='continue; preserve rejection, HOLD and incomplete stages',metrics=['position_error','orientation_error','joint_vaj','source_coverage','hard_violation_excess','target_execution_lag'],evaluation_windows=['full','motion','hold','transition','step'],controlled_factors=['amplitude','frequency','family','pipeline','method','feedback_delay'],repeats={'development':1,'pilot':3,'confirmatory':10})
 validate_definition(d);write_json(pathlib.Path(__file__).with_name('experiment.json'),d);return d
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--model',default='/workspace/models/r1.cos.urdf');p.add_argument('--output',default=str(pathlib.Path(__file__).parent/'inputs/generated'));a=p.parse_args();print(len(prepare(a.model,a.output)['units']))
