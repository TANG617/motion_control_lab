#!/usr/bin/env python3
"""Build explicit E12 cells without decoding real recordings or opening holdout."""
import argparse,copy,json,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from inputs import canonical_snapshot,save_input
from evidence import write_json,validate_definition,artifact
from data import inventory,save_inventory,TOPICS
def config(method,pipeline,delay=0.):
 return dict(method=method,pipeline=pipeline,dt_s=.001,duration_s=10.,source_period_s=.01,feedback_delay_s=delay,feedback_model='ideal-committed-state',initial_acceleration='zero',servo_gain_per_s=10.,posture_weight=.01,regularization=1e-8,backend='placo-native' if method=='placo-controlled' else 'eiquadprog',joint_limit_policy='model-position-and-velocity',maximum_accepted_hard_violation=1e-7,maximum_iterations_policy='reject',task_position_weight=1.,task_orientation_weight=1.,hqp_preservation_tolerance=1e-8,cartesian_velocity=.3,cartesian_acceleration=2.,cartesian_jerk=20.,cartesian_angular_velocity=1.,cartesian_angular_acceleration=5.,cartesian_angular_jerk=50.,joint_acceleration=10.,joint_jerk=100.,joint_target_derivatives='explicit-zero-stationary-endpoint',synchronization='time',lag_max_s=.1,lag_axis='root-frame-x',lag_estimator='fixed-grid-correlation-full-window',projection_policy='none',rejection_policy='HOLD-position-zero-velocity-continue',derivative_policy='native-velocity; joint-native-acceleration or backward-difference dt=.001; no lookahead')

def prepare(model,out,inventory_path=None,candidate_freeze=None):
 out=pathlib.Path(out);out.mkdir(parents=True,exist_ok=True)
 fixture=canonical_snapshot(model,fixture=True);fixture['input_id']='recording-fixture-zero';fixture['session_id']='fixture-session';fixture['split_id']='development';fixture['source_kind']='labelled-synthetic-recording-fixture'
 fixture['samples']=[dict(copy.deepcopy(fixture['samples'][0]),sequence=i,source_time_s=i*.01,event='source') for i in range(5)]
 desc=save_input(out/'fixture.json',fixture)
 inv=json.loads(pathlib.Path(inventory_path).read_text()) if inventory_path else {'records':[]}
 selected=json.loads(pathlib.Path(candidate_freeze).read_text())['selected'] if candidate_freeze else [];selected_by_method={s['method_id']:s['config'] for s in selected}
 units=[];script=pathlib.Path(__file__).parent
 for r in inv['records']+[dict(recording_id='recording-fixture-zero',fixture=True)]:
  isfixture=r.get('fixture',False);recording=r['recording_id'];descriptor=desc if isfixture else str(out/(recording+'.descriptor.json'))
  if not isfixture and not pathlib.Path(descriptor).exists():write_json(descriptor,dict(schema_version='input_descriptor.v1',input_id=recording,fixture=False,status='input-unavailable',canonical=None,raw_source=r['raw'],model=artifact(model),session_id=r['session_id'],split_id='development',exposure=r['exposure'],reason='raw not converted; source/session freeze pending; no holdout accessed'))
  reference=json.loads(pathlib.Path(descriptor).read_text())
  canonical=fixture if isfixture else json.loads(pathlib.Path(reference['canonical']['locator']).read_text()) if reference.get('canonical') else None
  for stratum,methods in [('historical',['production-static','mcc-hqp-2']),('controlled',['placo-controlled','mcc-weighted','mcc-hqp-2']),('confirmatory',['production-static','placo-controlled','mcc-weighted','mcc-hqp-2'])]:
   for method in methods:
    cfg=copy.deepcopy(selected_by_method.get(method,config(method,'cartesian-joint')));cfg.update(selected_method_provenance='development-candidate; E07-E10 selection and candidate freeze pending',duration_s=canonical['samples'][-1]['source_time_s']+2. if canonical else 10.,source_interval_policy='full-valid-source',settling_duration_s=2.,settling_policy='separate-window; thresholds pending preregistration',stratum=stratum)
    u=dict(case_id=recording+'-'+stratum,method_id=method,arm_id=method,input=descriptor,config=cfg,required=True,binary='mcl_study_e12',smoke_config={'duration_s':.025},repeats={'development':1,'pilot':3,'confirmatory':3},validation_command=[sys.executable,str(script/'verify.py'),'--request','{request}'])
    if canonical is None:u.update(available=False,unavailable_reason='canonical conversion, source/session mapping and candidate freeze pending; raw MCAP not decoded in this implementation task')
    if stratum=='confirmatory' and not isfixture and not (canonical and not isfixture and canonical.get('split_id')=='holdout' and canonical.get('exposure')=='unexposed' and candidate_freeze):u.update(available=False,unavailable_reason='unexposed holdout and candidate freeze unavailable; historical exposure cannot become holdout')
    if isfixture and stratum=='confirmatory':u.update(available=False,unavailable_reason='negative capability fixture: synthetic input cannot provide confirmatory holdout evidence')
    if canonical and canonical.get('split_id')=='holdout' and stratum!='confirmatory':u.update(available=False,unavailable_reason='holdout reserved for frozen confirmatory stratum')
    if method=='production-static':u.update(binary='mcl_baseline',command=[sys.executable,str(script/'baseline_adapter.py'),'--request','{request}','--binary','{binary}'],config={'identity':'production-static','stratum':stratum,'control_rate_hz':100},smoke_config={'maximum_samples':3});u.pop('validation_command',None)
    units.append(u)
 d=dict(schema_version='experiment.v2',experiment_id='E12',question='Frozen recorded-motion execution coverage and later unexposed session validation',units=units,failure_policy='continue failures; stop user interrupt; retain every required cell',metrics=['position_error','orientation_error','valid_completion_rate','hard_violation_excess','deadline_miss_rate','source_coverage','joint_vaj','lag','time_to_failure'],evaluation_windows=['source-full','settling'],controlled_factors=['recording','session','stratum','method','repeat','frozen-downstream'],repeats={'development':1,'pilot':3,'confirmatory':3},formal_readiness='not-ready: raw input conversion, exposure, freeze, RT attestation, thresholds pending',analysis='deferred',paper='deferred')
 for unit in units:
  if json.loads(pathlib.Path(unit['input']).read_text()).get('fixture'):unit['phases']=['development']
 if candidate_freeze:d['candidate_freeze']=artifact(candidate_freeze)
 d['metric_roles']={'position_error':'primary','orientation_error':'primary'}
 validate_definition(d);write_json(script/'definition.json',d);write_json(script/'inputs/conversion_recipe.json',dict(schema_version='conversion_recipe.v1',topics=TOPICS,timestamp_source='header_time_ns',timeline='fixed-period-one-to-one',fixed_period_s=.01,pairing_tolerance_ns=5000000,interpolation='none',frame='base_link',units='m,rad,s',clipping='none',missing_policy='input-invalid',initialization='latest causal full recorded joints; explicit zero velocity'))
 write_json(script/'inputs/required_units.json',[{'case_id':u['case_id'],'method_id':u['method_id'],'repeat_id':repeat,'status':'not-run' if u.get('available',True) else 'unavailable','reason':u.get('unavailable_reason','formal experiment not executed')} for u in units for repeat in range(3)])
 return d
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--model',default='/workspace/models/r1.cos.urdf');p.add_argument('--output',default=str(pathlib.Path(__file__).parent/'inputs/generated'));p.add_argument('--inventory');p.add_argument('--candidate-freeze');a=p.parse_args();print(len(prepare(a.model,a.output,a.inventory,a.candidate_freeze)['units']))
