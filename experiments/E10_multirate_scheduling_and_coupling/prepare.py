#!/usr/bin/env python3
"""Freeze synthetic trajectory/event assets and the E10 paired family matrix."""
import argparse,copy,json,pathlib,sys,math,random
LAB=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(LAB/'tools/mcc_placo_study'))
from inputs import canonical_snapshot,save_input,UrdfFk,JOINTS
from evidence import write_json,artifact

def base_config():
    return dict(method='mcc',mode='ServoStep',backend='proxqp',layers=2,workload=2,dt_s=.001,
      regularization=1e-8,regularization_policy="frozen native MCC/PlaCo default; common-math parity requires E06",
      primal_infeasibility_tolerance=None,eiquadprog_maximum_iterations=1000,
      minimum_position_improvement_m=1e-5,minimum_orientation_improvement_rad=1e-4,minimum_posture_improvement_rad=1e-5,
      placo_use_sparsity=True,placo_rewrite_equalities=True,warm_start=True,maximum_qp_iterations=1000,absolute_tolerance=1e-8,relative_tolerance=0.,
      hard_tolerance=1e-7,preservation_tolerance=1e-7,servo_gain=10.,posture_weight=1.,maximum_target_iterations=20,
      target_budget_ms=3.,position_tolerance=.001,orientation_tolerance=.01,observation='full',terminal='disabled',
      maximum_iterations_policy='reject',joint_limit_margin=0.,task_enforcement='soft-unit-normalization',
      scheduler='synchronous-divided',schedule_mode='realtime',solver_hz=1000.,secondary_hz=100.,target_hz=100.,
      feedback_hz=1000.,output_hz=1000.,duration_s=2.,coupling_enabled=True,proposal_delay_ms=0.,state_delay_ms=0.,
      pause_start_s=5.,pause_duration_s=.1,injection='none',load='none',load_cpu_budget_us=200.,cpu_ids=[],
      buffer_capacity=250000,virtual_primary_cost_ns=100000,virtual_secondary_cost_ns=200000,
      failure_policy='native reject; no prior failed iterate consumed; execution HOLD',
      coupling_policy='latest attempted proposal; rejected publication disables posture; no age expiry',
      feedback_model='ideal post-command kinematic state; independent capture and delayed delivery',
      initialization='measured q/v canonical initial state at time zero; no initial proposal; no hidden warm-up',
      windows={'initialization':[0.,.1],'steady':[.1,2.],'injection':[5.,5.1]})

def prepare(model,output=None):
    root=pathlib.Path(output) if output else pathlib.Path(__file__).parent;asset=root/'inputs/generated'/(artifact(__file__)['sha256'][:12]+'-'+artifact(model)['sha256'][:12]);asset.mkdir(parents=True,exist_ok=True);fk=UrdfFk(model);descriptors={}
    for family in ['hold-secondary','reachable-sine','bounded-difficult-window']:
        data=canonical_snapshot(model,perturbation=0);data['input_id']='e10-'+family;data['samples']=[];data['study_generator']=artifact(__file__)
        for i in range(600):
            t=i*.01;qgoal=list(data['initial_state']['q'])
            if family!='hold-secondary':
                for side in ['left','right']:qgoal[JOINTS.index(side+'_arm_joint6')]+=.025*math.sin(2*math.pi*.25*t)
            targets={side:fk.pose(JOINTS,qgoal,data['frames'][side]) for side in ['left','right']}
            if family=='bounded-difficult-window' and 2.<=t<2.5:
                for side in ['left','right']:targets[side]['position'][0]+=.06
            data['samples'].append(dict(sequence=i,source_time_s=t,q=data['initial_state']['q'],v=[0.]*20,targets=targets))
        data['posture_target']=list(data['initial_state']['q']);data['posture_target'][JOINTS.index('left_arm_joint3')]+=.1
        data['injection_schedule']={'clock':'run-relative','proposal_delay_ms':[0,5,20,50],'state_delay_ms':[0,5,20,50],
           'families':['pause','publication-failure','old-version'],'event_start_s':5.,'duration_s':.1,
           'smoke_override':{'event_start_s':.5,'duration_s':.1},'simultaneous_delay_factors':False}
        data['feedback_delivery_schedule']={'capture_period_s':.001,'delivery_delay_factor':'state_delay_ms','model':'ideal committed state'}
        descriptors[family]=save_input(asset/(family+'.json'),data)
    units=[];schedulers=['synchronous-full','synchronous-divided','asynchronous-shared-core','asynchronous-two-core']
    def add(family,scan,scheduler,coupling,patch=None,method='mcc',unavailable=None):
        c=base_config();c.update(scheduler=scheduler,coupling_enabled=coupling,secondary_hz=1000. if scheduler=='synchronous-full' else 100.,method=method);c.update(patch or {})
        case=family+'-'+scan;arm=scheduler+'-'+('coupled' if coupling else 'proposal-disabled')
        units.append(dict(case_id=case,method_id=method+'-controlled',arm_id=arm,input=descriptors[family],config=c,
          binary='mcl_study_e10',required=True,available=unavailable is None,unavailable_reason=unavailable,
          resource_requirements={'minimum_cpu_count':2 if scheduler=='asynchronous-two-core' or c['load']=='other-core' else 1,'explicit_cpu_ids':True},
          smoke_config={'duration_s':2.,'pause_start_s':.5,'windows':{'initialization':[0.,.1],'steady':[.1,2.],'injection':[.5,.6]}},
          phase_config={'pilot':{'duration_s':10.,'pause_start_s':5.},'confirmatory':{'duration_s':30.,'pause_start_s':5.}},
          validation_command=[sys.executable,str(pathlib.Path(__file__).parent/'verify.py'),'--request','{request}']))
    for family in descriptors:
        for scheduler in schedulers:
            for coupling in [True,False]:add(family,'baseline',scheduler,coupling)
    for scheduler in schedulers:
        for coupling in [True,False]:
            if scheduler!='synchronous-full':
                for rate in [20,50,100,200]:add('reachable-sine','rate'+str(rate),scheduler,coupling,{'secondary_hz':float(rate)})
            for feedback in [100,250]:add('reachable-sine','feedback'+str(feedback),scheduler,coupling,{'feedback_hz':float(feedback)})
            for factor in ['proposal_delay_ms','state_delay_ms']:
                for delay in [0,5,20,50]:add('reachable-sine',factor+'-'+str(delay),scheduler,coupling,{factor:float(delay)})
            for injection in ['pause','publication-failure','old-version']:add('hold-secondary',injection,scheduler,coupling,{'injection':injection})
            for load in ['none','same-core','other-core']:add('reachable-sine','load-'+load,scheduler,coupling,{'load':load})
            add('hold-secondary','virtual-schedule',scheduler,coupling,{'schedule_mode':'virtual','injection':'pause'})
            add('hold-secondary','native-capability',scheduler,coupling,method='placo',unavailable='PlaCo native solver lacks matching strict two-level orientation/posture topology; no external HQP rewrite')
    # A labelled real-app observation failure fixture is development-only, outside the192 scientific cells.
    fixture=canonical_snapshot(model,fixture=True);fixture['input_id']='e10-fixture-observation-overflow';fixture['session_id']='fixture-e10-observation';fixture['study_generator']=artifact(__file__)
    fixture['posture_target']=fixture['initial_state']['q'];fixture['injection_schedule']={'fixture':'fixed-capacity-observation-overflow','buffer_capacity':8}
    fixture_path=save_input(asset/'fixture-observation-overflow.json',fixture)
    config=base_config();config.update(schedule_mode='virtual',duration_s=.02,buffer_capacity=8,coupling_enabled=False,secondary_hz=100.,injection='none',windows={'fixture':[0.,.02]})
    units.append(dict(case_id='fixture-observation-overflow',method_id='fixture-observation',arm_id='real-mcc-virtual-buffer8',input=fixture_path,config=config,
      binary='mcl_study_e10',required=True,available=True,phases=['development'],repeats={'development':1},smoke_config={'duration_s':.02},
      expected_development_outcome='native exit2 with positive timing_buffer_overflow; validation missing-samples failure is expected and must remain visible',
      validation_command=[sys.executable,str(pathlib.Path(__file__).parent/'verify.py'),'--request','{request}']))
    random.Random(20260910).shuffle(units)
    definition=dict(schema_version='experiment.v2',experiment_id='E10',question='Separate reduced secondary work, asynchronous structure and added CPU resources',
      units=units,failure_policy='continue-preserve-all',metrics=['release_to_finish','deadline_miss_rate','proposal_age','state_age','cpu_time','hard_violation','full_quality_rate'],
      evaluation_windows={'initialization':'0–0.1 s, proposal initially unavailable','steady':'remaining declared duration','injection':'5–5.1 s; smoke 0.5–0.6 s'},
      controlled_factors=['scheduler','resource','secondary-rate','feedback-rate','four-independent-clocks','coupling','proposal-delay','state-delay','pause-failure-old-version','cpu-load'],
      repeats={'development':1,'pilot':3,'confirmatory':10},seed=20260910,formal_timing_requires='RT and frozen same-type physical core resource allocation')
    definition['metric_roles']={'position_error':'guardrail','orientation_error':'guardrail'}
    write_json(root/'definition.json',definition);return definition
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--model',default='/workspace/models/r1.cos.urdf');ap.add_argument('--output');a=ap.parse_args();d=prepare(a.model,a.output);print(json.dumps({'units':len(d['units']),'available':sum(u['available'] for u in d['units'])}))
