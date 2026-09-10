#!/usr/bin/env python3
"""Prepare immutable snapshot assets and enumerate the E09 factor matrix; no execution."""
import argparse,copy,itertools,json,pathlib,sys,random
LAB=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(LAB/'tools/mcc_placo_study'))
from inputs import canonical_snapshot,save_input,UrdfFk,ACTIVE,JOINTS
from evidence import write_json,artifact

def base_config():
    return dict(method='mcc',mode='ServoStep',backend='proxqp',layers=1,workload=0,dt_s=.001,
      cpu_ids=[],regularization=1e-8,regularization_policy="frozen native MCC/PlaCo default; common-math parity requires E06",
      primal_infeasibility_tolerance=None,eiquadprog_maximum_iterations=1000,
      minimum_position_improvement_m=1e-5,minimum_orientation_improvement_rad=1e-4,minimum_posture_improvement_rad=1e-5,
      placo_use_sparsity=True,placo_rewrite_equalities=True,warm_start=True,maximum_qp_iterations=1000,absolute_tolerance=1e-8,
      relative_tolerance=0.,hard_tolerance=1e-7,preservation_tolerance=1e-7,servo_gain=10.,
      posture_weight=1.,maximum_target_iterations=20,target_budget_ms=3.,position_tolerance=.001,
      orientation_tolerance=.01,observation='minimal',warmup_calls=100,measured_calls=1000,
      terminal='disabled',maximum_iterations_policy='reject',joint_limit_margin=0.,
      task_enforcement='soft-unit-normalization',posture_role='native-default',
      acceleration_constraints=False,collision_constraints=False,parity_admission='requires-E06; descriptive-until-approved')

def prepare(model, output=None):
    root=pathlib.Path(output) if output else pathlib.Path(__file__).parent
    asset=root/'inputs'/'generated';asset.mkdir(parents=True,exist_ok=True)
    fk=UrdfFk(model);descriptors={}
    for family in ['interior','near-boundary','active-set-change']:
        data=canonical_snapshot(model,perturbation=.002);data['input_id']='e09-'+family+'-motion-v2';data['samples']=[];data['study_generator']=artifact(__file__)
        for i in range(20):
            q=list(data['initial_state']['q']);qgoal=list(q)
            for side in ['left','right']:
                j=JOINTS.index(side+'_arm_joint6')
                if family=='near-boundary' or (family=='active-set-change' and i%4>=2):q[j]=data['limits']['upper'][j]-.0001
                qgoal=list(q);qgoal[j]=q[j]-.001 if i%2 else min(q[j]+.001,data['limits']['upper'][j])
                shoulder=JOINTS.index(side+'_arm_joint1');qgoal[shoulder]+=.02 if i%2 else -.02
                if side=='left':leftgoal=list(qgoal)
            targets={side:fk.pose(JOINTS,leftgoal if side=='left' else qgoal,data['frames'][side]) for side in ['left','right']}
            data['samples'].append(dict(sequence=i,source_time_s=i*.01,q=q,v=[0.]*20,targets=targets,
                link4_targets={side:fk.pose(JOINTS,q,side+'_arm_link4',tcp=(0,0,0)) for side in ['left','right']}))
        data['posture_target']=data['initial_state']['q'];data['snapshot_policy']='Frozen ordered q/v/target; no candidate feedback'
        descriptors[family]=save_input(asset/(family+'-motion-v2.json'),data)
    units=[]
    def unit(family,config,case_suffix='',reason=None):
        c=config;case='-'.join([family,c['mode'],'w'+str(c['workload']),'l'+str(c['layers']),case_suffix]).rstrip('-')
        arm='-'.join([c['method'],c['backend'],'warm'+str(int(c['warm_start'])),c['observation']])
        units.append(dict(case_id=case,method_id=c['method']+'-controlled',arm_id=arm,input=descriptors[family],config=c,
          binary='mcl_study_e09',required=True,available=reason is None,unavailable_reason=reason,
          resource_requirements={'minimum_cpu_count':1,'explicit_cpu_ids':True},
          smoke_config=dict(warmup_calls=2,measured_calls=8),phase_config={'pilot':{'warmup_calls':100,'measured_calls':10000},'confirmatory':{'warmup_calls':100,'measured_calls':10000}},
          validation_command=[sys.executable,str(pathlib.Path(__file__).parent/'verify.py'),'--request','{request}']))
    for mode,workload,layers,family,method,backend,warm,observation in itertools.product(
        ['ServoStep','TargetSolve'],range(4),[1,2,3],descriptors,['mcc','placo'],['proxqp','eiquadprog'],[False,True],['minimal','full','cpp-new']):
        c=base_config();c.update(mode=mode,workload=workload,layers=layers,method=method,backend=backend,warm_start=warm,observation=observation)
        reason=None
        if method=='placo' and backend!='eiquadprog':reason='PlaCo native problem backend is eiquadprog; ProxQP unavailable'
        elif method=='placo' and layers>1:reason='PlaCo native weighted/scaled solver does not expose strict semantic HQP'
        elif method=='placo' and warm:reason='PlaCo has no native warm-start on/off API; do not simulate with dummy solves'
        elif method=='mcc' and backend=='eiquadprog' and warm:reason='MCC eiquadprog has no native warm-start on/off API; factor unavailable'
        elif method=='mcc' and mode=='TargetSolve' and layers>1:reason='HierarchicalKinematicsSolver supports ServoStep only'
        elif layers==2 and workload==0:reason='Position-only workload has one semantic priority; cannot fabricate second layer'
        elif layers==3 and workload<2:reason='Third semantic layer requires posture/link4 physical workload'
        unit(family,c,reason=reason)
    for active in [JOINTS,[j for j in ACTIVE if not j.startswith('torso')]]:
        for workload,layers,observation in itertools.product(range(4),[1,2,3],['minimal','full','cpp-new']):
            c=base_config();c.update(workload=workload,layers=layers,observation=observation,active_joint_names=active)
            reason='Requested semantic layer is empty for this physical workload' if (layers==2 and workload<1 or layers==3 and workload<2) else None
            unit('interior',c,'dof'+str(len(active)),reason)
    random.Random(20260910).shuffle(units)
    definition=dict(schema_version='experiment.v2',experiment_id='E09',question='Full IK call cost under frozen physical workload and native capability',units=units,
      failure_policy='continue-preserve-all',metrics=['ik_call_time','cpu_time','allocation_count','hard_violation','full_quality_rate'],
      evaluation_windows={'cold':'first real IK call','warmup':'next 100 calls','steady':'next 1000 development or 10000 formal calls'},
      controlled_factors=['mode','workload','layers','active-set','backend','warm-start','observation','physical-active-DOF'],
      repeats={'development':3,'pilot':10,'confirmatory':10},seed=20260910,formal_timing_requires='RT and frozen resource/config/input/quality admission')
    definition['metric_roles']={'position_error':'guardrail','orientation_error':'guardrail'}
    write_json(root/'definition.json',definition);return definition
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--model',default='/workspace/models/r1.cos.urdf');ap.add_argument('--output');a=ap.parse_args();d=prepare(a.model,a.output);print(json.dumps({'units':len(d['units']),'available':sum(u['available'] for u in d['units'])}))
