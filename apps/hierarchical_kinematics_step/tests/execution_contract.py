#!/usr/bin/env python3
"""Bounded public-interface equivalence, native rejection and schedule checks."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path);a=p.parse_args()
if a.output is None:a.output=Path(tempfile.mkdtemp(prefix='mcl-hks-public-'))/'evidence'
a.output.mkdir(parents=True,exist_ok=False)
commands=[]
def run(args, expected=0):
    command=[str(a.binary),*map(str,args)]; result=subprocess.run(command,text=True,capture_output=True,timeout=30)
    commands.append(dict(command=command,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
    (a.output/'commands.json').write_text(json.dumps(commands,indent=2))
    assert result.returncode==expected,(command,result.stderr)
    return result

def write(name,value):
    path=a.output/name;path.write_text(json.dumps(value));return path

def request(name,layout='position-first',execution=None,observation=None,input=None,expect=0,target_space="frame"):
    inp=write(name+'-input.json',input or {'samples':[{},{}]})
    definition={'schema_version':'execution_request.v1','app_id':'mcl_hierarchical_kinematics_step','execution_structure':'snapshot',
        'input':{'path':str(inp),'sha256':hashlib.sha256(inp.read_bytes()).hexdigest(),'format':'json'},
        'app_config':{'profile':'hierarchical','target_space':target_space,'options':{'urdf':'/workspace/models/Psi_R1_visual_collision.urdf','hqp-layout':layout}},
        'execution':execution or {},'observation':observation or {},'tracking':{'test':name},'output_dir':str(a.output/name)}
    req=write(name+'-request.json',definition);run(['--request',req],expect)
    assert hashlib.sha256(inp.read_bytes()).hexdigest()==definition['input']['sha256']
    rows=[json.loads(line) for line in (a.output/name/'raw.jsonl').read_text().splitlines()]
    return req,rows

cap=json.loads(run(['--describe-capabilities']).stdout);assert cap['solver_mode']=='ServoStep'
_,two=request('two')
req,three=request('three','position-orientation-posture')
_,primary=request('primary','primary-only')
for rows,maximum in [(two,1),(three,2),(primary,0)]:
    calls=[r for r in rows if r['kind']=='red_call'];assert len(calls)==2
    assert all(c['accepted'] and c['selected_pass']==maximum for c in calls),(maximum,calls)
run(['--profile','hierarchical','teleop','--urdf','/workspace/models/Psi_R1_visual_collision.urdf','--hqp-layout','position-orientation-posture','--batch-input',a.output/'three-input.json','--batch-output',a.output/'ordinary'])
ordinary=[json.loads(l) for l in (a.output/'ordinary/raw.jsonl').read_text().splitlines()]
assert [(r['q'],r['v'],r['selected_pass']) for r in ordinary if r['kind']=='red_call']==[(r['q'],r['v'],r['selected_pass']) for r in three if r['kind']=='red_call']
# Same physical target through explicit frame and TCP contracts, nonzero app offset.
first=next(r for r in two if r['kind']=='red_call')
mapping=json.loads((a.output/'two/model_mapping.json').read_text())
frame_targets={};tcp_targets={}
for index,side in enumerate(['left','right']):
    position=first['actual_position_targets'][2*index]['position']; flat=first['actual_orientation_targets'][index]['rotation']
    rotation=[flat[i:i+3] for i in range(0,9,3)];offset=mapping[side+'_tcp_offset'];translation=[offset[3],offset[7],offset[11]]
    frame_targets[side]={'position':position,'rotation':rotation}
    tcp_targets[side]={'position':[position[i]+sum(rotation[i][j]*translation[j] for j in range(3)) for i in range(3)],'rotation':rotation}
_,frows=request('frame_pose',input={'samples':[{'targets':frame_targets}]})
_,trows=request('tcp_pose',input={'samples':[{'targets':tcp_targets}]},target_space='tcp')
fcall=next(r for r in frows if r['kind']=='red_call');tcall=next(r for r in trows if r['kind']=='red_call')
assert max(abs(x-y) for x,y in zip(fcall['q'],tcall['q']))<1e-8
assert max(abs(x-y) for x,y in zip(fcall['v'],tcall['v']))<1e-8
resolved=json.loads((a.output/'two/resolved_options.json').read_text());q=resolved['robot']['default_positions'];q[0]=100
_,badstate=request('initial_violation',input={'initial_state':{'q':q,'v':[0]*len(q)},'samples':[{}, {}, {}]},expect=1)
assert next(r for r in badstate if r['kind']=='begin')['input']['initial_state']['q'][0]==100
assert len([r for r in badstate if r['kind']=='release' and r['status']=='not-run'])==3

run(['--request',req,'--red-rate','20'],1)
run(['--profile','planned','teleop','--elbow-reference','pim-ik'],1)
run(['--pim-python','python3'],1)
run(['--elbow-socket','/tmp/no.sock'],1)
_,calls=request('counts',execution={'planned_releases':15,'warmup_calls':3,'schedule':'divided','yellow_divisor':3})
assert [r['phase'] for r in calls if r['kind']=='red_call']==['cold']+['warm-up']*3+['steady']*11
_,fault=request('capacity8',execution={'planned_releases':20},observation={'capacity':8},expect=1)
summary=json.loads((a.output/'capacity8/summary.json').read_text());assert summary['observation_overflow_count']==1 and summary['not_run_releases']==11
cpus=sorted(os.sched_getaffinity(0))
for label in ['async_same_core','async_dual_core']:
    if label=='async_dual_core' and len(cpus)<2:continue
    _,rows=request(label,execution={'planned_releases':20,'schedule':label,'timing_mode':'real','red_cpu':cpus[0],'yellow_cpu':cpus[1] if label=='async_dual_core' else cpus[0],'yellow_divisor':2})
    assert len([r for r in rows if r['kind']=='release'])==20
for mode in ['minimal','cpp-new']:
    request('observation_'+mode,observation={'mode':mode})
    native=[json.loads(line) for line in (a.output/('observation_'+mode)/'native_calls.jsonl').read_text().splitlines()]
    row=next(r for r in native if r['kind']=='native_solver_result' and r['worker']=='red')
    assert row['candidate_available'] and row['selected_pass'] is not None
    if mode=='minimal':assert 'passes' not in row and 'tasks' not in row
    else:assert row['cpp_new_count']>0 and row['cpp_new_bytes']>0
bad_observation=json.loads(req.read_text());bad_observation['observation']['buffer_capacity']=8
bad_observation_path=write('bad-observation-key.json',bad_observation);run(['--request',bad_observation_path,'--dump-resolved-options'],1)

bad=json.loads(req.read_text());bad['input']['sha256']='0'*64;badpath=write('bad-hash.json',bad);run(['--request',badpath],1)
(a.output/'result.json').write_text(json.dumps({'status':'passed','commands':len(commands),'normal_request_numerics_equal':True},indent=2))
print(a.output/'result.json')
