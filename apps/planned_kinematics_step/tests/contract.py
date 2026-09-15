#!/usr/bin/env python3
"""Public-entry parity and bounded native planner/weighted/replay contract checks."""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import resource
import struct
import subprocess
import tempfile


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def put(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')
    return path


def quaternion(r):
    # Rotation fixture conversion only; never used in the application algorithm.
    t = sum(r[i][i] for i in range(3))
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return [(r[2][1]-r[1][2])/s, (r[0][2]-r[2][0])/s, (r[1][0]-r[0][1])/s, s/4]
    i = max(range(3), key=lambda k: r[k][k]); j=(i+1)%3; k=(i+2)%3
    s=math.sqrt(1+r[i][i]-r[j][j]-r[k][k])*2
    q=[0.0]*4; q[i]=s/4; q[j]=(r[j][i]+r[i][j])/s; q[k]=(r[k][i]+r[i][k])/s
    q[3]=(r[k][j]-r[j][k])/s
    return q


def exercise(binary, out):
    out.mkdir(parents=True, exist_ok=False)
    calls=[]
    differences={}
    def same(a,b,label):
        errors=[]
        def check(x,y):
            if isinstance(x,dict):
                assert x.keys()==y.keys()
                for key in x:check(x[key],y[key])
            elif isinstance(x,list):
                assert len(x)==len(y)
                for u,v in zip(x,y):check(u,v)
            elif isinstance(x,float):
                errors.append(abs(x-y));assert abs(x-y)<=1e-12,(label,x,y)
            else:assert x==y,(label,x,y)
        check(a,b);differences[label]=max(errors,default=0.)
        put(out/'numerical_parity.json', {'absolute_roundoff_threshold':1e-12,'max_absolute_errors':differences})
    def run(args, label, succeeds=True):
        command=[str(binary), *map(str,args)]
        result=subprocess.run(command, capture_output=True, text=True, timeout=30)
        (out/(label+'.stdout')).write_text(result.stdout)
        (out/(label+'.stderr')).write_text(result.stderr)
        calls.append({'label':label, 'command':command,'exit_code':result.returncode})
        put(out/'commands.json', calls)
        assert (result.returncode==0)==succeeds, (label,result.returncode,result.stderr)
        return result
    fixtures=Path(__file__).parent
    data=json.loads((fixtures/'stationary.json').read_text())
    model=Path(data['model']['locator']); assert digest(model)==data['model']['sha256']
    source=put(out/'input.json',data)
    cfg=json.loads((fixtures/'config.json').read_text())
    config=put(out/'config.json',cfg)
    caps=json.loads(run(['--describe-capabilities'],'capabilities').stdout)
    assert caps['execution_structures']==['cartesian-weighted-ik-joint-otg']
    def request(label, config=cfg, input_path=source, fmt='json'):
        return put(out/(label+'.request.json'), {
            'schema_version':'execution_request.v1','app_id':'mcl_planned_kinematics_step',
            'execution_structure':'cartesian-weighted-ik-joint-otg',
            'input':{'path':str(input_path),'sha256':digest(input_path),'format':fmt},
            'app_config':config,'execution':{'runtime_mode':'virtual'},
            'observation':{},'tracking':{'case_id':label},'output_dir':str(out/label)})
    def rows(label):
        return [json.loads(line) for line in (out/label/'raw.jsonl').read_text().splitlines()]
    def attempts(label):return [r for r in rows(label) if r['record_type']=='attempt']
    def numeric(label):return [{k:r[k] for k in ['q','v','a','raw_ik_q','raw_ik_v','reference','ik_disposition','committed']} for r in attempts(label)]
    for solver in ['mcc','placo']:
        config_data={**cfg,'solver':solver};config=put(out/(solver+'.config.json'),config_data)
        req=request(solver+'-request',config_data)
        resolved=json.loads(run(['--request',req,'--dump-resolved-options'],solver+'-dump').stdout)
        assert resolved==config_data
        assert not (out/(solver+'-request')).exists()
        run(['--input',source,'--config',config,'--output-dir',out/(solver+'-direct')],solver+'-direct')
        run(['--request',req],solver+'-request')
        same(numeric(solver+'-request'),numeric(solver+'-direct'),solver+'-public-entries')
        assert len(attempts(solver+'-request'))==3
        assert all(r['ik_accepted'] and r['committed'] for r in attempts(solver+'-request'))
        run(['--request',req],solver+'-append-rejected',False)
    req=request('override')
    run(['--request',req,'--solver','placo'],'override',False)
    corrupted=json.loads(req.read_text());corrupted['input']['sha256']='0'*64
    run(['--request',put(out/'corrupt.json',corrupted)],'hash-rejected',False)
    run(['--request',request('hqp', {**cfg,'solver':'hqp'})],'hqp-rejected',False)
    run(['--request',request('projection', {**cfg,'projection_policy':'explicit-position-bounds'})],'projection-rejected',False)
    # Read-only preflight validates transitive model identity without initializing it.
    changed_model=copy.deepcopy(data);changed_model['model']['sha256']='0'*64
    model_source=put(out/'changed-model.json',changed_model)
    model_request=request('changed-model',cfg,model_source)
    run(['--request',model_request,'--dump-resolved-options'],'changed-model-dump',False)
    assert not (out/'changed-model').exists()
    run(['--request',request('ignored-json-replay',{**cfg,'replay':{}}),'--dump-resolved-options'],'ignored-json-replay',False)
    # .07/.01 is 7.000000000000001 in binary64: the declared nanosecond grid has 7 releases.
    for label,duration,expected in [('clock-exact',.07,7),('clock-partial',.071,8)]:
        run(['--request',request(label,{**cfg,'duration_s':duration})],label)
        status=json.loads((out/label/'native_status.json').read_text())
        assert status['planned_releases']==expected and status['attempts']==expected
        schedule=json.loads((out/label/'planned_schedule.json').read_text())
        assert schedule['duration_ns']==round(duration*1e9) and schedule['period_ns']==10000000
    # Native infeasible initial state, never clipped to bounds or committed.
    bad=copy.deepcopy(data);bad['initial_state']['q'][6]=bad['limits']['upper'][6]+1.0
    bad_source=put(out/'outside.json',bad)
    run(['--request',request('initial-outside',cfg,bad_source)],'initial-outside',False)
    failures=attempts('initial-outside')
    assert len(failures)==1 and not failures[0]['committed']
    assert failures[0]['q']==bad['initial_state']['q']
    assert sum(r['execution_state']=='not-run' for r in rows('initial-outside') if r['record_type']=='release')==2
    # IK can be native-accepted while downstream OTG rejects incompatible bounds.
    downstream=copy.deepcopy(data)
    downstream['limits']['upper'][6]=downstream['initial_state']['q'][6]-.1
    downstream_source=put(out/'downstream-incompatible.json',downstream)
    run(['--request',request('downstream-reject',cfg,downstream_source)],'downstream-reject',False)
    failure=attempts('downstream-reject')[0]
    assert failure['ik_accepted'] and not failure['committed'] and not failure['joint_plan_ok']
    assert failure['q']==downstream['initial_state']['q']
    # A nonstationary target traverses both planners; no fabricated target projection.
    moving=copy.deepcopy(data)
    moving['samples'].append(copy.deepcopy(moving['samples'][0]))
    moving['samples'][1]['source_time_s']=.01
    moving['samples'][1]['targets']['left']['position'][0]+=.0001
    moving_source=put(out/'moving.json',moving)
    run(['--request',request('moving', {**cfg,'duration_s':.1},moving_source)],'moving')
    motion=attempts('moving')
    assert motion[-1]['q'] != motion[0]['q']
    assert all(r['committed'] for r in motion)
    # Real shared CSV and MCAP decoders, then identical app-local solver/loop.
    csv_source=out/'stationary.csv'
    names=['timestamp_ns']+[side+'_'+field for side in ['left','right'] for field in ['frame_id','x','y','z','qx','qy','qz','qw']]
    csv_rows=[]
    for tick in range(3):
        row={'timestamp_ns':tick*10000000}
        for side in ['left','right']:
            p=data['samples'][0]['targets'][side]
            row[side+'_frame_id']=data['root_frame']
            for k,val in zip(['x','y','z','qx','qy','qz','qw'],p['position']+quaternion(p['rotation'])):row[side+'_'+k]=val
        csv_rows.append(row)
    with csv_source.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=names);writer.writeheader();writer.writerows(csv_rows)
    replay_cfg={**cfg,'robot_input':{'path':str(source),'sha256':digest(source)},'replay':{'left_stream':'left','right_stream':'right'}}
    run(['--request',request('csv',replay_cfg,csv_source,'csv')],'csv')
    ignored={**replay_cfg,'replay':{'timestamp_sorce':'log_time'}}
    run(['--request',request('replay-typo',ignored,csv_source,'csv'),'--dump-resolved-options'],'replay-typo',False)
    assert not (out/'replay-typo').exists()
    corrupt_robot={**replay_cfg,'robot_input':{'path':str(source),'sha256':'0'*64}}
    run(['--request',request('robot-hash',corrupt_robot,csv_source,'csv'),'--dump-resolved-options'],'robot-hash',False)
    assert not (out/'robot-hash').exists()
    from mcap.writer import Writer
    mcap_source=out/'stationary.mcap'
    with mcap_source.open('wb') as f:
        w=Writer(f);w.start(profile='ros2')
        schema=w.register_schema(name='geometry_msgs/msg/PoseStamped',encoding='ros2msg',data=b'std_msgs/Header header\ngeometry_msgs/Pose pose\n')
        channels={side:w.register_channel(topic=side,message_encoding='cdr',schema_id=schema) for side in ['left','right']}
        for tick in range(3):
            for side in ['left','right']:
                p=data['samples'][0]['targets'][side]
                frame=(data['root_frame']+'\0').encode()
                body=struct.pack('<iII',0,tick*10000000,len(frame))+frame
                body+=b'\0'*((-len(body))%8)
                body+=struct.pack('<7d',*(p['position']+quaternion(p['rotation'])))
                w.add_message(channel_id=channels[side],log_time=tick*10000000,publish_time=tick*10000000,data=b'\0\x01\0\0'+body,sequence=tick)
        w.finish()
    run(['--request',request('mcap',replay_cfg,mcap_source,'mcap')],'mcap')
    same(numeric('csv'),numeric('mcap'),'csv-mcap')
    assert len(attempts('csv'))==3
    assert digest(source)==json.loads((out/'mcc-request'/'binding.json').read_text())['input']['sha256']
    put(out/'summary.json',{'status':'passed','checks':['capabilities-no-model','public-entry-numerical-equivalence-mcc-placo','append-only','read-only-resolved-options','transitive-reference-hash-preflight','unknown-nested-option-rejection','integer-release-boundaries','hash-and-override-rejection','unsupported-hqp','no-candidate-projection','native-initial-outside-failure-and-not-run','accepted-ik-rejected-joint-plan','moving-planner-chain','shared-csv-mcap-decoders'], 'motion_case_max_duration_s':.1,'process_timeout_s':30})
    print(out/'summary.json')


if __name__=='__main__':
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--artifacts',type=Path)
    a=p.parse_args()
    if a.artifacts:exercise(a.binary.resolve(),a.artifacts.resolve())
    else:
        with tempfile.TemporaryDirectory(prefix='mcl-planned-contract-') as tmp:exercise(a.binary.resolve(),Path(tmp)/'evidence')
