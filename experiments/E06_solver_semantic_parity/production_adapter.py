#!/usr/bin/env python3
"""Mechanical canonical CSV / native trace adapter for frozen production-static.
The baseline owns all numerical behavior. It buffers its native trace until exit;
a crash can therefore preserve stderr without per-attempt state evidence.
"""
import argparse,csv,json,pathlib,subprocess,sys
import numpy as np
from scipy.spatial.transform import Rotation
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from evidence import write_json,artifact

def main():
 p=argparse.ArgumentParser();p.add_argument('--request',required=True);p.add_argument('--binary',required=True);a=p.parse_args();r=json.loads(pathlib.Path(a.request).read_text());d=r['input'];out=pathlib.Path(r['output_dir']);csv_path=out/'canonical_targets.csv'
 fields=['timestamp_ns']+[side+'_'+x for side in ['left','right'] for x in ['frame_id','x','y','z','qx','qy','qz','qw']]
 with csv_path.open('w') as f:
  w=csv.writer(f);w.writerow(fields)
  for i,s in enumerate(d['samples'][:r['config'].get('maximum_input_samples',len(d['samples']))]):
   row=[i*10000000]
   for side in ['left','right']:
    target=s['targets'][side];row += [d['root_frame']]+target['position']+Rotation.from_matrix(target['rotation']).as_quat().tolist()
   w.writerow(row)
 command=[a.binary,'replay','--urdf',d['model']['locator'],'--input',str(csv_path),'--input-format','csv','--left-stream','left','--right-stream','right','--timestamp-source','csv_timestamp','--target-period-ms','10','--execution-mode','batch','--ui','none','--viz','none','--terminal-input','off','--no-mcap','--output-dir',str(out/'native')]
 write_json(out/'production_command.json',command);write_json(out/'resolved_config.json',{'profile':'production-static','algorithm_overrides':False,'binary':artifact(a.binary),'canonical_adapter':'mechanical CSV only','native_trace_crash_limitation':'baseline buffers trace until finalization'})
 code=subprocess.call(command)
 traces=list((out/'native').rglob('trace.csv'))
 with (out/'raw.jsonl').open('w') as f:
  for trace in traces:
   for row in csv.DictReader(trace.open()):
    sequence=int(row['source_sequence']);sample=d['samples'][min(sequence,len(d['samples'])-1)];accepted=row['accepted'] in ('true','1');record=dict(record_type='attempt',attempt_sequence=int(row['attempt']),input_sequence=sequence,state_sequence=int(row['attempt']),task_revision=0,joint_names=d['joint_names'],q=[float(x) for x in row['positions'].split(';')],v=[float(x) for x in row['velocities'].split(';')],targets=sample['targets'],status=row['solver_status'],native_qp_status=None,native_qp_status_reason='frozen baseline exposes no native QP status',disposition='accepted' if accepted else 'rejected',solution_quality=row['solver_status'],selected_priority=None,highest_completed_priority=None,requested_passes=None,completed_passes=None,committed=accepted,committed_sequence=int(row['attempt'])+int(accepted),execution_state='committed' if accepted else 'HOLD',release_ns=None,start_ns=None,finish_ns=None,proposal_revision=None,proposal_created_ns=None,proposal_state_capture_ns=None,native_trace=row,native_trace_artifact=artifact(trace),timing_missing_reason='frozen native trace exposes duration only',source_time_s=float(row['source_time_from_start_ns'])/1e9)
    f.write(json.dumps(record)+'\n');f.flush()
 configs=list((out/'native').rglob('*config*.json'))
 for path in configs:
  native=json.loads(path.read_text())
  if native.get('schema_version')=='mcl.placo_baseline_config.v1':
   robot=native['robot'];write_json(out/'model_mapping.json',{'model':d['model'],'joint_names':robot['joint_names'],'active_joint_names':robot['active_joint_names'],'root_frame':robot['base_frame'],'frames':{s:robot[s+'_end_effector_frame'] for s in ['left','right']},'tcp_offsets':{s:robot[s+'_tcp_offset_xyz'] for s in ['left','right']},'limits':{'lower':robot['limited_lower'],'upper':robot['limited_upper'],'velocity':robot['velocity_limits']},'native_config_artifact':artifact(path),'units':d['units']})
 write_json(out/'method_semantics.json',{'method_id':'production-static','profile_frozen':True,'actual_native_config_artifacts':[artifact(x) for x in configs],'algorithm_overrides':False,'state_semantics':'production initial state then own accepted evolution; arbitrary canonical q/qdot not injectable','comparison_status':'historical baseline; semantic mismatch to controlled snapshots'})
 write_json(out/'native_status.json',{'exit_code':code,'trace_files':len(traces),'crash_evidence_limitation':'native baseline buffers trace until exit'})
 return code
if __name__=='__main__':raise SystemExit(main())
