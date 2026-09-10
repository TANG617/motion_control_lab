#!/usr/bin/env python3
"""Mechanical process adapter around the unchanged production-static executable."""
import argparse,csv,json,pathlib,subprocess,sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from evidence import write_json

def run(request,binary):
 inp=request['input'];out=pathlib.Path(request['output_dir']);samples=inp['samples'][:request['config'].get('maximum_samples',len(inp['samples']))];path=out/'paired.csv'
 fields=['timestamp_ns']+[s+'_'+k for s in ('left','right') for k in ['frame_id','x','y','z','qx','qy','qz','qw']]
 with path.open('w') as f:
  w=csv.DictWriter(f,fields);w.writeheader()
  for sample in samples:
   row={'timestamp_ns':round(sample['source_time_s']*1e9)}
   for s in ('left','right'):
    pose=sample['targets'][s];row[s+'_frame_id']=inp['root_frame'];row.update({s+'_'+k:v for k,v in zip(['x','y','z','qx','qy','qz','qw'],pose['position']+Rotation.from_matrix(pose['rotation']).as_quat().tolist())})
   w.writerow(row)
 command=[binary,'replay','--urdf',inp['model']['locator'],'--input',str(path),'--input-format','csv','--left-stream','left','--right-stream','right','--timestamp-source','csv_timestamp','--target-period-ms','10','--execution-mode','batch','--ui','none','--viz','none','--terminal-input','off','--no-mcap','--output-dir',str(out/'native')]
 write_json(out/'baseline_command.json',command);proc=subprocess.run(command)
 trace=out/'native/trace.csv'
 with (out/'raw.jsonl').open('w') as f:
  if trace.exists():
   for i,r in enumerate(csv.DictReader(trace.open())):
    sample=samples[min(i,len(samples)-1)];row=dict(record_type='attempt',attempt_sequence=i,input_sequence=sample['sequence'],state_sequence=i,task_revision=sample['sequence'],joint_names=inp['joint_names'],q=[float(x) for x in r['positions'].split(';')],v=[float(x) for x in r['velocities'].split(';')],targets=sample['targets'],status=r['solver_status'],native_qp_status=None,native_qp_status_reason='not exposed in baseline public trace',disposition='accepted' if r['accepted']=='true' else 'rejected',committed=r['accepted']=='true',committed_sequence=i,execution_state='committed',solution_quality=r['solver_status'],requested_passes=None,completed_passes=None,selected_priority=None,highest_completed_priority=None,release_ns=None,start_ns=None,finish_ns=None,proposal_revision=None,proposal_created_ns=None,proposal_state_capture_ns=None,timing_missing_reason='historical public trace contains solve_time_ms only',solve_time_ms=float(r['solve_time_ms']),accepted_position_tolerance=None,accepted_position_tolerance_reason='frozen baseline accepts returned best effort; not controlled parity')
    f.write(json.dumps(row)+'\n');f.flush()
 write_json(out/'native_status.json',dict(exit_code=proc.returncode,comparison='historical-only; production-static original task mask and 100 Hz preserved'));return proc.returncode
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--request',required=True);p.add_argument('--binary',required=True);a=p.parse_args();sys.exit(run(json.loads(pathlib.Path(a.request).read_text()),a.binary))
