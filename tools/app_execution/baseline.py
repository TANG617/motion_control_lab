#!/usr/bin/env python3
"""Mechanical CSV adapter to baseline's frozen public replay CLI; no solver overrides."""
import argparse,csv,pathlib,subprocess
from scipy.spatial.transform import Rotation
from contracts import read,write,validate_request,artifact

def main():
    p=argparse.ArgumentParser();p.add_argument('--request',required=True);p.add_argument('--binary',required=True);a=p.parse_args()
    r=validate_request(read(a.request),'mcl_baseline')
    if r['app_config']!={'profile':'production-static'}:raise ValueError('production-static algorithm overrides forbidden')
    d=read(r['input']['path']);out=pathlib.Path(r['output_dir']);out.mkdir(parents=True,exist_ok=False)
    source=out/'input.csv'
    with source.open('w') as f:
        w=csv.writer(f);w.writerow(['timestamp_ns']+[s+'_'+k for s in ('left','right') for k in ('frame_id','x','y','z','qx','qy','qz','qw')])
        samples=d['samples'][:1] if r['tracking'].get('smoke') else d['samples']
        for i,sample in enumerate(samples):
            row=[round(sample.get('source_time_s',i*.01)*1e9)]
            for side in ('left','right'):
                t=sample['targets'][side];row.extend([d['root_frame'],*t['position'],*Rotation.from_matrix(t['rotation']).as_quat().tolist()])
            w.writerow(row)
    command=[a.binary,'replay','--urdf',d['model']['locator'],'--input',str(source),'--input-format','csv','--left-stream','left','--right-stream','right','--timestamp-source','csv_timestamp','--target-period-ms','10','--execution-mode','batch','--ui','none','--viz','none','--terminal-input','off','--no-mcap','--output-dir',str(out/'native')]
    write(out/'command.json',command);write(out/'execution_request.json',r)
    write(out/'method_semantics.json',{'production_static_frozen':True,'algorithm_overrides':False,'snapshot_state_injection':False,'native_trace_buffered_until_exit':True,'binary':artifact(a.binary)})
    code=subprocess.call(command)
    write(out/'native_status.json',{'exit_code':code,'traces':[artifact(p) for p in sorted((out/'native').rglob('trace.csv'))]})
    return code
if __name__=='__main__':raise SystemExit(main())
