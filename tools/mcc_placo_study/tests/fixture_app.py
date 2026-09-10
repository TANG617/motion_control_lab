"""MOCK FIXTURE ONLY: subprocess contract, never a solver experiment."""
import argparse,json,pathlib,sys,time
p=argparse.ArgumentParser();p.add_argument('--request');a=p.parse_args();r=json.loads(pathlib.Path(a.request).read_text())
out=pathlib.Path(r['output_dir']);i=r['input']
with (out/'raw.jsonl').open('w') as f:
    f.write(json.dumps({'record_type':'attempt','attempt_sequence':0,'input_sequence':0,'status':'fixture-ok','q':i['initial_state']['q'],'joint_names':i['joint_names']})+'\n')
if r['config'].get('sleep'):time.sleep(30)
if r['config'].get('fail'):print('native fixture failure',file=sys.stderr);sys.exit(7)
