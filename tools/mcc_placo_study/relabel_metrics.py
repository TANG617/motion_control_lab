#!/usr/bin/env python3
"""Apply declared per-experiment metric roles into NEW evidence; no value recomputation."""
import argparse,json,pathlib
from evidence import artifact,read_json,write_json

def relabel(definition_path,run_root,output):
    definition=read_json(definition_path);roles=definition.get('metric_roles',{})
    if not roles:raise ValueError('explicit metric_roles required')
    if any(role not in ('primary','secondary','guardrail','diagnostic') for role in roles.values()):raise ValueError('invalid declared role')
    output=pathlib.Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    run=pathlib.Path(run_root).resolve();records=[]
    for source in sorted(run.glob('units/'+definition['experiment_id']+'/*/*/*/metrics.jsonl')):
        request=read_json(source.with_name('request.json'));target=output/source.relative_to(run);target.parent.mkdir(parents=True,exist_ok=True)
        count=0
        with target.open('x') as stream:
            for line in source.read_text().splitlines():
                row=json.loads(line)
                if row['metric_id'] not in roles:raise ValueError('undeclared metric role: '+row['metric_id'])
                row['role']=roles[row['metric_id']]
                stream.write(json.dumps(row,allow_nan=False,sort_keys=True)+'\n');count+=1
        records.append({'identity':request['identity'],'source':artifact(source),'corrected':artifact(target),'rows':count})
    if not records:raise ValueError('no metric files selected')
    result={'schema_version':'metric_role_correction.v1','operation':'only role field changed; values, clocks, identities and raw source locators retained','definition':artifact(definition_path),'metric_roles':roles,'records':records,'analysis':'deferred'}
    write_json(output/'revalidation.json',result);return result
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--definition',required=True);p.add_argument('--run',required=True);p.add_argument('--output',required=True);a=p.parse_args();r=relabel(a.definition,a.run,a.output);print(json.dumps({'units':len(r['records']),'rows':sum(x['rows'] for x in r['records'])}))
