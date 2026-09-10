#!/usr/bin/env python3
"""Create immutable campaign prerequisites and resolved definition; never runs experiments."""
import argparse,pathlib
from evidence import artifact,read_json,stable_hash,validate_definition,write_json
KINDS=('inputs','models','candidates','preregistration','resources','exposure','split')
def freeze(definition,attachments,attestation,output):
    definition=validate_definition(definition)
    if not definition.get('thresholds'):raise ValueError('numeric threshold preregistration required before freeze')
    if any(not attachments.get(k) for k in KINDS):raise ValueError('all campaign prerequisite families required')
    unsigned={k:v for k,v in definition.items() if k not in ('freeze','host_rt_attestation')}
    value={'schema_version':'campaign_freeze.v1','definition_hash':stable_hash(unsigned),
           **{k:[artifact(p) for p in attachments[k]] for k in KINDS}}
    out=pathlib.Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    write_json(out/'campaign_freeze.json',value)
    resolved={**unsigned,'freeze':artifact(out/'campaign_freeze.json'),'host_rt_attestation':artifact(attestation)}
    write_json(out/'definition.frozen.json',resolved)
    return out/'definition.frozen.json'
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--definition',required=True);p.add_argument('--host-attestation',required=True);p.add_argument('--output',required=True)
    for k in KINDS:p.add_argument('--'+k,action='append',required=True)
    a=p.parse_args();print(freeze(read_json(a.definition),{k:getattr(a,k) for k in KINDS},a.host_attestation,a.output))
