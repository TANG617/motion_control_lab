"""Prerequisite integrity checks; freezing does not execute or approve a campaign."""
import pathlib
from evidence import read_json,stable_hash,verify_artifact

def frozen_prerequisites(definition,env):
    reasons=[]; ref=definition.get('freeze')
    if not isinstance(ref,dict) or 'locator' not in ref or not verify_artifact(ref):
        return ['valid immutable freeze artifact missing or hash mismatch']
    f=read_json(ref['locator'])
    if f.get('schema_version')!='campaign_freeze.v1':reasons.append('unsupported campaign freeze version')
    unsigned={k:v for k,v in definition.items() if k not in ('freeze','host_rt_attestation')}
    if f.get('definition_hash')!=stable_hash(unsigned):reasons.append('definition changed after freeze')
    for k in ('inputs','models','candidates','preregistration','resources','exposure','split'):
        refs=f.get(k,[])
        if not refs:reasons.append('frozen '+k+' missing')
        for r in refs:
            if not verify_artifact(r):reasons.append('frozen '+k+' content missing or hash mismatch')
    att=definition.get('host_rt_attestation')
    if not isinstance(att,dict) or 'locator' not in att or not verify_artifact(att):reasons.append('host RT attestation artifact missing')
    else:
        a=read_json(att['locator'])
        if a.get('boot_id')!=env.get('boot_id'):reasons.append('host attestation boot mismatch')
        for k in ('preempt_rt_confirmed','physical_cores_same_type','frequency_policy_frozen','serial_benchmark_confirmed'):
            if a.get(k) is not True:reasons.append('host attestation lacks '+k)
        if not a.get('cpu_ids') or not set(a['cpu_ids'])<=set(env['affinity']):reasons.append('frozen cores outside effective affinity')
    thresholds=definition.get('thresholds')
    if not isinstance(thresholds,dict) or not thresholds:reasons.append('preregistered thresholds missing')
    else:
        for name,t in thresholds.items():
            if not isinstance(t,dict) or not isinstance(t.get('value'),(float,int)) or not t.get('unit') or not t.get('source'):
                reasons.append('threshold missing numeric value/unit/source: '+name)
    return reasons
