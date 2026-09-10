#!/usr/bin/env python3
"""Verify a frozen experiment evidence inventory, including partial/failed units."""
import argparse,json,pathlib
from evidence import read_json,verify_artifact

def check(path):
    inventory=read_json(path);problems=[]
    if inventory.get('schema_version')!='evidence_inventory.v1':problems.append('unknown evidence inventory schema')
    required={(u['experiment_id'],u['case_id'],u['arm_id'],u['repeat_id']) for u in inventory['required_units']}
    actual={(u['experiment_id'],u['case_id'],u['arm_id'],u['repeat_id']) for u in inventory['actual_units']}
    if required!=actual:problems.append('required/actual identities differ, including missing and not-run units')
    artifacts=list(inventory.get('artifacts',[]))+[inventory['source_inventory']]
    if inventory.get('build_inventory'):artifacts.append(inventory['build_inventory'])
    for unit in inventory['actual_units']:
        artifacts.extend(unit.get('artifacts',[]))
        if unit['status']!='completed' and not unit.get('reason'):problems.append('noncomplete unit missing reason')
    for a in artifacts:
        if not verify_artifact(a):problems.append('missing or tampered: '+a['locator'])
    return {'status':'failed' if problems else 'passed','checked_artifacts':len(artifacts),'unit_count':len(actual),'problems':problems}
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('inventory');a=p.parse_args();r=check(a.inventory);print(json.dumps(r,indent=2));raise SystemExit(r['status']!='passed')
