"""Public source and supplemental-run contracts for evidence repair v1."""
from pathlib import Path
import copy
import json
import importlib.util
from analysis import common as C

VERSION = 'evidence-repair.v1'
STAGE_COUNTS = {'E06':12, 'E08':56, 'E09-audit':24, 'E09-timing':216}
COMMON_ACCEPTANCE = {
    'schema_version':'servo_acceptance.v1',
    'mode':'ServoStep', 'native_rejection':'never-consumed',
    'finite_candidate_required':True,
    'joint_position_velocity_bounds':'same declared model/active set; velocity units',
    'hard_task_rows':'same declared equation; original hard tolerance',
    'constraint_repair':'forbidden', 'failure_behavior':'native exceptions propagate',
    'claim_scope':'complete named method under common external contract; not isolated backend',
}
COMMON_ACCEPTANCE_HASH = C.stable_hash(COMMON_ACCEPTANCE)










def evidence_class(unit):
    request=unit.get('request') or C.unit_json(unit,'request.json',{})
    inp=request.get('input',{})
    if inp.get('fixture'):
        return 'expected-failure-fixture' if unit.get('case_id')=='fixture-observation-overflow' else 'development-fixture'
    if unit.get('status')=='unavailable':
        if str(unit.get('reason','')).startswith('design admission blocked:'):return 'admission-blocked'
        return 'capability-unavailable'
    if unit.get('status')!='completed' or unit.get('validation',{}).get('status')!='passed':return 'observed-anomaly'
    return 'scientific-case-nonRT'


def verify_unit_binding(ctx,unit,declared,definition_path,frozen_inputs):
    """Reject internally consistent but undeclared replacement inputs/configurations."""
    config=copy.deepcopy(declared['config']);config.update(declared.get('phase_config',{}).get('development',{}))
    if unit['config']!=config or C.stable_hash(config)!=unit['config_hash']:
        raise ValueError('supplement actual configuration differs from frozen declaration')
    if unit['method_id']!=declared['method_id']:raise ValueError('supplement method identity differs')
    path=Path(declared['input'])
    if not path.is_absolute():path=Path(definition_path).parent/path
    descriptor_record=frozen_inputs.get(str(path.resolve()))
    if not descriptor_record:raise ValueError('supplement input descriptor not frozen')
    ctx.check_artifact(descriptor_record);descriptor=C.read_json(path);canonical=descriptor['canonical']
    if frozen_inputs.get(canonical['locator'])!=canonical:raise ValueError('supplement canonical input not frozen')
    ctx.check_artifact(canonical);data=C.read_json(canonical['locator']);model=data.get('model')
    if model:
        if frozen_inputs.get(model['locator'])!=model:raise ValueError('supplement model not frozen')
        ctx.check_artifact(model)
    if unit['input_hash']!=canonical['sha256'] or unit['model_hash']!=(model or {}).get('sha256'):
        raise ValueError('supplement input/model identity differs from frozen declaration')
    request=unit.get('request',{})
    if request:
        if any(request['identity'].get(k)!=unit.get(k) for k in request['identity']):
            raise ValueError('supplement request identity mismatch')
        if C.stable_hash(request['input'])!=C.stable_hash(data) or request['input_descriptor']!=descriptor:
            raise ValueError('supplement request input differs from frozen canonical input')


def load_supplement(ctx):
    """Read a closed repair index and its manifest-owned artifacts into a separate list."""
    d=ctx.definition['supplement'];ctx.check_artifact(d['declaration']);ctx.check_artifact(d['execution_index'])
    declaration=C.read_json(d['declaration']['locator']);index=C.read_json(d['execution_index']['locator'])
    if declaration['schema_version']!='evidence_repair.v1' or index['schema_version']!='repair_execution_index.v1':
        raise ValueError('supplement contract version')
    if declaration['base_campaign'] != ctx.definition['campaign'] or index['declaration'] != d['declaration']:
        raise ValueError('supplement belongs to a different base campaign/declaration')
    if index['status']!='closed':raise ValueError('supplement not closed')
    if declaration['stage_counts']!=STAGE_COUNTS:raise ValueError('supplement matrix changed')
    if ctx.definition.get('repair_source_archive'):
        a=ctx.definition['repair_source_archive'];ctx.check_artifact(a);archive=C.read_json(a['locator'])
        if archive['declaration']!=d['declaration'] or archive['execution_index']!=d['execution_index']:
            raise ValueError('frozen source archive belongs to a different supplement')
        expected_records={record['locator']:record for name in ('source_code','build','inputs') for record in declaration[name]}
        for runtime in declaration['runtime']:
            for record in [runtime['executable']]+runtime['libraries']:expected_records[record['locator']]=record
        actual_records={row['original']['locator']:row['original'] for row in archive['files']}
        if len(actual_records)!=len(archive['files']) or actual_records!=expected_records:
            raise ValueError('frozen source archive incomplete or duplicate')
        for row in archive['files']:
            snapshot=row['snapshot']
            if (snapshot['sha256']!=row['original']['sha256'] or snapshot['size_bytes']!=row['original']['size_bytes'] or
                    not Path(snapshot['locator']).resolve().is_relative_to(Path(a['locator']).resolve().parent)):
                raise ValueError('frozen source snapshot identity mismatch')
            ctx.check_artifact(snapshot)
    wanted={'A01':{'E06','E08'},'A02':{'E09'}}[ctx.definition['analysis_id']]
    frozen_inputs={a['locator']:a for a in declaration['inputs']}
    if len(frozen_inputs)!=len(declaration['inputs']):raise ValueError('duplicate frozen input identity')
    for a in frozen_inputs.values():ctx.check_artifact(a)
    if len(index['runs']) != len(STAGE_COUNTS) or {r['stage'] for r in index['runs']} != set(STAGE_COUNTS):
        raise ValueError('duplicate or missing supplemental stage')
    audit_run=next(r for r in index['runs'] if r['stage']=='E09-audit')
    ctx.check_artifact(audit_run['inventory']);ctx.check_artifact(audit_run['manifest'])
    verifier=C.ROOT/'experiments/E09_solver_cost_and_scalability/verify_evidence.py'
    spec=importlib.util.spec_from_file_location('supplement_timing_verifier',verifier)
    audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    timing_gates=audit.audit_gate(Path(audit_run['run_dir']))
    rows=[];seen=set();counts={k:0 for k in STAGE_COUNTS}
    for run in index['runs']:
        stage=run['stage'];ctx.check_artifact(run['manifest']);ctx.check_artifact(run['inventory'])
        ctx.check_artifact(run['definition']);ctx.check_artifact(declaration['stages'][stage])
        original=C.read_json(declaration['stages'][stage]['locator'])
        resolved=C.read_json(run['definition']['locator'])
        numeric=copy.deepcopy(resolved)
        if stage=='E09-timing':
            expected_timing,blocked=bind_timing(original,timing_gates)
            if resolved!=expected_timing:raise ValueError('timing gate evidence does not belong to indexed audit run')
            for u in numeric['units']:
                for field in ('native_audit_evidence','native_audit_gate_verdict'):u['config'].pop(field,None)
        if numeric!=original:raise ValueError('supplement configuration differs from frozen stage')
        m=C.read_json(run['manifest']['locator']);inv=C.read_json(run['inventory']['locator'])
        root=Path(run['run_dir']).resolve()
        if (run['experiment_id']!=stage[:3] or m.get('experiment_id')!=stage[:3] or
            m.get('run_id')!=run['run_id'] or inv.get('run_id')!=run['run_id'] or
            root.name!=run['run_id'] or any(Path(run[k]['locator']).resolve().parent!=root for k in ('manifest','inventory'))):
            raise ValueError('supplement run identity mismatch')
        if (m['units']!=inv['actual_units'] or inv.get('smoke') or inv.get('phase')!='development' or
                m.get('status') not in ctx.definition['source_policy']['allowed_statuses']):
            raise ValueError('supplement manifest/coverage mismatch')
        declared={}
        for u in resolved['units']:
            repeats=u.get('repeats',resolved['repeats'])
            if isinstance(repeats,dict):repeats=repeats['development']
            for repeat in range(repeats):
                key=C.unit_id(dict(u,experiment_id=stage[:3],repeat_id=repeat))
                if key in declared:raise ValueError('duplicate frozen supplemental identity')
                declared[key]=u
        expected={C.unit_id(u) for u in inv['required_units']}
        if len(expected)!=len(m['units']) or expected!={C.unit_id(u) for u in m['units']} or expected!=set(declared):
            raise ValueError('supplement required units incomplete')
        for a in inv.get('artifacts',[]):ctx.check_artifact(a)
        for field in ('source_inventory','build_inventory'):ctx.check_artifact(inv[field])
        for raw in m['units']:
            u=copy.deepcopy(raw);key=(stage,C.unit_id(u))
            if key in seen:raise ValueError('duplicate supplemental identity')
            seen.add(key);counts[stage]+=1
            if u['experiment_id'] not in wanted:continue
            u['directory']=str(Path(run['run_dir'])/'units'/u['experiment_id']/u['case_id']/u['arm_id']/str(u['repeat_id']))
            for a in u['artifacts']:
                if not Path(a['locator']).resolve().is_relative_to(Path(run['run_dir']).resolve()):raise ValueError('supplement artifact escapes run')
                ctx.check_artifact(a)
            u['request']=C.unit_json(u,'request.json',{});u['config']=u['request'].get('config',{})
            if not u['request']:u['config']=copy.deepcopy(declared[C.unit_id(u)]['config'])
            verify_unit_binding(ctx,u,declared[C.unit_id(u)],run['definition']['locator'],frozen_inputs)
            environment=C.unit_json(u,'environment.json',{})
            if environment:
                runtime=environment.get('native_runtime',environment.get('runtime',{}))
                candidates=[r for r in declaration['runtime'] if r['executable']==runtime.get('executable')]
                if len(candidates)!=1 or candidates[0]['libraries']!=runtime.get('libraries'):
                    raise ValueError('supplement actual native runtime differs from frozen runtime')
            if stage=='E09-timing' and (u['case_id'],u['arm_id']) in blocked and u['status']!='unavailable':
                raise ValueError('admission-blocked timing unit executed')
            u['repair_stage']=stage;u['evidence_class']=evidence_class(u);rows.append(u)
    if counts!=STAGE_COUNTS:raise ValueError('supplement stage counts disagree')
    ctx.supplemental_units=rows
    return counts


def bind_timing(definition,gates):
    """Only the predeclared audit evidence metadata may extend timing configuration."""
    resolved=copy.deepcopy(definition)
    blocked={}
    for u in resolved['units']:
        descriptor=C.read_json(u['input']);key=(descriptor['canonical']['sha256'],u['config']['workload'])
        gate=gates.get(key,{})
        artifacts=gate.get('evidence_artifacts',[])
        for a in artifacts:C.verify(a)
        from repair.common import COMMON_ACCEPTANCE_HASH
        admitted=(gate.get('admitted') is True and len(artifacts)==2 and
                  set(gate.get('method_ids',[]))=={'mcc-weighted-common-servo-v1','placo-common-servo-v1'} and
                  gate.get('acceptance_contract_hash')==COMMON_ACCEPTANCE_HASH)
        u['config']['native_audit_evidence']=artifacts
        u['config']['native_audit_gate_verdict']='admitted' if admitted else 'blocked'
        if not admitted:blocked[(u['case_id'],u['arm_id'])]='design admission blocked: '+json.dumps(gate.get('reasons',['missing matching audit']),ensure_ascii=False)
    return resolved,blocked
