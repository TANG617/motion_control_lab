"""Per-unit independent checks, outside the IK timer. Native states remain untouched."""
import pathlib
import numpy as np
from evidence import read_json,write_json
from inputs import UrdfFk
from metrics import position_error,orientation_error,hard_violation
from progress import SampleProgress

def verify_unit(output,canonical,identity=None,config=None,metric_roles=None,on_progress=None):
    output=pathlib.Path(output); raw=output/'raw.jsonl'; rows=[]; failures=[]; metric_rows=[]; result_rows=0
    identity=identity or {}; config=config or {}; metric_roles=metric_roles or {}
    if not raw.exists(): return {'status':'unavailable','reason':'raw.jsonl missing','rows':0}
    import json
    fk=UrdfFk(canonical['model']['locator']) if canonical.get('model') else None
    lines=raw.read_text().splitlines()
    progress=SampleProgress(output,'independent-fk',len(lines),on_progress)
    for line_no,line in enumerate(lines,1):
        progress.update(line_no-1)
        try: r=json.loads(line)
        except ValueError:
            failures.append({'line':line_no,'reason':'incomplete or invalid JSON'});continue
        if r.get('record_type','attempt') in ('attempt','analytic'): result_rows+=1
        if r.get('record_type','attempt')!='attempt': continue
        checks={'line':line_no,'attempt_sequence':r.get('attempt_sequence'),'native_status':r.get('status'),'measurements':[]}
        q=r.get('q'); names=r.get('joint_names',canonical.get('joint_names'))
        if q is None:
            checks['status']='unavailable';checks['reason']='q missing; analytic/app-specific verifier required'
        elif not np.all(np.isfinite(q)):
            checks['status']='failed';failures.append({'line':line_no,'reason':'nonfinite q'})
        elif names!=canonical.get('joint_names') or len(q)!=len(names):
            checks['status']='failed';failures.append({'line':line_no,'reason':'joint mapping mismatch'})
        else:
            checks['status']='passed'
            limits=canonical['limits']; viol=hard_violation(np.asarray(q),limits['lower'],limits['upper'])
            checks['position_limit_violation_rad']=viol.tolist()
            tolerance=r.get('accepted_position_tolerance') or 0.
            if np.any(viol>tolerance):
                checks['status']='failed';failures.append({'line':line_no,'reason':'position hard violation','maximum_rad':float(max(viol))})
            targets=r.get('targets')
            if targets is None:
                sequence=r.get('input_sequence',0); samples=canonical.get('samples',[])
                targets=next((s['targets'] for s in samples if s['sequence']==sequence),None)
            for side,target in (targets or {}).items():
                actual=fk.pose(names,q,canonical['frames'][side],canonical['tcp_offsets'][side],canonical['root_frame'])
                checks['measurements'].append({'side':side,'position_error_m':position_error(target['position'],actual['position']),
                                             'orientation_error_rad':orientation_error(target['rotation'],actual['rotation']),'fk':actual})
        for m in checks['measurements']:
            for name,unit in [('position_error_m','m'),('orientation_error_rad','rad')]:
                metric_rows.append({**identity,'schema_version':'metric_row.v2','metric_version':'study-metrics.v1',
                    'window_id':r.get('window_id','all'),'component_id':m['side'],'sample_id':r.get('attempt_sequence'),
                    'metric_id':name.rsplit('_',1)[0],'value':m[name],'unit':unit,'role':metric_roles.get(name.rsplit('_',1)[0],'diagnostic'),'status':'ok','reduction':'per-sample',
                    'source_locator':str(raw.resolve())+'#line='+str(line_no)})
        rows.append(checks)
    with (output/'independent_checks.jsonl').open('w') as f:
        for r in rows: f.write(json.dumps(r,allow_nan=False)+'\n')
    with (output/'metrics.jsonl').open('w') as f:
        for r in metric_rows: f.write(json.dumps(r,allow_nan=False)+'\n')
    progress.update(len(lines))
    return {'schema_version':'unit_validation.v1','status':'failed' if failures else ('passed' if rows and all(r['status']=='passed' for r in rows) else 'unavailable'),
            'rows':len(rows),'result_rows':result_rows,'failures':failures,'model_dependency':'independent XML URDF tree and NumPy; no candidate FK or diagnostics consumed'}
