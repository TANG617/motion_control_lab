#!/usr/bin/env python3
"""Independent schedule accounting and age checks on one executed unit."""
import argparse,json,pathlib,sys,math
LAB=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(LAB/'tools/mcc_placo_study'))
from evidence import write_json

def virtual_reference(duration_ns,period_ns,secondary_period_ns,primary_cost_ns,secondary_cost_ns,pause=None):
    """Hand-computable synchronous oracle: secondary executes before primary at divisible releases."""
    rows=[];now=0;revision=0;created=None;state_capture=None
    for seq,release in enumerate(range(0,duration_ns,period_ns)):
        now=max(now,release)
        if release%secondary_period_ns==0 and not(pause and pause[0]<=release<pause[1]):
            state_capture=release;now+=secondary_cost_ns;created=now;revision+=1
        start=now;skipped=start>=release+period_ns
        if not skipped:now+=primary_cost_ns
        rows.append(dict(sequence=seq,release_ns=release,start_ns=None if skipped else start,finish_ns=None if skipped else now,
            skipped=skipped,deadline_miss=skipped or now>release+period_ns,proposal_revision=revision,
            proposal_age_ns=None if created is None else start-created,state_age_ns=None if state_capture is None else start-state_capture))
    return rows

def check(events,attempts,config,native,initial_state=None):
    failures=[];duration=round(config['duration_s']*1e9);counts={};misses={}
    for worker,rate in [('primary','solver_hz'),('secondary','secondary_hz'),('target','target_hz'),('feedback-capture','feedback_hz'),('output','output_hz')]:
        period=round(1e9/config[rate]);expected=math.ceil(duration/period);rows=[e for e in events if e.get('worker')==worker];counts[worker]={'planned':expected,'retained':len(rows)}
        if len(rows)!=expected:failures.append(worker+'-release-coverage')
        if [r['sequence'] for r in rows]!=list(range(len(rows))):failures.append(worker+'-sequence-gap')
        for r in rows:
            if r['release_ns']!=r['sequence']*period:failures.append(worker+'-release-grid')
            miss=r['skipped'] or r['finish_ns']>r['deadline_ns']
            if miss!=r['deadline_miss']:failures.append(worker+'-miss-classification')
        misses[worker]=sum(r['deadline_miss'] for r in rows)
    previous_revision=0
    for r in attempts:
        if r.get('worker')!='primary':continue
        if 'attempt_sequence' not in r:
            failures.append('primary-missing-attempt_sequence');continue
        if r['proposal_created_ns'] is not None:
            if r['proposal_age_ns']!=r['start_ns']-r['proposal_created_ns'] or r['proposal_age_ns']<0:failures.append('proposal-age')
            if r['proposal_state_age_ns']!=r['start_ns']-r['proposal_state_capture_ns']:failures.append('proposal-state-age')
        if r['measured_state_age_ns']!=r['start_ns']-r['measured_state_capture_ns'] or r['measured_state_age_ns']<0:failures.append('feedback-age')
        if not config['coupling_enabled'] and r['coupling_enabled']:failures.append('disabled-proposal-consumed')
        publications=[e for e in events if e.get('worker')=='proposal-publication' and e['revision']==r['proposal_revision'] and e.get('publication_ns',e['delivery_ns'])<=r['start_ns']]
        if r['proposal_revision'] and not publications:failures.append('consumed-publication-missing')
        if 'proposal_repeated' in r and r['proposal_repeated']!=(r['proposal_revision']!=0 and r['proposal_revision']==previous_revision):failures.append('repeated-version-label')
        if 'proposal_stale_version' in r and r['proposal_stale_version']!=(r['proposal_revision']<previous_revision):failures.append('old-version-label')
        previous_revision=r['proposal_revision']
        if 'measured_q' in r:
            captures=[e for e in events if e.get('worker')=='feedback-capture' and e.get('sequence')==r['state_sequence'] and e.get('capture_ns')==r['measured_state_capture_ns']]
            if captures:
                capture=captures[-1]
                if capture.get('q')!=r['measured_q'] or capture.get('captured_committed_sequence')!=r['measured_state_committed_sequence']:failures.append('feedback-state-join')
            elif r['measured_state_capture_ns']!=0:failures.append('feedback-capture-missing')
            elif initial_state is not None and r['measured_q']!=initial_state['q']:failures.append('initial-feedback-mismatch')
        if publications and not publications[-1]['accepted'] and r['coupling_enabled']:failures.append('failed-proposal-consumed')
    previous_output=initial_state['q'] if initial_state is not None else None
    primary_by_id={r['attempt_sequence']:r for r in attempts if r.get('worker')=='primary' and 'attempt_sequence' in r}
    for e in events:
        if e.get('worker')!='output' or e.get('skipped'):continue
        if e.get('committed'):
            candidate=primary_by_id.get(e.get('command_attempt_sequence'))
            if candidate is None or candidate.get('disposition')!='accepted' or candidate['q']!=e['q']:failures.append('output-candidate-join')
        elif previous_output is not None and e.get('q')!=previous_output:failures.append('hold-position-changed')
        previous_output=e.get('q')
    if native['timing_buffer_overflow']:failures.append('observation-overflow-tail-unavailable')
    return dict(schema_version='e10.validation.v1',passed=not failures,failures=sorted(set(failures)),release_counts=counts,deadline_misses=misses,
       denominator='all planned releases including skipped',timing_claim='virtual semantics only' if config['schedule_mode']=='virtual' else 'bounded development observation, no formal timing claim')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--request',required=True);a=ap.parse_args();req=json.load(open(a.request));out=pathlib.Path(req['output_dir'])
    events=[json.loads(l) for l in (out/'worker_timeline.jsonl').read_text().splitlines()];attempts=[r for l in (out/'raw.jsonl').read_text().splitlines() if (r:=json.loads(l)).get('record_type')=='attempt']
    result=check(events,attempts,req['config'],json.load(open(out/'native_status.json')),req['input']['initial_state']);write_json(out/'schedule_validation.json',result);sys.exit(not result['passed'])
