#!/usr/bin/env python3
"""A02: fixed-source descriptive cost and scheduling analysis; never launches experiments."""
from __future__ import annotations
import collections
import csv
import itertools
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'tools/mcc_placo_study'))
from metrics import nearest_rank
from analysis.common import (base_row, cli, compatible, csv_rows, jsonl_rows, plotting,
    read_json, save_figure as _save_figure, stable_hash, unit_json, unit_path, write_json)

WORKERS = {'primary':'solver_hz', 'secondary':'secondary_hz', 'target':'target_hz',
           'feedback-capture':'feedback_hz', 'output':'output_hz'}
METHOD_COLORS={'production-static':'#777777','placo-controlled':'#D55E00','mcc-controlled':'#0072B2','mcc-weighted':'#0072B2','mcc-hqp2':'#009E73','mcc-hqp3':'#CC79A7'}

DIMENSIONS = ('active_dof','native_variable_count','native_free_variables','native_slack_variables',
              'task_position_rows','task_orientation_rows','task_posture_rows',
              'physical_position_bound_components','physical_velocity_bound_components')


def save_figure(ctx, fig, figure_id, name, sources, description, fields=None, selection=None):
    if ctx.definition.get('schema_version')=='analysis.v2':
        sources=list(sources)+['evaluation/base_evidence_classification.csv']
        selection=dict(selection or {},evidence_filter='E10 scientific timing excludes fixtures/capability/unresolved anomalies; statuses retained')
    return _save_figure(ctx,fig,figure_id,name,sources,description,fields,selection)


def quantiles(values):
    a = sorted(v for v in values if v is not None and math.isfinite(v))
    return {'samples':len(a), **{k:(nearest_rank(a,p))
        for k,p in [('p50',.5),('p95',.95),('p99',.99),('max',1)]}}


def ratio(n, d):
    return n/d if d else None


def quality_summary(rows, denominator):
    """Native quality labels are exhaustive, distinct from independent guardrail evidence."""
    counts=collections.Counter(r.get('solution_quality') or 'unknown' for r in rows)
    if sum(counts.values())!=len(rows):raise ValueError('native quality denominator mismatch')
    independent=collections.Counter(r.get('independent_status') or 'unavailable' for r in rows)
    if sum(independent.values())!=len(rows):raise ValueError('independent status denominator mismatch')
    return dict(native_quality_counts=dict(sorted(counts.items())),native_quality_total=len(rows),
        native_full_quality=counts['full'],native_full_quality_coverage=ratio(counts['full'],denominator),
        native_partial_hierarchy=counts['partial-hierarchy'],native_feasible_suboptimal=counts['feasible-suboptimal'],
        native_rejected=counts['rejected'],native_unknown=counts['unknown'],
        quality_provenance='raw.solution_quality native classification; not an independent task/guardrail quality guarantee',
        independent_check_status_counts=dict(sorted(independent.items())),
        independent_guardrail_checks_passed=independent['passed'],
        independent_guardrail_checks_passed_fraction=ratio(independent['passed'],denominator),
        independent_full_task_quality=None,
        independent_full_task_quality_reason='Independent checks and TCP errors retained separately; native full is not task attainment')


def ecdf_summary(rows, timing):
    if any(not r['all_calls_retained'] for r in timing):return []
    output=[]
    for window in ('cold','warmup','steady'):
        values=[r['ik_call_time_ms'] for r in rows if r['window_id']==window]
        if any(v is None or not math.isfinite(v) for v in values):return []
        for percentile in range(101):
            if values:output.append(dict(window=window,percentile=percentile,
                ik_call_time_ms=min(values) if percentile==0 else nearest_rank(values,percentile/100),
                samples=len(values),display='fixed percentile grid; all raw calls; invalid units excluded with status retained'))
    return output


def timing_summary(rows, config):
    """Full denominator, explicit windows; no quality filtering or averaging quantiles."""
    buckets = {w:[] for w in ('cold','warmup','steady')}; quality=collections.Counter(); errors=[]
    expected=1+config['warmup_calls']+config['measured_calls']; total=0
    for i,r in enumerate(rows):
        total+=1; w='cold' if i==0 else 'warmup' if i<=config['warmup_calls'] else 'steady'
        if r.get('attempt_sequence')!=i: errors.append('sequence-gap')
        if r.get('window_id')!=w: errors.append('window-boundary')
        duration=(r['finish_ns']-r['start_ns'])*1e-6
        if duration < 0: errors.append('negative-duration')
        if not math.isfinite(duration) or not isinstance(r.get('ik_call_time_ms'),(float,int)) or not math.isfinite(r['ik_call_time_ms']):errors.append('missing-or-nonfinite-timing')
        if r.get('ik_call_time_ms') is None or abs(duration-r['ik_call_time_ms'])>1e-12: errors.append('duration-mismatch')
        buckets[w].append(r);quality[r.get('solution_quality','unknown')]+=1
    if total!=expected:errors.append('incomplete-attempt-denominator')
    out=[]
    for w, rs in buckets.items():
        q=quantiles([r.get('ik_call_time_ms') for r in rs]); counts=collections.Counter(r.get('solution_quality','unknown') for r in rs)
        n=1 if w=='cold' else config['warmup_calls'] if w=='warmup' else config['measured_calls']
        row=dict(window=w,expected_attempts=n,observed_attempts=len(rs),**quality_summary(rs,n),all_calls_retained=not errors,
            timing_status='unavailable' if errors else 'observed-nonRT',problems=sorted(set(errors)),
            cpu_time_s=sum(r.get('cpu_time_s') or 0 for r in rs),cpu_samples=sum(r.get('cpu_time_s') is not None for r in rs))
        row.update({k+'_ms':v for k,v in q.items() if k!='samples'})
        if errors:
            for k in ('p50_ms','p95_ms','p99_ms','max_ms'): row[k]=None
        out.append(row)
    return out


def release_summary(events, config, native, planned_rows=None):
    """Missing suffix is not-run; missing interior/overflow is unknown, never success."""
    rows=[]; duration=round(config['duration_s']*1e9); overflow=bool(native.get('timing_buffer_overflow'))
    for worker, rate in WORKERS.items():
        period=round(1e9/config[rate]); expected=math.ceil(duration/period)
        declared=([r for r in planned_rows if r['worker']==worker] if planned_rows is not None else
            [dict(worker=worker,sequence=i,release_ns=i*period,deadline_ns=(i+1)*period) for i in range(expected)])
        grid={int(r['sequence']):r for r in declared}; planned=len(declared)
        plan_errors=int(len(grid)!=planned or sorted(grid)!=list(range(expected)))
        plan_errors+=sum(int(r['release_ns'])!=int(r['sequence'])*period or int(r['deadline_ns'])!=(int(r['sequence'])+1)*period for r in declared)
        observed={}; duplicate=0; bad_grid=0; invalid_intervals=0
        for e in events:
            if e.get('worker')!=worker:continue
            seq=e['sequence'];duplicate+=seq in observed;observed[seq]=e
            if not e.get('skipped') and (not isinstance(e.get('start_ns'),(int,float)) or not isinstance(e.get('finish_ns'),(int,float)) or e['finish_ns']<e['start_ns']):invalid_intervals+=1
            bad_grid+=seq not in grid or e.get('release_ns')!=int(grid.get(seq,{}).get('release_ns',-1)) or e.get('deadline_ns')!=int(grid.get(seq,{}).get('deadline_ns',-1))
        last=max(observed,default=-1); misses=skips=longest=current=missing=notrun=0; durations=[]; latencies=[]; release_finishes=[]
        for seq in range(planned):
            e=observed.get(seq)
            if e is None:
                if seq>last and not overflow and native.get('operation') not in ('completed',None):notrun+=1
                else:missing+=1
                current=0;continue
            skip=bool(e.get('skipped'));skips+=skip
            miss=skip or (e.get('finish_ns') is not None and e['finish_ns']>e['deadline_ns'])
            misses+=miss;current=current+1 if miss else 0;longest=max(longest,current)
            if not skip and e.get('start_ns') is not None and e.get('finish_ns') is not None:
                durations.append((e['finish_ns']-e['start_ns'])*1e-6)
                latencies.append((e['start_ns']-e['release_ns'])*1e-6)
                release_finishes.append((e['finish_ns']-e['release_ns'])*1e-6)
        complete=not (overflow or missing or notrun or duplicate or bad_grid or plan_errors or invalid_intervals)
        q=quantiles(durations); lag=quantiles(latencies); release_finish=quantiles(release_finishes)
        rows.append(dict(worker=worker,planned_releases=planned,retained_releases=len(observed),
            skipped_releases=skips,observed_deadline_misses=misses,deadline_miss_fraction=ratio(misses,planned) if complete else None,
            deadline_miss_fraction_lower_bound=ratio(misses,planned),deadline_miss_fraction_status='exact' if complete else 'lower-bound-only',
            denominator='all planned releases',planned_source='planned_releases.csv' if planned_rows is not None else 'fixture config oracle',plan_errors=plan_errors,unknown_releases=missing,not_run_releases=notrun,
            longest_observed_miss_run=longest,longest_miss_status='exact' if complete else 'lower-bound',
            window_completed=complete,overflow=overflow,duplicate_releases=duplicate,grid_errors=bad_grid,invalid_intervals=invalid_intervals,
            observed_until_s=(max((e.get('finish_ns') or e.get('release_ns',0) for e in observed.values()),default=0))*1e-9,
            tail_status='observed-nonRT' if complete else 'unavailable',
            duration_p50_ms=q['p50'] if complete else None,duration_p95_ms=q['p95'] if complete else None,
            duration_p99_ms=q['p99'] if complete else None,observed_duration_max_ms=q['max'],
            start_lag_p99_ms=lag['p99'] if complete else None,
            **{'release_to_finish_'+k+'_ms':release_finish[k] if complete else None for k in ('p50','p95','p99','max')}))
    return rows


def resource_identity(config, summary, environment):
    platform=environment.get('platform',{})
    cpus=[summary.get('effective_primary_cpu'),summary.get('effective_secondary_cpu')]
    return dict(schedule_mode=summary.get('schedule_mode',config.get('schedule_mode','snapshot')),
        scheduler=config.get('scheduler','snapshot'),requested_cpu_ids=config.get('cpu_ids'),
        effective_primary_cpu=cpus[0],effective_secondary_cpu=cpus[1],
        effective_cpu_count=len(set(c for c in cpus if c is not None)) if any(c is not None for c in cpus) else None,
        thread_count=summary.get('thread_count'),boot_id=platform.get('boot_id'),
        core_topology=platform.get('cores'),affinity=platform.get('affinity'),
        kernel=platform.get('kernel',platform.get('uname')),os_scheduler=platform.get('scheduler'),
        realtime_flag=platform.get('realtime_flag'),cgroup=platform.get('cgroup'),
        affinity_evidence='native effective CPU selection; independent thread affinity trace unavailable',rt_claim=False,
        timing_interpretation='virtual-semantics' if config.get('schedule_mode')=='virtual' else 'nonRT-development',
        resource_source='resource_summary.json' if summary else 'unavailable; arm name is not evidence')


def compact_display(rows, fields, bins=400):
    """Only display reduction: min/max per fixed bin, boundaries, every event transition."""
    if len(rows)<=bins:return rows
    keep={0,len(rows)-1};size=max(1,math.ceil(len(rows)/bins))
    for start in range(0,len(rows),size):
        indices=range(start,min(start+size,len(rows)))
        for f in fields:
            good=[i for i in indices if isinstance(rows[i].get(f),(float,int))]
            if good:keep.update((min(good,key=lambda i:rows[i][f]),max(good,key=lambda i:rows[i][f])))
    discrete=('solution_quality','proposal_revision','proposal_stale_version','proposal_repeated','coupling_enabled','skipped','deadline_miss')
    for i in range(1,len(rows)):
        if any(rows[i].get(k)!=rows[i-1].get(k) for k in discrete):keep.update((i-1,i))
    return [rows[i] for i in sorted(keep)]


def provenance(unit):
    c=unit['config']; return dict(base_row(unit),backend=c.get('backend'),warm_start=c.get('warm_start'),
        observation=c.get('observation'),mode=c.get('mode'),layers=c.get('layers'),workload=c.get('workload'),
        scheduler=c.get('scheduler'),schedule_mode=c.get('schedule_mode','snapshot'),
        secondary_hz=c.get('secondary_hz'),coupling_enabled=c.get('coupling_enabled'),
        load=c.get('load'),injection=c.get('injection'),feedback_model=c.get('feedback_model'),
        config_hash=stable_hash(c),historical_baseline=unit['method_id']=='production-static')


def check_errors(unit):
    path=unit_path(unit,'independent_checks.jsonl'); errors={}
    if path:
        for r in jsonl_rows(path):
            m=r.get('measurements',[])
            errors[r['attempt_sequence']]={
                'position_error_m':max((x['position_error_m'] for x in m),default=None),
                'orientation_error_rad':max((x['orientation_error_rad'] for x in m),default=None),
                'independent_status':r.get('status')}
    return errors


def pairing(ctx, summaries, resources, admission_rows=()):
    rows=[]; bycase=collections.defaultdict(list)
    for u in ctx.units: bycase[(u['experiment_id'],u['case_id'],u['repeat_id'])].append(u)
    for group in bycase.values():
        for l,r in itertools.combinations(sorted(group,key=lambda x:x['arm_id']),2):
            changes=[k for k in sorted(set(l['config'])|set(r['config'])) if l['config'].get(k)!=r['config'].get(k)]
            if not changes:continue
            factors=[f for f,keys in ctx.definition['pairing']['allowed_factors'].items() if set(changes)<=set(keys)]
            if not factors:continue
            factor=factors[0];reasons=compatible(l,r,ctx.definition['pairing']['allowed_factors'][factor])
            for policy in admission_rows:
                if policy.get('scope')=='method-policy' and {policy.get('left_method_id'),policy.get('right_method_id')}=={l['method_id'],r['method_id']} and policy.get('admitted')!='True':
                    reasons.append('pinned-A01-method-policy:'+policy.get('reasons','denied'))
            left=provenance(l);right=provenance(r);li=left['unit_id'];ri=right['unit_id']
            lr=resources.get(li,{});rr=resources.get(ri,{})
            for k in ('boot_id','schedule_mode','timing_interpretation'):
                if lr.get(k)!=rr.get(k):reasons.append('environment:'+k)
            resource_changed=any(lr.get(k)!=rr.get(k) for k in ('effective_primary_cpu','effective_secondary_cpu','thread_count'))
            if resource_changed and factor!='scheduler':reasons.append('actual-resource-mismatch')
            a=summaries.get(li,{});b=summaries.get(ri,{})
            if not a or not b:reasons.append('timing-unavailable')
            if not a.get('p50_ms') or not b.get('p50_ms'):reasons.append('timing-unavailable')
            rows.append(dict(experiment_id=l['experiment_id'],case_id=l['case_id'],repeat_id=l['repeat_id'],
                baseline=li,candidate=ri,factor=factor,changed_fields=changes,admitted=not reasons,reasons=sorted(set(reasons)),
                interpretation='resource-change descriptive comparison' if resource_changed else 'within-condition descriptive comparison',
                resource_changed=resource_changed,baseline_resources=lr,candidate_resources=rr,
                baseline_native_full_quality_coverage=a.get('native_full_quality_coverage'),candidate_native_full_quality_coverage=b.get('native_full_quality_coverage'),
                baseline_cpu_time_s=a.get('cpu_time_s'),candidate_cpu_time_s=b.get('cpu_time_s'),
                baseline_state_age_p99_ns=a.get('proposal_state_age_ns_p99'),candidate_state_age_p99_ns=b.get('proposal_state_age_ns_p99'),
                baseline_position_error_p99_m=a.get('position_error_m_p99'),candidate_position_error_p99_m=b.get('position_error_m_p99'),
                median_time_ratio=ratio(a.get('p50_ms'),b.get('p50_ms')) if not reasons else None,
                a01_admission_policy='cross-MCC/PlaCo blocked; production-static historical',
                causal_claim=False))
    return rows


def analyze_base(ctx):
    admission=ctx.definition.get('admission_source')
    if not admission:raise ValueError('A02 requires pinned A01 parity_table and pair_admission')
    admission_rows=[]
    for key in ('parity_table','pair_admission'):
        rows=list(csv_rows(admission[key]['locator']))
        if not rows:raise ValueError('empty A01 '+key)
        ctx.table('a01_'+key,rows)
        if key=='pair_admission':admission_rows=rows
    ecdf=[];timing=[];dimensions=[];deadlines=[];trades=[];platforms=[];timelines=[];costs=[];quality=[]
    summary_index={};resource_index={};families={};family_by_unit={}
    for u in ctx.units:
        req=unit_json(u,'request.json',{});family=req.get('input',{}).get('input_id','unavailable')
        family_by_unit[base_row(u)['unit_id']]=family
        if u['repeat_id']==0:
            key=(u['experiment_id'],family);families[key]=min(families.get(key,u['case_id']),u['case_id'])
    for index,u in enumerate(sorted(ctx.units,key=lambda x:base_row(x)['unit_id'])):
        b=provenance(u);uid=b['unit_id'];c=u['config'];raw=unit_path(u,'raw.jsonl')
        env=unit_json(u,'environment.json',{});res=unit_json(u,'resource_summary.json',{})
        resource=resource_identity(c,res,env);resource_index[uid]=resource
        platforms.append(dict(b,**resource,worker_cpu_time_s=res.get('worker_cpu_time_s'),
            wall_duration_s=res.get('wall_duration_s'),runtime=env.get('runtime'),
            native_dependencies=env.get('native_runtime'),timing_boundaries=unit_json(u,'timing_boundaries.json')))
        if not raw:continue
        family=family_by_unit[uid];representative=u['repeat_id']==0 and u['case_id']==families[(u['experiment_id'],family)]
        # Only compact scalar fields are retained per unit. Raw vectors/Jacobians are discarded immediately.
        attempts=[];dims=collections.Counter();decomposition=collections.defaultdict(list)
        for r in jsonl_rows(raw):
            if r.get('record_type')!='attempt':continue
            if u['experiment_id']=='E10' and r.get('worker')!='primary':continue
            small={k:r.get(k) for k in ('attempt_sequence','window_id','start_ns','finish_ns','release_ns',
                'ik_call_time_ms','cpu_time_s','solution_quality','source_time_s','proposal_age_ns',
                'proposal_state_age_ns','proposal_created_ns','proposal_state_capture_ns','measured_state_age_ns','proposal_revision','proposal_stale_version',
                'proposal_repeated','coupling_enabled')}
            if u['experiment_id']=='E10':small['ik_call_time_ms']=(r['finish_ns']-r['start_ns'])*1e-6
            attempts.append(small)
            dims[tuple(r.get(k) for k in DIMENSIONS)]+=1
            for k in ('assembly_time_ms','allocation_count','allocation_bytes'):
                if r.get(k) is not None:decomposition[k].append(r[k])
            passes=r.get('passes',[])
            for k in ('qp_time_ms','iterations'):
                values=[p[k] for p in passes if p.get(k) is not None]
                if values:decomposition['native_pass_'+k].append(sum(values))
        errors=check_errors(u)
        for r in attempts:r.update(errors.get(r['attempt_sequence'],{}))
        for dim,n in dims.items():dimensions.append(dict(b,**dict(zip(DIMENSIONS,dim)),attempts=n,
            matrix_dimension_interpretation='native reported fields only; null is unavailable, not zero'))
        for k,values in decomposition.items():costs.append(dict(b,metric=k,**quantiles(values),denominator=len(attempts),
            interpretation='native diagnostics; nested pass sum is not comparable standalone assembly cost'))
        for metric in ('position_error_m','orientation_error_rad'):
            quality.append(dict(b,metric=metric,**quantiles([r.get(metric) for r in attempts]),
                denominator=len(attempts),aggregation='maximum across measured end effectors per attempt'))
        if u['experiment_id']=='E09':
            ts=timing_summary(attempts,c)
            ecdf.extend(dict(b,**r) for r in ecdf_summary(attempts,ts))
            for row in ts:timing.append(dict(b,**row))
            summary_index[uid]=next(x for x in ts if x['window']=='steady')
        else:
            events_path=unit_path(u,'worker_timeline.jsonl'); events=[]
            if events_path:
                for event in jsonl_rows(events_path):
                    if event.get('worker') in WORKERS:events.append({k:event.get(k) for k in ('worker','sequence','release_ns','start_ns','finish_ns','deadline_ns','skipped','deadline_miss')})
            native=unit_json(u,'native_status.json',{})
            planned_path=unit_path(u,'planned_releases.csv')
            if not planned_path:raise ValueError('executed E10 missing declared planned_releases.csv: '+uid)
            ds=release_summary(events,c,native,list(csv_rows(planned_path)))
            deadlines.extend(dict(b,**r) for r in ds)
            full=collections.Counter(r.get('solution_quality','unknown') for r in attempts)
            t=quantiles([r['ik_call_time_ms'] for r in attempts]); primary=next(r for r in ds if r['worker']=='primary')
            tr=dict(b,**resource,planned_releases=primary['planned_releases'],observed_attempts=len(attempts),
                **quality_summary(attempts,primary['planned_releases']),
                cpu_time_s=res.get('worker_cpu_time_s'),wall_duration_s=res.get('wall_duration_s'),
                cpu_fraction=ratio(res.get('worker_cpu_time_s'),res.get('wall_duration_s')) if res.get('worker_cpu_time_s') is not None else None,
                windows=c.get('windows'),tail_status=primary['tail_status'],
                p50_ms=t['p50'] if primary['window_completed'] else None,
                p99_ms=t['p99'] if primary['window_completed'] else None)
            for f in ('proposal_age_ns','proposal_state_age_ns','measured_state_age_ns','position_error_m','orientation_error_rad'):
                q=quantiles([r.get(f) for r in attempts]);tr.update({f+'_'+k:v for k,v in q.items()})
                if not primary['window_completed']:
                    for k in ('p50','p95','p99'):tr[f+'_'+k]=None
            trades.append(tr);summary_index[uid]=tr
            if representative:
                for e in compact_display([e for e in events if e['worker']=='primary'],('start_ns','finish_ns')):
                    timelines.append(dict(b,scene_family=family,trace_kind='release',time_s=e['release_ns']*1e-9,
                        windows=c.get('windows'),**{k:v for k,v in e.items() if k not in b}))
        if representative:
            for r in compact_display(attempts,('ik_call_time_ms','proposal_age_ns','proposal_state_age_ns','measured_state_age_ns','position_error_m','orientation_error_rad')):
                timelines.append(dict(b,scene_family=family,trace_kind='attempt',time_s=r.get('source_time_s'),
                    windows=c.get('windows'),**{k:v for k,v in r.items() if k not in b}))
        if index%25==0:ctx.progress('reduce',index+1,len(ctx.units))
    for name,rows in [('timing_quantiles',timing),('timing_ecdf',ecdf),('problem_dimensions',dimensions),('deadline_coverage',deadlines),
        ('resource_tradeoffs',trades),('T03_platform',platforms),('representative_timeline',timelines),
        ('cost_components',costs),('quality_coverage',quality)]:ctx.table(name,rows)
    pairs=pairing(ctx,summary_index,resource_index,admission_rows);ctx.table('paired_runtime_effects',pairs)
    ctx.table('observation_overhead',[r for r in pairs if r['factor']=='observation'])
    counts=collections.Counter((u['status'],u['validation']['status']) for u in ctx.units)
    write_json(ctx.output/'evaluation/validation.json',dict(status='passed',source_units=len(ctx.units),
        counts={a+'/'+b:n for (a,b),n in counts.items()},source_policy='all required units retained',
        causal_cross_method_admission=False,scientific_conclusion='deferred',native_apps_launched=0))
    text=['# A02：运行成本与调度探索性分析','',
        '本报告读取固定 E09–E10 原始证据，描述单个 case/process repeat 的观测；不代表实时性、WCET 或总体优胜结论。',
        '',f'来源包含 {len(ctx.units)} 个声明单元。状态计数：`{dict(counts)}`。这些是单元状态，不是求解成功率。',
        '', '## 观察与证据', '',
        f'E09 产出 {len(timing)} 行 cold/warmup/steady 全调用分布；失败调用保留在分母。E10 产出 {len(deadlines)} 行 worker release 账目和 {len(trades)} 行资源—质量记录。',
        'P50/P95/P99 使用 nearest-rank，按进程分别计算，不平均 P99。问题规模只使用 raw 报告的实际字段，backend 隐藏的矩阵维度保持 unavailable。',
        'native full 仅指原生 solution_quality 标签，不等于独立任务达成或约束证据。native_quality_counts 保留全部分类，独立核验状态与 TCP 误差另列。',
        'E10 deadline 分母包括全部计划 release。skipped 属于 miss；缺样与未运行后缀分别保留，存在缺样或溢出时尾部分位数不可用。连续 miss 在完整 release 索引上计算，缺样时为已观测下界；整个窗口的精确 miss 比例置空，另列 observed misses / planned 的下界。',
        'CPU 与线程来自 resource_summary.json 和当次 environment.json；不能从 arm 名字推断真实资源。virtual 与真实线程分层，后者全部为非 RT 开发观测。',
        '', '## 对照资格与限制','',
        '固定引用 A01 parity_table 和 pair_admission。MCC–PlaCo 接受策略差异阻止严格因果配对，production-static 是历史基线。仅全部控制字段、身份、环境匹配的单因素对照产出描述性比值；资源改变显式标识。',
        '缺失阶段计时、配置名暗示的诊断禁用、不可见 backend 重写规模不得补成零或推断事实。年龄与误差关联只用于探索。',
        '', '## 下一步条件','',
        '先处理不可用组合和不完整时序，再在独立冻结、RT 环境及串行 benchmark 中补齐正式证据。A03、正式实验、显著性检验及论文继续 deferred。',
        '', '## 数据与图表索引','']
    text+=['','## 本轮数值观察','', '| 实验 | 单元状态 / 核验状态 | 单元数 |','|---|---|---:|']
    for exp in ('E09','E10'):
        for (status,validation),n in sorted(collections.Counter((u['status'],u['validation']['status']) for u in ctx.units if u['experiment_id']==exp).items()):
            text.append(f'| {exp} | {status} / {validation} | {n} |')
    text+=['',f'单因素候选对照 {len(pairs)} 对，其中 {sum(r["admitted"] for r in pairs)} 对通过身份、控制变量、资源及 A01 政策检查。这些比值仍是非 RT 描述性结果。','',
        'E09 下面固定选择各 mode 按 unit_id 排序首个 repeat 0 的 steady 过程，未按结果优劣挑选：','',
        '| 过程 | 样本 / 计划 | P50 ms | P99 ms | native full / 计划 |','|---|---:|---:|---:|---:|']
    for mode in sorted({r.get('mode') for r in timing if r.get('mode')}):
        candidates=sorted([r for r in timing if r.get('mode')==mode and r['repeat_id']==0 and r['window']=='steady'],key=lambda r:r['unit_id'])
        if candidates:
            r=candidates[0];text.append(f'| {r["unit_id"]} | {r["observed_attempts"]} / {r["expected_attempts"]} | {r["p50_ms"]} | {r["p99_ms"]} | {r["native_full_quality"]} / {r["expected_attempts"]} |')
    overflow_units={r['unit_id'] for r in deadlines if r['overflow']}
    text+=['',f'E10 有 {len(overflow_units)} 个来源单元记录观测溢出。所有相关 worker 的尾部分位数保持 unavailable；不能解释为零时延或零 miss。', '',
        '| 模式 / 调度 | primary 计划 releases 总量（各窗口之和） | 已观测 miss | skipped | unknown | not-run |','|---|---:|---:|---:|---:|---:|']
    for group in sorted({(r['schedule_mode'],r['scheduler']) for r in deadlines}):
        rs=[r for r in deadlines if r['worker']=='primary' and (r['schedule_mode'],r['scheduler'])==group]
        values=[sum(r[k] for r in rs) for k in ('planned_releases','observed_deadline_misses','skipped_releases','unknown_releases','not_run_releases')]
        text.append('| '+' / '.join(group)+' | '+' | '.join(map(str,values))+' |')
    text+=['','以上合计只用于完整性核对，不能将不同 case 或资源的 release 当作独立重复，也不据此排名。所有过程与误差/年龄、资源身份均可在下列源表逐行追溯。','']
    for name in ('timing_quantiles','timing_ecdf','problem_dimensions','deadline_coverage','resource_tradeoffs','T03_platform','paired_runtime_effects','observation_overhead','quality_coverage','cost_components','representative_timeline','a01_parity_table','a01_pair_admission'):
        text.append(f'- [{name}](evaluation/{name}.csv)')
    text+=['','[图表来源与哈希](figures/index.json)','']
    (ctx.output/'report.md').write_text('\n'.join(text))


def render_base(ctx):
    """Reads only analysis CSV tables, never raw experiment files."""
    plt=plotting()
    excluded=set()
    if ctx.definition.get('schema_version')=='analysis.v2':
        excluded={r['unit_id'] for r in csv_rows(ctx.output/'evaluation/base_evidence_classification.csv') if r['scientific_timing_eligible']!='True'}
    def read(n):
        rows=list(csv_rows(ctx.output/'evaluation'/f'{n}.csv'))
        return [r for r in rows if r.get('unit_id') not in excluded] if n in ('resource_tradeoffs','representative_timeline') else rows
    ecdf=read('timing_ecdf');timing=read('timing_quantiles');dims=read('problem_dimensions');trades=read('resource_tradeoffs');timeline=read('representative_timeline')
    steady=[r for r in timing if r.get('window')=='steady' and r.get('p99_ms')]
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for method in sorted({r['method_id'] for r in steady}):
        rs=[r for r in steady if r['method_id']==method]
        axes[0].scatter([float(r['p50_ms']) for r in rs],[float(r['p99_ms']) for r in rs],s=9,alpha=.5,label=method,color=METHOD_COLORS.get(method,'#0072B2'))
        axes[1].scatter([float(r['native_full_quality_coverage']) for r in rs],[float(r['max_ms']) for r in rs],s=9,alpha=.5,label=method,color=METHOD_COLORS.get(method,'#0072B2'))
    axes[0].set(xlabel='Per-process P50 IK (ms)',ylabel='Per-process P99 IK (ms)',title='F05 | all calls, non-RT');axes[1].set(xlabel='Native full / declared calls',ylabel='Observed maximum IK (ms)')
    if steady:axes[0].legend(fontsize=7)
    save_figure(ctx,fig,'F05','timing-and-quality',['evaluation/timing_quantiles.csv'],'All process repeats, no pooled or averaged P99',['p50_ms','p99_ms','max_ms','native_full_quality_coverage'])
    dim_by=collections.defaultdict(list)
    for row in dims:dim_by[row['unit_id']].append(row)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for mode in sorted({r.get('mode','') for r in steady}):
        rs=[r for r in steady if r.get('mode')==mode and r['unit_id'] in dim_by]
        for ax,key in zip(axes,('native_variable_count','task_position_rows')):
            unique={r['unit_id']:{v.get(key) for v in dim_by[r['unit_id']] if v.get(key)} for r in rs}
            good=[r for r in rs if len(unique[r['unit_id']])==1]
            ax.scatter([float(next(iter(unique[r['unit_id']]))) for r in good],[float(r['p99_ms']) for r in good],s=8,alpha=.4,label=mode)
            ax.set(xlabel=key+' (reported)',ylabel='Per-process P99 IK (ms)',title='F06 | actual dimensions; hidden rows unavailable')
    if steady:axes[0].legend(fontsize=7)
    save_figure(ctx,fig,'F06','actual-dimensions',['evaluation/timing_quantiles.csv','evaluation/problem_dimensions.csv'],'Reported dimensions only; variable dimension units omitted from point plot and retained in source table',['native_variable_count','task_position_rows','p99_ms'])
    for mode in sorted({r.get('mode','') for r in ecdf}):
        fig,ax=plt.subplots(figsize=(8,4));curves=collections.defaultdict(list)
        for row in ecdf:
            if row.get('mode')==mode and row.get('window')=='steady':curves[row['unit_id']].append(row)
        for uid,rs in sorted(curves.items()):
            ax.plot([float(r['ik_call_time_ms']) for r in rs],[float(r['percentile'])/100 for r in rs],alpha=.15,lw=.5,color=METHOD_COLORS.get(rs[0]['method_id'],'#0072B2'))
        ax.set(xlabel='IK call time (ms)',ylabel='Within-process empirical quantile grid',title='F05 | '+mode+'; one curve per process, no pooled repeats')
        save_figure(ctx,fig,'F05','distribution-'+mode,['evaluation/timing_ecdf.csv'],
            'All calls incl rejected/partial; fixed percentile grid for display, no pooling',
            ['ik_call_time_ms','percentile','unit_id','window'],{'mode':mode,'window':'steady'})
    groups=collections.defaultdict(list)
    for r in timeline:
        if r['experiment_id']=='E10':groups[(r['scene_family'],r['schedule_mode'])].append(r)
    for number,(family,rows) in enumerate(sorted(groups.items())):
        fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
        unit_groups=collections.defaultdict(list)
        for r in rows:unit_groups[r['unit_id']].append(r)
        for arm_index,(uid,rs) in enumerate(sorted(unit_groups.items())):
            color=METHOD_COLORS.get(rs[0]['method_id'],'#0072B2');label=rs[0]['arm_id'];style=('-', '--', ':', '-.')[arm_index%4]
            calls=[r for r in rs if r['trace_kind']=='attempt'];events=[r for r in rs if r['trace_kind']=='release']
            good=[r for r in calls if r.get('proposal_state_age_ns') and r.get('time_s')]
            axes[0].plot([float(r['time_s']) for r in good],[float(r['proposal_state_age_ns'])*1e-6 for r in good],lw=.6,color=color,linestyle=style,label=label)
            good=[r for r in calls if r.get('position_error_m') and r.get('time_s')]
            axes[1].plot([float(r['time_s']) for r in good],[float(r['position_error_m']) for r in good],lw=.6,color=color,linestyle=style)
            executed=[r for r in events if r.get('start_ns') and r.get('finish_ns')]
            axes[2].hlines([arm_index]*len(executed),[float(r['start_ns'])*1e-9 for r in executed],[float(r['finish_ns'])*1e-9 for r in executed],color=color,lw=1)
            misses=[r for r in events if r.get('deadline_miss')=='True']
            axes[2].scatter([float(r['time_s']) for r in misses],[arm_index]*len(misses),s=3,color='red')
        windows=json.loads(rows[0]['windows']) if rows[0].get('windows') else {}
        for ax in axes:
            if 'injection' in windows:ax.axvspan(*windows['injection'],color='orange',alpha=.14)
            if windows:ax.set_xlim(0,max(v[1] for v in windows.values()))
        axes[0].set(ylabel='Proposal captured-state age (ms)',title='F07 | '+' / '.join(family)+' | non-RT development');axes[0].legend(fontsize=5,ncol=3)
        axes[1].set_ylabel('Max TCP error (m)')
        axes[2].set(xlabel='Declared run time (s)',ylabel='Arm execution intervals; red = miss')
        axes[2].set_yticks(range(len(unit_groups)),[rs[0]['arm_id'] for _,rs in sorted(unit_groups.items())],fontsize=5)
        save_figure(ctx,fig,'F07',f'timeline-{number:03d}',['evaluation/representative_timeline.csv'],
            'Lexicographic first case per input family, repeat 0; all arms; full declared windows',
            ['time_s','start_ns','finish_ns','proposal_state_age_ns','position_error_m','deadline_miss','skipped'],{'family':family,'unit_ids':sorted(unit_groups),'windows':windows})
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    stratum=lambda r:(r.get('timing_interpretation',''),r.get('scheduler',''),r.get('effective_primary_cpu',''),r.get('effective_secondary_cpu',''),r.get('thread_count',''))
    for group_index,group in enumerate(sorted({stratum(r) for r in trades})):
        mode=' / '.join(group)
        rs=[r for r in trades if stratum(r)==group and r.get('cpu_time_s')]
        axes[0].scatter([float(r['cpu_time_s']) for r in rs],[float(r['native_full_quality_coverage']) for r in rs],s=15,label=mode,marker=('o','s','^','v','D','+','x','*')[group_index%8],color='#0072B2')
        good=[r for r in rs if r.get('proposal_state_age_ns_p99') and r.get('position_error_m_p99')]
        axes[1].scatter([float(r['proposal_state_age_ns_p99'])*1e-6 for r in good],[float(r['position_error_m_p99']) for r in good],s=15,label=mode,marker=('o','s','^','v','D','+','x','*')[group_index%8],color='#0072B2')
    axes[0].set(xlabel='Worker CPU time (s)',ylabel='Native full / planned primary releases',title='F08 | separate resource strata in source table')
    axes[1].set(xlabel='Proposal captured-state age P99 (ms)',ylabel='TCP error P99 (m)',title='Descriptive association; no causal claim')
    if trades:axes[0].legend(fontsize=5,loc='best')
    save_figure(ctx,fig,'F08','resources-ages-quality',['evaluation/resource_tradeoffs.csv','evaluation/paired_runtime_effects.csv'],'All units, actual resources and separate virtual/nonRT identities',['cpu_time_s','native_full_quality_coverage','proposal_state_age_ns_p99','position_error_m_p99'])
    platforms=read('T03_platform');unique={}
    for r in platforms:
        if r.get('boot_id'):
            key=tuple(r.get(k,'') for k in ('experiment_id','schedule_mode','scheduler','effective_primary_cpu','effective_secondary_cpu','thread_count','os_scheduler','realtime_flag'))
            unique[key]=r
    fig,ax=plt.subplots(figsize=(12,max(2,0.3*len(unique)+1)));ax.axis('off')
    header=['Experiment','Mode','App scheduler','CPU primary','CPU secondary','Threads','OS policy','RT flag']
    if unique:
        table=ax.table(cellText=[list(k) for k in sorted(unique)],colLabels=header,loc='center');table.auto_set_font_size(False);table.set_fontsize(6);table.scale(1,1.4)
    ax.set_title('T03 | frozen source environment; non-RT development; unavailable is not zero')
    save_figure(ctx,fig,'T03','platform',['evaluation/T03_platform.csv'],'Per-unit full topology/dependencies retained in CSV; unique resource selections displayed',['experiment_id','schedule_mode','scheduler','effective_primary_cpu','effective_secondary_cpu','thread_count','os_scheduler','realtime_flag'])


def analyze(ctx):
    if ctx.definition.get('schema_version')=='analysis.v2':
        import evidence_v2
        return evidence_v2.analyze(ctx,analyze_base,sys.modules[__name__])
    return analyze_base(ctx)


def render(ctx):
    render_base(ctx)
    if ctx.definition.get('schema_version')=='analysis.v2':
        import evidence_v2
        evidence_v2.render(ctx)


if __name__=='__main__':raise SystemExit(cli(sys.modules[__name__],Path(__file__).with_name('definition.json')))
