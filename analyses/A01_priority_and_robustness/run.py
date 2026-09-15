#!/usr/bin/env python3
"""A01: frozen E06–E08 semantic, priority and robustness exploratory analysis."""
from __future__ import annotations
import collections
import itertools
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'tools/mcc_placo_study'))
from analysis import common as C
from metrics import secondary_gain

# These prefixes are the generator's explicit scene names, not outcome-derived bins.
FAMILIES = ('analytic', 'r1_hold_tcp', 'r1_orientation_conflict', 'r1_weight_sweep', 'r1_task_transition',
            'joint_limit', 'unreachable_one_arm', 'moving_state', 'singular',
            'near_singular', 'conflicting_tasks', 'collision_diagnostic',
            'initial_infeasible', 'fixture_compressed_events', 'hard_primary', 'recovery')
METHODS = ['production-static', 'placo-controlled', 'mcc-weighted', 'mcc-hqp-2', 'mcc-hqp-3']
COLORS = dict(zip(METHODS, ['#777777', '#D55E00', '#0072B2', '#009E73', '#CC79A7']))
MARKERS = dict(zip(METHODS, ['x', 's', 'o', '^', 'D']))


def rows(unit, name):
    p = C.unit_path(unit, name)
    if p:
        yield from (C.jsonl_rows(p) if name.endswith('.jsonl') else C.array_rows(p))


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def maxabs(values):
    values = [abs(v) for v in (values or []) if finite(v)]
    return max(values) if values else None


def gain(reference, candidate):
    if not finite(reference) or not finite(candidate):
        return None, None, 'unavailable'
    result = secondary_gain(reference, candidate)
    return result['absolute'], result['relative'], 'not-applicable-zero-reference' if result['status']=='not-applicable' else result['status']


def family(unit):
    case = unit['case_id']
    return next((p for p in FAMILIES if case.startswith(p)), case)


def stratum(unit, request):
    if request.get('input', {}).get('analytic'): return 'analytic-oracle'
    if request.get('input', {}).get('fixture'): return 'compressed-fixture'
    if 'collision' in unit['case_id']: return 'soft-collision-diagnostic'
    return 'R1-evolving' if unit['config'].get('state_mode') == 'evolving' else 'R1-snapshot'


def new_aggregate():
    return dict(count=0, numeric_count=0, total=0., minimum=None, maximum=None, last=None)


def add(agg, value):
    agg['count'] += 1
    if finite(value):
        agg['numeric_count'] += 1; agg['total'] += value; agg['last'] = value
        agg['minimum'] = value if agg['minimum'] is None else min(value, agg['minimum'])
        agg['maximum'] = value if agg['maximum'] is None else max(value, agg['maximum'])


def summary(agg, prefix):
    return {prefix+'_samples': agg['count'], prefix+'_numeric_samples': agg['numeric_count'],
            prefix+'_mean': agg['total']/agg['numeric_count'] if agg['numeric_count'] else None,
            prefix+'_min': agg['minimum'], prefix+'_max': agg['maximum'], prefix+'_last': agg['last']}


def select_points(data, fields, event_fields=(), bins=200):
    """Display only: first/last, every event edge (both sides), per-bin extrema."""
    if len(data) <= 2*bins: return data
    selected = {0, len(data)-1}
    for i in range(1, len(data)):
        if any(data[i].get(k) != data[i-1].get(k) for k in event_fields): selected.update([i-1, i])
    width = max(1, math.ceil(len(data)/bins))
    for start in range(0, len(data), width):
        indexes = range(start, min(start+width, len(data)))
        for key in fields:
            numeric = [i for i in indexes if finite(data[i].get(key))]
            if numeric:
                selected.update([min(numeric, key=lambda i: data[i][key]), max(numeric, key=lambda i: data[i][key])])
    return [data[i] for i in sorted(selected)]


def pair_rows(units):
    groups = collections.defaultdict(list)
    for u in units: groups[(u['experiment_id'], u['case_id'], u['repeat_id'])].append(u)
    for key, group in sorted(groups.items()):
        for left, right in itertools.combinations(sorted(group, key=C.unit_id), 2):
            secondary = left['method_id'] == right['method_id'] and left['config'].get('primary_only') != right['config'].get('primary_only')
            comparison = 'secondary' if secondary else 'hierarchy'
            allowed = ('primary_only', 'secondary_tasks') if secondary else ('layers',)
            reasons = C.compatible(left, right, allowed=allowed)
            yield dict(scope='case-pair', experiment_id=key[0], case_id=key[1], repeat_id=key[2],
                       left_unit_id=C.unit_id(left), right_unit_id=C.unit_id(right),
                       left_method_id=left['method_id'], right_method_id=right['method_id'],
                       comparison=comparison, admitted=not reasons, reasons=reasons,
                       input_hash=left.get('input_hash'), model_hash=left.get('model_hash'), window_id=left.get('window_id'))
    for left, right in itertools.combinations(METHODS, 2):
        reasons = []
        if 'production-static' in (left, right): reasons.append('historical-production-static')
        if ('placo' in left) != ('placo' in right): reasons.append('E06-native-acceptance-mismatch')
        yield dict(scope='method-policy', experiment_id='E06', case_id='all-required-conditions', repeat_id='',
                   left_unit_id='', right_unit_id='', left_method_id=left, right_method_id=right,
                   comparison='policy-only', admitted=False, reasons=reasons or ['requires-specific-case-pair'],
                   input_hash='', model_hash='', window_id='')


def trace_summary(unit, request, representative):
    """Read each raw attempt once; retain compact identity and all disposition counts."""
    counts = collections.Counter(); samples = []; first = None; last = None
    raw_by_line = {}  # only compact diagnostic join records, no matrices/Jacobians retained
    dimensions = set(); events = []
    for line, row in enumerate(rows(unit, 'raw.jsonl'), 1):
        if row.get('record_type') == 'attempt_started': counts['started'] += 1; continue
        counts['result'] += 1
        accepted = row.get('committed') is True or row.get('accepted') is True
        counts['accepted' if accepted else 'not_accepted'] += 1
        disposition = row.get('disposition', row.get('status', 'unknown')) or 'unspecified'
        counts['disposition:'+disposition] += 1
        if finite(row.get('completed_passes')) and finite(row.get('requested_passes')):
            counts['hierarchy:full' if row['completed_passes']==row['requested_passes'] else 'hierarchy:partial'] += 1
        t = row.get('source_time_s'); first = t if first is None else first; last = t
        compact = dict(source_line=line, attempt_sequence=row.get('attempt_sequence'), source_time_s=t,
                       native_disposition=disposition, committed=row.get('committed'), accepted=row.get('accepted'),
                       task_revision=row.get('task_revision'), secondary_enabled=row.get('secondary_enabled'),
                       completed_passes=row.get('completed_passes'), requested_passes=row.get('requested_passes'),
                       native_status_code=row.get('native_status_code'), native_solution_quality=row.get('native_solution_quality'))
        raw_by_line[line] = compact
        n = row.get('internal_primary_variable_count')
        if n is not None: dimensions.add(n)
        if row.get('record_type') == 'analytic':
            compact['analytic_candidate'] = row.get('candidate')
            compact['analytic_problem'] = row.get('problem')
        if representative:
            scales = [s.get('value') for s in row.get('scales', []) if s.get('evaluated')]
            compact = dict(compact, scale_min=min(scales) if scales else None, scale_max=max(scales) if scales else None)
            samples.append(compact)
        event = (disposition, row.get('committed'), row.get('task_revision'), row.get('secondary_enabled'))
        if not events or events[-1]['event'] != event:
            events.append(dict(compact, event=event))
    return dict(result_rows=counts['result'], started_rows=counts['started'], accepted_rows=counts['accepted'],
                not_accepted_rows=counts['not_accepted'], dispositions=dict(counts), actual_variables=sorted(dimensions),
                first_time_s=first, last_time_s=last), raw_by_line, samples, events


def initial_violation(request):
    data = request.get('input', {}); q=data.get('initial_state', {}).get('q', [])
    limits=data.get('limits', {}); lo=limits.get('lower', []); hi=limits.get('upper', [])
    if not q or len(q)!=len(lo) or len(q)!=len(hi): return None
    return max(max(0., l-x, x-h) for x,l,h in zip(q,lo,hi))


def anomaly_kind(unit, scale_failed, initial, raw_info):
    if unit['status'] == 'crashed': return 'native-crash', '原生进程异常退出；没有完整候选结果，不能推断求解质量。'
    if scale_failed:
        accepted = sum(r.get('committed') is True for r in scale_failed)
        return 'scale-equation', f'独立缩放方程超出原 hard_tolerance；{accepted} 条对应 committed=true。有限差分与原生任务行差异的贡献尚未分离，不修改失败状态。'
    if unit['validation'].get('failures'):
        if initial is not None and initial > 0:
            return 'initial-hard-violation', '输入初始状态已越界；需区分拒绝后保留的 measured state 与接受新越界状态。该核验失败不自动等同候选造成越界。'
        return 'hard-violation', '硬约束核验失败，保留原始状态；根因未确定。'
    return 'unresolved-validation', '失败证据不足以确定单一原因；保留原 validator 结果。'


def analyze(ctx):
    units = sorted(ctx.units, key=C.unit_id)
    representatives = {}
    for u in units:
        if u['repeat_id'] == 0:
            key = (u['experiment_id'], family(u), u['config'].get('state_mode'))
            representatives[key] = min(u['case_id'], representatives.get(key, u['case_id']))
    ctx.table('pair_admission', list(pair_rows(units)))
    parity=[]; semantics=[]; priorities=[]; oracles=[]; secondary=[]; summaries=[]; recoveries=[]
    anomalies=[]; anomaly_checks=[]; timeline=[]; transitions=[]; availability=[]
    for index, unit in enumerate(units, 1):
        request=C.unit_json(unit, 'request.json', {}); config=unit['config']; base=C.base_row(unit)
        base.update(family=family(unit), stratum=stratum(unit, request), state_mode=config.get('state_mode'),
                    layers=config.get('layers'), secondary_weight=config.get('secondary_weight'),
                    primary_only=config.get('primary_only'), enforcement=config.get('enforcement'),
                    scale_group=config.get('scale_group'), preservation_tolerance=config.get('preservation_tolerance'),
                    hard_tolerance=config.get('hard_tolerance'), feedback_model=config.get('feedback_model'))
        sem=C.unit_json(unit, 'method_semantics.json', {})
        semantics.append(dict(base, config=config, method_semantics=sem,
                              active_dofs=len(request.get('input', {}).get('active_joint_names', [])),
                              unit_status_reason=unit.get('reason')))
        availability.append(dict(base, reason=unit.get('reason'), interpretation='required unit status; not solve success'))
        if unit['experiment_id']=='E06':
            checks=list(rows(unit, 'parity_checks.json'))
            for check in checks: parity.append(dict(base, **check))
            if not checks: parity.append(dict(base, field='source-availability', status='unavailable', reason=unit.get('reason')))
        is_rep=unit['repeat_id']==0 and representatives.get((unit['experiment_id'], family(unit), config.get('state_mode')))==unit['case_id']
        raw, byline, native_samples, event_rows=trace_summary(unit, request, is_rep)
        for r in event_rows: transitions.append(dict(base, **r))
        initial=initial_violation(request)
        agg=collections.defaultdict(new_aggregate); statuscounts=collections.Counter()
        tasks=collections.defaultdict(lambda: collections.defaultdict(new_aggregate)); taskstatus=collections.defaultdict(collections.Counter)
        for check in rows(unit, 'priority_checks.json'):
            task=check.get('task','unknown'); taskstatus[task][check.get('status','unknown')]+=1
            add(tasks[task]['drift'],maxabs(check.get('drift')))
            add(tasks[task]['preservation_ratio'],check.get('preservation_ratio'))
            add(tasks[task]['residual'],maxabs(check.get('independent_final_residual')))
        for task, metrics in sorted(tasks.items()):
            r=dict(base, task=task, task_unit='rad/s' if 'orientation' in task or task=='posture' else 'm/s',
                   status_counts=dict(taskstatus[task]), reference='same-tick native optimum; independently reconstructed final residual')
            for metric, values in metrics.items(): r.update(summary(values,metric))
            priorities.append(r)
        for check in rows(unit, 'oracle_checks.json'):
            add(agg['oracle_gap'],check.get('reference_objective_gap',maxabs(check.get('objective_gaps'))))
            statuscounts['oracle:'+check.get('status','unknown')]+=1
        oracles.append(dict(base, **summary(agg['oracle_gap'],'objective_gap'), status_counts=dict(statuscounts),
                            comparison_basis='objective and feasibility; no candidate-vector equality; R1 reference only position primary'))
        secondary_tasks=collections.defaultdict(lambda: collections.defaultdict(new_aggregate))
        for check in rows(unit, 'secondary_effects.json'):
            task=check.get('task','unknown')
            for name in ['candidate_secondary_cost','primary_only_secondary_cost','absolute','relative']:
                add(secondary_tasks[task][name],check.get(name))
        for task, metrics in sorted(secondary_tasks.items()):
            r=dict(base, task=task, reference_scope='independent bounded-LS at same linearization; not paired method effect')
            for metric, values in metrics.items():r.update(summary(values,metric))
            secondary.append(r)
        constraint_points=[]
        for check in rows(unit, 'constraint_trace.json'):
            add(agg['position_violation_rad'],maxabs(check.get('position_raw_violation')))
            add(agg['velocity_violation_rad_s'],maxabs(check.get('velocity_raw_violation')))
            statuscounts['initial-feasible:'+str(check.get('initial_position_feasible'))]+=1
            if is_rep:
                line=check.get('source_line');join=byline.get(line,{})
                constraint_points.append(dict(source_line=line,attempt_sequence=check.get('attempt_sequence'),source_time_s=join.get('source_time_s'),position_violation_rad=maxabs(check.get('position_raw_violation')),velocity_violation_rad_s=maxabs(check.get('velocity_raw_violation')),native_disposition=check.get('native_disposition')))
        for point in select_points(constraint_points,['position_violation_rad','velocity_violation_rad_s'],['native_disposition']):
            timeline.append(dict(base,series='constraints',**point))
        trajectory=collections.defaultdict(list)
        for check in rows(unit, 'progress_recovery.json'):
            side=check.get('side','unknown')
            for metric in ['position_error_m','orientation_error_rad']:add(agg[side+'_'+metric],check.get(metric))
            if is_rep:trajectory[side].append(check)
        for side, data in sorted(trajectory.items()):
            for point in select_points(data,['position_error_m','orientation_error_rad'],['native_committed']):
                timeline.append(dict(base, series='tracking', **point))
        for point in select_points(native_samples,['scale_min','scale_max'],['committed','native_disposition','task_revision','secondary_enabled']):
            timeline.append(dict(base, series='native', **point))
        for rec in rows(unit, 'progress_recovery_metrics.json'):
            prog=[p.get('value') for p in rec.get('directional_progress',[]) if finite(p.get('value'))]
            reduction=rec.get('error_reduction_m',[])
            recoveries.append(dict(base, side=rec.get('side'),
                directional_progress_last_m=prog[-1] if prog else None, directional_progress_max_m=max(prog) if prog else None,
                directional_progress_status='measured' if prog else 'not-applicable-or-unavailable',
                error_reduction_last_m=reduction[-1] if reduction else None,
                recovery=rec.get('recovery', {'status':'unavailable','reason':'no recovery metric in source'}),
                stall=rec.get('stall',{}), thresholds_provisional=rec.get('thresholds_provisional_development')))
        scale_failed=[]
        for check_index, check in enumerate(rows(unit, 'scale_equation_checks.json')):
            residual=maxabs(check.get('equation_residual'));add(agg['scale_residual'], residual);add(agg['scale'],check.get('scale'))
            statuscounts['scale:'+check.get('status','unknown')]+=1
            if check.get('status')=='failed':
                join=byline.get(check.get('source_line'),{})
                compact_check={k:v for k,v in check.items() if k not in ('independent_J','baseline_velocity','desired_velocity')}
                evidence=dict(base, check_index=check_index, **compact_check, **{k:v for k,v in join.items() if k not in check},
                              equation_residual_max=residual, source_file=str(C.unit_path(unit,'scale_equation_checks.json')),
                              raw_file=str(C.unit_path(unit,'raw.jsonl')))
                scale_failed.append(evidence);anomaly_checks.append(evidence)
        result=dict(base, **raw, initial_position_violation_rad=initial, check_status_counts=dict(statuscounts),
                    positive_effect_eligible=unit['status']=='completed' and unit['validation']['status']=='passed')
        for metric, values in agg.items():result.update(summary(values,metric))
        summaries.append(result)
        if unit['experiment_id']=='E08' and (unit['validation']['status']=='failed' or unit['status'] not in ('completed','unavailable')):
            kind, diagnosis=anomaly_kind(unit,scale_failed,initial,raw)
            failedjoin=[dict(f, raw=byline.get(f.get('line'),{})) for f in unit['validation'].get('failures',[])]
            stderr=C.unit_path(unit,'stderr.log')
            # Native crash logs are bounded; full original file stays hash-linked in sources.
            excerpt=stderr.read_text(errors='replace')[-6000:] if stderr and unit['status']=='crashed' else ''
            anomalies.append(dict(base, anomaly=kind, diagnosis=diagnosis,
                                  initial_position_violation_rad=initial, scale_failed_checks=len(scale_failed),
                                  scale_max_residual=max((x['equation_residual_max'] for x in scale_failed),default=None),
                                  scale_failed_committed=sum(x.get('committed') is True for x in scale_failed),
                                  first_failed_source_line=scale_failed[0]['source_line'] if scale_failed else None,
                                  validation_failures=failedjoin, stderr_excerpt=excerpt,
                                  validation_path=str(C.unit_path(unit,'validation.json')),
                                  raw_path=str(C.unit_path(unit,'raw.jsonl')), required_followup='核验语义及原生状态归因；任何改动后另建补验 run，保持本轮失败不变'))
        if index%25==0:ctx.progress('reduce',index,len(units))
    ctx.table('parity_table',parity);ctx.table('method_semantics',semantics);ctx.table('availability',availability)
    ctx.table('priority_checks',priorities);ctx.table('oracle_objectives',oracles);ctx.table('secondary_costs',secondary)
    ctx.table('robustness_cells',summaries);ctx.table('recovery',recoveries);ctx.table('anomalies',anomalies)
    ctx.table('anomaly_scale_checks',anomaly_checks);ctx.table('representative_timeline',timeline);ctx.table('task_transitions',transitions)
    pairs=paired_secondary(units,secondary);ctx.table('paired_effects',pairs)
    report(ctx, summaries, anomalies, pairs, priorities, recoveries)
    if ctx.definition.get("schema_version", "analysis.v1") == "analysis.v2":
        import e07_evidence, e08_evidence
        e07_evidence.analyze(ctx)
        e08_evidence.analyze(ctx)


def paired_secondary(units, secondary):
    lookup={(r['unit_id'],r['task']):r for r in secondary}
    effects=[]
    for u in units:
        if u['experiment_id']!='E07' or u['config'].get('primary_only'):continue
        candidates=[p for p in units if p['experiment_id']=='E07' and p['case_id']==u['case_id'] and p['method_id']==u['method_id'] and p['repeat_id']==u['repeat_id'] and p['config'].get('primary_only')]
        records=[r for (key,task),r in lookup.items() if key==C.unit_id(u)]
        if not records:records=[dict(task='all-secondary',candidate_secondary_cost_mean=None)]
        for r in records:
            reasons=['missing-primary-only-reference'] if len(candidates)!=1 else C.compatible(candidates[0],u,('primary_only','secondary_tasks'))
            reference=lookup.get((C.unit_id(candidates[0]),r['task'])) if len(candidates)==1 else None
            rcost=reference.get('candidate_secondary_cost_mean') if reference else None
            absolute,relative,status=gain(rcost,r.get('candidate_secondary_cost_mean'))
            if reasons or reference is None:
                absolute=relative=None;status='unavailable';reasons+=[] if reference else ['reference-task-cost-unavailable']
            effects.append(dict(C.base_row(u),task=r['task'],reference_unit_id=C.unit_id(candidates[0]) if len(candidates)==1 else None,
                                reference_secondary_cost=rcost,candidate_secondary_cost=r.get('candidate_secondary_cost_mean'),
                                absolute_gain=absolute,relative_gain=relative,status=status,reasons=reasons,
                                interpretation='same-case process-mean task cost; full trajectory; no independent tick inference'))
    return effects


def report(ctx, summaries, anomalies, pairs, priorities, recoveries):
    counts=collections.Counter((r['experiment_id'],r['native_status'],r['validation_status']) for r in summaries)
    lines=['# A01：E06–E08 探索性分析','',
           '本分析只读取固定批次的原始证据及已完成的独立核验，不重新求解或修改原有判定。单元 completed / passed 不表示每次调用均求解成功。', '',
           '## 来源、覆盖与准入', '', '| 实验 | 原生单元状态 | 核验状态 | 单元数 |','|---|---|---|---:|']
    lines += [f'| {a} | {b} | {c} | {n} |' for (a,b,c),n in sorted(counts.items())]
    lines += ['', 'E06 逐字段条件见 [parity_table.csv](evaluation/parity_table.csv)，完整配置见 [method_semantics.csv](evaluation/method_semantics.csv)，逐 case 配对见 [pair_admission.csv](evaluation/pair_admission.csv)。MCC–PlaCo 接受策略差异继续阻止严格因果比较；production-static 单列历史基线。输入映射通过不能替代语义准入。', '',
              '## E07 优先级与冗余', '',
              f'保留 {sum(r["experiment_id"]=="E07" for r in priorities)} 个逐单元逐任务的保持检查汇总；解析 oracle、R1 snapshot、演化轨迹分别分层。原始 drift 依任务标记 m/s 或 rad/s，preservation ratio 仅为原容差归一化。R1 reference 是同 tick 原生最优残差的独立最终矩阵复算，不能宣称完整独立 HQP oracle。', '',
              f'同 case primary-only 配对共 {len(pairs)} 行，{sum(p["status"]=="ok" for p in pairs)} 行具有非零参考及完整比较条件；其余明确保留不可用或零分母。独立 bounded-LS 次任务成本另表保存，不冒充候选 primary-only 对照。', '',
              '[优先级](evaluation/priority_checks.csv) · [oracle](evaluation/oracle_objectives.csv) · [次任务配对](evaluation/paired_effects.csv) · [任务切换](evaluation/task_transitions.csv)', '',
              '## E08 异常与困难场景', '',
              f'本批次识别 {len(anomalies)} 个执行或核验异常单元；异常本身仍为失败，不经分析重分类成成功。', '',
              '| case / 方法 | 证据类型 | 初始越界 rad | 失败缩放检查 | 解释 |','|---|---|---:|---:|---|']
    for r in anomalies:
        lines.append(f'| {r["case_id"]} / {r["method_id"]} | {r["anomaly"]} | {r["initial_position_violation_rad"]} | {r["scale_failed_checks"]} | {r["diagnosis"]} |')
    lines += ['', '[逐异常诊断页面](anomalies.md) · [逐异常原始行和状态](evaluation/anomalies.csv) · [全部失败缩放检查](evaluation/anomaly_scale_checks.csv) · [完整困难场景汇总](evaluation/robustness_cells.csv) · [恢复及停滞](evaluation/recovery.csv)', '',
              '初始越界单元需结合 committed、native disposition 与保留的 measured state 阅读。MCC 拒绝后冻结 measured state 导致的初始越界，不等同接受了新的越界解。缩放检查必须区分 accepted 超差与 rejected/HOLD 上的候选/保留状态解释：前者需要定位容差边界及独立有限差分与原生任务行差异，后者需要核对失败输出的契约，不能归为被执行的失败轨迹。PlaCo 崩溃保留 stderr 原文和零结果覆盖。', '',
              f'恢复记录共 {sum(r["experiment_id"]=="E08" for r in recoveries)} 行。未恢复保留 censored，snapshot 无恢复区间，零方向保留 not-applicable；阈值是原开发声明中的暂定值。collision 场景仅为 soft diagnostic，不能当作硬碰撞保证。', '',
              '## 图与适用范围', '',
              '图表索引：[figures/index.json](figures/index.json)。F02 条件与能力、T01 方法身份，F03 分层保持/收益/扫描及固定代表轨迹，F04 困难配置进度与失败。代表 case 使用固定 family 规则和字典序首项、repeat 0；数值统计全量，显示仅按固定桶保留极值与所有事件边界。', '',
              '当前产出属于非实时探索性描述；没有 bootstrap、显著性检验或总体优胜排名。后续需先解决接受策略对齐、E08 异常归因和正式阈值冻结，再考虑正式统计。A03、正式实验与论文 deferred。', '']
    lines += ['## 可直接复查的数值观察','',
              '下表的调用分母包含所有记录的结果行，accepted/committed 只是原生接受标记，不能替代完整质量或任务达到率。不可运行单元仍在前述 required 覆盖表。','',
              '| 实验 / 方法 | 原始结果行 | accepted 或 committed 行 | 其他结果行 |','|---|---:|---:|---:|']
    rawgroups=collections.defaultdict(list)
    for row in summaries:rawgroups[(row['experiment_id'],row['method_id'])].append(row)
    for (exp,method),rs in sorted(rawgroups.items()):
        lines.append(f'| {exp} / {method} | {sum(r["result_rows"] for r in rs)} | {sum(r["accepted_rows"] for r in rs)} | {sum(r["not_accepted_rows"] for r in rs)} |')
    lines += ['', '保持比率是逐任务与原 tolerance 的比较；下面最大值跨该列所列 case/task 汇总行取 max，不进行显著性推断，不能当作总体方法排名。', '',
              '| 分层 / 任务 | 有数值的单元任务行 | 单元任务 max 比率的最大值 | max 比率大于 1 的行数 |','|---|---:|---:|---:|']
    groups=collections.defaultdict(list)
    for row in priorities:
        if row['experiment_id']=='E07' and finite(row.get('preservation_ratio_max')):groups[(row['stratum'],row['task'])].append(row)
    for (strat,task),rs in sorted(groups.items()):
        lines.append(f'| {strat} / {task} | {len(rs)} | {max(r["preservation_ratio_max"] for r in rs):.9g} | {sum(r["preservation_ratio_max"]>1 for r in rs)} |')
    lines += ['', '全部数据与指标单位见前述 CSV；原始记录、源码与输出哈希由本分析 manifest 固定。', '']
    (ctx.output/'report.md').write_text('\n'.join(lines))
    detail=['# E08 异常逐单元诊断','', '本页保留每一个原失败状态；字段均来自同一冻结 run，定位表列出原 raw 行号。', '']
    for r in anomalies:
        detail += ['## '+r['unit_id'],'',r['diagnosis'],'',
                   f'原生状态：`{r["native_status"]}`；核验：`{r["validation_status"]}`。初始越界：{r["initial_position_violation_rad"]} rad。',
                   f'缩放失败检查：{r["scale_failed_checks"]}；其中 committed=true：{r["scale_failed_committed"]}。最大残差：{r["scale_max_residual"]}；首个 raw 行：{r["first_failed_source_line"]}。','',
                   f'[原 validation]({r["validation_path"]}) · [原 raw]({r["raw_path"]}) · [全部缩放失败行](evaluation/anomaly_scale_checks.csv)','',
                   '```json', json.dumps(r['validation_failures'],ensure_ascii=False,indent=2), '```','']
        if r['stderr_excerpt']:detail += ['原生 stderr（尾部，完整文件见来源清单）：','```text',r['stderr_excerpt'],'```','']
        detail += [r['required_followup'],'']
    (ctx.output/'anomalies.md').write_text('\n'.join(detail))


def number(row,key):
    try:
        v=float(row.get(key,''));return v if math.isfinite(v) else None
    except (ValueError,TypeError):return None


def read_table(ctx,name):return list(C.csv_rows(ctx.output/'evaluation'/f'{name}.csv'))


def save_figure(ctx,fig,figure_id,name,sources,description,fields=None,selection=None):
    if fields is None:
        import csv
        columns={}
        for source in sources:
            with (ctx.output/source).open(newline='') as stream:columns[source]=next(csv.reader(stream),[])
        fields=[source+':'+field for source,names in columns.items() for field in names]
    C.save_figure(ctx,fig,figure_id,name,sources,description,fields=fields,selection=selection)


def refresh_report_counts(ctx):
    """Regenerate scoped report counts from frozen tables, including render-only runs."""
    import re
    path=ctx.output/'report.md'
    if not path.exists():return
    counts={'E07_priority_rows':sum(r['experiment_id']=='E07' for r in read_table(ctx,'priority_checks')),
            'E08_recovery_rows':sum(r['experiment_id']=='E08' for r in read_table(ctx,'recovery'))}
    text=path.read_text()
    updated=re.sub(r'保留 \d+ 个逐单元逐任务的保持检查汇总',f'保留 {counts["E07_priority_rows"]} 个逐单元逐任务的保持检查汇总',text)
    updated=re.sub(r'恢复记录共 \d+ 行',f'恢复记录共 {counts["E08_recovery_rows"]} 行',updated)
    path.write_text(updated)
    C.write_json(ctx.output/'evaluation/report_scope_checks.json',dict(counts,
        status='passed',source_tables=['evaluation/priority_checks.csv','evaluation/recovery.csv'],
        report_revised=updated!=text,numerical_tables_changed=False))


def render(ctx):
    refresh_report_counts(ctx)
    plt=C.plotting()
    units=read_table(ctx,'robustness_cells'); priorities=read_table(ctx,'priority_checks'); effects=read_table(ctx,'paired_effects')
    admission=read_table(ctx,'pair_admission'); anomalies=read_table(ctx,'anomalies')
    # Consistent method styles and all-required denominators; no selected winner.
    fig,ax=plt.subplots(figsize=(10,4))
    labels=[]; yes=[]; no=[]
    for method in METHODS:
        selected=[r for r in units if r['method_id']==method]
        labels.append(method);yes.append(sum(r['native_status']=='completed' and r['validation_status']=='passed' for r in selected));no.append(len(selected)-yes[-1])
    ax.bar(labels,yes,label='unit complete + validation passed',color='#0072B2');ax.bar(labels,no,bottom=yes,label='failed / unavailable',color='#D55E00',hatch='//')
    ax.set_ylabel('Required units (not solve successes)');ax.set_title('F02: E06–E08 coverage; cross MCC / PlaCo causal comparisons blocked');ax.legend(fontsize=8);ax.tick_params(axis='x',rotation=15)
    save_figure(ctx,fig,'F02','conditions-and-coverage',['evaluation/robustness_cells.csv','evaluation/pair_admission.csv'],'All required unit statuses and paired causal admission gate',fields=['native_status','validation_status','admitted'])
    parity=read_table(ctx,'parity_table')
    if parity:
        import numpy as np
        ids=sorted({r['unit_id'] for r in parity});fields=sorted({r['field'] for r in parity})
        values=np.zeros((len(ids),len(fields)))
        codes={'pass':1,'semantic_mismatch':-1,'failed':-1,'unavailable':0}
        for r in parity:values[ids.index(r['unit_id']),fields.index(r['field'])]=codes.get(r['status'],0)
        fig,ax=plt.subplots(figsize=(13,max(4,len(ids)*.27)))
        ax.imshow(values,vmin=-1,vmax=1,cmap='RdYlGn',aspect='auto')
        for i in range(len(ids)):
            for j in range(len(fields)):ax.text(j,i,{-1:'X',0:'?',1:'P'}[int(values[i,j])],ha='center',va='center',fontsize=6)
        ax.set_xticks(range(len(fields)),fields,rotation=55,ha='right',fontsize=7)
        ax.set_yticks(range(len(ids)),[x.removeprefix('E06/') for x in ids],fontsize=6)
        ax.set_title('F02: frozen E06 semantic audit — P pass, X mismatch, ? unavailable; input pass does not grant causal parity')
        save_figure(ctx,fig,'F02','semantic-condition-matrix',['evaluation/parity_table.csv'],'Every E06 required condition and source semantic field',fields=['unit_id','field','status','reason'])
    sem=read_table(ctx,'method_semantics')
    fig,ax=plt.subplots(figsize=(14,4));ax.axis('off');cells=[]
    for m in METHODS:
        rs=[r for r in sem if r['method_id']==m];configs=[json.loads(r['config']) for r in rs]
        values=lambda key: ', '.join(sorted({str(c.get(key,'not-recorded')) for c in configs}))
        cells.append([m,values('mode'),values('enforcement'),values('scale_group'),values('layers'),
                      values('qp_iterations'),values('normalization'),
                      'historical' if m=='production-static' else 'acceptance mismatch across MCC / PlaCo'])
    tab=ax.table(cellText=cells,colLabels=['Method','Mode','Enforcement','Scale','Layers*','QP iter','Normalization','Admission'],loc='center',cellLoc='left',colWidths=[.12,.11,.10,.08,.06,.08,.12,.23]);tab.auto_set_font_size(False);tab.set_fontsize(6);tab.scale(1,2.8)
    ax.set_title('T01: recorded semantics (* method ID also specifies HQP layers); missing = not recorded; full controlled factors in CSV',fontsize=9)
    save_figure(ctx,fig,'T01','methods',['evaluation/method_semantics.csv','evaluation/pair_admission.csv'],'Actual frozen modes, enforcement, scaling and numerical settings; identities and hash pins in source CSV',fields=['method_id','config','method_semantics','active_dofs'])
    for strat in sorted({r['stratum'] for r in units if r['experiment_id']=='E07'}):
        fig,axes=plt.subplots(1,2,figsize=(10,4))
        for method in METHODS:
            rs=[r for r in priorities if r['experiment_id']=='E07' and r['stratum']==strat and r['method_id']==method and number(r,'preservation_ratio_max') is not None]
            axes[0].scatter(range(len(rs)),[number(r,'preservation_ratio_max') for r in rs],s=14,marker=MARKERS[method],color=COLORS[method],label=method)
        axes[0].axhline(1,color='black',ls='--',label='declared tolerance');axes[0].set_yscale('symlog',linthresh=1e-3);axes[0].set_ylabel('max |drift| / tolerance (dimensionless)');axes[0].legend(fontsize=6)
        selected_effects=[r for r in effects if any(u['unit_id']==r['unit_id'] and u['stratum']==strat for u in units)]
        ec=collections.Counter(r['status'] for r in selected_effects)
        axes[1].bar(list(ec),list(ec.values()),color='#777777',hatch='//');axes[1].set_ylabel('Same-case secondary task pairs')
        axes[1].tick_params(axis='x',rotation=20)
        axes[0].set_xlabel('Stable sorted unit / task index')
        fig.suptitle('F03: '+strat+' — process/case descriptions, no tick independence')
        save_figure(ctx,fig,'F03','priority-'+strat,['evaluation/priority_checks.csv','evaluation/paired_effects.csv','evaluation/robustness_cells.csv'],'Stratum-specific same-tick preservation and same-case primary-only effect',selection={'stratum':strat})
    for task in sorted({r['task'] for r in effects if r['status']=='ok'}):
        fig,ax=plt.subplots(figsize=(8,3.5))
        for method in METHODS:
            rs=[r for r in effects if r['task']==task and r['method_id']==method and r['status']=='ok']
            ax.scatter(range(len(rs)),[number(r,'absolute_gain') for r in rs],marker=MARKERS[method],color=COLORS[method],label=method)
        ax.axhline(0,color='black',lw=.7);ax.set_ylabel('Mean squared task-residual reduction');ax.set_xlabel('Stable case / repeat index');ax.legend(fontsize=6)
        ax.set_title('F03: same-case primary-only benefit / '+task)
        save_figure(ctx,fig,'F03','secondary-'+task,['evaluation/paired_effects.csv'],'One task per panel, matched primary-only reference, unavailable and zero references remain in source table',fields=['task','absolute_gain','status','reasons'],selection={'task':task})
    oracle=read_table(ctx,'oracle_objectives')
    fig,ax=plt.subplots(figsize=(9,4))
    for method in METHODS:
        data=[r for r in oracle if r['experiment_id']=='E07' and r['stratum']=='analytic-oracle' and r['method_id']==method and number(r,'objective_gap_max') is not None]
        ax.scatter(range(len(data)),[number(r,'objective_gap_max') for r in data],marker=MARKERS[method],color=COLORS[method],label=method)
    ax.set_yscale('symlog',linthresh=1e-8)
    ax.set_ylabel('Maximum independent objective gap');ax.set_xlabel('Stable per-method analytic case index');ax.legend(fontsize=7);ax.set_title('F03: analytic objective / feasibility reference, multi-solution safe')
    save_figure(ctx,fig,'F03','analytic-oracle',['evaluation/oracle_objectives.csv'],'Independent analytic objective reference; no joint-vector equality')
    # Weight/layer scans retain each task/case and separate physical input strata.
    strata=sorted({r['stratum'] for r in priorities if r['experiment_id']=='E07'})
    fig,axes=plt.subplots(1,max(1,len(strata)),figsize=(max(8,4*len(strata)),4),squeeze=False)
    for ax,strat in zip(axes[0],strata):
        for method in METHODS:
            rs=[r for r in priorities if r['experiment_id']=='E07' and r['stratum']==strat and r['method_id']==method and number(r,'secondary_weight') is not None and number(r,'preservation_ratio_max') is not None]
            ax.scatter([number(r,'secondary_weight') for r in rs],[number(r,'preservation_ratio_max') for r in rs],color=COLORS[method],marker=MARKERS[method],s=12,label=method)
        ax.set_xscale('symlog',linthresh=.1);ax.set_yscale('symlog',linthresh=.001);ax.axhline(1,color='black',ls='--');ax.set_xlabel('Declared secondary weight');ax.set_ylabel('max |drift| / tolerance');ax.legend(fontsize=6);ax.set_title(strat)
    fig.suptitle('F03: weight / hierarchy scan — full matrix, input strata separated')
    save_figure(ctx,fig,'F03','weight-layer-scan',['evaluation/priority_checks.csv'],'All case/task points by input stratum; method identifies declared hierarchy layers')
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    counts=collections.Counter(r['anomaly'] for r in anomalies)
    axes[0].bar(list(counts),list(counts.values()),color='#D55E00',hatch='//');axes[0].set_ylabel('Failed units');axes[0].tick_params(axis='x',rotation=20)
    rs=[r for r in units if r['experiment_id']=='E08']
    for method in METHODS:
        data=[r for r in rs if r['method_id']==method and number(r,'scale_residual_max') is not None]
        axes[1].scatter(range(len(data)),[number(r,'scale_residual_max') for r in data],s=15,marker=MARKERS[method],color=COLORS[method],label=method)

    for tol in sorted({number(r,'hard_tolerance') for r in rs if number(r,'hard_tolerance') is not None}):
        axes[1].axhline(tol,color='black',ls='--',label=f'source hard tolerance {tol:g}')
    axes[1].set_yscale('symlog',linthresh=1e-9);axes[1].set_ylabel('Maximum scaling-equation residual (m/s)');axes[1].set_xlabel('Stable per-method case index');axes[1].legend(fontsize=6);fig.suptitle('F04: failures retained; accepted / rejected states distinguished in evidence tables')
    save_figure(ctx,fig,'F04','anomaly-overview',['evaluation/anomalies.csv','evaluation/robustness_cells.csv','evaluation/anomaly_scale_checks.csv'],'Full failed units and scale residual maxima, original tolerance unchanged')
    recovery=read_table(ctx,'recovery')
    fig,axes=plt.subplots(1,2,figsize=(10,4));states=collections.Counter()
    for method in METHODS:
        data=[r for r in recovery if r['experiment_id']=='E08' and r['method_id']==method]
        recovered=[];stalled=[]
        for r in data:
            rec=json.loads(r['recovery']);stall=json.loads(r['stall'])
            state='censored' if rec.get('censored') else rec.get('status','unavailable');states[state]+=1
            if finite(rec.get('value')):recovered.append(rec['value'])
            if finite(stall.get('total_s')):stalled.append(stall['total_s'])
        axes[0].scatter(range(len(recovered)),recovered,color=COLORS[method],marker=MARKERS[method],label=method)
    axes[0].set_ylabel('Observed recovery time (s)');axes[0].set_xlabel('Stable per-method case / side index');axes[0].legend(fontsize=6)
    axes[1].bar(list(states),list(states.values()),color='#777777',hatch='//');axes[1].set_ylabel('All required observed recovery records');axes[1].tick_params(axis='x',rotation=20)
    fig.suptitle('F04: recovery coverage; censored observations are not zero-time recoveries')
    save_figure(ctx,fig,'F04','recovery-coverage',['evaluation/recovery.csv'],'Full E08 recovery and censoring, original provisional thresholds',fields=['recovery','stall','method_id','case_id','side'])
    timeline=read_table(ctx,'representative_timeline')
    groups=collections.defaultdict(list)
    for r in timeline:groups[(r['experiment_id'],r['family'],r['state_mode'],r['case_id'])].append(r)
    for (exp,fam,mode,case),group in sorted(groups.items()):
        if exp not in ('E07','E08'):continue
        fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
        for arm in sorted({r['arm_id'] for r in group}):
            armrows=[r for r in group if r['arm_id']==arm]
            method=armrows[0]['method_id']; style='--' if 'primary-only' in arm else '-'
            for side in ('left','right'):
                data=[r for r in armrows if r.get('series')=='tracking' and r.get('side')==side and number(r,'source_time_s') is not None]
                data.sort(key=lambda r:number(r,'source_time_s'))
                if data:axes[0].plot([number(r,'source_time_s') for r in data],[number(r,'position_error_m') for r in data],color=COLORS[method],ls=style,marker='.' if side=='right' else None,markevery=max(1,len(data)//12),label=arm+'/'+side,lw=.8)
            native=[r for r in armrows if r.get('series')=='native' and number(r,'source_time_s') is not None]
            native.sort(key=lambda r:number(r,'source_time_s'))
            scaled=[r for r in native if number(r,'scale_min') is not None]
            if scaled:axes[1].plot([number(r,'source_time_s') for r in scaled],[number(r,'scale_min') for r in scaled],color=COLORS[method],ls=style,lw=.8,label=arm)
            for a,b in zip(native,native[1:]):
                if a.get('task_revision')!=b.get('task_revision') or a.get('secondary_enabled')!=b.get('secondary_enabled'):
                    for ax in axes:ax.axvline(number(b,'source_time_s'),color=COLORS[method],ls=':',lw=.5)
            constraints=[r for r in armrows if r.get('series')=='constraints' and number(r,'source_time_s') is not None and number(r,'position_violation_rad') is not None]
            constraints.sort(key=lambda r:number(r,'source_time_s'))
            if constraints:axes[2].plot([number(r,'source_time_s') for r in constraints],[number(r,'position_violation_rad') for r in constraints],color=COLORS[method],ls=style,lw=.8,label=arm)
        axes[0].set_ylabel('TCP position error (m)');axes[1].set_ylabel('Minimum evaluated task scale');axes[2].set_ylabel('Position violation (rad)');axes[2].set_xlabel('Original declared source time (s)')
        for ax in axes:
            handles,_=ax.get_legend_handles_labels()
            if handles:ax.legend(fontsize=5,ncol=3)
        fig.suptitle(case+' / '+str(mode)+' — fixed family representative; repeat 0; dotted lines = recorded task events',fontsize=8)
        save_figure(ctx,fig,'F03' if exp=='E07' else 'F04',fam+'-'+str(mode),['evaluation/representative_timeline.csv','evaluation/task_transitions.csv'], 'Original full window, extrema and all event transitions retained',selection={'case_id':case,'family':fam,'repeat_id':0,'state_mode':mode})

    if ctx.definition.get("schema_version", "analysis.v1") == "analysis.v2":
        import e07_evidence, e08_evidence
        e07_evidence.render(ctx)
        e08_evidence.render(ctx)


if __name__ == '__main__':raise SystemExit(C.cli(sys.modules[__name__],HERE/'definition.json'))
