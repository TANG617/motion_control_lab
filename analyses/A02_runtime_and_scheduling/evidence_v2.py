"""A02 v2 supplemental audits and guarded full-method timing; immutable v1 baseline counts."""
import collections
from pathlib import Path
from analysis import common as C
from repair.common import COMMON_ACCEPTANCE_HASH,evidence_class


def analyze(ctx,base_analyze,helpers):
    if len(ctx.units)!=5593:raise ValueError('A02 base coverage must remain 5593')
    base_analyze(ctx)
    classification=[]
    for u in ctx.units:
        cls=evidence_class(u)
        classification.append(dict(C.base_row(u),evidence_class=cls,
            scientific_timing_eligible=cls=='scientific-case-nonRT',
            reason='fixture/capability/anomaly retained separately; never relabel old failure'))
    ctx.table('base_evidence_classification',classification)
    audits=[];timing=[];quality=[];dimensions=[]
    supplement=getattr(ctx,'supplemental_units',[])
    if len(supplement)!=240:raise ValueError('A02 supplemental coverage must remain 24 audit + 216 timing')
    for u in supplement:
        report=C.unit_json(u,'common_servo_audit.json',{})
        row=dict(C.base_row(u),repair_stage=u['repair_stage'],workload=u['config'].get('workload'),
            observation=u['config'].get('observation'),scene_family=u.get('request',{}).get('input',{}).get('input_id','unavailable'),acceptance_contract_hash=report.get('acceptance_contract_hash'),
            canonical_problem_hash=report.get('canonical_problem_hash'),design_comparable=report.get('design_comparable',False),
            observation_complete=report.get('observation_complete',False),claim_eligible=report.get('claim_eligible',False),
            reasons=report.get('reasons',[u.get('reason','supplement unavailable')]),
            claim_scope='complete named methods, not isolated backend',source_artifacts=report.get('source_artifacts',[]))
        audits.append(row)
        if u['repair_stage']!='E09-timing':continue
        raw=C.unit_path(u,'raw.jsonl')
        if not raw:continue
        attempts=[];dims=collections.Counter()
        for r in C.jsonl_rows(raw):
            if r.get('record_type')!='attempt':continue
            attempts.append({k:r.get(k) for k in ('attempt_sequence','window_id','start_ns','finish_ns','ik_call_time_ms','cpu_time_s','solution_quality')})
            dims[tuple(r.get(k) for k in helpers.DIMENSIONS)]+=1
        errors=helpers.check_errors(u)
        for r in attempts:r.update(errors.get(r['attempt_sequence'],{}))
        for t in helpers.timing_summary(attempts,u['config']):timing.append(dict(row,**t))
        quality.append(dict(row,**helpers.quality_summary(attempts,1+u['config']['warmup_calls']+u['config']['measured_calls'])))
        for d,n in dims.items():dimensions.append(dict(row,**dict(zip(helpers.DIMENSIONS,d)),attempts=n))
    ctx.table('supplemental_audit',audits);ctx.table('supplemental_timing',timing)
    ctx.table('supplemental_quality',quality);ctx.table('supplemental_dimensions',dimensions)
    groups=collections.defaultdict(list)
    for r in timing:
        if r['window']=='steady':groups[(r['input_hash'],r['model_hash'],r['workload'],r['observation'],r['repeat_id'])].append(r)
    pairs=[]
    for key,rs in sorted(groups.items()):
        reasons=[]
        if len(rs)!=2 or len({r['method_id'] for r in rs})!=2:reasons.append('missing-or-duplicate-full-method')
        for r in rs:
            if not r['claim_eligible'] or not r['design_comparable'] or not r['observation_complete']:reasons.append('hash-bound-native-audit-not-eligible')
            if r['acceptance_contract_hash']!=COMMON_ACCEPTANCE_HASH:reasons.append('external-contract-mismatch')
            if r['native_status']!='completed' or r['validation_status']!='passed' or not r['all_calls_retained']:reasons.append('native-or-observation-failed')
        left,right=(sorted(rs,key=lambda r:r['method_id'])+[{}]*2)[:2]
        pairs.append(dict(input_hash=key[0],model_hash=key[1],workload=key[2],observation=key[3],repeat_id=key[4],
            left_method_id=left.get('method_id'),right_method_id=right.get('method_id'),left_unit=left.get('unit_id'),right_unit=right.get('unit_id'),
            admitted=not reasons,reasons=sorted(set(reasons)),acceptance_contract_hash=COMMON_ACCEPTANCE_HASH,
            left_p50_ms=left.get('p50_ms'),right_p50_ms=right.get('p50_ms'),
            complete_method_median_time_ratio=helpers.ratio(left.get('p50_ms'),right.get('p50_ms')) if not reasons else None,
            ratio_numerator='left_method_id steady-window per-process P50 IK time (ms)',
            ratio_denominator='right_method_id steady-window per-process P50 IK time (ms)',
            ratio_interpretation='left_p50_ms / right_p50_ms; >1 means left method slower; <1 means left method faster; 1 means equal observed medians',
            left_native_quality_counts=left.get('native_quality_counts'),right_native_quality_counts=right.get('native_quality_counts'),
            claim_scope='full named method under common audited equations; not isolated backend',
            exclusion_policy='hash-bound actual native evidence; no fixed-name cross-method block'))
    ctx.table('supplemental_method_pairs',pairs)
    counts=collections.Counter(r['evidence_class'] for r in classification)
    text=['\n## v2 定向补证与证据分类','',
        f'旧批次仍为 {len(ctx.units)} 单元；新增 {len(supplement)} 单元单独计数（24 项全快照 native 数学审计、216 项计时）。',
        f'旧证据分类：`{dict(counts)}`。E10 fixture-observation-overflow 的 cap8 失败属于预期的观测容量 fixture；旧 failed 状态保留，v2 科学计时图排除所有 fixture、能力缺项与未解决异常。',
        f'新增审计/计时中 claim_eligible={sum(r["claim_eligible"] for r in audits)} / {len(audits)}；steady 方法对照 {len(pairs)} 对，其中 {sum(r["admitted"] for r in pairs)} 对具备完整的哈希绑定外部合同与原生数学审计。',
        '新方法不按名称自动获准。只有实际 native task A/b、归一化/权重、硬边界、regularization 与独立 XML 模型重建共同数学问题一致，且全 20 快照完成，才开放对应计时。原生拒绝不被共同合同豁免。',
        '通过的比较仅说明完整命名方法在同一共同合同下的非 RT 观测，不隔离 backend 因果贡献。旧 MCC/PlaCo 身份的既有准入限制保持原样。',
        '配对指标 `complete_method_median_time_ratio = left_p50_ms / right_p50_ms`：分子为该行 `left_method_id` 的 steady 窗口逐进程 P50 IK 耗时，分母为 `right_method_id` 的同条件耗时（均为 ms）。大于 1 表示左侧方法更慢，小于 1 表示左侧方法更快，等于 1 表示已观测中位数相同；未准入或分母为零时不提供比值。left/right 按显式方法身份排序，不暗示优劣。',
        '', '[旧证据分类](evaluation/base_evidence_classification.csv) · [新增审计](evaluation/supplemental_audit.csv) · [新增计时](evaluation/supplemental_timing.csv) · [完整方法配对](evaluation/supplemental_method_pairs.csv)','']
    report=ctx.output/'report.md'
    base_text=report.read_text()
    header=['# A02 v2：旧批次与定向补证分别评价','',
        f'**旧批次 {len(ctx.units)} 单元；新增 E09 {len(supplement)} 单元（24 审计 + 216 计时），独立统计，不合并分母。**','',
        '**E10 cap8 是预期失败 fixture。** `fixture-observation-overflow` 的旧 `failed` 状态完整保留；它用于验证观测溢出，v2 科学计时图排除 fixture，源表与完整性账目保留。','',
        '**新共同方法仅依据哈希绑定的实际 native 数学审计准入。** `mcc-weighted-common-servo-v1` 与 `placo-common-servo-v1` 不继承旧方法身份的统一阻止规则，也不凭新名称获准；只有固定审计与共同外部接受合同满足条件才生成完整方法对照。原生拒绝始终保留，不宣称隔离 backend 收益。','',
        '下文“旧批次”部分的状态与方法限制只适用于原 5,593 个单元。新增 240 个单元的准入、失败和结果见后续“v2 定向补证”部分；所有结果均为非 RT 探索性观察。','',
        '[新增审计与准入](evaluation/supplemental_audit.csv) · [新增完整方法对照](evaluation/supplemental_method_pairs.csv) · [旧证据分类](evaluation/base_evidence_classification.csv)','',
        '## 旧批次：5,593 单元的原始分析','']
    if base_text.startswith('# '):base_text=base_text.split('\n',1)[1] if '\n' in base_text else base_text
    report.write_text('\n'.join(header)+base_text+'\n'+'\n'.join(text))
    C.write_json(ctx.output/'evaluation/v2_validation.json',dict(base_units=len(ctx.units),supplemental_units=len(supplement),
        base_evidence_classes=dict(counts),supplemental_counts=dict(collections.Counter(u['repair_stage'] for u in supplement)),
        native_apps_launched=0,base_statuses_unchanged=True,scientific_status='exploratory nonRT; formal claims deferred'))


OBSERVATIONS=('minimal','full','cpp-new')
FAMILY_MARKERS={'e09-interior-motion-v2':'o','e09-near-boundary-motion-v2':'s',
                'e09-active-set-change-motion-v2':'^'}
METHOD_COLORS={'mcc-weighted-common-servo-v1':'#0072B2','placo-common-servo-v1':'#D55E00'}


def render(ctx):
    """Table-only plot: observation columns, quantile rows, unpooled process points."""
    plt=C.plotting();rows=list(C.csv_rows(ctx.output/'evaluation/supplemental_timing.csv'))
    eligible=[r for r in rows if r['window']=='steady' and r['claim_eligible']=='True' and r.get('p50_ms') and r.get('p99_ms')]
    for r in eligible:
        if r.get('observation') not in OBSERVATIONS or r.get('scene_family') not in FAMILY_MARKERS or r.get('method_id') not in METHOD_COLORS:
            raise ValueError('supplemental figure identity outside fixed observation/family/method declaration')
    fig,axes=plt.subplots(2,3,figsize=(13,7),sharex=True,sharey='row')
    for column,observation in enumerate(OBSERVATIONS):
        panel=[r for r in eligible if r['observation']==observation]
        for row_index,metric in enumerate(('p50_ms','p99_ms')):
            ax=axes[row_index,column]
            for method,color in METHOD_COLORS.items():
                for family,marker in FAMILY_MARKERS.items():
                    rs=[r for r in panel if r['method_id']==method and r['scene_family']==family]
                    if rs:ax.scatter([int(r['workload']) for r in rs],[float(r[metric]) for r in rs],
                        color=color,marker=marker,s=20,alpha=.6,linewidths=.45,
                        label=method+' / '+family)
            ax.set_xticks(range(4));ax.set_xlim(-.3,3.3)
            if row_index==0:ax.set_title('Observation: '+observation)
            else:ax.set_xlabel('Frozen workload')
            if column==0:ax.set_ylabel('Per-process '+('P50' if row_index==0 else 'P99')+' IK (ms)')
            if not panel:ax.text(.5,.5,'No eligible timing\nAudit failures retained',ha='center',va='center',transform=ax.transAxes)
    from matplotlib.lines import Line2D
    handles=[Line2D([],[],color=color,marker='o',linestyle='None',label=method) for method,color in METHOD_COLORS.items()]
    handles += [Line2D([],[],color='#555555',marker=marker,linestyle='None',label=family) for family,marker in FAMILY_MARKERS.items()]
    fig.legend(handles=handles,loc='lower center',ncol=3,fontsize=7,frameon=False)
    fig.suptitle('E09 supplement | full named methods, non-RT | each point is one process repeat',fontsize=11)
    fig.subplots_adjust(top=.89,bottom=.16,hspace=.15,wspace=.08)
    selection=dict(window='steady',observations=list(OBSERVATIONS),
        family_markers=FAMILY_MARKERS,method_colors=METHOD_COLORS,
        independent_display_unit='one process repeat; no repeat pooling or averaged P99',
        selected_processes=[{k:r[k] for k in ('unit_id','input_hash','model_hash','scene_family','observation','repeat_id','workload','method_id')} for r in sorted(eligible,key=lambda r:r['unit_id'])],
        admitted_process_count=len(eligible),all_source_rows=len(rows),
        rejection_policy='failed, missing or ineligible units remain in audit and timing tables')
    C.save_figure(ctx,fig,'F05','supplemental-common-methods',['evaluation/supplemental_timing.csv','evaluation/supplemental_method_pairs.csv','evaluation/supplemental_audit.csv'],
        'Observation-faceted, input-family markers and fixed method colors; all eligible process repeats shown without pooling',
        ['workload','p50_ms','p99_ms','observation','scene_family','input_hash','model_hash','repeat_id','unit_id','method_id','claim_eligible','acceptance_contract_hash'],selection)
