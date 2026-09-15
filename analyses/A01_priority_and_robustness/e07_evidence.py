"""A01 v2 E06 common admission and E07 offline evidence integration."""
from pathlib import Path
import collections
import hashlib
import importlib.util
import json
import numpy as np
from analysis import common as C
from repair.common import evidence_class
from metrics import secondary_gain
HERE=Path(__file__).resolve().parent
MODULE=HERE.parents[1]/'experiments/E07_hqp_priority_and_redundancy/reanalyze_evidence.py'
spec=importlib.util.spec_from_file_location('e07_independent_repair',MODULE);R=importlib.util.module_from_spec(spec);spec.loader.exec_module(R)

def paired_costs(units,rows):
    lookup={(r['unit_id'],r['task']):r for r in rows};result=[]
    for u in units:
        c=u['config']
        if c.get('primary_only'):continue
        references=[p for p in units if p['case_id']==u['case_id'] and p['method_id']==u['method_id'] and p['repeat_id']==u['repeat_id'] and p['config'].get('primary_only')]
        tasks=[r for r in rows if r['unit_id']==C.unit_id(u) and r['secondary']]
        if not tasks:tasks=[dict(task='secondary-unavailable',weighted_cost_mean=None,raw_cost_mean=None,complete=False)]
        for candidate in tasks:
            reasons=[] if len(references)==1 else ['missing-or-duplicate-primary-only-reference']
            if not candidate.get('declared',True):reasons.append('task-not-declared-by-full-configuration')
            reference=lookup.get((C.unit_id(references[0]),candidate['task'])) if len(references)==1 else None
            if reference is None:reasons.append('reference-task-cost-unavailable')
            if len(references)==1:reasons+=C.compatible(references[0],u,('primary_only','secondary_tasks'))
            if not candidate['complete'] or reference and not reference['complete']:reasons.append('incomplete-declared-window')
            mode=c.get('state_mode')
            if reference and mode=='snapshot' and reference['linearization_sequence_hash']!=candidate['linearization_sequence_hash']:reasons.append('snapshot-linearization-sequence-mismatch')
            for flavor in ('raw','weighted'):
                rcost=reference.get(flavor+'_cost_mean') if reference else None;cost=candidate.get(flavor+'_cost_mean')
                values=secondary_gain(rcost,cost) if not reasons and rcost is not None and cost is not None else dict(absolute=None,relative=None,status='not-applicable-task' if 'task-not-declared-by-full-configuration' in reasons else 'unavailable')
                result.append(dict(C.base_row(u),task=candidate['task'],cost_definition=flavor,reference_unit_id=C.unit_id(references[0]) if len(references)==1 else None,reference_cost=rcost,candidate_cost=cost,absolute_gain=values['absolute'],relative_gain=values['relative'],status=values['status'],reasons=reasons,denominator='all declared attempts, process mean',reference_samples=reference.get('measured_count') if reference else None,candidate_samples=candidate.get('measured_count'),interpretation='same linearization snapshot sequence' if mode=='snapshot' else 'each method own evolving states over full declared window; not same-state nullspace gain'))
    return result

def reduce_unit(u):
    from threadpoolctl import threadpool_limits, threadpool_info
    with threadpool_limits(limits=1):
        result=_reduce_unit(u)
        libraries=sorted(threadpool_info(),key=lambda row:row.get('filepath',''))
        for row in result[2]:row['numerical_libraries']=libraries
        return result

def _reduce_unit(u):
    allcost=[];references=[];coverage=[];cache={}
    request=C.unit_json(u,'request.json',{});data=request.get('input',{});c=u['config'];base=C.base_row(u);p=C.unit_path(u,'raw.jsonl');aggs={};count=0;statehash=hashlib.sha256();accepted=0
    fk=None if data.get('analytic') or not p else R.CachedGeometry(data['model']['locator'])
    for line,row in enumerate(C.jsonl_rows(p) if p else [],1):
        if row.get('record_type') not in ('analytic','attempt'):continue
        count+=1;accepted+=bool(row.get('committed') or row.get('accepted'))
        statehash.update(json.dumps([row.get('input_sequence'),row.get('input_state'),row.get('targets'),row.get('problem')],sort_keys=True,separators=(',',':')).encode())
        tasks=R.task_rows(data,c,row,fk);x=R.output_vector(data,row)
        for cost in R.costs(tasks,x):
            a=aggs.setdefault(cost['task'],dict(task=cost['task'],secondary=cost['secondary'],declared=cost['declared'],weight=cost['weight'],raw_cost_sum=0.,weighted_cost_sum=0.,measured_count=0,solve_disabled_count=0))
            a['solve_disabled_count']+=not cost['solve_enabled']
            if cost['raw_cost'] is not None:
                a['measured_count']+=1;a['raw_cost_sum']+=cost['raw_cost'];a['weighted_cost_sum']+=cost['weighted_cost']
        if R.reference_selected(data,c,row):
            lo,hi=R.velocity_bounds(data,c,row)
            tasklist=[dict(t,A=t['A'].tolist(),b=t['b'].tolist()) for t in tasks]
            key=C.stable_hash(dict(tasks=tasklist,lower=lo.tolist(),upper=hi.tolist(),regularization=c['regularization'],preservation=c['preservation_tolerance'],hard=c['hard_tolerance'],method=u['method_id']))
            if key not in cache:cache[key]=R.sequential_reference(tasks,lo,hi,c['regularization'],c['preservation_tolerance'],c['hard_tolerance'],u['method_id'])
            ref=cache[key];gaps=[]
            if x is not None:
                for level in ref['levels']:
                    A=np.asarray(level['A'],float).reshape(-1,len(x))
                    soft_cost=float(.5*np.sum((A@x-np.asarray(level['b'],float))**2))
                    gap=soft_cost-level['objective']
                    regularized_gap=soft_cost+.5*c['regularization']*float(x@x)-level['regularized_objective']
                    gaps.append(dict(level=level['level'],objective_gap=gap,soft_objective_gap=gap,regularized_objective_gap=regularized_gap,reference_status=level['status']))
            references.append(dict(base,source_line=line,raw_path=str(p),attempt_sequence=row.get('attempt_sequence'),source_time_s=row.get('source_time_s'),reference_status=ref['status'],reference_reason=ref['reason'],reference_key=key,reference=ref,native_equivalence_status='unattested-library-source-binding',native_source_inventory_path=str(Path(u['directory']).parents[4]/'source_inventory.json'),native_source_issue_id='provenance-Core-source-inventory-old',native_optimality_verdict='unavailable',reference_contract='independent convex model; source policy recoverable via frozen clean git revision; native library/source binding unattested',reference_preservation_band_fraction=.5,candidate_objective_gaps=gaps,native_committed=row.get('committed'),candidate_native_accepted=row.get('accepted'),candidate_output_available=x is not None,reference_selection='all nonfixture snapshots; declared task-switch instants in evolving',original_validation_unchanged=True))
    expected=c.get('steps',0);complete=count==expected and count>0
    for a in aggs.values():
        measured=a['measured_count'];allcost.append(dict(base,**a,raw_cost_mean=a['raw_cost_sum']/measured if measured else None,weighted_cost_mean=a['weighted_cost_sum']/measured if measured else None,observed_count=count,expected_count=expected,complete=complete and (measured==expected or not a['declared']),metric_status='measured' if a['declared'] and measured==expected else 'not-applicable-undeclared-task' if not a['declared'] else 'unavailable',linearization_sequence_hash=statehash.hexdigest(),state_mode=c.get('state_mode'),native_accepted_count=accepted,evidence_class=evidence_class(u),cost_equation='0.5 || A(q_input) v_recorded - gain*target_error(q_input) ||^2; weighted multiplies task weight',output_semantics='recorded output, including held measured velocity on rejected R1 attempts',source_raw=str(p)))
    coverage.append(dict(base,expected_attempts=expected,observed_attempts=count,task_rows=len(aggs),complete=complete,evidence_class=evidence_class(u),status='evidence-recomputed' if complete else 'still-insufficient'))
    return allcost,references,coverage


def recover_source_policy(ctx):
    import hashlib
    import subprocess
    source=next(s for s in ctx.definition['sources'] if s['experiment_id']=='E07')
    manifest=C.read_json(source['manifest']['locator']);inventory=manifest['source_inventory'];ctx.check_artifact(inventory)
    core=next(s for s in C.read_json(inventory['locator']) if s['scope']=='/workspace/components/motion_control_core')
    outputs=[];directory=ctx.output/'evaluation/provenance';directory.mkdir(parents=True,exist_ok=True)
    for name in ('hierarchical_problem.cpp','requirement_compiler.cpp'):
        relative='components/motion_control_core/src/optimization/'+name
        command=['git','-C',core['git_root'],'show',core['revision']+':'+relative]
        blob=subprocess.run(command,capture_output=True)
        oid_command=['git','-C',core['git_root'],'rev-parse',core['revision']+':'+relative]
        oid=subprocess.run(oid_command,capture_output=True,text=True)
        current=C.artifact(Path(core['git_root'])/relative);ctx.check_artifact(current)
        row=dict(path=relative,command=command,oid_command=oid_command,git_show_exit_code=blob.returncode,git_oid_exit_code=oid.returncode,current_source=current)
        if blob.returncode==0 and oid.returncode==0:
            path=directory/name;path.write_bytes(blob.stdout);digest=hashlib.sha256(blob.stdout).hexdigest()
            row.update(status='recovered-from-frozen-revision',git_blob_oid=oid.stdout.strip(),sha256=digest,reconstructed_source=C.artifact(path),current_bytes_equal=digest==current['sha256'])
        else:row.update(status='unavailable',error=blob.stderr.decode(errors='replace') or oid.stderr)
        outputs.append(row)
    plan=C.read_json(ctx.definition['campaign']['plan']['locator']);ctx.check_artifact(plan['runtime_inventory'])
    runtime=C.read_json(plan['runtime_inventory']['locator'])['mcl_study_e07']
    result=dict(schema_version='source_policy_recovery.v1',issue_id='provenance-Core-source-inventory-old',old_source_inventory=inventory,
        frozen_git_revision=core['revision'],frozen_scoped_dirty_status=core['dirty_status'],files=outputs,
        source_policy_status='recovered' if all(r['status']=='recovered-from-frozen-revision' for r in outputs) else 'partial-or-unavailable',
        installed_runtime_inventory=plan['runtime_inventory'],recorded_native_runtime=runtime,
        native_library_source_binding='unattested: runtime hashes identify binaries; no frozen build attestation binds these source bytes to those hashes')
    C.write_json(ctx.output/'evaluation/e07_source_policy_recovery.json',result)
    return result


def e06_summary(unit):
    audit=C.unit_json(unit,'common_servo_audit.json',{})
    checks=C.unit_json(unit,'common_servo_checks.json',[])
    native=list(C.jsonl_rows(C.unit_path(unit,'raw.jsonl'))) if C.unit_path(unit,'raw.jsonl') else []
    attempts=[r for r in native if r.get('record_type')=='attempt']
    tasks=[t for r in checks for t in r.get('tasks',[])]
    result=dict(C.base_row(unit),repair_stage=unit.get('repair_stage'),source_scope='supplement only; old E06 remains separate',
        design_comparable=audit.get('design_comparable',False),observation_complete=audit.get('observation_complete',False),claim_eligible=audit.get('claim_eligible',False),
        expected_attempts=unit['config'].get('steps'),observed_attempts=len(attempts),audit_attempts=len(checks),task_checks=len(tasks),
        native_accepted_count=sum(r.get('native_accepted') is True for r in attempts),
        candidate_available_count=sum(r.get('candidate_available') is True for r in attempts),
        common_quality_passed_count=sum(r.get('common_quality_passed') is True for r in attempts),
        committed_count=sum(r.get('committed') is True for r in attempts),
        bounds_match_count=sum(r.get('bounds_passed') is True for r in checks),
        max_A_gap=max((t['max_A_gap'] for t in tasks),default=None),
        max_target_velocity_gap=max((t['max_b_velocity_gap'] for t in tasks),default=None),
        max_A_audit_ratio=max((t['max_A_gap']/t['tolerance'] for t in tasks),default=None),
        max_target_audit_ratio=max((t['max_b_velocity_gap']/t['tolerance'] for t in tasks),default=None),
        acceptance_contract_hash=audit.get('acceptance_contract_hash'),reasons=audit.get('reasons',['missing-native-audit']),
        audit_scope='native actual rows versus independent declaration; quality and commits counted separately',
        source_artifacts=[a for a in unit.get('artifacts',[]) if Path(a['locator']).name in ('raw.jsonl','common_servo_checks.json','common_servo_audit.json')])
    return result


def analyze(ctx):
    from concurrent.futures import ProcessPoolExecutor
    from importlib.metadata import version
    recovery=recover_source_policy(ctx)
    core=Path('/workspace/components/motion_control_core')
    policy_sources=[C.artifact(core/'src/optimization'/name) for name in ('hierarchical_problem.cpp','requirement_compiler.cpp')]
    for source in policy_sources:ctx.check_artifact(source)
    C.write_json(ctx.output/'evaluation/e07_reference_contract.json',dict(schema_version='e07_reference_contract.v1',source_artifacts=policy_sources,
        native_equivalence='unattested-library-source-binding',source_policy_recovery=C.artifact(ctx.output/'evaluation/e07_source_policy_recovery.json'),issue_id='provenance-Core-source-inventory-old',
        regularization='0.5 lambda ||v||² at each pass, center zero',preservation_band_fraction=.5,
        hard_tolerance='original declared tolerance unchanged',kkt_stationarity_tolerance=1e-6,kkt_complementarity_tolerance=1e-9,
        packages={name:version(name) for name in ('numpy','scipy','threadpoolctl')},
        reference_optimizer=dict(api='scipy.optimize.minimize',method='SLSQP',analytic_gradient=True,ftol=1e-12,maxiter=200),
        kkt_optimizer=dict(api='scipy.optimize.lsq_linear',method='trf',lsq_solver='dense exact (default resolution)',tol=1e-12,lsmr_tol=1e-12,max_iter=200,nonnegative_multipliers=True),
        parallelism=dict(worker_count=min(16,max(1,sum(u['experiment_id']=='E07' for u in ctx.units))),worker_policy='ProcessPoolExecutor.map chunksize 1; deterministic source order',per_worker_blas_threads=1,actual_library_versions_and_threads='evaluation/e07_repair_coverage.csv numerical_libraries, recorded inside each worker thread limit',thread_limit_api='threadpoolctl.threadpool_limits',native_apps_started=0)))
    units=sorted([u for u in ctx.units if u["experiment_id"]=="E07"],key=C.unit_id)
    allcost=[];references=[];coverage=[]
    with ProcessPoolExecutor(max_workers=min(16,max(1,len(units)))) as pool:
        for ix,(a,b,c) in enumerate(pool.map(reduce_unit,units,chunksize=1),1):
            allcost.extend(a);references.extend(b);coverage.extend(c)
            if ix%10==0:ctx.progress("E07-recompute",ix,len(units))
    ctx.table('e07_actual_task_costs',allcost);ctx.table('e07_actual_primary_only_effects',paired_costs(units,allcost));ctx.table('e07_independent_references',references);ctx.table('e07_repair_coverage',coverage)
    audit=[]
    for u in sorted([u for u in ctx.supplemental_units if u['experiment_id']=='E06'],key=C.unit_id):
        a=C.unit_json(u,'common_servo_audit.json',{});audit.append(dict(C.base_row(u),repair_stage=u['repair_stage'],design_comparable=a.get('design_comparable',False),observation_complete=a.get('observation_complete',False),claim_eligible=a.get('claim_eligible',False),acceptance_contract_hash=a.get('acceptance_contract_hash'),reasons=a.get('reasons',['missing-native-audit']),audit_path=str(C.unit_path(u,'common_servo_audit.json')),claim_scope=a.get('claim_scope')))
    ctx.table('e06_common_admission',audit)
    ctx.table('e06_common_numeric_summary',[e06_summary(u) for u in sorted(ctx.supplemental_units,key=C.unit_id) if u['experiment_id']=='E06'])
    e06=[u for u in ctx.supplemental_units if u['experiment_id']=='E06'];pairs=[]
    import itertools
    for left,right in itertools.combinations(e06,2):
        if (left['case_id'],left['repeat_id'])!=(right['case_id'],right['repeat_id']):continue
        reasons=C.compatible(left,right,())
        pairs.append(dict(case_id=left['case_id'],left_unit=C.unit_id(left),right_unit=C.unit_id(right),admitted=not reasons,reasons=reasons,claim='complete methods under common contract; not isolated backend'))
    ctx.table('e06_common_pairs',pairs)
    ctx.table('evidence_classification',[dict(C.base_row(u),origin='base',evidence_class=evidence_class(u)) for u in ctx.units]+[dict(C.base_row(u),origin=u['repair_stage'],evidence_class=evidence_class(u)) for u in ctx.supplemental_units])

def render(ctx):
    def read(name):return list(C.csv_rows(ctx.output/'evaluation'/(name+'.csv')))
    costs=read('e07_actual_task_costs');pairs=read('e07_actual_primary_only_effects');refs=read('e07_independent_references');coverage=read('e07_repair_coverage');audit=read('e06_common_admission');e06=read('e06_common_numeric_summary')
    recovery=C.read_json(ctx.output/'evaluation/e07_source_policy_recovery.json')
    recovered=sum(r['status']=='recovered-from-frozen-revision' for r in recovery['files']);same=sum(r.get('current_bytes_equal',False) for r in recovery['files'])
    lines=['# E06/E07 定向补证','',f'旧 E07 全部 {len(coverage)} 单元按原声明窗口重算；新增 E06 {len(audit)} 单元单独统计，不与旧批次拼接。','',
        '## E06 共同 ServoStep','',f'实际方程设计准入 {sum(r["design_comparable"]=="True" for r in audit)}/{len(audit)}；观测完整 {sum(r["observation_complete"]=="True" for r in audit)}/{len(audit)}。共同接受合同只支持完整方法比较，不支持纯 backend 因果结论。',
        '[实际矩阵误差、原生接受/候选/共同质量/提交计数](evaluation/e06_common_numeric_summary.csv) · [逐单元实际方程准入](evaluation/e06_common_admission.csv) · [逐 case 受控条件准入](evaluation/e06_common_pairs.csv)','',
        '## E07 实际 primary-only 成本','',f'重算 {len(costs)} 个逐单元任务成本，包含 primary-only 求解禁用任务；完整配置未声明的 link4/posture 等保留 not-applicable，不测量为零。配对产生 {len(pairs)} 个原始/加权成本结果，其中 {sum(r["status"]=="ok" for r in pairs)} 项有完整非零参考。零分母、缺项及未完整窗口独立标记。',
        'snapshot 比较相同线性化状态序列。演化窗口使用每个方法自身实际状态，结果表示完整轨迹表现，不表示同状态瞬时零空间收益。原生拒绝后的 HOLD 速度按原记录测量，不能称为新的求解候选。','',
        '[全部任务成本](evaluation/e07_actual_task_costs.csv) · [真实 primary-only 配对](evaluation/e07_actual_primary_only_effects.csv) · [完整覆盖](evaluation/e07_repair_coverage.csv)','',
        '## 独立逐层参考','',f'覆盖 {len(refs)} 个原生观测时刻，其中 {sum(r["reference_status"]=="certified" for r in refs)} 个获得参考优化器收敛、原硬可行性、独立 KKT 驻点与互补性证书。未收敛或证书不足保持 unavailable。',
        '参考按原声明硬界、当前源码保持带半宽 0.5 × 声明保持容差逐层求解；按目标值和可行性比较多解，不强制关节向量一致。KKT 参考驻点容差为 1e-6，互补性容差为 1e-9；这些值仅评估离线优化器，不改变候选硬约束或旧 validation。',
        f'问题 provenance-Core-source-inventory-old：旧逐文件清单遗漏 Core 优化器源码。本 run 从冻结 git revision 恢复 {recovered}/2 个策略源文件，其中 {same}/2 与当前源码字节一致；恢复文件、原 scoped dirty_status、命令和哈希另存。库与 app 的历史运行哈希明确；缺少编译证明将这些源字节绑定到旧库哈希，故该绑定仍为 unattested。数值证书适用于恢复策略对应的离线问题；旧原生最优性裁定保持 unavailable，不能把目标差直接解释成算法优劣。',
        '[每个参考证书与原 raw 行号](evaluation/e07_independent_references.csv) · [参考合同及策略源码哈希](evaluation/e07_reference_contract.json) · [源码恢复与编译绑定边界](evaluation/e07_source_policy_recovery.json)','',
        '## 证据解释','', '旧批次及新补跑是不同构建/环境身份。补证分类与原执行/核验状态并列保留；本轮均为非 RT 开发证据，不能支持实时尾延迟结论或总体优胜排名。','']
    (ctx.output/'report_E06_E07_evidence.md').write_text('\n'.join(lines))
    report=ctx.output/'report.md';text=report.read_text();marker='## 定向补证（analysis.v2）'
    if marker in text:text=text.split(marker)[0]
    prefix_title='# A01 v2：定向补证入口'
    separator='\n---\n\n'
    if text.startswith(prefix_title):text=text.split(separator,1)[1]
    classification=read('evidence_classification')
    old_count=sum(r['origin']=='base' for r in classification)
    supplemental=[r for r in classification if r['origin']!='base']
    e06_count=sum(r['experiment_id']=='E06' for r in supplemental);e08_count=sum(r['experiment_id']=='E08' for r in supplemental)
    common_pairs=read('e06_common_pairs')
    navigation=[prefix_title,'',
        f'本轮覆盖旧批次 **{old_count} 个单元**与新增补证 **{len(supplemental)} 个单元**（E06：{e06_count}；E08：{e08_count}），两部分分别统计。','',
        '**直接阅读：[E06/E07 补证与当前准入](report_E06_E07_evidence.md) · [E08 异常归因](report_E08_evidence.md)。**','',
        f'后续旧 v1 章节、旧 F02 图和旧准入结论仅适用于旧方法身份。新增 {len(common_pairs)} 组共同 ServoStep 对照采用新身份，准入及实际方程证据见 [共同对照表](evaluation/e06_common_pairs.csv) 与 [补证 F02](figures/F02-common-servo-supplement.png)。','',
        '新证据保留原始失败和科学限制；不同批次不合并为一次同环境实验。']
    report.write_text('\n'.join(navigation)+separator+text.rstrip()+'\n')
    plt=C.plotting()
    if e06:
        fig,axes=plt.subplots(1,3,figsize=(17,max(5,len(e06)*.34)),gridspec_kw={'width_ratios':[1,1.35,1.6]})
        labels=[r['case_id']+' / '+r['method_id'] for r in e06];yy=np.arange(len(e06))
        stages=['design_comparable','observation_complete','claim_eligible'];matrix=np.array([[r[k]=='True' for k in stages] for r in e06],float)
        axes[0].imshow(matrix,vmin=0,vmax=1,cmap='RdYlGn',aspect='auto');axes[0].set_yticks(yy,labels,fontsize=6);axes[0].set_xticks(range(3),['Design','Observed','Claim'],rotation=30,fontsize=8)
        for i,r in enumerate(matrix):
            for j,v in enumerate(r):axes[0].text(j,i,'P' if v else 'X',ha='center',va='center',fontsize=8,color='white')
        counts=['native_accepted_count','candidate_available_count','common_quality_passed_count','committed_count'];values=np.array([[int(r[k]) for k in counts] for r in e06])
        axes[1].imshow(values,aspect='auto',cmap='Blues',vmin=0,vmax=max(1,int(np.max(values))));axes[1].set_yticks(yy,[]);axes[1].set_xticks(range(4),['Native accepted','Candidate','Common quality','Committed'],rotation=30,ha='right',fontsize=8)
        for i,r in enumerate(values):
            for j,v in enumerate(r):axes[1].text(j,i,str(v)+'/'+e06[i]['observed_attempts'],ha='center',va='center',fontsize=8,color='white' if v>.5*max(1,int(np.max(values))) else 'black')
        for key,marker,label in [('max_A_audit_ratio','o','Task matrix / original audit tolerance'),('max_target_audit_ratio','x','Target velocity / original audit tolerance')]:
            audit_points=[(float(r[key]),i) for i,r in enumerate(e06) if r.get(key)]
            if audit_points:axes[2].scatter([max(x,1e-17) for x,i in audit_points],[i for x,i in audit_points],s=18,marker=marker,label=label)
        axes[2].set_xscale('log');axes[2].axvline(1.,color='black',ls='--',lw=.7);axes[2].set_ylim(len(e06)-.5,-.5);axes[2].set_yticks(yy,[]);axes[2].set_xlabel('Numerical audit ratio; 1 = original tolerance');axes[2].legend(fontsize=7,loc='upper right')
        axes[0].set_title('Three admission stages');axes[1].set_title('Distinct counts / observed attempts');axes[2].set_title('All task equations; coefficient and velocity conversion')
        fig.suptitle('F02 supplement: six cases × two common ServoStep methods; repeat 0; old batch separate',fontsize=11);fig.tight_layout()
        C.save_figure(ctx,fig,'F02','common-servo-supplement',['evaluation/e06_common_numeric_summary.csv','evaluation/e06_common_pairs.csv'],'All twelve supplemental E06 units; original batch remains separately plotted. Hash-bound raw/check/audit artifacts in source_artifacts CSV field. Display only: zero audit ratio plotted at 1e-17 on logarithmic axis; numerical tables unchanged.',
            fields=['design_comparable','observation_complete','claim_eligible','native_accepted_count','candidate_available_count','common_quality_passed_count','committed_count','max_A_gap','max_target_velocity_gap','bounds_match_count','source_artifacts'],
            selection={'case_ids':sorted({r['case_id'] for r in e06}),'repeat_id':0,'scope':'all supplemental E06 units','window':'full original declared snapshot'})
    valid=[r for r in pairs if r['status'] in ('ok','not-applicable') and r['cost_definition']=='weighted'];tasks=sorted({r['task'] for r in valid})
    fig,axes=plt.subplots(max(1,len(tasks)),1,figsize=(10,max(4,len(tasks)*2.5)),squeeze=False)
    for ax,task in zip(axes[:,0],tasks):
        groups=collections.defaultdict(list)
        for r in valid:
            if r['task']==task:groups[(r['method_id'],r['interpretation'])].append(float(r['absolute_gain']))
        labels=[]
        for i,(key,values) in enumerate(sorted(groups.items())):
            labels.append(key[0]+' / '+('snapshot' if key[1].startswith('same') else 'evolving'));ax.scatter([i]*len(values),values,s=9,alpha=.55)
        ax.axhline(0,color='black',lw=.7);ax.set_xticks(range(len(labels)),labels,rotation=12,ha='right',fontsize=6);ax.set_ylabel('Cost gain');ax.set_title(task+'; units: '+('m²/s²' if 'elbow' in task else 'problem units²' if 'analytic' in task else 'rad²/s²'),fontsize=9)
    fig.suptitle('F03 supplement: actual primary-only; all complete pairs; task units separate',fontsize=10);fig.tight_layout()
    C.save_figure(ctx,fig,'F03','actual-primary-only-repair',['evaluation/e07_actual_primary_only_effects.csv'],'All complete pairs, full source windows; zero-reference absolute gains retained; no relative view',fields=['absolute_gain','method_id','interpretation','status','cost_definition','task'],selection={'cases':'all eligible declared cases','window':'full original declaration','repeat':'all declared'})
