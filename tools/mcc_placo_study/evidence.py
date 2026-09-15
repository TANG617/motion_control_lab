"""Experiment-only evidence contracts. No solver ownership or analysis execution."""
from __future__ import annotations
import hashlib, json, os, pathlib, platform, re, subprocess

ROOT = pathlib.Path(__file__).resolve().parents[2]
IDENTITY = ('experiment_id','case_id','method_id','arm_id','repeat_id','session_id','split_id','window_id','component_id','input_hash','model_hash','config_hash')
STATES = ('not-run','unavailable','running','completed','failed','crashed','interrupted','input-invalid')

def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()

def stable_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def read_json(path):
    with open(path) as f: return json.load(f)

def write_json(path, value):
    path=pathlib.Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.tmp')
    temp.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n'); temp.replace(path)

def artifact(path):
    path=pathlib.Path(path).resolve()
    return {'locator':str(path),'sha256':digest(path),'size_bytes':path.stat().st_size}

def verify_artifact(item):
    p=pathlib.Path(item['locator'])
    return p.is_file() and digest(p)==item['sha256'] and ('size_bytes' not in item or p.stat().st_size==item['size_bytes'])

def load_manifest(path):
    value=read_json(path)
    if value['schema_version']=='run_manifest.v1': return value # preserve legacy identity verbatim
    if value['schema_version']!='run_manifest.v2': raise ValueError('unknown manifest version')
    if value['run_kind']!='experiment' or not re.fullmatch(r'E\d{2}',value['experiment_id']):
        raise ValueError('only experiment v2 supported; Analysis extension deferred')
    return value

def validate_definition(d):
    if d['schema_version']!='experiment.v2' or not re.fullmatch(r'E\d{2}',d['experiment_id']): raise ValueError('experiment identity')
    for key in ('question','units','failure_policy','metrics','evaluation_windows','controlled_factors'): 
        if key not in d: raise ValueError('missing '+key)
    if not d['units']: raise ValueError('empty required matrix')
    seen=set()
    for u in d['units']:
        for k in ('case_id','method_id','arm_id','input','config','required'): 
            if k not in u: raise ValueError('unit missing '+k)
        key=(u['case_id'],u['arm_id'])
        if key in seen: raise ValueError('duplicate case/arm')
        seen.add(key)
        for k in key:
            if not re.fullmatch(r'[A-Za-z0-9_.-]+',k): raise ValueError('unsafe unit identity')
        if not ('binary' in u or 'command' in u): raise ValueError('no executable declared')
    return d

def source_fingerprint(path):
    path=pathlib.Path(path).resolve()
    def git(*args): return subprocess.run(['git','-C',str(path),*args],capture_output=True,text=True,check=True).stdout
    top=pathlib.Path(git('rev-parse','--show-toplevel').strip())
    tracked=git('ls-files','--full-name','-z','--',str(path)).split('\0')
    untracked=git('ls-files','--full-name','--others','--exclude-standard','-z','--',str(path)).split('\0')
    files=[]
    for name in sorted(set(tracked+untracked)):
        p=top/name
        if not name or any(x in p.parts for x in ('runs','__pycache__')) or not p.is_file(): continue
        files.append({'path':str(p.relative_to(top)),'sha256':digest(p)})
    return {'scope':str(path),'git_root':str(top),'revision':git('rev-parse','HEAD').strip(),
            'dirty_status':git('status','--porcelain','--',str(path)), 'files':files,'content_hash':stable_hash(files)}

def runtime_fingerprint(binary):
    p=pathlib.Path(binary)
    if not p.is_file(): return {'locator':str(p),'status':'unavailable'}
    result={'executable':artifact(p),'libraries':[]}
    proc=subprocess.run(['ldd',str(p)],capture_output=True,text=True)
    result['ldd']=proc.stdout+proc.stderr
    for name in sorted(set(re.findall(r'(/[^\s()]+)',proc.stdout))):
        if pathlib.Path(name).is_file(): result['libraries'].append(artifact(name))
    return result

def environment():
    def read(p):
        try: return pathlib.Path(p).read_text().strip()
        except OSError: return None
    affinity=sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else []
    cores=[]
    for cpu in affinity:
        base=f'/sys/devices/system/cpu/cpu{cpu}'
        cores.append({'cpu':cpu,'core_id':read(base+'/topology/core_id'),'package':read(base+'/topology/physical_package_id'),
                      'core_type':read(base+'/topology/core_type'),'governor':read(base+'/cpufreq/scaling_governor')})
    return {'kernel':platform.release(),'boot_id':read('/proc/sys/kernel/random/boot_id'),
            'realtime_flag':read('/sys/kernel/realtime'),'affinity':affinity,'cores':cores,
            'scheduler':os.sched_getscheduler(0) if hasattr(os,'sched_getscheduler') else None,
            'cgroup':read('/proc/self/cgroup'),'container':pathlib.Path('/.dockerenv').exists(),
            'boundary':'container observations; host RT/core/frequency confirmation required',
            'environment':{k:os.environ[k] for k in ('MCL_BINARY','MCL_INSTALL_PREFIX','LD_LIBRARY_PATH','COLCON_DEFAULTS_FILE') if k in os.environ}}
