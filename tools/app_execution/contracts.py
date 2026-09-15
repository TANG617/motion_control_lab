"""Public execution data/provenance helpers; no app-private imports or algorithms."""
from __future__ import annotations
import hashlib,json,pathlib,subprocess,re
import jsonschema
ROOT=pathlib.Path(__file__).resolve().parents[2]
SCHEMA=ROOT/'contracts/execution/execution_request.v1.schema.json'

def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def stable_hash(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def read(path):
    def unique(pairs):
        result={}
        for k,v in pairs:
            if k in result:raise ValueError('duplicate JSON key: '+k)
            result[k]=v
        return result
    return json.loads(pathlib.Path(path).read_text(),object_pairs_hook=unique)
def write(path,value):
    path=pathlib.Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')
def artifact(path):
    p=pathlib.Path(path).resolve();return {'path':str(p),'sha256':digest(p),'size_bytes':p.stat().st_size}
def verify(ref):
    path=pathlib.Path(ref.get('path',ref.get('locator','')))
    if not path.is_absolute():raise ValueError('source must be explicit absolute path')
    if not path.is_file() or digest(path)!=ref['sha256']:raise ValueError('missing or corrupt source: '+str(path))
    return path

def validate_request(request,app_id=None):
    jsonschema.Draft202012Validator(read(SCHEMA)).validate(request)
    if app_id and request['app_id']!=app_id:raise ValueError('app identity mismatch')
    verify(request['input'])
    if request['input']['format']=='json':
        data=read(request['input']['path'])
        if data.get('model'):verify(data['model'])
    return request

def capabilities(binary):
    p=pathlib.Path(binary).resolve()
    result=subprocess.run([str(p),'--describe-capabilities'],check=True,capture_output=True,text=True,timeout=10)
    data=json.loads(result.stdout)
    return {'binary':artifact(p),'document':data,'sha256':stable_hash(data)}

def descriptor_input(path):
    d=read(path)
    if d['schema_version']!='input_descriptor.v1':raise ValueError('input descriptor schema mismatch')
    if not d.get('canonical'):raise ValueError(d.get('reason','canonical input missing'))
    p=verify(d['canonical']);data=read(p)
    if data.get('model'):verify(data['model'])
    return {'path':str(p),'sha256':d['canonical']['sha256'],'format':'json'},data,d

def validate_definition(d):
    if d.get('execution_contract')!='execution_request.v1':raise ValueError('not a public execution declaration')
    if d.get('schema_version')!='experiment.v2':raise ValueError('experiment schema mismatch')
    jsonschema.Draft202012Validator(read(ROOT/'contracts/definitions/experiment.v2.schema.json')).validate(d)
    legacy=d.get('legacy_definition')
    if 'legacy_definition' in d:
        verify(legacy)
    seen=set()
    for u in d['units']:
        identity=(u['case_id'],u['arm_id'])
        if any(not re.fullmatch(r'[A-Za-z0-9_.-]+',x) or x in ('.','..') for x in identity):raise ValueError('unsafe unit identity')
        if identity in seen:raise ValueError('duplicate case/arm identity')
        seen.add(identity)
        for k in ('app','execution_structure','config','execution','observation','input','method_id','required'):
            if k not in u:raise ValueError('unit missing '+k)
        if u['app']=='mcl_baseline':
            if u['config']!={'profile':'production-static'}:raise ValueError('baseline algorithm overrides forbidden')
        elif u['app'] not in {'mcl_step','mcl_target','mcl_hierarchical_kinematics_step','mcl_optimization_problem','mcl_planned_kinematics_step'}:
            raise ValueError('unknown explicit app identity')
        if ('migration' in u) != (legacy is not None):
            raise ValueError('migration and legacy_definition must be declared together')
        if 'migration' in u:
            migration=u['migration']
            if migration['status'] not in {'mapped','native_unsupported','retired_research_method','migration_defect'}:
                raise ValueError('unknown migration status')
            if migration['legacy_definition']!=legacy:
                raise ValueError('illegal cross-declaration migration binding')
    return d


def prepare_definition(source,destination):
    """Validate and copy an explicit current declaration without migration or execution."""
    source=pathlib.Path(source);destination=pathlib.Path(destination)
    validate_definition(read(source))
    destination.parent.mkdir(parents=True,exist_ok=True)
    with destination.open('xb') as output:
        output.write(source.read_bytes())
    return destination


def runtime_inventory(binary):
    import re
    p=pathlib.Path(binary).resolve()
    result=subprocess.run(['ldd',str(p)],capture_output=True,text=True,check=True)
    libraries=[]
    for name in sorted(set(re.findall(r'(/[^\s()]+)',result.stdout))):
        if pathlib.Path(name).is_file():libraries.append(artifact(name))
    if 'not found' in result.stdout:raise ValueError('unresolved installed runtime: '+result.stdout)
    return {'binary':artifact(p),'libraries':libraries,'ldd':result.stdout,'dynamic_model_libraries':'app-native HARP sidecar records late-loaded Predictor/runtime/model hashes'}

def collision_resources(model_path):
    """Read-only URI audit for the native local collision-geometry loader.

    Does not rewrite URDFs, download assets, select another model or disable collision.
    Package URI resolution remains owned by the native loader.
    """
    import urllib.parse,xml.etree.ElementTree as ET
    model_path=pathlib.Path(model_path);missing=[];local=[]
    for node in ET.parse(model_path).getroot().findall('./link/collision/geometry/mesh'):
        name=node.attrib['filename'];uri=urllib.parse.urlparse(name)
        if uri.scheme in ('http','https'):
            missing.append({'uri':name,'reason':'native collision loader does not support remote URI schemes'})
        elif not uri.scheme or uri.scheme=='file':
            p=pathlib.Path(uri.path if uri.scheme else name)
            if not p.is_absolute():p=model_path.parent/p
            if p.is_file():local.append(artifact(p))
            else:missing.append({'uri':name,'reason':'local collision mesh missing','resolved_path':str(p)})
    return {'status':'resource_missing' if missing else 'passed','missing':missing,'local_assets':local}
