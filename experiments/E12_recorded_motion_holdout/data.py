#!/usr/bin/env python3
"""Immutable inventory, conservative session grouping and explicit MCAP conversion.
Inventory hashes raw bytes only. convert refuses holdout before opening raw MCAP.
"""
import argparse,copy,csv,hashlib,json,math,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/mcc_placo_study'))
from evidence import artifact,stable_hash,write_json,verify_artifact
from inputs import canonical_snapshot,save_input
TOPICS={'left':'/hal/tracker/htc/left/calib_target_pose','right':'/hal/tracker/htc/right/calib_target_pose','joints':'/mc/ik/joint_states'}

def grouped_split(records,seed=20260910):
 parent={r['recording_id']:r['recording_id'] for r in records}
 def find(x):
  while parent[x]!=x:x=parent[x]
  return x
 def union(a,b):parent[find(b)]=find(a)
 for i,a in enumerate(records):
  for b in records[i+1:]:
   same=a['session_id']==b['session_id'] or a['raw']['sha256']==b['raw']['sha256']
   overlap=(a.get('source_id') and a.get('source_id')==b.get('source_id') and a.get('interval_ns') and b.get('interval_ns') and max(a['interval_ns'][0],b['interval_ns'][0])<=min(a['interval_ns'][1],b['interval_ns'][1]))
   if same or overlap:union(a['recording_id'],b['recording_id'])
 groups={}
 for r in records:groups.setdefault(find(r['recording_id']),[]).append(r)
 ids={root:'session-group-'+stable_hash(sorted({r['session_id'] for r in rows}))[:16] for root,rows in groups.items()}
 ordered=sorted(groups,key=lambda g:hashlib.sha256(f'{seed},{ids[g]}'.encode()).hexdigest());n=len(ordered);result=[]
 for index,g in enumerate(ordered):
  split='development' if n<5 or index<math.floor(.6*n) else 'tuning' if index<math.floor(.8*n) else 'holdout'
  exposure='unexposed' if all(r['exposure']=='unexposed' for r in groups[g]) else 'exposed-or-unknown'
  if exposure!='unexposed' and split=='holdout':split='development'
  for r in groups[g]:result.append(dict(recording_id=r['recording_id'],session_id=r['session_id'],group_id=ids[g],split_id=split,exposure=exposure))
 return dict(schema_version='session_split.v1',seed=seed,groups=n,policy='session and duplicate/overlap connected components; unknown sessions merged; exposed/unknown never holdout',C6='eligible-pending-freeze' if n>=5 and any(r['split_id']=='holdout' for r in result) else 'evidence-insufficient',records=result)

def inventory(locator,metadata=None):
 metadata=metadata or {};records=[]
 for p in sorted(pathlib.Path(locator).glob('*.mcap')):
  m=metadata.get(p.name,{});records.append(dict(recording_id=p.stem,raw=artifact(p),relative_path=p.name,session_id=m.get('session_id','unknown-common-session'),session_provenance=m.get('session_provenance','unknown; conservatively merged'),source_id=m.get('source_id'),interval_ns=m.get('interval_ns'),exposure=m.get('exposure','unknown-historically-available-to-E05'),exposure_reason=m.get('exposure_reason','existing mount and historical E05 use; no unexposed attestation'),quality='not-decoded',source_topics=TOPICS,duplicate_of=None))
 seen={}
 for r in records:
  h=r['raw']['sha256'];r['duplicate_of']=seen.get(h);seen.setdefault(h,r['recording_id'])
 return dict(schema_version='dataset_inventory.v1',locator=str(pathlib.Path(locator).resolve()),content_access='hash-only; no MCAP decode',records=records,inventory_hash=stable_hash(records))

def save_inventory(inv,out):
 out=pathlib.Path(out);out.mkdir(parents=True,exist_ok=False);write_json(out/'dataset_inventory.json',inv);split=grouped_split(inv['records']);write_json(out/'session_split.json',split)
 for name,fields,rows in [('dataset_inventory.csv',['recording_id','locator','sha256','size_bytes','session_id','quality','duplicate_of'],[dict(recording_id=r['recording_id'],**r['raw'],session_id=r['session_id'],quality=r['quality'],duplicate_of=r['duplicate_of']) for r in inv['records']]),('exposure_register.csv',['recording_id','exposure','exposure_reason'],[{k:r[k] for k in ['recording_id','exposure','exposure_reason']} for r in inv['records']])]:
  with (out/name).open('w') as f:w=csv.DictWriter(f,fields);w.writeheader();w.writerows(rows)
 return split

def object_dict(x):
 if isinstance(x,(int,float,str,bool)) or x is None:return x
 if isinstance(x,(list,tuple)):return [object_dict(y) for y in x]
 if isinstance(x,dict):return {k:object_dict(v) for k,v in x.items()}
 return {k:object_dict(getattr(x,k)) for k in x.__slots__} if hasattr(x,'__slots__') else {k:object_dict(v) for k,v in vars(x).items()}
def decode(raw,topics):
 from mcap.reader import make_reader
 from mcap_ros2.decoder import DecoderFactory
 with open(raw,'rb') as f:
  reader=make_reader(f,validate_crcs=True,decoder_factories=[DecoderFactory()])
  for schema,channel,message,decoded in reader.iter_decoded_messages(topics=list(topics.values())):
   yield dict(topic=channel.topic,schema=schema.name,schema_encoding=schema.encoding,message_encoding=channel.message_encoding,log_time_ns=message.log_time,publish_time_ns=message.publish_time,data=object_dict(decoded))

def header_time(data):
 stamp=data['header']['stamp'];return stamp['sec']*1000000000+stamp['nanosec']
def rotation(q):
 x,y,z,w=[q[k] for k in ('x','y','z','w')];norm=x*x+y*y+z*z+w*w
 if abs(norm-1)>1e-5:raise ValueError('non-unit recorded quaternion; no silent normalization')
 return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
def convert_messages(messages,model,recipe,record,split,allow_confirmatory=False):
 if split['split_id']=='holdout' and not allow_confirmatory:raise ValueError('holdout conversion forbidden during implementation/development')
 streams={key:[] for key in recipe['topics']};reverse={v:k for k,v in recipe['topics'].items()}
 for m in messages:
  key=reverse[m['topic']];m=copy.deepcopy(m);m['header_time_ns']=header_time(m['data']);m['logical_ns']=m[recipe['timestamp_source']];streams[key].append(m)
 for key,items in streams.items():
  if not items:raise ValueError('missing required stream '+key)
  if any(b['logical_ns']<=a['logical_ns'] for a,b in zip(items,items[1:])):raise ValueError('nonmonotonic or duplicate source timestamp '+key)
 base=canonical_snapshot(model);base.update(input_id=record['recording_id'],source_kind='recorded-mcap',fixture=record.get('fixture',False),session_id=split['session_id'],split_id=split['split_id'],exposure=record.get('exposure','unknown'),raw_source=record['raw'],conversion_recipe=recipe,converter=artifact(__file__),samples=[])
 right=streams['right'];used=set();pairs=[]
 for left in streams['left']:
  candidates=[(abs(r['logical_ns']-left['logical_ns']),j,r) for j,r in enumerate(right) if j not in used];delta,j,r=min(candidates,default=(math.inf,-1,None))
  if delta>recipe['pairing_tolerance_ns']:raise ValueError('unpaired target; no sample silently dropped')
  used.add(j);pairs.append((left,r))
 if len(used)!=len(right):raise ValueError('unpaired right suffix')
 t0=max(m['logical_ns'] for m in pairs[0]);initials=[m for m in streams['joints'] if m['logical_ns']<=t0]
 if not initials:raise ValueError('missing causal joint initialization')
 initial=initials[-1]['data']
 if len(initial['name'])!=len(initial['position']) or len(set(initial['name']))!=len(initial['name']):raise ValueError('input-invalid: ambiguous recorded joint names/positions')
 lookup=dict(zip(initial['name'],initial['position']))
 if any(n not in lookup for n in base['joint_names']):raise ValueError('missing initial full joint mapping')
 q=[lookup[n] for n in base['joint_names']]
 if not all(math.isfinite(value) for value in q):raise ValueError('input-invalid: nonfinite recorded initialization')
 if any(value<lower or value>upper for value,lower,upper in zip(q,base['limits']['lower'],base['limits']['upper'])):raise ValueError('input-invalid: recorded initialization outside model position bounds; no clipping')
 base['initial_state']={'q':q,'v':[0.]*len(q),'velocity_policy':'explicit-zero; recorded output used only for initialization'}
 for sequence,pair in enumerate(pairs):
  targets={};provenance={}
  for key,m in zip(('left','right'),pair):
   d=m['data']
   if d['header']['frame_id']!=base['root_frame']:raise ValueError('frame transform required; recipe cannot silently relabel')
   targets[key]={'position':[d['pose']['position'][k] for k in ('x','y','z')],'rotation':rotation(d['pose']['orientation'])};provenance[key]={k:m[k] for k in ['topic','schema','schema_encoding','message_encoding','header_time_ns','log_time_ns','publish_time_ns']}
  logical=max(m['logical_ns'] for m in pair);time=(logical-t0)/1e9 if recipe['timeline']=='source-time' else sequence*recipe['fixed_period_s']
  base['samples'].append(dict(sequence=sequence,source_time_s=time,q=q,v=[0.]*len(q),targets=targets,source_provenance=provenance,event='source'))
 base['derivative_policy']='no interpolation; targets zero-order held online; pair original times before one-to-one retime';return base

def confirmatory_gate(record,split,definition_path,model=None,recipe=None):
 """Validate frozen metadata and RT before any holdout raw open/decode."""
 from evidence import environment
 from freeze import frozen_prerequisites
 definition=json.loads(pathlib.Path(definition_path).read_text());env=environment();reasons=frozen_prerequisites(definition,env)
 if env.get('realtime_flag')!='1':reasons.append('confirmed PREEMPT_RT runtime required')
 if reasons:raise ValueError('formal-not-ready: '+'; '.join(reasons))
 frozen=json.loads(pathlib.Path(definition['freeze']['locator']).read_text())
 if model is not None and not any(ref==artifact(model) for ref in frozen['models']):raise ValueError('conversion model differs from freeze')
 if recipe is not None and not any(pathlib.Path(ref['locator']).suffix=='.json' and stable_hash(json.loads(pathlib.Path(ref['locator']).read_text()))==stable_hash(recipe) for ref in frozen['inputs']+frozen['resources']):raise ValueError('conversion recipe not pinned')
 def metadata(ref):
  p=pathlib.Path(ref['locator'])
  if p.suffix=='.csv':return list(csv.DictReader(p.open()))
  if p.suffix=='.json':
   value=json.loads(p.read_text());return value.get('records',value) if isinstance(value,dict) else value
  return []
 raw_pinned=any(ref['sha256']==record['raw']['sha256'] and ref['locator']==record['raw']['locator'] for ref in frozen['inputs'])
 for ref in frozen['inputs']:
  value=metadata(ref)
  if isinstance(value,list):raw_pinned |= any(r.get('recording_id')==record['recording_id'] and r.get('raw')==record['raw'] for r in value)
 if not raw_pinned:raise ValueError('holdout raw source absent from frozen input inventory')
 split_matches=[r for ref in frozen['split'] for r in metadata(ref) if isinstance(r,dict) and r.get('recording_id')==record['recording_id']]
 if len(split_matches)!=1 or split_matches[0]!=split or split['split_id']!='holdout':raise ValueError('holdout split differs from frozen session assignment')
 exposures=[r for ref in frozen['exposure'] for r in metadata(ref) if isinstance(r,dict) and r.get('recording_id')==record['recording_id']]
 if len(exposures)!=1 or exposures[0].get('exposure')!='unexposed' or record.get('exposure')!='unexposed':raise ValueError('holdout exposure is not frozen unexposed')
 candidates=[json.loads(pathlib.Path(ref['locator']).read_text()) for ref in frozen['candidates']]
 if not any(c.get('schema_version')=='candidate_freeze.v1' and c.get('selected') for c in candidates):raise ValueError('selected candidate freeze missing')
 return True

def convert(record,split,model,recipe,out,phase='development',frozen_definition=None):
 allowed=False
 if split['split_id']=='holdout':
  if phase!='confirmatory' or not frozen_definition:raise ValueError('holdout denied before raw open')
  allowed=confirmatory_gate(record,split,frozen_definition,model,recipe)
 if not verify_artifact(record['raw']):raise ValueError('immutable raw hash mismatch')
 canonical=convert_messages(decode(record['raw']['locator'],recipe['topics']),model,recipe,record,split,allow_confirmatory=allowed)
 if not verify_artifact(record['raw']):raise ValueError('raw changed during conversion')
 if pathlib.Path(out).exists():raise FileExistsError('refusing canonical overwrite')
 return save_input(out,canonical)
if __name__=='__main__':
 p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True);i=sub.add_parser('inventory');i.add_argument('--locator',default='/mnt/mcap_dataset');i.add_argument('--metadata');i.add_argument('--output',required=True);c=sub.add_parser('convert');c.add_argument('--inventory',required=True);c.add_argument('--split',required=True);c.add_argument('--recording',required=True);c.add_argument('--recipe',required=True);c.add_argument('--model',required=True);c.add_argument('--output',required=True);c.add_argument('--phase',choices=['development','confirmatory'],default='development');c.add_argument('--frozen-definition');a=p.parse_args()
 if a.command=='inventory':save_inventory(inventory(a.locator,json.loads(pathlib.Path(a.metadata).read_text()) if a.metadata else None),a.output)
 else:
  inv=json.loads(pathlib.Path(a.inventory).read_text());split=json.loads(pathlib.Path(a.split).read_text());r=next(r for r in inv['records'] if r['recording_id']==a.recording);s=next(s for s in split['records'] if s['recording_id']==a.recording);print(convert(r,s,a.model,json.loads(pathlib.Path(a.recipe).read_text()),a.output,a.phase,a.frozen_definition))
