import copy,importlib.util,json,pathlib,sys,tempfile,unittest
HERE=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE));import data,candidates
MODEL='/workspace/models/r1.cos.urdf'
def record(i,session=None,exposure='unexposed',hash=None):return dict(recording_id=str(i),session_id=session or str(i),exposure=exposure,raw={'sha256':hash or str(i),'locator':'/must-not-open.mcap'},source_id=None,interval_ns=None)
class DataTest(unittest.TestCase):
 def test_duplicates_and_overlap_connected(self):
  a,b,c=record(1),record(2,hash='1'),record(3)
  b.update(source_id='capture',interval_ns=[0,10]);c.update(source_id='capture',interval_ns=[9,20]);s=data.grouped_split([a,b,c]);self.assertEqual(s['groups'],1);self.assertEqual(len({r['split_id'] for r in s['records']}),1)
 def test_exposure_not_washed_by_split(self):
  rows=[record(i,exposure='exposed') for i in range(10)];s=data.grouped_split(rows);self.assertNotIn('holdout',[r['split_id'] for r in s['records']]);self.assertEqual(s['C6'],'evidence-insufficient')
 def test_session_no_cross_split(self):
  rows=[record(i,session=str(i//2)) for i in range(12)];s=data.grouped_split(rows)
  for session in {r['session_id'] for r in rows}:self.assertEqual(len({r['split_id'] for r in s['records'] if r['session_id']==session}),1)
 def test_holdout_denied_before_open(self):
  with self.assertRaisesRegex(ValueError,'holdout denied'):data.convert(record(0),{'split_id':'holdout'},MODEL,{},'/must-not-write')
 def test_missing_stream_and_bad_frame(self):
  recipe={'topics':data.TOPICS,'timestamp_source':'header_time_ns','pairing_tolerance_ns':5,'timeline':'source-time'}
  with self.assertRaisesRegex(ValueError,'missing required stream'):data.convert_messages([],MODEL,recipe,record(0),{'split_id':'development'})
 def test_equal_budget_and_holdout(self):
  cs=[dict(candidate_id=m,method_id=m,config={}) for m in ('mcc','placo')];rs=[dict(recording_id='r',split_id='development')];p=candidates.search_plan(cs,rs,2,3);self.assertEqual(len(p['trials']),12)
  with self.assertRaisesRegex(ValueError,'holdout'):candidates.search_plan(cs,[dict(recording_id='r',split_id='holdout')],1)
  with self.assertRaises(ValueError):candidates.search_plan([],rs,1)
 def test_partial_status_freeze_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   d=pathlib.Path(d);p=candidates.search_plan([dict(candidate_id='a',method_id='mcc',config={})],[dict(recording_id='r',split_id='development')],1);data.write_json(d/'plan',p);data.write_json(d/'result',[dict(p['trials'][0],status='not-run')]);data.write_json(d/'reg',{})
   with self.assertRaisesRegex(ValueError,'not completed'):candidates.freeze([{'candidate_id':'a','config':{}}],data.artifact(d/'plan'),data.artifact(d/'result'),data.artifact(d/'reg'),d/'freeze')
 def test_hash_only_inventory_preserves_raw(self):
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d)/'bad.mcap';p.write_bytes(b'explicit corrupt MCAP fixture');before=p.read_bytes();inv=data.inventory(d);self.assertEqual(inv['records'][0]['quality'],'not-decoded');self.assertEqual(p.read_bytes(),before)
if __name__=='__main__':unittest.main()

class McapConversionTest(unittest.TestCase):
 def test_real_mcap_fixture_decode_pair_retime_and_raw_immutability(self):
  from mcap_ros2.writer import Writer
  header='''std_msgs/Header header
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
  pose='''std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
'''+header[header.index('================================================================================'):]
  joints='''std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
'''+header[header.index('================================================================================'):]
  with tempfile.TemporaryDirectory() as directory:
   p=pathlib.Path(directory)/'explicit-fixture.mcap';base=data.canonical_snapshot(MODEL,fixture=True)
   with p.open('wb') as f:
    w=Writer(f);ps=w.register_msgdef('geometry_msgs/msg/PoseStamped',pose);js=w.register_msgdef('sensor_msgs/msg/JointState',joints)
    h={'stamp':{'sec':1,'nanosec':0},'frame_id':'base_link'}
    w.write_message(data.TOPICS['joints'],js,dict(header=h,name=list(base['joint_names']),position=list(base['initial_state']['q']),velocity=[0.]*20,effort=[]),log_time=1000000000,publish_time=1000000000)
    for i in range(2):
     h={'stamp':{'sec':1,'nanosec':i*20000000},'frame_id':'base_link'}
     for s in ('left','right'):w.write_message(data.TOPICS[s],ps,dict(header=h,pose=dict(position=dict(x=.5+i*.01,y=.2,z=.9),orientation=dict(x=0.,y=0.,z=0.,w=1.))),log_time=1000000000+i*20000000,publish_time=1000000000+i*20000000)
    w.finish()
   before=data.artifact(p);r=dict(record(0),raw=before,fixture=True);split=dict(split_id='development',session_id='fixture');recipe=dict(topics=data.TOPICS,timestamp_source='header_time_ns',pairing_tolerance_ns=100,timeline='fixed-period-one-to-one',fixed_period_s=.01)
   descriptor=data.convert(r,split,MODEL,recipe,pathlib.Path(directory)/'canonical.json');canonical=json.loads(pathlib.Path(directory,'canonical.json').read_text());self.assertEqual([s['source_time_s'] for s in canonical['samples']],[0.,.01]);self.assertEqual(canonical['samples'][1]['source_provenance']['left']['header_time_ns'],1020000000);self.assertEqual(before,data.artifact(p));self.assertTrue(pathlib.Path(descriptor).exists())

class SelectionTest(unittest.TestCase):
 def test_selection_keeps_failed_candidates_and_equal_budget(self):
  cs=[dict(candidate_id=x,method_id='mcc',config={'gain':i}) for i,x in enumerate(('a','b'))];plan=candidates.search_plan(cs,[dict(recording_id='r',split_id='tuning')],1)
  results=[dict(t,status='completed' if t['candidate_id']=='a' else 'failed',guardrails_passed=True,full_quality_coverage=1.,selection_metrics={'position_error':.1}) for t in plan['trials']]
  chosen=candidates.select(plan,results,{'metric':'position_error','direction':'minimize'});self.assertEqual(chosen['selected'][0]['candidate_id'],'a');self.assertEqual(len(chosen['all_candidates']['mcc']),2)
 def test_selection_missing_and_empty_are_not_success(self):
  plan=candidates.search_plan([dict(candidate_id='a',method_id='mcc',config={})],[dict(recording_id='r',split_id='tuning')],1)
  with self.assertRaisesRegex(ValueError,'missing'):candidates.select(plan,[],{'metric':'position_error','direction':'minimize'})

class FreezeGateTest(unittest.TestCase):
 def test_confirmatory_gate_checks_pins_without_opening_raw(self):
  from unittest.mock import patch
  import evidence,freeze
  with tempfile.TemporaryDirectory() as directory:
   d=pathlib.Path(directory);r=record(7);split=dict(recording_id='7',session_id='7',split_id='holdout',exposure='unexposed')
   payloads={'inventory':{'records':[r]},'split':{'records':[split]},'exposure':{'records':[{'recording_id':'7','exposure':'unexposed'}]},'candidates':{'schema_version':'candidate_freeze.v1','selected':[{'candidate_id':'fixture-only'}]}}
   for name,value in payloads.items():data.write_json(d/(name+'.json'),value)
   frozen={'inputs':[data.artifact(d/'inventory.json')],'split':[data.artifact(d/'split.json')],'exposure':[data.artifact(d/'exposure.json')],'candidates':[data.artifact(d/'candidates.json')]};data.write_json(d/'freeze.json',frozen);data.write_json(d/'definition.json',{'freeze':data.artifact(d/'freeze.json')})
   with patch.object(evidence,'environment',return_value={'realtime_flag':'1'}),patch.object(freeze,'frozen_prerequisites',return_value=[]):
    self.assertTrue(data.confirmatory_gate(r,split,d/'definition.json'))
    with self.assertRaisesRegex(ValueError,'split differs'):data.confirmatory_gate(r,dict(split,session_id='wrong'),d/'definition.json')
    with self.assertRaisesRegex(ValueError,'not frozen unexposed'):data.confirmatory_gate(dict(r,exposure='exposed'),split,d/'definition.json')
   with patch.object(evidence,'environment',return_value={'realtime_flag':'0'}),patch.object(freeze,'frozen_prerequisites',return_value=[]):
    with self.assertRaisesRegex(ValueError,'PREEMPT_RT'):data.confirmatory_gate(r,split,d/'definition.json')
 def test_search_max30_and_quality_precedes_primary_score(self):
  with self.assertRaisesRegex(ValueError,'30'):candidates.search_plan([dict(candidate_id=str(i),method_id='mcc',config={}) for i in range(31)],[dict(split_id='development')],1)
  plan=candidates.search_plan([dict(candidate_id=x,method_id='mcc',config={}) for x in ('full','partial')],[dict(split_id='development')],1)
  rows=[dict(t,status='completed',guardrails_passed=True,full_quality_coverage=1. if t['candidate_id']=='full' else .5,selection_metrics={'position_error':1. if t['candidate_id']=='full' else .01}) for t in plan['trials']]
  self.assertEqual(candidates.select(plan,rows,{'metric':'position_error','direction':'minimize'})['selected'][0]['candidate_id'],'full')
 def test_native_evidence_measure_select_freeze_fixture(self):
  # Explicit synthetic evidence fixture tests the extraction/freeze plumbing, not algorithms.
  with tempfile.TemporaryDirectory() as directory:
   d=pathlib.Path(directory);canonical=data.canonical_snapshot(MODEL,fixture=True);descriptor=data.save_input(d/'fixture.json',canonical)
   cfg={'duration_s':.001,'dt_s':.001};policy={'metric':'position_error','direction':'minimize','maximum_position_violation_rad':0.,'minimum_source_coverage':1.,'maximum_execution_holds':0}
   plan=candidates.search_plan([dict(candidate_id='fixture',method_id='mcc',config=cfg)],[dict(recording_id='fixture',split_id='development',descriptor=descriptor)],1,policy=policy);trial=plan['trials'][0];unit=d/'trial/runs/run/units/unit';unit.mkdir(parents=True)
   raw={'record_type':'attempt','ik_committed':True,'completed_passes':2,'requested_passes':2,'input_sequence':0,'execution_state':'committed'};(unit/'raw.jsonl').write_text(json.dumps(raw)+'\n')
   metric={'metric_id':'position_error','value':.002,'status':'ok','window_id':'source-full'};(unit/'metrics.jsonl').write_text(json.dumps(metric)+'\n');data.write_json(unit/'validation.json',{'status':'passed'});(unit/'stage_checks.jsonl').write_text(json.dumps({'executed_position_violation_rad':0.,'raw_ik_position_violation_rad':0.})+'\n')
   measurement=candidates.measure_trial(d/'trial',trial,policy);self.assertTrue(measurement['guardrails_passed']);rows=[dict(trial,status='completed',**measurement)];chosen=candidates.select(plan,rows,policy)
   data.write_json(d/'plan.json',plan);data.write_json(d/'results.json',rows);data.write_json(d/'policy.json',policy)
   result=candidates.freeze(chosen['selected'],data.artifact(d/'plan.json'),data.artifact(d/'results.json'),data.artifact(d/'policy.json'),d/'frozen.json');self.assertEqual(result['schema_version'],'candidate_freeze.v1')
   with self.assertRaises(FileExistsError):candidates.freeze(chosen['selected'],data.artifact(d/'plan.json'),data.artifact(d/'results.json'),data.artifact(d/'policy.json'),d/'frozen.json')

class MalformedInputTest(unittest.TestCase):
 def messages(self):
  base=data.canonical_snapshot(MODEL,fixture=True);header={'stamp':{'sec':1,'nanosec':0},'frame_id':'base_link'}
  source=[]
  for side in ('left','right'):
   source.append(dict(topic=data.TOPICS[side],schema='geometry_msgs/msg/PoseStamped',schema_encoding='ros2msg',message_encoding='cdr',log_time_ns=1000000000,publish_time_ns=1000000000,data=dict(header=copy.deepcopy(header),pose={'position':{'x':.5,'y':.2,'z':.9},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}})))
  source.insert(0,dict(topic=data.TOPICS['joints'],schema='sensor_msgs/msg/JointState',schema_encoding='ros2msg',message_encoding='cdr',log_time_ns=1000000000,publish_time_ns=1000000000,data=dict(header=header,name=list(base['joint_names']),position=list(base['initial_state']['q']))))
  return source
 def convert(self,messages):
  recipe=dict(topics=data.TOPICS,timestamp_source='header_time_ns',pairing_tolerance_ns=100,timeline='source-time')
  return data.convert_messages(messages,MODEL,recipe,dict(record(0),fixture=True),dict(split_id='development',session_id='fixture'))
 def test_malformed_mcap_decoder_preserves_bytes(self):
  from mcap.exceptions import InvalidMagic
  with tempfile.TemporaryDirectory() as directory:
   p=pathlib.Path(directory)/'explicit-corrupt-fixture.mcap';blob=b'INVALID MCAP explicit fixture';p.write_bytes(blob)
   with self.assertRaises(InvalidMagic):list(data.decode(p,data.TOPICS))
   self.assertEqual(p.read_bytes(),blob)
 def test_bad_frame_unpaired_and_nonmonotonic_are_rejected(self):
  messages=self.messages();messages[1]['data']['header']['frame_id']='undeclared_frame'
  with self.assertRaisesRegex(ValueError,'frame transform required'):self.convert(messages)
  messages=self.messages();messages[2]['data']['header']['stamp']['nanosec']=1000
  with self.assertRaisesRegex(ValueError,'unpaired target'):self.convert(messages)
  messages=self.messages();messages.append(copy.deepcopy(messages[1]))
  with self.assertRaisesRegex(ValueError,'nonmonotonic'):self.convert(messages)
 def test_invalid_initialization_never_reaches_candidate_or_gets_repaired(self):
  for value,reason in [(float('nan'),'nonfinite'),(100.,'outside model')]:
   messages=self.messages();messages[0]['data']['position'][0]=value
   with self.assertRaisesRegex(ValueError,reason):self.convert(messages)
  messages=self.messages();messages[0]['data']['name'][1]=messages[0]['data']['name'][0]
  with self.assertRaisesRegex(ValueError,'ambiguous'):self.convert(messages)
