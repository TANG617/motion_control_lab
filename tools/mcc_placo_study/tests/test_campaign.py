"""Coordinator boundary tests; fake conversions are not experiment evidence."""
import contextlib,io,pathlib,re,sys,tempfile,unittest
from unittest.mock import patch
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import campaign,study
from evidence import artifact,read_json,write_json

class Campaign(unittest.TestCase):
    def test_conversion_failure_continues_and_confirmatory_stays_blocked(self):
        with tempfile.TemporaryDirectory() as temp:
            root=pathlib.Path(temp)
            records=[{'recording_id':key,'raw':{'locator':'unused','sha256':'unused'}} for key in ('bad','good','holdout')]
            write_json(root/'input.json',{'records':records})
            write_json(root/'split.json',{'records':[{'recording_id':r['recording_id'],'session_id':'session','split_id':'holdout' if r['recording_id']=='holdout' else 'development'} for r in records]})
            units=[{'case_id':r['recording_id']+'-'+stratum,'method_id':method,'available':False,'unavailable_reason':'conversion pending','config':{'stratum':stratum,'duration_s':10,'settling_duration_s':2}} for r in records for stratum,method in [('historical','production-static'),('controlled','mcc-hqp-2'),('confirmatory','mcc-hqp-2')]]
            write_json(root/'definitions/E12.json',{'units':units})
            plan={'root':str(root),'dataset_inventory':artifact(root/'input.json'),'session_split':artifact(root/'split.json'),'recipe':{'locator':'unused'},'model':{'locator':'unused'},'progress_total':3,'progress_definition':'test','minimum_free_bytes':0,'conversion_timeout_s':1}
            coordinator=campaign.Coordinator(plan);calls=[]
            def fake_convert(command,out,timeout):
                key=command[command.index('--recording')+1];calls.append(key)
                if key=='bad':return 7
                canonical=out/'canonical.json';write_json(canonical,{'split_id':'development','samples':[{'source_time_s':4.2}]})
                write_json(canonical.with_suffix('.descriptor.json'),{'canonical':artifact(canonical)})
                return 0
            output=io.StringIO()
            with patch.object(coordinator,'process',side_effect=fake_convert),contextlib.redirect_stdout(output):
                resolved=coordinator.convert_inputs()
            self.assertEqual(calls,['bad','good'])
            self.assertEqual([r['status'] for r in coordinator.conversions],['input-invalid','converted','input-invalid'])
            actual=read_json(resolved)['units']
            self.assertEqual([u['available'] for u in actual],[False,False,False,True,True,False,False,False,False])
            self.assertEqual(actual[3]['config']['duration_s'],10)
            self.assertEqual(actual[4]['config']['duration_s'],6.2)
            self.assertEqual(read_json(root/'definitions/E12.json')['units'],units)
            counters=[]
            for line in output.getvalue().splitlines():
                match=re.fullmatch(r'PROGRESS: ([0-9]+)/([0-9]+) stage=[A-Za-z0-9_-]+',line)
                self.assertIsNotNone(match);counters.append(int(match[1]))
                self.assertNotIn('session',line)
            self.assertEqual(counters,sorted(counters));self.assertEqual(counters[-1],3)

    def test_storage_gate_keeps_terminal_evidence_without_starting_process(self):
        with tempfile.TemporaryDirectory() as temp:
            root=pathlib.Path(temp)
            unit={'case_id':'storage-blocked','arm_id':'fixture','method_id':'fixture','required':True,'config':{}}
            selection=[{'definition':{'experiment_id':'E06'},'unit':unit,'repeat_id':0}]
            row={k:unit[k] for k in ('case_id','arm_id','method_id','required')};row.update(experiment_id='E06',repeat_id=0,missing=[])
            args=study.parser().parse_args(['--output-root',str(root/'runs')]);events=[]
            with patch.object(study,'resolve_input',side_effect=AssertionError('must not read input')),patch.object(study,'source_fingerprint',return_value={}):
                code=study.execute(selection,{'units':[row]},args,on_event=lambda kind,state:events.append((kind,state['status'])),unit_gate=lambda item:'storage reserve')
            self.assertEqual(code,1);self.assertEqual(events,[('started','not-run'),('finished','unavailable')])
            inventory=next((root/'runs').glob('*/inventory.json'))
            self.assertEqual(campaign.check(inventory)['status'],'passed')

if __name__=='__main__':unittest.main()
