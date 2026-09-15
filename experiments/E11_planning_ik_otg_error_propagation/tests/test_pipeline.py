import importlib.util,pathlib,sys,unittest
HERE=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'));spec=importlib.util.spec_from_file_location('legacy_planner_fixture',HERE/'declarations/study_prepare.py');prepare=importlib.util.module_from_spec(spec);spec.loader.exec_module(prepare)
class PipelineTest(unittest.TestCase):
 def test_all_planner_factor_settings_equal(self):
  configs=[prepare.config('mcc-hqp-2',p) for p in prepare.PIPELINES]
  for c in configs:c.pop('pipeline')
  self.assertTrue(all(c==configs[0] for c in configs));self.assertEqual(configs[0]['projection_policy'],'none')
 def test_zero_and_known_tcp_offset(self):
  from inputs import canonical_snapshot,UrdfFk
  import numpy as np
  data=canonical_snapshot('/workspace/models/r1.cos.urdf',fixture=True);fk=UrdfFk(data['model']['locator']);q=data['initial_state']['q'];frame=data['frames']['left'];a=fk.pose(data['joint_names'],q,frame,[0,0,0]);b=fk.pose(data['joint_names'],q,frame,[0,0,.1]);self.assertAlmostEqual(np.linalg.norm(np.array(b['position'])-a['position']),.1,places=12)
 def test_delay_state_versions(self):
  dt=.001
  for delay in (0,.005,.02):
   lag=round(delay/dt)
   self.assertEqual([max(0,t-lag) for t in (0,3,21)],[0,max(0,3-lag),21-lag])
 def test_piecewise_quintic_knots_are_c2(self):
  for t,target in [(0,0),(2.5,1),(5,0),(7.5,-1),(10,0)]:self.assertAlmostEqual(prepare.smooth_wave(t),target,places=12)
  h=1e-4
  for t in (2.5,5.,7.5,10.):
   left=prepare.smooth_wave(t-h);center=prepare.smooth_wave(t);right=prepare.smooth_wave(t+h)
   self.assertLess(abs(left-right),1e-9)
   self.assertLess(abs((right-left)/(2*h)),1e-6)
   self.assertLess(abs((right-2*center+left)/(h*h)),.002)
if __name__=='__main__':unittest.main()
