import pathlib,sys,tempfile,unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from evidence import *
from inputs import *
from metrics import *
class Contracts(unittest.TestCase):
    def test_math(self):
        self.assertAlmostEqual(position_error([0,0,0],[3,4,0]),5)
        self.assertAlmostEqual(orientation_error(np.eye(3),rotation([0,0,1],.3)),.3)
        self.assertIsNone(secondary_gain(0,0)['relative'])
        self.assertEqual(nearest_rank([9,1,3,2],.75),3)
        self.assertEqual([window_for_call(i,2) for i in range(4)],['cold','warm-up','warm-up','steady'])
        self.assertEqual(deadlines([{'deadline_ns':10,'finish_ns':10},{'deadline_ns':20,'skipped':True},{'deadline_ns':30,'finish_ns':31}])['consecutive_misses'],2)
        self.assertEqual(paired_admission({'x':['a','b']},{('x','a'):'completed'})[0]['status'],'unavailable')
        self.assertEqual(preservation_ratio([0],[.2],[.1]),2)
        with self.assertRaises(ValueError):preservation_ratio([0],[0],[0])
    def test_hash_and_legacy(self):
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t)/'a';p.write_text('original');a=artifact(p);self.assertTrue(verify_artifact(a));p.write_text('tampered');self.assertFalse(verify_artifact(a))
            write_json(p,{'schema_version':'run_manifest.v1','experiment_id':'E01'});self.assertEqual(load_manifest(p)['experiment_id'],'E01')
            write_json(p,{'schema_version':'run_manifest.v2','run_kind':'analysis','experiment_id':'E01'})
            with self.assertRaises(ValueError):load_manifest(p)
    def test_independent_fk(self):
        with tempfile.TemporaryDirectory() as t:
            p=pathlib.Path(t)/'m.urdf';p.write_text('<robot name="test"><link name="base_link"/><link name="tip"/><joint name="j" type="revolute"><parent link="base_link"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1"/></joint></robot>')
            fk=UrdfFk(p);pose=fk.pose(['j'],[np.pi/2],'tip',[1,0,0]);np.testing.assert_allclose(pose['position'],[0,1,0],atol=1e-12)
    def test_seed(self):
        a=canonical_snapshot('/workspace/models/r1.cos.urdf',perturbation=.01)
        b=canonical_snapshot('/workspace/models/r1.cos.urdf',perturbation=.01)
        self.assertEqual(stable_hash(a),stable_hash(b))
if __name__=='__main__':unittest.main()
