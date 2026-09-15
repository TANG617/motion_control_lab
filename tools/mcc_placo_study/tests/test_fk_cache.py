import pathlib, sys, tempfile, unittest
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from inputs import UrdfFk, JOINTS, Q0, canonical_snapshot
from legacy_fk_reference import UrdfFk as LegacyFk


class FkCache(unittest.TestCase):
    def test_r1_bit_identical_moving_states_and_jacobians(self):
        model='/workspace/models/r1.cos.urdf'
        old,new=LegacyFk(model),UrdfFk(model)
        rng=np.random.default_rng(710)
        for q in [np.array(Q0),*(np.array(Q0)+rng.uniform(-.02,.02,20) for _ in range(4))]:
            for frame in ['left_arm_ee_link','right_arm_ee_link','left_arm_link4']:
                for root in ['base_link','torso_link']:
                    if root not in old.transforms(JOINTS,q):continue
                    self.assertEqual(old.pose(JOINTS,q,frame,root=root),new.pose(JOINTS,q,frame,root=root))
                np.testing.assert_array_equal(old.jacobian(JOINTS,q,frame),new.jacobian(JOINTS,q,frame))
        self.assertGreater(new._cached.cache_info().hits,0)
        self.assertLessEqual(new._cached.cache_info().currsize,256)

    def test_cache_preserves_ownership_joint_order_and_signed_zero(self):
        fk=UrdfFk('/workspace/models/r1.cos.urdf');q=list(Q0)
        initial=fk.pose(JOINTS,q,'left_arm_ee_link')
        frames=fk.transforms(JOINTS,q);frames['left_arm_ee_link'][:]=123
        self.assertEqual(initial,fk.pose(JOINTS,q,'left_arm_ee_link'))
        self.assertEqual(initial,fk.pose(JOINTS[::-1],q[::-1],'left_arm_ee_link'))
        before=fk._cached.cache_info().misses;q[0]=-0.
        fk.pose(JOINTS,q,'left_arm_ee_link')
        self.assertEqual(fk._cached.cache_info().misses,before+1)
        q[6]+=.1
        self.assertNotEqual(initial,fk.pose(JOINTS,q,'left_arm_ee_link'))

    def test_mimic_prismatic_out_of_order_tree(self):
        text='''<robot name="test"><link name="base_link"/><link name="a"/><link name="b"/><link name="c"/>
        <joint name="m" type="revolute"><parent link="b"/><child link="c"/><axis xyz="0 1 0"/><mimic joint="j" multiplier="-2" offset=".1"/></joint>
        <joint name="p" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/><origin xyz=".2 0 0" rpy=".1 .2 .3"/></joint>
        <joint name="j" type="revolute"><parent link="base_link"/><child link="a"/><axis xyz="0 0 1"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as temp:
            path=pathlib.Path(temp)/'model.urdf';path.write_text(text)
            old,new=LegacyFk(path),UrdfFk(path)
            for q in [[0.,0.],[.3,.5],[-.2,-.1]]:
                self.assertEqual(old.pose(['j','p'],q,'c'),new.pose(['j','p'],q,'c'))
                np.testing.assert_array_equal(old.jacobian(['j','p'],q,'c'),new.jacobian(['j','p'],q,'c'))

if __name__=='__main__':unittest.main()
