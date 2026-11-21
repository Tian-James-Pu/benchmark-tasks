"""Independent contact interpretation tests; no synthetic data enters real trials."""
import pathlib
import sys
import unittest
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from contact_rules import classify,normalize_signed_rows,symmetric_rows
from native_observer import Observe
from recipe_scoring import RecipeScore, FoodObservation, FinishObservation

ROOT = '/World/envs/env_0/'


def row(actor, normal=(0, 0, 1), force=.034):
    return (ROOT + actor, force, np.array(normal), np.array([0., 0., .005]))


def support(bottom):
    poses = np.array([[0., 0., .01, 0., 0., 0., 1.],
                      [0., 0., .025, 0., 0., 0., 1.]])
    return classify({'gnocchi_00': bottom, 'gnocchi_01': [row('gnocchi_00')]},
                    poses, np.zeros(3), np.eye(3), np.zeros(3), np.eye(3), .0475, .03556)


class Array:
    def __init__(self, data): self.data = np.asarray(data)
    def numpy(self): return self.data


def contact_reader(count=1, start=0, names=None):
    observer = Observe(None)
    observer.dt = .01
    observer.actor_cache = {}
    observer.sensor_names = ['gnocchi_00']
    data = tuple(Array(a) for a in (
        [[.03]] * 4, [[0., 0., 0.]] * 4, [[0., 0., 1.]] * 4,
        [[0.]] * 4, [count], [start], [7] * 4))
    observer.contacts = SimpleNamespace(
        sensor_paths=[ROOT+"gnocchi_00"], max_contact_data_count=4, get_raw_contact_data=lambda dt: data,
        get_other_actor_paths_from_ids=lambda ids: names)
    observer.wp = SimpleNamespace(array=lambda data, **kwargs: data, uint64=np.uint64)
    return observer


class AdapterOutcomes(unittest.TestCase):
    def test_shaft_support_excludes_release_without_counting_retrieval(self):
        shaft=(ROOT+'spoon',.034,np.array([0.,0.,1.]),np.array([.15,0.,.02]))
        x=support([row('plate'),shaft])
        self.assertTrue(x['plate'].all())
        self.assertTrue(x['tool'].all())
        self.assertFalse(x['spoon'].any())
        score=RecipeScore(2);score.ready=[True,True];score.spooned=[True,True]
        obs=[FoodObservation(0.,100.,False,False,True,True,bool(x['tool'][i])) for i in range(2)]
        self.assertEqual(score.step(2.,obs,FinishObservation(True,True,True,True)),0.)

    def test_negative_only_contact_preserves_upward_support(self):
        raw={'gnocchi_00':[row('plate',(0,0,-1),-.034)],'plate':[]}
        paths={name:ROOT+name for name in raw}
        rows=symmetric_rows(normalize_signed_rows(raw),paths)
        self.assertAlmostEqual(rows['gnocchi_00'][0][1],.034)
        np.testing.assert_array_equal(rows['gnocchi_00'][0][2],[0,0,1])
        np.testing.assert_array_equal(rows['plate'][0][2],[0,0,-1])

    def test_opposite_signed_reports_are_one_physical_pair(self):
        raw={'gnocchi_00':[row('plate',(0,0,-1),-.034)],
             'plate':[row('gnocchi_00',(0,0,-1),.034)]}
        rows=symmetric_rows(normalize_signed_rows(raw),{name:ROOT+name for name in raw})
        self.assertEqual([len(rows[k]) for k in raw],[1,1])
        self.assertAlmostEqual(sum(f*n[2] for a,f,n,p in rows['gnocchi_00']),.034)

    def test_zero_contact_does_not_supply_weight(self):
        self.assertEqual(normalize_signed_rows({'gnocchi_00':[row('plate',force=-0.)]}),{'gnocchi_00':[]})

    def test_gripper_holds_bottom_of_plate_contacting_stack(self):
        x = support([row('plate'), row('LeftArm/Geometry/finger', (1, 0, 0))])
        self.assertTrue(x['plate'].all())
        self.assertTrue(x['robot'].all())
        score = RecipeScore(2)
        score.ready = [True, True]
        score.spooned = [True, True]
        obs = [FoodObservation(0., 100., False, False, bool(x['plate'][i]),
                               True, bool(x['robot'][i])) for i in range(2)]
        self.assertEqual(score.step(2., obs, FinishObservation(True, True, True, True)), 0.)

    def test_releasing_stack_restores_valid_partial_outcome(self):
        x = support([row('plate')])
        self.assertTrue(x['plate'].all())
        self.assertFalse(x['robot'].any())
        score = RecipeScore(2)
        score.ready = [True, True]
        score.spooned = [True, True]
        obs = [FoodObservation(0., 100., False, False, True, True) for _ in range(2)]
        self.assertEqual(score.step(1., obs, FinishObservation(True, True, True, True)), 1.)

    def test_gripper_supported_spoon_stack_cannot_mark_retrieval(self):
        x = support([row('spoon'), row('RightArm/Geometry/finger', (1, 0, 0))])
        self.assertFalse(x['spoon'].any())

    def test_unknown_actor_is_not_silently_clean(self):
        with self.assertRaisesRegex(RuntimeError, 'Unresolved'):
            contact_reader(names=['']).contact_rows()

    def test_sparse_buffer_overflow_is_detected(self):
        with self.assertRaisesRegex(RuntimeError, 'buffer'):
            contact_reader(count=2, start=3, names=[ROOT+'plate']).contact_rows()

    def test_valid_contact_is_kept_with_actor_identity(self):
        reader = contact_reader(names=[ROOT+'plate'])
        rows = reader.contact_rows()
        self.assertEqual(rows['gnocchi_00'][0][0], ROOT+'plate')
        self.assertEqual(reader.actor_cache, {7: ROOT+'plate'})


if __name__ == '__main__': unittest.main(verbosity=2)
