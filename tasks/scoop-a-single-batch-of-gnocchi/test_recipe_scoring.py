"""Adversarial outcome tests; no native physics or contact certification."""
from dataclasses import replace
import unittest

from recipe_scoring import FoodObservation as Food, FinishObservation as Finish
from recipe_scoring import RecipeScore, plate_supported

HOT = Food(1., 100., True, False, False, True)
DRY = Food(0., 100., False, False, False, True)
FINISH = Finish(True, True, True, True)
ACTIVE = Finish(False, False, True, False)


def cook(score, present=None):
    present = set(range(score.n)) if present is None else set(present)
    observations = [HOT if i in present else DRY for i in range(score.n)]
    score.step(.05, observations, ACTIVE)
    score.step(60., observations, ACTIVE)


def serve(score, indices):
    indices = set(indices)
    score.step(.05, [replace(DRY, spoon_support=i in indices) for i in range(score.n)], ACTIVE)
    observations = [replace(DRY, on_plate_support_path=i in indices) for i in range(score.n)]
    return score.step(1., observations, FINISH)


class Outcomes(unittest.TestCase):
    def test_idle_never_scores_for_cooking_or_switching_off_alone(self):
        s = RecipeScore()
        cook(s)
        self.assertEqual(s.step(30., [HOT] * 32, FINISH), 0.)

    def test_one_lost_piece_does_not_block_the_other_31(self):
        s = RecipeScore()
        cook(s, range(31))
        self.assertEqual(serve(s, range(31)), 31 / 32)
        self.assertFalse(s.ready[31])

    def test_raw_piece_on_plate_does_not_erase_cooked_partial_work(self):
        s = RecipeScore()
        cook(s, range(31))
        self.assertEqual(serve(s, range(32)), 31 / 32)

    def test_early_spoonful_can_return_finish_and_be_retrieved_again(self):
        s = RecipeScore(2)
        s.step(.05, [HOT, HOT], ACTIVE)
        s.step(20., [HOT, HOT], ACTIVE)
        s.step(40., [DRY, HOT], ACTIVE)
        self.assertEqual(s.dwell, [20., 60.])
        self.assertEqual(serve(s, [0, 1]), .5)
        s.step(40., [HOT, DRY], ACTIVE)
        self.assertTrue(all(s.ready))
        self.assertFalse(s.spooned[0])
        self.assertEqual(serve(s, [0, 1]), 1.)

    def test_turning_off_heat_does_not_invalidate_hot_residual_water(self):
        s = RecipeScore(1)
        s.step(.05, [HOT], FINISH)
        s.step(60., [HOT], FINISH)
        self.assertEqual(serve(s, [0]), 1.)

    def test_dry_cold_and_shallow_immersion_do_not_cook(self):
        for observation in [DRY, replace(HOT, water_c=94.9), replace(HOT, immersion=.749)]:
            s = RecipeScore(1)
            s.step(.05, [HOT], ACTIVE)
            s.step(1000., [observation], ACTIVE)
            self.assertEqual(s.dwell[0], 0.)
            self.assertFalse(s.ready[0])

    def test_supported_surface_position_is_not_a_float_event(self):
        for support in ['spoon_support', 'tool_or_gripper_support', 'on_plate_support_path']:
            s = RecipeScore(1)
            s.step(300., [replace(HOT, **{support: True})], ACTIVE)
            self.assertFalse(s.floated[0])
            self.assertEqual(serve(s, [0]), 0.)

    def test_direct_hand_transfer_does_not_substitute_for_spoon_retrieval(self):
        s = RecipeScore(1)
        cook(s)
        self.assertEqual(s.step(10., [replace(DRY, on_plate_support_path=True)], FINISH), 0.)

    def test_spoon_then_hand_delivery_requires_new_spoon_retrieval(self):
        s = RecipeScore(1)
        cook(s)
        s.step(.05, [replace(DRY, spoon_support=True)], ACTIVE)
        self.assertTrue(s.spooned[0])
        s.step(.05, [replace(DRY, tool_or_gripper_support=True, gripper_carried=True)], ACTIVE)
        self.assertFalse(s.spooned[0])
        self.assertEqual(s.step(1., [replace(DRY, on_plate_support_path=True)], FINISH), 0.)
        self.assertEqual(serve(s, [0]), 1.)

    def test_incidental_touch_of_plated_food_does_not_erase_spoon_delivery(self):
        s = RecipeScore(1)
        cook(s)
        self.assertEqual(serve(s, [0]), 1.)
        s.step(.05, [replace(DRY, on_plate_support_path=True,
                            tool_or_gripper_support=True)], ACTIVE)
        self.assertTrue(s.spooned[0])
        self.assertEqual(s.step(1., [replace(DRY, on_plate_support_path=True)], FINISH), 1.)

    def test_tool_supported_or_near_plate_food_does_not_count(self):
        for change in [{'on_plate_support_path': False}, {'tool_or_gripper_support': True}]:
            s = RecipeScore(1)
            cook(s)
            self.assertEqual(serve(s, [0]), 1.)
            obs = replace(DRY, on_plate_support_path=True, **{k: v for k, v in change.items() if k != 'on_plate_support_path'})
            if 'on_plate_support_path' in change:
                obs = replace(obs, **change)
            self.assertEqual(s.step(.05, [obs], FINISH), 0.)

    def test_each_unfinished_station_condition_caps_score(self):
        for field in ['burner_off', 'spoon_released_and_resting', 'cookware_stable', 'hands_clear']:
            s = RecipeScore()
            cook(s)
            serve(s, range(32))
            self.assertEqual(s.step(.05, [replace(DRY, on_plate_support_path=True)] * 32,
                                    replace(FINISH, **{field: False})), .5)

    def test_transient_full_pose_is_not_full_score(self):
        s = RecipeScore(1)
        cook(s)
        s.step(.05, [replace(DRY, spoon_support=True)], ACTIVE)
        plated = replace(DRY, on_plate_support_path=True)
        self.assertEqual(s.step(.99, [plated], FINISH), 0.)
        self.assertEqual(s.step(.01, [plated], FINISH), 1.)
        self.assertEqual(s.step(.05, [replace(plated, still=False)], FINISH), 0.)

    def test_unanchored_contact_cycle_is_not_plate_support(self):
        self.assertEqual(plate_supported(4, [], [(0, 1), (1, 2), (2, 0)]), (False,) * 4)
        self.assertEqual(plate_supported(4, [0], [(0, 1), (1, 2), (2, 1)]),
                         (True, True, True, False))

    def test_time_subdivision_preserves_cooking_and_reset_clears_it(self):
        a, b = RecipeScore(1), RecipeScore(1)
        for s in [a, b]:
            s.step(.05, [HOT], ACTIVE)
        a.step(60., [HOT], ACTIVE)
        for _ in range(1200):
            b.step(.05, [HOT], ACTIVE)
        self.assertTrue(a.ready[0] and b.ready[0])
        self.assertAlmostEqual(a.dwell[0], b.dwell[0], places=8)
        b.reset()
        self.assertFalse(any(b.ready + b.spooned + b.floated))
        self.assertEqual(b.last_score, 0.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
