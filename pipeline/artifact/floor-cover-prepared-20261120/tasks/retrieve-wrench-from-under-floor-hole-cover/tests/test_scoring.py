"""CPU checks of the scorer: every outcome in task.json, and the states that must not count as covered or as put away.

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest

from yam_arena import task_file

T = task_file.load(pathlib.Path(__file__).resolve().parents[1] / "task.py").module
m = T.m
TOP = T.TOP
WR = np.array([0.0955, 0.0145, 0.0043])  # the wrench's half extents lying flat along x


def cover_state(dx=0.0, dy=0.0, dz=0.0, tilt_deg=0.0, flipped=False, moving=False):
    q = np.array([0.0, 0.0, 0.0, 1.0])
    if tilt_deg:
        h = math.radians(tilt_deg) / 2
        q = np.array([math.sin(h), 0.0, 0.0, math.cos(h)])
    if flipped:
        q = np.array([1.0, 0.0, 0.0, 0.0])
    pos = np.array([T.HOLE_C[0] + dx, T.HOLE_C[1] + dy, T.DECK_Z + T.COVER_T / 2 + dz])
    v = np.array([0.05, 0, 0]) if moving else np.zeros(3)
    return m.PartState(pos=pos, quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3), low=pos, high=pos)


def wrench_state(centre, moving=False):
    c = np.asarray(centre, float)
    v = np.array([0.05, 0, 0]) if moving else np.zeros(3)
    return task_file.ObjectState(pos=c, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=v, ang_vel=np.zeros(3), low=c - WR, high=c + WR)


IN_OPENING = (T.WRENCH_C[0], T.WRENCH_C[1], TOP + WR[2])
ON_BASKET_FLOOR = (T.BASKET_POS[0], T.BASKET_POS[1], TOP + T.BASKET_FLOOR + WR[2])
ON_DECK = (0.125, 0.0, T.DECK_Z + WR[2])
FAR = np.full((24, 3), 5.0)  # fingertips far away


def tips_at(point):
    return np.tile(np.asarray(point, float), (24, 1))


def score_of(cover, wrench, tips=FAR):
    return T.tier(T.measure(cover, wrench, tips))


def test_start_scores_zero():
    assert score_of(cover_state(), wrench_state(IN_OPENING)) == 0.0


def test_full_credit():
    assert score_of(cover_state(), wrench_state(ON_BASKET_FLOOR)) == 1.0


def test_held_wrench_above_basket_floor_cannot_score_full():
    """The earlier review's point: a wrench held over the basket floor is not put away."""
    for lift in (0.006, 0.010, 0.025, 0.06):
        c = np.array(ON_BASKET_FLOOR) + [0, 0, lift]
        held = tips_at(c)
        r = T.measure(cover_state(), wrench_state(c), held)
        assert not r["put_away"]
        assert T.tier(r) == 0.5
        # even with the fingers away, a wrench that is not down on the floor is not put away
        assert score_of(cover_state(), wrench_state(c)) == 0.5


def test_wrench_on_floor_but_still_gripped_is_075():
    c = np.array(ON_BASKET_FLOOR)
    assert score_of(cover_state(), wrench_state(c), tips_at(c + [0, 0.02, 0.005])) == 0.75


def test_moving_wrench_is_not_put_away():
    assert score_of(cover_state(), wrench_state(ON_BASKET_FLOOR, moving=True)) == 0.5


def test_gripper_on_cover_is_075():
    c = np.array([T.HOLE_C[0], T.HOLE_C[1], T.DECK_Z + 0.01])
    assert score_of(cover_state(), wrench_state(ON_BASKET_FLOOR), tips_at(c)) == 0.75


def test_hole_left_open_is_025():
    assert score_of(cover_state(dy=0.25), wrench_state(ON_BASKET_FLOOR)) == 0.25


@pytest.mark.parametrize("kw", [dict(dz=T.CLEAT_H), dict(dz=0.005), dict(dy=0.012), dict(dx=-0.015), dict(tilt_deg=3.0), dict(flipped=True)])
def test_not_covered(kw):
    """On a cleat (19 mm up), 5 mm proud, 12 or 15 mm off centre, tilted 3 degrees, or upside down (cleats up)."""
    assert score_of(cover_state(**kw), wrench_state(ON_BASKET_FLOOR)) == 0.25


def test_covered_within_play():
    assert score_of(cover_state(dx=0.006, dy=-0.006), wrench_state(ON_BASKET_FLOOR)) == 1.0


def test_wrench_out_and_hole_covered_is_05():
    assert score_of(cover_state(), wrench_state(ON_DECK)) == 0.5


def test_wrench_out_hole_open_is_zero():
    assert score_of(cover_state(dy=0.25), wrench_state(ON_DECK)) == 0.0


def test_wrench_still_in_opening_is_zero_even_uncovered():
    assert score_of(cover_state(dy=0.25), wrench_state(IN_OPENING)) == 0.0


def test_wrench_on_basket_rim_is_not_put_away():
    c = (T.BASKET_POS[0], T.BASKET_POS[1] + T.BASKET_IN_Y, TOP + 0.132 + WR[2])
    assert score_of(cover_state(), wrench_state(c)) == 0.5


def test_score_is_capped_before_the_hold():
    T.TRIAL.reset()
    T.TRIAL.last = T.measure(cover_state(), wrench_state(ON_BASKET_FLOOR), FAR)
    T.TRIAL.hold = T.HOLD_STEPS - 1
    assert T.score({}) == 0.75 and not T.success({})
    T.TRIAL.hold = T.HOLD_STEPS
    assert T.score({}) == 1.0 and T.success({})
    T.TRIAL.reset()
    assert T.score({}) == 0.0


def test_geometry():
    assert 2 * T.CLEAT_OUT < T.HOLE  # the cleats drop into the opening
    assert (T.HOLE - 2 * T.CLEAT_OUT) / 2 == pytest.approx(0.006)
    assert T.COVER - T.HOLE == pytest.approx(0.100)  # 50 mm bearing on the deck all round
    assert T.SLOT_W < 0.0508  # the hand slot is narrower than OSHA's 2 in "hole"
