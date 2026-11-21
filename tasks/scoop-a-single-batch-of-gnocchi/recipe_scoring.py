"""Outcome scoring for a single gnocchi batch, independent of simulator bindings.

The adapter must derive support from physical contacts, not pose proximity.
This module does not certify the adapter, heat model, or food physics.
"""
from dataclasses import dataclass
import math


def plate_supported(n, plate_contacts, food_support_edges):
    """Return food with a directed support path to the plate.

    plate_contacts are food indices actually supported by the plate.
    Each edge (lower, upper) means lower provides upward support to upper.
    A floating cycle or a path rooted only in the spoon is not plate support.
    """
    supported = set(plate_contacts)
    edges = tuple(food_support_edges)
    if any(not isinstance(i, int) or not 0 <= i < n
           for i in list(supported) + [i for edge in edges for i in edge]):
        raise ValueError("Invalid food index in support graph")
    while True:
        enlarged = supported | {upper for lower, upper in edges if lower in supported}
        if enlarged == supported:
            return tuple(i in supported for i in range(n))
        supported = enlarged


@dataclass(frozen=True)
class FoodObservation:
    immersion: float
    water_c: float
    free_float: bool
    spoon_support: bool
    on_plate_support_path: bool
    still: bool
    tool_or_gripper_support: bool = False
    disqualified: bool = False
    gripper_carried: bool = False


@dataclass(frozen=True)
class FinishObservation:
    burner_off: bool
    spoon_released_and_resting: bool
    cookware_stable: bool
    hands_clear: bool


class RecipeScore:
    """Float, cook another60 immersed seconds, spoon out, and serve.

    Cooking is per piece. The standard batch-wide stopwatch is a sufficient
    observation shortcut, not a dependency that makes one lost piece zero all.
    Water>=95C and immersion>=75% are declared operational tolerances, not
    measured gelatinization thresholds. dt is elapsed simulated/real time.
    """
    def __init__(self, n=32, dwell_s=60.0, settle_s=1.0):
        if n <= 0 or dwell_s <= 0 or settle_s <= 0:
            raise ValueError("Positive count and times required")
        self.n, self.dwell_s, self.settle_s = n, dwell_s, settle_s
        self.reset()

    def reset(self):
        self.floated = [False] * self.n
        self.dwell = [0.0] * self.n
        self.ready = [False] * self.n
        self.spooned = [False] * self.n
        self.stable = [0.0] * self.n
        self.finish_hold = 0.0
        self.last_score = 0.0

    def step(self, dt, observations, finish):
        if not math.isfinite(dt) or dt <= 0 or len(observations) != self.n:
            raise ValueError("Finite positive dt and one observation per piece required")
        good = []
        for i, obs in enumerate(observations):
            if (not math.isfinite(obs.immersion) or not 0 <= obs.immersion <= 1
                    or not math.isfinite(obs.water_c)):
                raise ValueError("Invalid immersion or water temperature")
            hot = obs.immersion >= .75 and obs.water_c >= 95.0
            # A first observed float does not grant the preceding interval.
            if self.floated[i] and hot:
                self.dwell[i] += dt
            unsupported_float = (obs.free_float and not obs.spoon_support
                                 and not obs.tool_or_gripper_support
                                 and not obs.on_plate_support_path)
            if unsupported_float and hot:
                self.floated[i] = True
            self.ready[i] = self.ready[i] or self.dwell[i] + 1e-9 >= self.dwell_s

            # Reimmersion or hand carrying requires a subsequent spoon retrieval.
            # Incidental touch is not carrying; completed cooking is preserved.
            if obs.immersion > .05 or obs.gripper_carried:
                self.spooned[i] = False
            elif obs.spoon_support and self.ready[i]:
                self.spooned[i] = True

            current = (self.ready[i] and self.spooned[i]
                       and obs.on_plate_support_path and obs.still
                       and not obs.tool_or_gripper_support and not obs.disqualified
                       and not obs.gripper_carried)
            self.stable[i] = self.stable[i] + dt if current else 0.0
            good.append(current and self.stable[i] + 1e-9 >= self.settle_s)

        finished = (finish.burner_off and finish.spoon_released_and_resting
                    and finish.cookware_stable and finish.hands_clear)
        self.finish_hold = self.finish_hold + dt if finished else 0.0
        fraction = sum(good) / self.n
        # Prevent transient1.0 before every finishing condition has held1s.
        finish_stable = self.finish_hold + 1e-9 >= self.settle_s
        self.last_score = fraction if finish_stable else min(fraction, .5)
        return self.last_score
