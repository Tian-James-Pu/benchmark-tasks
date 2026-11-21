"""Scorer checks that run without the simulator: python tasks/clear-mr-unsafe-items-off-cart-before-mri/check_scoring.py

They build object states by hand and check task.py's `evaluate`/`points` and the Trial's record of a lost item."""

import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import task as t  # noqa: E402

TOP = t.TOP


class Box:
    """An axis-aligned item: what object_states gives for a SimReady prop."""

    def __init__(self, lo, hi, v=0.0):
        self.low, self.high = np.array(lo, float), np.array(hi, float)
        self.lin_vel, self.ang_vel = np.array([v, 0, 0]), np.zeros(3)


class Phone:
    """The phone as mechkit reports it: an origin and a rotation."""

    def __init__(self, x, y, z, v=0.0):
        self.pos, self.lin_vel, self.ang_vel = np.array([x, y, z]), np.array([v, 0, 0]), np.zeros(3)

    def to_world(self, p):
        return np.asarray(p) + self.pos


FAR = np.array([[0.2, 0.0, TOP + 0.4]] * 24)  # both grippers well away


def start():
    """The starting layout, as the simulator reports it once settled."""
    return {
        "phone": Phone(0.33, -0.155, t.SHELF_Z),
        "scissors": Box((0.301, 0.095, 0.829), (0.473, 0.144, 0.836)),
        "otoscope": Box((0.269, -0.067, 0.768), (0.461, 0.007, 0.822)),
        "water_bottle": Box((0.510, 0.070, 0.772), (0.572, 0.131, 0.940)),
        "glove_box": Box((0.350, 0.031, 0.773), (0.467, 0.209, 0.829)),
        "gauze_roll": Box((0.435, -0.166, 0.773), (0.485, -0.114, 0.815)),
    }


def cleared_layout():
    s = start()
    s["phone"] = Phone(0.215, -0.295, TOP)
    s["scissors"] = Box((0.243, 0.315, TOP - 0.001), (0.416, 0.367, TOP + 0.005))
    s["otoscope"] = Box((0.166, -0.415, TOP - 0.004), (0.358, -0.340, TOP + 0.050))
    return s


def check(name, got, want):
    assert got == want, f"{name}: got {got}, want {want}"
    print(f"ok  {name}: {got}")


def main():
    c, safe, lost = t.evaluate(start(), FAR)
    check("start: nothing cleared", (len(c), safe, lost, t.points(c, safe)), (0, True, [], 0.0))

    c, safe, lost = t.evaluate(cleared_layout(), FAR)
    check("all three in the hold areas, supplies on the cart", (sorted(c), safe, t.points(c, safe)), (["otoscope", "phone", "scissors"], True, 0.75))

    s = cleared_layout()
    s["gauze_roll"] = Box((0.275, -0.314, TOP), (0.326, -0.262, TOP + 0.042))
    c, safe, _ = t.evaluate(s, FAR)
    check("gauze roll taken off too: capped", (len(c), safe, t.points(c, safe)), (3, False, 0.5))

    s = cleared_layout()
    s["phone"] = Phone(0.20, -0.155, TOP)  # centre in front of the cart, its far end over the front lip, outside the tape
    c, _, _ = t.evaluate(s, FAR)
    check("phone just off the cart, not in a hold area", "phone" in c, False)

    s = cleared_layout()
    s["phone"] = Phone(0.215, -0.235, TOP)  # 4 mm over the tape's outer edge
    c, _, _ = t.evaluate(s, FAR)
    check("phone partly outside the tape", "phone" in c, False)

    s = cleared_layout()
    tips = FAR.copy()
    tips[12:] = [0.26, -0.40, TOP + 0.03]  # the right gripper still at the otoscope
    c, _, _ = t.evaluate(s, tips)
    check("otoscope still held", "otoscope" in c, False)

    s = cleared_layout()
    s["scissors"] = Box((0.243, 0.315, TOP + 0.02), (0.416, 0.367, TOP + 0.026))
    c, _, _ = t.evaluate(s, FAR)
    check("scissors 2 cm above the bench (held up)", "scissors" in c, False)

    s = cleared_layout()
    s["phone"] = Phone(0.215, -0.295, TOP, v=0.05)
    c, _, _ = t.evaluate(s, FAR)
    check("phone still sliding", "phone" in c, False)

    s = start()
    s["otoscope"] = Box((0.2, -0.7, 0.10), (0.39, -0.62, 0.15))
    _, _, lost = t.evaluate(s, FAR)
    check("otoscope on the floor", lost, ["otoscope"])

    # the record of a lost item lives in the Trial and is cleared only by its reset (the kit's reset hook)
    trial = t.Trial()
    trial.lost = True
    trial.reset()
    check("reset clears the record", trial.lost, False)
    print("all checks passed")


if __name__ == "__main__":
    main()
