"""TEMPORARY: measure how hard the right arm pushes along +x (a sprung plate). Delete after use."""
import json, math, os, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
SCENE = "station"
TOP = TABLE_TOP[SCENE]
K = 200.0
LOOKS = {"p": m.Look(m.rgb(0.8, 0.2, 0.2))}
plate = m.slider("plate", "X", 0.0, 0.15, [("pl", m.box((0.01, 0.08, 0.10), (0.005, 0, 0)), "p")], [m.col_box("pl_c", (0.01, 0.08, 0.10), (0.005, 0, 0))], spring=K, damping=5.0, mass=0.2)
PROBE = m.mechanism("probe", (0.21, -0.06, TOP + 0.11), [plate], LOOKS, frame_parts=[("f", m.box((0.01, 0.01, 0.01), (0.2, 0, -0.1)), "p")])
OBJECTS = [PROBE]
LOG = os.environ.get("OUTLET_LOG")
N = [0]
def _step(env):
    N[0] += 1
    if LOG and N[0] % 5 == 0:
        s = m.state(env, "probe")
        with open(LOG, "a") as f:
            f.write(json.dumps({"n": N[0], "d": s.joint("plate"), "F": K * s.joint("plate"), "tip": m.fingertips(env)[12].round(3).tolist()}) + "\n")
m.on_step(_step)
def score(o): return 0.0
def success(o): return False
def demo():
    yaw = float(os.environ.get("OUTLET_YAW", "0"))
    z = TOP + 0.11 + 0.013
    return {"right": [Waypoint(2.0, (0.15, -0.06, TOP + 0.20), 0.0, "a", yaw), Waypoint(1.5, (0.18, -0.06, z), 0.0, "b", yaw),
                      Waypoint(4.0, (0.30, -0.06, z), 0.0, "push", yaw), Waypoint(3.0, (0.30, -0.06, z), 0.0, "hold", yaw)]}
