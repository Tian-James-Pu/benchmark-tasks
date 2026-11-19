"""Temporary probe (deleted after use): how hard the right arm pushes straight down when sent D below a stiff sprung plate,
and how far its fingertips drift sideways while it pushes. Logs one JSON line per control step."""

from __future__ import annotations

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
PX, PY = float(os.environ.get("PROBE_X", "0.26")), float(os.environ.get("PROBE_Y", "-0.10"))
PLATE_TOP = TOP + 0.022
K = 20000.0
LOG = os.environ.get("PROBE_LOG", "/tmp/probe_force.jsonl")
LOOKS = {"plate": m.Look(color=m.rgb(0.6, 0.6, 0.62)), "base": m.Look(color=m.rgb(0.2, 0.2, 0.2))}

plate = m.slider("plate", "Z", -0.03, 0.0, [("p", m.box((0.06, 0.06, 0.004), (0, 0, -0.002)), "plate")],
                 [m.col_box("p_c", (0.06, 0.06, 0.004), (0, 0, -0.002))], mass=0.1, spring=K, damping=60.0, friction=0.0, effort=5000.0)
SCALE = m.mechanism("scale", (PX, PY, PLATE_TOP), [plate], LOOKS, frame_parts=[("b", m.box((0.07, 0.07, 0.004), (0, 0, -0.05)), "base")])
OBJECTS = [SCALE]

_T = [0]


def _step(env):
    _T[0] += 1
    s = m.state(env, "scale")
    tips = m.fingertips(env)[12:]
    rec = {"k": _T[0], "defl_mm": -s.joint("plate") * 1000, "force_N": -s.joint("plate") * K,
           "tip_mean": [float(v) for v in tips.mean(axis=0)], "tip_min_z": float(tips[:, 2].min())}
    with open(LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")


def _reset(env):
    _T[0] = 0
    with open(LOG, "a") as f:
        f.write(json.dumps({"reset": True}) + "\n")


m.on_step(_step)
m.on_reset(_reset)


def score(objects):
    return 0.0


def success(objects):
    return False


def demo():
    tcp0 = PLATE_TOP + 0.013
    w = [Waypoint(2.5, (PX, PY, tcp0 + 0.04), FINGER_CLOSED, "hover"), Waypoint(1.5, (PX, PY, tcp0 + 0.04), FINGER_CLOSED, "hold hover")]
    for d in (0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12):
        w += [Waypoint(1.5, (PX, PY, tcp0 - d), FINGER_CLOSED, f"to D {d}"), Waypoint(1.5, (PX, PY, tcp0 - d), FINGER_CLOSED, f"hold D {d}")]
    w += [Waypoint(1.5, (PX, PY, tcp0 + 0.05), FINGER_CLOSED, "up")]
    return {"right": w}
