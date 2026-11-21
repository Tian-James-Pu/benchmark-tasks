"""Ready a 70 mm travel refractor for showing the sun: the full-aperture solar filter on the front of the dew shield and secured by its
three nylon thumbscrews, and the finder capped. The instruction names none of this.

The telescope is a Celestron Travel Scope 70 optical tube (70 mm f/5.7, 431.8 mm long, 98.4 mm across the dew shield), parked pointing
straight up on a low alt-az stand whose head pans on a friction clutch about a post 75 mm behind the tube's axis. Its filter is the maker's
own EclipSmart filter for that tube (item 94221: an ABS cell 105.1 mm outside and 100.6 mm inside, 23 mm deep, 29 g, 42 mm of solar film,
three nylon safety screws 120 degrees apart). The filter lies face up on a foam boss in the open accessory case, and the finder's cap on a
smaller boss beside it.

Model (declared here, and in the README):
- Each thumbscrew is a hinge on the cell (its own axis, pointing in at the dew shield) with a nylon tip on a slider along the same axis.
  Turning the screw drives the tip in at 1.27 mm a turn (1/4-20 thread assumed), up to the dew shield: the tip's contact is computed each
  step from the cell's pose on the tube. A screw turned 0.4 mm (about a third of a turn) past first contact is "snug": its hinge is locked,
  as a finger-tight screw stops turning, and its tip is held 0.2 mm into its contact.
- The tips start 0.5 mm proud of the cell's inner wall, which leaves the cell 0.6 mm of play each side on the dew shield.
- A parallel gripper can only turn a screw by pinching its head with the fingers pointing along the screw, which the arms can do from the
  robot's side of the tube only: two screws can be reached as the filter goes on, and the third faces away until the telescope is panned.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0  # the full state must hold 3 s (the scorer also asks 1 s of stillness inside it)

# ---------------------------------------------------------------------------------------------------------------------------------
# Geometry (world: x forward from the arms, y left, z up; metres)

P = np.array([0.36, 0.0])  # the stand's pan axis
ARM = 0.075  # the tube's axis from the pan axis (Vixen saddle on the head, the head's tilt axis beside the tube)
PSI0 = math.pi  # the tube's bearing from the pan axis at the start: towards the robot
TUBE_Z0, TUBE_Z1 = 0.100, 0.100 + 0.4318  # the OTA's back end and the dew shield's rim, above the table
R_TUBE = 0.041  # the main tube's outside radius (82 mm: from product photos, an estimate)
R_DEW = 0.0492  # the dew shield: 98.4 mm across (Celestron's "optical tube diameter")
DEW_Z0 = TUBE_Z1 - 0.110  # the dew shield's lower end
FINDER_R = R_TUBE + 0.036  # the finder's axis from the tube's: on its bracket
FINDER_REL = -math.pi / 2  # the finder's bearing from the tube's axis, relative to the tube's bearing from the pan axis: to the robot's left at the start
FINDER_Z0, FINDER_Z1 = 0.255, 0.395  # the 5x24 finder's eyepiece end and its objective cell's front
R_FCELL = 0.016  # the finder's objective cell: 32 mm across (estimate)
HANDLE_Z, HANDLE_R0, HANDLE_R1 = 0.085, 0.03, 0.165  # the pan handle: height, and its rod from and to, out from the pan axis
EYE_REL = math.radians(135.0)  # the eyepiece's bearing (on the 45-degree diagonal), relative to the tube's bearing from the pan axis
HANDLE_BEAR = -math.pi / 2  # the handle's bearing relative to the tube's: a quarter turn clockwise of it, seen from above (to the robot's left)

# The filter cell (its frame: origin at the centre of the plate's inner face, the face that rests on the dew shield's rim; +z out of the front)
R_CELL_OUT, R_CELL_IN = 0.05255, 0.0503  # 105.1 mm and 100.6 mm (Celestron 94221)
PLATE = 0.0025  # the front plate's thickness
SKIRT = 0.023 - PLATE  # the skirt below the plate's inner face (23 mm overall)
R_FILM = 0.021  # 42 mm clear aperture
SCREW_Z = PLATE - 0.014  # the thumbscrews' axes, 14 mm behind the front face
HEAD_R, HEAD_L = 0.005, 0.008  # knurled heads: 10 mm across, standing 8 mm proud
TIP_R = 0.0025
TIP_PROUD = 0.0005  # the tips stand 0.5 mm in from the inner wall at the start
PITCH = 0.00127  # 1/4-20
TIP_MAX = 0.005  # a screw backed fully in stands 5 mm proud of the inner wall
SNUG = 0.0004  # turned this far (about a third of a turn) past first contact with the dew shield: snug
PRELOAD = 0.0002  # a snug tip's set into the nylon and the shield's paint: its frozen target past first contact
TIP_K, TIP_EFFORT = 20000.0, 20.0
SCREW_BEARINGS = (2 * math.pi / 3, 4 * math.pi / 3, 0.0)  # in the cell's frame; s1 and s2 face the robot as it goes on, s3 faces away

# The cap (its frame: origin at the centre of its inner top face; open end down)
R_CAP_IN, R_CAP_OUT, CAP_DEPTH, CAP_TOP = 0.0166, 0.0184, 0.014, 0.002  # a slip fit: 0.6 mm of play each side on the 32 mm cell

# The accessory case on the right, foam bosses on its top
CASE_C, CASE_SIZE = np.array([0.225, -0.165]), np.array([0.15, 0.16, 0.15])
CELL0 = np.array([0.22, -0.13])  # the filter's boss
BOSS_R, BOSS_H = 0.046, 0.045
CAP0 = np.array([0.28, -0.205])  # the cap's boss
CAPBOSS_R, CAPBOSS_H = 0.0160, 0.075  # the cap sits on its boss as on the finder
CASE_TOP = CASE_SIZE[2]

# ---------------------------------------------------------------------------------------------------------------------------------
# Looks

LOOKS = {
    "abs": m.Look(color=m.rgb(0.05, 0.05, 0.055), roughness=0.55),
    "film": m.Look(color=m.rgb(0.78, 0.78, 0.80), metallic=0.9, roughness=0.15),
    "nylon": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.5),
    "tube": m.Look(color=m.rgb(0.07, 0.07, 0.08), metallic=0.3, roughness=0.35),
    "lens": m.Look(color=m.rgb(0.10, 0.16, 0.20), metallic=0.2, roughness=0.05),
    "silver": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.8, roughness=0.35),
    "grey": m.Look(color=m.rgb(0.35, 0.35, 0.37), roughness=0.6),
    "case": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.4, roughness=0.4),
    "foam": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.95),
    "rubber": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.9),
    "white": m.Look(color=m.rgb(0.85, 0.85, 0.85), roughness=0.5),
}

# ---------------------------------------------------------------------------------------------------------------------------------
# Joints on axes that are not x, y or z of the part: the kit writes every joint frame unrotated, so a joint listed here gets its frame
# turned about z (both bodies alike); its "X" axis then points along that bearing.

_JOINT_YAW: dict[str, float] = {}
_plain_joint_usda = m._joint_usda


def _joint_usda(link):
    text = _plain_joint_usda(link)
    yaw = _JOINT_YAW.get(link.name)
    if yaw is None:
        return text
    q = f"({math.cos(yaw / 2):.7f}, 0, 0, {math.sin(yaw / 2):.7f})"
    for k in ("0", "1"):
        text = text.replace(f"quatf physics:localRot{k} = (1, 0, 0, 0)", f"quatf physics:localRot{k} = {q}")
    return text


m._joint_usda = _joint_usda


def _along(mesh_z: m.Mesh, bearing: float) -> m.Mesh:
    """A mesh built along +z, laid along the horizontal bearing (radians from +x)."""
    return mesh_z.turned(90, "y").turned(math.degrees(bearing), "z")


def _cyl_between(r: float, a, b, segments: int = 24) -> m.Mesh:
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    n = float(np.linalg.norm(d))
    mesh = m.cylinder(r, n, (0, 0, n / 2), segments=segments)
    z = d / n
    x = np.cross([0, 0, 1.0], z) if abs(z[2]) < 0.99 else np.array([1.0, 0, 0])
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z], axis=1)
    return m.Mesh(mesh.v @ R.T + a, mesh.f)


# ---------------------------------------------------------------------------------------------------------------------------------
# The telescope on its stand: the base is fixed; everything above the pan clutch turns on "pan" about z through P.

# the finder's objective cell: its front edge rounded off by 1 mm over its last 1.5 mm, as a moulded cell's is
FCELL_MESH = m.lathe([(0.0, FINDER_Z1 - 0.03), (R_FCELL, FINDER_Z1 - 0.03), (R_FCELL, FINDER_Z1 - 0.0015), (R_FCELL - 0.001, FINDER_Z1),
                      (0.0, FINDER_Z1)], segments=40)


def _scope():
    # in the part's frame: origin on the table under the pan axis
    t = ARM * np.array([math.cos(PSI0), math.sin(PSI0)])  # the tube's axis
    fdir = np.array([math.cos(PSI0 + FINDER_REL), math.sin(PSI0 + FINDER_REL)])  # the finder's side of the tube
    edir = np.array([math.cos(PSI0 + EYE_REL), math.sin(PSI0 + EYE_REL)])  # the diagonal turns the eyepiece out, back beside the post
    f = t + (FINDER_R) * fdir
    hb = PSI0 + HANDLE_BEAR
    hdir = np.array([math.cos(hb), math.sin(hb), 0.0])
    base_parts = [
        ("base", m.cylinder(0.075, 0.02, (0, 0, 0.01), segments=40), "grey"),
        ("post_low", m.cylinder(0.018, 0.045, (0, 0, 0.0425)), "silver"),
    ]
    base_cols = [m.col_cylinder("base_c", 0.075, 0.02, (0, 0, 0.01)), m.col_cylinder("post_low_c", 0.018, 0.045, (0, 0, 0.0425))]
    tube_mid = (TUBE_Z0 + DEW_Z0) / 2
    head_mesh = m.box((0.05, 0.06, 0.07), (ARM * 0.30, 0.0, 0.30)).turned(math.degrees(PSI0), "z")
    parts = [
        ("clutch", m.cylinder(0.024, 0.02, (0, 0, 0.075)), "rubber"),
        ("post", m.cylinder(0.018, 0.22, (0, 0, 0.085 + 0.11)), "silver"),
        ("head", head_mesh, "grey"),
        ("saddle", m.box((0.016, 0.045, 0.14), (ARM * 0.62, 0.0, 0.30)).turned(math.degrees(PSI0), "z"), "grey"),
        ("tube", m.cylinder(R_TUBE, DEW_Z0 - TUBE_Z0, (t[0], t[1], tube_mid), segments=48), "tube"),
        ("dew", m.lathe([(R_DEW - 0.0012, DEW_Z0), (R_DEW, DEW_Z0), (R_DEW, TUBE_Z1 - 0.0015), (R_DEW - 0.0015, TUBE_Z1), (R_DEW - 0.003, TUBE_Z1),
                         (R_DEW - 0.003, TUBE_Z1 - 0.004), (0.0, TUBE_Z1 - 0.004)], segments=64).moved((t[0], t[1], 0)), "tube"),
        ("objective", m.cylinder(0.036, 0.002, (t[0], t[1], TUBE_Z1 - 0.004)), "lens"),
        ("focuser", m.cylinder(0.020, 0.035, (t[0], t[1], TUBE_Z0 - 0.0175)), "tube"),
        ("diagonal", m.box((0.035, 0.035, 0.035), (t[0], t[1], TUBE_Z0 - 0.052)), "tube"),
        ("eyepiece", _cyl_between(0.017, (t[0] + 0.015 * edir[0], t[1] + 0.015 * edir[1], TUBE_Z0 - 0.052),
                                  (t[0] + 0.070 * edir[0], t[1] + 0.070 * edir[1], TUBE_Z0 - 0.052)), "rubber"),
        ("bracket", m.box((0.03, 0.04, 0.10), ((t[0] + f[0]) / 2, (t[1] + f[1]) / 2, (FINDER_Z0 + FINDER_Z1) / 2 - 0.01)), "tube"),
        ("finder", m.cylinder(0.0125, FINDER_Z1 - FINDER_Z0 - 0.03, (f[0], f[1], FINDER_Z0 + (FINDER_Z1 - FINDER_Z0 - 0.03) / 2)), "tube"),
        ("fcell", FCELL_MESH.moved((f[0], f[1], 0.0)), "tube"),
        ("flens", m.cylinder(0.012, 0.001, (f[0], f[1], FINDER_Z1 - 0.0015)), "lens"),
        ("handle_rod", _cyl_between(0.006, HANDLE_R0 * hdir + (0, 0, HANDLE_Z), (HANDLE_R1 - 0.045) * hdir + (0, 0, HANDLE_Z)), "silver"),
        ("handle_grip", _cyl_between(0.010, (HANDLE_R1 - 0.05) * hdir + (0, 0, HANDLE_Z), HANDLE_R1 * hdir + (0, 0, HANDLE_Z)), "rubber"),
        ("handle_hub", m.cylinder(0.022, 0.02, (0, 0, HANDLE_Z)), "grey"),
    ]
    cols = [
        m.col_cylinder("post_c", 0.018, 0.22, (0, 0, 0.085 + 0.11)),
        m.col_hull("head_c", head_mesh),
        m.col_cylinder("tube_c", R_TUBE, DEW_Z0 - TUBE_Z0, (t[0], t[1], tube_mid)),
        m.col_hull("dew_c", m.cylinder(R_DEW, TUBE_Z1 - DEW_Z0 - 0.0015, (t[0], t[1], (DEW_Z0 + TUBE_Z1 - 0.0015) / 2), segments=48)),
        m.col_hull("dew_lip_c", m.cylinder(R_DEW - 0.0015, 0.0015, (t[0], t[1], TUBE_Z1 - 0.00075), segments=48)),
        m.col_cylinder("focuser_c", 0.020, 0.035, (t[0], t[1], TUBE_Z0 - 0.0175)),
        m.col_box("diagonal_c", (0.035, 0.035, 0.035), (t[0], t[1], TUBE_Z0 - 0.052)),
        m.col_hull("eyepiece_c", _cyl_between(0.017, (t[0] + 0.015 * edir[0], t[1] + 0.015 * edir[1], TUBE_Z0 - 0.052),
                                              (t[0] + 0.070 * edir[0], t[1] + 0.070 * edir[1], TUBE_Z0 - 0.052), 12)),
        m.col_box("bracket_c", (0.03, 0.04, 0.10), ((t[0] + f[0]) / 2, (t[1] + f[1]) / 2, (FINDER_Z0 + FINDER_Z1) / 2 - 0.01)),
        m.col_cylinder("finder_c", 0.0125, FINDER_Z1 - FINDER_Z0 - 0.03, (f[0], f[1], FINDER_Z0 + (FINDER_Z1 - FINDER_Z0 - 0.03) / 2)),
        m.col_hull("fcell_c", FCELL_MESH.moved((f[0], f[1], 0.0))),
        m.col_hull("handle_rod_c", _cyl_between(0.006, HANDLE_R0 * hdir + (0, 0, HANDLE_Z), (HANDLE_R1 - 0.045) * hdir + (0, 0, HANDLE_Z), 12)),
        m.col_hull("handle_grip_c", _cyl_between(0.010, (HANDLE_R1 - 0.05) * hdir + (0, 0, HANDLE_Z), HANDLE_R1 * hdir + (0, 0, HANDLE_Z), 12)),
        m.col_cylinder("handle_hub_c", 0.022, 0.02, (0, 0, HANDLE_Z)),
    ]
    pan = m.hinge("pan", "Z", (0.0, 0.0, 0.07), float("-inf"), float("inf"), parts, cols, mass=1.3, friction=0.8, damping=0.8,
                  inertia=(0.03, 0.03, 0.012))
    return m.mechanism("scope", (P[0], P[1], TOP), [pan], LOOKS, frame_parts=base_parts, frame_colliders=base_cols, solver_iters=32)


# ---------------------------------------------------------------------------------------------------------------------------------
# The filter cell: a free articulation; three screw hinges and three tip sliders on axes pointing in at the dew shield.

def _cell():
    parts = [
        ("plate", m.lathe([(0.0, 0.0), (R_CELL_OUT, 0.0), (R_CELL_OUT, PLATE - 0.0006), (R_CELL_OUT - 0.0006, PLATE), (R_FILM + 0.004, PLATE),
                           (R_FILM + 0.004, PLATE + 0.0015), (R_FILM + 0.0015, PLATE + 0.0015), (R_FILM + 0.0015, PLATE - 0.0008),
                           (0.0, PLATE - 0.0008)], segments=64), "abs"),
        ("film", m.cylinder(R_FILM + 0.001, 0.0003, (0, 0, PLATE - 0.0006), segments=48), "film"),
        ("skirt", m.lathe([(R_CELL_IN, -SKIRT), (R_CELL_OUT, -SKIRT), (R_CELL_OUT, 0.0), (R_CELL_IN, 0.0), (R_CELL_IN, -SKIRT)], segments=64), "abs"),
    ]
    cols = [m.col_cylinder("plate_c", R_CELL_OUT, PLATE, (0, 0, PLATE / 2))]
    n = 20
    wall = R_CELL_OUT - R_CELL_IN
    width = 2 * math.pi * R_CELL_OUT / n + 0.002
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        seg = m.box((wall, width, SKIRT), ((R_CELL_IN + R_CELL_OUT) / 2, 0, -SKIRT / 2)).turned(math.degrees(a), "z")
        cols.append(m.col_hull(f"skirt{k}_c", seg))
    links = []
    for i, b in enumerate(SCREW_BEARINGS, start=1):
        s = np.array([math.cos(b), math.sin(b), 0.0])
        head_mesh = _along(m.cylinder(HEAD_R, HEAD_L, (0, 0, HEAD_L / 2), segments=16), b).moved(R_CELL_OUT * s + (0, 0, SCREW_Z))
        knurl = _along(m.cylinder(HEAD_R + 0.0004, 0.002, (0, 0, HEAD_L - 0.001), segments=12), b).moved(R_CELL_OUT * s + (0, 0, SCREW_Z))
        boss = _along(m.cylinder(0.0045, 0.0015, (0, 0, 0.00075), segments=16), b).moved(R_CELL_OUT * s + (0, 0, SCREW_Z))
        parts.append((f"boss{i}", boss, "abs"))
        _JOINT_YAW[f"s{i}"] = b + math.pi  # the joint's X points in, at the dew shield: positive is clockwise seen from the head (tightening)
        _JOINT_YAW[f"t{i}"] = b + math.pi
        links.append(m.hinge(f"s{i}", "X", tuple(R_CELL_OUT * s + (0, 0, SCREW_Z)), float("-inf"), float("inf"),
                             [("head", head_mesh, "nylon"), ("knurl", knurl, "nylon")],
                             [m.col_hull(f"s{i}_head_c", head_mesh)], mass=0.002, friction=0.03, damping=0.002, armature=1e-4,
                             inertia=(2.5e-8, 4e-8, 4e-8)))
        tip_mesh = _along(m.cylinder(TIP_R, 0.004, (0, 0, 0.002), segments=12), b).moved((R_CELL_IN - TIP_PROUD) * s + (0, 0, SCREW_Z))
        links.append(m.slider(f"t{i}", "X", -0.0005, TIP_MAX, [("tip", tip_mesh, "white")], [m.col_hull(f"t{i}_c", tip_mesh)],
                              at=tuple((R_CELL_IN - TIP_PROUD) * s + (0, 0, SCREW_Z)), mass=0.0005, spring=TIP_K, damping=20.0,
                              effort=TIP_EFFORT, friction=0.0, armature=1e-4, inertia=(1e-8, 1e-8, 1e-8)))
    z0 = TOP + CASE_TOP + BOSS_H
    return m.mechanism("filter", (CELL0[0], CELL0[1], z0), links, LOOKS, frame_parts=parts, frame_colliders=cols, free_mass=0.023,
                       frame_inertia=(2.2e-5, 2.2e-5, 4.0e-5), friction=(0.9, 0.8), solver_iters=48)


def _cap():
    parts = [("cap", m.lathe([(R_CAP_IN, -CAP_DEPTH), (R_CAP_OUT, -CAP_DEPTH), (R_CAP_OUT, CAP_TOP - 0.0008), (R_CAP_OUT - 0.0008, CAP_TOP),
                               (0.0, CAP_TOP), (0.0, 0.0), (R_CAP_IN, 0.0), (R_CAP_IN, -CAP_DEPTH)], segments=40), "abs")]
    cols = [m.col_cylinder("top_c", R_CAP_OUT, CAP_TOP, (0, 0, CAP_TOP / 2))]
    n = 12
    wall = R_CAP_OUT - R_CAP_IN
    width = 2 * math.pi * R_CAP_OUT / n + 0.001
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        cols.append(m.col_hull(f"w{k}_c", m.box((wall, width, CAP_DEPTH), ((R_CAP_IN + R_CAP_OUT) / 2, 0, -CAP_DEPTH / 2)).turned(math.degrees(a), "z")))
    return m.free("cap", (CAP0[0], CAP0[1], TOP + CASE_TOP + CAPBOSS_H), parts, cols, LOOKS, mass=0.006, friction=(0.9, 0.8), com=(0.0, 0.0, -0.004))


def _case():
    c = CASE_C
    lo, hi = c - CASE_SIZE[:2] / 2, c + CASE_SIZE[:2] / 2
    parts = [
        ("shell", m.box((CASE_SIZE[0], CASE_SIZE[1], CASE_TOP - 0.012), (c[0], c[1], (CASE_TOP - 0.012) / 2)), "case"),
        ("foam", m.box((CASE_SIZE[0] - 0.008, CASE_SIZE[1] - 0.008, 0.012), (c[0], c[1], CASE_TOP - 0.006)), "foam"),
        ("rim", m.box((CASE_SIZE[0] + 0.004, CASE_SIZE[1] + 0.004, 0.006), (c[0], c[1], CASE_TOP - 0.015)), "silver"),
        ("boss", m.cylinder(BOSS_R, BOSS_H, (CELL0[0], CELL0[1], CASE_TOP + BOSS_H / 2), segments=48), "foam"),
        ("capboss", m.cylinder(CAPBOSS_R, CAPBOSS_H, (CAP0[0], CAP0[1], CASE_TOP + CAPBOSS_H / 2)), "foam"),
    ]
    cols = [
        m.col_box("shell_c", (CASE_SIZE[0], CASE_SIZE[1], CASE_TOP), (c[0], c[1], CASE_TOP / 2)),
        m.col_cylinder("boss_c", BOSS_R, BOSS_H, (CELL0[0], CELL0[1], CASE_TOP + BOSS_H / 2)),
        m.col_cylinder("capboss_c", CAPBOSS_R, CAPBOSS_H, (CAP0[0], CAP0[1], CASE_TOP + CAPBOSS_H / 2)),
    ]
    del lo, hi
    return m.fixed("case", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(1.0, 0.9))


SCOPE, FILTER, CAP, CASE = _scope(), _cell(), _cap(), _case()

# -- distractors (SimReady): the observing log and a coffee mug, on the left of the table
NOTEBOOK = SimReady("observing_log", pos=(0.14, 0.25, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                    usd="sm_misc_notebook_a01_01.usd", yaw=0.25)
MUG = SimReady("mug", pos=(0.52, 0.26, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=0.6)

OBJECTS = [SCOPE, FILTER, CAP, CASE, NOTEBOOK, MUG]

# ---------------------------------------------------------------------------------------------------------------------------------
# Where things are, from the pan angle (world)


def tube_axis(pan: float) -> np.ndarray:
    a = PSI0 + pan
    return np.array([P[0] + ARM * math.cos(a), P[1] + ARM * math.sin(a)])


def finder_axis(pan: float) -> np.ndarray:
    a = PSI0 + pan
    return tube_axis(pan) + FINDER_R * np.array([math.cos(a + FINDER_REL), math.sin(a + FINDER_REL)])


RIM_Z = TOP + TUBE_Z1
FCELL_Z = TOP + FINDER_Z1


def _out_of_cylinder(local: np.ndarray, r: float, z0: float, z1: float) -> np.ndarray:
    """Distance of each point (part frame) from a solid cylinder about z, of radius r, from z0 to z1."""
    rho = np.hypot(local[:, 0], local[:, 1])
    dr = np.maximum(rho - r, 0.0)
    dz = np.maximum(np.maximum(z0 - local[:, 2], local[:, 2] - z1), 0.0)
    return np.hypot(dr, dz)


def _contact_depth(f, pan: float, bearing: float):
    """How far (m) screw `bearing`'s tip must travel in from its start to touch the dew shield, given where the cell now sits; None when
    the tip's line misses the dew shield (the filter is off the tube, or the tip is above the rim or below the shield)."""
    s = np.array([math.cos(bearing), math.sin(bearing), 0.0])
    start = (R_CELL_IN - TIP_PROUD) * s + np.array([0.0, 0.0, SCREW_Z])
    w0 = f.to_world(start[None, :])[0]
    u = f.rotation() @ (-s)  # the tip's travel, world
    t = tube_axis(pan)
    if not (TOP + DEW_Z0 + TIP_R <= w0[2] <= RIM_Z - TIP_R):
        return None
    w = np.array([w0[0] - t[0], w0[1] - t[1]])
    uh = u[:2]
    a = float(uh @ uh)
    if a < 0.8:  # the screw's axis is far from level: the cell is tipped
        return None
    bq = float(w @ uh)
    c = float(w @ w) - R_DEW ** 2
    disc = bq * bq - a * c
    if disc < 0.0:
        return None
    d = (-bq - math.sqrt(disc)) / a  # the first meeting with the shield's outside along the tip's travel
    if d < -0.002 or d > TIP_MAX + 0.002:
        return None
    return d


class Trial:
    """Every control step: the screws drive their tips; the state is read; harms are latched until a reset (and only then).

    A screw's tip goes where the screw's turns drive it (1.27 mm a turn), but no further than the dew shield: its contact is computed from the
    cell's pose on the tube, so a tip never shoves the cell or sinks into the shield. Turned 0.4 mm (about a third of a turn) past first
    contact, the screw is snug: its hinge is locked, as a finger-tight thumbscrew stops turning, and its tip is held 0.2 mm into its contact,
    so the three snug tips clamp the cell. A snug screw that loses its contact (the cell pulled off) is freed again."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.film_touched = False
        self.dropped = False
        self.locked: dict[int, float] = {}  # screw -> its tip's frozen target
        self.still = 0
        self.full_now = False
        self.score = 0.0
        self.info: dict = {}
        self.steps = 0
        self.raw = [None, None, None]  # the screws' joint readings: PhysX wraps an unlimited joint, so the turns are counted here
        self.theta = [0.0, 0.0, 0.0]

    def _turns(self, f):
        for k in range(3):
            raw = f.joint(f"s{k + 1}")
            if self.raw[k] is None:
                self.theta[k] = raw
            else:
                d = raw - self.raw[k]
                d -= 2 * math.pi * round(d / (2 * math.pi))
                self.theta[k] += d
            self.raw[k] = raw

    def _screws(self, env, f, pan):
        snug = []
        contact = []
        for i, b in zip((1, 2, 3), SCREW_BEARINGS):
            driven = PITCH * self.theta[i - 1] / (2 * math.pi)
            dc = _contact_depth(f, pan, b)
            contact.append(None if dc is None else round(dc * 1000, 2))
            if i in self.locked:
                frozen = self.locked[i]
                if dc is not None and dc <= frozen + 0.0005:  # still bearing on the shield
                    snug.append(True)
                    m.set_target(env, "filter", f"t{i}", frozen)
                    continue
                m.unlock(env, "filter", f"s{i}")  # the cell was pulled off: the screw turns again
                del self.locked[i]
            if dc is not None and driven - dc >= SNUG:
                frozen = float(np.clip(dc + PRELOAD, -0.0005, TIP_MAX))
                m.lock(env, "filter", f"s{i}", stiffness=5.0, effort=0.6)
                self.locked[i] = frozen
                m.set_target(env, "filter", f"t{i}", frozen)
                snug.append(True)
                continue
            target = driven if dc is None else min(driven, dc - 0.0001)
            m.set_target(env, "filter", f"t{i}", float(np.clip(target, -0.0005, TIP_MAX)))
            snug.append(False)
        return snug, contact

    def step(self, env):
        self.steps += 1
        f = m.state(env, "filter")
        sc = m.state(env, "scope")
        cap = m.state(env, "cap")
        self._turns(f)
        pan = sc.joint("pan")
        snug, contact = self._screws(env, f, pan)
        # the filter seated on the dew shield: its plate on the rim, square, on the tube's axis
        R = f.rotation()
        axis = R[:, 2]
        tilt = math.degrees(math.acos(float(np.clip(axis[2], -1.0, 1.0))))
        t = tube_axis(pan)
        off = float(np.hypot(f.pos[0] - t[0], f.pos[1] - t[1]))
        dz = float(f.pos[2] - RIM_Z)
        seated = tilt <= 3.0 and off <= 0.002 and -0.001 <= dz <= 0.0015
        # the cap on the finder: closed, square, its inner top face within 2 mm of the objective cell's front
        Rc = cap.rotation()
        ctilt = math.degrees(math.acos(float(np.clip(Rc[2, 2], -1.0, 1.0))))
        fa = finder_axis(pan)
        coff = float(np.hypot(cap.pos[0] - fa[0], cap.pos[1] - fa[1]))
        cdz = float(cap.pos[2] - FCELL_Z)
        capped = ctilt <= 5.0 and coff <= 0.003 and -0.001 <= cdz <= 0.002
        # hands
        tips = m.fingertips(env)
        lf = f.to_local(tips)
        d_cell = _out_of_cylinder(lf, R_CELL_OUT + HEAD_L, -SKIRT, PLATE)
        lc = cap.to_local(tips)
        d_cap = _out_of_cylinder(lc, R_CAP_OUT, -CAP_DEPTH, CAP_TOP)
        released = bool(d_cell.min() >= 0.03 and d_cap.min() >= 0.03)
        # harms: a fingertip on the film (within the aperture, at most 6 mm out from the front face); the filter dropped onto the table
        rho = np.hypot(lf[:, 0], lf[:, 1])
        if bool(np.any((rho <= R_FILM) & (lf[:, 2] >= -0.001) & (lf[:, 2] <= PLATE + 0.006))):
            self.film_touched = True
        if f.pos[2] < TOP + 0.05:
            self.dropped = True
        # stillness over the last second
        v = max(float(np.linalg.norm(f.lin_vel)), float(np.linalg.norm(cap.lin_vel)))
        self.still = self.still + 1 if v < 0.01 else 0
        n_snug = sum(snug)
        harm = self.film_touched or self.dropped
        secured = seated and n_snug == 3
        self.full_now = bool(secured and capped and released and self.still >= 20 and not harm)
        if harm:
            s = 0.0
        elif self.full_now:
            s = 1.0
        elif secured and released:
            s = 0.5
        elif seated and released and n_snug >= 1:
            s = 0.25
        else:
            s = 0.0
        self.score = float(s)
        self.info = dict(step=self.steps, pan=round(math.degrees(pan), 1), tilt=round(tilt, 2), off_mm=round(off * 1000, 2), dz_mm=round(dz * 1000, 2),
                         snug=snug, theta=[round(math.degrees(x), 1) for x in self.theta], contact_mm=contact,
                         tip_mm=[round(f.joint(f"t{i}") * 1000, 2) for i in (1, 2, 3)], seated=seated, capped=capped, coff_mm=round(coff * 1000, 2),
                         cdz_mm=round(cdz * 1000, 2), ctilt=round(ctilt, 1), released=released, d_cell=round(float(d_cell.min()), 3),
                         d_cap=round(float(d_cap.min()), 3), film=self.film_touched, dropped=self.dropped, score=self.score,
                         fpos=[round(float(x), 4) for x in f.pos], cpos=[round(float(x), 4) for x in cap.pos])
        if os.environ.get("TASK_TRACE_TIPS"):
            self.info["tips"] = [[round(float(v) * 1000, 1) for v in q] for q in lf]
        trace = os.environ.get("TASK_TRACE")
        if trace and self.steps % 10 == 0:
            import json
            with open(trace, "a") as fh:
                fh.write(json.dumps(self.info) + "\n")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full_now)


def score(objects) -> float:
    return float(TRIAL.score)


# ---------------------------------------------------------------------------------------------------------------------------------
# The scripted demo (open loop, from the published starting layout): see demo_plan.py

def demo():
    import types
    from demo_plan import plan
    return plan(os.environ.get("TASK_DEMO", "full"), types.SimpleNamespace(**globals()))
