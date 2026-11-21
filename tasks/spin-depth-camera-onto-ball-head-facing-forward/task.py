"""Mount an Intel RealSense D435 depth camera on a desk-clamp arm's ball head so that it faces the robot's workspace.

On the station table (standing in for the desk of a robot data-collection rig): an articulating camera arm, clamped
to the desk's far edge, holds a ball head 16 cm above the desk, between the two arms. The head's ball is locked by its
knob, but its panning base is loose: everything above the base swivels freely about the vertical. On the head's
35 mm platform stands a 1/4"-20 stud with a knurled lock wheel run down to the platform. A strip of blue tape on the
desk runs from under the head towards the arms: the forward line the camera's lenses must face along. The D435
(90 x 25 x 25 mm, 75 g) stands on the desk to the right, lenses facing away from the arms.

Model (declared in code; every collider is real contact, and nothing is triggered by proximity alone):
- Head ("head", a fixed articulation; origin on the stud's axis at the platform's top face). Frame: the arm, its
  clamp and the head's panning base. Link "pan" (hinge about z, unlimited): the head above its base (socket body with
  its locked ball, neck, platform and the stud), on a loose pan base (damping PAN_DAMP N m s/rad).
- Lock wheel (links "lw_rise", a slider along the stud, and "lw", its turn, children of "pan"): a 40 mm knurled
  wheel on the stud's thread. Each control step its height follows its turn: 1.27 mm per turn (1/4"-20), rising
  when turned anticlockwise seen from above, from the platform (0) to 3 mm up; a spring of 20,000 N/m, at most 8 N,
  holds it there, so a wheel run up against the camera presses it with at most 8 N. Run up hard against the camera
  (touching it, its spring loaded by at least 0.2 mm) it is held from turning, as a jam nut is, until the camera is
  backed off it by more than 0.6 mm.
- Camera ("camera", free, 75 g): an aluminium bar 90 x 25 x 25 mm with its 1/4"-20 hole on the axis of its underside,
  8 mm deep, modelled at the thread's minor diameter (a 5.4 mm square bore). The stud is the thread's major diameter
  (6.35 mm), its last millimetre turned down to 4.4 mm (the lead): set over the stud, the camera drops onto the lead
  and rests on the first thread, 9 mm above the platform, until the thread catches.
- The thread catches when, with the camera resting on the first thread (its base no more than 9.5 mm above the
  platform, its hole's axis within 0.6 mm of the stud's, tilted no more than 5 degrees), it is turned clockwise (seen
  from above) past the angle at which the two thread starts meet (CATCH_DEG, relative to the head). A PhysX D6 joint,
  authored switched off, is then switched on: it keeps the camera on the stud's axis and square to it, adds thread
  drag (THREAD_DAMP N m s/rad between camera and head) and draws the camera down 1.27 mm per clockwise turn. Turned
  back anticlockwise past the catch, the camera comes off the thread (the joint is switched off).
- The thread drag is ten times the pan base's: a camera turned while nobody holds the head turns the head with it
  and barely goes on.

Scoring (the tracker reads the end state every control step; `score()` returns it). Gap = the camera base's height
above the lock wheel's top when the wheel is down on the platform (the camera base above the platform, less the
wheel's 4.0 mm). Aim = the angle between the camera's lens axis and the taped forward line, seen from above.
- 0.0 if the camera has ever dropped (fallen faster than 0.8 m/s), cleared only at a reset.
- 1.0: thread caught, gap at most 1.0 mm, aim within 5 degrees, the head turned no more than 5 degrees on its base
  from where it started, the lock wheel against the camera (no more than 0.3 mm between them), every fingertip point
  at least 3 cm from the camera and the head above its base, and all of it still, held for 1 s.
- 0.75: caught, gap at most 1.0 mm and aim within 5 degrees, without the rest of 1.0.
- 0.5: caught and gap at most 1.0 mm, aim more than 5 degrees off.
- 0.25: caught, gap more than 1.0 mm (on, but loose).
- 0.0 otherwise (not on the thread).
The partial levels need no hands clear (they are progress); full credit does, and does not outlast the state.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
INSTRUCTION = "Mount the camera so it faces the robot's workspace."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]
DEBUG = os.environ.get("SPIN_DEBUG", "1") == "1"

# -- layout ---------------------------------------------------------------------------------------------------------
HX, HY = 0.40, -0.16  # the stud's axis
ZP = TOP + 0.100  # the head's platform top face
DESK = TOP - ZP  # the desk top in the head's frame
FORWARD_DEG = 180.0  # the taped forward line points from the head towards the arms (-x)

# -- the thread and the stud (head frame: origin on the stud's axis at the platform's top face) --------------------
PITCH = 0.0254 / 20  # 1/4"-20 UNC: 1.27 mm per turn
R_MAJOR = 0.00635 / 2
R_LEAD = 0.0022  # the stud's last millimetre, turned down: the lead the hole drops over
WHEEL_R, WHEEL_T = 0.020, 0.004  # the knurled lock wheel: 40 mm across, 4 mm thick
STUD_TOP = 0.010  # the stud stands 10 mm above the platform: 6 mm proud of the wheel
Z_SHOULDER = STUD_TOP - 0.001  # where the camera's base rests on the first thread, unthreaded
Z_SEAT = WHEEL_T  # the camera's base on the wheel, the wheel down on the platform
T_SEAT = (Z_SHOULDER - Z_SEAT) / PITCH * 360.0  # degrees of clockwise turn from the catch to seated: 1417.3
AIM_REL = 180.0  # the camera's turn relative to the head (deg) at which it faces along the forward line
SEAT_REL = AIM_REL - 180.0  # it seats half a turn past facing forward: a camera run down hard faces backwards
CATCH_DEG = (SEAT_REL + T_SEAT) % 360.0  # the relative turn at which the thread starts meet: 337.3
WHEEL_RISE_MAX = 0.003
GAP_MAX, WHEEL_GAP_MAX, AIM_TOL, PAN_TOL, HANDS = 0.0010, 0.0003, 5.0, 5.0, 0.03
PAN_DAMP = 0.002  # N m s/rad: the loose pan base
THREAD_DAMP = 0.02  # N m s/rad: the thread's running drag, ten times the pan base's

# -- the camera (its frame: origin at the centre of its underside, on its hole's axis; lenses facing +x) -------------
CAM_L, CAM_D, CAM_H = 0.090, 0.025, 0.025  # long (y), deep (x, lens axis), tall: Intel datasheet table 3-52
CAM_MASS = 0.075  # datasheet: 75 g nominal
HOLE_HALF, HOLE_DEPTH = 0.0027, 0.008  # the bore at the thread's minor diameter (5.4 mm), 8 mm deep
CAM_START = (0.23, -0.18)  # on the desk, lenses facing away from the arms (yaw 0)
CAM_START_YAW = 0.0
PLACE_REL = 0.0  # set on the stud as it stood on the desk: lenses away from the arms

LOOKS = {
    "alu": m.Look(color=m.rgb(0.42, 0.43, 0.45), metallic=0.8, roughness=0.35),
    "alu_dark": m.Look(color=m.rgb(0.20, 0.21, 0.22), metallic=0.7, roughness=0.4),
    "glass": m.Look(color=m.rgb(0.02, 0.02, 0.03), roughness=0.08),
    "lens": m.Look(color=m.rgb(0.05, 0.06, 0.09), metallic=0.3, roughness=0.15),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "rubber": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.9),
    "steel": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.3),
    "knurl": m.Look(color=m.rgb(0.12, 0.12, 0.13), metallic=0.6, roughness=0.5),
    "white": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.5),
    "red": m.Look(color=m.rgb(0.75, 0.10, 0.08), roughness=0.5),
    "tape": m.Look(color=m.rgb(0.10, 0.35, 0.80), roughness=0.7),
    "clamp": m.Look(color=m.rgb(0.10, 0.10, 0.11), metallic=0.5, roughness=0.5),
}


# -- head ---------------------------------------------------------------------------------------------------------------
def _tube(a, b, r):
    return m.tube([np.asarray(a, float), (np.asarray(a, float) + np.asarray(b, float)) / 2, np.asarray(b, float)], r, sides=16)


def _head():
    # the frame: the head's panning base, the arm's knuckle under it, two arm segments and the clamp on the desk's far
    # edge (the table's far edge is 0.53 m beyond the stud, the desk top 0.10 m below the platform)
    knuckle = (0.0, 0.0, -0.075)
    elbow = (0.25, 0.0, -0.020)
    clamp_top = (0.515, 0.0, DESK + 0.020)
    fparts = [
        ("pan_base", m.cylinder(0.020, 0.010, (0.0, 0.0, -0.055)), "alu_dark"),
        ("spigot", m.cylinder(0.006, 0.012, (0.0, 0.0, -0.066)), "steel"),
        ("knuckle", m.sphere(0.013, knuckle), "clamp"),
        ("seg_a", _tube(knuckle, elbow, 0.010), "clamp"),
        ("elbow", m.sphere(0.014, elbow), "clamp"),
        ("seg_b", _tube(elbow, clamp_top, 0.010), "clamp"),
        ("jaw_top", m.box((0.050, 0.045, 0.020), (0.515, 0.0, DESK + 0.010)), "clamp"),
        ("spine", m.box((0.015, 0.045, 0.110), (0.5475, 0.0, DESK - 0.025)), "clamp"),
        ("jaw_bot", m.box((0.050, 0.045, 0.015), (0.515, 0.0, DESK - 0.0675)), "clamp"),
        ("screw", m.cylinder(0.005, 0.040, (0.515, 0.0, DESK - 0.095)), "steel"),
    ]
    fcols = [
        m.col_cylinder("pan_base_c", 0.020, 0.010, (0.0, 0.0, -0.055)),
        m.col_hull("knuckle_c", m.sphere(0.013, knuckle)),
        m.col_hull("seg_a_c", _tube(knuckle, elbow, 0.010)),
        m.col_hull("seg_b_c", _tube(elbow, clamp_top, 0.010)),
        m.col_box("jaw_top_c", (0.050, 0.045, 0.020), (0.515, 0.0, DESK + 0.010)),
    ]
    # the pan link: socket body (its ball locked by the knob), ball, neck, platform with its rubber pad, the stud
    pparts = [
        ("socket", m.cylinder(0.016, 0.033, (0.0, 0.0, -0.0335)), "alu"),
        ("knob", m.cylinder(0.008, 0.012, (0.0, -0.022, -0.035), axis="y"), "black"),
        ("ball", m.sphere(0.011, (0.0, 0.0, -0.019)), "steel"),
        ("neck", m.cylinder(0.006, 0.008, (0.0, 0.0, -0.009)), "alu"),
        ("platform", m.cylinder(0.0175, 0.004, (0.0, 0.0, -0.003)), "alu"),
        ("pad", m.cylinder(0.0170, 0.001, (0.0, 0.0, -0.0005)), "rubber"),
        ("stud", m.cylinder(R_MAJOR, Z_SHOULDER, (0.0, 0.0, Z_SHOULDER / 2)), "steel"),
        ("lead", m.cylinder(R_LEAD, STUD_TOP - Z_SHOULDER, (0.0, 0.0, (STUD_TOP + Z_SHOULDER) / 2)), "steel"),
        ("pan_mark", m.box((0.004, 0.0015, 0.010), (-0.0160, 0.0, -0.040)), "white"),
    ]
    pcols = [
        m.col_cylinder("socket_c", 0.016, 0.033, (0.0, 0.0, -0.0335)),
        m.col_cylinder("platform_c", 0.0175, 0.005, (0.0, 0.0, -0.0025)),
        m.col_cylinder("lead_c", R_LEAD, STUD_TOP - Z_SHOULDER, (0.0, 0.0, (STUD_TOP + Z_SHOULDER) / 2)),
    ]
    pan = m.hinge("pan", "Z", (0.0, 0.0, 0.0), float("-inf"), float("inf"), pparts, pcols, mass=0.12, damping=PAN_DAMP,
                  friction=0.0, inertia=(4e-5, 4e-5, 2e-5))
    # the stud's threaded body as a collider of its own (a slider held at zero by a stiff spring): the first thread the
    # camera's bore rests on until the thread catches. Once caught, the camera is held by the thread joint instead and
    # this collider is drawn down into the platform, out of the bore (what is seen of the stud stays with "pan")
    stud_body = m.slider("stud_body", "Z", -0.011, 0.0, (), (m.col_cylinder("stud_c", R_MAJOR, Z_SHOULDER, (0.0, 0.0, Z_SHOULDER / 2)),),
                         parent="pan", mass=0.01, spring=50000.0, damping=50.0, friction=0.0, effort=200.0, inertia=(1e-7, 1e-7, 1e-7))
    rise = m.slider("lw_rise", "Z", 0.0, WHEEL_RISE_MAX, (), (), parent="pan", mass=0.005, spring=20000.0, damping=20.0,
                    friction=0.0, effort=8.0, inertia=(1e-7, 1e-7, 1e-7))
    knurl = m.join(*[m.box((0.0016, 0.0018, WHEEL_T), (WHEEL_R * math.cos(a), WHEEL_R * math.sin(a), WHEEL_T / 2)).turned(math.degrees(a), "z")
                     for a in np.linspace(0, 2 * math.pi, 40, endpoint=False)])
    wparts = [
        ("wheel", m.cylinder(WHEEL_R - 0.0004, WHEEL_T, (0.0, 0.0, WHEEL_T / 2), segments=40), "knurl"),
        ("knurl", knurl, "knurl"),
        ("wheel_mark", m.box((0.006, 0.0015, 0.0004), (0.011, 0.0, WHEEL_T + 0.0002)), "white"),
    ]
    wcols = [m.col_cylinder("wheel_c", WHEEL_R, WHEEL_T, (0.0, 0.0, WHEEL_T / 2))]
    wheel = m.hinge("lw", "Z", (0.0, 0.0, 0.0), float("-inf"), float("inf"), wparts, wcols, parent="lw_rise", mass=0.010,
                    damping=0.001, friction=0.0, inertia=(7e-7, 7e-7, 1.3e-6))
    return m.mechanism("head", (HX, HY, ZP), [pan, stud_body, rise, wheel], LOOKS, frame_parts=fparts, frame_colliders=fcols,
                       friction=(1.0, 0.9), solver_iters=64)


HEAD = _head()


# -- camera -------------------------------------------------------------------------------------------------------------
def _camera():
    hl, hd = CAM_L / 2, CAM_D / 2
    body = m.join(m.box((CAM_D, CAM_L - CAM_D, CAM_H), (0.0, 0.0, CAM_H / 2)),
                  m.cylinder(hd, CAM_H, (0.0, hl - hd, CAM_H / 2), axis="z"),
                  m.cylinder(hd, CAM_H, (0.0, -(hl - hd), CAM_H / 2), axis="z"))
    face = m.box((0.0006, CAM_L - 0.012, CAM_H - 0.006), (hd + 0.0003, 0.0, CAM_H / 2))
    parts = [("body", body, "alu"), ("face", face, "glass")]
    for i, (y, r) in enumerate(((0.0175, 0.0042), (-0.0325, 0.0042), (-0.0040, 0.0036), (0.0330, 0.0038))):  # left/right imagers, projector, RGB
        parts.append((f"lens{i}", m.cylinder(r, 0.0008, (hd + 0.0007, y, CAM_H / 2), axis="x"), "lens"))
    parts.append(("hole", m.cylinder(R_MAJOR, 0.0004, (0.0, 0.0, 0.0001)), "black"))
    h, d = HOLE_HALF, HOLE_DEPTH
    cols = [
        m.col_box("upper_c", (CAM_D, CAM_L, CAM_H - d), (0.0, 0.0, d + (CAM_H - d) / 2)),
        m.col_box("left_c", (CAM_D, hl - h, d), (0.0, (hl + h) / 2, d / 2)),
        m.col_box("right_c", (CAM_D, hl - h, d), (0.0, -(hl + h) / 2, d / 2)),
        m.col_box("front_c", (hd - h, 2 * h, d), ((hd + h) / 2, 0.0, d / 2)),
        m.col_box("back_c", (hd - h, 2 * h, d), (-(hd + h) / 2, 0.0, d / 2)),
    ]
    return parts, cols


_cp, _cc = _camera()
CAMERA = m.free("camera", (CAM_START[0], CAM_START[1], TOP + 0.0003), _cp, _cc, LOOKS, mass=CAM_MASS, friction=(0.9, 0.8),
                torsional_patch=0.01, solver_iters=64, yaw_deg=CAM_START_YAW, com=(0.0, 0.0, CAM_H / 2))

# -- the taped forward line on the desk: 12 mm blue tape from under the head towards the arms, with an arrowhead -------
_tape = [("line", m.box((0.150, 0.012, 0.0005), (-0.090, 0.0, 0.00025)), "tape"),
         ("arrow", m.prism([(-0.180, 0.0), (-0.160, -0.016), (-0.160, 0.016)], 0.0, 0.0005), "tape")]
TAPE = m.fixed("tape", (HX, HY, TOP + 0.0002), _tape, (), LOOKS)

# -- distractors (SimReady): a mug, a roll of tape, a screwdriver ------------------------------------------------------
MUG = SimReady("mug", pos=(0.42, 0.24, TOP), size=(0.114, 0.090, 0.080), mass=0.30, yaw=2.6,
               package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda")
TAPE_ROLL = SimReady("tape_roll", pos=(0.56, 0.05, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
                     package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd")
SCREWDRIVER = SimReady("screwdriver", pos=(0.56, -0.33, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=0.3, convex=True,
                       package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd")

OBJECTS = [HEAD, CAMERA, TAPE, MUG, TAPE_ROLL, SCREWDRIVER]


# -- the thread joint: a PhysX D6 joint between the head's pan link and the camera, authored switched off --------------
THREAD_PATH: list[str] = []


def _x_to_z():
    """(w, x, y, z) of the rotation taking +x onto +z (the joint's x axis along the stud)."""
    s = math.sqrt(0.5)
    return (s, 0.0, -s, 0.0)


def _author_thread(prim) -> None:
    from pxr import Gf, UsdPhysics

    stage = prim.GetStage()
    cam_path = str(prim.GetPath())
    env = cam_path.rsplit("/", 1)[0]
    path = f"{cam_path}/thread_joint"
    joint = UsdPhysics.Joint.Define(stage, path)  # a plain Joint is a D6
    joint.CreateBody0Rel().SetTargets([f"{env}/head/pan"])
    joint.CreateBody1Rel().SetTargets([cam_path])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(Gf.Quatf(*_x_to_z()))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(*_x_to_z()))
    joint.CreateJointEnabledAttr(False)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    p = joint.GetPrim()
    for dof in ("transY", "transZ", "rotY", "rotZ"):  # low > high: locked
        lim = UsdPhysics.LimitAPI.Apply(p, dof)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    lim = UsdPhysics.LimitAPI.Apply(p, "transX")
    lim.CreateLowAttr(float(Z_SEAT - 0.0006))
    lim.CreateHighAttr(float(Z_SHOULDER + 0.0006))
    turn = UsdPhysics.DriveAPI.Apply(p, "rotX")  # angular drives take degrees
    turn.CreateTypeAttr("force")
    turn.CreateStiffnessAttr(0.0)
    turn.CreateDampingAttr(float(THREAD_DAMP * math.pi / 180.0))
    turn.CreateTargetVelocityAttr(0.0)
    turn.CreateMaxForceAttr(1.0e6)
    slide = UsdPhysics.DriveAPI.Apply(p, "transX")
    slide.CreateTypeAttr("force")
    slide.CreateStiffnessAttr(100000.0)
    slide.CreateDampingAttr(100.0)
    slide.CreateTargetPositionAttr(float(Z_SHOULDER))
    slide.CreateMaxForceAttr(200.0)
    THREAD_PATH[:] = [path]


_orig_arena_object = m._arena_object


def _arena_object_with_thread(spec):
    obj = _orig_arena_object(spec)
    if spec.name == "camera":
        cfg = obj.object_cfg.spawn
        spawn = cfg.func

        def spawn_and_thread(prim_path, scfg, *a, **k):
            prim = spawn(prim_path, scfg, *a, **k)
            _author_thread(prim)
            return prim

        cfg.func = spawn_and_thread
    return obj


m._arena_object = _arena_object_with_thread
_ATTRS: dict = {}


def _thread_attrs():
    if not _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(THREAD_PATH[0])
        _ATTRS["on"] = UsdPhysics.Joint(prim).GetJointEnabledAttr()
        _ATTRS["z"] = UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr()
    return _ATTRS


def set_thread(on: bool, z: float | None = None) -> None:
    a = _thread_attrs()
    if z is not None:
        a["z"].Set(float(z))
    if bool(a["on"].Get()) != bool(on):
        a["on"].Set(bool(on))


# -- the scorer ---------------------------------------------------------------------------------------------------------
def _wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _yaw(rot) -> float:
    return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))


def measure(head, cam, tips=None) -> dict:
    """Everything the score reads. The head's frame is fixed; the pan link turns about its z by the pan joint."""
    pan = head.deg("pan")
    rp = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(pan))
    rc = cam.rotation()
    d = rp.T @ (np.asarray(cam.pos) - np.asarray(head.pos))
    out = {
        "pan": pan,
        "x": float(d[0]), "y": float(d[1]), "z": float(d[2]),  # the camera's origin in the pan link's frame
        "off": float(math.hypot(d[0], d[1])),
        "tilt": float(math.degrees(math.acos(np.clip(rc[2, 2], -1.0, 1.0)))),
        "rel": _wrap(_yaw(rc) - pan),  # the camera's turn relative to the head
        "aim": _wrap(_yaw(rc) - FORWARD_DEG),  # its lens axis against the taped line, seen from above
        "rise": float(head.joint("lw_rise")),
        "wheel": float(head.deg("lw")),
        "v": float(np.linalg.norm(cam.lin_vel)), "vz": float(cam.lin_vel[2]), "w": float(np.linalg.norm(cam.ang_vel)),
    }
    out["gap"] = out["z"] - Z_SEAT
    out["wheel_gap"] = out["z"] - (WHEEL_T + out["rise"])
    if tips is not None:
        q = (np.asarray(tips) - np.asarray(cam.pos)) @ rc  # fingertips in the camera's frame
        ex = np.maximum(np.abs(q) - np.array([CAM_D / 2, CAM_L / 2, CAM_H / 2]), 0.0)
        ex[:, 2] = np.maximum(np.abs(q[:, 2] - CAM_H / 2) - CAM_H / 2, 0.0)
        cam_gap = float(np.min(np.linalg.norm(ex, axis=1)))
        h = (np.asarray(tips) - np.asarray(head.pos))  # the head above its base: a cylinder r 20 mm from z -50 to +10 mm
        dr = np.maximum(np.hypot(h[:, 0], h[:, 1]) - 0.020, 0.0)
        dz = np.maximum(np.maximum(h[:, 2] - STUD_TOP, -0.050 - h[:, 2]), 0.0)
        out["hand_gap"] = min(cam_gap, float(np.min(np.hypot(dr, dz))))
    return out


def level(c: dict, caught: bool) -> float:
    """The progress level of a state (no hands, hold or wheel rule)."""
    if not caught:
        return 0.0
    if c["gap"] > GAP_MAX:
        return 0.25
    if abs(c["aim"]) > AIM_TOL:
        return 0.5
    return 0.75


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm (and the thread's state) is cleared
        self.dropped = False
        self.caught = False
        self.stud_down = False
        self.jammed = False
        self.rel_unwrapped = None  # the camera's turn relative to the head, followed continuously (deg)
        self.rel_catch = 0.0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.steps = 0
        self.prev = None
        if env is not None:
            try:
                set_thread(False, Z_SHOULDER)
            except Exception as e:  # noqa: BLE001
                print(f"[spin] reset: {e}", flush=True)

    def _thread(self, env, c):
        rel = c["rel"]
        if self.rel_unwrapped is None:
            self.rel_unwrapped = rel
        else:
            self.rel_unwrapped += _wrap(rel - _wrap(self.rel_unwrapped))
        r = self.rel_unwrapped
        if not self.caught:
            on_lead = c["z"] <= Z_SHOULDER + 0.0005 and c["off"] <= 0.0006 and c["tilt"] <= 5.0
            prev = self.prev_rel if self.prev_rel is not None else r
            # the thread starts meet at CATCH_DEG (mod 360): caught if the camera turned clockwise across it this step
            k = math.floor((prev - CATCH_DEG) / 360.0)
            crossed = r < prev and (r - CATCH_DEG) <= 360.0 * k <= (prev - CATCH_DEG)
            if on_lead and crossed:
                self.caught = True
                self.rel_catch = CATCH_DEG + 360.0 * k
                set_thread(True, Z_SHOULDER)
                m.set_target(env, "head", "stud_body", -0.0105)
                print(f"[spin] thread caught at step {self.steps}, rel {r:.1f}", flush=True)
        else:
            turned = self.rel_catch - r  # clockwise degrees since the catch
            if turned < -15.0:  # backed off past the thread's start: off the thread
                self.caught = False
                set_thread(False, Z_SHOULDER)
                self.stud_down = True
                print(f"[spin] off the thread at step {self.steps}", flush=True)
            else:
                set_thread(True, max(Z_SEAT - 0.0005, Z_SHOULDER - PITCH * turned / 360.0))
        if not self.caught and self.stud_down and (c["off"] > 0.006 or c["z"] > STUD_TOP + 0.002):
            m.set_target(env, "head", "stud_body", 0.0)  # the camera is clear of the stud: its first thread is back
            self.stud_down = False
        self.prev_rel = r

    def step(self, env):
        self.steps += 1
        head, cam = m.state(env, "head"), m.state(env, "camera")
        c = measure(head, cam, m.fingertips(env))
        if self.steps == 1:
            self.prev_rel = None
        self._thread(env, c)
        # the lock wheel's thread: its height follows its turn (anticlockwise from above raises it)
        target = float(np.clip(PITCH * math.radians(c["wheel"]) / (2 * math.pi), 0.0, WHEEL_RISE_MAX))
        m.set_target(env, "head", "lw_rise", target)
        # a jam nut: once the wheel is run up hard against the camera (it presses the camera's base, its thread loaded),
        # a finger can no longer turn it; it turns again once the camera is backed off it
        jam = self.caught and c["wheel_gap"] <= WHEEL_GAP_MAX and target - c["rise"] >= 0.0002
        if jam and not self.jammed:
            m.lock(env, "head", "lw")
            self.jammed = True
            print(f"[spin] lock wheel jammed at step {self.steps}", flush=True)
        elif self.jammed and c["wheel_gap"] > 0.0006:
            m.unlock(env, "head", "lw")
            self.jammed = False
        now = np.r_[cam.pos, c["rel"], c["pan"]]
        prev = self.prev if self.prev is not None else now
        c["still"] = bool(np.max(np.abs(now[:3] - prev[:3])) < 0.0005 and abs(_wrap(now[3] - prev[3])) < 0.5 and abs(_wrap(now[4] - prev[4])) < 0.5)
        self.prev = now
        c["caught"] = self.caught
        if c["vz"] < -0.8:
            self.dropped = True
        lv = level(c, self.caught)
        ok = (not self.dropped and lv == 0.75 and abs(c["pan"]) <= PAN_TOL and c["wheel_gap"] <= WHEEL_GAP_MAX
              and c["hand_gap"] >= HANDS and c["still"])
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and not ok:  # credit does not outlast the state that earned it
            self.full = False
        self.score = 0.0 if self.dropped else (1.0 if self.full else lv)
        self.last = c
        if DEBUG and (self.steps % 20 == 0 or self.steps < 3):
            print(f"[spin] step {self.steps} caught {self.caught} z {c['z'] * 1000:+6.2f} xy {c['x'] * 1000:+5.2f} {c['y'] * 1000:+5.2f} off {c['off'] * 1000:4.2f} sb {head.joint('stud_body') * 1000:+5.2f} tilt {c['tilt']:4.1f} "
                  f"rel {c['rel']:+7.1f} unw {self.rel_unwrapped:+8.1f} aim {c['aim']:+7.1f} pan {c['pan']:+6.1f} gap {c['gap'] * 1000:+5.2f} "
                  f"wheel {c['wheel']:+7.1f} rise {c['rise'] * 1000:4.2f} wgap {c['wheel_gap'] * 1000:+5.2f} hands {c['hand_gap'] * 1000:5.0f} "
                  f"still {c['still']} drop {self.dropped} score {self.score:.2f} full {self.full}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demonstration (open loop) ---------------------------------------------------------------------------
class Orient(float):
    rot = np.eye(3)


def _orient(r):
    o = Orient(0.0)
    o.rot = r
    return o


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def down(yaw_deg: float):
    return _orient(_plain_grip(math.radians(yaw_deg)))


def level_minus_y():
    """Level, pointing -y (in from the left arm's side), the fingers closing along x."""
    return _orient(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(-90.0)) @ _plain_grip(math.radians(-90.0)))


F_CAM = -0.0095  # on the camera's 25 mm depth (12.5 mm a side): pressed 3 mm
F_SOCKET = -0.0125  # on the head's 32 mm socket body (16 mm a side): pressed 3.5 mm
F_SOCKET_OPEN = -0.030
N_STROKES = int(os.environ.get("SPIN_STROKES", "7"))  # 180 degrees each: 7 take the camera from placed to facing forward
N_NUDGES = int(os.environ.get("SPIN_NUDGES", "4"))
PLACE_FIX = np.array([0.0013, 0.0016, 0.0])  # where the camera sat off the stud in the first trials (grasp offset)
LAST_TRIM = float(os.environ.get("SPIN_TRIM", "4.0"))  # degrees added to the last stroke (open-loop calibration)
NUDGE_IN = float(os.environ.get("SPIN_NUDGE_IN", "0.0025"))


def cam_base(turn_deg: float) -> float:
    """The camera base's expected height above the platform after `turn_deg` clockwise from where it was placed."""
    to_catch = (PLACE_REL - CATCH_DEG) % 360.0  # placed facing backwards (rel 0): the thread catches 22.7 degrees on
    return Z_SHOULDER - PITCH * max(0.0, turn_deg - to_catch) / 360.0


def demo():
    """Left: in from its side, level, closes on the head's socket body and holds the head still on its loose base.
    Right: picks the camera off the desk across its depth, carries it over the stud, lenses still facing away, and
    lowers it until its hole drops onto the stud's lead. It then turns it clockwise in strokes of half a turn (let go,
    turn back, take it again), seven in all: the thread catches early in the first, and the seventh stops it facing
    along the tape, half a turn short of seated. It lets go and rolls the lock wheel up with the end of its closed
    fingers, stroke by stroke, until the wheel jams against the camera's base; then both hands let go and leave."""
    up = np.array([0.0, 0.0, 1.0])
    head = np.array([HX, HY, ZP])
    # left: hold the socket body at its middle, 31 mm below the platform
    lg = level_minus_y()
    sock = head + np.array([0.0, 0.0, -0.031])
    t_left_in = 2.5 + 1.5 + 0.6
    left = [
        Waypoint(2.5, tuple(sock + np.array([0.0, 0.12, 0.02])), F_SOCKET_OPEN, "beside the head", lg),
        Waypoint(1.5, tuple(sock + np.array([0.0, 0.002, 0.0])), F_SOCKET_OPEN, "round the socket body", lg),
        Waypoint(0.6, tuple(sock + np.array([0.0, 0.002, 0.0])), F_SOCKET, "hold the head", lg),
    ]
    # right: pick the camera
    cam = np.array([CAM_START[0], CAM_START[1], TOP])
    tcp_cam = CAM_H - 0.004  # TCP below the camera's top: the pads cover its upper 17 mm
    g0 = 90.0  # fingers closing along x: across the camera's depth
    right = [
        Waypoint(2.5, tuple(cam + up * (tcp_cam + 0.08)), FINGER_OPEN, "over the camera", down(g0)),
        Waypoint(1.5, tuple(cam + up * tcp_cam), FINGER_OPEN, "round the camera", down(g0)),
        Waypoint(0.6, tuple(cam + up * tcp_cam), F_CAM, "grip the camera", down(g0)),
        Waypoint(1.2, tuple(cam + up * (tcp_cam + 0.13)), F_CAM, "lift", down(g0)),
        Waypoint(2.0, tuple(head + PLACE_FIX + up * (Z_SHOULDER + tcp_cam + 0.02)), F_CAM, "over the stud", down(g0)),
        Waypoint(1.5, tuple(head + PLACE_FIX + up * (Z_SHOULDER + tcp_cam + 0.004)), F_CAM, "down to the stud", down(g0)),
        Waypoint(1.0, tuple(head + PLACE_FIX + up * (Z_SHOULDER + tcp_cam - 0.0015)), F_CAM, "onto the lead", down(g0)),
    ]
    z0 = Z_SHOULDER + tcp_cam - 0.0015
    right += [  # turn it back a little and forward again, so that its bore settles over the stud's lead
        Waypoint(0.8, tuple(head + PLACE_FIX + up * z0), F_CAM, "turn back to find the lead", down(g0 + 30.0)),
        Waypoint(0.8, tuple(head + PLACE_FIX + up * z0), F_CAM, "and forward", down(g0)),
    ]
    turned = 0.0
    press = -0.0010  # TCP this far below where the camera sits: a light push down onto the thread
    for k in range(N_STROKES):
        stroke = 180.0 + (LAST_TRIM if k == N_STROKES - 1 else 0.0)
        for piece in range(3):
            turned += stroke / 3
            yaw = g0 - (turned - 180.0 * k)
            z = cam_base(turned) + tcp_cam + press
            right.append(Waypoint(0.9, tuple(head + up * z), F_CAM, f"stroke {k + 1}", down(yaw)))
        z = cam_base(turned) + tcp_cam
        right += [
            Waypoint(0.4, tuple(head + up * z), FINGER_OPEN, "let go", down(g0 - 180.0 - (stroke - 180.0))),
            Waypoint(0.5, tuple(head + up * (z + 0.032)), FINGER_OPEN, "lift off", down(g0 - 180.0 - (stroke - 180.0))),
        ]
        if k < N_STROKES - 1:
            right += [
                Waypoint(0.6, tuple(head + up * (z + 0.032)), FINGER_OPEN, "turn back", down(g0 - 90.0)),
                Waypoint(0.6, tuple(head + up * (z + 0.032)), FINGER_OPEN, "turn back", down(g0)),
                Waypoint(0.6, tuple(head + up * (z + press)), FINGER_OPEN, "down round the camera", down(g0)),
                Waypoint(0.4, tuple(head + up * (z + press)), F_CAM, "take the camera", down(g0)),
            ]
    # the lock wheel: the closed fingers' end (36 mm wide along x) beside the camera's front face, at the wheel's rim
    z_tip = -0.003 + 0.013  # fingertips 3 mm below the platform's top: the fingers stand outside its 17.5 mm radius
    x_end = -(WHEEL_R - NUDGE_IN)  # the fingers' +x end this far from the stud's axis: pressed NUDGE_IN into the rim
    tcp_x = x_end - 0.018
    right += [
        Waypoint(1.0, tuple(head + np.array([tcp_x, 0.012, 0.06])), FINGER_CLOSED, "up and over the wheel", down(0.0)),
    ]
    for k in range(N_NUDGES):
        right += [
            Waypoint(0.6, tuple(head + np.array([tcp_x, 0.012, z_tip + 0.010])), FINGER_CLOSED, f"wheel stroke {k + 1}", down(0.0)),
            Waypoint(0.4, tuple(head + np.array([tcp_x, 0.012, z_tip])), FINGER_CLOSED, "on the rim", down(0.0)),
            Waypoint(1.0, tuple(head + np.array([tcp_x, -0.012, z_tip])), FINGER_CLOSED, "roll the wheel up", down(0.0)),
            Waypoint(0.4, tuple(head + np.array([tcp_x, -0.012, z_tip + 0.010])), FINGER_CLOSED, "lift", down(0.0)),
        ]
    right += [
        Waypoint(1.0, tuple(head + np.array([-0.08, -0.06, 0.08])), FINGER_OPEN, "right clear", down(0.0)),
        Waypoint(1.5, (0.22, -0.30, TOP + 0.20), FINGER_OPEN, "right away", down(0.0)),
    ]
    t_right = sum(w.duration for w in right)
    left += [
        Waypoint(t_right - t_left_in - 1.5, tuple(sock + np.array([0.0, 0.002, 0.0])), F_SOCKET, "hold the head while the camera goes on", lg),
        Waypoint(0.6, tuple(sock + np.array([0.0, 0.002, 0.0])), F_SOCKET_OPEN, "let go of the head", lg),
        Waypoint(1.2, tuple(sock + np.array([0.0, 0.12, 0.0])), F_SOCKET_OPEN, "back out", lg),
        Waypoint(1.5, tuple(sock + np.array([-0.06, 0.16, 0.06])), FINGER_OPEN, "left away", lg),
        Waypoint(4.0, tuple(sock + np.array([-0.06, 0.16, 0.06])), FINGER_OPEN, "wait", lg),
    ]
    right.append(Waypoint(t_left_in + 1.5 + 0.6 + 1.2 + 1.5 + 4.0 - 1.5, (0.22, -0.30, TOP + 0.20), FINGER_OPEN, "wait", down(0.0)))
    return {"left": left, "right": right}
