"""On the station table: a Hasbro Beyblade Burst Beystadium (a 39.4 cm plastic bowl) centred 30 cm in front of the arms. Left of it a
Beyblade Burst Supergrip ripcord launcher lies on its flat side, a Beyblade Burst top already locked onto its prongs and its 9-inch
toothed ripcord pushed in. Pulling the ripcord turns the prongs and the top through the launcher's gears; when the ripcord's teeth run
out the top drops off, spinning. Distractors: a child's plastic cup, an apple and a notebook. The goal: launch the top so that it
spins in the stadium for 10 seconds."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
import beykit as bk  # noqa: E402
import demokit as dk  # noqa: E402  its grips carry a full TCP rotation through yam_scripted's waypoints
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
DT = 0.05  # control step
PHYS_DT = 1.0 / 240.0

# -- the Beystadium: fixed, centred 30 cm in front of the arms ------------------------------------------------------------
SC = np.array([0.30, 0.0])
INSIDE_R = bk.WALL_R - 0.0245  # the top's centre within this of the stadium's centre: wholly inside the inner wall
OUT_R = bk.STADIUM_R + 0.01  # beyond this the top has left the stadium (a ring-out)

# -- the launcher (frame: origin at the centre of the head's underside, z up through the grip, ripcord along y) -------------
HEAD = (0.060, 0.066, 0.026)  # x, y, z
POST = (0.024, 0.030, 0.070)  # the grip
POST_C = (0.018, 0.0, HEAD[2] + POST[2] / 2)  # its +x face flush with the head's, so the launcher lies flat on that side
STRIP_Z = 0.013  # the ripcord's slot, at the head's mid-height, on the launcher's axis
STRIP_W, STRIP_T = 0.012, 0.002
HANDLE = (0.020, 0.045, 0.030)
HANDLE_GAP = 0.025  # strip between the handle and the head when pushed fully in
HANDLE_Y = -HEAD[1] / 2 - HANDLE_GAP - HANDLE[1] / 2
STRIP_Y0, STRIP_Y1 = -HEAD[1] / 2 - HANDLE_GAP, 0.126  # the strip, inserted: 0.184 m from the handle to its tail
PINION_Y = 0.020  # where the strip's teeth mesh with the launcher's pinion
S_EXIT = STRIP_Y1 - PINION_Y  # cord pulled this far: its last tooth leaves the pinion and the top is let go (0.106 m)
S_OUT = STRIP_Y1 + HEAD[1] / 2  # and this far: the strip is out of the launcher
R_EFF = 0.0029  # the prongs turn one revolution per 18.2 mm of cord (gear train, estimate)
TOP_GAP = 0.001  # between the top's upper face and the head
TOP_TIP_Z = -(TOP_GAP + bk.TOP_PROFILE[-1][1])  # the top's tip, in the launcher frame
I_RING = 1.5e-6  # the prong ring and gears, about the axis (estimate)
ARM_LOADED = (bk.TOP_I3 + I_RING) / R_EFF ** 2  # the ring and top, felt at the cord: 1.25 kg
ARM_EMPTY = I_RING / R_EFF ** 2
LAUNCHER_MASS = 0.070

# -- the top's goal -------------------------------------------------------------------------------------------------------
TILT_MAX = 30.0  # degrees: on its tip (the forge disc's rim touches the floor near 39 degrees)
SPIN_MIN = 30.0  # rad/s
CLEAR = 0.05  # every fingertip point at least this far from the top's centre of mass
GOAL_S = 10.0

LOOKS = {
    "stadium": m.Look(color=m.rgb(0.35, 0.62, 0.95), roughness=0.25, opacity=1.0),
    "launcher": m.Look(color=m.rgb(0.93, 0.93, 0.95), roughness=0.5),
    "grip": m.Look(color=m.rgb(0.10, 0.30, 0.85), roughness=0.6),
    "prong": m.Look(color=m.rgb(0.25, 0.25, 0.28), roughness=0.5),
    "cord": m.Look(color=m.rgb(0.95, 0.80, 0.10), roughness=0.5),
    "teeth": m.Look(color=m.rgb(0.70, 0.55, 0.05), roughness=0.5),
    "handle": m.Look(color=m.rgb(0.90, 0.15, 0.12), roughness=0.5),
    "layer": m.Look(color=m.rgb(0.85, 0.10, 0.12), roughness=0.45),
    "disc": m.Look(color=m.rgb(0.72, 0.72, 0.75), metallic=0.85, roughness=0.3),
    "tip": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.4),
}

# -- the stadium ---------------------------------------------------------------------------------------------------------
_SMESH = bk.stadium_mesh()
STADIUM = m.fixed("stadium", (SC[0], SC[1], TOP), [("bowl", _SMESH, "stadium")], [bk.col_mesh("bowl_c", _SMESH)], LOOKS, friction=(0.30, 0.25))

# -- the launcher ----------------------------------------------------------------------------------------------------------
_head = m.box(HEAD, (0, 0, HEAD[2] / 2))
_post = m.box(POST, POST_C)
_ring = m.cylinder(0.024, 0.002, (0, 0, -0.0005))
_prongs = m.join(*[m.box((0.006, 0.004, 0.004), (0.020 * math.cos(a), 0.020 * math.sin(a), -0.001)).turned(0) for a in (0.0, 2.1, 4.2)])
_slot_mark = m.box((STRIP_W + 0.004, 0.002, STRIP_T + 0.004), (0, -HEAD[1] / 2 - 0.0005, STRIP_Z))
FRAME_PARTS = [("head", _head, "launcher"), ("post", _post, "grip"), ("ring", _ring, "prong"), ("prongs", _prongs, "prong"), ("slot", _slot_mark, "prong")]
FRAME_COLS = [m.col_box("head_c", HEAD, (0, 0, HEAD[2] / 2)), m.col_box("post_c", POST, POST_C)]

_strip_len = STRIP_Y1 - STRIP_Y0
_strip = m.box((STRIP_W, _strip_len, STRIP_T), (0, (STRIP_Y0 + STRIP_Y1) / 2, STRIP_Z))
_teeth = m.box((0.002, _strip_len - 0.03, STRIP_T + 0.0006), (STRIP_W / 2 - 0.001, (STRIP_Y0 + 0.03 + STRIP_Y1) / 2, STRIP_Z))
_handle = m.box(HANDLE, (0, HANDLE_Y, STRIP_Z))
CORD = m.slider("cord", "Y", -0.20, 0.0, [("strip", _strip, "cord"), ("teeth", _teeth, "teeth"), ("handle", _handle, "handle")],
                [m.col_box("strip_c", (STRIP_W, _strip_len, STRIP_T), (0, (STRIP_Y0 + STRIP_Y1) / 2, STRIP_Z)), m.col_box("handle_c", HANDLE, (0, HANDLE_Y, STRIP_Z))],
                mass=0.008, damping=0.3, friction=0.3, armature=ARM_LOADED)

# lying on the flat side (the head's and grip's +x faces) on the table, axis along +x (grip away from the arms), top towards the arms
LIE_Q = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # 90 degrees about y: launcher z -> world +x, launcher x -> world -z
L0 = np.array([0.16, 0.27, TOP + HEAD[0] / 2 + 0.0005])
LAUNCHER = m.mechanism("launcher", tuple(L0), [CORD], LOOKS, frame_parts=FRAME_PARTS, frame_colliders=FRAME_COLS, free_mass=LAUNCHER_MASS,
                       start_quat=LIE_Q, frame_inertia=(6.6e-5, 6.8e-5, 3.9e-5), friction=(0.8, 0.7))

# -- the top (a free rigid body; locked to the prongs while the cord is in mesh) ---------------------------------------------
_tp = bk.TOP_PROFILE
TOP_PARTS = [("tip", bk.revolve(_tp[:5] + [(0.0, _tp[4][1])], 24), "tip"),
             ("driver", bk.revolve([(0.0, 0.011), (0.011, 0.011), (0.011, 0.0175), (0.0, 0.0175)], 24), "tip"),
             ("disc", bk.revolve([(0.0, 0.0172), (0.0215, 0.0172), (0.0215, 0.0238), (0.0, 0.0238)], 48), "disc"),
             ("layer", bk.revolve([(0.0, 0.0236)] + _tp[8:], 48), "layer"),
             ("blade", m.join(*[m.box((0.010, 0.006, 0.005), (0.023 * math.cos(a), 0.023 * math.sin(a), 0.031)).turned(0) for a in (0.5, 2.6, 4.7)]), "layer")]


def _attached_pose(lpos, lrot, phi):
    """The top's origin (its tip) and rotation when locked to the prongs, turned phi about the launcher's axis."""
    c, s = math.cos(phi), math.sin(phi)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return lpos + lrot @ np.array([0.0, 0.0, TOP_TIP_Z]), lrot @ rz


_R_LIE = m.quat_matrix(LIE_Q)
_p_top0, _r_top0 = _attached_pose(L0, _R_LIE, 0.0)
TIP_R = 0.003  # the performance tip: a ball end (estimate)
TOP_BODY = bk.spinning_free("top", tuple(_p_top0), TOP_PARTS, bk.top_colliders(tip_r=TIP_R), LOOKS, bk.TOP_MASS, (0.0, 0.0, bk.TOP_COM_Z), (bk.TOP_I1, bk.TOP_I1, bk.TOP_I3),
                            friction=(0.30, 0.25), ang_damping=0.0, start_quat=tuple(bk.quat_from_matrix(_r_top0)))
K_AIR = 0.03  # 1/s: air drag on the spin (about the top's own axis only), applied every substep once the top is free

# -- distractors (SimReady, loose; scoring ignores them) ---------------------------------------------------------------------
CUP = SimReady("plastic_cup", pos=(0.52, 0.32, TOP), size=(0.064, 0.064, 0.100), mass=0.03, package="dishware_plastic_cup_a01",
               usd="sm_dishware_plastic_cup_a01_01.usd")
APPLE = SimReady("apple", pos=(0.56, -0.25, TOP), size=(0.070, 0.067, 0.068), mass=0.18, package="food_produce_apple_b01",
                 usd="sm_food_produce_apple_b01_01.usd")
NOTEBOOK = SimReady("notebook", pos=(0.36, -0.43, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                    usd="sm_misc_notebook_a01_01.usd", yaw=0.15)

OBJECTS = [STADIUM, LAUNCHER, TOP_BODY, CUP, APPLE, NOTEBOOK]


# -- the launcher's gears and prongs, every physics substep -----------------------------------------------------------------


class Launcher:
    """Rack and pinion with a one-way clutch: pulling the cord out spins the prong ring at (cord speed) / R_EFF; pushing it in or
    slowing down leaves the ring coasting. While the cord's teeth are in mesh the top is locked to the ring (its pose and velocity are
    written each substep from the launcher's); the substep the last tooth leaves the pinion the top is let go with the ring's spin and
    the launcher's own motion, and from then on it is an ordinary free body."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.omega, self.phi, self.released, self.release_step, self.release_omega = 0.0, 0.0, False, None, 0.0
        self.release_tilt, self.release_height, self.release_r, self.exit_speed = None, None, None, 0.0
        self.substeps = 0
        if env is not None:
            env.unwrapped.scene["top"].permanent_wrench_composer.reset()
            asset = env.unwrapped.scene["launcher"]
            j = asset.find_joints("cord_joint")[0][0]
            asset.write_joint_armature_to_sim_index(armature=m._tensor(asset, ARM_LOADED), joint_ids=[j])
            self._attach(env)

    def _launcher(self, sc):
        asset = sc["launcher"]
        i = asset.body_names.index("frame")
        origin = bk._np(sc.env_origins[0])
        pose = bk._np(asset.data.body_link_pose_w)[0, i]
        vel = bk._np(asset.data.body_link_vel_w)[0, i]
        j = asset.find_joints("cord_joint")[0][0]
        s = -float(bk._np(asset.data.joint_pos)[0, j])
        v = -float(bk._np(asset.data.joint_vel)[0, j])
        return pose[:3] - origin, m.quat_matrix(pose[3:7]), vel[:3], vel[3:6], s, v

    def _air(self, env):
        import torch
        top = env.unwrapped.scene["top"]
        q = bk._np(top.data.root_link_pose_w)[0, 3:7]
        w = bk._np(top.data.root_vel_w)[0, 3:6]
        axis = m.quat_matrix(q)[:, 2]
        tq = -K_AIR * bk.TOP_I3 * float(w @ axis) * axis
        t = torch.tensor([[[float(v) for v in tq]]], dtype=torch.float32, device=top.device)
        top.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.zeros_like(t), torques=t, body_ids=[0], is_global=True)

    def _attach(self, env):
        lpos, lrot, lv, lw, s, v = self._launcher(env.unwrapped.scene)
        self._write(env, lpos, lrot, lv, lw)

    def _write(self, env, lpos, lrot, lv, lw):
        p, r = _attached_pose(lpos, lrot, self.phi)
        axis = lrot[:, 2]
        com = p + r @ np.array([0.0, 0.0, bk.TOP_COM_Z])
        lin = lv + np.cross(lw, com - lpos)
        ang = lw - self.omega * axis  # a right-spin top: clockwise seen from above
        bk.write_body(env, "top", p, bk.quat_from_matrix(r), lin=lin, ang=ang)

    def substep(self, env):
        if self.released:
            self._air(env)
            return
        self.substeps += 1
        lpos, lrot, lv, lw, s, v = self._launcher(env.unwrapped.scene)
        if s < S_EXIT:
            self.omega = max(self.omega * (1.0 - 0.05 * PHYS_DT), v / R_EFF if v > 0 else 0.0)
        self.phi += self.omega * PHYS_DT
        self._write(env, lpos, lrot, lv, lw)
        if s >= S_EXIT:
            self.released, self.release_omega, self.exit_speed = True, self.omega, v
            p, _ = _attached_pose(lpos, lrot, self.phi)
            self.release_tilt = float(np.degrees(np.arccos(np.clip(lrot[2, 2], -1.0, 1.0))))
            self.release_height = float(p[2] - TOP - bk.bowl_z(0.0))
            self.release_r = float(np.hypot(p[0] - SC[0], p[1] - SC[1]))
            asset = env.unwrapped.scene["launcher"]
            j = asset.find_joints("cord_joint")[0][0]
            asset.write_joint_armature_to_sim_index(armature=m._tensor(asset, ARM_EMPTY), joint_ids=[j])


LAUNCH = Launcher()


def _install_substep(env):
    sc = env.unwrapped.scene
    if getattr(sc, "_bey_substep", False):
        return
    orig = sc.write_data_to_sim

    def write_data_to_sim():
        try:
            LAUNCH.substep(env)
        except Exception as e:  # noqa: BLE001
            if not getattr(sc, "_bey_err", False):
                import traceback
                sc._bey_err = True
                print(f"[beyblade] substep error {e!r}\n{traceback.format_exc()}", flush=True)
        return orig()

    sc.write_data_to_sim = write_data_to_sim
    sc._bey_substep = True


# -- the scorer -------------------------------------------------------------------------------------------------------------


class Trial:
    """Spin time: from the release (the cord's last tooth leaving the pinion) until the first control step at which the top is not
    spinning on its tip inside the stadium with both grippers clear of it. Latched once it reaches 10 s; a ring-out at any time is 0."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded outcome is cleared
        self.step_n, self.t_release, self.spin_time, self.ended, self.ring_out, self.full, self.score = 0, None, 0.0, False, False, False, 0.0
        self.why = ""

    def step(self, env):
        self.step_n += 1
        t = self.step_n * DT
        st = m.state(env, "top")
        rot = st.rotation()
        com = st.pos + rot @ np.array([0.0, 0.0, bk.TOP_COM_Z])
        r = float(np.hypot(com[0] - SC[0], com[1] - SC[1]))
        if LAUNCH.released and self.t_release is None:
            self.t_release = t
        if self.t_release is not None:
            if r > OUT_R or com[2] < TOP - 0.01:
                self.ring_out = True
            if not self.ended:
                tilt = st.tilt_deg()
                spin = float(-st.ang_vel @ rot[:, 2])
                gap = float(np.linalg.norm(m.fingertips(env) - com, axis=1).min())
                ok = r <= INSIDE_R and tilt <= TILT_MAX and spin >= SPIN_MIN and gap >= CLEAR
                if ok:
                    self.spin_time = t - self.t_release
                else:
                    self.ended = True
                    self.why = f"r {r:.3f} tilt {tilt:.1f} spin {spin:.0f} gap {gap:.3f}"
                if self.spin_time >= GOAL_S and not self.ring_out:
                    self.full = True
        if self.full:
            self.score = 1.0
        elif self.ring_out:
            self.score = 0.0
        else:
            self.score = 0.5 if self.spin_time >= 5.0 else (0.25 if self.spin_time >= 2.0 else 0.0)


TRIAL = Trial()


def _reset(env):
    _install_substep(env)
    LAUNCH.reset(env)
    TRIAL.reset(env)


m.on_reset(_reset)
m.on_step(TRIAL.step)


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo ------------------------------------------------------------------------------------------------------

GRASP_LZ = 0.075  # where along the grip the left hand holds it (launcher z)
_grip_local = np.array([POST_C[0], 0.0, GRASP_LZ])
LIFT_Z = TOP + 0.20
TIP_ABOVE = 0.03  # the top's tip this far above the bowl's centre at launch


def _grasp_lying():
    return L0 + _R_LIE @ _grip_local


def _launch_tcp():
    origin_z = TOP + bk.bowl_z(0.0) + TIP_ABOVE - TOP_TIP_Z
    return np.array([SC[0], SC[1], origin_z]) + _grip_local


def demo(pull_t: float = 0.25, pull_to: float = 0.170, rip_dir=(0.0, -1.0, 0.0)):
    g = _grasp_lying()
    c = _launch_tcp()
    down, level = dk.down(0.0), dk.level(0.0)
    held = FINGER_CLOSED
    left = [
        Waypoint(2.0, (g[0], g[1], TOP + 0.10), FINGER_OPEN, "above grip", down),
        Waypoint(1.2, (g[0], g[1], TOP + 0.016), FINGER_OPEN, "down to grip", down),
        Waypoint(0.6, (g[0], g[1], TOP + 0.016), held, "grip", down),
        Waypoint(1.2, (g[0], g[1], LIFT_Z), held, "lift", down),
        Waypoint(3.0, (c[0], c[1], LIFT_Z), held, "turn upright over the centre", level),
        Waypoint(1.0, tuple(c), held, "lower", level),
        Waypoint(3.3 + pull_t + 2.0, tuple(c), held, "hold level until the other hand is clear", level),
        Waypoint(1.0, (c[0], c[1], c[2] + 0.08), held, "lift", level),
        Waypoint(1.5, (c[0], c[1] + 0.12, c[2] + 0.08), held, "set the launcher aside", level),
        Waypoint(13.0, (c[0], c[1] + 0.12, c[2] + 0.08), held, "hold", level),
    ]
    h = np.array([SC[0], SC[1] + HANDLE_Y, c[2] - GRASP_LZ + STRIP_Z])  # the handle once the launcher is in place
    hz = h[2] - HANDLE[2] / 2 + 0.004 + 0.013  # fingertips 4 mm above the handle's underside
    d = np.asarray(rip_dir, float)
    end = np.array([h[0], h[1], hz]) + pull_to * d
    rgrip = dk.down(90.0)
    right = [
        Waypoint(6.0, (h[0], h[1] - 0.10, TOP + 0.20), FINGER_OPEN, "wait aside", rgrip),
        Waypoint(3.0, (h[0], h[1] - 0.10, TOP + 0.20), FINGER_OPEN, "wait", rgrip),
        Waypoint(1.5, (h[0], h[1], hz + 0.05), FINGER_OPEN, "above handle", rgrip),
        Waypoint(0.8, (h[0], h[1], hz), FINGER_OPEN, "down to handle", rgrip),
        Waypoint(0.6, (h[0], h[1], hz), held, "grip handle", rgrip),
        Waypoint(0.4, (h[0], h[1], hz), held, "set", rgrip),
        Waypoint(pull_t, tuple(end), held, "rip", rgrip),
        Waypoint(0.4, tuple(end), held, "hold", rgrip),
        Waypoint(0.4, tuple(end), FINGER_OPEN, "let go of the handle", rgrip),
        Waypoint(0.8, (end[0], end[1], end[2] + 0.10), FINGER_OPEN, "up", rgrip),
        Waypoint(1.0, (end[0] - 0.05, end[1] - 0.05, TOP + 0.22), FINGER_OPEN, "aside", rgrip),
        Waypoint(14.0, (end[0] - 0.05, end[1] - 0.05, TOP + 0.22), FINGER_OPEN, "hold", rgrip),
    ]
    return {"left": left, "right": right}
