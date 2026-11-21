"""Unsubmitted full-size 91-pin contour-gauge feasibility coupon.

Only native rigid-body/contact and passive prismatic joints. No pin pose writes,
relatching, scripted attachment or lock. End-shell dimensions, mass and joint
friction are disclosed estimates from manufacturer photographs, not calibration.
Score always remains zero: these are diagnostic metrics, not task validation.
"""
import dataclasses
import json
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mechkit as m
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Diagnostic only: grasp the contour gauge frame, contact the profile fixture, lift it, and put it flat on the board."
SUCCESS_HOLD_S = 0.0
PIN_COUNT = 91
PITCH = .130 / PIN_COUNT
PIN_WIDTH = .00133
PIN_MASS = .0006
JOINT_FRICTION = float(os.environ.get("COUPON_JOINT_FRICTION", ".05"))
GRASP_Y = .084  # corrected from .095 after actual tapered-pad intersection check
RESULT = pathlib.Path(os.environ.get("COUPON_RESULT", str(HERE / "coupon-result.json")))
LOOKS = {
    "frame": m.Look(color=(.10, .12, .12), roughness=.6),
    "pin": m.Look(color=(.02, .50, .56), roughness=.5),
    "wood": m.Look(color=(.57, .39, .22), roughness=.8),
    "board": m.Look(color=(.73, .62, .45), roughness=.8),
}


def capsule_outline(half_x=.022, straight_y=.0625):
    # Real frame is rounded. Dimensions other than overall envelope are estimates.
    pts = []
    for angle in np.linspace(0, math.pi, 17):
        pts.append((half_x * math.cos(angle), straight_y + half_x * math.sin(angle)))
    for angle in np.linspace(math.pi, 2 * math.pi, 17):
        pts.append((half_x * math.cos(angle), -straight_y + half_x * math.sin(angle)))
    return pts


outline = capsule_outline()
top_shell = m.prism(outline, .014, .020)
bottom_shell = m.prism(outline, 0, .003)
frame_parts = [("top", top_shell, "frame"), ("bottom", bottom_shell, "frame")]
frame_cols = [m.col_hull("top_c", top_shell), m.col_hull("bottom_c", bottom_shell)]
# Rounded exposed frame ends. They do not create extra handles or stands.
for sign in (-1, 1):
    end_name = "negative" if sign < 0 else "positive"
    end_outline = [(x, y) for x, y in outline if sign * y >= .065]
    end_outline += [(-.02186, sign * .065), (.02186, sign * .065)]
    # Sort around the centroid to keep the convex cap winding consistent.
    center = np.mean(end_outline, axis=0)
    end_outline.sort(key=lambda xy: math.atan2(xy[1] - center[1], xy[0] - center[0]))
    shape = m.prism(end_outline, .003, .014)
    frame_parts.append((f"end_{end_name}", shape, "frame"))
    frame_cols.append(m.col_hull(f"end_{end_name}_c", shape))
# The manufacturer's side lever is present as a fixed visual envelope only.
# Its locking action is explicitly absent from this first coupon.
lever = m.box((.008, .011, .012), (0, -.090, .010))
frame_parts.append(("unmodelled_lock_lever", lever, "pin"))
frame_cols.append(m.col_hull("lever_envelope_c", lever))

pins = []
for i in range(PIN_COUNT):
    y = (i - (PIN_COUNT - 1) / 2) * PITCH
    body = m.box((.105, PIN_WIDTH, .007), (0, y, .0065))
    # All 91 source-scale pins collide with the fixture, floor and robot.
    pins.append(m.slider(f"pin{i}", "X", -.015, .015,
                         [("body", body, "pin")], [m.col_hull("pin_c", body)],
                         at=(0, y, .0065), mass=PIN_MASS,
                         spring=0., damping=.002, friction=JOINT_FRICTION,
                         effort=1., armature=.00001, gravity=True,
                         com=(0, y, .0065),
                         inertia=(PIN_MASS * (.007**2 + PIN_WIDTH**2) / 12,
                                  PIN_MASS * (.105**2 + .007**2) / 12,
                                  PIN_MASS * (.105**2 + PIN_WIDTH**2) / 12)))

gauge = m.mechanism("gauge", (.32, 0, TOP + .0003), pins, LOOKS,
                    frame_parts=frame_parts, frame_colliders=frame_cols,
                    free_mass=.12, friction=(.45, .35), solver_iters=32)
original_build = gauge.build


def native_gauge_build():
    text = original_build()
    text = text.replace("physxArticulation:enabledSelfCollisions = 0",
                        "physxArticulation:enabledSelfCollisions = 1")
    # Mechkit's 4 mm offsets overwhelm a ~1.43 mm pin pitch.
    text = text.replace("physxCollision:contactOffset = 0.004",
                        "physxCollision:contactOffset = 0.00005")
    text = text.replace('uniform token physics:axis = "X"',
                        'bool physics:collisionEnabled = 1\n        uniform token physics:axis = "X"')
    return text


gauge = dataclasses.replace(gauge, build=native_gauge_build)

# A three-depth contact fixture, solely a coupon probe, not a claimed casing.
probe_parts, probe_cols = [], []
for i, (y, front) in enumerate([(-.020, .403), (0, .398), (.020, .408)]):
    shape = m.box((.020, .0195, .030), (front + .010, y, .015))
    probe_parts.append((f"step{i}", shape, "wood"))
    probe_cols.append(m.col_hull(f"step{i}_c", shape))
probe = m.fixed("profile_probe", (0, 0, TOP), probe_parts, probe_cols, LOOKS,
                friction=(.4, .3))
board_shape = m.box((.15, .25, .020), (.18, 0, .010))
board = m.fixed("board", (0, 0, TOP), [("board", board_shape, "board")],
                [m.col_hull("board_c", board_shape)], LOOKS, friction=(.5, .4))
OBJECTS = [gauge, probe, board]


class Metrics:
    def reset(self, env=None):
        self.n = 0
        self.samples = []
        self.start_z = None
        self.max_lift = 0.
        self.max_joint_abs = 0.

    def step(self, env):
        self.n += 1
        s = m.state(env, "gauge")
        q = np.array([s.joint(f"pin{i}") for i in range(PIN_COUNT)])
        if not np.isfinite(q).all() or not np.isfinite(s.pos).all():
            raise RuntimeError("Non-finite coupon state")
        if self.start_z is None:
            self.start_z = float(s.pos[2])
        self.max_lift = max(self.max_lift, float(s.pos[2]) - self.start_z)
        self.max_joint_abs = max(self.max_joint_abs, float(np.abs(q).max()))
        sample = dict(step=self.n, frame_pos=s.pos.tolist(),
                      tilt_deg=float(np.degrees(np.arccos(np.clip(s.rotation()[2, 2], -1, 1)))),
                      frame_speed=float(np.linalg.norm(s.lin_vel)),
                      min_pin_mm=float(q.min() * 1000), max_pin_mm=float(q.max() * 1000),
                      pin_q_mm=(q * 1000).tolist())
        if self.n % 20 == 0:
            robot = env.unwrapped.scene["left_robot"]
            gripper_id = robot.find_bodies("gripper")[0][0]
            pose = m._np(robot.data.body_link_pose_w)[0, gripper_id].copy()
            pose[:3] -= m._np(env.unwrapped.scene.env_origins[0])
            opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
            tcp = pose[:3] + m.quat_matrix(pose[3:7]) @ np.array([0., 0., .134])
            sample.update(left_gripper_pose_xyzw=pose.tolist(),
                          left_opening_mm=opening * 1000,
                          left_tcp_world=tcp.tolist(),
                          left_tcp_in_frame=s.to_local(tcp)[0].tolist())
            self.samples.append(sample)
            print("CONTOUR_COUPON " + json.dumps({k: v for k, v in sample.items() if k != "pin_q_mm"}), flush=True)
            self.save(sample)

    def save(self, sample):
        RESULT.parent.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps(dict(diagnostic_only=True, task_validation=False,
                          pin_count=PIN_COUNT, joint_friction_coefficient=JOINT_FRICTION,
                          friction_not_measured_Newtons=True, locking_not_modelled=True,
                          max_lift_mm=self.max_lift * 1000,
                          max_joint_abs_mm=self.max_joint_abs * 1000,
                          latest=sample, samples=self.samples), indent=2))


METRICS = Metrics()
METRICS.reset()
m.on_reset(METRICS.reset)
m.on_step(METRICS.step)


def score(objects):
    return 0.0


def success(objects):
    return False


def waypoints():
    # Native end-cap contact only. No pose feedback or object state mutation.
    yaw = -math.pi / 2
    grasp_z = TOP + .022
    closed = -.014
    return {"left": [
        Waypoint(1.5, (.32, GRASP_Y, TOP + .12), FINGER_OPEN, "approach unlevered end", yaw),
        Waypoint(1.5, (.32, GRASP_Y, grasp_z), FINGER_OPEN, "down to end cap", yaw),
        Waypoint(1.0, (.32, GRASP_Y, grasp_z), closed, "native end-frame pinch", yaw),
        Waypoint(1.0, (.32, GRASP_Y, grasp_z), closed, "settle grasp", yaw),
        Waypoint(2.0, (.353, GRASP_Y, grasp_z), closed, "press pins against profile probe", yaw),
        Waypoint(1.0, (.353, GRASP_Y, grasp_z), closed, "hold profile contact", yaw),
        Waypoint(1.5, (.320, GRASP_Y, grasp_z), closed, "withdraw without relatching", yaw),
        Waypoint(1.5, (.320, GRASP_Y, TOP + .102), closed, "lift eighty millimetres", yaw),
        Waypoint(1.5, (.180, GRASP_Y, TOP + .102), closed, "carry flat over board", yaw),
        Waypoint(1.5, (.180, GRASP_Y, TOP + .042), closed, "flat setdown on board", yaw),
        Waypoint(1.0, (.180, GRASP_Y, TOP + .042), FINGER_OPEN, "release", yaw),
        Waypoint(1.5, (.180, GRASP_Y, TOP + .142), FINGER_OPEN, "withdraw hand", yaw),
        Waypoint(2.0, (.180, GRASP_Y, TOP + .142), FINGER_OPEN, "observe final state", yaw),
    ]}


from closedloop import placeholder_demo
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation

demo = placeholder_demo


def demo_script(ctx):
    arm = ctx.left
    plan = waypoints()["left"]
    first = plan[0]
    rot = grip_rotation(first.yaw)
    q1, pe, re = ik.solve(first.pos, ik.matrix_to_quat_xyzw(rot), np.array(READY),
                          base_pos=arm.base_pos, iters=200)
    arm.worst_ik_mm = max(arm.worst_ik_mm, pe * 1000)
    arm.worst_ik_deg = max(arm.worst_ik_deg, float(np.degrees(re)))
    q0 = arm.q.copy()
    # Robot joint commands only: avoid the ready-to-yawed-grip Cartesian singularity.
    for i in range(1, round(first.duration / arm.dt) + 1):
        f = ik.smoothstep(i / round(first.duration / arm.dt))
        arm.q = q0 + f * (q1 - q0)
        arm.finger = FINGER_OPEN
        arm.pos, arm.rot = ik.tcp_pose(arm.q, arm.base_pos)
        yield
    for w in plan[1:]:
        print("COUPON_PHASE " + w.label, flush=True)
        yield from arm.move(w.pos, w.duration, w.finger, rot=rot)
        if w.label == "down to end cap":
            yield from arm.trim_to_measured(w.pos, tol=.0015, max_comp=.008, max_duration=.8)
