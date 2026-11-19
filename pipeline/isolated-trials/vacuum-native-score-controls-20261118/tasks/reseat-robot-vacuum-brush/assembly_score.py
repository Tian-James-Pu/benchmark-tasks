"""Pure numeric proposal; NOT connected to the task or a simulation.

All poses are native link origins in the authored coordinate convention, never
COM poses. In mechkit the moving links' geometry uses the parent coordinates at
joint zero. Consequently a revolute link origin moves when rotating about its
off-origin pivot. Supply actual body_link_pose_w, not root pose plus stale q.
Distances are metres, quaternions xyzw, angles radians unless named otherwise.
"""
from dataclasses import dataclass
from itertools import product
import math
import numpy as np

AX, AZ = .018, .0775
NUMERIC_EPS = 1e-10
PENETRATION_ALLOWANCE = .0001  # proposed 0.1 mm numerical allowance, not clearance


@dataclass(frozen=True)
class Pose:
    pos: tuple
    quat_xyzw: tuple

    def rotation(self):
        p = np.asarray(self.pos, dtype=float)
        q = np.asarray(self.quat_xyzw, dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise ValueError('invalid native pose')
        if abs(np.linalg.norm(q) - 1.) > 1e-3:
            raise ValueError('non-unit native quaternion')
        x, y, z, w = q / np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    def world(self, points):
        return np.asarray(points) @ self.rotation().T + np.asarray(self.pos)

    def local(self, points):
        return (np.asarray(points) - np.asarray(self.pos)) @ self.rotation()


@dataclass(frozen=True)
class Assembly:
    vacuum: Pose
    roller: Pose
    cover: Pose
    motor_socket: Pose
    outer_bearing: Pose
    right_catch: Pose
    left_catch: Pose


@dataclass(frozen=True)
class BodyVelocity:
    """Measured rigid-body twist, both vectors in WORLD axes.

    Linear velocity is AT reference_position_world, explicitly either the native
    COM or native link origin. Pair body_com_vel_w with body_com_pose_w, or pair
    body_link_vel_w with body_link_pose_w. Never mix these or differentiate q.
    """
    linear_world_m_s: tuple
    angular_world_rad_s: tuple
    reference_position_world: tuple
    reference_kind: str  # exactly 'com' or 'link_origin'

    def vectors(self):
        arrays = [np.asarray(x, float) for x in
                  (self.linear_world_m_s, self.angular_world_rad_s, self.reference_position_world)]
        if self.reference_kind not in ('com', 'link_origin') or any(
                x.shape != (3,) or not np.isfinite(x).all() for x in arrays):
            raise ValueError('invalid native body velocity/reference convention')
        return arrays

    def at(self, point_world):
        v, w, reference = self.vectors()
        return v + np.cross(w, np.asarray(point_world)-reference)


@dataclass(frozen=True)
class Kinematics:
    vacuum: BodyVelocity
    roller: BodyVelocity
    cover: BodyVelocity
    motor_socket: BodyVelocity
    outer_bearing: BodyVelocity
    right_catch: BodyVelocity
    left_catch: BodyVelocity


def motion_metrics(a: Assembly, motion: Kinematics):
    """Stability at physical support features; axial rotor spin is permitted.

    No velocities are reconstructed from joints. Rigid-body transport from the
    measured velocity reference point handles off-pivot authored link origins.
    """
    for name in a.__dataclass_fields__:
        item = getattr(motion, name)
        _, _, reference = item.vectors()
        if item.reference_kind == 'link_origin' and np.linalg.norm(
                reference-np.asarray(getattr(a, name).pos)) > 1e-6:
            raise ValueError('link-origin velocity paired with a different pose origin')
    features = {
        'vacuum': [(0, 0, .019), (-.075, 0, .008), (AX, -.099, AZ), (AX, .096, AZ)],
        'cover': [(x, side*.1135, -.003) for x in (-.017, .017) for side in (-1, 1)],
        'roller': [(0, -.096, 0), (0, .096, 0)],
        'motor_socket': [(AX, -.099, AZ)],
        'outer_bearing': [(0, .096, 0)],
        'right_catch': [(-.034, -.086, -.003)],
        'left_catch': [(-.034, .086, -.003)],
    }
    speeds = {name: max(float(np.linalg.norm(getattr(motion, name).at(getattr(a, name).world(p))))
                       for p in points) for name, points in features.items()}
    # These components are intended to remain stationary, unlike the inner rotor.
    stationary = ('vacuum', 'cover', 'outer_bearing', 'right_catch', 'left_catch')
    angular = {name: float(np.linalg.norm(getattr(motion, name).vectors()[1]))
               for name in stationary}
    transverse, axial = {}, {}
    housing_w = motion.vacuum.vectors()[1]
    for name in ('roller', 'motor_socket'):
        axis = getattr(a, name).rotation()[:, 1]
        relative_w = getattr(motion, name).vectors()[1]-housing_w
        axial[name] = float(np.dot(relative_w, axis))
        transverse[name] = float(np.linalg.norm(relative_w-axial[name]*axis))
    stable = (max(speeds.values()) <= .002 and max(angular.values()) <= .05
              and max(transverse.values()) <= .05)
    return dict(stable=bool(stable), support_feature_speeds_m_s=speeds,
                stationary_angular_speeds_rad_s=angular,
                rotor_transverse_angular_speeds_rad_s=transverse,
                permitted_axial_speeds_rad_s=axial)


@dataclass(frozen=True)
class RuntimeEvidence:
    # No defaults: missing contact/velocity/support instrumentation must fail closed.
    instrumentation_valid: bool
    robot_contact_free: bool  # ALL robot links versus ALL assembly bodies, each physics step
    min_robot_clearance_m: float  # actual geometry distance, not TCP/fingertip distance
    chassis_supported: bool
    roller_supported: bool
    cover_supported: bool  # loose cover supported suffices for partial; no falling loose part
    kinematics: Kinematics  # actual native twists, not velocities inferred from joint q
    intact: bool
    episode_invalid: bool  # authoritative persistent damage/invalid-state history, if present

    def released_stable(self, assembly):
        stable = motion_metrics(assembly, self.kinematics)['stable']
        return bool(math.isfinite(self.min_robot_clearance_m) and self.min_robot_clearance_m >= 0
                    and self.instrumentation_valid and self.robot_contact_free
                    and self.min_robot_clearance_m >= .005
                    and self.chassis_supported and self.roller_supported
                    and self.cover_supported and self.intact and not self.episode_invalid and stable)


def corners(center, size):
    return np.asarray(center) + np.asarray(list(product((-1., 1.), repeat=3))) * np.asarray(size)/2


def between(v, low, high):
    v = np.asarray(v)
    return bool(np.all(v >= low-NUMERIC_EPS) and np.all(v <= high+NUMERIC_EPS))


def overlap(a0, a1, b0, b1):
    return float(max(0., min(a1, b1)-max(a0, b0)))


def angle_degrees(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1., 1.))))


def root_constraints(vacuum, roller=None, cover=None):
    """Necessary conditions using actual root poses only; NEVER assembly proof.

    Allows archived records lacking native moving-link poses to demonstrate a
    geometric rejection without synthesizing missing socket/bearing/catch data.
    """
    out = {}
    rb = vacuum.rotation()
    if roller is not None:
        error = angle_degrees(rb[:, 1], roller.rotation()[:, 1])
        out.update(roller_axis_error_deg=error, roller_axis_ok=bool(error <= 1.))
    if cover is not None:
        feet = np.array([[x, side*y, -.003] for side in (-1, 1)
                         for x in (-.017, .017) for y in (.1125, .1145)])
        feet_body = vacuum.local(cover.world(feet))
        gaps = feet_body[:, 2]-.089
        on_deck = bool(between(abs(feet_body[:, 1]), .112, .160)
                       and between(feet_body[:, 0], AX-.023, AX+.023))
        error = angle_degrees(rb[:, 2], cover.rotation()[:, 2])
        out.update(cover_foot_gaps_m=gaps.tolist(), cover_normal_error_deg=error,
                   cover_feet_on_deck=on_deck,
                   cover_seated=bool(on_deck and between(gaps, -.0001, .0008) and error <= 1.))
    return out


def geometry(a: Assembly):
    """Return inspectable dimensions and Boolean predicates; no score/history writes."""
    # Force validation of every pose, including a detached/invalid secondary link.
    for name in a.__dataclass_fields__:
        getattr(a, name).rotation()
    e = PENETRATION_ALLOWANCE
    rb = a.vacuum.rotation()
    roots = root_constraints(a.vacuum, a.roller, a.cover)
    axis_error = roots['roller_axis_error_deg']
    # Live square socket orientation is necessary: motor output is free to rotate.
    peg = a.motor_socket.local(a.roller.world(corners((0, -.096, 0), (.008, .005, .008))))
    radial_extent = float(max(np.max(abs(peg[:, 0]-AX)), np.max(abs(peg[:, 2]-AZ))))
    depth = float(-.094 - peg[:, 1].min())
    # The 5 mm peg is nominally inserted 4.5 mm into the 10 mm-deep socket.
    # It may legitimately protrude 0.5 mm beyond the mouth. All corner radial
    # bounds conservatively cover even that small protruding length.
    pivot = np.array([AX, -.099, AZ])
    motor_pivot_error = float(np.linalg.norm(a.vacuum.local(a.motor_socket.world(pivot))-pivot))
    motor_axis_error = angle_degrees(rb[:, 1], a.motor_socket.rotation()[:, 1])
    key_ok = bool(axis_error <= 1. and motor_axis_error <= .2
                  and motor_pivot_error <= .00025 and radial_extent <= .0043+e
                  and .004 <= depth <= .0055 and peg[:, 1].min() >= -.104-e)
    # Inspect the actual outer-bearing link, not just the roller root or joint angle.
    bearing = a.vacuum.local(a.outer_bearing.world(corners((0, .096, 0), (.009, .005, .008))))
    bearing_pivot_error = float(np.linalg.norm(a.roller.local(a.outer_bearing.world((0, .096, 0)))
                                              - np.array([0, .096, 0])))
    bearing_gap = float(bearing[:, 2].min()-(AZ-.004))
    bearing_y_overlap = overlap(bearing[:, 1].min(), bearing[:, 1].max(), .094, .098)
    bearing_ok = bool(between(bearing[:, 0], AX-.0048-e, AX+.0048+e)
                      and -.0001 <= bearing_gap <= .0003
                      and bearing[:, 2].max() <= AZ+.0045
                      and bearing_y_overlap >= .0035 and bearing_pivot_error <= .00025)
    roller_ok = bool(key_ok and bearing_ok)

    hooks = []
    for y in (-.085, -.035, .035, .085):
        v = a.vacuum.local(a.cover.world(corners((.044, y, -.004), (.012, .010, .003))))
        # The four points of the upper shoulder, not the projected AABB top only.
        top = a.vacuum.local(a.cover.world(np.array([[x, yy, -.0025]
                              for x in (.038, .050) for yy in (y-.005, y+.005)])))
        x_overlap = overlap(v[:, 0].min(), v[:, 0].max(), AX+.0425, AX+.0515)
        ok = (between(v[:, 1], y-.007-e, y+.007+e)
              and v[:, 0].max() <= AX+.0515+e and x_overlap >= .006
              and between(top[:, 2], .090-.0008, .090+e))
        hooks.append(dict(ok=bool(ok), x_overlap_m=x_overlap,
                          roof_gaps_m=(.090-top[:, 2]).tolist()))

    catches = []
    for side, link in ((-1, a.right_catch), (1, a.left_catch)):
        # Upper retaining shoulder is z=-4 mm and outward Y=106..112.5 mm.
        local_top = np.array([[x, side*y, -.004] for x in (-.042, -.026)
                              for y in (.106, .1125)])
        v = a.vacuum.local(link.world(local_top))
        outward = side*v[:, 1]
        # Check native link is still the original cover slider, not independently
        # placed beside the receiver. Slider travel is 0..4 mm inward.
        rel_origin = a.cover.local(link.world((0, 0, 0)))
        rel_rotation = a.cover.rotation().T @ link.rotation()
        rotation_error = math.degrees(math.acos(float(np.clip((np.trace(rel_rotation)-1)/2, -1, 1))))
        slider_ok = (abs(rel_origin[0]) <= .00025 and abs(rel_origin[2]) <= .00025
                     and -.00425 <= side*rel_origin[1] <= .00025 and rotation_error <= .2)
        # Require positive overlap with the lip and under-lip clearance. The
        # nominal upper shoulder is 1.5 mm below the lip; wider-than-zero allowed
        # gap is therefore intentional and is NOT itself seating evidence.
        capture = float(outward.max()-.109)
        x_overlap = overlap(v[:, 0].min(), v[:, 0].max(), AX-.0415, AX-.0245)
        ok = (slider_ok and capture >= .001 and outward.max() <= .115+e
              and x_overlap >= .014 and between(v[:, 2], .0878, .090+e))
        catches.append(dict(ok=bool(ok), lip_overlap_m=capture,
                            x_overlap_m=x_overlap, slider_integrity=bool(slider_ok)))

    # Real end-frame feet overlap the chassis deck outside |Y|=112 mm.
    # Four corner samples per end ensure tilted/hovering covers do not receive
    # full credit merely because one center or hook is near its nominal pose.
    foot_gap = roots['cover_foot_gaps_m']
    cover_normal_error = roots['cover_normal_error_deg']
    seated = roots['cover_seated']
    cover_ok = bool(seated and all(h['ok'] for h in hooks) and all(c['ok'] for c in catches))
    return dict(roller_ok=roller_ok, cover_ok=cover_ok, assembled=bool(roller_ok and cover_ok),
                key_ok=key_ok, bearing_ok=bearing_ok, peg_depth_m=depth,
                peg_radial_extent_m=radial_extent, roller_axis_error_deg=axis_error,
                motor_pivot_error_m=motor_pivot_error, motor_axis_error_deg=motor_axis_error,
                bearing_pivot_error_m=bearing_pivot_error, bearing_floor_gap_m=bearing_gap,
                bearing_y_overlap_m=bearing_y_overlap, hooks=hooks, catches=catches,
                cover_seated=seated, cover_foot_gaps_m=foot_gap,
                cover_normal_error_deg=cover_normal_error)


class Scorer:
    """One call per observed physics interval, with measured elapsed time.

    A missing/invalid sample resets continuous dwell. No 'ever succeeded' credit.
    Only explicit episode reset clears state. Duplicate timestamps do not add time.
    Sampling gaps over 0.1 s reset dwell; normal integration should be every
    physics step so transient robot contacts cannot hide between control frames.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous_time = None
        self.previous_partial = self.previous_full = False
        self.partial_dwell = self.full_dwell = 0.
        self.invalid_latched = False

    def update(self, now_s, a, runtime):
        if not math.isfinite(now_s):
            raise ValueError('timestamp must be finite')
        if self.previous_time is not None and now_s <= self.previous_time:
            raise ValueError('timestamps must strictly increase; explicitly reset a new episode')
        dt = 0. if self.previous_time is None else now_s-self.previous_time
        self.previous_time = now_s
        released = False
        if runtime is not None and runtime.episode_invalid:
            self.invalid_latched = True
        try:
            g = geometry(a)
            released = runtime is not None and runtime.released_stable(a) and not self.invalid_latched
            partial = bool(g['roller_ok'] and released)
            full = bool(g['assembled'] and released)
        except (ValueError, TypeError, AttributeError):
            g, partial, full = {'invalid_input': True}, False, False
        contiguous = 0. < dt <= .100000001
        self.partial_dwell = self.partial_dwell+dt if partial and self.previous_partial and contiguous else 0.
        self.full_dwell = self.full_dwell+dt if full and self.previous_full and contiguous else 0.
        self.previous_partial, self.previous_full = partial, full
        score = 1. if self.full_dwell >= 3.-NUMERIC_EPS else .5 if self.partial_dwell >= 1.-NUMERIC_EPS else 0.
        return dict(score=score, success=bool(score == 1.), geometry=g,
                    released_stable=bool(released), episode_invalid_latched=self.invalid_latched,
                    partial_dwell_s=self.partial_dwell,
                    full_dwell_s=self.full_dwell)
