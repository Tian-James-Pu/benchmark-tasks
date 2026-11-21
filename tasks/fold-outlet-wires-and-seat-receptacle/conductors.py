"""conductors: stiff solid-copper building wire on a receptacle, as one articulation for a yam_arena task file.

Written for this task. It builds on mechkit (``mechkit.py`` next to it): a ``wired_device`` is a mechkit ``Part``
whose USD text this module writes, so mechkit spawns it, reports it to ``score()`` and runs its hooks.

The articulation:

- ``frame``: the cable clamp, welded to the world (the cable's sheath ends in it);
- each conductor: a chain of rigid links from the clamp, joined by two-axis bending joints (D6: bend about the
  joint's y and z, no stretch, no twist). Every joint has a stiff spring whose rest angle is the wire's permanent set
  and whose torque is capped at the wire's yield moment: bent past that, the joint yields, and ``Copper.step``
  moves the rest angle to where the wire now is, less its elastic springback. That is an elastic, plastic bend
  with work hardening: each joint's yield moment grows with the plastic rotation it has taken.
- ``device``: the receptacle, welded to the end of the first conductor; the other conductors' ends are welded to it
  by loop-closing joints outside the articulation's tree (the terminal screws);
- extra links on the device (its two captive mounting screws), as ordinary mechkit ``Link``s.

Everything is authored in the part's frame at its starting pose: joint zero is the starting shape.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import mechkit as m


@dataclass(frozen=True)
class Wire:
    """One conductor. ``points``: its centreline from the clamp to the terminal, in the part's frame (m), one row per
    joint node: the first is at the clamp, the last at the terminal. ``radius``: the outside (insulation, or bare copper)."""

    name: str
    points: tuple
    radius: float
    look: str
    copper_d: float = 0.002053  # 12 AWG
    mass_per_m: float = 0.0360  # kg/m: copper 29.6 g/m plus insulation

    def nodes(self) -> np.ndarray:
        return np.asarray(self.points, float)

    def length(self) -> float:
        p = self.nodes()
        return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


@dataclass(frozen=True)
class CopperLaw:
    """Bending law per joint, from beam theory for solid copper of diameter d over a link length L:
    elastic stiffness E*I/L; first yield at sigma_y*pi*d^3/32 rising to sigma_y*d^3/6 fully plastic; linear hardening
    in the yield moment with plastic rotation, up to a cap (the moment at the hardened flow stress)."""

    E: float = 117e9  # Pa, annealed copper
    sigma_y: float = 69e6  # Pa, annealed (soft-drawn) copper, 0.2 % offset
    sigma_cap: float = 200e6  # Pa, flow stress well into hardening
    harden: float = 0.03  # N m of extra yield moment per radian of accumulated plastic rotation (per joint)

    def stiffness(self, d: float, L: float) -> float:
        return self.E * math.pi * d ** 4 / 64.0 / L

    def yield_moment(self, d: float) -> float:
        return self.sigma_y * d ** 3 / 6.0

    def cap_moment(self, d: float) -> float:
        return self.sigma_cap * d ** 3 / 6.0


def _quat_from_x(d) -> tuple:
    """A rotation (w, x, y, z) taking +x to the direction d."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    x = np.array([1.0, 0.0, 0.0])
    c = float(np.dot(x, d))
    if c > 1 - 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    if c < -1 + 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    axis = np.cross(x, d)
    axis /= np.linalg.norm(axis)
    a = math.acos(c)
    s = math.sin(a / 2)
    return (math.cos(a / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _q(q) -> str:
    return f"({q[0]:.7f}, {q[1]:.7f}, {q[2]:.7f}, {q[3]:.7f})"


def _p(p) -> str:
    return f"({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})"


def _body(name: str, mass: float, com, inertia, parts, colliders, gravity: bool = True, filtered=()) -> str:
    apis = '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"' + (', "PhysicsFilteredPairsAPI"' if filtered else "")
    filt = "" if not filtered else "\n        rel physics:filteredPairs = [" + ", ".join(f"</{m._ROOT}/{f}>" for f in filtered) + "]"
    grav = "" if gravity else "\n        bool physxRigidBody:disableGravity = 1"
    return f"""
    def Xform "{name}" (
        prepend apiSchemas = [{apis}]
    )
    {{
        float physics:mass = {mass:.6f}
        point3f physics:centerOfMass = {_p(com)}
        float3 physics:diagonalInertia = ({inertia[0]:.3e}, {inertia[1]:.3e}, {inertia[2]:.3e})
        float physxRigidBody:maxDepenetrationVelocity = 0.5
        float physxRigidBody:linearDamping = 0.2
        float physxRigidBody:angularDamping = 1.0{grav}{filt}
{m._visual(parts)}{"".join(colliders)}
    }}"""


def _d6(name: str, body0: str, body1: str, at, q, bend_deg: float) -> str:
    """A bending joint at `at`, its x axis along the wire (rotation q): bends about y and z, locked otherwise."""
    lim = ", ".join(f'"PhysicsLimitAPI:{d}"' for d in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"))
    drv = '"PhysicsDriveAPI:rotY", "PhysicsDriveAPI:rotZ"'
    locked = "".join(f"""
        float limit:{d}:physics:low = 1
        float limit:{d}:physics:high = -1""" for d in ("transX", "transY", "transZ", "rotX"))
    bends = "".join(f"""
        float limit:{d}:physics:low = {-bend_deg:.3f}
        float limit:{d}:physics:high = {bend_deg:.3f}
        uniform token drive:{d}:physics:type = "force"
        float drive:{d}:physics:stiffness = 0
        float drive:{d}:physics:damping = 0
        float drive:{d}:physics:maxForce = 1""" for d in ("rotY", "rotZ"))
    return f"""
    def PhysicsJoint "{name}" (
        prepend apiSchemas = [{lim}, {drv}]
    )
    {{
        rel physics:body0 = </{m._ROOT}/{body0}>
        rel physics:body1 = </{m._ROOT}/{body1}>
        point3f physics:localPos0 = {_p(at)}
        quatf physics:localRot0 = {_q(q)}
        point3f physics:localPos1 = {_p(at)}
        quatf physics:localRot1 = {_q(q)}{locked}{bends}
    }}"""


def _fixed_joint(name: str, body0: str, body1: str, at, loop: bool) -> str:
    ex = "\n        bool physics:excludeFromArticulation = 1" if loop else ""
    return f"""
    def PhysicsFixedJoint "{name}"
    {{{ex}
        rel physics:body0 = </{m._ROOT}/{body0}>
        rel physics:body1 = </{m._ROOT}/{body1}>
        point3f physics:localPos0 = {_p(at)}
        quatf physics:localRot0 = (1, 0, 0, 0)
        point3f physics:localPos1 = {_p(at)}
        quatf physics:localRot1 = (1, 0, 0, 0)
    }}"""


def seg_name(wire: Wire, i: int) -> str:
    return f"{wire.name}_s{i}"


def joint_names(wire: Wire) -> list[str]:
    """The articulation joint (prim) names of a wire's bends, clamp end first."""
    return [f"{wire.name}_b{i}" for i in range(len(wire.points) - 1)]


@dataclass(frozen=True)
class DeviceSpec:
    parts: tuple
    colliders: tuple
    mass: float
    com: tuple
    inertia: tuple
    filtered_segments: int = 2  # each wire's last links that may touch the device (they sit on its terminal)


def wired_device(name: str, wires: list[Wire], device: DeviceSpec, looks: dict, frame_parts=(), frame_colliders=(),
                 extra_links=(), law: CopperLaw = CopperLaw(), armature: float = 2e-4, damping: float = 0.004,
                 bend_deg: float = 150.0, solver_iters: int = 64) -> m.Part:
    """The clamp (frame), each wire's links and the device, as one mechkit part. ``extra_links``: mechkit ``Link``s whose
    ``parent`` is "device". The first wire carries the device in the articulation's tree; the others close loops."""
    wires = list(wires)

    def build() -> str:
        bodies, joints = [], []
        for wi, w in enumerate(wires):
            p = w.nodes()
            n = len(p) - 1
            for i in range(n):
                a, b = p[i], p[i + 1]
                L = float(np.linalg.norm(b - a))
                mass = w.mass_per_m * L
                I = max(mass * L * L / 12.0, 1e-9)
                tube = m.tube([a, b], w.radius, sides=10)
                core = m.tube([a + 0.12 * (b - a), b - 0.12 * (b - a)], w.radius * 0.999, sides=8)
                bodies.append(_body(seg_name(w, i), mass, (a + b) / 2, (I, I, I), ((f"v{i}", tube, w.look),),
                                    (m.col_hull(f"c{i}", core),)))
            for i in range(n):
                parent = "frame" if i == 0 else seg_name(w, i - 1)
                d = p[i + 1] - p[i] if i == 0 else (p[i + 1] - p[i - 1])
                joints.append(_d6(f"{w.name}_b{i}", parent, seg_name(w, i), p[i], _quat_from_x(d), bend_deg))
        filtered = [seg_name(w, len(w.points) - 2 - k) for w in wires for k in range(device.filtered_segments)]
        bodies.append(_body("device", device.mass, device.com, device.inertia, device.parts, device.colliders, filtered=filtered))
        w0 = wires[0]
        joints.append(_fixed_joint("terminal_" + w0.name, seg_name(w0, len(w0.points) - 2), "device", w0.nodes()[-1], loop=False))
        for w in wires[1:]:
            joints.append(_fixed_joint("terminal_" + w.name, "device", seg_name(w, len(w.points) - 2), w.nodes()[-1], loop=True))
        for k in extra_links:
            bodies.append(m._link_usda(k))
            joints.append(m._joint_usda(k))
        body = f"""
    def Xform "frame" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {{
        float physics:mass = 1.0
        float3 physics:diagonalInertia = (0.01, 0.01, 0.01)
{m._visual(frame_parts)}{"".join(frame_colliders)}
    }}
{"".join(bodies)}
    def PhysicsFixedJoint "world_joint"
    {{
        rel physics:body1 = </{m._ROOT}/frame>
    }}
{"".join(joints)}"""
        attrs = f"""
    bool physxArticulation:enabledSelfCollisions = 1
    int physxArticulation:solverPositionIterationCount = {solver_iters}
    int physxArticulation:solverVelocityIterationCount = 4"""
        return m._file(m._root_name(name), '"PhysicsArticulationRootAPI", "PhysxArticulationAPI"', attrs, body, looks, (0.6, 0.5))

    gains = {}
    for w in wires:
        L = w.length() / (len(w.points) - 1)
        k = law.stiffness(w.copper_d, L)
        gains[f"{w.name}_b.*"] = (k, damping, law.yield_moment(w.copper_d), 0.0, armature)
    for k in extra_links:
        gains[k.joint] = k.gains()
    limits = tuple((k.joint, *(math.radians(v) if k.kind == "hinge" else v for v in (k.lower, k.upper))) for k in extra_links
                   if math.isfinite(k.lower) and math.isfinite(k.upper))
    return m.Part(name, pos=(0.0, 0.0, 0.0), size=(0.1, 0.1, 0.1), role="jointed", build=build, friction=(0.6, 0.5),
                  limits=limits, drives=tuple(gains.items()), start_joints=())


class Copper:
    """The plastic law, run once per control step from a mechkit ``on_step`` hook. ``reset`` (from ``on_reset``) puts
    every joint back to the fresh wire: no set, no hardening."""

    def __init__(self, part: str, wires: list[Wire], law: CopperLaw = CopperLaw()):
        self.part, self.wires, self.law = part, list(wires), law
        self.ids = None
        self.k = self.my = self.cap = None
        self.target = self.plastic = None

    def _setup(self, env):
        asset = env.unwrapped.scene[self.part]
        names = list(asset.joint_names)
        ids, k, my, cap = [], [], [], []
        for w in self.wires:
            L = w.length() / (len(w.points) - 1)
            for j, n in enumerate(names):
                if n.startswith(f"{w.name}_b"):
                    ids.append(j)
                    k.append(self.law.stiffness(w.copper_d, L))
                    my.append(self.law.yield_moment(w.copper_d))
                    cap.append(self.law.cap_moment(w.copper_d))
        self.ids, self.k, self.my, self.cap = np.array(ids), np.array(k), np.array(my), np.array(cap)
        self.names = [names[i] for i in ids]

    def reset(self, env):
        if self.ids is None:
            self._setup(env)
        self.target = np.zeros(len(self.ids))
        self.plastic = np.zeros(len(self.ids))
        self._write(env, self.my)

    def moments(self) -> np.ndarray:
        return np.minimum(self.my + self.law.harden * self.plastic, self.cap)

    def step(self, env):
        if self.ids is None:
            self.reset(env)
        import torch
        asset = env.unwrapped.scene[self.part]
        q = m._np(asset.data.joint_pos)[0, self.ids]
        mom = self.moments()
        el = mom / self.k  # the largest elastic angle
        over = q - self.target
        flow = np.where(np.abs(over) > el, over - np.sign(over) * el, 0.0)
        self.target = self.target + flow
        self.plastic = self.plastic + np.abs(flow)
        self._write(env, self.moments())

    def _write(self, env, mom):
        import torch
        asset = env.unwrapped.scene[self.part]
        dev = asset.device
        ids = [int(i) for i in self.ids]
        asset.set_joint_position_target(torch.tensor([self.target], dtype=torch.float32, device=dev), joint_ids=ids)
        asset.write_joint_effort_limit_to_sim(torch.tensor([mom], dtype=torch.float32, device=dev), joint_ids=ids)


def poses(env, part: str) -> dict:
    """Every body's (R, p) now, in the station frame."""
    base = env.unwrapped
    asset = base.scene[part]
    origin = m._np(base.scene.env_origins[0])
    pq = m._np(asset.data.body_link_pose_w)[0]
    return {n: (m.quat_matrix(pq[i, 3:7]), pq[i, :3] - origin) for i, n in enumerate(asset.body_names)}


def wire_points(P: dict, wire: Wire, per_seg: int = 3) -> np.ndarray:
    """Points along a wire's centreline now (station frame): ``per_seg`` per link, then its terminal end."""
    p = wire.nodes()
    out = []
    for i in range(len(p) - 1):
        R, t = P[seg_name(wire, i)]
        for f in np.linspace(0.0, 1.0, per_seg, endpoint=False):
            out.append(R @ (p[i] + f * (p[i + 1] - p[i])) + t)
    R, t = P[seg_name(wire, len(p) - 2)]
    out.append(R @ p[-1] + t)
    return np.array(out)


def device_velocity(env, part: str, body: str = "device"):
    """A body's linear and angular velocity now (station frame)."""
    asset = env.unwrapped.scene[part]
    v = m._np(asset.data.body_link_vel_w)[0, list(asset.body_names).index(body)]
    return v[3:6], v[:3]
