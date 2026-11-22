"""The coin for the spinning-quarter task: a US quarter as a free rigid body set up to spin fast, plus small helpers.

Plain NumPy plus mechkit's USD writers; importing it needs no Isaac Sim. `spinning_free`, `write_body` and `quat_from_matrix`
follow the helpers of the same names in the Beyblade launch task's beykit.py (tasks/rip-beyblade-ripcord-launch-into-stadium).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as m

# -- the coin: United States Mint specification for the quarter dollar (clad) ---------------------------------------------
COIN_MASS = 0.00567  # kg
COIN_D = 0.02426  # m, diameter
COIN_T = 0.00175  # m, thickness
COIN_R = COIN_D / 2
EDGE_R = 0.0002  # m: the rounded corner of the rim (estimate; the reeded edge meets each face in a small radius)
# a uniform disc: about its own axis m r^2 / 2; about a diameter m (3 r^2 + t^2) / 12
COIN_I_AXIS = COIN_MASS * COIN_R ** 2 / 2
COIN_I_DIAM = COIN_MASS * (3 * COIN_R ** 2 + COIN_T ** 2) / 12


def coin_profile(n_round: int = 4):
    """(r, z) from the axis on the lower face round the rounded rim to the axis on the upper face (coin frame: z along the axis)."""
    h, rr, R = COIN_T / 2, EDGE_R, COIN_R
    prof = [(0.0, -h)]
    for k in range(n_round + 1):  # lower corner
        a = -math.pi / 2 + (math.pi / 2) * k / n_round
        prof.append((R - rr + rr * math.cos(a), -h + rr + rr * math.sin(a)))
    for k in range(n_round + 1):  # upper corner
        a = (math.pi / 2) * k / n_round
        prof.append((R - rr + rr * math.cos(a), h - rr + rr * math.sin(a)))
    prof.append((0.0, h))
    return prof


def revolve(profile, segments: int = 64, offset: float = 0.0) -> m.Mesh:
    """A closed solid of revolution about z from an (r, z) outline that starts and ends on the axis. Faces point outwards."""
    prof = [(float(r), float(z)) for r, z in profile]
    ang = np.linspace(0.0, 2 * math.pi, segments, endpoint=False) + offset
    verts, rings = [], []
    for r, z in prof:
        if r < 1e-9:
            rings.append([len(verts)])
            verts.append((0.0, 0.0, z))
        else:
            rings.append(list(range(len(verts), len(verts) + segments)))
            verts.extend((r * math.cos(a), r * math.sin(a), z) for a in ang)
    faces = []
    for a, b in zip(rings[:-1], rings[1:]):
        for k in range(segments):
            k1 = (k + 1) % segments
            if len(a) == 1 and len(b) == 1:
                continue
            if len(a) == 1:
                faces.append((a[0], b[k1], b[k]))
            elif len(b) == 1:
                faces.append((a[k], a[k1], b[0]))
            else:
                faces.append((a[k], a[k1], b[k1]))
                faces.append((a[k], b[k1], b[k]))
    v, f = np.asarray(verts, float), np.asarray(faces, int)
    vol = np.einsum("ij,ij->i", v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum() / 6.0
    if vol < 0:
        f = f[:, ::-1].copy()
    return m.Mesh(v, f)


def spinning_free(name: str, pos, parts, colliders, looks, mass: float, inertia, friction=(0.3, 0.25), ang_damping: float = 0.0,
                  torsional_patch: float | None = None, start_quat=None, yaw_deg: float = 0.0, solver_iters: int = 32) -> m.Part:
    """A free rigid body set up to spin fast: PhysX's gyroscopic term on, the angular speed cap raised, an explicit inertia
    (centre of mass at the origin), no sleeping, and an angular damping for air drag."""
    attrs = f"""
    float physics:mass = {mass:.6f}
    point3f physics:centerOfMass = (0, 0, 0)
    float3 physics:diagonalInertia = ({inertia[0]:.4e}, {inertia[1]:.4e}, {inertia[2]:.4e})
    bool physxRigidBody:enableGyroscopicForces = 1
    float physxRigidBody:maxAngularVelocity = 200000
    float physxRigidBody:maxDepenetrationVelocity = 0.3
    float physxRigidBody:angularDamping = {ang_damping:.5f}
    float physxRigidBody:linearDamping = 0
    float physxRigidBody:sleepThreshold = 0
    float physxRigidBody:stabilizationThreshold = 0"""
    root = m._root_name(name)
    return m.Part(name, pos=tuple(pos), size=(0.03, 0.03, 0.03), mass=mass, role="free", friction=friction, torsional_patch=torsional_patch,
                  solver_iters=solver_iters, yaw_deg=yaw_deg, start_quat=start_quat,
                  build=lambda: m._file(root, '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"', attrs, m._visual(parts) + "".join(colliders), looks, friction))


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def write_body(env, name: str, pos, quat_xyzw, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0)) -> None:
    import torch

    sc = env.unwrapped.scene
    asset = sc[name]
    origin = sc.env_origins[0]
    pose = torch.tensor([[*map(float, pos), *map(float, quat_xyzw)]], dtype=torch.float32, device=origin.device)
    pose[:, :3] += origin.to(torch.float32)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([[*map(float, lin), *map(float, ang)]], dtype=torch.float32, device=origin.device)
    asset.write_root_velocity_to_sim_index(root_velocity=v)


# -- per-part collision settings ----------------------------------------------------------------------------------------------
# mechkit gives every free part a 5 mm contact offset. That is wider than the coin is thick, and PhysX then lets a 1.75 mm disc
# standing on its edge rock on contacts it generates early. The coin gets its own, smaller offsets here.

COLLISION: dict[str, dict] = {}


def collision(name: str, **props) -> None:
    """Collision properties (Isaac Lab CollisionPropertiesCfg fields) for the free part `name`, replacing mechkit's defaults."""
    COLLISION[name] = props


_plain_arena_object = m._arena_object


def _arena_object(spec):
    obj = _plain_arena_object(spec)
    if spec.name in COLLISION:
        import isaaclab.sim as sim_utils

        for attr in ("spawn_cfg_addon", "_spawn_cfg_addon"):
            addon = getattr(obj, attr, None)
            if isinstance(addon, dict) and "collision_props" in addon:
                addon["collision_props"] = sim_utils.CollisionPropertiesCfg(**COLLISION[spec.name])
                break
        else:
            raise RuntimeError(f"coinkit: cannot find the collision settings of {spec.name}")
    return obj


m._arena_object = _arena_object
