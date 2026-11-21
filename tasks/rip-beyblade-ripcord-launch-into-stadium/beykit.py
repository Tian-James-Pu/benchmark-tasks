"""Geometry and physics helpers for the Beyblade launch task: the Beystadium bowl, the top, and their colliders.

Plain NumPy plus mechkit's USD writers. Shared by task.py (and nothing else); importing it needs no Isaac Sim.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as m

# -- surfaces of revolution -------------------------------------------------------------------


def revolve(profile, segments: int = 96) -> m.Mesh:
    """A closed solid of revolution about z from an (r, z) outline traced from the axis at the bottom round to the axis at
    the top (r 0 at both ends). Faces point outwards."""
    prof = [(float(r), float(z)) for r, z in profile]
    ang = np.linspace(0.0, 2 * math.pi, segments, endpoint=False)
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
    # orient outwards: the signed volume of a closed outward mesh is positive
    vol = np.einsum("ij,ij->i", v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum() / 6.0
    if vol < 0:
        f = f[:, ::-1].copy()
    return m.Mesh(v, f)


def col_mesh(name: str, mesh: m.Mesh) -> str:
    """A triangle-mesh collider (exact, concave). Only for a fixed part: PhysX allows it on static bodies only."""
    v, f = mesh.v, mesh.f
    head = m._COL_HEAD.replace('"MaterialBindingAPI"', '"PhysicsMeshCollisionAPI", "MaterialBindingAPI"')
    return f"""
    def Mesh "{name}" {head}
        uniform token physics:approximation = "none"
        int[] faceVertexCounts = [{", ".join("3" for _ in range(len(f)))}]
        int[] faceVertexIndices = [{", ".join(str(int(i)) for i in f.reshape(-1))}]
        point3f[] points = [{m._fmt(v)}]
        uniform token subdivisionScheme = "none"
    }}"""


def col_sphere(name: str, radius: float, center=(0.0, 0.0, 0.0)) -> str:
    return f"""
    def Sphere "{name}" {m._COL_HEAD}
        double radius = {radius:.6f}
        double3 xformOp:translate = ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""


# -- the Beystadium ----------------------------------------------------------------------------

STADIUM_R = 0.197  # outer radius: 15.5 in across (retailer listing)
RIM_H = 0.075  # rim top above the table (estimate)
FLOOR_Z = 0.012  # the bowl's lowest point above the table (estimate)
BOWL_R = 0.170  # the dish reaches this radius ...
BOWL_EDGE_Z = 0.052  # ... at this height, as a parabola from the centre (estimate)
WALL_R = 0.185  # the inner wall meets the rim here


def bowl_z(r: float) -> float:
    """Height of the stadium's floor above the table at radius r (inside the dish)."""
    r = min(abs(r), BOWL_R)
    return FLOOR_Z + (BOWL_EDGE_Z - FLOOR_Z) * (r / BOWL_R) ** 2


def stadium_profile():
    inner = [(r, bowl_z(r)) for r in np.linspace(BOWL_R, 0.0, 23)]
    return [(0.0, 0.0), (STADIUM_R, 0.0), (STADIUM_R, RIM_H), (WALL_R, RIM_H), (BOWL_R + 0.004, BOWL_EDGE_Z + 0.008)] + inner


def stadium_mesh(segments: int = 128) -> m.Mesh:
    return revolve(stadium_profile(), segments)


# -- the top -------------------------------------------------------------------------------------

TIP_R = 0.0012  # the performance tip's rounded point (estimate for a sharp tip)
# (r, z) from the point up: performance tip, forge disc, energy layer (estimates, see README)
TOP_PROFILE = [(0.0, 0.0), (0.0016, 0.0004), (0.0045, 0.0060), (0.0075, 0.0110), (0.0110, 0.0130), (0.0110, 0.0170),
               (0.0215, 0.0175), (0.0215, 0.0235), (0.0245, 0.0245), (0.0245, 0.0330), (0.0160, 0.0370), (0.0, 0.0375)]
TOP_MASS = 0.040
TOP_COM_Z = 0.023
TOP_I3 = 9.0e-6  # about the spin axis, kg m^2
TOP_I1 = 6.0e-6  # about a diameter through the centre of mass


def top_body_mesh() -> m.Mesh:
    return revolve(TOP_PROFILE, 32)


def top_colliders(prefix: str = "top", tip_r: float = TIP_R) -> tuple[str, ...]:
    body = revolve([(0.0, max(0.0030, 2.2 * tip_r))] + [p for p in TOP_PROFILE if p[1] >= 0.0060], 24)
    return (col_sphere(f"{prefix}_tip_c", tip_r, (0.0, 0.0, tip_r)), m.col_hull(f"{prefix}_body_c", body))


def spinning_free(name: str, pos, parts, colliders, looks, mass: float, com, inertia, friction=(0.9, 0.8), ang_damping: float = 0.03,
                  start_quat=None, yaw_deg: float = 0.0) -> m.Part:
    """A free rigid body set up to spin fast: PhysX's gyroscopic term on, the angular speed cap raised, an explicit inertia,
    no torsional friction patch (a point contact), and a small angular damping for air drag."""
    attrs = f"""
    float physics:mass = {mass:.6f}
    point3f physics:centerOfMass = ({com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f})
    float3 physics:diagonalInertia = ({inertia[0]:.4e}, {inertia[1]:.4e}, {inertia[2]:.4e})
    bool physxRigidBody:enableGyroscopicForces = 1
    float physxRigidBody:maxAngularVelocity = 200000
    float physxRigidBody:angularDamping = {ang_damping:.5f}
    float physxRigidBody:linearDamping = 0
    float physxRigidBody:sleepThreshold = 0
    float physxRigidBody:stabilizationThreshold = 0"""
    root = m._root_name(name)
    return m.Part(name, pos=tuple(pos), size=(0.05, 0.05, 0.05), mass=mass, role="free", friction=friction, torsional_patch=None, solver_iters=32,
                  yaw_deg=yaw_deg, start_quat=start_quat,
                  build=lambda: m._file(root, '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"', attrs, m._visual(parts) + "".join(colliders), looks, friction))


# -- writing a rigid body's state from a hook ---------------------------------------------------------


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


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    """xyzw."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)
