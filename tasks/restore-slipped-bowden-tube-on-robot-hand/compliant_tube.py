"""Discrete hollow beam for tube/contact diagnostics; not calibrated hardware.

Segments conserve their length and have two bending degrees of freedom at
each junction. A straight stress-free tube is represented by nonzero spring
rest angles when the supplied initial route is curved. There is no seat test,
attachment to the housing, or change in stiffness when an end enters a hole.
Axial stretch, torsion, ovalization, plastic kinking and creep are omitted.
"""

from dataclasses import dataclass, replace
import math
import numpy as np
import mechkit as m


def _basis(direction):
    z = direction / np.linalg.norm(direction)
    reference = np.array([1., 0., 0.])
    if abs(z @ reference) > .95:
        reference = np.array([0., 1., 0.])
    x = reference - z * (z @ reference)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def _straight_rest(parent, child):
    """Rx(a) Ry(b) maps child to parent for routes pointing along positive Z."""
    radius = math.hypot(child[0], child[2])
    if min(parent[2], child[2]) <= 0 or abs(parent[0]) >= radius:
        raise ValueError('This diagnostic discretization requires a +Z-monotone route')
    b = math.asin(parent[0] / radius) - math.atan2(child[0], child[2])
    z = -math.sin(b)*child[0] + math.cos(b)*child[2]
    a = math.atan2(child[1], z) - math.atan2(parent[1], parent[2])
    return a, b


@dataclass
class Tube:
    part: object
    points: np.ndarray
    bodies: tuple
    rest_targets: dict
    properties: dict
    _target_cache: object = None

    def set_rest_targets(self, env):
        """Batch constant material rest angles, including during reset settling."""
        import torch
        asset = env.unwrapped.scene[self.part.name]
        if self._target_cache is None or self._target_cache[0] is not asset:
            names = list(self.rest_targets)
            ids = [asset.joint_names.index(n+'_joint') for n in names]
            targets = torch.tensor([[self.rest_targets[n] for n in names]],
                                   dtype=torch.float32, device=asset.device)
            self._target_cache = asset, ids, targets
        _, ids, targets = self._target_cache
        asset.set_joint_position_target(targets, joint_ids=ids)


def make_tube(name, position, points, *, outer_radius=.0015,
              inner_radius=.00075, density=2150., modulus=496e6,
              damping_ratio=.2, radial_facets=24, fixed_root=False, start=()):
    """Build an uncalibrated elastic beam with explicit physical dimensions.

    2150 kg/m³ and 496 MPa are generic PTFE resin reference values from
    Chemours' product-selection guide, not measurements of the hand's tube.
    Each virtual bend carrier takes 0.1% of its segment's lumped mass; this
    small numerical approximation is included in the reported total mass.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        raise ValueError('Expected an N-by-3 tube route')
    if not np.isfinite(points).all() or not 0 < inner_radius < outer_radius:
        raise ValueError('Invalid tube geometry')
    vectors = np.diff(points, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    if np.any(lengths <= 0) or density <= 0 or modulus <= 0:
        raise ValueError('Tube lengths and material values must be positive')
    directions = vectors / lengths[:, None]
    area = math.pi*(outer_radius**2-inner_radius**2)
    second_moment = math.pi/4*(outer_radius**4-inner_radius**4)
    looks = {'tube': m.Look(color=m.rgb(.90, .90, .85), roughness=.5)}
    links, rests, metadata, bodies = [], {}, {}, []
    base_parts = base_colliders = None
    stiffnesses = []
    for i, (a, b, length, direction) in enumerate(zip(
            points[:-1], points[1:], lengths, directions, strict=True)):
        body = 'frame' if i == 0 else f'segment_{i}'
        bodies.append(body)
        center = (a+b)/2
        basis = _basis(direction)
        mass = density*area*length
        carrier_mass = 0. if i == 0 else mass*.001
        solid_mass = mass-carrier_mass
        transverse = solid_mass*(3*(outer_radius**2+inner_radius**2)+length**2)/12
        axial = solid_mass*(outer_radius**2+inner_radius**2)/2
        inertia = (transverse, transverse, axial)
        parts, colliders = [], []
        for j in range(radial_facets):
            lo, hi = 2*math.pi*j/radial_facets, 2*math.pi*(j+1)/radial_facets
            polygon = [(r*math.cos(t), r*math.sin(t)) for r, t in
                       ((outer_radius, lo), (outer_radius, hi),
                        (inner_radius, hi), (inner_radius, lo))]
            wedge = m.prism(polygon, -length/2, length/2)
            wedge = m.Mesh(wedge.v @ basis.T + center, wedge.f)
            parts.append((f'wall_{j}', wedge, 'tube'))
            colliders.append(m.col_hull(f'wall_{j}_c', wedge))
        metadata[body] = dict(mass=solid_mass, center=center,
                              inertia=inertia, principal_axes=basis)
        if i == 0:
            base_parts, base_colliders = parts, colliders
            continue
        carrier = f'bend_{i}'
        stiffness = modulus*second_moment/((lengths[i-1]+length)/2)
        if fixed_root and i == 1:
            # The first segment is the clamp. Only the free segment's half
            # cell bends at this boundary; do not include the rigid clamp.
            stiffness = 2*modulus*second_moment/length
        stiffnesses.append(float(stiffness))
        damping = 2*damping_ratio*math.sqrt(stiffness*transverse)
        rest_x, rest_y = _straight_rest(directions[i-1], direction)
        rests[carrier], rests[body] = rest_x, rest_y
        parent = None if i == 1 else f'segment_{i-1}'
        carrier_inertia = (carrier_mass*outer_radius**2/6,)*3
        links.append(m.hinge(carrier, 'X', a, -85., 85., (), (),
            mass=carrier_mass, com=tuple(a), inertia=carrier_inertia,
            spring=stiffness, damping=damping, friction=0., effort=2.,
            armature=0., gravity=True, parent=parent))
        links.append(m.hinge(body, 'Y', a, -85., 85., parts, colliders,
            mass=solid_mass, com=tuple(center), inertia=inertia,
            spring=stiffness, damping=damping, friction=0., effort=2.,
            armature=0., gravity=True, parent=carrier))
        metadata[carrier] = dict(mass=carrier_mass, center=a,
                                 inertia=carrier_inertia, principal_axes=np.eye(3))
    part = m.mechanism(name, position, links, looks,
        frame_parts=base_parts, frame_colliders=base_colliders,
        free_mass=None if fixed_root else metadata['frame']['mass'],
        frame_inertia=metadata['frame']['inertia'], solver_iters=64,
        friction=(.15, .10), start=start)
    original_build = part.build
    def build():
        from pxr import Gf, Sdf, Usd, UsdPhysics
        from scipy.spatial.transform import Rotation
        # Author physical masses/COMs precisely; tiny carrier masses must not
        # round to zero through the general kit's five-decimal formatter.
        text = original_build().replace(
            '"PhysicsRigidBodyAPI", "PhysicsMassAPI"]',
            '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]')
        layer = Sdf.Layer.CreateAnonymous('.usda')
        if not layer.ImportFromString(text):
            raise ValueError('Invalid tube USD')
        stage = Usd.Stage.Open(layer)
        root = stage.GetDefaultPrim()
        root.GetAttribute('physxArticulation:solverVelocityIterationCount').Set(8)
        for body, values in metadata.items():
            prim = root.GetChild(body)
            mass = UsdPhysics.MassAPI(prim)
            mass.GetMassAttr().Set(float(values['mass']))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*values['center']))
            mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*values['inertia']))
            q = Rotation.from_matrix(values['principal_axes']).as_quat()
            mass.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))
            for key in ('linearDamping', 'angularDamping'):
                prim.CreateAttribute('physxRigidBody:'+key, Sdf.ValueTypeNames.Float).Set(0.)
        for link in links:
            joint = UsdPhysics.Joint(root.GetChild(link.joint))
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(*link.at))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(*link.at))
        return layer.ExportToString()
    return Tube(replace(part, build=build), points.copy(), tuple(bodies), rests,
        {'length_m': float(lengths.sum()), 'mass_kg': float(density*area*lengths.sum()),
         'outer_radius_m': outer_radius, 'inner_radius_m': inner_radius,
         'density_kg_m3': density, 'flexural_modulus_pa': modulus,
         'second_moment_m4': second_moment, 'bending_stiffness_Nm_per_rad': stiffnesses,
         'carrier_mass_fraction': .001, 'segments': len(lengths)})
