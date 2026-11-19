"""Independent alternating plates, freely rotating rollers and a closed pin loop.

No poses are overwritten, joints attached/detached, or transmission flags used.
Lateral joint parameters are mandatory inputs because specimen calibration is
still missing. This is a mechanism builder, not a validated task or demo.
"""
from dataclasses import dataclass, replace
import math
import numpy as np
import mechkit as m
from chain_geometry import (PITCH, INNER_WIDTH, PIN_LENGTH, ROLLER_RADIUS,
                            PLATE_HEIGHT, PLATE_THICKNESS, link_frames)
from scaled_convex import scaled_convex_part

STEEL_DENSITY = 7850.
PIN_RADIUS = .00365/2  # ISO082 cross-check; selected KMC tolerance unresolved
CONTACT_OFFSET = .00005
LOOKS = {'steel': m.Look(color=m.rgb(.59, .61, .62), metallic=.85, roughness=.35)}


@dataclass(frozen=True)
class PinResponse:
    lateral_limit_deg: float
    lateral_stiffness_nm_per_rad: float
    lateral_damping_nms_per_rad: float
    provenance: str
    calibrated: bool = False

    def __post_init__(self):
        if not 0 < self.lateral_limit_deg < 15:
            raise ValueError('Pin lateral play must be explicit and mechanically bounded')
        if min(self.lateral_stiffness_nm_per_rad, self.lateral_damping_nms_per_rad) < 0:
            raise ValueError('Passive joint response cannot inject energy')
        if not self.provenance:
            raise ValueError('Record measurement or explicit uncalibrated assumption')


def _fine(text):
    return text.replace('contactOffset = 0.004', f'contactOffset = {CONTACT_OFFSET}')


def _plate_pieces(z):
    r, waist = PLATE_HEIGHT/2, .0028
    # Two semicircular ends and two trapezoids preserve the stamped waist.
    # Pin holes are omitted from external collision surfaces and mass estimate.
    left = np.array([[PITCH/-2+r*math.cos(t), r*math.sin(t)]
                     for t in np.linspace(math.pi/2, 3*math.pi/2, 17)])
    right = np.array([[PITCH/2+r*math.cos(t), r*math.sin(t)]
                      for t in np.linspace(-math.pi/2, math.pi/2, 17)])
    outlines = [left, right,
                np.array([[-PITCH/2, -r], [0, -waist], [0, waist], [-PITCH/2, r]]),
                np.array([[0, -waist], [PITCH/2, -r], [PITCH/2, r], [0, waist]])]
    return [m.prism(p, z-PLATE_THICKNESS/2, z+PLATE_THICKNESS/2) for p in outlines]


def _mesh_mass_inertia(meshes):
    # Signed tetrahedral integration through trimesh, offline dependency.
    import trimesh
    volume = 0.; inertia = np.zeros((3, 3))
    for shape in meshes:
        mesh = trimesh.Trimesh(shape.v, shape.f, process=True)
        if not mesh.is_watertight or mesh.volume <= 0:
            raise ValueError('Mass geometry must be a positively oriented closed mesh')
        mass = mesh.volume*STEEL_DENSITY
        center = mesh.center_mass
        volume += mesh.volume
        inertia += mesh.moment_inertia*STEEL_DENSITY + mass*(np.eye(3)*center.dot(center)-np.outer(center, center))
    if np.max(np.abs(inertia-np.diag(np.diag(inertia)))) > 1e-10:
        raise ValueError('Expected symmetric local principal axes')
    return volume*STEEL_DENSITY, np.diag(inertia)


def _body(name, position, yaw, meshes, extra='', filtered=()):
    mass, inertia = _mesh_mass_inertia(meshes)
    part = m.free(name, position, [(f'visual_{i}', s, 'steel') for i,s in enumerate(meshes)],
                  [_fine(m.col_hull(f'collider_{i}', s)) for i,s in enumerate(meshes)],
                  LOOKS, mass=mass, friction=(.15, .10), torsional_patch=None,
                  solver_iters=64, yaw_deg=math.degrees(yaw), com=(0.,0.,0.))
    part = replace(part, contact_offset=CONTACT_OFFSET)
    original = part.build
    def build():
        text = original()
        text = text.replace('"PhysicsRigidBodyAPI", "PhysicsMassAPI"',
                            '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"')
        text = text.replace('point3f physics:centerOfMass',
            f'int physxRigidBody:solverPositionIterationCount = 64\n    int physxRigidBody:solverVelocityIterationCount = 8\n    float physics:mass = {mass:.12g}\n    float3 physics:diagonalInertia = ({inertia[0]:.12g}, {inertia[1]:.12g}, {inertia[2]:.12g})\n    point3f physics:centerOfMass')
        if filtered:
            text = text.replace('"PhysxRigidBodyAPI"',
                                '"PhysxRigidBodyAPI", "PhysicsFilteredPairsAPI"')
            targets = ', '.join(f'</World/envs/env_0/{x}>' for x in filtered)
            text = text.replace('float3 physics:diagonalInertia',
                                f'rel physics:filteredPairs = [{targets}]\n    float3 physics:diagonalInertia')
        end = text.rfind('}')
        return text[:end] + extra + text[end:]
    return scaled_convex_part(replace(part, build=build))


def _pin_joint(name, other, yaw0, yaw1, response):
    schemas = ['PhysicsLimitAPI:'+a for a in ('transX','transY','transZ','rotX','rotY')]
    schemas += ['PhysicsDriveAPI:rotX', 'PhysicsDriveAPI:rotY']
    limits = '\n'.join(f'float limit:{a}:physics:low = 1\nfloat limit:{a}:physics:high = -1'
                       for a in ('transX','transY','transZ'))
    for a in ('rotX','rotY'):
        limits += f'''
        float limit:{a}:physics:low = {-response.lateral_limit_deg}
        float limit:{a}:physics:high = {response.lateral_limit_deg}
        token drive:{a}:physics:type = "force"
        float drive:{a}:physics:stiffness = {response.lateral_stiffness_nm_per_rad*math.pi/180}
        float drive:{a}:physics:damping = {response.lateral_damping_nms_per_rad*math.pi/180}
        float drive:{a}:physics:targetPosition = 0
        '''
    return f'''
    def PhysicsJoint "next_pin" (prepend apiSchemas = {schemas!r}) {{
        rel physics:body0 = </P_{name}>
        rel physics:body1 = </World/envs/env_0/{other}>
        point3f physics:localPos0 = ({PITCH/2}, 0, 0)
        point3f physics:localPos1 = ({-PITCH/2}, 0, 0)
        quatf physics:localRot0 = ({math.cos(yaw0/2)}, 0, 0, {-math.sin(yaw0/2)})
        quatf physics:localRot1 = ({math.cos(yaw1/2)}, 0, 0, {-math.sin(yaw1/2)})
        bool physics:collisionEnabled = false
        {limits}
    }}
    '''.replace("'", '"')


def closed_chain(pins_xy, plane_height, response, name_prefix='chain'):
    centers, yaws = link_frames(pins_xy)
    count = len(centers)
    parts = []
    names = [f'{name_prefix}_{i:03}' for i in range(count)]
    for i, (center, yaw) in enumerate(zip(centers, yaws)):
        inner = i % 2 == 0
        offset = INNER_WIDTH/2 + PLATE_THICKNESS/2
        if not inner:
            offset += PLATE_THICKNESS+.0001
        meshes = [shape for z in (-offset, offset) for shape in _plate_pieces(z)]
        if not inner:
            meshes += [m.cylinder(PIN_RADIUS, PIN_LENGTH, (x,0,0), segments=24)
                       for x in (-PITCH/2, PITCH/2)]
        joint = _pin_joint(names[i], names[(i+1)%count], yaw, yaws[(i+1)%count], response)
        parts.append(_body(names[i], (*center,plane_height), yaw, meshes, joint))
        if inner:
            for side, delta in (('a', -1), ('b', 1)):
                name = names[i]+'_roller_'+side
                point = center + delta*PITCH/2*np.array([math.cos(yaw), math.sin(yaw)])
                h = INNER_WIDTH*.96
                roller = m.lathe([(PIN_RADIUS,-h/2), (ROLLER_RADIUS,-h/2),
                                  (ROLLER_RADIUS,h/2),(PIN_RADIUS,h/2),(PIN_RADIUS,-h/2)])
                # Cylinder exterior convex collision fills only the pin bore,
                # filtered solely against this roller's two connected plates.
                joint = f'''
                def PhysicsRevoluteJoint "bearing" {{
                    rel physics:body0 = </World/envs/env_0/{names[i]}>
                    rel physics:body1 = </P_{name}>
                    token physics:axis = "Z"
                    point3f physics:localPos0 = ({delta*PITCH/2},0,0)
                    point3f physics:localPos1 = (0,0,0)
                    bool physics:collisionEnabled = false
                }}
                '''
                parts.append(_body(name, (*point,plane_height), yaw, [roller], joint,
                                   filtered=(names[(i+delta)%count],)))
    return parts


def chain_assembly(name, origin, pins_xy, plane_height, response):
    """One USD reference containing independent rigid bodies and loop joints.

    Cross-body targets stay inside the same reference namespace. Exporting each
    body as a separate USD reference would discard targets outside its root.
    The scene root has no rigid body or articulation API: the closed pin loop is
    a rigid-body constraint graph, not a reduced-coordinate articulation tree.
    Runtime observations must read the individual child bodies, not this root.
    """
    bodies = closed_chain(pins_xy, plane_height, response)
    def build():
        from pxr import Gf, Sdf, Usd, UsdGeom
        stage = Usd.Stage.CreateInMemory()
        root = stage.DefinePrim('/P_'+name, 'Xform')
        stage.SetDefaultPrim(root)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        for body in bodies:
            source = Sdf.Layer.CreateAnonymous('.usda')
            if not source.ImportFromString(body.build()):
                raise ValueError('Cannot parse chain body '+body.name)
            old = Sdf.Path('/P_'+body.name)
            new = root.GetPath().AppendChild(body.name)
            Sdf.CopySpec(source, old, stage.GetRootLayer(), new)
            prim = stage.GetPrimAtPath(new)
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*body.pos))
            xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(body.yaw_deg)
            def mapped(path):
                if path.HasPrefix(old):
                    return path.ReplacePrefix(old,new)
                external = Sdf.Path('/World/envs/env_0')
                if path.HasPrefix(external):
                    return path.ReplacePrefix(external,root.GetPath())
                return path
            for child in Usd.PrimRange(prim):
                for rel in child.GetRelationships():
                    targets = rel.GetTargets()
                    if targets:
                        rel.SetTargets([mapped(p) for p in targets])
                for attr in child.GetAttributes():
                    paths = attr.GetConnections()
                    if paths:
                        attr.SetConnections([mapped(p) for p in paths])
        return stage.GetRootLayer().ExportToString()
    part = m.fixed(name, origin, [], looks={})
    return replace(part, build=build), bodies
