"""Carrier-free two-axis tube discretization for independent validation.

Each cell has its physical hollow-cylinder mass and inertia. A D6 articulation
joint supplies the two bending degrees of freedom; translations and twist are
locked. Its axes map to the previous model's X/Y bending axes. This changes
the dynamic discretization, so previous beam/contact results do not validate
it. Geometry, density, modulus, damping ratio and EI/length stiffness remain
the same. Torsion, ovalization, plasticity and creep remain omitted.
"""
from dataclasses import dataclass, replace
import numpy as np
import mechkit as m
from compliant_tube import make_tube as original_make_tube, _basis


@dataclass
class Tube:
    part: object
    points: np.ndarray
    bodies: tuple
    rest_targets: dict
    joint_stiffness: dict
    properties: dict
    mass_properties: dict
    body_centers: dict
    _target_cache: object = None

    def set_rest_targets(self, env):
        import torch
        asset = env.unwrapped.scene[self.part.name]
        if self._target_cache is None or self._target_cache[0] is not asset:
            names = list(self.rest_targets)
            ids = [asset.joint_names.index(n) for n in names]
            targets = torch.tensor([[self.rest_targets[n] for n in names]],
                                   dtype=torch.float32, device=asset.device)
            self._target_cache = asset, ids, targets
        _, ids, targets = self._target_cache
        asset.set_joint_position_target(targets, joint_ids=ids)


def make_tube(name, position, points, **kwargs):
    if kwargs.get('start'):
        raise ValueError('Nonzero initial D6 coordinates are not implemented')
    original = original_make_tube(name, position, points, **kwargs)
    original_build = original.part.build
    points = original.points.copy()
    props = original.properties
    ro, ri = props['outer_radius_m'], props['inner_radius_m']
    density = props['density_kg_m3']
    damping_ratio = kwargs.get('damping_ratio', .2)
    centers, masses, rests, stiffnesses, drives = {}, {}, {}, {}, []
    for i, (a,b) in enumerate(zip(points[:-1], points[1:])):
        body = original.bodies[i]
        length = float(np.linalg.norm(b-a))
        mass = density*np.pi*(ro*ro-ri*ri)*length
        principal = np.array([mass*(3*(ro*ro+ri*ri)+length*length)/12]*2
                             + [mass*(ro*ro+ri*ri)/2])
        basis = _basis(b-a)
        centers[body] = (a+b)/2
        masses[body] = dict(mass=mass, center=np.zeros(3),
                           inertia_matrix=basis@np.diag(principal)@basis.T,
                           principal=principal, basis=basis)
        if i:
            k = props['bending_stiffness_Nm_per_rad'][i-1]
            damping = float(2*damping_ratio*np.sqrt(k*principal[0]))
            for axis, old in ((1,f'bend_{i}'), (2,f'segment_{i}')):
                dof = f'bend_{i}_joint:{axis}'
                rests[dof] = original.rest_targets[old]
                stiffnesses[dof] = k
                drives.append((dof,(k,damping,2.,0.,0.)))
    base = centers['frame']

    def build():
        from pxr import Usd, Sdf, UsdPhysics, UsdGeom, Gf, Vt
        from scipy.spatial.transform import Rotation
        layer = Sdf.Layer.CreateAnonymous('.usda')
        if not layer.ImportFromString(original_build()):
            raise ValueError('Invalid original beam USD')
        stage = Usd.Stage.Open(layer)
        root = stage.GetDefaultPrim()
        for i in range(1,len(points)-1):
            for child in (f'bend_{i}_joint', f'segment_{i}_joint', f'bend_{i}'):
                stage.RemovePrim(root.GetPath().AppendChild(child))
        for body, values in masses.items():
            prim = root.GetChild(body)
            xf = UsdGeom.Xformable(prim)
            if xf.GetOrderedXformOps():
                raise ValueError('Expected original common-origin body coordinates')
            xf.AddTranslateOp().Set(Gf.Vec3d(*(centers[body]-base)))
            mass = UsdPhysics.MassAPI(prim)
            mass.GetMassAttr().Set(float(values['mass']))
            mass.GetCenterOfMassAttr().Set(Gf.Vec3f(0.))
            mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*values['principal']))
            q = Rotation.from_matrix(values['basis']).as_quat()
            mass.GetPrincipalAxesAttr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    if UsdGeom.Xformable(child).GetOrderedXformOps():
                        raise ValueError('Apply convex scaling after body centering')
                    mesh = UsdGeom.Mesh(child)
                    v = np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
                    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
                        (v-centers[body]).astype(np.float32)))
        # Joint X -> body Z; joint Y -> body X; joint Z -> body Y.
        # Equal Y/Z drives satisfy the spherical-drive symmetry requirement.
        orientation = Gf.Quatf(.5,Gf.Vec3f(-.5,-.5,-.5))
        for i in range(1,len(points)-1):
            parent,child = original.bodies[i-1:i+1]
            joint = UsdPhysics.Joint.Define(stage,root.GetPath().AppendChild(f'bend_{i}_joint'))
            joint.CreateBody0Rel().SetTargets([root.GetPath().AppendChild(parent)])
            joint.CreateBody1Rel().SetTargets([root.GetPath().AppendChild(child)])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(points[i]-centers[parent])))
            joint.CreateLocalPos1Attr(Gf.Vec3f(*(points[i]-centers[child])))
            joint.CreateLocalRot0Attr(orientation)
            joint.CreateLocalRot1Attr(orientation)
            prim = joint.GetPrim()
            for axis in ('transX','transY','transZ','rotX'):
                limit = UsdPhysics.LimitAPI.Apply(prim,axis)
                limit.CreateLowAttr(1.)
                limit.CreateHighAttr(-1.)
            for axis in ('rotY','rotZ'):
                limit = UsdPhysics.LimitAPI.Apply(prim,axis)
                limit.CreateLowAttr(-85.)
                limit.CreateHighAttr(85.)
                drive = UsdPhysics.DriveAPI.Apply(prim,axis)
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(0.)
                drive.CreateDampingAttr(0.)
                drive.CreateMaxForceAttr(2.)
        return layer.ExportToString()

    part = replace(original.part,build=build,pos=tuple(np.asarray(position)+base),
                   drives=tuple(drives),start_joints=(),limits=())
    props = dict(props,carrier_mass_fraction=0.,
                 mass_representation='uniform hollow cells; no virtual carrier',
                 joint_representation='D6 with two angular DOFs; new discretization requires validation')
    return Tube(part,points,original.bodies,rests,stiffnesses,props,masses,centers)


def energy(env,tube,height_origin=0.):
    asset = env.unwrapped.scene[tube.part.name]
    poses = m._np(asset.data.body_link_pose_w)[0]
    velocities = m._np(asset.data.body_com_vel_w)[0]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    kinetic = gravity = maximum_speed = 0.
    for name,values in tube.mass_properties.items():
        i = asset.body_names.index(name)
        rotation = m.quat_matrix(poses[i,3:])
        velocity,omega = velocities[i,:3],rotation.T@velocities[i,3:]
        kinetic += .5*values['mass']*(velocity@velocity)+.5*omega@values['inertia_matrix']@omega
        gravity += values['mass']*9.81*(poses[i,2]-origin[2]-height_origin)
        maximum_speed = max(maximum_speed,float(np.linalg.norm(velocity)))
    q = m.state(env,tube.part.name).joints
    bending = sum(.5*tube.joint_stiffness[n]*(q[n]-rest)**2
                  for n,rest in tube.rest_targets.items())
    return dict(kinetic_j=float(kinetic),gravity_j=float(gravity),
                bending_j=float(bending),total_j=float(kinetic+gravity+bending),
                maximum_com_speed_m_s=maximum_speed)
