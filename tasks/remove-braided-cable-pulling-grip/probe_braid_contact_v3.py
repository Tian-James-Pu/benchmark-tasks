"""Engineering coupon: inferred elastic-wire braid contact compression.

Version3 retains corrected anchors and tests native D6 elastic rod joints.
NOT a task or feasibility demonstration. Score is always zero. Both wire ends
are clamped to a test fixture and a driven annular platen compresses the mouth.
No object pose, braid radius or cable-release flag changes during simulation.
Strand dimensions are inferred. Seven freely slipping steel filaments provide
an uncalibrated lower-bound bending model; the authored curved shape is the
stress-free reference for this coupon, not measured commercial preload. The manufactured end-loop shape remains a
geometry approximation. No credit, media submission or robot validation follows
from this diagnostic alone.
"""
import json
import math
import pathlib
import sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_scene_patch = m._Finder.targets['yam_arena.scene']
def fine_physics(scene):
    original_scene_patch(scene)
    scene.PHYSICS_DT = 1/1200
m._Finder.targets['yam_arena.scene'] = fine_physics

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.
INSTRUCTION = 'Observe the engineering braid compression coupon.'
C = np.array([.30, 0, TOP+.15])
RADIUS = .00025
CONTACT_OFFSET = .00002
DENSITY = 7850.
YOUNG = 200e9
FILAMENT_RADIUS = RADIUS/3
WIRE_AREA = 7*math.pi*FILAMENT_RADIUS**2
BEND_EI = 7*YOUNG*math.pi*FILAMENT_RADIUS**4/4
TORSION_GJ = BEND_EI/(1+.3)
data = np.load(pathlib.Path(__file__).with_name('inferred-braid-centrelines.npz'))
PATHS = [data[k].copy() for k in sorted(data.files)]
LOOKS = {'steel': m.Look(color=m.rgb(.65,.67,.69),metallic=.8,roughness=.4),
         'cable': m.Look(color=m.rgb(.08,.09,.10),roughness=.7),
         'fixture': m.Look(color=m.rgb(.25,.45,.65),metallic=.3)}


def build_braid():
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, '/P_braid')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    from pxr import UsdShade
    material = UsdShade.Material.Define(stage, '/P_braid/material')
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(.4)
    mat.CreateDynamicFrictionAttr(.3)
    mat.CreateRestitutionAttr(0.)
    for wi, points in enumerate(PATHS):
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        arc = np.r_[0, np.cumsum(lengths)]
        mids = (arc[:-1]+arc[1:])/2
        names = [f'/P_braid/w{wi}_s{i:04d}' for i in range(len(lengths))]
        rotations = []
        for si, (a,b,length) in enumerate(zip(points[:-1], points[1:], lengths)):
            path = names[si]
            direction = (b-a)/length
            # Frame X is the segment axis; the other axes only affect inertia.
            helper = np.array([0.,0.,1.]) if abs(direction[2]) < .9 else np.array([0.,1.,0.])
            z = np.cross(direction, helper); z /= np.linalg.norm(z)
            y = np.cross(z, direction)
            rot = np.column_stack((direction,y,z)); rotations.append(rot)
            q = Rotation.from_matrix(rot).as_quat()
            body = UsdGeom.Xform.Define(stage,path)
            body.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
            body.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
            prim = body.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            mass = DENSITY*WIRE_AREA*length
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr(float(mass))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0))
            inertia = (mass*RADIUS**2/2, mass*(3*RADIUS**2+length**2)/12,
                       mass*(3*RADIUS**2+length**2)/12)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
            prim.AddAppliedSchema('PhysxRigidBodyAPI')
            prim.AddAppliedSchema('PhysxContactReportAPI')
            for name,value,kind in [
                ('physxRigidBody:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int),
                ('physxRigidBody:solverVelocityIterationCount',8,Sdf.ValueTypeNames.Int),
                ('physxRigidBody:maxDepenetrationVelocity',.05,Sdf.ValueTypeNames.Float),
                ('physxRigidBody:linearDamping',0.,Sdf.ValueTypeNames.Float),
                ('physxRigidBody:angularDamping',0.,Sdf.ValueTypeNames.Float),
                ('physxContactReport:threshold',0.,Sdf.ValueTypeNames.Float)]:
                prim.CreateAttribute(name,kind).Set(value)
            cap = UsdGeom.Capsule.Define(stage,path+'/wire')
            cap.CreateAxisAttr('X'); cap.CreateRadiusAttr(RADIUS)
            # Segment endpoints are capsule sphere centres, matching the audit.
            cap.CreateHeightAttr(float(length))
            cap.CreateDisplayColorAttr([Gf.Vec3f(.45,.47,.49)])
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            cap.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
            cap.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(CONTACT_OFFSET)
            cap.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
            UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(material, materialPurpose='physics')
            close = np.where(np.abs(mids-mids[si]) <= .0015)[0]
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
                [Sdf.Path(names[j]) for j in close if j != si])
        # Elastic rod joints retain actual segment mass/inertia. Rest frames
        # agree in the authored shape; no runtime rest-angle/radius adjustment.
        for si in range(1,len(lengths)):
            joint = UsdPhysics.Joint.Define(stage,f'/P_braid/j{wi}_{si:04d}')
            joint.CreateBody0Rel().SetTargets([Sdf.Path(names[si-1])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(names[si])])
            joint.CreateLocalPos0Attr(Gf.Vec3f(float(lengths[si-1]/2),0,0))
            joint.CreateLocalPos1Attr(Gf.Vec3f(float(-lengths[si]/2),0,0))
            relative = Rotation.from_matrix(rotations[si].T@rotations[si-1]).as_quat()
            joint.CreateLocalRot0Attr(Gf.Quatf(1,0,0,0))
            joint.CreateLocalRot1Attr(Gf.Quatf(float(relative[3]),Gf.Vec3f(*relative[:3])))
            joint.CreateCollisionEnabledAttr(False)
            prim = joint.GetPrim()
            for dof in ('transX','transY','transZ'):
                limit = UsdPhysics.LimitAPI.Apply(prim,dof)
                limit.CreateLowAttr(1.); limit.CreateHighAttr(-1.)
            mean_length = (lengths[si-1]+lengths[si])/2
            for axis,modulus in [('rotX',TORSION_GJ),('rotY',BEND_EI),('rotZ',BEND_EI)]:
                stiffness = modulus/mean_length
                inertias = []
                for length in lengths[si-1:si+1]:
                    mass = DENSITY*WIRE_AREA*length
                    inertias.append(mass*RADIUS**2/2 if axis=='rotX' else
                                    mass*(3*RADIUS**2+length**2)/12)
                effective_inertia = 1/(1/inertias[0]+1/inertias[1])
                damping = .4*math.sqrt(stiffness*effective_inertia)
                drive = UsdPhysics.DriveAPI.Apply(prim,axis)
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(float(stiffness*math.pi/180))
                drive.CreateDampingAttr(float(damping*math.pi/180))
                drive.CreateTargetPositionAttr(0.)
                drive.CreateMaxForceAttr(.1)
        # Both crimp ends are fixed to the world in this coupon only.
        for label,si,sign,point in [('a',0,-1,points[0]),('b',len(lengths)-1,1,points[-1])]:
            joint = UsdPhysics.FixedJoint.Define(stage,f'/P_braid/clamp_{wi}_{label}')
            joint.CreateBody1Rel().SetTargets([Sdf.Path(names[si])])
            joint.CreateLocalPos1Attr(Gf.Vec3f(float(sign*lengths[si]/2),0,0))
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(C+point)))
            q = Rotation.from_matrix(rotations[si]).as_quat()
            joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    return stage.GetRootLayer().ExportToString()


braid = m.Part('braid',pos=tuple(C),size=(.11,.02,.02),role='fixed',build=build_braid)
def fine_col(text): return text.replace('contactOffset = 0.004','contactOffset = 0.00002')
# A 13mm rigid mandrel isolates braid mechanics; it is not a finished cable model.
mandrel_mesh = m.cylinder(.0065,.31,(.195,0,0),axis='x')
mandrel_collider = f'''
def Cylinder "mandrel_c" {fine_col(m._COL_HEAD)}
    uniform token axis = "X"
    double radius = .0065
    double height = .31
    double3 xformOp:translate = (.195, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}}
'''
mandrel = m.fixed('mandrel',tuple(C),[('mandrel',mandrel_mesh,'cable')],
                 [mandrel_collider],LOOKS,friction=(.4,.3))
parts,cols = [],[]
for i in range(24):
    a,b = 2*math.pi*i/24,2*math.pi*(i+1)/24
    yz = [(r*math.cos(t),r*math.sin(t)) for r,t in [(.00665,a),(.012,a),(.012,b),(.00665,b)]]
    mesh = m.prism(yz,-.001,.001)
    mesh = m.Mesh(mesh.v[:,[2,0,1]]+[.412-C[0],0,0],mesh.f)
    parts.append((f'ring{i}',mesh,'fixture'));cols.append(fine_col(m.col_hull(f'ring{i}_c',mesh)))
platen = m.mechanism('platen',tuple(C),[m.slider('travel','X',-.030,.003,parts,cols,
    mass=.02, spring=100.,damping=.1,effort=3.,armature=0.,gravity=False,
    com=(.112,0,0),inertia=(1.8e-6,9e-7,9e-7))],LOOKS,solver_iters=64,friction=(.3,.25))
OBJECTS = [braid,mandrel,platen]
VIEW = None
STEP = 0


def reset(env=None):
    global VIEW,STEP
    VIEW = None; STEP = 0


def observe(env):
    global VIEW,STEP
    STEP += 1
    from isaaclab_physx.physics import PhysxManager
    if VIEW is None:
        VIEW = PhysxManager.get_physics_sim_view().create_rigid_body_view('/World/envs/env_0/braid/w*')
    q = VIEW.get_transforms()
    q = np.asarray(q.numpy() if hasattr(q,'numpy') else q)
    if q.shape != (sum(len(p)-1 for p in PATHS),7) or not np.isfinite(q).all():
        raise RuntimeError(f'Invalid braid state {q.shape}')
    local = q[:,:3]-C
    if local[:,0].min() < -.02 or local[:,0].max() > .20:
        raise RuntimeError(f'Braid escaped coupon axial bounds: {local[:,0].min()}..{local[:,0].max()}')
    target = -.025*min(max((STEP-20)/40,0),1)
    m.set_target(env,'platen','travel',target)
    if STEP == 1 or STEP % 5 == 0:
        p = m.state(env,'platen')
        print('BRAID_COUPON',STEP,json.dumps({'target_m':target,'platen_m':p.joint('travel'),
              'max_x_m':float(local[:,0].max()),'min_x_m':float(local[:,0].min()),
              'mean_radius_last_20mm_m':float(np.linalg.norm(local[local[:,0]>.085,1:],axis=1).mean())
                  if np.any(local[:,0]>.085) else None,'bodies':len(q),'bend_ei_Nm2':BEND_EI,'segment_mass_model':'7filament steel, inferred'}),flush=True)


m.on_reset(reset); m.on_step(observe)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right':[Waypoint(6.,(.20,-.25,TOP+.22),-.04695,'idle engineering fixture',0.)]}
