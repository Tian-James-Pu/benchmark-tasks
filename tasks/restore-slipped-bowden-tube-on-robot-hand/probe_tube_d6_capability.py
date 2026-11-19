"""Two-axis articulation API diagnostic, not a tube/hand validation.

A straight two-cell PTFE beam uses one D6 joint with locked translations and
twist. This checks actual joint names, commanded axes and finite motion before
attempting a carrier-free curved beam. It is not dynamically equivalent to
the previous two-hinge representation and does not count as a task demo.
"""
import pathlib
import sys
from dataclasses import replace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from compliant_tube import make_tube
from scaled_convex import scaled_convex_part
from physics_hooks import on_physics_step
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

_patch = m._Finder.targets['yam_arena.scene']
def physics_scene(scene):
    _patch(scene)
    scene.PHYSICS_DT = 1./480.
m._Finder.targets['yam_arena.scene'] = physics_scene
SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Inspect two independent bending axes in a tube section.'
SUCCESS_HOLD_S = 0.
C = np.array([.30, 0., TOP+.15])
POINTS = np.array([[0.,0.,0.],[0.,0.,.01],[0.,0.,.02]])
tube = make_tube('tube', C, POINTS, fixed_root=True)
original_build = tube.part.build
K = tube.properties['bending_stiffness_Nm_per_rad'][0]
RO, RI = .0015, .00075
MASS = 2150.*np.pi*(RO*RO-RI*RI)*.01
IT = MASS*(3*(RO*RO+RI*RI)+.01**2)/12
IA = MASS*(RO*RO+RI*RI)/2
DAMPING = 2*.2*np.sqrt(K*IT)

def build():
    from pxr import Usd, Sdf, UsdPhysics, Gf
    layer = Sdf.Layer.CreateAnonymous('.usda')
    if not layer.ImportFromString(original_build()):
        raise ValueError('Invalid original tube')
    stage = Usd.Stage.Open(layer)
    root = stage.GetDefaultPrim()
    for name in ('bend_1_joint','segment_1_joint','bend_1'):
        stage.RemovePrim(root.GetPath().AppendChild(name))
    mass = UsdPhysics.MassAPI(root.GetChild('segment_1'))
    mass.GetMassAttr().Set(float(MASS))
    mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(IT,IT,IA))
    joint = UsdPhysics.Joint.Define(stage, root.GetPath().AppendChild('bend_joint'))
    joint.CreateBody0Rel().SetTargets([root.GetPath().AppendChild('frame')])
    joint.CreateBody1Rel().SetTargets([root.GetPath().AppendChild('segment_1')])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.,0.,.01))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,.01))
    # Joint X is the tube axis (world Z); Y/Z are its two bending axes.
    q = Gf.Quatf(float(np.sqrt(.5)),Gf.Vec3f(0.,-np.sqrt(.5),0.))
    joint.CreateLocalRot0Attr(q)
    joint.CreateLocalRot1Attr(q)
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

tube.part = replace(tube.part,build=build,
    drives=(('.*',(K,float(DAMPING),2.,0.,0.)),),start_joints=(),limits=())
tube.part = scaled_convex_part(tube.part)
OBJECTS = [tube.part]

class Trial:
    def reset(self,env=None):
        self.active = env is not None
        self.time = 0.
        self.steps = 0
        self.target = None
    def physics(self,env,dt):
        if not self.active:
            return
        self.time += dt
        asset = env.unwrapped.scene['tube']
        if self.target is None:
            import torch
            print('D6_CAPABILITY_NAMES',asset.joint_names,asset.body_names,flush=True)
            if asset.num_joints != 2:
                raise RuntimeError('Expected exactly two angular degrees of freedom')
            self.target = torch.zeros((1,2),device=asset.device)
        self.target[:] = 0.
        if self.time >= .25:
            self.target[0,0] = .02
        if self.time >= .5:
            self.target[0,1] = -.03
        asset.set_joint_position_target(self.target)
    def step(self,env):
        self.steps += 1
        asset = env.unwrapped.scene['tube']
        state = m.state(env,'tube')
        values = m._np(asset.data.body_link_pose_w)[0]
        print('D6_CAPABILITY_STATE',self.steps,{
            'time_s':self.time,'joints':state.joints,
            'targets':m._np(self.target).tolist() if self.target is not None else None,
            'body_poses':values.tolist(),'finite':bool(np.isfinite(values).all()),
            'material_stiffness_nm_per_rad':K,'segment_mass_kg':MASS},flush=True)

trial = Trial()
trial.reset()
m.on_reset(trial.reset)
on_physics_step(trial.physics)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right':[Waypoint(1.,(.20,-.25,TOP+.22),-.04695,'idle joint diagnostic',0.)]}
