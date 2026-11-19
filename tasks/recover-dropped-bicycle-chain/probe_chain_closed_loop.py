"""Commission closed-chain physics at full pitch and chainstay length; score zero.

This is an explicitly incomplete planar test fixture, NOT the bicycle task.
Rear cage dimensions, spring and pin response are uncalibrated assumptions.
Frame, front cage, extra gears, offset chainlines, freehub, dropped starting
fault and native robot manipulation are not represented by this diagnostic.
"""
from pathlib import Path
import sys
import math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as m
from chain_geometry import Wheel, Route, standard_sprocket_polygon, ROLLER_RADIUS
from chain_mechanics import chain_assembly, PinResponse
from scaled_convex import scaled_convex_part
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Get the bicycle ready to pedal.'
SUCCESS_HOLD_S = 0.
ORIGIN = np.array([.32,.22,TOP+.172])
_patch = m._Finder.targets['yam_arena.scene']
def fine_scene(scene):
    _patch(scene)
    scene.PHYSICS_DT = 1./480.
m._Finder.targets['yam_arena.scene'] = fine_scene

rear = np.array([-math.sqrt(.425**2-.066**2),.066])
guide = rear + [.012,-.060]
cage_angle = math.radians(-48.489767236827944)
tension = guide + .075*np.array([math.sin(cage_angle),-math.cos(cage_angle)])
route = Route([Wheel('front', (0.,0.),22),Wheel('rear',tuple(rear),11),
               Wheel('guide',tuple(guide),13,-1),Wheel('tension',tuple(tension),15)])
pins, residual = route.pins(106)
if abs(residual)>1e-8:
    raise ValueError('Chain does not close')
response = PinResponse(1.,.01,.00001,'Uncalibrated first physics commissioning; not a selected task response')
CHAIN, CHAIN_BODIES = chain_assembly('chain',ORIGIN,pins[:,::-1],0.,response)
LOOKS = {'steel':m.Look(color=m.rgb(.45,.47,.5),metallic=.8,roughness=.4),
         'cage':m.Look(color=m.rgb(.18,.2,.22),metallic=.7,roughness=.45)}

def fine(c):
    return c.replace('contactOffset = 0.004','contactOffset = 0.00005')

def sprocket(index):
    wheel = route.wheels[index]
    relative = pins-wheel.center
    wrapped = np.abs(np.linalg.norm(relative,axis=1)-wheel.radius)<1e-7
    angles=np.arctan2(relative[wrapped,1],relative[wrapped,0])
    phase=math.atan2(np.sin(angles*wheel.teeth).mean(),np.cos(angles*wheel.teeth).mean())/wheel.teeth
    from convex_partition import partition
    from shapely.geometry import Point
    from shapely.geometry.polygon import orient
    polygon=standard_sprocket_polygon(wheel.teeth,phase=math.pi/2-phase,arc_samples=12)
    center=np.r_[np.asarray(wheel.center)[::-1],0.]
    # A bored test sprocket; complete manufacturer spider/shift features are
    # deliberately outside this first constraint/contact commissioning.
    gaps=[polygon.distance(Point(*(p[::-1]-center[:2])))-ROLLER_RADIUS for p in pins]
    if min(gaps)<-.00001:
        raise ValueError('Initial roller-tooth interpenetration exceeds mesh discretization bound')
    pieces, partition_audit=partition(polygon)
    print('SPROCKET_PARTITION',wheel.name,partition_audit,flush=True)
    shapes=[m.prism(np.array(orient(t,sign=1.).exterior.coords)[:-1],-.0009,.0009).moved(center)
            for t in pieces]
    visual=m.join(*shapes)
    cols=[fine(m.col_hull('tooth_'+str(i),s)) for i,s in enumerate(shapes)]
    return center,[('sprocket',visual,'steel')],cols

links=[]
for i,name in ((0,'crank'),(1,'rear'),(2,'guide'),(3,'tension')):
    center,parts,cols=sprocket(i)
    mass=.06 if i<2 else .018
    r=route.wheels[i].radius
    links.append(m.hinge(name,'Z',tuple(center),-float('inf'),float('inf'),parts,cols,
                         mass=mass,spring=0.,damping=.003 if i==0 else .00002,
                         friction=0.,effort=.02,armature=0.,gravity=True,com=tuple(center),
                         parent='cage' if i>=2 else None,
                         inertia=(mass*r*r/4,mass*r*r/4,mass*r*r/2)))

gp=np.r_[guide[::-1],0.];tp=np.r_[tension[::-1],0.]
direction=tp-gp
angle=math.degrees(math.atan2(direction[1],direction[0]))
parts=[];cols=[]
for side in (-1,1):
    shape=m.box((.087,.010,.0015)).turned(angle,'z').moved((gp+tp)/2+[0,0,side*.005])
    parts.append(('cage_side_'+str(side).replace('-','n'),shape,'cage'))
    cols.append(fine(m.col_hull('cage_c_'+str(side).replace('-','n'),shape)))
cage=m.hinge('cage','Z',tuple(gp),-70.,70.,parts,cols,mass=.04,
             spring=.6,damping=.005,friction=0.,effort=1.,armature=0.,gravity=True,
             com=tuple((gp+tp)/2),inertia=(3e-5,3e-5,6e-5))
DRIVE=m.mechanism('drive',tuple(ORIGIN),[cage,*links],LOOKS,solver_iters=64,
                  friction=(.15,.10))
DRIVE=scaled_convex_part(DRIVE)
OBJECTS=[CHAIN,DRIVE]

class Observe:
    def __init__(self):
        self.view=None;self.step=0
    def reset(self,env):
        self.step=0
        m.set_target(env,'drive','cage',math.radians(15.))
        import torch
        asset=env.unwrapped.scene['drive']
        index=asset.joint_names.index('crank_joint')
        target=torch.tensor([[-1.]],device=asset.device)
        asset.set_joint_velocity_target(target,joint_ids=[index])
    def observe(self,env):
        if self.view is None:
            import omni.physics.tensors as tensors
            self.sim=tensors.create_simulation_view('torch')
            self.sim.set_subspace_roots('/')
            self.view=self.sim.create_rigid_body_view('/World/envs/env_0/chain/*')
        self.step+=1
        if self.step==1 or self.step%10==0:
            poses=m._np(self.view.get_transforms())
            velocities=m._np(self.view.get_velocities())
            state=m.state(env,'drive')
            print('CHAIN_STATE',self.step,dict(rigid_bodies=len(poses),finite=bool(np.isfinite(poses).all()),
                 bounds_m=[poses[:,:3].min(0).tolist(),poses[:,:3].max(0).tolist()],
                 maximum_linear_speed_m_s=float(np.linalg.norm(velocities[:,:3],axis=1).max()),
                 drive_joint_rad=state.joints),flush=True)
observer=Observe()
m.on_reset(observer.reset)
m.on_step(observer.observe)
def score(objects):return 0.
def success(objects):return False
def demo():
    return {'left':[Waypoint(3.,(.20,.3,TOP+.25),-.04695,'idle mechanism commissioning',0.)]}
