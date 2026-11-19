"""Engineering screwdriver grip/orientation probe. No task score enabled."""
import pathlib, sys, numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
import simready as sr
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE='station'
TOP=TABLE_TOP[SCENE]
INSTRUCTION='Engineering tool feasibility probe.'
SUCCESS_HOLD_S=0.
_original_prepare=sr._prepare
def _fine_tool_contacts(prim,spec):
    _original_prepare(prim,spec)
    if spec.name=='driver':
        from pxr import Usd, UsdPhysics, Sdf
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                p.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0001)
                p.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
sr._prepare=_fine_tool_contacts
OBJECTS=[sr.SimReady(name='driver',pos=(.24,.18,TOP+.001),size=(.148,.027,.026),
    mass=.03647007420659065,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=True,scale=1.,yaw=-np.pi/2)]
class Trial:
    def reset(self,env=None):self.steps=0
    def step(self,env):
        self.steps+=1
        if self.steps==1 or self.steps%20==0:
            from yam_arena.scene import object_states
            d=object_states(env,['driver'])['driver']
            print('DRIVER_PROBE',self.steps,{'pos':d.pos.tolist(),'quat':d.quat_xyzw.tolist(),'low':d.low.tolist(),'high':d.high.tolist(),'finger_points':m.fingertips(env).tolist()},flush=True)
t=Trial();t.reset();m.on_reset(t.reset);m.on_step(t.step)
def score(objects):return 0.
def success(objects):return False
from dataclasses import dataclass
from yam_arena import demo as _orientation_demo, ik
@dataclass
class Oriented(Waypoint):
    approach: tuple = (0, 0, -1)
    closing: tuple = (0, 1, 0)

def rotation(w):
    if not isinstance(w, Oriented):
        return _orientation_demo.grip_rotation(w.yaw)
    z=np.asarray(w.approach,dtype=float); z/=np.linalg.norm(z)
    x=np.asarray(w.closing,dtype=float); x-=x.dot(z)*z; x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))

def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        g=np.asarray(w.pos,dtype=float); q=rotation(w)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label
        t-=w.duration; p,r,f=g,q,w.finger
    return p,r,f,'done'

_orientation_demo.ArmPlan.target=target

def demo():
    def w(t,p,f,label,a=(0,0,-1),c=(-1,0,0)):
        return Oriented(t,p,f,label,0,a,c)
    return {'left':[
        w(1.5,(.24,.195,TOP+.12),-.025,'hover handle'),
        w(1.5,(.24,.195,TOP+.016),-.025,'approach handle'),
        w(1.,(.24,.195,TOP+.016),0.,'grip handle'),
        w(2.,(.24,.195,TOP+.15),0.,'lift driver'),
        w(2.,(.24,.12,TOP+.35),0.,'turn tip up',(0,-1,0)),
        w(1.5,(.24,.12,TOP+.17),0.,'lower beside connector',(0,-1,0)),
        w(2.,(.30,.002,TOP+.175),0.,'tool service pose',(0,-1,0)),
        w(2.,(.30,.002,TOP+.175),0.,'hold tool',(0,-1,0))]}
