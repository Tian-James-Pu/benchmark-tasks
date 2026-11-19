"""Contact-only prototype of a utensil drawer jammed by an elevated ladle handle."""
import math
import pathlib
import sys
import numpy as np
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from simready import SimReady
SCENE='station'
TOP=TABLE_TOP[SCENE]
FRONT=.43
FLOOR=.07
LOOKS={'wood':m.Look(m.rgb(.80,.69,.51),roughness=.8),'white':m.Look(m.rgb(.93,.93,.9),roughness=.6),'steel':m.Look(m.rgb(.67,.68,.69),metallic=.8)}
def slab(name,lo,hi,look):
    lo,hi=np.asarray(lo),np.asarray(hi);size=hi-lo;centre=(lo+hi)/2
    return (name,m.box(size,centre),look),m.col_box(name+'_c',size,centre)
frame=[];fcols=[]
for spec in [('worktop',(.405,-.26,.18),(.86,.26,.21),'wood'),('rail',(.43,-.24,.125),(.46,.24,.18),'white'),('left',(.43,.232,0),(.85,.25,.20),'white'),('right',(.43,-.25,0),(.85,-.232,.20),'white'),('back',(.835,-.232,0),(.85,.232,.20),'white')]:
    v,c=slab(*spec);frame.append(v);fcols.append(c)
drawer=[];dcols=[]
for spec in [('bottom',(.448,-.22,.055),(.83,.22,FLOOR),'wood'),('front',(.43,-.25,.005),(.448,.25,.105),'white'),('sideL',(.448,.22,FLOOR),(.83,.232,.125),'wood'),('sideR',(.448,-.232,FLOOR),(.83,-.22,.125),'wood'),('rear',(.818,-.22,FLOOR),(.83,.22,.125),'wood'),('pull',(.383,-.14,.075),(.398,.14,.09),'steel'),('stemL',(.395,.11,.075),(.43,.125,.09),'steel'),('stemR',(.395,-.125,.075),(.43,-.11,.09),'steel')]:
    v,c=slab(*spec);drawer.append(v);dcols.append(c)
DRAWER=m.mechanism('cabinet',(.4,.44,TOP),[m.slider('drawer','X',-.36,0,drawer,dcols,mass=2.5,damping=8,friction=.15,gravity=True,com=(.64,0,.06),inertia=(.04,.04,.07))],LOOKS,frame_parts=frame,frame_colliders=fcols,friction=(.8,.6),solver_iters=64,yaw_deg=-90)
# A 31.2cm ladle rests bowl-down on the floor with its neck over a loose spoon.
# Its 30degree pitch is ordinary gravity/contact geometry, not a spring or joint.
a=math.radians(18)
R=np.array([[0,-math.cos(a),math.sin(a)],[1,0,0],[0,math.sin(a),math.cos(a)]])
from scipy.spatial.transform import Rotation
R=np.array([[0,1,0],[-1,0,0],[0,0,1]])@R
q=tuple(Rotation.from_matrix(R).as_quat())
OBJECTS=[DRAWER,
 SimReady('ladle',pos=(.4,-.255,TOP+FLOOR+0.011617933074693321),size=(.0942,.3122,.0963),mass=.17,package='kitchenware_ladle_d01',usd='sm_kitchenware_ladle_d01_01.usd',quat_xyzw=q),
 SimReady('spoon',pos=(.4,-.195,TOP+FLOOR),size=(.0518,.2397,.0277),mass=.06,yaw=-math.pi/2,package='kitchenware_spoon_k01',usd='sm_kitchenware_spoon_k01_01.usd'),
 SimReady('fork',pos=(.25,-.2,TOP+FLOOR),size=(.03,.20,.015),mass=.035,yaw=-math.pi/2,package='kitchenware_fork_a01',usd='sm_kitchenware_fork_a01_01.usd'),
]
SUCCESS_HOLD_S=3.0
_n=0
def score(objects):
    global _n
    _n+=1
    if _n%20==1:
        st=objects['cabinet'];print('DRAWER_PROBE',_n,st.joint('drawer'),[(n,np.asarray(objects[n].pos).round(4).tolist(),np.asarray(objects[n].quat_xyzw).round(4).tolist()) for n in ('ladle','spoon','fork') if n in objects],flush=True)
    return 0.
def success(objects):return False
from yam_arena import demo as _demo
class Orient(float):
    rot=np.eye(3)
_plain=getattr(_demo.grip_rotation,'_plain',_demo.grip_rotation)
def _rotation(yaw=0.):
    return yaw.rot if hasattr(yaw,'rot') else _plain(yaw)
_rotation._plain=_plain
_demo.grip_rotation=_rotation
PULL=Orient(0.)
PULL.rot=np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])
def demo():
    return {'right':[],'left':[
        Waypoint(2.5,(.34,.10,TOP+.14),-.025),
        Waypoint(2.,(.34,.07,TOP+.083),-.025,yaw=PULL),
        Waypoint(1.,(.34,.055,TOP+.083),-.025,yaw=PULL),
        Waypoint(1.,(.34,.055,TOP+.083),FINGER_CLOSED,yaw=PULL),
        Waypoint(2.,(.34,.12,TOP+.083),FINGER_CLOSED,yaw=PULL),
        Waypoint(1.,(.34,.12,TOP+.083),-.025,yaw=PULL),
        Waypoint(4.,(.34,.12,TOP+.10),FINGER_OPEN),
        Waypoint(1.5,(.30,.20,TOP+.14),FINGER_OPEN)]}

# Physics diagnostic only: constant 5N pull, never a success demonstration.
@m.on_step
def _force_probe(env):
    m.push(env,'cabinet',(0,5.,0),body='drawer')
def demo():return {'left':[],'right':[]}

OBJECTS=[o for o in OBJECTS if o.name in ("cabinet","fork")]
