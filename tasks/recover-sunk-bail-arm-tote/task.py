"""Commercial400mm swing-bar stack/nest container recovery, contact prototype."""
import pathlib,sys,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from simready import SimReady
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED
TOP=TABLE_TOP['station'];SCENE='station';SUCCESS_HOLD_S=0.
INSTRUCTION='Make the two produce totes a stable stack, keeping their contents in the totes.'
LOOK={'crate':m.Look(color=m.rgb(.12,.32,.65)),'bar':m.Look(color=m.rgb(.09,.10,.11))}
L=.400;W=.300;H=.135;C=(.38,0,TOP)
v=np.load(pathlib.Path(__file__).with_name('crate_visual.npz'))
VV=v['vertices'].copy();z=VV[:,2]/H
VV[:,0]*=(.350+.050*z)/(.364+.036*z);VV[:,1]*=(.250+.050*z)/(.264+.036*z)
VIS=m.Mesh(VV,v['faces'])
def hull(name,mesh):return m.col_hull(name.replace('-','m').replace('.','p'),mesh).replace('contactOffset = 0.004','contactOffset = 0.001')
def panel(axis,side):
 # Tapered load-bearing side wall, no invented grasp handle.
 vertices=[]
 for zz in (.008,.119):
  xx=.175+.018*zz/H;yy=.125+.018*zz/H
  if axis==0:
   for a,b in ((side*xx,-yy),(side*(xx-.003),-yy),(side*(xx-.003),yy),(side*xx,yy)):vertices.append((a,b,zz))
  else:
   for a,b in ((-xx,side*yy),(-xx,side*(yy-.003)),(xx,side*(yy-.003)),(xx,side*yy)):vertices.append((a,b,zz))
 return m.Mesh(np.array(vertices),np.array([[0,1,2],[0,2,3],[4,7,6],[4,6,5],[0,4,5],[0,5,1],[1,5,6],[1,6,2],[2,6,7],[2,7,3],[3,7,4],[3,4,0]]))
def tote(name,pos,quat=None,fault=False):
 parts=[('vented_shell',VIS,'crate')];cols=[]
 for axis in (0,1):
  for side in (-1,1):cols.append(hull(f'wall{axis}_{side}',panel(axis,side)))
 cols.append(m.col_box('floor',(.344,.244,.007),(0,0,.006)))
 for side in (-1,1):
  # Reinforced rim reproduces the commercial rim, no added grip tabs.
  for tag,size,center in [('end',(.015,.300,.014),(side*.1925,0,.128)),('side',(.370,.015,.014),(0,side*.1425,.128))]:
   shape=m.box(size,center);parts.append((f'rim{tag}{side}',shape,'crate'));cols.append(hull(f'rimc{tag}{side}',shape))
  for yy in (-.115,.115):
   shape=m.box((.025,.018,.007),(side*.146,yy,.0035));parts.append((f'foot{side}{yy}',shape,'crate'));cols.append(hull(f'footc{side}{yy}',shape))
 links=[]
 for side,n in ((-1,'near'),(1,'far')):
  px=side*.183;bx=side*(.223 if fault and side<0 else .143);pz=.123
  pts=np.array([[px,-.146,pz],[bx,-.146,pz],[bx,.146,pz],[px,.146,pz]])
  shapes=[];coll=[]
  for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
   sh=m.tube([a,b],.005,12);shapes.append((f'bar{i}',sh,'bar'));coll.append(hull(f'barc{i}',sh))
  links.append(m.hinge(n,'Y',(px,0,pz),0 if fault and side<0 else (-180 if side<0 else 0),180 if fault and side<0 else (0 if side<0 else 180),shapes,coll,mass=.055,gravity=True,friction=.001,damping=.015,armature=.0001,inertia=(.0004,.0001,.0004)))
 starts=[('near',0),('far',0)]
 return m.mechanism(name,pos,links,LOOK,frame_parts=[(f'frame_{i}',p[1],p[2]) for i,p in enumerate(parts)],frame_colliders=cols,free_mass=.710,friction=(.45,.35),solver_iters=64,start=starts,start_quat=quat,frame_inertia=(.0016,.0024,.0036))
a=math.radians(-8)
LOWER=tote('lower',C,fault=True)
UPPER=tote('upper',(.38,0,TOP+.107),quat=(0,math.sin(a/2),0,math.cos(a/2)))
OBJECTS=[LOWER,UPPER]
# Grocery contents and worktop distractors are actual SimReady props.
for i,(xx,yy) in enumerate(((.28,-.055),(.28,.055),(.45,-.055),(.45,.055))):
 OBJECTS.append(SimReady(f'lower_apple{i}',pos=(xx,yy,TOP+.01),size=(.075,.075,.075),mass=.18,package='food_produce_apple_b01',usd='sm_food_produce_apple_b01_01.usd'))
for i,(xx,yy) in enumerate(((.31,-.055),(.31,.055),(.44,-.055),(.44,.055))):
 OBJECTS.append(SimReady(f'upper_apple{i}',pos=(xx,yy,TOP+.14),size=(.075,.075,.075),mass=.18,package='food_produce_apple_b01',usd='sm_food_produce_apple_b01_01.usd'))
OBJECTS +=[SimReady('tape',pos=(.60,-.26,TOP),size=(.1,.1,.05),mass=.06,package='tape_clear_a01',usd='sm_tape_clear_a01_01.usd'),SimReady('mug',pos=(.60,.26,TOP),size=(.114,.09,.08),mass=.3,package='dishware_mug_a01',usd='sm_dishware_mug_a01_01.usda')]
class Trial:
 def reset(self,env=None):self.steps=0
 def step(self,env):
  self.steps+=1
  if self.steps in (1,20,40,80):
   print('TOTE_STATE',self.steps,{n:(m.state(env,n).pos.tolist(),m.state(env,n).quat_xyzw.tolist(),m.state(env,n).joints) for n in ('lower','upper')},flush=True)
T=Trial();T.reset();m.on_reset(T.reset);m.on_step(T.step)
def score(objects):return 0.
def success(objects):return False
def demo():return {}
