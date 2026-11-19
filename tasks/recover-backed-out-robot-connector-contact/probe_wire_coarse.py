"""SR-045 segmented wire contact probe. NOT a benchmark task.

This120mm articulated wire probes grasping with bending compliance.
Its spring approximation is not calibrated and does not validate the final harness.
No score is enabled. No gripper attachment or artificial support is used.
"""
import pathlib,sys,math
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN
SCENE='station'; TOP=TABLE_TOP[SCENE]; SUCCESS_HOLD_S=0.
INSTRUCTION='Grasp and lift the short wire coupon.'
X=.30;Y=-.09;R=.0012;Z=TOP+R
look={'insulation':m.Look(color=m.rgb(.8,.08,.05),roughness=.65)}
mesh=m.cylinder(R,.070,(0,0,0),axis='y')
col=m.col_hull('wire_c',mesh).replace('contactOffset = 0.004','contactOffset = 0.0001')
# Four30mm segments with alternating flexion axes. Bending stiffness is
# provisional and deliberately varied in separate trials; it is not calibrated.
L=.030; SEG_MASS=.000645
links=[]
for i in range(1,4):
 shape=m.cylinder(R,L,(0,i*L,0),axis='y')
 col=m.col_hull(f'segment{i}_c',shape).replace('contactOffset = 0.004','contactOffset = 0.0001')
 links.append(m.hinge(f'segment{i}','X' if i%2 else 'Z',(0,(i-.5)*L,0),-80,80,
  [(f'wire{i}',shape,'insulation')],[col],mass=SEG_MASS,spring=.01,damping=.00005,
  friction=0,effort=.15,armature=1e-9,gravity=True,com=(0,i*L,0),
  inertia=(4.9e-8,4.6e-10,4.9e-8),parent=f'segment{i-1}' if i>1 else None))
root=m.cylinder(R,L,(0,0,0),axis='y')
OBJECTS=[m.mechanism('wire',(X,Y,Z),links,look,frame_parts=[('wire0',root,'insulation')],
 frame_colliders=[m.col_hull('root_c',root).replace('contactOffset = 0.004','contactOffset = 0.0001')],
 free_mass=SEG_MASS,friction=(.65,.50),solver_iters=64,frame_inertia=(4.9e-8,4.6e-10,4.9e-8))]

class Trial:
 def reset(self,env=None):self.steps=0
 def step(self,env):
  self.steps+=1
  if self.steps in (1,40,80,120,160,200,240,280):
   s=m.state(env,'wire');print('WIRE_PROBE',self.steps,s.pos.tolist(),s.quat_xyzw.tolist(),{k:round(v,5) for k,v in s.joints.items()},flush=True)
t=Trial();t.reset();m.on_reset(t.reset);m.on_step(t.step)
def score(objects):return 0.
def success(objects):return False
def demo():
 w=lambda d,p,f,label:Waypoint(d,p,f,label,math.pi/2)
 return {'right':[w(2,(X,Y,TOP+.10),-.012,'hover'),w(2,(X,Y,TOP+.0128),-.012,'approach wire'),w(2,(X,Y,TOP+.0128),-.0011,'pinch'),w(3,(X,Y,TOP+.10),-.0011,'lift'),w(3,(X,Y,TOP+.10),-.0011,'hold'),w(1,(X,Y,TOP+.10),-.012,'release'),w(1,(X,Y,TOP+.15),-.012,'withdraw')]}
