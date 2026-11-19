"""SR-045 scale gate: grip a2.4mm wire coupon. NOT a benchmark task.

This straight70mm coupon is rigid solely to isolate pad/diameter contact.
It does not validate flexible harness behavior, contact seating or retention.
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
OBJECTS=[m.free('wire',(X,Y,Z),[('wire',mesh,'insulation')],[col],look,mass=.0015,friction=(.65,.50),solver_iters=64)]
class Trial:
 def reset(self,env=None):self.steps=0
 def step(self,env):
  self.steps+=1
  if self.steps in (1,40,80,120,160,200,240,280):
   s=m.state(env,'wire');print('WIRE_PROBE',self.steps,s.pos.tolist(),s.quat_xyzw.tolist(),flush=True)
t=Trial();t.reset();m.on_reset(t.reset);m.on_step(t.step)
def score(objects):return 0.
def success(objects):return False
def demo():
 w=lambda d,p,f,label:Waypoint(d,p,f,label,math.pi/2)
 return {'right':[w(2,(X,Y,TOP+.10),-.012,'hover'),w(2,(X,Y,TOP+.0128),-.012,'approach wire'),w(2,(X,Y,TOP+.0128),0.,'pinch'),w(3,(X,Y,TOP+.10),0.,'lift'),w(3,(X,Y,TOP+.10),0.,'hold'),w(1,(X,Y,TOP+.10),-.012,'release'),w(1,(X,Y,TOP+.15),-.012,'withdraw')]}
