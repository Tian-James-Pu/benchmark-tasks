"""Full-scale source-asset guard reach check. No repaired-task score exists."""
import pathlib
import sys
import math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from asset import build
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

SCENE='station'
TOP=TABLE_TOP[SCENE]
INSTRUCTION='Inspect the saw guard.'
SUCCESS_HOLD_S=0.
C=np.array([.23,0.,TOP+.142])
# Rest on the motor housing's side. No dimensions or grasp features enlarged.
Q=(0.,math.sqrt(.5),0.,math.sqrt(.5))
OBJECTS=[m.Part('saw',pos=tuple(C),size=(.287,.350,.235),role='jointed',
    build=build,start_quat=Q,
    drives=(('blade_spin',(0.,.002,9.,0.,0.)),
            ('guard_pivot',(.3,.03,4.,0.,0.)),
            ('trigger_travel',(200.,.8,40.,0.,0.))))]

class Trial:
    def reset(self,env=None):self.steps=0
    def step(self,env):
        self.steps+=1
        if self.steps==1 or self.steps%10==0:
            s=m.state(env,'saw')
            print('SAW_GUARD_REACH',self.steps,{'pos':s.pos.tolist(),
                'quat':s.quat_xyzw.tolist(),'joints':s.joints},flush=True)
trial=Trial()
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    z=TOP+.214
    return {'right':[Waypoint(2.,(.18,-.15,z),-.04695,'approach guard',0.),
                     Waypoint(2.,(.215,-.070,z),-.04695,'guard reach',0.),
                     Waypoint(1.,(.215,-.070,z),-.008,'pinch existing rim',0.),
                     Waypoint(2.,(.248,-.095,z),-.008,'move guard rim',.35),
                     Waypoint(1.,(.248,-.095,z),-.04695,'release',.35),
                     Waypoint(2.,(.20,-.25,z),-.04695,'clear',0.)]}
