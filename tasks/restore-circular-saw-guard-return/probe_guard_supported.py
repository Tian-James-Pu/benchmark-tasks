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
C=np.array([.20,-.05,TOP+.142])
# Rest on the motor housing's side. No dimensions or grasp features enlarged.
Q=(-.5,.5,.5,.5)
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
    return {'left':[Waypoint(2.,(.16,.17,TOP+.185),-.04695,'approach actual handle',0.),
                    Waypoint(2.,(.16,.17,TOP+.155),-.04695,'handle reach',0.),
                    Waypoint(1.,(.16,.17,TOP+.155),-.018,'hold saw by handle',0.),
                    Waypoint(12.,(.16,.17,TOP+.155),-.018,'support guard check',0.),
                    Waypoint(1.,(.16,.17,TOP+.155),-.04695,'release handle',0.),
                    Waypoint(2.,(.18,.25,TOP+.19),-.04695,'clear',0.)],
            'right':[Waypoint(5.,(.15,-.20,z),-.04695,'wait for support',math.pi/2),
                     Waypoint(2.,(.13,-.13,z),-.04695,'approach guard side',math.pi/2),
                     Waypoint(2.,(.13,-.065,z),-.04695,'guard reach',math.pi/2),
                     Waypoint(1.,(.13,-.065,z),-.008,'pinch existing rim',math.pi/2),
                     Waypoint(3.,(.162,-.084,z),-.008,'move guard rim',math.pi/2+.35),
                     Waypoint(1.,(.162,-.084,z),-.04695,'release',math.pi/2+.35),
                     Waypoint(3.,(.15,-.20,z),-.04695,'clear',math.pi/2)]}
