"""Passive engaged-hook control; no robot repair or task credit."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
_original_scene_patch=m._Finder.targets['yam_arena.scene']
def fine_scene(scene):
    _original_scene_patch(scene);scene.PHYSICS_DT=1./960.
m._Finder.targets['yam_arena.scene']=fine_scene
from spring_scene_v3 import objects
from spring_mechanics_v3 import ExtensionSpring
from yam_arena.demo import Waypoint
SCENE='station'
INSTRUCTION='Inspect the guard return mechanism.'
SUCCESS_HOLD_S=0.
OBJECTS=objects(engaged=True)
SPRING=ExtensionSpring()
class Trial:
    def reset(self,env=None):self.steps=0
    def step(self,env):
        self.steps+=1
        if self.steps==1 or self.steps%5==0:print('SPRING_PASSIVE',self.steps,SPRING.last,flush=True)
trial=Trial();m.on_reset(trial.reset);m.on_step(trial.step)
def score(objects):return 0.
def success(objects):return False
def demo():return {'left':[Waypoint(3.,(.18,.28,.90),-.04695,'clear')], 'right':[Waypoint(3.,(.18,-.28,.90),-.04695,'clear')]}
