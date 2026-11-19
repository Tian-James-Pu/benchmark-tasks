"""Physical pick recovery attempt. Scoring remains disabled until validated."""
import pathlib,sys,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
_original_scene_patch=m._Finder.targets['yam_arena.scene']
def fine_scene(scene):
    _original_scene_patch(scene);scene.PHYSICS_DT=1./960.
m._Finder.targets['yam_arena.scene']=fine_scene
from spring_scene_v6 import objects
from spring_mechanics_v7 import ExtensionSpring
from service_pick import make_pick
from oriented_demo_v2 import tilted,tcp_for_tip
from yam_arena.demo import Waypoint
SCENE='station';TOP=.7475
INSTRUCTION='Restore this saw guard to working condition.'
SUCCESS_HOLD_S=0.
OBJECTS=objects(False)+[make_pick()]
# Preserve millimetre-scale tool collision margins after free-object spawning.
_orig=m._arena_object
def with_tool_contacts(spec):
    obj=_orig(spec)
    if spec.name=='service_pick':
        import isaaclab.sim as su
        obj.spawn_cfg_addon['collision_props']=su.CollisionPropertiesCfg(contact_offset=.0001,rest_offset=0.,torsional_patch_radius=.003,min_torsional_patch_radius=.002)
    return obj
m._arena_object=with_tool_contacts
SPRING=ExtensionSpring()
import spring_render
from spring_scene import C,Q
hook_spec=OBJECTS[1]
a=math.radians(hook_spec.yaw_deg)
hr=np.array([[math.cos(a),-math.sin(a),0.],[math.sin(a),math.cos(a),0.],[0.,0.,1.]])
OBJECTS.append(spring_render.part(spring_render.points(C,m.quat_matrix(Q),np.array(hook_spec.pos),hr)))
SPRING_RENDER=spring_render.Render(SPRING)
class Trial:
    def reset(self,env=None):self.steps=0
    def step(self,env):
        self.steps+=1
        if self.steps==1 or self.steps%10==0:
            s=m.state(env,'service_pick')
            print('PICK_RECOVERY',self.steps,dict(SPRING.last,pick_pos=s.pos.tolist(),pick_quat=s.quat_xyzw.tolist()),flush=True)
trial=Trial();m.on_reset(trial.reset);m.on_step(trial.step)
def score(objects):return 0.
def success(objects):return False

def demo():
    z=TOP+.0145
    return {'left':[Waypoint(2.,(.16,.17,TOP+.185),-.04695,'approach handle'),
                    Waypoint(2.,(.16,.17,TOP+.155),-.04695,'reach handle'),
                    Waypoint(1.,(.16,.17,TOP+.155),-.018,'support saw'),
                    Waypoint(3.,(.16,.17,TOP+.155),-.018,'hold')],
            'right':[Waypoint(1.5,(.23,-.22,TOP+.10),-.04695,'approach pick',math.pi/2),
                     Waypoint(1.5,(.23,-.22,z),-.04695,'reach handle',math.pi/2),
                     Waypoint(1.,(.23,-.22,z),0.,'pinch handle',math.pi/2),
                     Waypoint(1.5,(.23,-.22,TOP+.12),0.,'lift pick',math.pi/2),
                     tilted(1.,(.23,-.18,TOP+.13),0.,'tilt for service'),
                     tilted(1.5,(.23,-.18,TOP+.13),0.,'hold tilted pick')]}
