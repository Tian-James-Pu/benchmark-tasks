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
from spring_mechanics_v6 import ExtensionSpring
from service_pick import make_pick
from oriented_demo import tilted,tcp_for_tip
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
    finger=-.0065;z=TOP+.0205
    right=[Waypoint(1.5,(.23,-.22,TOP+.10),-.04695,'approach pick',math.pi/2),
           Waypoint(1.5,(.23,-.22,z),-.04695,'grasp handle',math.pi/2),
           Waypoint(1.,(.23,-.22,z),finger,'pinch handle',math.pi/2),
           Waypoint(1.5,(.23,-.22,TOP+.12),finger,'lift pick',math.pi/2)]
    # Settled hook measurement from passive control, before any robot contact.
    hook=np.array([.1901,-.0054,.95704]);yaw=-math.pi/4
    contact=hook+np.array([.0010*math.cos(yaw),.0010*math.sin(yaw),0.])
    right += [tilted(2.,tcp_for_tip(contact+[0,0,.015]),finger,'approach spring eye'),
              tilted(1.5,tcp_for_tip(contact+[0,0,-.0003]),finger,'insert pick in open eye')]
    # Pull outside the anchor's radius, then past it, letting the open mouth
    # enter the post from its free side once the pick is withdrawn.
    pivot=np.array([.20,.015,.959])
    for angle in np.linspace(230.,305.,7):
        a=math.radians(angle);q=a+math.pi/2
        tip=pivot+np.array([.030*math.cos(a)+.001*math.cos(q),.030*math.sin(a)+.001*math.sin(q),-.0015])
        right.append(tilted(.6,tcp_for_tip(tip),finger,'stretch spring around bearing'))
    q=math.radians(35.)
    tangent=np.array([math.cos(q),math.sin(q),0.]);radial=np.array([math.sin(q),-math.cos(q),0.])
    pin=pivot+.024*radial
    outside=pin+.007*tangent+.006*radial+[0,0,-.0015]
    seated=pin+.007*tangent+[0,0,-.0015]
    right += [tilted(1.,tcp_for_tip(outside),finger,'pass anchor'),
              tilted(1.,tcp_for_tip(seated),finger,'align open hook mouth'),
              tilted(1.,tcp_for_tip(seated+[0,0,.012]),finger,'withdraw pick for retention'),
              tilted(2.,tcp_for_tip(seated+[0,-.05,.025]),finger,'observe automatic return')]
    left=[Waypoint(2.,(.16,.17,TOP+.185),-.04695,'approach saw handle'),
          Waypoint(2.,(.16,.17,TOP+.155),-.04695,'reach handle'),
          Waypoint(1.,(.16,.17,TOP+.155),-.018,'support saw'),
          Waypoint(20.,(.16,.17,TOP+.155),-.018,'support spring repair')]
    return {'left':left,'right':right}
