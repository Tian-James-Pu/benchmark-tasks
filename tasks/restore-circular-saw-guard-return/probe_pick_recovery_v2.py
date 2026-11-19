"""Physical pick recovery attempt. Scoring remains disabled until validated."""
import pathlib,sys,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
_original_scene_patch=m._Finder.targets['yam_arena.scene']
def fine_scene(scene):
    _original_scene_patch(scene);scene.PHYSICS_DT=1./480.
m._Finder.targets['yam_arena.scene']=fine_scene
from spring_scene_v6 import objects
from spring_mechanics_v7 import ExtensionSpring
from service_pick import make_pick
from calibrated_demo import tip_step,OBS
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


CAPTURE={}
def eye_target(dz=0.):
    h=OBS.get('hook')
    if h is None:return np.array([.184,-.008,.9408])+[0,0,dz]
    return h.pos+h.rotation()@np.array([-.0003,0.,0.])+[0,0,dz]

def freeze_eye(env):
    if trial.steps in (119,190):
        h=m.state(env,'spring_hook');s=m.state(env,'saw')
        CAPTURE['eye']=h.pos.copy();CAPTURE['rotation']=h.rotation();CAPTURE['pivot']=s.pos+s.rotation()@np.array([-.055,0.,.065])
        CAPTURE['pin']=CAPTURE['pivot']+np.array([.024*math.sin(s.joints['guard_pivot']),-.024*math.cos(s.joints['guard_pivot']),.0165])
        print('SPRING_CAPTURE_TARGET',{k:v.tolist() for k,v in CAPTURE.items()},flush=True)
m.on_step(freeze_eye)

def capture_point(delta=(0.,0.,0.)):
    return CAPTURE.get('eye',np.array([.184,-.008,.9408]))+np.asarray(delta)

def arc_point(angle):
    pivot=CAPTURE.get('pivot',np.array([.20,.015,.9425]));a=math.radians(angle);q=a+math.pi/2
    return pivot+np.array([.030*math.cos(a)+.001*math.cos(q),.030*math.sin(a)+.001*math.sin(q),.015])

def anchor_point(outward=0.,dz=0.):
    pin=CAPTURE.get('pin',np.array([.2138,-.0047,.959]));q=math.radians(35.)
    return pin+.007*np.array([math.cos(q),math.sin(q),0.])+outward*np.array([math.sin(q),-math.cos(q),0.])+[0,0,dz-.0005]

def demo():
    z=TOP+.0145
    right=[Waypoint(1.5,(.23,-.22,TOP+.10),-.04695,'approach pick',math.pi/2),
           Waypoint(1.5,(.23,-.22,z),-.04695,'grasp handle',math.pi/2),
           Waypoint(1.,(.23,-.22,z),0.,'pinch handle',math.pi/2),
           Waypoint(1.5,(.23,-.22,TOP+.12),0.,'lift pick',math.pi/2),
           Waypoint(.5,(.23,-.22,TOP+.12),0.,'measure held-tool transform',math.pi/2),
           tip_step(2.,lambda:eye_target(.015),'approach observed spring eye'),
           tip_step(1.5,lambda:eye_target(-.0005),'enter observed spring eye'),
           tip_step(1.,lambda:capture_point((.003,-.003,0.)),'tension spring hook')]
    for angle in np.linspace(245.,305.,7):
        right.append(tip_step(.7,lambda a=angle:arc_point(a),'route hook outside anchor'))
    right += [tip_step(1.,lambda:anchor_point(.006),'pass anchor'),
              tip_step(1.,lambda:anchor_point(0.),'align hook mouth with pin'),
              tip_step(1.,lambda:anchor_point(0.,.015),'release spring onto pin'),
              tip_step(2.,lambda:anchor_point(.04,.025),'observe retention')]
    return {'left':[Waypoint(2.,(.16,.17,TOP+.185),-.04695,'approach saw handle'),
                    Waypoint(2.,(.16,.17,TOP+.155),-.04695,'reach handle'),
                    Waypoint(1.,(.16,.17,TOP+.155),-.018,'support saw'),
                    Waypoint(22.,(.16,.17,TOP+.155),-.018,'support repair')], 'right':right}
