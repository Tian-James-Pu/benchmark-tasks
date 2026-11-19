"""No robot task score. Source-scale fixed-plug extraction contact experiment.

The axis guide and external 3 N test force are an explicit measurement fixture,
not a robot demo. The commercial lock reconstruction remains unvalidated.
"""
import pathlib,sys,math,os
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
import lock_geometry as g
from scaled_convex import scaled_convex_part
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

ANGLE=0.0
PHYSICS_HZ=1920
original=m._Finder.targets['yam_arena.scene']
def patch(scene):
    original(scene)
    scene.PHYSICS_DT=1/PHYSICS_HZ
m._Finder.targets['yam_arena.scene']=patch
SCENE='station'
TOP=TABLE_TOP[SCENE]
C=np.array([.30,0,TOP+.12])
INSTRUCTION='Inspect the key fragment contact fixture.'
SUCCESS_HOLD_S=0
LOOKS={'brass':m.Look(color=m.rgb(.70,.50,.16),metallic=.8,roughness=.3),
       'steel':m.Look(color=m.rgb(.65,.68,.71),metallic=.8,roughness=.3),
       'fixture':m.Look(color=m.rgb(.24,.27,.29),metallic=.5,roughness=.6)}
quat=(0,math.sin(math.radians(ANGLE)/2),0,math.cos(math.radians(ANGLE)/2))

plug_cells=g.plug_cells()
plug_parts,plug_colliders=g.parts_and_colliders(plug_cells,'plug','brass')
# Static geometry can use triangle-mesh contact. All source-scale convex pieces
# remain available for a future physically rotating plug.
plug_visual=m.join(*plug_cells).turned(ANGLE,'y')
plug=m.fixed('plug',C,[('plug',plug_visual,'brass')],
             [m.col_hull('plug_c',plug_visual).replace('"convexHull"','"none"')],LOOKS,friction=(.2,.15))
housing_cells=g.housing_cells()
housing=m.join(*housing_cells)
shell=m.fixed('housing',C,[('shell',housing,'steel')],
              [m.col_hull('shell_c',housing).replace('"convexHull"','"none"')],LOOKS,friction=(.2,.15))

pins=[]
for i in range(6):
    pin=g.bottom_pin(i)
    mass=8500*math.pi*g.PIN_R**2*g.BOTTOM_LENGTHS[i]
    iz=mass*g.PIN_R**2/2
    ix=mass*(3*g.PIN_R**2+g.BOTTOM_LENGTHS[i]**2)/12
    pins.append(m.slider(f'pin{i}','Z',-.004,.004,[(f'pin{i}',pin,'brass')],
        [m.col_hull(f'pin{i}_c',pin)],mass=mass,friction=0,damping=.001,
        armature=0,gravity=True,com=(0,g.CENTERS[i],g.ROOTS[i]-g.R+g.BOTTOM_LENGTHS[i]/2),
        inertia=(ix,ix,iz)))
bottoms=scaled_convex_part(m.mechanism('bottom_pins',C,pins,LOOKS,start_quat=quat,friction=(.2,.15),solver_iters=64))

drivers=[]
for i in range(6):
    length=(.235 if g.CUTS[i]<=3 else .200 if g.CUTS[i]<=6 else .165)*g.INCH
    pin=m.cylinder(g.PIN_R,length,(0,g.CENTERS[i],g.R+length/2),segments=24)
    mass=8500*math.pi*g.PIN_R**2*length
    iz=mass*g.PIN_R**2/2
    ix=mass*(3*g.PIN_R**2+length**2)/12
    drivers.append(m.slider(f'driver{i}','Z',-.004,.006,[(f'driver{i}',pin,'brass')],
        [m.col_hull(f'driver{i}_c',pin)],mass=mass,friction=0,damping=.01,
        spring=140,effort=2,armature=0,gravity=True,
        com=(0,g.CENTERS[i],g.R+length/2),inertia=(ix,ix,iz)))
top_pins=scaled_convex_part(m.mechanism('drivers',C,drivers,LOOKS,friction=(.2,.15),solver_iters=64))

key_cells=g.key_cells()
key_parts,key_colliders=g.parts_and_colliders(key_cells,'key','brass')
mass=8500*sum(g.ConvexHull(c.v).volume for c in key_cells)
keylink=m.slider('key','Y',-.040,.001,key_parts,key_colliders,mass=mass,friction=0,
                 damping=.03,armature=0,gravity=True,com=(0,g.LENGTH/2,-.002),
                 inertia=(mass*g.LENGTH**2/12,mass*g.HEIGHT**2/12,mass*g.LENGTH**2/12))
key=scaled_convex_part(m.mechanism('fragment',C,[keylink],LOOKS,start_quat=quat,
                      friction=(.2,.15),solver_iters=64))
base=m.fixed('fixture',(.30,0,TOP),[('base',m.box((.08,.08,.012),(0,0,.006)),'fixture'),
        ('stand',m.box((.03,.012,.11),(0,g.LENGTH+.007,.061)),'fixture')],
        [m.col_box('base_c',(.08,.08,.012),(0,0,.006)),
         m.col_box('stand_c',(.03,.012,.11),(0,g.LENGTH+.007,.061))],LOOKS)
OBJECTS=[base,plug,shell,bottoms,top_pins,key]

# A constant spring rest point is set during all physics steps, including
# initial settling. It is a stated generic force assumption, not calibrated.
from physics_hooks import on_physics_step
@on_physics_step
def preload(env,dt):
    for i in range(6): m.set_target(env,'drivers',f'driver{i}',-.0014)

class Trial:
    def reset(self,env=None):
        self.steps=0
        if env is not None:
            print('LOCK_SOURCE_DIMENSIONS',g.source_dimensions(),flush=True)
    def step(self,env):
        self.steps+=1
        m.push(env,'fragment',(0,-3.,0),body='key')
        if self.steps==1 or self.steps%5==0:
            k=m.state(env,'fragment')
            print('LOCK_CONTACT',self.steps,{'angle_deg':ANGLE,'key_y_m':k.joint('key'),
                'bottom_pins_m':m.state(env,'bottom_pins').joints,
                'drivers_m':m.state(env,'drivers').joints,
                'root_m':k.pos.tolist()},flush=True)
trial=Trial()
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right':[Waypoint(1.,(.20,-.25,TOP+.22),-.04695,'measurement fixture',0.)]}
