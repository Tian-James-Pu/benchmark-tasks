"""Full-scale NSRI driver pickup/access gate. Diagnostic score is always zero.

Clamp/feeding dynamics are not enabled in this first bounded gate. Nothing here
is evidence of a complete repair. Original-size case and driver remain physical.
"""
import math,pathlib,sys
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=0.
INSTRUCTION='Restore reliable screw presentation at the robotic feeder pickup.'
FRONT=.26;CASE_POS=np.array([FRONT,0,TOP]);TOOL_POS=np.array([.410,-.170,TOP+.0092])
LOOK={n:m.Look(color=m.rgb(*v),metallic=metal,roughness=.45) for n,v,metal in [('case',(.76,.77,.79),.15),('steel',(.5,.54,.58),.8),('dark',(.055,.06,.07),0),('red',(.72,.035,.025),0),('cover',(.15,.17,.18),.1)]}
def col(n,g,sdf=False):
    s=m.col_hull(n,g).replace('contactOffset = 0.004','contactOffset = 0.00005')
    if sdf:s=s.replace('"PhysicsMeshCollisionAPI"','"PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"').replace('physics:approximation = "convexHull"','physics:approximation = "sdf"\n int physxSDFMeshCollision:sdfResolution = 512')
    return s

def add(p,c,n,g,look='steel',sdf=False):
    n=n.replace('.','p').replace('-','n');p.append((n,g,look));c.append(col(n+'_c',g,sdf))
def safe(p):
    b=p.build
    return replace(p,build=lambda:b().replace('contactOffset = 0.004','contactOffset = 0.00005'))

# Dimensioned full unit, free on its original four rubber feet. No bench anchor.
cp,cc=[],[]
add(cp,cc,'bottom',m.box((.181,.123,.003),(.0905,0,.003)),'dark')
# Cast exterior envelope and the front step, with actual50.8mm service shelf.
for side in [-1,1]:
    g=m.prism([(0,.0045),(.181,.0045),(.181,.1432),(.0508,.1432),(.0508,.0847),(0,.0847)],-.001,.001)
    g=m.Mesh(g.v[:,[0,2,1]],g.f[:,::-1]).moved((0,side*.0605,0));add(cp,cc,'side_'+str(side).replace('-','n'),g,'case')
add(cp,cc,'back',m.box((.002,.119,.139),(.180,0,.074)),'case')
add(cp,cc,'front',m.box((.002,.119,.0802),(.001,0,.0446)),'case')
add(cp,cc,'step',m.box((.0508,.119,.002),(.0254,0,.0837)),'case')
add(cp,cc,'main_front',m.box((.002,.119,.0585),(.0518,0,.11395)),'case')
# Covered original hopper; no open top invented as a robot access route.
add(cp,cc,'upper_cover',m.box((.127,.119,.0018),(.1155,0,.1441)),'cover')
for x in [.014,.167]:
    for y in [-.047,.047]:add(cp,cc,'foot'+str(x)+str(y),m.cylinder(.006,.0015,(x,y,.00075)),'dark')
# Original front working geometry, immobilized in this diagnostic only.
add(cp,cc,'escaper',m.cylinder(.020,.0015,(.021,-.018,.08545)),'steel')
for y in [.0149,.0211]:add(cp,cc,'rail'+str(y),m.box((.039,.003,.006),(.0313,y,.0817)))
add(cp,cc,'holding_plate',m.box((.031,.032,.0012),(.0275,.018,.0878)))
for y in [.003,.033]:add(cp,cc,'bracket_edge'+str(y),m.box((.0012,.002,.030),(.0436,y,.1034)))
add(cp,cc,'bracket_bridge',m.box((.0012,.032,.007),(.0436,.018,.115)))
add(cp,cc,'bracket_lower',m.box((.0012,.032,.008),(.0436,.018,.0924)))
# Original attaching bosses and heads. M3/AF2.5 reconstruction from photo,
# not a manufacturer part-size callout. The physical thread gate will replace
# this diagnostic fixed pair before any completed manipulation is scored.
for i,y in enumerate([.008,.028]):
    add(cp,cc,'boss'+str(i),m.box((.006,.006,.010),(.0472,y,.099)))
    add(cp,cc,'attaching_head'+str(i),m.cylinder(.00275,.003,(.0417,y,.099),axis='x'))
# Original power switch and visible pickup sensor housing.
add(cp,cc,'switch',m.box((.004,.012,.016),(-.0015,-.035,.052)),'dark')
add(cp,cc,'pickup_sensor',m.box((.010,.009,.006),(.022,-.047,.0897)),'dark')
CASE=safe(m.free('feeder',CASE_POS,cp,cc,LOOK,mass=3.,friction=(.6,.5),solver_iters=64,com=(.098,0,.065)))

# Wiha42425: tip x0; blade60mm; whole160mm;18mm maximum cap diameter.
# Radius profile follows manufacturer photograph, disclosed reconstruction.
# The end cap rotates freely, so gripping it cannot falsely drive the shaft.
hp,hc=[],[]
profile=[(0,-.140),(.0075,-.140),(.006,-.125),(.005,-.110),(.0048,-.095),(.0057,-.080),(.0055,-.073),(.0042,-.065),(.0035,-.060),(0,-.060)]
g=m.lathe(profile,48).turned(90,'y');add(hp,hc,'tapered_handle',g,'dark',True)
add(hp,hc,'nose',m.cylinder(.0035,.004,(-.062,0,0),axis='x'),'red')
add(hp,hc,'shaft',m.cylinder(.00155,.055,(-.0325,0,0),axis='x'))
a=np.arange(6)*math.pi/3;rad=.0025/math.sqrt(3)
hex_tip=m.prism(np.c_[rad*np.cos(a),rad*np.sin(a)],-.005,0).turned(90,'y')
add(hp,hc,'hex_tip',hex_tip)
ep,ec=[],[];cap=m.lathe([(0,-.160),(.0084,-.160),(.009,-.157),(.0088,-.148),(.0075,-.140),(0,-.140)],48).turned(90,'y');add(ep,ec,'rotating_cap',cap,'red',True)
caplink=m.hinge('cap','X',(0,0,0),-float('inf'),float('inf'),ep,ec,mass=.003,gravity=True,friction=0,damping=0,armature=0,com=(-.150,0,0),inertia=(1.1e-7,1.5e-7,1.5e-7))
DRIVER=safe(m.mechanism('driver',TOOL_POS,[caplink],LOOK,frame_parts=hp,frame_colliders=hc,free_mass=.014,frame_inertia=(2e-7,2e-5,2e-5),friction=(.6,.45),solver_iters=64))
OBJECTS=[CASE,DRIVER]
class Observe:
    def reset(self,env=None):self.n=0;self.view=None
    def step(self,env):
        self.n+=1
        if self.view is None:
            import isaaclab.sim as su
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[p for p in su.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            sensors=[str(p.GetPath()) for p in bodies if '/RightArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right')]
            filters=[str(p.GetPath()) for p in bodies if '/driver/' in str(p.GetPath())]
            self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=1024)
        if self.n==1 or self.n%10==0:
            ss={}
            for name in ['feeder','driver']:
                s=m.state(env,name);ss[name]=dict(p=s.pos.tolist(),q=s.quat_xyzw.tolist(),v=s.lin_vel.tolist(),w=s.ang_vel.tolist(),joints=s.joints)
            r=env.unwrapped.scene['right_robot'];ss['finger_q']=m._np(r.data.joint_pos)[0,-2:].tolist();ss['finger_force']=self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist();ss['finger_effort']=m._np(r.data.applied_torque)[0,-2:].tolist()
            print('FEEDER_GATE',self.n,ss,flush=True)
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    p=TOOL_POS+[-.110,0,.014]
    return {'right':[Waypoint(1.5,tuple(p+[0,0,.080]),-.025,'approach driver'),Waypoint(1.5,tuple(p),-.025,'beside original handle'),Waypoint(1.5,tuple(p),-.002,'grasp handle'),Waypoint(2.,tuple(p+[0,0,.100]),-.002,'lift driver'),Waypoint(1.,tuple(p+[0,0,.100]),-.002,'measure grasp'),Waypoint(2.,(.193,.008,TOP+.150),-.002,'carry above service position'),Waypoint(1.,(.193,.008,TOP+.150),-.002,'observe')]}
