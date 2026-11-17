"""Recover a wooden spatula obstructing a full-size countertop dishwasher.

Leave the utensil on the counter and the rack fully inside. A light external
rotation check measures the lower spray arm after the robot withdraws.
"""
import math,pathlib,sys
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from simready import SimReady
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Restore free rotation of the dishwasher’s lower spray arm, leaving its rack fully inside and the loose wooden utensil on the counter.'
SUCCESS_HOLD_S=0.0
FRONT=.33;AXIS=np.array([.525,0,TOP+.088]);HOME=.525;RACK0=.415;RIM=.110
LOOKS={
 'white':m.Look(color=m.rgb(.88,.88,.87),roughness=.45),
 'steel':m.Look(color=m.rgb(.56,.58,.6),metallic=.75,roughness=.35),
 'gray':m.Look(color=m.rgb(.51,.52,.53),roughness=.7),
 'dark':m.Look(color=m.rgb(.11,.12,.13),roughness=.55),
 'glass':m.Look(color=m.rgb(.33,.40,.43),roughness=.15),
 'counter':m.Look(color=m.rgb(.62,.58,.51),roughness=.8)}

def body(name,meshes,pos=(0,0,TOP),fixed=True,mass=1,friction=(.4,.3)):
 parts=[('part'+str(i),mesh,look) for i,(mesh,look) in enumerate(meshes)]
 cols=[m.col_hull('c'+str(i),mesh) for i,(mesh,look) in enumerate(meshes)]
 return m.fixed(name,pos,parts,cols,LOOKS,friction=friction) if fixed else m.free(name,pos,parts,cols,LOOKS,mass=mass,friction=friction)
# Overall envelope 430 x 430 x 460 mm; 340 x 350 x 320 mm interior.
# Wall/tank thicknesses follow the manufacturer's difference of these envelopes.
shell=body('dishwasher',[
 (m.box((.43,.43,.065),(FRONT+.215,0,.0325)),'white'),
 (m.box((.43,.045,.395),(FRONT+.215,.1925,.2625)),'white'),
 (m.box((.43,.045,.395),(FRONT+.215,-.1925,.2625)),'white'),
 (m.box((.06,.34,.32),(FRONT+.40,0,.225)),'white'),
 (m.box((.43,.34,.075),(FRONT+.215,0,.4225)),'white'),
 (m.box((.004,.34,.32),(FRONT+.372,0,.225)),'steel')])
# A normal counter supports the full appliance footprint beyond the robot table.
counter=body('counter_extension',[(m.box((.42,.48,.035),(.68,0,-.0175)),'counter'),
 (m.box((.045,.045,TOP),(.85,.205,-TOP/2)),'counter'),
 (m.box((.045,.045,TOP),(.85,-.205,-TOP/2)),'counter')])
# Door rests at its ordinary fully-open mechanical stop. The hinge permits closing.
doorparts=[('door',m.box((.31,.41,.025),(-.155,0,0)),'white'),
 ('window',m.box((.26,.32,.003),(-.155,0,.014)),'glass')]
door=m.mechanism('door',(FRONT,0,TOP+.069),[m.hinge('door','Y',(0,0,0),0,90,doorparts,
 [m.col_box('door_c',(.31,.41,.025),(-.155,0,0))],mass=1.3,gravity=True,com=(-.155,0,0),
 friction=.15,damping=.3,armature=0,inertia=(.019,.012,.025))],LOOKS,friction=(.3,.25))
# Source rack envelope 337.82 x 347.98 x 190.5 mm. Main cage and cup rail
# locations are image-based reconstruction, not manufacturer CAD dimensions.
rp=[];rc=[]
def rod(name,a,b,r=.0025):
 name='rod'+str(len(rp))
 mesh=m.tube([a,((np.array(a)+b)/2).tolist(),b],r,sides=10)
 rp.append((name,mesh,'gray'));rc.append(m.col_hull(name+'_c',mesh))
for z in [.035,RIM]:
 for y in [-.162,.162]:rod('side'+str(y)+str(z),[-.1715,y,z],[.1715,y,z])
 for x in [-.1715,.1715]:rod('end'+str(x)+str(z),[x,-.162,z],[x,.162,z])
for x in np.linspace(-.148,.148,10):
 rod('floor'+str(x),[x,-.155,.035],[x,.155,.035],.002)
 for y in [-.162,.162]:rod('sidewire'+str(x)+str(y),[x,y,.035],[x,y,RIM],.002)
for y in [-.115,-.055,.005,.065,.125]:
 rod('longfloor'+str(y),[-.1715,y,.035],[.1715,y,.035],.002)
for x in np.linspace(-.125,.125,6):
 for y in [-.09,.07]:rod('tine'+str(x)+str(y),[x,y,.035],[x+.01,y,.112],.002)
# Raised side cup rack support, visible on source replacement rack photograph.
for x in [-.145,0,.145]:rod('cupupright'+str(x),[x,.14,.035],[x,.14,.187])
rod('cuptop',[-.155,.14,.187],[.155,.14,.187])
rod('cuplower',[-.155,.14,.155],[.155,.14,.155])
# Real rolling wheels on a free basket. The rack may also lift out, as it can
# in the appliance; there is no axial constraint or rigid attachment to the case.
from dataclasses import replace
wheels=[]
for x in [-.13,0,.13]:
 for y in [-.16441,.16441]:
  n='wheel'+str(len(wheels));at=(x,y,.017)
  mesh=m.cylinder(.017,.009,at,axis='y',segments=20)
  wheels.append(m.hinge(n,'Y',at,-36000,36000,[(n,mesh,'gray')],
   [m.col_hull(n+'_c',mesh)],mass=.015,gravity=True,com=at,
   inertia=(1.18e-6,2.17e-6,1.18e-6),friction=.00004,damping=.00001,armature=0))
rack=m.mechanism('rack',(RACK0,0,TOP+.084),wheels,LOOKS,frame_parts=rp,
 frame_colliders=rc,free_mass=.76,frame_inertia=(.009,.010,.014),
 friction=(.35,.25),solver_iters=64)
_rack_build=rack.build
def build_rooted_rack():
 from pxr import Usd,Sdf,UsdPhysics
 layer=Sdf.Layer.CreateAnonymous('.usda')
 if not layer.ImportFromString(_rack_build()):raise ValueError('Invalid rack USD')
 stage=Usd.Stage.Open(layer);root=stage.GetDefaultPrim();frame=root.GetChild('frame')
 attrs=[(a.GetName(),a.GetTypeName(),a.Get()) for a in root.GetAttributes() if a.GetName().startswith('physxArticulation:')]
 root.RemoveAPI(UsdPhysics.ArticulationRootAPI);root.RemoveAppliedSchema('PhysxArticulationAPI')
 UsdPhysics.ArticulationRootAPI.Apply(frame);frame.AddAppliedSchema('PhysxArticulationAPI')
 for name,kind,value in attrs:
  frame.CreateAttribute(name,kind).Set(value);root.RemoveProperty(name)
 return layer.ExportToString()
rack=replace(rack,build=build_rooted_rack)
tracks=body('rack_tracks',[(m.box((.35,.01,.019),(FRONT+.195,y,.0745)),'gray') for y in [-.16441,.16441]])
# Lower spray arm: ordinary tapered body, no invented through-opening.
outline=[(-.155,-.013),(-.14,-.020),(0,-.025),(.14,-.020),(.155,-.013),(.155,.013),(.14,.020),(0,.025),(-.14,.020),(-.155,.013)]
armmesh=m.prism(outline,-.005,.005)
ap=[('arm',armmesh,'gray'),('hub',m.cylinder(.014,.015,(0,0,-.008)),'gray')]
ac=[m.col_hull('arm_c',armmesh),m.col_cylinder('hub_c',.014,.015,(0,0,-.008))]
arm=m.mechanism('spray_arm',AXIS,[m.hinge('arm','Z',(0,0,0),-36000,36000,ap,ac,mass=.055,
 friction=.0001,damping=.00015,armature=0,inertia=(.00001,.00045,.00045))],LOOKS,friction=(.25,.2),solver_iters=32)
# Full-scale SimReady 300 x 56 x 6 mm spatula, not enlarged for the gripper.
a=np.array([-.70710678,0,.70710678]);b=np.array([0,1,0]);rot=np.column_stack((b,a,np.cross(b,a)))
quat=tuple(Rotation.from_matrix(rot).as_quat())
spatula=SimReady('spatula',pos=(.35,.105,TOP+.201),size=(.056,.300,.006),mass=.06,
 package='kitchenware_spatula_b01',usd='sm_kitchenware_spatula_b01_01.usd',quat_xyzw=quat,convex=False,angular_damping=.3)
cup=SimReady('counter_cup',pos=(.60,-.36,TOP+.001),size=(.119,.084,.102),mass=.20,
 package='dishware_cup_tea_a01',usd='sm_dishware_cup_tea_a01_01.usd',convex=True)
banana=SimReady('counter_banana',pos=(.40,-.52,TOP+.001),size=(.039,.197,.074),mass=.13,
 package='food_banana_a01',usd='sm_food_banana_a01_01.usda',convex=True)
OBJECTS=[counter,shell,door,tracks,rack,arm,spatula,cup,banana]

from completion import evaluate,CompletionTracker

class Trial:
 def reset(self,env=None):
  self.steps=0;self.angle=0.;self.env=env;self.tracker=CompletionTracker();self.conditions={}
 def step(self,env):
  import torch
  self.env=env;self.steps+=1
  base=env.unwrapped;sp=base.scene['spatula'];rs=m.state(env,'rack')
  pose=m._np(sp.data.root_link_pose_w)[0];vel=m._np(sp.data.root_vel_w)[0]
  origin=m._np(base.scene.env_origins[0])
  self.angle=m.state(env,'spray_arm').deg('arm')
  self.conditions=evaluate(top=TOP,spatula_pos=pose[:3]-origin,spatula_rotation=m.quat_matrix(pose[3:]),
   spatula_velocity=vel[:3],spatula_omega=vel[3:],rack_pos=rs.pos,rack_rotation=m.quat_matrix(rs.quat_xyzw),
   rack_velocity=rs.lin_vel,rack_omega=rs.ang_vel,fingertips=m.fingertips(env))
  self.tracker.update(self.conditions,self.angle)
  # An external 1 mN m test, equivalent to gently turning the unloaded arm.
  # This is a measurement actuator, not a simulated water jet or wash cycle.
  asset=base.scene['spray_arm'];i=int(asset.find_bodies('arm')[0][0])
  force=torch.zeros((1,1,3),device=asset.device);torque=force.clone()
  if self.tracker.apply_test_torque:torque[0,0,2]=.001
  asset.permanent_wrench_composer.set_forces_and_torques_index(forces=force,torques=torque,body_ids=[i],is_global=True)
  if self.steps==1 or self.steps%20==0:
   print('DISHWASHER_STATE',dict(step=self.steps,arm_deg=self.angle,rack_pos=rs.pos.tolist(),
    rack_quat=rs.quat_xyzw.tolist(),spatula_pose=pose.tolist(),score=self.tracker.score,
    rotation_start=self.tracker.start_angle,rotation_passed=self.tracker.rotation_passed,
    conditions=self.conditions),flush=True)
TRIAL=Trial();TRIAL.reset();m.on_reset(TRIAL.reset);m.on_step(TRIAL.step)

def score(objects):return TRIAL.tracker.score

def success(objects):return score(objects)==1.0

from closedloop import placeholder_demo, grip_rot
from yam_arena import ik
from yam_arena.constants import READY
demo=placeholder_demo

def joint_approach(arm,pos,rot,duration,finger):
 # A smooth joint-space approach avoids an orientation singularity in the
 # straight Cartesian interpolation from the ready pose. Robot actions only.
 q1,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),np.array(READY),base_pos=arm.base_pos,iters=150)
 arm.worst_ik_mm=max(arm.worst_ik_mm,pe*1000);arm.worst_ik_deg=max(arm.worst_ik_deg,float(np.degrees(re)))
 q0=arm.q.copy();f0=arm.finger
 for i in range(1,round(duration/arm.dt)+1):
  f=ik.smoothstep(i/round(duration/arm.dt));arm.q=q0+f*(q1-q0);arm.finger=f0+f*(finger-f0)
  arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base_pos)
  yield

def demo_script(ctx):
 L,R=ctx.left,ctx.right
 rr=Rotation.from_euler('y',-15,degrees=True).as_matrix()@grip_rot(yaw=math.pi/2)
 yield from R.hold(.5)
 rackstate=ctx.obj('rack');rx=float(rackstate.pos[0])
 grip=np.array([rx-.1715,-.13,TOP+.084+RIM-.005])
 yield from joint_approach(R,grip+[0,0,.025],rr,1.6,-.018)
 yield from R.move(grip,1.2,-.018,rot=rr)
 yield from R.trim_to_measured(grip,tol=.0015,max_comp=.01,max_duration=.8)
 yield from R.grip(0.,.6)
 yield from R.hold(.3)
 print('DEMO_RACK_STABILIZED',ctx.obj('rack').pos.tolist(),'gap',R.gap(),flush=True)
 if not R.holding(.0015):return
 sp=ctx.obj('spatula');rot=m.quat_matrix(sp.quat_xyzw)
 head=sp.pos+rot@np.array([.00000184,.110+.03703091,.003-.0030884])
 lr=grip_rot(yaw=math.atan2(-rot[1,1],-rot[0,1]),pitch=math.radians(82),roll=math.pi/2)
 yield from joint_approach(L,head+[-.05,0,0],lr,1.6,-.012)
 yield from L.hold(.3)
 sp=ctx.obj('spatula');rot=m.quat_matrix(sp.quat_xyzw)
 head=sp.pos+rot@np.array([.00000184,.110+.03703091,.003-.0030884])
 lr=grip_rot(yaw=math.atan2(-rot[1,1],-rot[0,1]),pitch=math.radians(82),roll=math.pi/2)
 yield from L.move(head,1.8,-.008,rot=lr)
 yield from L.trim_to_measured(head,tol=.0015,max_comp=.015,max_duration=1.2)
 print('DEMO_FIRST_TARGET',head.tolist(),'measured',L.measured_pos.tolist(),flush=True)
 if not ctx.offline and np.linalg.norm(L.measured_pos-head)>.005:return
 yield from L.grip(0.,.6)
 yield from L.hold(.4)
 print('DEMO_FIRST_GAP',L.gap(),flush=True)
 if not L.holding(.0015):return
 extracted=head+[-.10,0,.035]
 yield from L.move(extracted,2.5,rot=lr)
 yield from L.hold(.3)
 print('DEMO_FIRST_WITHDRAW',ctx.obj('spatula').pos.tolist(),'rack',ctx.obj('rack').pos.tolist(),flush=True)
 # The left hand keeps the utensil fixed while the other hand returns the rack.
 yield from R.grip(-.018,.4)
 yield from R.move(grip+[-.06,0,.015],1.2,rot=rr)
 rackstate=ctx.obj('rack');rx=float(rackstate.pos[0]);pushrot=grip_rot(pitch=math.pi/2)
 contact=np.array([rx-.1715-.0155,-.13,float(rackstate.pos[2])+RIM])
 yield from joint_approach(R,contact+[-.045,0,0],pushrot,1.6,0.)
 yield from R.move(contact,1.2,0.,rot=pushrot)
 yield from R.move(contact+[HOME-rx,0,0],3.,rot=pushrot)
 print('DEMO_RACK_RETURN_HELD',ctx.obj('rack').pos.tolist(),'spatula',ctx.obj('spatula').pos.tolist(),flush=True)
 yield from R.move(contact+[HOME-rx-.06,0,0],1.,rot=pushrot)
 yield from R.home(1.5)
 # Release onto the existing door/rack support, then grasp the exposed neck.
 yield from L.grip(-.018,.5)
 yield from L.move(extracted+[.05,0,.07],1.4,rot=lr)
 yield from L.hold(.8)
 sp=ctx.obj('spatula');rot=m.quat_matrix(sp.quat_xyzw)
 neck=sp.pos+rot@np.array([.00000184,.03703091,.003-.0030884])+np.array([0,0,.008])
 lr=grip_rot(yaw=math.atan2(-rot[1,1],-rot[0,1]),pitch=math.pi/4,roll=0.)
 print('DEMO_NECK_TARGET',neck.tolist(),'rack',ctx.obj('rack').pos.tolist(),flush=True)
 yield from joint_approach(L,neck+[0,0,.11],lr,1.6,-.035)
 yield from L.move(neck,1.6,-.035,rot=lr)
 yield from L.trim_to_measured(neck,tol=.0015,max_comp=.015,max_duration=1.0)
 print('DEMO_NECK_MEASURED',L.measured_pos.tolist(),flush=True)
 if not ctx.offline and np.linalg.norm(L.measured_pos-neck)>.006:return
 yield from L.grip(0.,.6)
 yield from L.hold(.3)
 print('DEMO_SECOND_GAP',L.gap(),flush=True)
 if not L.holding(.0015):return
 held=ctx.obj('spatula');heldrot=m.quat_matrix(held.quat_xyzw)
 withdrawn=neck+[-.16,0,.025]
 yield from L.move(withdrawn,2.5,rot=lr)
 yield from L.hold(.3)
 raised=withdrawn+[.02,0,.120]
 yield from L.move(raised,2.5,rot=lr)
 yield from L.hold(.4)
 lifted=ctx.obj('spatula')
 print('DEMO_SECOND_LIFT',held.pos.tolist(),lifted.pos.tolist(),'rack',ctx.obj('rack').pos.tolist(),flush=True)
 if ctx.offline or lifted.pos[2]-held.pos[2]<.08:
  # A released utensil on the open door is a recoverable intermediate state.
  yield from L.grip(-.035,.4)
  yield from L.hold(.6)
  sp=ctx.obj('spatula');rot=m.quat_matrix(sp.quat_xyzw)
  flat_grip=grip_rot(yaw=math.atan2(-rot[1,1],-rot[0,1]))
  pickup=sp.pos+np.array([0,0,.011])
  print('DEMO_DOOR_PICKUP',pickup.tolist(),flush=True)
  if not ctx.offline and (abs(rot[2,1])>.15 or not TOP+.077<sp.pos[2]<TOP+.100):return
  yield from joint_approach(L,pickup+[0,0,.06],flat_grip,1.6,-.022)
  yield from L.move(pickup,1.8,-.022,rot=flat_grip)
  yield from L.trim_to_measured(pickup,tol=.0015,max_comp=.012,max_duration=.8)
  print('DEMO_DOOR_TARGET',pickup.tolist(),'measured',L.measured_pos.tolist(),flush=True)
  if not ctx.offline and np.linalg.norm(L.measured_pos-pickup)>.006:return
  yield from L.grip(0.,.6)
  yield from L.hold(.3)
  print('DEMO_DOOR_GAP',L.gap(),flush=True)
  if not L.holding(.0015):return
  raised=pickup+[0,0,.07]
  yield from L.move(raised,1.8,rot=flat_grip)
  # Clear the basket, then move ahead of the fixed robot base before moving out.
  lane=raised.copy();lane[1]=.20
  yield from L.move(lane,1.0,rot=flat_grip)
  forward=lane.copy();forward[0]=.26
  yield from L.move(forward,1.5,rot=flat_grip)
  side=forward.copy();side[1]=.49
  yield from L.move(side,1.5,rot=flat_grip)
  target=np.array([.38,.49,TOP+.014])
  yield from L.move(target,2.,rot=flat_grip)
  yield from L.grip(-.025,.5)
  yield from L.move(target+[0,0,.10],1.2,rot=flat_grip)
  yield from L.home(1.5)
 else:
  side=raised.copy();side[1]=.41
  yield from L.move(side,2.,rot=lr)
  tilt=math.pi/4;bx=np.array([0,1,0]);by=np.array([-math.cos(tilt),0,math.sin(tilt)])
  placebody=np.column_stack((bx,by,np.cross(bx,by)))
  placegrip=placebody@(lr.T@heldrot).T
  target=np.array([.28,.41,TOP+.187*math.sin(tilt)+.010])
  yield from L.move(target,2.5,rot=placegrip)
  yield from L.grip(-.018,.5)
  yield from L.move(target+[0,0,.12],1.4,rot=lr)
  yield from L.home(1.5)
 # Return any basket travel caused by the final withdrawal.
 rackstate=ctx.obj('rack');rx=float(rackstate.pos[0])
 if rx<HOME-.002:
  contact=np.array([rx-.1715-.0155,-.13,float(rackstate.pos[2])+RIM])
  yield from joint_approach(R,contact+[-.045,0,0],pushrot,1.6,0.)
  yield from R.move(contact,1.2,0.,rot=pushrot)
  yield from R.move(contact+[HOME-rx,0,0],3.,rot=pushrot)
  print('DEMO_FINAL_RACK_RETURN',ctx.obj('rack').pos.tolist(),flush=True)
  yield from R.move(contact+[HOME-rx-.06,0,0],1.,rot=pushrot)
  yield from R.home(1.5)
 yield from joint_approach(L,np.array([.14,.15,TOP+.16]),grip_rot(),1.5,FINGER_OPEN)
 yield from joint_approach(R,np.array([.16,-.30,TOP+.16]),grip_rot(),1.5,FINGER_OPEN)
 yield from L.hold(15.)
