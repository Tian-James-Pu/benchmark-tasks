"""Original hardware, compressor housing grasp nearer the center of mass.
No repair success claimed; instrument actual relative roll and bench support.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import service_base_v40 as base
from service_base_v40 import *
from yam_arena.demo import DOWN_GRIP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
import magnetic_return_v21
YAW=-90.
_BATTERY=None
_OFF_R=np.array([[-0.9998284284871217, 0.0024079136132420043, -0.018366143709897125], [0.005704829240797364, 0.9833484884040483, -0.18164031825385976], [0.017622945436183805, -0.18171392978361567, -0.9831935105811987]])
_OFF_P=np.array([0.12105980521899527, -0.08592541340668224, 0.04178073364859142])
@m.on_step
def left_grasp(env):
 if base._CTX is None:return
 a=base._CTX.left;p,r=base.frame();q=a.measured_rot
 if base._k==10:
  asset=env.unwrapped.scene['stapler'];print('NATIVE_MASS '+json.dumps({'body_names':asset.body_names,'mass_kg':m._np(asset.data.body_mass).tolist(),'local_com_m':m._np(asset.data.body_com_pos_b).tolist()}),flush=True)
 if base._k%10==0:print('HANDLE_GRASP '+json.dumps({'step':base._k,'frame_pos':p.tolist(),'frame_R':r.tolist(),'tcp':a.measured_pos.tolist(),'tcp_R':q.tolist(),'frame_in_tcp':(q.T@(p-a.measured_pos)).tolist(),'relative_R':(q.T@r).tolist(),'finger_q':env.unwrapped.scene['left_robot'].data.joint_pos[0,6:].detach().cpu().numpy().tolist()}),flush=True)

def carry_script(ctx):
 base._CTX=ctx;L=ctx.left
 p,r=base.frame();grip=p+r@np.array([.123,.095,-.005]);D=quat_xyzw_to_matrix(DOWN_GRIP)
 rot=r@axis_angle_matrix(np.array([1.,0.,0.]),np.radians(-20.))@axis_angle_matrix(np.array([0.,0.,1.]),np.radians(YAW))@D
 yield from L.move(grip+np.array([-.06,0,.08]),1.2,-.045,rot=rot)
 yield from L.move(grip+np.array([0,0,.035]),1.2,-.045,rot=rot)
 yield from L.hold(.4)
 yield from L.move(grip,1.0,-.045,rot=rot)
 yield from L.grip(-.020,.8)
 yield from L.hold(.5)
 yield from L.move(L.pos+np.array([0,0,.100]),2.,-.020,rot=rot)
 yield from L.hold(.5)
 yield from L.move(L.pos+np.array([-.100,.060,.050]),2.5,-.020,rot=rot)
 yield from L.hold(2.)

_VIEW=None
@m.on_step
def grasp_contact(env):
 global _VIEW
 if _VIEW is None:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics
  import isaaclab.sim as sim_utils
  paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  sensors=[next(p for p in paths if '/LeftArm/' in p and p.endswith('/'+n)) for n in ['tip_left','tip_right','gripper','link5']]
  target=[next(p for p in paths if '/stapler/' in p and p.endswith('/frame'))]
  _VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[target]*4,max_contact_data_count=2048)
  print('HANDLE_CONTACT_PATHS '+json.dumps({'sensors':sensors,'target':target}),flush=True)
 if base._k%5==0:
  f=_VIEW.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy()
  print('HANDLE_CONTACT '+json.dumps({'step':base._k,'normal_force_N':f[:,0,:].tolist()}),flush=True)

@m.on_step
def battery_pose(env):
 global _BATTERY
 a=m.state(env,'stapler');b=m.state(env,'battery')
 _BATTERY=(b.pos,b.rotation(),b.joint('clip_0'),b.joint('clip_2'))
 if base._k%5:return
 print('BATTERY_ROBOT '+json.dumps({'step':base._k,'battery_in_frame':(a.rotation().T@(b.pos-a.pos)).tolist(),'battery_R_in_frame':(a.rotation().T@b.rotation()).tolist(),'clips':[b.joint('clip_0'),b.joint('clip_2')],'cover_m':a.joint('cover'),'release_m':a.joint('release')}),flush=True)

def carried_frame(L):
 return (L.pos+L.rot@_OFF_P,L.rot@_OFF_R) if base._POSES is None else base.frame()

def demo_script(ctx):
 yield from carry_script(ctx)
 L,R=ctx.left,ctx.right
 relative=_OFF_R if base._POSES is None else L.measured_rot.T@base.frame()[1]
 bodyR=axis_angle_matrix(np.array([0.,0.,1.]),np.radians(150.))
 yield from L.move((.25,.18,.95),4.,-.020,rot=bodyR@relative.T)
 yield from L.hold(1.)
 bp,br=carried_frame(L);rr=np.column_stack([br[:,2],-br[:,0],-br[:,1]])
 target=bp+br@np.array([.216,.252,0])
 yield from R.move(target+br[:,1]*.080,1.5,-.045,rot=rr)
 yield from R.hold(.3)
 for _ in range(3):
  bp,br=carried_frame(L);rr=np.column_stack([br[:,2],-br[:,0],-br[:,1]]);target=bp+br@np.array([.216,.252,0])+br[:,1]*.080
  measured=R.pos if base._POSES is None else R.measured_pos
  yield from R.move(R.pos+(target-measured)*.8,.6,-.045,rot=rr)
  yield from R.hold(.2)
 bp,br=carried_frame(L);target=bp+br@np.array([.216,.252,0])
 yield from R.move(target,1.5,-.045,rot=rr)
 yield from R.hold(.3)
 yield from R.grip(-.020,1.)
 yield from R.hold(.5)
 if _BATTERY is not None:
  print('BATTERY_GRIP_GATE '+json.dumps({'clips':list(_BATTERY[2:]),'min_pressed_deg':float(np.degrees(min(-_BATTERY[2],_BATTERY[3])))}),flush=True)
  if min(-_BATTERY[2],_BATTERY[3])<np.radians(7.5):return
 bp,br=carried_frame(L)
 yield from R.move(R.pos+br[:,1]*.080,3.,-.020,rot=R.rot.copy())
 yield from R.hold(2.)
