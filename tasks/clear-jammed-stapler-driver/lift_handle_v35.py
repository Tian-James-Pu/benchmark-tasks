"""Original hardware, upper handle grasp to reduce gravity moment.
No repair success claimed; instrument actual relative roll and bench support.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import access_axial_v18 as base
from access_axial_v18 import *
from yam_arena.demo import DOWN_GRIP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
YAW=-90.
@m.on_step
def left_grasp(env):
 if base._CTX is None:return
 a=base._CTX.left;p,r=base.frame();q=a.measured_rot
 if base._k==10:
  asset=env.unwrapped.scene['stapler'];print('NATIVE_MASS '+json.dumps({'body_names':asset.body_names,'mass_kg':m._np(asset.data.body_mass).tolist(),'local_com_m':m._np(asset.data.body_com_pos_b).tolist()}),flush=True)
 if base._k%10==0:print('HANDLE_GRASP '+json.dumps({'step':base._k,'frame_pos':p.tolist(),'frame_R':r.tolist(),'tcp':a.measured_pos.tolist(),'tcp_R':q.tolist(),'frame_in_tcp':(q.T@(p-a.measured_pos)).tolist(),'relative_R':(q.T@r).tolist(),'finger_q':env.unwrapped.scene['left_robot'].data.joint_pos[0,6:].detach().cpu().numpy().tolist()}),flush=True)

def demo_script(ctx):
 base._CTX=ctx;L=ctx.left
 p,r=base.frame();grip=p+r@np.array([.20419868637110017,.095,.010]);D=quat_xyzw_to_matrix(DOWN_GRIP)
 rot=r@axis_angle_matrix(np.array([1.,0.,0.]),np.radians(-10.))@axis_angle_matrix(np.array([0.,0.,1.]),np.radians(YAW))@D
 yield from L.move(grip+np.array([-.06,0,.08]),1.2,-.035,rot=rot)
 yield from L.move(grip+np.array([0,0,.035]),1.2,-.035,rot=rot)
 yield from L.hold(.4)
 yield from L.move(grip,1.0,-.035,rot=rot)
 yield from L.grip(-.010,.8)
 yield from L.hold(.5)
 yield from L.move(L.pos+np.array([-.100,.060,.130]),3.,-.010,rot=rot)
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
