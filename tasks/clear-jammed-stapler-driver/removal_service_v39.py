"""Original hardware, compressor housing grasp nearer the center of mass.
No repair success claimed; instrument actual relative roll and bench support.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import access_axial_v30 as base
from access_axial_v30 import *
from yam_arena.demo import DOWN_GRIP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
import magnetic_return_v21
YAW=-90.
_REMOVAL_STAGE="reset"
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

# Full native driver return followed by real robot housing carry. Still a
# service-state diagnostic: magazine starts open and battery is already removed.
def demo_script(ctx):
 global _REMOVAL_STAGE
 yield from base.demo_script(ctx)
 L,R=ctx.left,ctx.right
 if base._DRIVER is not None and base._DRIVER<.039:raise RuntimeError('Driver return incomplete before removal')
 yield from R.hold(1.)
 yield from R.move(R.pos+np.array([-.030,-.120,0]),1.2,-.008,rot=R.rot.copy())
 yield from L.grip(-.04695,.6)
 yield from L.move(L.pos+np.array([0,0,.020]),.7,-.04695,rot=L.rot.copy())
 yield from L.move(L.pos+np.array([-.080,0,0]),1.,-.04695,rot=L.rot.copy())
 _REMOVAL_STAGE="carry"
 yield from carry_script(ctx)

 # Offline uses a recorded loaded housing grasp. Native execution remeasures
 # the actual body/gripper transform before each physical tilt.
 for angle,height in [(25.,1.00),(-25.,1.05)]:
  _REMOVAL_STAGE="tilt_"+str(angle)
  relative=np.array([[-0.9998284284871217, 0.0024079136132420043, -0.018366143709897125], [0.005704829240797364, 0.9833484884040483, -0.18164031825385976], [0.017622945436183805, -0.18171392978361567, -0.9831935105811987]]) if base._POSES is None else L.measured_rot.T@base.frame()[1]
  target=axis_angle_matrix(np.array([0.,1.,0.]),np.radians(angle))@axis_angle_matrix(np.array([1.,0.,0.]),np.radians(-35.))
  yield from L.move((.20,.10,height),3.,-.020,rot=target@relative.T)
  yield from L.hold(3.)

@m.on_step
def removal_observe(env):
 if base._CTX is None or base._k%10:return
 body=m.state(env,'stapler');wire=m.state(env,'bent_staple')
 print('REMOVAL_POSE '+json.dumps({'step':base._k,'stage':_REMOVAL_STAGE,'wire_in_body':(body.rotation().T@(wire.pos-body.pos)).tolist(),'wire_world':wire.pos.tolist(),'driver_m':body.joint('driver'),'body_pos':body.pos.tolist(),'body_R':body.rotation().tolist()}),flush=True)
