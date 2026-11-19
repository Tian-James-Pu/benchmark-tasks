"""Physical axial driver-reset diagnostic with completed v14 case/feed rails.

Privileged poses guide a scripted robot only. No object pose/velocity/force is
assigned by this controller. Native spring, geometry, and material are unchanged.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from contact_full_case_v14 import *
import axial_clearance_v15
_POSES=None;_k=0;_DRIVER=None;_CTX=None
from yam_arena import ik,station
@m.on_step
def poses(env):
 global _POSES,_k,_DRIVER
 _k+=1
 a=env.unwrapped.scene['screwdriver'];b=m.state(env,'stapler')
 origin=env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
 pose=a.data.root_link_pose_w[0].detach().cpu().numpy().astype(float)
 _POSES=(pose[:3]-origin,m.quat_matrix(pose[3:7]),b.pos,b.rotation());_DRIVER=b.joint('driver')
 robot=env.unwrapped.scene['right_robot'];rq=robot.data.joint_pos[0,:6].detach().cpu().numpy().astype(float)
 tcp,tcpR=ik.tcp_pose(rq,station.BASES['right'])
 if _k%10==0:
  tip=_POSES[0]+_POSES[1]@np.array([.10286,0,.01321])
  print('AXIAL_POSE '+json.dumps({'step':_k,'driver_m':b.joint('driver'),'tool_root':_POSES[0].tolist(),'tool_rotation':_POSES[1].tolist(),'tool_tip':tip.tolist(),'tip_in_frame':(_POSES[3].T@(tip-_POSES[2])).tolist(),'frame_pos':b.pos.tolist(),'frame_rotation':b.rotation().tolist(),'measured_tcp':tcp.tolist(),'measured_tcp_rotation':tcpR.tolist(),'native_tool_root_in_tcp':(tcpR.T@(_POSES[0]-tcp)).tolist(),'command_tcp':_CTX.right.pos.tolist() if _CTX is not None else None,'command_q':_CTX.right.q.tolist() if _CTX is not None else None,'measured_q':rq.tolist()}),flush=True)

def frame():
 return (NOSE,np.eye(3)) if _POSES is None else _POSES[2:]

def tip(R):
 return R.pos+np.array([.11286,0,-.026]) if _POSES is None else _POSES[0]+_POSES[1]@np.array([.10286,0,.01321])

def align(R):
 if _POSES is None:return R.rot.copy()
 _,body_R=frame();now=_POSES[1]
 targets=[body_R,body_R@np.diag([1.,-1.,-1.])]
 target=max(targets,key=lambda x:np.trace(x@now.T))
 return target@now.T@R.measured_rot

def demo_script(ctx):
 global _CTX
 _CTX=ctx
 L,R=ctx.left,ctx.right;O=-.04695;H=-.008
 fp,fr=frame();grip=fp+fr@np.array([.208,.130,.028])
 yield from L.move(grip+np.array([-.06,0,.08]),1.2,O)
 yield from L.move(grip+np.array([0,0,.015]),1.2,O)
 yield from L.move(grip,.5,O)
 yield from L.grip(-.017,.6)
 pick=np.array([TOOL[0]-.010,TOOL[1],TOP+.023])
 yield from R.move(pick+np.array([0,0,.11]),1.2,O)
 yield from R.move(pick,1.2,O)
 yield from R.grip(H,.6)
 yield from R.move(pick+np.array([0,0,.12]),1.2,H)
 yield from R.hold(.3)
 # Flat blade approximately across the feed dimension fits the central step;
 # the unchanged round shaft fits the photo-supported service opening.
 yield from R.move(R.pos,1.,H,rot=align(R))
 yield from R.hold(.2)
 yield from R.move((.245,-.110,TOP+.17),1.2,H,rot=R.rot.copy())
 for _ in range(3):
  fp,fr=frame();target=fp+fr@np.array([-.012,-.0008,0])
  yield from R.move(R.pos+target-tip(R),1.,H,rot=align(R))
  yield from R.hold(.2)
 fp,fr=frame();start=R.pos.copy();rot=R.rot.copy()
 for i in range(200):
  yield from R.move(start+fr@np.array([.0005*(i+1),0,0]),.05,H,rot=rot)
  if _DRIVER is not None and _DRIVER>=.0395:break
 print('AXIAL_PUSH_STOP '+json.dumps({'driver_m':_DRIVER,'command_travel_m':.0005*(i+1)}),flush=True)
 yield from R.hold(1.)
 yield from R.move(start,2.5,H,rot=rot)
 yield from R.move(R.pos+np.array([0,0,.08]),1.,H,rot=rot)
 yield from R.hold(.5)
