"""Inactive robot-only gravity removal plan through the existing open feed path."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import access_axial_v18 as base
from access_axial_v18 import *
from yam_arena.demo import DOWN_GRIP
from yam_arena.urdf import quat_xyzw_to_matrix,axis_angle_matrix

def demo_script(ctx):
 yield from base.demo_script(ctx)
 L,R=ctx.left,ctx.right;H=-.008
 # Withdraw clear of the service mouth before returning to the pickup attitude.
 yield from R.move(R.pos+np.array([-.025,0,0]),.8,H,rot=R.rot.copy())
 yield from R.move(R.pos,.8,H,rot=quat_xyzw_to_matrix(DOWN_GRIP))
 yield from R.move(R.pos+np.array([0,0,.12]),1.,H,rot=R.rot.copy())
 yield from R.move((.17,-.30,TOP+.17),1.5,H,rot=R.rot.copy())
 fp,fr=base.frame();lp=L.pos if base._POSES is None else L.measured_pos
 lr=L.rot if base._POSES is None else L.measured_rot
 offset=lr.T@(fp-lp);relative=lr.T@fr
 yield from L.move((.20,.10,.90),2.,-.017,rot=L.rot.copy())
 for angle,height in ((45.,.90),(-45.,1.05)):
  target=axis_angle_matrix(np.array([0.,1.,0.]),np.radians(angle))@axis_angle_matrix(np.array([1.,0.,0.]),np.radians(-35.))
  yield from L.move((.20,.10,height),2.,-.017,rot=target@relative.T)
  yield from L.hold(3.)
