"""Original-mass handle grasp/lift gate, no repair success claimed."""
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
 if base._k%10==0:print('HANDLE_GRASP '+json.dumps({'step':base._k,'frame_pos':p.tolist(),'frame_R':r.tolist(),'tcp':a.measured_pos.tolist(),'tcp_R':q.tolist(),'frame_in_tcp':(q.T@(p-a.measured_pos)).tolist(),'relative_R':(q.T@r).tolist()}),flush=True)

def demo_script(ctx):
 base._CTX=ctx;L=ctx.left
 p,r=base.frame();grip=p+r@np.array([.208,.130,.028]);D=quat_xyzw_to_matrix(DOWN_GRIP)
 rot=r@axis_angle_matrix(np.array([0.,0.,1.]),np.radians(YAW))@D
 yield from L.move(grip+np.array([-.06,0,.08]),1.2,-.030,rot=rot)
 yield from L.move(grip+np.array([-.020,0,.025]),1.2,-.030,rot=rot)
 yield from L.move(grip,.7,-.030,rot=rot)
 yield from L.grip(-.010,.8)
 yield from L.hold(.5)
 yield from L.move(L.pos+np.array([-.100,.060,.130]),3.,-.010,rot=rot)
 yield from L.hold(2.)
