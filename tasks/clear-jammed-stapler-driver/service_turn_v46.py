"""Robot isolation then bimanual service reorientation; unchanged hardware."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import isolation_park_v45 as prior
from isolation_park_v45 import *
from yam_arena.urdf import axis_angle_matrix

def demo_script(ctx):
 yield from prior.demo_script(ctx)
 if prior._BAT is not None and np.linalg.norm(prior._BAT.pos-base.frame()[0])<.100:return
 L,R=ctx.left,ctx.right;O=-.045
 p,B=base.frame();left_local=np.array([.115,.155,0.]);right_local=np.array([.213,.160,0.])
 down=axis_angle_matrix(np.array([0.,1.,0.]),np.radians(-30))@np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])@np.diag([-1.,-1.,1.])
 above=-down[:,2]*.080
 lp=p+B@left_local;rp=p+B@right_local
 yield from ctx.both(L.move(lp+above+np.array([0,.080,0.]),2.,O,rot=down),R.move(rp+above+np.array([0,-.080,0.]),2.,O,rot=down))
 yield from ctx.both(L.move(lp+above,1.,O,rot=down),R.move(rp+above,1.,O,rot=down))
 yield from ctx.both(L.move(lp,2.,O,rot=down),R.move(rp,2.,O,rot=down))
 yield from ctx.both(L.grip(-.020,1.),R.grip(-.012,1.))
 yield from L.hold(.7)
 yield from ctx.both(L.move(L.pos+np.array([0,0,.050]),2.5,-.020,rot=L.rot),R.move(R.pos+np.array([0,0,.050]),2.5,-.012,rot=R.rot))
 yield from L.hold(.7)
 p,B=base.frame()
 if base._POSES is None:p=p+np.array([0,0,.050])
 local=[]
 for arm in [L,R]:
  ap=arm.pos if base._POSES is None else arm.measured_pos
  ar=arm.rot if base._POSES is None else arm.measured_rot
  local.append((B.T@(ap-p),B.T@ar))
 target=np.array([.180,-.200,TOP+.100])
 for i in range(120):
  f=(i+1)/120;s=f*f*(3-2*f);A=axis_angle_matrix(np.array([1.,0.,0.]),-np.pi/2*s);BR=A@B;BP=p*(1-s)+target*s
  yield from ctx.both(L.move(BP+BR@local[0][0],.05,-.020,rot=BR@local[0][1]),R.move(BP+BR@local[1][0],.05,-.012,rot=BR@local[1][1]))
 yield from L.hold(1.)
 yield from ctx.both(L.move(L.pos+np.array([0,0,-.0593]),2.,-.020,rot=L.rot),R.move(R.pos+np.array([0,0,-.0593]),2.,-.012,rot=R.rot))
 yield from L.hold(1.)
 yield from ctx.both(L.grip(O,.6),R.grip(O,.6))
 yield from ctx.both(L.move(L.pos+np.array([0,.080,.080]),1.5,O,rot=L.rot),R.move(R.pos+np.array([0,.080,.080]),1.5,O,rot=R.rot))
 yield from L.hold(2.)
