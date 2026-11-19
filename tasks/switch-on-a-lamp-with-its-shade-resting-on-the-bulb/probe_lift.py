import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *

def demo():
    Z=np.diag([-1.,-1.,1.]);R=Z@m.quat_matrix((.1331256926,.2467458844,.5069984198,.8150745630))
    p=np.array([*XY,TOP])+Z@(np.array((.3342469633,-.0350831263,.8352735043))-np.array([*XY,TOP]))
    target=p+R@np.array([SHADE_R1,0,SHADE_H+.004])
    g=Grip(0.);g.rot=np.column_stack((-R[:,0],R[:,1],-R[:,2]))
    high=target+np.array([0,0,.05]);lift=target+np.array([0,0,.12])
    out={}
    out['right']=[Waypoint(3.,high,-.023,'approach shade rim',g),
      Waypoint(2.,target,-.023,'straddle linen rim',g),
      Waypoint(1.,target,0.,'grasp bound rim',g),
      Waypoint(3.,lift,0.,'lift shade from harp',g),
      Waypoint(2.,lift,0.,'observe physical lift',g)]
    a=ArmPlan([],station.BASES['left']);r=Grip(0.);r.rot=a.start_rot
    out['left']=[Waypoint(11.,a.start_pos,-.04695,'idle',r)]
    return out
