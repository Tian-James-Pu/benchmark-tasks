"""Keep both original robot arms at their existing READY targets for5seconds."""
from yam_arena import demo as d,ik,station

def hold_target(self,time):
    return self.start_pos,self.start_rot,d.FINGER_OPEN,'read-only support calibration'
d.ArmPlan.target=hold_target

def demo():
    return {side:[d.Waypoint(5.,tuple(ik.tcp_pose(d.READY,station.BASES[side])[0]),d.FINGER_OPEN)] for side in ('left','right')}
