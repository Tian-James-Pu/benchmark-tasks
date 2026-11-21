"""Policy-only full-orientation waypoints; native IK/action limits are retained."""
from dataclasses import dataclass
import numpy as np
from yam_arena import demo,ik
@dataclass
class Pose:
    duration:float
    pos:tuple
    finger:float
    label:str
    rotation:object
    yaw:float=0.
    on_enter:object=None
_old=demo.ArmPlan.target

def target(plan,t):
    if not plan.waypoints or not isinstance(plan.waypoints[0],Pose):return _old(plan,t)
    ends=np.cumsum([w.duration for w in plan.waypoints])
    k=int(np.searchsorted(ends,t,side='right'))
    if k==len(ends):
        w=plan.waypoints[-1];return np.array(w.pos),w.rotation,w.finger,'finished'
    w=plan.waypoints[k]
    if getattr(plan,'_pose_phase',None)!=k:
        plan._pose_phase=k
        if w.on_enter is not None:w.on_enter()
    elapsed=0. if k==0 else ends[k-1]
    previous=plan.waypoints[k-1] if k else None
    p0=np.array(previous.pos) if previous else plan.start_pos
    r0=previous.rotation if previous else plan.start_rot
    f0=previous.finger if previous else plan.finger_start
    s=ik.smoothstep((t-elapsed)/w.duration)
    return (1-s)*p0+s*np.array(w.pos),ik.slerp_matrix(r0,w.rotation,s),(1-s)*f0+s*w.finger,w.label

demo.ArmPlan.target=target
