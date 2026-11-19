"""Ordinary interpolated 6-D TCP waypoints; no object poses are changed."""
from dataclasses import dataclass
import numpy as np
from yam_arena import demo as d,ik
from yam_arena.demo import Waypoint

@dataclass
class Oriented(Waypoint):
    rotation: object=None


def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        g=np.asarray(w.pos,float)
        q=np.asarray(w.rotation) if isinstance(w,Oriented) and w.rotation is not None else d.grip_rotation(w.yaw)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label
        t-=w.duration;p,r,f=g,q,w.finger
    return p,r,f,'done'
d.ArmPlan.target=target


def tilted(duration,pos,finger,label,pitch=15.):
    a=np.deg2rad(pitch);R=np.array([[1.,0.,0.],[0.,np.cos(a),-np.sin(a)],[0.,np.sin(a),np.cos(a)]])
    return Oriented(duration,tuple(pos),finger,label,np.pi/2,R@d.grip_rotation(np.pi/2))


def tcp_for_tip(point,pitch=15.):
    a=np.deg2rad(pitch);R=np.array([[1.,0.,0.],[0.,np.cos(a),-np.sin(a)],[0.,np.sin(a),np.cos(a)]])
    return np.asarray(point)-R@np.array([0.,.100,-.013])
