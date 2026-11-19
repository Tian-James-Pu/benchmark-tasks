"""Pose-observation-based scripted tool control; robot joint actions only.

A scripted demonstration may observe object poses. The held-tool transform is
measured once after pickup; no simulated object pose or attachment is written.
"""
from dataclasses import dataclass
import numpy as np
import mechkit as m
from yam_arena import demo as d,ik,station
from yam_arena.demo import Waypoint

OBS={};CAL=None;COUNT=0

def rz(a):
    return np.array([[np.cos(a),-np.sin(a),0.],[np.sin(a),np.cos(a),0.],[0.,0.,1.]])
def rx(a):
    return np.array([[1.,0.,0.],[0.,np.cos(a),-np.sin(a)],[0.,np.sin(a),np.cos(a)]])

def reset(env=None):
    global CAL,COUNT
    CAL=None;COUNT=0;OBS.clear()

def observe(env):
    global CAL,COUNT
    COUNT+=1
    pick=m.state(env,'service_pick');hook=m.state(env,'spring_hook');saw=m.state(env,'saw')
    q=m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0,:6]
    tcp,rot=ik.tcp_pose(q,station.BASES['right'])
    OBS.update(pick=pick,hook=hook,saw=saw,tcp=tcp,tcp_rot=rot)
    if COUNT==115:
        CAL=(rot.T@(pick.pos-tcp),rot.T@pick.rotation())
        print('PICK_CALIBRATION',{'relative_position':CAL[0].tolist(),'relative_rotation':CAL[1].tolist()},flush=True)
m.on_reset(reset);m.on_step(observe)

@dataclass
class ToolPoint(Waypoint):
    tip: object=None
    pitch: float=15.


def tool_goal(w):
    r0=d.grip_rotation(np.pi/2);p0=rz(np.pi/2)
    relp,relr=CAL if CAL is not None else (r0.T@np.array([0.,0.,-.007]),r0.T@p0)
    tip=np.array(w.tip() if callable(w.tip) else w.tip,float)
    toolr=rx(np.deg2rad(w.pitch))@p0
    tcpr=toolr@relr.T
    tcpp=tip-toolr@np.array([.100,0.,-.006])-tcpr@relp
    return tcpp,tcpr


def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        if isinstance(w,ToolPoint):g,q=tool_goal(w)
        else:g,q=np.asarray(w.pos,float),d.grip_rotation(w.yaw)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label
        t-=w.duration;p,r,f=g,q,w.finger
    return p,r,f,'done'
d.ArmPlan.target=target


def tip_step(duration,tip,label,pitch=15.):
    return ToolPoint(duration,(0,0,0),0.,label,np.pi/2,tip,pitch)
