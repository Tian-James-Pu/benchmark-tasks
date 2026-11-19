"""Held-after-retention scorer control; initially repaired fixture.

This is not a repair demonstration. It compares velocity and bounded-pose
stillness observers, then holds and releases a repaired assembly. All object
motion comes from the unchanged robot controls and contacts.
"""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation
import mechkit as m
import runtime
import retention_video_view
_f=m._Finder.targets['yam_arena.scene']
def coarser(scene):_f(scene);scene.PHYSICS_DT=1./480.
m._Finder.targets['yam_arena.scene']=coarser
from crutch_handle_layout import part,POS,Q,R,AXIS,TOP
from ferrule_axial import make
from yam_arena import demo as d,ik,station
from yam_arena.demo import Waypoint
SCENE='station';INSTRUCTION='Observe unsupported ferrule retention.';SUCCESS_HOLD_S=0.
OBJECTS=[part(),make('new_ferrule',POS,Q),make('old_ferrule',(.35,-.42,TOP+.019),Q,worn=True)]
from distractors import objects as workshop_objects
OBJECTS+=workshop_objects()
from scoring_guarded_draft import Score
from support_contacts_guarded_draft import reported
OBJECTS=[reported(obj) if obj.name in ('crutch','new_ferrule','old_ferrule') else obj for obj in OBJECTS]
from scoring_v4 import Score as LegacyScore
result=Score();legacy=LegacyScore()
DATA={};COUNT=0
def reset(env=None):
    global COUNT
    COUNT=0;DATA.clear()
def observe(env):
    global COUNT
    COUNT+=1;c=m.state(env,'crutch');f=m.state(env,'new_ferrule')
    if COUNT==20:DATA['initial']=c
    if COUNT==130:
        for side in ('left','right'):
            q=m._np(env.unwrapped.scene[side+'_robot'].data.joint_pos)[0,:6]
            p,r=ik.tcp_pose(q,station.BASES[side])
            DATA[side+'_cal']=(r.T@(c.pos-p),r.T@c.rotation())
        DATA['baseline']=c
    if 210<=COUNT<=360 or COUNT%20==0:
        print('POSE_OBSERVER_COMPARE',COUNT,{'candidate':result.last,'legacy':legacy.last,'crutch_pos':c.pos.tolist(),'crutch_rotation':c.rotation().tolist(),'foot_pos':f.pos.tolist(),'foot_rotation':f.rotation().tolist(),'crutch_speed':float(np.linalg.norm(c.lin_vel)),'crutch_omega':float(np.linalg.norm(c.ang_vel)),'foot_speed':float(np.linalg.norm(f.lin_vel)),'foot_omega':float(np.linalg.norm(f.ang_vel)),'minimum_finger_distance_m':float(np.linalg.norm(m.fingertips(env)-f.pos,axis=1).min())},flush=True)
    if COUNT==1 or COUNT%10==0:
        rel=c.to_local(f.pos)[0];axis=c.rotation()[:,2]
        floor=f.pos[2]-.019*np.sqrt(max(0,1-f.rotation()[2,2]**2))
        print('UNSUPPORTED',COUNT,{'relative_m':rel.tolist(),'shaft_up_deg':math.degrees(math.asin(float(np.clip(axis[2],-1,1)))),'foot_floor_clearance_m':float(floor-TOP),'foot':f.pos.tolist(),'crutch':c.pos.tolist(),'clear_new_distance_m':float(np.linalg.norm(m.fingertips(env)-f.pos,axis=1).min()),'crutch_speed':float(np.linalg.norm(c.lin_vel)),'crutch_omega':float(np.linalg.norm(c.ang_vel)),'foot_speed':float(np.linalg.norm(f.lin_vel)),'foot_omega':float(np.linalg.norm(f.ang_vel))},flush=True)
m.on_reset(reset);m.on_step(observe)
@dataclass
class Point(Waypoint):goal:object=None
def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        g,q=w.goal() if isinstance(w,Point) else (np.asarray(w.pos,float),d.grip_rotation(w.yaw))
        if t<w.duration:
            a=ik.smoothstep(t/w.duration);return p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label
        t-=w.duration;p,r,f=g,q,w.finger
    return p,r,f,'done'
d.ArmPlan.target=target
def initial(dz,side):
    c=DATA.get('initial');p=POS if c is None else c.pos;r=R if c is None else c.rotation()
    return p+r@np.array([0,0,.30 if side=='left' else .12])+[0,0,dz],d.grip_rotation(math.atan2(r[1,2],r[0,2]))
def tilted(frac,side):
    c=DATA.get('baseline');p=POS+[0,0,.06] if c is None else c.pos;r=R if c is None else c.rotation()
    axis=r[:,2];horizontal=axis.copy();horizontal[2]=0.;horizontal/=np.linalg.norm(horizontal)
    n=np.cross(horizontal,[0,0,1.]);start_angle=math.asin(float(np.clip(axis[2],-1,1)))
    a=Rotation.from_rotvec([0,0,math.radians(-53.291155)*frac]).as_matrix()@Rotation.from_rotvec(n*(math.radians(70)-start_angle)*frac).as_matrix()
    rr=a@r;pp=p*(1-frac)+np.array([.10,.20,.82])*frac
    gr=d.grip_rotation(.927295)
    relp,relr=DATA.get(side+'_cal',(-gr.T@(AXIS*.30+[0,0,.007]),gr.T@R))
    tcp_r=rr@relr.T
    return pp-tcp_r@relp,tcp_r
def returned(frac,side):
    c=DATA.get('baseline');p=POS+[0,0,.06] if c is None else c.pos;r=R if c is None else c.rotation()
    axis=r[:,2];horizontal=axis.copy();horizontal[2]=0.;horizontal/=np.linalg.norm(horizontal)
    n=np.cross(horizontal,[0,0,1.]);start_angle=math.asin(float(np.clip(axis[2],-1,1)))
    a=Rotation.from_rotvec([0,0,math.radians(-53.291155)]).as_matrix()@Rotation.from_rotvec(n*(math.radians(70)-start_angle)).as_matrix()
    init=DATA.get('initial');ip=POS if init is None else init.pos;ir=R if init is None else init.rotation()
    rr=ik.slerp_matrix(a@r,ir,frac);pp=np.array([.10,.20,.82])*(1-frac)+ip*frac
    gr=d.grip_rotation(1.1592794807)
    relp,relr=DATA.get(side+'_cal',(-gr.T@(AXIS*.30+[0,0,.007]),gr.T@R))
    tcp_r=rr@relr.T
    return pp-tcp_r@relp,tcp_r
def step(t,dz,f,label,side):return Point(t,(0,0,0),f,label,.927295,lambda:initial(dz,side))
def score(objects):return float(result.value)
def success(objects):return bool(result.value==1.)
def demo():
    result={}
    for side in ('left','right'):
        plan=[step(2.,.10,-.04695,'approach shaft',side),step(1.5,.007,-.04695,'reach shaft',side),
              step(1.,.007,-.006,'grip shaft',side),step(1.5,.067,-.006,'lift crutch',side),step(1.,.067,-.006,'observe held pose',side)]
        for frac in np.linspace(.1,1.,10):plan.append(Point(.4,(0,0,0),-.006,'tilt full crutch',.927295,lambda f=frac,s=side:tilted(f,s)))
        plan.append(Point(6.,(0,0,0),-.006,'observe unsupported foot',.927295,lambda s=side:tilted(1.,s)))
        for frac in np.linspace(.1,1.,10):plan.append(Point(.4,(0,0,0),-.006,'return while holding',1.1592794807,lambda f=frac,s=side:returned(f,s)))
        plan.append(Point(3.,(0,0,0),-.006,'keep fingers on resting shaft',1.1592794807,lambda s=side:returned(1.,s)))
        plan.extend([Point(1.,(0,0,0),-.04695,'release resting shaft',1.1592794807,lambda s=side:returned(1.,s)),step(2.,.20,-.04695,'clear released crutch',side),step(4.,.20,-.04695,'observe released assembly',side)])
        result[side]=plan
    return result
