"""Diagnostic physical pickup and keyed insertion; score remains zero.

The scripted controller observes simulator poses. It moves only robot joints;
no roller attachment, object pose write or pose-triggered catch is used.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
import probe_pick_v3 as pickup
from dataclasses import dataclass
from yam_arena import demo as d,ik,station
COUNT=0;SNAP={};LIVE={};ABORT=False;LAST={}

def observe(env):
 global COUNT,ABORT
 COUNT+=1
 v=m.state(env,'vacuum');r=m.state(env,'roller');LIVE.update(vacuum=v,roller=r)
 if COUNT==40:SNAP['initial']=r
 if COUNT==260:
  q=m._np(env.unwrapped.scene['left_robot'].data.joint_pos)[0,:6]
  tp,tr=ik.tcp_pose(q,station.BASES['left']);SNAP.update(lift=r,body=v,
   grip_pos=tr.T@(r.pos-tp),grip_rot=tr.T@r.rotation())
  rel=v.rotation().T@r.rotation();theta=math.atan2(rel[0,2],rel[0,0])
  socket=v.joint('motor_socket');SNAP['phase']=socket+round((theta-socket)/(np.pi/2))*(np.pi/2)
  dz=r.pos[2]-SNAP['initial'].pos[2]
  if dz<.025:ABORT=True
  if not ABORT and not select_phase(q):ABORT=True
  print('INSERT_CALIBRATION',COUNT,dict(lift_m=float(dz),abort=ABORT,
   body_pos=v.pos.tolist(),body_rot=v.rotation().tolist(),roller_pos=r.pos.tolist(),
   roller_rot=r.rotation().tolist(),grip_pos=SNAP['grip_pos'].tolist(),
   grip_rot=SNAP['grip_rot'].tolist(),phase=float(SNAP['phase'])),flush=True)
 if COUNT>=260 and COUNT%20==0:
  rel=v.to_local(r.pos)[0];rr=v.rotation().T@r.rotation()
  peg=rel+rr@np.array([0.,-.0985,0.]);bearing=rel+rr@np.array([0.,.096,0.])
  print('INSERT_STATE',COUNT,dict(center=rel.tolist(),peg_tip=peg.tolist(),
   bearing_center=bearing.tolist(),rotation=rr.tolist(),socket=v.joint('motor_socket'),
   outer_bearing=r.joint('outer_bearing'),abort=ABORT),flush=True)
m.on_step(observe)

@dataclass
class Point(Waypoint):goal:object=None

def target(self,t):
 if ABORT and id(self) in LAST:return LAST[id(self)]
 pos,rot,finger=self.start_pos,self.start_rot,self.finger_start
 for w in self.waypoints:
  p,r=w.goal() if isinstance(w,Point) else (np.asarray(w.pos,float),pickup._wp_rot(w))
  if t<w.duration:
   f=ik.smoothstep(t/w.duration)
   result=(pos+(p-pos)*f,ik.slerp_matrix(rot,r,f),finger+(w.finger-finger)*f,w.label)
   LAST[id(self)]=result;return result
  t-=w.duration;pos,rot,finger=p,r,w.finger
 return pos,rot,finger,'done'
d.ArmPlan.target=target

def point(seconds,finger,label,goal):return Point(seconds,(0.,0.,0.),finger,label,0.,goal)
def pickup_goal(dz):
 r=SNAP.get('initial');pos=np.array([.4278,.0052,.8531]) if r is None else r.pos
 a=math.radians(35);rot=pickup._rot((math.sin(a),0,-math.cos(a)),(-math.cos(a),0,-math.sin(a)))
 return pos+np.array([-.0056,-.0084,.0112+dz]),rot

def object_goal(stage):
 v=SNAP.get('body',LIVE.get('vacuum'))
 origin=C if v is None else v.pos;body=np.eye(3) if v is None else v.rotation()
 phase=SNAP.get('phase',0.);c,s=np.cos(phase),np.sin(phase)
 Ry=np.array([[c,0,s],[0,1,0],[-s,0,c]])
 tilt={0:0.,1:8.,2:3.,3:0.,4:0.}[stage];a=np.radians(tilt)
 Rx=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
 rel=Rx@Ry
 if stage==0:pos=np.array([AX,.01,AZ+.055])
 elif stage==4:pos=np.array([AX,0.,AZ])
 else:
  tip={1:np.array([AX,-.090,AZ+.004]),2:np.array([AX,-.0955,AZ]),3:np.array([AX,-.097,AZ])}[stage]
  pos=tip-rel@np.array([0.,-.0985,0.])
 return origin+body@pos,body@rel

def insertion_goal(stage):
 if 'grip_pos' not in SNAP:return pickup_goal(.05)
 p,r=object_goal(stage);rot=r@SNAP['grip_rot'].T
 return p-rot@SNAP['grip_pos'],rot


def select_phase(q_start):
 """Choose a physically equivalent square-key phase with a reachable wrist path."""
 initial=SNAP['phase'];candidates=[]
 for offset in (0,-1,1,2):
  SNAP['phase']=initial+offset*np.pi/2
  q=np.array(q_start,float);p0,r0=ik.tcp_pose(q,station.BASES['left'])
  pe_max=re_max=0.;margin=10.
  for stage in range(5):
   p1,r1=insertion_goal(stage)
   for f in np.linspace(0,1,9)[1:]:
    p=p0+(p1-p0)*f;r=ik.slerp_matrix(r0,r1,float(f))
    q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,station.BASES['left'],iters=60)
    pe_max=max(pe_max,pe);re_max=max(re_max,re)
    margin=min(margin,float(min(np.min(q-ik.LOWER),np.min(ik.UPPER-q))))
   p0,r0=p1,r1
  pitch=float(np.degrees(np.arccos(np.clip(-r0[2,2],-1,1))))
  candidates.append(dict(phase=float(SNAP['phase']),max_position_error_m=pe_max,
   max_rotation_error_deg=float(np.degrees(re_max)),minimum_joint_margin_rad=margin,
   final_approach_from_down_deg=pitch))
 feasible=[x for x in candidates if x['max_position_error_m']<.002 and x['max_rotation_error_deg']<2 and x['minimum_joint_margin_rad']>.02]
 if feasible:SNAP['phase']=min(feasible,key=lambda x:x['final_approach_from_down_deg'])['phase']
 else:SNAP['phase']=initial
 print('INSERT_PHASE_AUDIT',dict(candidates=candidates,chosen=SNAP['phase'],feasible=bool(feasible)),flush=True)
 return bool(feasible)

def withdraw_goal():
 p,r=insertion_goal(4);return p+np.array([0.,0.,.09]),r

def demo():
 a=math.radians(35)
 first=pickup.RotWaypoint(3.,(.36,.08,TOP+.23),-.027,'approach brush',0.,(math.sin(a),0,-math.cos(a)),(-math.cos(a),0,-math.sin(a)))
 plan=[first,point(3.,-.027,'above observed roller',lambda:pickup_goal(.07)),
  point(2.,-.027,'straddle observed roller',lambda:pickup_goal(0.)),
  point(1.,FINGER_CLOSED,'grip roller',lambda:pickup_goal(0.)),
  point(2.,FINGER_CLOSED,'lift roller',lambda:pickup_goal(.05)),
  point(2.,FINGER_CLOSED,'measure grip',lambda:pickup_goal(.05))]
 for stage,seconds,label in [(0,3.,'align key phase above bay'),(1,3.,'bring drive tip to mouth'),(2,2.,'enter key'),(3,2.,'lower bearing'),(4,2.,'seat brush')]:
  plan.append(point(seconds,FINGER_CLOSED,label,lambda stage=stage:insertion_goal(stage)))
 plan.extend([point(1.,FINGER_CLOSED,'settle seated brush',lambda:insertion_goal(4)),
  point(1.,-.027,'release brush',lambda:insertion_goal(4)),
  point(2.,-.027,'withdraw from brush',withdraw_goal)])
 return {'left':plan}
