"""Score-disabled physical brush handoff; no attachment or object state writes."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import probe_pick_contacts_v4 as contact
import probe_insert_v1 as control
from probe_insert_v1 import *
OBJECTS=contact.OBJECTS
OFFSET=.045
_old_pickup=control.pickup_goal
RIGHT_GRIP={};ABORT_REASON=None

def pickup_goal(dz):
 p,r=_old_pickup(dz)
 initial=control.SNAP.get('initial')
 axis=np.array([0,1,0]) if initial is None else initial.rotation()[:,1]
 return p+OFFSET*axis,r
control.pickup_goal=pickup_goal

def desired_rotation():
 body=control.SNAP.get('body',control.LIVE.get('vacuum'))
 br=np.eye(3) if body is None else body.rotation()
 phi=control.SNAP.get('phase',0.);c,s=np.cos(phi),np.sin(phi)
 return br@np.array([[c,0,s],[0,1,0],[-s,0,c]])

def left_object_goal(stage):
 lifted=control.SNAP.get('lift')
 p=np.array([.427,.005,.88]) if lifted is None else lifted.pos
 r=np.eye(3) if lifted is None else lifted.rotation()
 if stage==0:return p+np.array([0,0,.06]),r
 if stage==1:return p+np.array([0,0,.06]),desired_rotation()
 return np.array([.30,0,.96]),desired_rotation()

def left_goal(stage):
 if 'grip_pos' not in control.SNAP:return pickup_goal(.05)
 p,r=left_object_goal(stage);rot=r@control.SNAP['grip_rot'].T
 tcp=p-rot@control.SNAP['grip_pos']
 if stage==3:tcp=tcp+np.array([-.06,.10,.015])
 return tcp,rot

def right_goal(stage):
 a=np.radians(25.);z=np.array([np.sin(a),0,-np.cos(a)]);x=np.array([np.cos(a),0,np.sin(a)])
 r=np.column_stack((x,np.cross(z,x),z))
 p=np.array([.30,0,.96])+desired_rotation()@np.array([0,-.065,0])-r@np.array([0,0,.023])
 if stage==0:p+=np.array([0,-.08,0])
 if stage==2:p+=np.array([0,0,.035])
 return p,r

def select_transfer_phase(q_start):
 initial=control.SNAP['phase'];candidates=[]
 for offset in [0,-1,1,2]:
  control.SNAP['phase']=initial+offset*np.pi/2
  q=np.array(q_start);p0,r0=ik.tcp_pose(q,station.BASES['left']);max_pe=max_re=0.
  for stage in [0,1,2,3]:
   p1,r1=left_goal(stage)
   for f in np.linspace(0,1,13)[1:]:
    p=p0+(p1-p0)*f;r=ik.slerp_matrix(r0,r1,float(f))
    q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,station.BASES['left'],iters=80)
    max_pe=max(max_pe,pe);max_re=max(max_re,re)
   p0,r0=p1,r1
  candidates.append(dict(phase=control.SNAP['phase'],offset=offset,position_error_m=max_pe,rotation_error_deg=float(np.degrees(max_re))))
 good=[x for x in candidates if x['position_error_m']<.002 and x['rotation_error_deg']<2]
 control.SNAP['phase']=min(good,key=lambda x:abs(x['offset']))['phase'] if good else initial
 print('HANDOFF_PHASE_AUDIT',dict(candidates=candidates,chosen=control.SNAP['phase']),flush=True)
 return bool(good)
control.select_phase=select_transfer_phase

def fail(reason):
 global ABORT_REASON
 ABORT_REASON=reason;print('HANDOFF_ABORT',control.COUNT,reason,flush=True)
 raise RuntimeError('Handoff diagnostic guard: '+reason)

_original_action7=d.ArmPlan.action7
def guarded_action7(self,time):
 action,label=_original_action7(self,time)
 p,r,_,_=self.target(time);actual_p,actual_r=ik.tcp_pose(action[:6],self.base_pos)
 pe=float(np.linalg.norm(actual_p-p));re=float(np.linalg.norm(ik._rot_error(r,actual_r)))
 if pe>.002 or re>np.radians(2):fail(f'commanded IK residual {pe:.6f}m/{np.degrees(re):.3f}deg')
 if control.COUNT%20==0:print('HANDOFF_COMMAND',json.dumps(dict(step=control.COUNT,base=self.base_pos.tolist(),tcp=p.tolist(),q=action[:6].tolist(),pe_m=pe,re_deg=float(np.degrees(re)),label=label)),flush=True)
 return action,label
d.ArmPlan.action7=guarded_action7

def observe_handoff(env):
 n=control.COUNT
 if control.ABORT:fail('pickup or phase guard failed')
 if n==440:
  roller=m.state(env,'roller');p,r=left_object_goal(2)
  pe=float(np.linalg.norm(roller.pos-p));re=float(np.linalg.norm(ik._rot_error(r,roller.rotation())))
  print('HANDOFF_ALIGNMENT',dict(position_error_m=pe,rotation_error_deg=float(np.degrees(re))),flush=True)
  if pe>.008 or re>np.radians(8):fail('left held brush did not reach transfer pose')
 if n in (600,680,760):
  q=m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0,:6]
  p,r=ik.tcp_pose(q,station.BASES['right']);roller=m.state(env,'roller')
  grip=r.T@(roller.pos-p);rot=r.T@roller.rotation()
  if n==600:
   pairs=contact.LAST_CONTACTS
   forces={tip:max([x['peak_normal_resultant_N'] for x in pairs if x['body']=='roller/frame' and x['robot']=='RightArm/'+tip],default=0.) for tip in ['tip_left','tip_right']}
   print('HANDOFF_RIGHT_CONTACT',forces,flush=True)
   if min(forces.values())<.5:fail('no two-finger roller contact before left release')
   RIGHT_GRIP.update(position=grip,rotation=rot,roller_position=roller.pos.copy())
  else:
   displacement=float(np.linalg.norm(grip-RIGHT_GRIP['position']))
   print('HANDOFF_RETAINED',n,dict(grip_displacement_m=displacement,lift_m=float(roller.pos[2]-RIGHT_GRIP['roller_position'][2])),flush=True)
   if displacement>.008:fail('brush slipped from right gripper after left release')
  print('HANDOFF_RIGHT_POSE',n,dict(tcp=p.tolist(),roller=roller.pos.tolist(),grip_position=grip.tolist(),grip_rotation=rot.tolist()),flush=True)
m.on_step(observe_handoff)

def reset(env=None):
 global ABORT_REASON
 RIGHT_GRIP.clear();ABORT_REASON=None
m.on_reset(reset)

def demo():
 P=control.point
 left=control.demo()['left'][:6]
 left.extend([P(3.,FINGER_CLOSED,'raise held brush',lambda:left_goal(0)),P(3.,FINGER_CLOSED,'align drive phase in free space',lambda:left_goal(1)),P(3.,FINGER_CLOSED,'carry to transfer space',lambda:left_goal(2)),P(8.,FINGER_CLOSED,'hold during right grasp',lambda:left_goal(2)),P(1.,FINGER_OPEN,'release left grip',lambda:left_goal(2)),P(3.,FINGER_OPEN,'withdraw left hand',lambda:left_goal(3)),P(4.,FINGER_OPEN,'hold clear',lambda:left_goal(3))])
 ready=ik.tcp_pose(d.READY,station.BASES['right'])
 right=[P(22.,FINGER_OPEN,'wait at ready',lambda:ready),P(3.,FINGER_OPEN,'approach from drive end',lambda:right_goal(0)),P(3.,FINGER_OPEN,'straddle free roller end',lambda:right_goal(1)),P(1.,FINGER_CLOSED,'grip brush',lambda:right_goal(1)),P(5.,FINGER_CLOSED,'hold during left release',lambda:right_goal(1)),P(2.,FINGER_CLOSED,'lift transferred brush',lambda:right_goal(2)),P(2.,FINGER_CLOSED,'inspect independent right hold',lambda:right_goal(2))]
 return dict(left=left,right=right)
