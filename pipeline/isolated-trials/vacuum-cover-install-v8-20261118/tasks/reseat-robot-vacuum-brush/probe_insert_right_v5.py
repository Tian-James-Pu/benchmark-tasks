"""Physical drive insertion with a raised bearing-side approach and observed phase.
Uses observed-pose convergence before further lowering. Hardware is unchanged; no full repair is claimed.

Only robot commands are changed. The controller observes current output phase;
no object pose, joint, attachment or collision property is written.
"""
import sys,json,itertools
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_handoff_v3 import *
import probe_handoff_v3 as h
from scipy.spatial.transform import Rotation
STATE={};TRACE=[]
DURATIONS={'above':3.,'preentry':3.,'enter':.6,'lower':6.,'seat':1.5,'settle':2.,'release':1.,'withdraw':2.,'done':2.}

def ry(phi):return np.array([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
def rx(angle):return np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
def endpoint(stage):
 phi=STATE['phase'];r=ry(phi)
 if stage=='above':return np.array([AX,.01,AZ+.055]),r
 if stage in ['preentry','wait','enter']:
  r=rx(np.radians(6.))@r;y=.006 if stage in ['preentry','wait'] else .00015
  return np.array([AX,y,AZ+.0001+.096*np.sin(np.radians(6.))]),r
 if stage=='lower':return np.array([AX,0,AZ+.0001]),r
 return np.array([AX,0,AZ]),r

def wrapped_phase(angle):return (angle+np.pi/4)%(np.pi/2)-np.pi/4

def begin(stage):
 STATE.update(stage=stage,progress_s=0.,start=h.control.COUNT,from_p=STATE['goal_p'].copy(),from_r=STATE['goal_r'].copy())
 print('RIGHT_INSERT_STAGE',h.control.COUNT,stage,flush=True)

def initialize(env):
 v=m.state(env,'vacuum');roller=m.state(env,'roller');q=m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0,:6]
 tcp,r=ik.tcp_pose(q,station.BASES['right']);gp=r.T@(roller.pos-tcp);gr=r.T@roller.rotation()
 p0,r0=h.right_goal(2);localp=v.rotation().T@(p0+r0@gp-v.pos);localr=v.rotation().T@r0@gr
 rel=v.rotation().T@roller.rotation();phi=float(np.arctan2(rel[0,2],rel[0,0]));socket=v.joint('motor_socket')
 STATE.update(grip_p=gp,grip_r=gr,phase=phi,goal_p=localp,goal_r=localr,position_bias=np.zeros(3),rotation_bias=np.zeros(3),socket_previous=socket,socket_speed=0.,cmd=(p0,r0,FINGER_CLOSED,'start insertion'),minimum_phase_error=10.)
 begin('above')
 print('RIGHT_INSERT_INIT',json.dumps(dict(phase=phi,socket=socket,grip_p=gp.tolist(),grip_r=gr.tolist())),flush=True)

def trajectory_pose(stage,elapsed):
 targetp,targetr=endpoint(stage)
 if stage=='lower':
  f=ik.smoothstep(min(1.,elapsed/DURATIONS[stage]));angle=6*(1-f)
  knots=np.array([[0,0],[3.5,.000025],[3.7,.000025],[3.8,.00003],[4,.00003],[4.15,.00003],[4.2,.00005],[4.3,.00006],[4.5,.00008],[5,.00012],[6,.00015]])
  lp=np.array([AX,float(np.interp(angle,knots[:,0],knots[:,1])),AZ+.0001+.096*np.sin(np.radians(angle))]);lr=rx(np.radians(angle))@ry(STATE['phase'])
 elif stage in ['above','preentry','enter','seat']:
  f=ik.smoothstep(min(1.,elapsed/DURATIONS[stage]));lp=STATE['from_p']+(targetp-STATE['from_p'])*f;lr=ik.slerp_matrix(STATE['from_r'],targetr,f)
 else:lp,lr=targetp,targetr
 return lp,lr

def observe(env):
 n=h.control.COUNT
 if n<820:return
 if not STATE:initialize(env)
 v=m.state(env,'vacuum');roller=m.state(env,'roller');stage=STATE['stage'];elapsed=(n-STATE['start'])/20
 q=m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0,:6];actual_tcp,actual_r=ik.tcp_pose(q,station.BASES['right']);observed_grip=actual_r.T@(roller.pos-actual_tcp);grip_drift=float(np.linalg.norm(observed_grip-STATE['grip_p']));grip_turn=float(np.degrees(np.linalg.norm(ik._rot_error(STATE['grip_r'],actual_r.T@roller.rotation()))))
 if stage not in ['release','withdraw','done'] and (grip_drift>.008 or grip_turn>8.):h.fail(f'roller moved in right grip: {grip_drift:.6f}m/{grip_turn:.3f}deg')
 socket=v.joint('motor_socket');speed=(socket-STATE['socket_previous'])*20;STATE['socket_previous']=socket;STATE['socket_speed']=speed
 path_elapsed=STATE['progress_s'] if stage=='lower' else elapsed
 lp,lr=trajectory_pose(stage,path_elapsed)
 STATE.update(goal_p=lp,goal_r=lr)
 p=v.pos+v.rotation()@lp;r=v.rotation()@lr
 error=p-roller.pos;rot_error=ik._rot_error(r,roller.rotation())
 hand_force=sum(x['peak_normal_resultant_N'] for x in h.contact.LAST_CONTACTS if x['body'].startswith('vacuum/') and x['robot'].startswith(('LeftArm/','RightArm/')))
 if hand_force>10.:h.fail(f'robot/housing contact exceeded diagnostic10N guard: {hand_force:.3f}')
 force=sum(x['peak_normal_resultant_N'] for x in h.contact.LAST_CONTACTS if x['body'].startswith('roller/') and x['robot'].startswith('vacuum/'))
 if stage not in ['release','withdraw','done']:
  if force>30.:h.fail(f'roller/housing contact exceeded diagnostic30N guard: {force:.3f}')
  delta=.08*error;length=np.linalg.norm(delta)
  if length>.0003:delta*=.0003/length
  STATE['position_bias']=np.clip(STATE['position_bias']+delta,-.015,.015)
  delta=.08*rot_error;length=np.linalg.norm(delta)
  if length>np.radians(.15):delta*=np.radians(.15)/length
  rb=STATE['rotation_bias']+delta;length=np.linalg.norm(rb)
  if length>np.radians(6):rb*=np.radians(6)/length
  STATE['rotation_bias']=rb
 tcpR=Rotation.from_rotvec(STATE['rotation_bias']).as_matrix()@r@STATE['grip_r'].T
 tcp=p-tcpR@STATE['grip_p']+STATE['position_bias'];finger=FINGER_CLOSED
 if stage=='release':finger=FINGER_CLOSED+(-.022-FINGER_CLOSED)*ik.smoothstep(min(1.,elapsed))
 if stage in ['withdraw','done']:
  lift=.08*(ik.smoothstep(min(1.,elapsed/2)) if stage=='withdraw' else 1.);finger=-.022+(FINGER_OPEN+.022)*float(np.clip((lift-.03)/.05,0,1));tcp+=np.array([0,0,lift])
 STATE['cmd']=(tcp,tcpR,finger,'right insertion '+stage)
 # Advance the lowering path only after observed roller tracking converges.
 # Holding the target permits normal axial correction before more descent.
 local_error=v.rotation().T@error
 # The8mm key in the8.6mm bore allows0.3mm nominal radial clearance.
 # Reserve50um of that clearance; keep the constraining insertion axis
 # within50um and stop progression above3N rather than forcing the flange.
 aligned=(abs(local_error[1])<.00005 and np.max(np.abs(local_error[[0,2]]))<.00025
          and np.linalg.norm(rot_error)<np.radians(.2) and force<3.)
 if stage=='lower':
  if aligned:STATE['progress_s']=min(DURATIONS['lower'],STATE['progress_s']+.05)
  if elapsed>45.:h.fail('Lowering tracking did not converge within45s')
 if stage=='enter' and elapsed>10.:h.fail('Key entry tracking did not converge within10s')
 phase_error=wrapped_phase(socket+speed*.3-STATE['phase']);STATE['minimum_phase_error']=min(STATE['minimum_phase_error'],abs(phase_error))
 if n%10==0:
  print('RIGHT_INSERT_STATUS',json.dumps(dict(step=n,stage=stage,elapsed_s=elapsed,path_elapsed_s=path_elapsed,tracking_aligned=bool(aligned),local_error_m=local_error.tolist(),grip_drift_m=grip_drift,grip_turn_deg=grip_turn,position_error_m=float(np.linalg.norm(error)),rotation_error_deg=float(np.degrees(np.linalg.norm(rot_error))),socket_phase=socket,socket_speed=speed,predicted_key_error_deg=float(np.degrees(phase_error)),position_bias=STATE['position_bias'].tolist(),rotation_bias_deg=np.degrees(STATE['rotation_bias']).tolist(),roller_housing_normal_N=force,roller_in_body=v.to_local(roller.pos)[0].tolist(),bearing_joint=roller.joint('outer_bearing'))),flush=True)
 if stage=='wait':
  if np.linalg.norm(error)<.0005 and np.linalg.norm(rot_error)<np.radians(.8) and abs(phase_error)<np.radians(.5):begin('enter')
  elif elapsed>30.:h.fail('No well-aligned current output phase within30s observation window')
 elif (path_elapsed>=DURATIONS[stage] and (stage not in ['enter','lower'] or aligned)):
  next_stage={'above':'preentry','preentry':'wait','enter':'lower','lower':'seat','seat':'settle','settle':'release','release':'withdraw','withdraw':'done','done':'done'}[stage]
  if stage=='settle':
   center=v.to_local(roller.pos)[0];axis=v.rotation().T@roller.rotation()[:,1]
   relative=v.rotation().T@roller.rotation()
   corners=np.array(list(itertools.product([-.004,.004],[-.0985,-.0935],[-.004,.004])))
   key=(corners@relative.T+center-np.array([AX,-.099,AZ]))@ry(socket)
   bearing_center=center+relative@np.array([0,.096,0])
   bearing_rot=relative@ry(roller.joint('outer_bearing'))
   bearing=np.array(list(itertools.product([-.0045,.0045],[-.0025,.0025],[-.004,.004])))@bearing_rot.T+bearing_center
   checks=dict(center_error_m=float(np.linalg.norm(center-np.array([AX,0,AZ]))),axis_error_deg=float(np.degrees(np.arccos(np.clip(axis[1],-1,1)))),key_radial_extent_m=float(np.max(np.abs(key[:,[0,2]]))),key_tip_depth_m=float(.005-key[:,1].min()),bearing_x_extent_m=float(np.max(np.abs(bearing[:,0]-AX))),bearing_low_z_m=float(bearing[:,2].min()))
   print('RIGHT_INSERT_GEOMETRY',json.dumps(checks),flush=True)
   if checks['center_error_m']>.0006 or checks['axis_error_deg']>.5 or checks['key_radial_extent_m']>.0044 or checks['key_tip_depth_m']<.0035 or checks['bearing_x_extent_m']>.0049 or checks['bearing_low_z_m']<AZ-.0041 or checks['bearing_low_z_m']>AZ-.0034:h.fail('Brush key or bearing not seated; holding for diagnostic termination')
   print('RIGHT_INSERT_SEATED',json.dumps(dict(center=center.tolist(),phase_error_deg=float(np.degrees(wrapped_phase(socket-STATE['phase']))))),flush=True)
  if next_stage!=stage:begin(next_stage)
m.on_step(observe)

previous_target=d.ArmPlan.target
def target(self,time):
 if self.base_pos[1]<0 and time>=41. and STATE:return STATE['cmd']
 return previous_target(self,time)
d.ArmPlan.target=target

@m.on_reset
def reset(env=None):STATE.clear();TRACE.clear()

def demo():
 plans=h.demo();P=h.control.point
 plans['left'].append(P(75.,FINGER_OPEN,'keep left hand clear',lambda:h.left_goal(3)))
 plans['right'].append(P(75.,FINGER_CLOSED,'observed-phase insertion',lambda:h.right_goal(2)))
 return plans
