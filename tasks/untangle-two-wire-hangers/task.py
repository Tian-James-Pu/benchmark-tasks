"""Two full-size bent-wire hangers interlocked by an open hook.
All interactions are rigid-body contacts; no grasp detection drives objects.
"""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from simready import SimReady
from yam_arena.task_file import TABLE_TOP
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena import demo as _demo
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix
# Extend open-loop waypoints with explicit rotation matrices for the elevated hanger.
_original_grip_rotation = _demo.grip_rotation
def _grip_rotation(yaw=0.):
    return np.asarray(yaw).reshape(3,3) if isinstance(yaw,tuple) else _original_grip_rotation(yaw)
_demo.grip_rotation = _grip_rotation
SIDE = tuple((axis_angle_matrix(np.array([0.,1.,0]), -math.pi/3) @ _original_grip_rotation(math.pi/2)).ravel())
SCENE='station'
TOP=TABLE_TOP[SCENE]
INSTRUCTION='Separate the tangled hangers and hang the blue one on the empty peg.'
SUCCESS_HOLD_S=0.0
R=.002
LOOKS={'white':m.Look(color=m.rgb(.87,.88,.84)), 'blue':m.Look(color=m.rgb(.07,.28,.75)), 'rack':m.Look(color=m.rgb(.24,.25,.27),metallic=.7)}
# One closed triangular shoulder/crossbar loop plus the continuous open hook.
BODY=np.array([[-.2,0,0],[.2,0,0],[.012,0,.15],[0,0,.17],[-.012,0,.15],[-.2,0,0]])
HOOK=np.array([[0,0,.15]]+[[.027*math.cos(a),0,.205+.027*math.sin(a)] for a in np.linspace(-math.pi/2,-2*math.pi,29)])
def transform(points,white):
    return points[:,[1,0,2]] if white else points

def hanger(name,pos,white):
    paths=[transform(BODY,white),transform(HOOK,white)]
    parts=[]; cols=[]
    for j,path in enumerate(paths):
        parts.append((f'wire{j}',m.tube(path,R,10),name))
        for k,(a,b) in enumerate(zip(path[:-1],path[1:])):
            n=max(1,int(np.ceil(np.linalg.norm(b-a)/.020)))
            pp=np.linspace(a,b,n+1)
            for e,(aa,bb) in enumerate(zip(pp[:-1],pp[1:])):
                cols.append(m.col_hull(f'w{j}_{k}_{e}',m.tube([aa,bb],R,8)).replace('contactOffset = 0.004','contactOffset = 0.001'))
    quat=np.array([-.04841904,-.00917529,-.05837943,.99707729]) if white else np.array([-.12769566,.00773444,-.05995888,.98996919])
    quat=quat/np.linalg.norm(quat)
    return m.free(name,pos,parts,cols,LOOKS,mass=.070,friction=(.6,.45),torsional_patch=.005,solver_iters=64,start_quat=tuple(quat))
WHITE=hanger('white',(.30560401,-.03559732,.98793155),True)
BLUE=hanger('blue',(.31901082,.09372908,.74943447),False)
def rod(name,a,b,r=.004):
    return (name,m.tube([a,b],r,12),'rack'),m.col_hull(name+'_c',m.tube([a,b],r,12))
rackparts=[]; rackcols=[]
for name,a,b,r in [
 ('upper_post',(.49,.29,0),(.49,.29,.49),.008),
 ('upper_arm',(.49,.29,.465),(.49,-.02,.465),.006),
 ('occupied_peg',(.49,-.02,.465),(.26,-.02,.465),.004),
 ('lower_post',(.30,.39,0),(.30,.39,.27),.008),
 ('empty_peg',(.30,.39,.235),(.30,.21,.235),.004),
]:
 p,c=rod(name,a,b,r);rackparts.append(p);rackcols.append(c)
for k,x in enumerate((.285,.315)):
    collar=m.cylinder(.012,.004,(x,-.02,.465),axis='x')
    rackparts.append((f'collar{k}',collar,'rack'));rackcols.append(m.col_hull(f'collar{k}_c',collar))
RACK=m.fixed('rack',(0,0,TOP),rackparts,rackcols,LOOKS)
MUG=SimReady('mug',pos=(.57,.24,TOP),size=(.114,.09,.08),mass=.30,package='dishware_mug_a01',usd='sm_dishware_mug_a01_01.usda')
TAPE=SimReady('tape',pos=(.57,-.25,TOP),size=(.10,.10,.05),mass=.06,package='tape_clear_a01',usd='sm_tape_clear_a01_01.usd')
OBJECTS=[WHITE,BLUE,RACK,MUG,TAPE]
# Exact distances between the authored straight wire segments.
SEGMENTS={n:np.array([(a,b) for path in (transform(BODY,white),transform(HOOK,white)) for a,b in zip(path[:-1],path[1:])]) for n,white in [('white',True),('blue',False)]}
def point_gaps(points,segments):
    a=segments[:,0];u=segments[:,1]-a
    w=points[:,None,:]-a[None,:,:]
    t=np.clip(np.sum(w*u[None,:,:],axis=2)/np.sum(u*u,axis=1),0,1)
    return np.linalg.norm(w-t[:,:,None]*u[None,:,:],axis=2)
def wire_gap(A,B):
    u=A[:,1]-A[:,0];v=B[:,1]-B[:,0];w=A[:,None,0]-B[None,:,0]
    aa=np.sum(u*u,axis=1)[:,None];cc=np.sum(v*v,axis=1)[None,:]
    bb=u@v.T;dd=np.sum(u[:,None,:]*w,axis=2);ee=np.sum(v[None,:,:]*w,axis=2)
    den=aa*cc-bb*bb;safe=np.where(den>1e-16,den,1.)
    ss=(bb*ee-cc*dd)/safe;tt=(aa*ee-bb*dd)/safe
    valid=(den>1e-16)&(ss>=0)&(ss<=1)&(tt>=0)&(tt<=1)
    inner=np.linalg.norm(w+ss[:,:,None]*u[:,None,:]-tt[:,:,None]*v[None,:,:],axis=2)
    return min(float(np.min(point_gaps(A.reshape(-1,3),B))),float(np.min(point_gaps(B.reshape(-1,3),A))),float(np.min(np.where(valid,inner,np.inf))))-2*R
class Trial:
 def reset(self,env=None):
    self.harm=False; self.hold=0; self.full=False; self.value=0.; self.steps=0
 def step(self,env):
    self.steps+=1
    states={n:m.state(env,n) for n in SEGMENTS}
    segments={n:s.to_world(SEGMENTS[n].reshape(-1,3)).reshape(-1,2,3) for n,s in states.items()}
    pts={n:seg.reshape(-1,3) for n,seg in segments.items()}
    tips=m.fingertips(env)
    # Leaving the supported work area is a permanent failure. Rigid wire cannot be torn or bent.
    for p in pts.values():
      if np.any(p[:,2]<TOP-.04):self.harm=True
    separation=wire_gap(segments['white'],segments['blue'])
    off=all(float(np.min(point_gaps(tips,seg)))-R>.03 for seg in segments.values())
    resting=all(np.linalg.norm(s.lin_vel)<.015 and np.linalg.norm(s.ang_vel)<.12 for s in states.values())
    # Direct geometry: each peg must pass through the interior of that hanger's hook and the hanger must hang vertically.
    seats=[]
    for n,endpoints,white in [('white',[(.26,-.02,TOP+.465),(.49,-.02,TOP+.465)],True),('blue',[(.30,.39,TOP+.235),(.30,.21,TOP+.235)],False)]:
      st=states[n];line=st.to_local(np.array(endpoints));axis=0 if white else 1
      delta=line[1]-line[0];den=delta[axis]
      t=-line[0,axis]/den if abs(den)>1e-8 else -1.
      local=line[0]+t*delta;u=local[1] if white else local[0]
      seats.append(0.03<t<.97 and abs(u)<.016 and .215<local[2]<.229 and st.tilt_deg()<12 and float(np.min(pts[n][:,2]))>TOP+.005)
    good=separation>.03 and all(seats) and off and resting
    self.hold=self.hold+1 if good and not self.harm else 0
    self.full=self.hold>=60 and not self.harm
    self.value=0. if self.harm else (1. if self.full else (.5 if separation>.03 and off and resting else 0.))
    if self.steps in (1,40,100,250,400,600):
      print('HANGER_STATE',self.steps,{n:(s.pos.tolist(),s.quat_xyzw.tolist(),s.lin_vel.tolist(),s.ang_vel.tolist()) for n,s in states.items()},'sep',separation,'seats',seats,'off',off,flush=True)
TRIAL=Trial();TRIAL.reset();m.on_reset(TRIAL.reset);m.on_step(TRIAL.step)
def score(objects):return float(TRIAL.value)
def success(objects):return bool(TRIAL.full and not TRIAL.harm)
def demo():
    # Fixed waypoints calculated from the published settled-layout constants, never observed state.
    q=np.array([-.11802126,.00982829,-.08240226,.98953736]);q/=np.linalg.norm(q)
    rb=quat_xyzw_to_matrix(q)
    p=np.array([.32026294,.09987739,.74944389])+rb@np.array([-.08,0,.0957447])
    side=axis_angle_matrix(np.array([1.,0,0]),-5*math.pi/12)@_original_grip_rotation(0.)
    grip=tuple(side.ravel())
    p=p+np.array([0,0,0])
    clear=p+rb@np.array([-.045,0,.060])
    orient=tuple((rb.T@side).ravel())
    support=(.286,-.13,TOP+.254)
    def w(t,p,f,label,yaw=0):return Waypoint(t,tuple(p),f,label,yaw)
    return {
      'left':[w(1.5,(.16,.30,TOP+.14),FINGER_OPEN,'route behind white'),w(1.5,(p[0],p[1]+.09,p[2]),FINGER_OPEN,'approach blue shoulder',grip),w(1,p,FINGER_OPEN,'at blue shoulder',grip),w(1,p,FINGER_CLOSED,'grasp blue',grip),w(1,p,FINGER_CLOSED,'wait for white',grip),w(3,clear,FINGER_CLOSED,'guide wire through open hook throat',grip),w(3,(.175,.282,TOP+.15),FINGER_CLOSED,'carry clear',orient),w(2,(.22,.282,TOP+.15),FINGER_CLOSED,'over empty peg',orient),w(2,(.22,.282,TOP+.111),FINGER_CLOSED,'lower hook onto peg',orient),w(1,(.22,.282,TOP+.111),FINGER_OPEN,'release blue',orient),w(1,(.22,.35,TOP+.111),FINGER_OPEN,'back away',orient),w(1,(.22,.35,TOP+.111),FINGER_OPEN,'clear',orient),w(10,(.22,.35,TOP+.111),FINGER_OPEN,'wait',orient)]
    }
