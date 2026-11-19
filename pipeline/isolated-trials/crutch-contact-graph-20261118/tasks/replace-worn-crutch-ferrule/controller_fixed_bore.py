"""Robot-only repair preserving both measured shaft-to-gripper transforms.

A shared object rotation passes through a reachable intermediate pose before
vertical bench-supported insertion. No independent wrist rotation is commanded.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
import mechkit as m
from yam_arena import demo as d,ik,station
from yam_arena.demo import Waypoint
from crutch_handle_layout import POS,R,TOP
COUNT=0;SNAP={};LIVE={};CONTACTS=None;AUDIT=None;NATIVE=None;ABORT=False;LAST={}
INITIAL_AT=30;BARE_AT=260;LIFT_AT=380;INSERT_AT=820;INSTALLED_AT=1160;RETURN_AT=1260
MID_PITCH_DEG=60.;MID_YAW_DEG=-45.678133;FINAL_YAW_DEG=-66.
MIDDLE=np.array([.10,.20,.78])
ALIGN_OFFSET=np.zeros(3);ALIGN_BLEND=1.;ALIGN_ROTATION=np.eye(3)
LOAD_ROTATION=np.eye(3);LOAD_ALLOWED=False;LOAD_ERROR_DEG=0.
INSERT_COUNT=None;INSERT_HEIGHT=.038;INSERT_ANGLE=0.;INSERT_START=None
INSERT_CLEAR_STEPS=0;INSERT_RETREAT=False;INSERT_ABORT_REASON=None;INSERT_BIAS=0.;INSERT_BIAS_ALLOWED=False;INSERT_BIAS_ERROR=0.

def reset(env=None):
    global LOAD_ROTATION,LOAD_ALLOWED,LOAD_ERROR_DEG,COUNT,ABORT,INSERT_COUNT,INSERT_HEIGHT,INSERT_ANGLE,INSERT_START,INSERT_CLEAR_STEPS,INSERT_RETREAT,INSERT_ABORT_REASON,INSERT_BIAS,INSERT_BIAS_ALLOWED,INSERT_BIAS_ERROR
    LOAD_ROTATION=np.eye(3);LOAD_ALLOWED=False;LOAD_ERROR_DEG=0.
    INSERT_COUNT=None;INSERT_HEIGHT=.038;INSERT_ANGLE=0.;INSERT_START=None
    INSERT_CLEAR_STEPS=0;INSERT_RETREAT=False;INSERT_ABORT_REASON=None;INSERT_BIAS=0.;INSERT_BIAS_ALLOWED=False;INSERT_BIAS_ERROR=0.
    COUNT=0;ABORT=False;SNAP.clear();LIVE.clear();LAST.clear()
def observe(env):
    global COUNT,ABORT
    COUNT+=1;c=m.state(env,'crutch');n=m.state(env,'new_ferrule');o=m.state(env,'old_ferrule');LIVE.update(c=c,new=n)
    if COUNT==INITIAL_AT:SNAP.update(initial=c,old=m.state(env,'old_ferrule'),new=n)
    if COUNT==BARE_AT:SNAP['bare']=c
    if COUNT==LIFT_AT:
        SNAP['lift']=c;SNAP['grasp']={}
        for side in ('left','right'):
            q=m._np(env.unwrapped.scene[side+'_robot'].data.joint_pos)[0,:6]
            tp,tr=ik.tcp_pose(q,station.BASES[side]);SNAP['grasp'][side]=(tr.T@(c.pos-tp),tr.T@c.rotation())
        print('RIGID_PAIR_CAL',COUNT,{'crutch_pos':c.pos.tolist(),'crutch_rotation':c.rotation().tolist(),'grips':{side:{'position':v[0].tolist(),'rotation':v[1].tolist()} for side,v in SNAP['grasp'].items()}},flush=True)
    if COUNT==INSTALLED_AT:SNAP['installed_foot']=n
    if COUNT==RETURN_AT:SNAP['return']=c
    if INSERT_AT-20<=COUNT<=INSTALLED_AT+20 and n.rotation()[2,2]<.96:
        ABORT=True;print('BENCH_INSERT_ABORT',COUNT,'replacement tipped',flush=True)
    if COUNT==1 or COUNT%20==0:
        q={side:m._np(env.unwrapped.scene[side+'_robot'].data.joint_pos)[0,:6].tolist() for side in ('left','right')}
        print('BENCH_REPAIR_STATE',COUNT,{'old_pos':o.pos.tolist(),'old_rotation':o.rotation().tolist(),'old_velocity':o.lin_vel.tolist(),'old_omega':o.ang_vel.tolist(),'crutch_pos':c.pos.tolist(),'crutch_rotation':c.rotation().tolist(),'foot_pos':n.pos.tolist(),'foot_rotation':n.rotation().tolist(),'relative_foot_m':c.to_local(n.pos)[0].tolist(),'joints':q,'contacts':None if CONTACTS is None else dict(CONTACTS.last),'abort':ABORT},flush=True)
m.on_reset(reset);m.on_step(observe)
@dataclass
class Point(Waypoint):goal:object=None

def target(self,t):
    if ABORT and id(self) in LAST:return LAST[id(self)]
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        g,q=w.goal() if isinstance(w,Point) else (np.asarray(w.pos,float),d.grip_rotation(w.yaw))
        if t<w.duration:
            a=ik.smoothstep(t/w.duration);out=(p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label);LAST[id(self)]=out;return out
        t-=w.duration;p,r,f=g,q,w.finger
    return p,r,f,'done'
d.ArmPlan.target=target

def point(t,f,label,goal):return Point(t,(0,0,0),f,label,0.,goal)
def old_goal(dz,withdraw=0.):
    o=SNAP.get('old');p=POS if o is None else o.pos;r=R if o is None else o.rotation();a=r[:,2]
    return p+a*(.028-withdraw)+[0,0,dz],d.grip_rotation(np.arctan2(a[1],a[0]))
def old_step(t,dz,f,label,withdraw=0.):return point(t,f,label,lambda:old_goal(dz,withdraw))
def handle_goal(dz):
    c=SNAP.get('initial');p=POS if c is None else c.pos;r=R if c is None else c.rotation()
    return p+r@np.array([0.,-.10,.70])+[0,0,dz],d.grip_rotation(np.arctan2(r[1,1],r[0,1]))
def handle_step(t,dz,f,label):return point(t,f,label,lambda:handle_goal(dz))
def shaft_goal(side,dz):
    c=SNAP.get('bare',LIVE.get('c'));p=POS if c is None else c.pos;r=R if c is None else c.rotation();a=r[:,2];z=.40 if side=='left' else .22
    return p+a*z+[0,0,dz],d.grip_rotation(np.arctan2(a[1],a[0]))
def shaft_step(t,side,dz,f,label):return point(t,f,label,lambda:shaft_goal(side,dz))
def axis_arc(a,b,f):
    a=np.asarray(a,float);a=a/np.linalg.norm(a);b=np.asarray(b,float);b=b/np.linalg.norm(b);n=np.cross(a,b);s=np.linalg.norm(n)
    if s<1e-10:return a.copy()
    return Rotation.from_rotvec(n/s*np.arctan2(s,a@b)*f).apply(a)
def foot_pose():
    # Use the observed upright bore frame as a fixed insertion reference.
    # A tipping loose foot must not steer the entire two-arm grasp.
    n=SNAP.get('bore',LIVE.get('new',SNAP.get('new')))
    return (np.array([.35,-.05,TOP]),np.array([0.,0.,1.])) if n is None else (n.pos.copy(),n.rotation()[:,2])
def rigid_tcp(p,r,side,load_fraction=1.):
    if 'grasp' not in SNAP:return shaft_goal(side,.067)
    rp,rr=SNAP['grasp'][side];base_r=ALIGN_ROTATION@r
    # Rotate the complete measured pair about its commanded physical shaft tip.
    # This preserves both shaft-to-gripper transforms and the commanded tip point.
    loaded=Rotation.from_rotvec(Rotation.from_matrix(LOAD_ROTATION).as_rotvec()*load_fraction).as_matrix()@base_r
    tip=p+ALIGN_OFFSET*ALIGN_BLEND+base_r[:,2]*.008
    root=tip-loaded[:,2]*.008;tr=loaded@rr.T
    return root-tr@rp,tr

def rigid_object_pose(fraction,stage):
    c=SNAP.get('lift',SNAP.get('bare',LIVE.get('c')))
    p=POS+[0,0,.06] if c is None else c.pos;r=R if c is None else c.rotation();axis=r[:,2];fp,fa=foot_pose()
    if COUNT>=RETURN_AT and 'installed_foot' in SNAP:
        anchor=SNAP['installed_foot'];fp=anchor.pos.copy();fa=anchor.rotation()[:,2]
    normal=np.cross(axis,fa);norm=np.linalg.norm(normal);theta=np.arctan2(norm,axis@fa)
    normal=normal/max(norm,1e-12)
    middle_angle=np.radians(MID_PITCH_DEG)-np.arcsin(np.clip(axis[2],-1,1))
    if stage==0:
        angle=middle_angle*fraction;yaw=np.radians(MID_YAW_DEG)*fraction;root=p*(1-fraction)+MIDDLE*fraction
    else:
        angle=middle_angle*(1-fraction)+theta*fraction;yaw=np.radians(MID_YAW_DEG*(1-fraction)+FINAL_YAW_DEG*fraction);root=MIDDLE*(1-fraction)+(fp+fa*.10)*fraction
    rr=Rotation.from_rotvec(fa*yaw).as_matrix()@Rotation.from_rotvec(normal*angle).as_matrix()@r
    return root,rr

def upright_goal(fraction,side,stage=0):
    p,r=rigid_object_pose(fraction,stage);return rigid_tcp(p,r,side)
def upright_step(t,fraction,side,label,stage=0):return point(t,0.,label,lambda:upright_goal(fraction,side,stage))
def update_insertion_command():
    """One shared, rate-limited robot target; never alter object state or force."""
    global LOAD_ROTATION,LOAD_ALLOWED,LOAD_ERROR_DEG,INSERT_COUNT,INSERT_HEIGHT,INSERT_ANGLE,INSERT_START,INSERT_CLEAR_STEPS,INSERT_RETREAT,INSERT_ABORT_REASON,INSERT_BIAS,INSERT_BIAS_ALLOWED,INSERT_BIAS_ERROR
    if INSERT_COUNT==COUNT:return
    if INSERT_COUNT is not None and COUNT!=INSERT_COUNT+1:
        print('INSERT_COMMAND_ERROR',{'previous':INSERT_COUNT,'current':COUNT},flush=True)
        raise RuntimeError('Insertion command must update once per control step')
    INSERT_COUNT=COUNT;t=(COUNT-INSERT_AT)/20.
    if t<=1.5:a,b,f=.038,.020,max(0.,t/1.5)
    elif t<=3.:a,b,f=.020,.004,(t-1.5)/1.5
    elif t<=4.:a,b,f=.004,.0008,t-3.
    else:a,b,f=.0008,.0008,1.
    nominal=a+(b-a)*float(ik.smoothstep(f))
    root=LIVE['c'];foot=LIVE['new'];axis=root.rotation()[:,2]
    tip=foot.rotation().T@(root.pos+axis*.008-foot.pos)
    lateral=float(np.linalg.norm(tip[:2]));angle=float(np.degrees(np.arccos(np.clip(axis@foot.rotation()[:,2],-1.,1.))))
    load=0. if CONTACTS is None else float(CONTACTS.last.get('foot_external_n') or 0.)
    # Correct only measured axial tracking lag, with no force or state writes.
    # This is shared by both arms and bounded independently of the nominal stroke.
    arm_clear=(AUDIT is not None and AUDIT.ready and AUDIT.last.get('correction_allowed',False))
    if (INSERT_BIAS>0. or Rotation.from_matrix(LOAD_ROTATION).magnitude()>1e-8) and not arm_clear:
        INSERT_ABORT_REASON='axial correction stopped by arm contact, joint, effort or grip guard'
    # The completed slow axial twist is followed by measured transverse alignment.
    # Native evidence must show liner contact without any fixed-frame loading in
    # every480Hz sample of the previous complete interval. Never infer this from
    # only external support or net force, which can hide opposing shaft contacts.
    fixed_axis=foot_pose()[1];cross=np.cross(axis,fixed_axis);sn=float(np.linalg.norm(cross))
    error_vector=cross/max(sn,1e-12)*np.arctan2(sn,np.clip(axis@fixed_axis,-1.,1.))
    LOAD_ERROR_DEG=float(np.degrees(np.linalg.norm(error_vector)))
    native_ok=(NATIVE is not None and NATIVE.ready and NATIVE.last.get('step') in (COUNT,COUNT-1) and NATIVE.last.get('liner_only',False))
    twist_done=(INSERT_START is not None and COUNT>=INSERT_START+80 and abs(INSERT_ANGLE)<1e-8)
    LOAD_ALLOWED=bool(twist_done and native_ok and arm_clear and not INSERT_RETREAT and INSERT_ABORT_REASON is None and .009<tip[2]<=.0412 and lateral<=.0005 and .15<LOAD_ERROR_DEG<=2. and foot.rotation()[2,2]>=np.cos(np.radians(1.)) and abs(foot.pos[2]-foot_pose()[0][2])<=.002 and .8<load<40.)
    if LOAD_ALLOWED:
        increment=error_vector*min(.08,np.radians(.025)/max(np.linalg.norm(error_vector),1e-12))
        proposed=Rotation.from_rotvec(increment).as_matrix()@LOAD_ROTATION
        # Reject an increment at saturation, rather than integrating a hidden state.
        if Rotation.from_matrix(proposed).magnitude()<=np.radians(2.):LOAD_ROTATION=proposed
        else:LOAD_ALLOWED=False
    INSERT_BIAS_ERROR=float(tip[2]-.008-nominal)
    rate_room=(INSERT_HEIGHT-(nominal-INSERT_BIAS)<=.0005 or INSERT_BIAS_ERROR<0.)
    INSERT_BIAS_ALLOWED=bool(not LOAD_ALLOWED and rate_room and arm_clear and .009<tip[2]<=.0412 and lateral<=.0005 and angle<=1. and foot.rotation()[2,2]>=np.cos(np.radians(1.)) and abs(foot.pos[2]-foot_pose()[0][2])<=.002 and .8<load<40.)
    if tip[2]<.0455 and lateral>.001:INSERT_ABORT_REASON='shaft displaced at mouth'
    if foot.rotation()[2,2]<np.cos(np.radians(3.)):INSERT_ABORT_REASON='foot tilted'
    if load>45. or INSERT_ABORT_REASON:INSERT_RETREAT=True
    elif load<35.:INSERT_RETREAT=False
    if INSERT_RETREAT:
        INSERT_BIAS=max(0.,INSERT_BIAS-.00025)
    elif INSERT_BIAS_ALLOWED and abs(INSERT_BIAS_ERROR)>.001:
        INSERT_BIAS=float(np.clip(INSERT_BIAS+np.clip(.1*INSERT_BIAS_ERROR,-.00025,.00025),0.,.040))
    desired=min(.05,INSERT_HEIGHT+.0005) if INSERT_RETREAT else nominal-INSERT_BIAS
    INSERT_HEIGHT+=float(np.clip(desired-INSERT_HEIGHT,-.0005,.0005))
    eligible=(not INSERT_RETREAT and INSERT_ABORT_REASON is None and .010<tip[2]<=.0412 and lateral<=.0005 and angle<=1. and foot.rotation()[2,2]>=np.cos(np.radians(1.)) and load<20.)
    INSERT_CLEAR_STEPS=INSERT_CLEAR_STEPS+1 if eligible else 0
    if INSERT_START is None and COUNT<=INSERT_AT+40 and INSERT_CLEAR_STEPS>=5:INSERT_START=COUNT
    desired_angle=0.
    if INSERT_START is not None and not INSERT_RETREAT and INSERT_ABORT_REASON is None:
        phase=(COUNT-INSERT_START)/20.
        if 0.<phase<4.:
            envelope=float(ik.smoothstep(min(1.,phase/.5,(4.-phase)/.5)))
            desired_angle=float(np.radians(4.)*envelope*np.sin(2.*np.pi*phase/4.))
    INSERT_ANGLE+=float(np.clip(desired_angle-INSERT_ANGLE,-np.radians(.4),np.radians(.4)))

def insertion_twist_angle():return INSERT_ANGLE

def insert_goal(delta,side):
    fp,axis=foot_pose()
    active=INSERT_AT<=COUNT<=INSTALLED_AT
    if active:
        update_insertion_command();delta=INSERT_HEIGHT
    else:
        # Resume lifting from the last commanded insertion height without a jump.
        if COUNT>INSTALLED_AT and delta<.038:delta=INSERT_HEIGHT
        if delta>=.075 and 'installed_foot' in SNAP:
            installed=SNAP['installed_foot'];fp=installed.pos.copy();axis=installed.rotation()[:,2]
    _,r=rigid_object_pose(1.,1)
    if active:r=Rotation.from_rotvec(axis*INSERT_ANGLE).as_matrix()@r
    return rigid_tcp(fp+axis*delta,r,side)
def insert_step(t,delta,side,label):return point(t,0.,label,lambda:insert_goal(delta,side))
def return_goal(fraction,side,stage=2):
    # Reverse the verified two-stage path; fade the measured common offset
    # before setting the repaired crutch back on the ordinary bench.
    f=fraction;lift=SNAP.get('lift',LIVE.get('c'));initial=SNAP.get('initial',lift)
    if stage==0:
        _,r=rigid_object_pose(1.-f,1)
        anchor=SNAP.get('installed_foot',SNAP.get('new'));fp,axis=foot_pose() if anchor is None else (anchor.pos,anchor.rotation()[:,2])
        p=(fp+axis*.075)*(1.-f)+MIDDLE*f;offset_fraction=1.-.5*f
    elif stage==1:
        p,r=rigid_object_pose(1.-f,0);offset_fraction=.5*(1.-f)
    else:
        lp=POS+[0,0,.06] if lift is None else lift.pos;lr=R if lift is None else lift.rotation()
        ip=POS if initial is None else initial.pos;ir=R if initial is None else initial.rotation()
        p=lp*(1.-f)+ip*f;r=ik.slerp_matrix(lr,ir,f);offset_fraction=0.
    r=ALIGN_ROTATION.T@ik.slerp_matrix(np.eye(3),ALIGN_ROTATION,offset_fraction)@r
    return rigid_tcp(p+ALIGN_OFFSET*ALIGN_BLEND*(offset_fraction-1.),r,side,load_fraction=offset_fraction)
def return_step(t,fraction,side,finger,label,stage=2):return point(t,finger,label,lambda:return_goal(fraction,side,stage))
def clear_goal(side):
    p,r=return_goal(1.,side);return p+[0,0,.15],r

def demo():
    plans={}
    for side in ['left','right']:
        if side=='left':
            plan=[handle_step(2.,.10,-.04695,'observe original handgrip'),handle_step(1.,.10,-.04695,'approach handgrip'),handle_step(1.5,.007,-.04695,'reach handgrip'),handle_step(1.,.007,0.,'grasp handgrip'),handle_step(5.,.007,0.,'counterhold old foot removal'),handle_step(1.,.007,-.04695,'release crutch on bench'),handle_step(1.5,.16,-.04695,'withdraw vertically from original handgrip'),shaft_step(2.,side,.10,-.04695,'approach lower shaft'),shaft_step(1.,side,.007,-.04695,'reach shaft'),shaft_step(1.,side,.007,0.,'grasp shaft')]
        else:
            plan=[old_step(2.,.10,-.04695,'observe worn foot'),old_step(1.,.10,-.04695,'approach worn foot'),old_step(1.5,.007,-.04695,'reach worn foot'),old_step(1.,.007,-.008,'grasp worn rubber'),old_step(2.5,.007,-.008,'remove worn foot',.075),old_step(1.,.067,-.008,'lift worn foot',.075),Waypoint(1.,(.35,-.42,TOP+.08),-.008,'set worn foot aside',1.1592794807),Waypoint(.5,(.35,-.42,TOP+.08),-.04695,'release worn foot',1.1592794807),Waypoint(1.5,(.35,-.42,TOP+.22),-.04695,'withdraw vertically from worn foot',1.1592794807),Waypoint(1.,(.35,-.42,TOP+.22),-.04695,'wait for handle withdrawal',1.1592794807),shaft_step(2.,side,.10,-.04695,'approach lower shaft'),shaft_step(1.,side,.007,-.04695,'reach shaft'),shaft_step(1.,side,.007,0.,'grasp shaft')]
        plan += [shaft_step(1.5,side,.067,0.,'lift crutch with both hands'),shaft_step(.5,side,.067,0.,'observe lifted shaft'),upright_step(0.,0.,side,'preserve measured pair grasp')]
        plan += [upright_step(.6,float(f),side,'shared rotation to intermediate pose') for f in np.linspace(.1,1.,10)]
        plan += [upright_step(1.,1.,side,'settle intermediate grasp')]
        plan += [upright_step(.4,float(f),side,'transfer pair to vertical insertion pose',stage=1) for f in np.linspace(.1,1.,10)]
        plan += [upright_step(4.,1.,side,'settle and align above bore',stage=1),insert_step(1.,.039,side,'approach open bore'),insert_step(5.,.039,side,'settle and refine alignment near mouth without contact'),insert_step(1.,.038,side,'align tip at mouth'),insert_step(1.5,.020,side,'start bench-supported insertion'),insert_step(1.5,.004,side,'press toward sole'),insert_step(1.,.0008,side,'seat against sole'),insert_step(13.,.0008,side,'observe seating with bounded axial tracking correction'),insert_step(2.,.075,side,'lift foot clear of external supports'),insert_step(3.,.075,side,'observe unsupported retention')]
        plan += [return_step(.4,float(f),side,0.,'reverse transfer to intermediate pose',stage=0) for f in np.linspace(.1,1.,10)]
        plan += [return_step(.6,float(f),side,0.,'reverse tilt with both grips',stage=1) for f in np.linspace(.1,1.,10)]
        plan += [return_step(2.,1.,side,0.,'lower repaired crutch to bench',stage=2)]
        plan += [return_step(1.,1.,side,0.,'settle crutch on bench'),return_step(1.,1.,side,-.04695,'release repaired crutch'),point(2.,-.04695,'clear both hands',lambda s=side:clear_goal(s)),point(3.,-.04695,'observe released assembly',lambda s=side:clear_goal(s))]
        plans[side]=plan
    return plans
