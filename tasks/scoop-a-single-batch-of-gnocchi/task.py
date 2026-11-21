"""Single-batch gnocchi task under native qualification.
Independent buoyancy/heat approximation, native contact, no relatching.
Per-piece operational recipe readiness and actual support-path serving score.
"""
import json,math,pathlib,sys
from dataclasses import replace
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import mechkit as m
import batch_physics as b
from heat_model import Heat,circle_overlap,HOT_C
from recipe_scoring import RecipeScore,FoodObservation,FinishObservation,plate_supported
from pose_plan import Pose
from yam_arena import ik,demo as d
from yam_arena.task_file import TABLE_TOP
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=0.
INSTRUCTION='Finish cooking this batch of gnocchi: let them float, then cook for one additional minute in hot water. Serve them on the plate using the slotted spoon, switch off both burners, and leave the spoon resting on the table.'
PAN=np.array([.32,.055]);RADIUS=.12;HOB_TOP=TOP+.003;RING_TOP=TOP+.008;FLOOR=RING_TOP+.005;SURFACE=FLOOR+.040;RIM=FLOOR+.060
WATER_DEPTH=.040;WATER_MASS=np.pi*RADIUS**2*WATER_DEPTH*1000.;PAN_MASS=1.1+WATER_MASS
HOB_FRONT=.137;HOB_CENTER=np.array([HOB_FRONT+.255,PAN[1]]);FRONT_ZONE=PAN.copy();REAR_ZONE=np.array([HOB_FRONT+.390,PAN[1]])
PLATE=np.array([.175,.310]);PLATE_Z=TOP+.006
LOOK={'metal':m.Look(color=(.65,.68,.70),metallic=.8,roughness=.28),'black':m.Look(color=(.045,.045,.05),roughness=.75),'food':m.Look(color=(.90,.78,.52),roughness=.8),'white':m.Look(color=(.91,.92,.88),roughness=.35),'water':m.Look(color=(.38,.65,.81),opacity=.12,roughness=.1),'red':m.Look(color=(.8,.08,.03))}

def tiny(part):
    old=part.build
    return replace(part,build=lambda:old().replace('contactOffset = 0.004','contactOffset = 0.0002').replace('"PhysicsRigidBodyAPI", "PhysicsMassAPI"','"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxContactReportAPI"'))

def ry(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def hull_block(vertices):
    return m.Mesh(np.array(vertices),np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]))
def wall_mesh(r0,r1,z0,z1,a0,a1):
    xy=[(r0*np.cos(a0),r0*np.sin(a0)),(r1*np.cos(a0),r1*np.sin(a0)),(r1*np.cos(a1),r1*np.sin(a1)),(r0*np.cos(a1),r0*np.sin(a1))]
    return hull_block([(x,y,z) for z in (z0,z1) for x,y in xy])

#Installed flush Cookology SEP300SS-sized two-zone hob; housing is below the worktop.
#Round top controls follow manufacturer photograph, with no artificial grip bars.
hob_parts=[('deck',m.box((.510,.290,.003)),'metal'),('front_ring',m.cylinder(.0725,.005,center=(* (FRONT_ZONE-HOB_CENTER),.004)),'black'),('rear_ring',m.cylinder(.09,.005,center=(* (REAR_ZONE-HOB_CENTER),.004)),'black')]
hob_cols=[m.col_box('deck',(.510,.290,.003)),m.col_cylinder('front_ring',.0725,.005,center=(* (FRONT_ZONE-HOB_CENTER),.004)),m.col_cylinder('rear_ring',.09,.005,center=(* (REAR_ZONE-HOB_CENTER),.004))]
hob=m.fixed('hob',(*HOB_CENTER,TOP+.0015),hob_parts,hob_cols,LOOK)
#Native loose1.1kg dry pan plus water mass. Geometry follows24cm inner/6cm depth/47cm length.
parts=[('induction_foot',m.cylinder(.098,.002,center=(0,0,.001)),'metal'),('base',m.cylinder(.122,.003,center=(0,0,.0035)),'metal')]
cols=[m.col_cylinder('induction_foot',.098,.002,center=(0,0,.001)),m.col_cylinder('base',.122,.003,center=(0,0,.0035))]
for k in range(40):
    mesh=wall_mesh(.12,.122,.005,.065,k*2*np.pi/40,(k+1)*2*np.pi/40)
    parts.append((f'wall{k}',mesh,'metal'));cols.append(m.col_hull(f'wall{k}',mesh))
parts.append(('handle',m.box((.226,.022,.007),center=(.235,0,.057)),'metal'))
cols.append(m.col_box('handle',(.226,.022,.007),center=(.235,0,.057)))
pan=tiny(m.free('pan',(*PAN,RING_TOP+.0003),parts,cols,LOOK,mass=PAN_MASS,friction=(.5,.4),com=(.010,0,.025),solver_iters=48))
#Visual water plane only; updated independently of physical bodies to follow a translated/tilted vessel.
water=m.fixed('water_surface',(*PAN,SURFACE-.0005),[('surface',m.cylinder(.119,.001),'water')],[],LOOK)
plate_parts=[('well',m.cylinder(.085,.006,center=(0,0,.003)),'white')];plate_cols=[m.col_cylinder('well',.085,.006,center=(0,0,.003))]
for k in range(32):
    q=wall_mesh(.085,.100,.003,.018,k*2*np.pi/32,(k+1)*2*np.pi/32)
    plate_parts.append((f'rim{k}',q,'white'));plate_cols.append(m.col_hull(f'rim{k}',q))
plate=tiny(m.free('plate',(*PLATE,TOP+.0003),plate_parts,plate_cols,LOOK,mass=.38,friction=(.5,.4),solver_iters=48))
#Manufacturer overall envelope322.58x71.12x44.45mm,104.3g; bowl/handle subdivision is explicit approximation.
BOWL_X=.0475;BOWL_Y=.03556;DEPTH=.012;THICK=.002
sparts=[];scols=[]
#Thin perforated bowl strips. Actual gaps exist in colliders; no solid invisible scoop floor.
for j,y in enumerate(np.arange(-.029,.0291,.005)):
    yl,yh=y-.0015,y+.0015
    xmax=BOWL_X*np.sqrt(max(0.,1-(max(abs(yl),abs(yh))/BOWL_Y)**2))
    xs=np.linspace(-xmax,xmax,9)
    for k,(xl,xh) in enumerate(zip(xs[:-1],xs[1:])):
        xy=[(xl,yl),(xh,yl),(xh,yh),(xl,yh)]
        z=lambda x,y:DEPTH*((x/BOWL_X)**2+(y/BOWL_Y)**2)
        q=hull_block([(x,y,z(x,y)-shift) for shift in (THICK,0.) for x,y in xy])
        name=f'bowl{j}_{k}';sparts.append((name,q,'black'));scols.append(m.col_hull(name,q))
for k in range(40):
    a0,a1=k*2*np.pi/40,(k+1)*2*np.pi/40
    xy=[(.90*BOWL_X*np.cos(a0),.90*BOWL_Y*np.sin(a0)),(BOWL_X*np.cos(a0),BOWL_Y*np.sin(a0)),(BOWL_X*np.cos(a1),BOWL_Y*np.sin(a1)),(.90*BOWL_X*np.cos(a1),.90*BOWL_Y*np.sin(a1))]
    q=hull_block([(x,y,DEPTH*((x/BOWL_X)**2+(y/BOWL_Y)**2)-shift) for shift in (THICK,0.) for x,y in xy])
    name=f'bowlrim{k}';sparts.append((name,q,'black'));scols.append(m.col_hull(name,q))
for name,x0,x1,w,h,z0,z1 in [('neck',.043,.105,.015,.006,.012,.025),('grip',.105,.27508,.028,.018,.025,.03345237245)]:
    xy=[(x0,-w/2),(x1,-w/2),(x1,w/2),(x0,w/2)]
    q=hull_block([(x,y,(z0+(z1-z0)*(x-x0)/(x1-x0))+off) for off in (-h/2,h/2) for x,y in xy])
    sparts.append((name,q,'black'));scols.append(m.col_hull(name,q))
SPOON_VERTS=np.concatenate([x[1].v for x in sparts])
REST_ROT=ry(np.deg2rad(5.0));REST_Z=TOP-float((SPOON_VERTS@REST_ROT.T)[:,2].min())+.0003
REST=np.array([.200,.465,REST_Z])
spoon=tiny(m.free('spoon',REST,sparts,scols,LOOK,mass=.104326,friction=(.8,.55),torsional_patch=.006,start_quat=tuple(ik.matrix_to_quat_xyzw(REST_ROT)),solver_iters=48))
G_NAMES=[f'gnocchi_{i:02}' for i in range(b.N)]
unit=m.sphere(1.,rings=10,segments=16)
mesh=m.Mesh(unit.v*b.HALF,unit.f)
#All food enters in one batch. Uniform ring-packed initial floor layout, not per-spoon piles.
locations=[]
for y in np.arange(-.075,.076,.025):
    for x in np.arange(-.084,.085,.028):
        if x*x+y*y<.095**2:locations.append((x,y))
locations=sorted(locations,key=lambda xy:xy[0]*xy[0]+xy[1]*xy[1])[:b.N]
food=[tiny(m.free(name,(* (PAN+locations[i]),FLOOR+b.HALF[2]+.0003),[('dough',mesh,'food')],[m.col_hull('dough',mesh)],LOOK,mass=b.MASS,friction=(.25,.2),torsional_patch=.001,solver_iters=32,yaw_deg=(i*137.5)%180)) for i,name in enumerate(G_NAMES)]
#Both physical rotary controls are modeled. Rear zone startsOFF; front begins at level3.
KNOB_CENTERS={'front':np.array([HOB_FRONT+.035,PAN[1]-.040]),'rear':np.array([HOB_FRONT+.035,PAN[1]+.040])}
def control(name,center,start):
    mesh=m.cylinder(.020,.018,center=(0,0,.009))
    rotor=m.hinge('dial','Z',(0,0,0),0,300,[('round_dial',mesh,'metal'),('pointer',m.box((.008,.002,.0003),center=(.012,0,.0182)),'black')],[m.col_hull('dial_c',mesh)],mass=.030,friction=.02,damping=.01,gravity=False,effort=5,armature=.0001)
    marks=[('off',m.box((.006,.003,.0004),center=(.027,0,.0002)),'black')]
    for k in range(1,7):
        a=np.deg2rad(k*50);marks.append((f'level{k}',m.cylinder(.0012,.0004,center=(.027*np.cos(a),.027*np.sin(a),.0002)),'black'))
    return m.mechanism(name,(*center,HOB_TOP+.0005),[rotor],LOOK,frame_parts=marks,start=[('dial',start)])
front_knob=control('front_control',KNOB_CENTERS['front'],np.deg2rad(150))
rear_knob=control('rear_control',KNOB_CENTERS['rear'],0.)
OBJECTS=[hob,pan,water,plate,spoon,front_knob,rear_knob]+food

from native_observer import Observe
from types import SimpleNamespace
O=Observe(SimpleNamespace(**globals()));m.on_before_reset(O.before_reset);m.on_reset(O.reset);m.on_step(O.step)

GRIP_SLOPE=(.03345237245-.025)/(.27508-.105)
GRIP_ANGLE=np.arctan(GRIP_SLOPE)
GRIP_LOCAL=np.array([.145,0.,.025+(.145-.105)*GRIP_SLOPE]);GRIP_UP=np.array([-np.sin(GRIP_ANGLE),0.,np.cos(GRIP_ANGLE)]);TCP_LOCAL=GRIP_LOCAL.copy()
HAND_LOCAL=np.array([[0.,-np.cos(GRIP_ANGLE),-np.sin(GRIP_ANGLE)],[-1.,0.,0.],[0.,np.sin(GRIP_ANGLE),-np.cos(GRIP_ANGLE)]])
#Orthonormalise the rounded hand basis.
u,_,v=np.linalg.svd(HAND_LOCAL);HAND_LOCAL=ry(np.deg2rad(10.))@u@v

def w(seconds,origin,rot,finger,label):
    return Pose(seconds,tuple(np.array(origin)+rot@TCP_LOCAL),finger,label,rot@HAND_LOCAL)

SWEEP_X=[.265,.310,.360,.280,.330,.375,.265,.310,.360,.330]
def select_scoop(lap,index,closed):
    """Demo-only native-state feedback chooses a remaining ordinary spoonful.
    No object state/physics/scorer is changed; actual contacts still perform every transfer.
    """
    x=SWEEP_X[index];pan_xy=PAN.copy();plate_xy=PLATE.copy()
    if getattr(O,'initialized',False) and hasattr(O,'pose'):
        pan_xy=O.pan_pose[:2].copy()
        plate_xy=m.state(O.env,'plate').pos[:2].copy()
        points=O.pose[O.wet>.05,:2]
        if len(points):
            options=np.linspace(pan_xy[0]-.070,pan_xy[0]+.070,29)
            counts=[np.sum((np.abs(points[:,0]-a)<.026)&(np.abs(points[:,1]-pan_xy[1])<.105)) for a in options]
            x=float(options[int(np.argmax(counts))])
    heading=rz(np.deg2rad(75. if x<.35 else 90.));scoop_rot=heading@ry(np.deg2rad(-22.))
    #The native pan is loose; follow its actual translation. All dimensions stay full scale.
    targets=[([x,pan_xy[1]+.038,RIM+.025],heading),([x,pan_xy[1]+.038,SURFACE-.020],scoop_rot),([x,pan_xy[1]-.044,SURFACE-.020],scoop_rot),([x,pan_xy[1]-.044,RIM+.024],heading),([plate_xy[0],plate_xy[1],RIM+.025],heading)]
    delta=plate_xy-PLATE
    for j,waypoint in enumerate(lap):
        if j<len(targets):
            pos,rot=targets[j];fresh=w(waypoint.duration,pos,rot,closed,waypoint.label)
            waypoint.pos=fresh.pos;waypoint.rotation=fresh.rotation
        elif j<len(lap)-1:
            waypoint.pos=tuple(np.array(waypoint.pos)+np.r_[delta,0.])

def demo():
    closed=-.006;opened=-.035;rest=REST;rot=REST_ROT
    r=[w(3.,rest+[0,0,.07],rot,opened,'approach_native_handle'),w(2.,rest,rot,opened,'descend_native_handle'),w(1.,rest,rot,closed,'pinch_native_handle'),w(2.,rest+[0,0,.065],rot,closed,'lift_spoon')]
    heading=rz(np.pi/2);carry=np.array([.32,.17,RIM+.025])
    r.append(w(3.,carry,heading,closed,'turn_above_rim'))
    r.append(w(95.,carry,heading,closed,'observe_whole_batch_float_then_extra_minute'))
    #Ordinary spoonfuls from different lateral positions in one batch. Tool orientation changes within natural wrist access.
    for k,x in enumerate(SWEEP_X):
        heading=rz(np.deg2rad(75. if x<.35 else 90.));scoop_rot=heading@ry(np.deg2rad(-22.))
        before=np.array([x,PAN[1]+.038,SURFACE-.020]);after=np.array([x,PAN[1]-.044,SURFACE-.020])
        lap=[w(2.,[x,PAN[1]+.038,RIM+.025],heading,closed,f'scoop{k}_approach'),w(2.,before,scoop_rot,closed,f'scoop{k}_immerse'),w(3.,after,scoop_rot,closed,f'scoop{k}_slide'),w(2.,[x,PAN[1]-.044,RIM+.024],heading,closed,f'scoop{k}_lift_drain'),w(1.5,[.175,.310,RIM+.025],heading,closed,f'scoop{k}_carry_no_turn'),w(2.,[.190,.310,TOP+.105],rz(0),closed,f'scoop{k}_carry'),w(2.,[.190,.310,TOP+.045],rz(0),closed,f'scoop{k}_lower_level'),w(4.,[.190,.310,TOP+.045],ry(np.deg2rad(-50)),closed,f'scoop{k}_tip_to_plate'),w(1.,[.190,.310,TOP+.045],ry(np.deg2rad(-50)),closed,f'scoop{k}_drain_release'),w(2.,[.190,.310,TOP+.105],rz(0),closed,f'scoop{k}_raise_clear'),w(2.,[.190,.310,TOP+.105],rz(np.pi/2),closed,f'scoop{k}_turn_clear'),w(2.,carry,rz(np.pi/2),closed,f'scoop{k}_return')]
        lap[0].on_enter=lambda lap=lap,index=k:select_scoop(lap,index,closed)
        r.extend(lap)
    r.extend([w(3.,rest+[0,0,.065],rot,closed,'return_spoon'),w(2.,rest,rot,closed,'set_spoon_down'),w(1.,rest,rot,opened,'release_spoon'),w(3.,rest+[0,0,.06],rot,opened,'clear'),w(3.,rest+[0,0,.06],rot,opened,'observe_finish')])
    #Native side pinch in the narrow front-facing sector, with ordinary regrasp.
    #A full150degree sweep hits the loose pan; short20degree turns keep the fingers clear.
    xy=KNOB_CENTERS['front'];z=HOB_TOP+.018;dial_open=-.026;dial_closed=-.014
    def knob(seconds,height,yaw,finger,label):
        rotation=rz(math.radians(yaw))@ry(math.radians(-25.))@d.grip_rotation(0.)
        #Real finger pads reach the dial from the front; wider carriers stay clear of the pan.
        pad_center=np.r_[xy,HOB_TOP+.0005+.018-.005]
        tcp=pad_center-rotation@np.array([0.,0.,.0065])+np.array([0.,0.,height-z])
        return Pose(seconds,tuple(tcp),finger,label,rotation)
    right=[knob(3.,z+.080,10,dial_open,'approach_front_control'),knob(106.,z+.080,10,dial_open,'leave_cooking_heat_on'),knob(2.,z,10,dial_open,'descend_round_dial')]
    for turn in range(9):
        pinch=knob(1.,z,10,dial_closed,f'dial{turn}_pinch')
        rotate=knob(2.,z,-10,dial_closed,f'dial{turn}_turn')
        def limit_turn(waypoint=rotate):
            remaining=max(0.,math.degrees(getattr(O,'front',math.radians(150))))
            target=knob(waypoint.duration,z,10-min(20.,remaining),dial_closed,waypoint.label)
            waypoint.rotation=target.rotation;waypoint.pos=target.pos
        rotate.on_enter=limit_turn
        release=knob(.8,z,-10,dial_open,f'dial{turn}_release')
        def follow_turn(release=release,rotate=rotate):
            release.rotation=rotate.rotation.copy();release.pos=rotate.pos
        release.on_enter=follow_turn
        right.extend([pinch,rotate,release,knob(.8,z,10,dial_open,f'dial{turn}_regrasp')])
    right.append(knob(2.,z+.080,10,dial_open,'clear_front_control'))
    return {'left':r,'right':right}
PHASE_ENDS=np.cumsum([v.duration for v in demo()['left']])
PHASE_LABELS=[v.label for v in demo()['left']]
def phase_at(t):return PHASE_LABELS[min(int(np.searchsorted(PHASE_ENDS,t,side='right')),len(PHASE_LABELS)-1)]
def score(objects):return getattr(O,'last_score',0.)
def success(objects):return score(objects)>=1.

O.c.phase_at=phase_at
