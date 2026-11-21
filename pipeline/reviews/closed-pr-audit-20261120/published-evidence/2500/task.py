"""Recover a physically obstructed Jetson Nano controller cooling fan."""
from pathlib import Path
from dataclasses import replace
import sys,math,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as m
from hardware import fan,jetson,LOOKS
from service_hardware import camera,guard,supports,fan_wires,power_switch,SERVICE_LOOKS,col
from scaled_convex import scaled_convex_part
from simready import SimReady
from tool_safety import ToolGeometry
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED

SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=0.
INSTRUCTION='Restore the controller cooling fan and put the removed debris in the parts tray.'
camera()
_original=m._Finder.targets['yam_arena.scene']
def patch(scene):
    _original(scene);scene.PHYSICS_DT=1./2400.
m._Finder.targets['yam_arena.scene']=patch
BOARD=np.array([.30,-.08,TOP+.012])
FAN=BOARD+np.array([0,.0023,.0282712])
TRAY=np.array([.30,-.25,TOP])
POWER=np.array([.17,-.19,TOP])
TAIL_A=np.array([.012,0,.004]);TAIL_B=np.array([.059,0,.069])
length=np.linalg.norm(TAIL_B-TAIL_A);center=(TAIL_A+TAIL_B)/2
axis=(TAIL_B-TAIL_A)/length
u=np.array([0.,1.,0.]);v=np.cross(axis,u)
shape=m.box((.003,.001,length))
shape=m.Mesh(shape.v@np.column_stack([u,v,axis]).T+center,shape.f)
tail=m.free('tail',tuple(FAN),[('nylon_offcut',shape,'nylon')],
    [col('nylon_collision',shape)],LOOKS,mass=length*.003*.001*1140,
    friction=(.35,.25),com=tuple(center))

def parts_tray():
    parts=[];cols=[]
    for name,size,at in [('floor',(.106,.106,.003),(0,0,.0015)),
        ('left',(.003,.106,.020),(-.0515,0,.010)),
        ('right',(.003,.106,.020),(.0515,0,.010)),
        ('front',(.100,.003,.020),(0,-.0515,.010)),
        ('back',(.100,.003,.020),(0,.0515,.010))]:
        mesh=m.box(size,at);parts.append((name,mesh,'blue'));cols.append(col(name+'_col',mesh))
    return m.fixed('parts_tray',tuple(TRAY),parts,cols,SERVICE_LOOKS)

def power_cord():
    parts=[]
    # The low-voltage cable is strain-relieved on the service bench. Its 90 mm
    # USB-C-to-switch segment follows the supply manufacturer's DC-cord drawing.
    jack=BOARD+np.array([-.042,-.055,.004])
    boot=m.box((.012,.020,.006),tuple(jack+[0,-.013,0]))
    plug=m.box((.0084,.007,.0026),tuple(jack+[0,-.002,0]))
    parts += [('usb_c_boot',boot,'black'),('usb_c_shell',plug,'silver')]
    pts=[jack+[0,-.023,0],(.246,-.174,TOP+.003),(.234,-.194,TOP+.003),
         (.216,-.203,TOP+.003),(.200,-.190,TOP+.006)]
    parts.append(('output_cable',m.tube(pts,.0019,sides=12),'black'))
    supply=[(.140,-.190,TOP+.006),(.115,-.190,TOP+.004),(.092,-.23,TOP+.004),
            (.090,-.38,TOP+.004),(.090,-.64,TOP+.004)]
    parts.append(('supply_lead',m.tube(supply,.0019,sides=12),'black'))
    # Secured wiring is visual; it is outside the robot extraction path.
    return m.fixed('power_cord',(0,0,0),parts,(),SERVICE_LOOKS)

OBJECTS=[jetson(tuple(BOARD)),fan(tuple(FAN)),scaled_convex_part(tail),
    guard(tuple(FAN+[0,0,.0104])),supports(BOARD,TOP),fan_wires(BOARD,FAN),
    power_switch(tuple(POWER)),power_cord(),parts_tray(),
    SimReady('long_nose_pliers',pos=(.42,.11,TOP),size=(.235,.069,.024),mass=.18,
      package='handTool_pliers_long_nose_a01',usd='sm_handTool_pliers_long_nose_a01_01.usd'),
    SimReady('phillips_screwdriver',pos=(.42,.19,TOP),size=(.20,.03,.03),mass=.10,
      package='handTool_screwdriver_phillips_ph1_a01',usd='sm_handTool_screwdriver_phillips_ph1_a01_01.usd')]

def endpoints(state):
    return np.asarray([TAIL_A,TAIL_B])@state.rotation().T+state.pos

def clearance(state):
    a,b=endpoints(state);c=FAN+np.array([0,0,.0052]);d=b-a
    t=float(np.clip((c-a).dot(d)/d.dot(d),0,1))
    return float(np.linalg.norm(a+t*d-c))>.029

def contained(state):
    # Every corner must be within the tray walls and below their top edge.
    pts=shape.v@state.rotation().T+state.pos-TRAY
    return bool(np.all(np.abs(pts[:,:2])<.049) and np.all(pts[:,2]>.002)
        and np.all(pts[:,2]<.0205) and np.linalg.norm(state.lin_vel)<.02)

def segment_box_distance(a,b,center,rotation,half):
    """Exact distance from a segment to an oriented box, in metres."""
    a=(a-center)@rotation;b=(b-center)@rotation;d=b-a
    cuts=[0.,1.]
    for i in range(3):
        if abs(d[i])>1e-12:
            cuts += [float(t) for t in ((-half[i]-a[i])/d[i],(half[i]-a[i])/d[i]) if 0.<t<1.]
    cuts=sorted(cuts);candidates=list(cuts)
    for lo,hi in zip(cuts,cuts[1:]):
        middle=a+d*(lo+hi)/2
        active=np.abs(middle)>half
        denominator=float(np.sum(d[active]**2))
        if denominator>1e-20:
            offset=a[active]-np.sign(middle[active])*half[active]
            candidates.append(float(np.clip(-offset.dot(d[active])/denominator,lo,hi)))
    return min(float(np.linalg.norm(np.maximum(np.abs(a+t*d)-half,0))) for t in candidates)

def finger_boxes(points):
    """Finger bounds from gripper pose and symmetric joint-7 kinematics."""
    boxes=[]
    for finger in np.asarray(points).reshape(4,3,2,3):
        y=finger[2,0]-finger[0,0];y/=np.linalg.norm(y)
        z=finger[0,1]-finger[0,0];z/=np.linalg.norm(z)
        x=np.cross(y,z)
        boxes.append((finger.mean(axis=(0,1)),np.column_stack((x,y,z)),np.array([.0045,.018,.015])))
    return boxes

def released(state,objects):
    a,b=endpoints(state)
    # Capsule radius bounds the offcut's 3 x 1 mm cross-section.
    margin=.020+math.hypot(.0015,.0005)
    if not monitor.finger_boxes:return False
    if any(segment_box_distance(a,b,*box)<margin for box in monitor.finger_boxes):return False
    for name in ('long_nose_pliers','phillips_screwdriver'):
        tool=objects[name]
        if segment_box_distance(a,b,(tool.low+tool.high)/2,np.eye(3),(tool.high-tool.low)/2)<margin:return False
    return True

class Monitor:
    def reset(self,env=None):
        self.steps=0;self.running_s=0.;self.omega=0.;self.powered=True;self.last_drive=None;self.unsafe=False;self.finger_boxes=[];self.coasting=True;self.tool_geometry=ToolGeometry()
        if env is not None:self.tool_geometry.reset(env)
    def step(self,env):
        import torch
        self.steps+=1
        asset=env.unwrapped.scene['fan'];j=asset.joint_names.index('rotor_joint')
        self.omega=float(m._np(asset.data.joint_vel)[0,j])
        angle=m.state(env,'power').joints['rocker_joint']
        # Electrical contact follows the mechanical rocker; intermediate
        # positions are unpowered. This changes motor torque, never body pose.
        self.powered=bool(angle>math.radians(-12.))
        # Once isolation has reached rest, deliberate manual rotor movement is
        # service access, not motor coast-down. Re-energizing rearms the latch.
        if self.powered:self.coasting=True
        elif abs(self.omega)<=1.:self.coasting=False
        if self.last_drive is not self.powered:
            # Powered: speed-controller damping. Unpowered: estimated passive
            # bearing/air drag, allowing coast-down instead of active braking.
            damping=.000004 if self.powered else .00000008
            asset.write_joint_damping_to_sim(torch.tensor([[damping]],device=asset.device),joint_ids=[j])
            self.last_drive=self.powered
        asset.set_joint_velocity_target(torch.tensor([[523.5988 if self.powered else 0.]],device=asset.device),joint_ids=[j])
        state=m.state(env,'tail')
        self.finger_boxes=finger_boxes(m.fingertips(env))
        grid=np.array([(x,y,z) for x in (-.0045,0,.0045) for y in np.linspace(-.018,.018,5) for z in np.linspace(-.015,.015,7)])
        finger_points=np.concatenate([grid@rotation.T+center for center,rotation,half in self.finger_boxes])-FAN
        in_rotor=(np.linalg.norm(finger_points[:,:2],axis=1)<.0185)&(finger_points[:,2]>.003)&(finger_points[:,2]<.0104)
        tool_hits,_=self.tool_geometry.audit(env,FAN)
        if (self.powered or self.coasting) and (np.any(in_rotor) or tool_hits):self.unsafe=True
        self.running_s=self.running_s+.05 if self.powered and self.omega>418.879 and clearance(state) else 0.
        if self.steps==1 or self.steps%20==0:
            a,b=endpoints(state)
            finger_clearance=min(segment_box_distance(a,b,*box) for box in self.finger_boxes)-math.hypot(.0015,.0005)
            print('FAN_SERVICE',self.steps,{'omega_rad_s':self.omega,'running_s':self.running_s,
                'power_angle_deg':math.degrees(angle),'clear':clearance(state),'contained':contained(state),'unsafe':self.unsafe,
                'finger_clearance_m':finger_clearance,'tool_intrusions':tool_hits,'coasting':self.coasting,
                'tail_position':state.pos.tolist(),'tail_quat':state.quat_xyzw.tolist()},flush=True)
monitor=Monitor();monitor.reset();m.on_step(monitor.step);m.on_reset(monitor.reset)

def score(objects):
    if monitor.unsafe:return 0.
    # Pose gates apply to every tier. Fixed fastenings are part of the setup;
    # the simulation does not permit removing the board, guard, or fan mount.
    for name,reference in [('jetson',BOARD),('fan',FAN),('guard',FAN+[0,0,.0104])]:
        state=objects[name]
        rotation_error=2.*math.acos(float(np.clip(abs(state.quat_xyzw[3]),0.,1.)))
        if np.linalg.norm(state.pos-reference)>.002 or rotation_error>math.radians(3.):return 0.
    tail=objects['tail']
    if not clearance(tail):return 0.
    if not contained(tail) or not released(tail,objects):return .3
    return 1. if monitor.running_s>=10.-1e-6 and monitor.powered and monitor.omega>418.879 else .6
def success(objects):return score(objects)==1.

def demo():
    yaw=-.385
    return {'right':[
        Waypoint(2.,(.3515,-.0775,.88),FINGER_OPEN,'approach protruding offcut',yaw),
        Waypoint(1.,(.3515,-.0775,.8201),FINGER_OPEN,'align on external end',yaw),
        Waypoint(.7,(.3515,-.0775,.8201),FINGER_CLOSED,'pinch nylon',yaw),
        Waypoint(1.2,(.394,-.0874,.8444),FINGER_CLOSED,'draw clear through guard',yaw),
        Waypoint(1.5,(.32,-.26,.84),FINGER_CLOSED,'carry to parts tray',yaw),
        Waypoint(.7,(.32,-.26,TOP+.037),FINGER_CLOSED,'lower into tray',yaw),
        Waypoint(.5,(.32,-.26,TOP+.037),FINGER_OPEN,'release debris',yaw),
        Waypoint(1.,(.25,-.25,.90),FINGER_OPEN,'withdraw',yaw),
        Waypoint(10.,(.25,-.25,.90),FINGER_OPEN,'observe continuous fan operation',yaw)]}
