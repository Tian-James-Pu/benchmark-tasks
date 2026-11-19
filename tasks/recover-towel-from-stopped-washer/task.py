"""Full-scale washer native door-contact development gate; all scores are zero.
The door is mechanically free for this isolated access diagnostic. Emergency
lock, drain fluid, and deformable towel are not yet represented or validated.
"""
from pathlib import Path
from dataclasses import dataclass,replace
import sys,math,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.task_file import TABLE_TOP
from yam_arena import demo as motion,ik,station
from yam_arena.demo import Waypoint
from yam_arena.urdf import axis_angle_matrix
from shapely.geometry import Polygon,Point
from shapely import constrained_delaunay_triangles
SCENE='station';INSTRUCTION='Take the towel out of the stopped washer.';SUCCESS_HOLD_S=0.
CX=-.50;FRONT=.72;PEDESTAL=.390;HINGE=np.array([CX-.180,FRONT,PEDESTAL+.455])
LOOK={k:m.Look(color=m.rgb(*v),roughness=.5,metallic=.1) for k,v in {'white':(.84,.85,.86),'dark':(.08,.085,.095),'steel':(.55,.57,.58),'black':(.025,.025,.025),'blue':(.20,.28,.36)}.items()}
LOOK['glass']=m.Look(color=m.rgb(.42,.52,.56),opacity=.33,roughness=.1)
def col(n,g):return m.col_hull(n,g).replace('contactOffset = 0.004','contactOffset = 0.0005')
def add(p,c,n,g,k='white'):p.append((n,g,k));c.append(col(n+'_c',g))
# Original envelope on its ordinary listed pedestal; no object placed atop table.
p,c=[],[]
add(p,c,'pedestal',m.box((.598,.590,.390),(CX,FRONT+.295,.195)))
for n,size,pos in [('left_side',(.002,.590,.848),(CX-.298,FRONT+.295,.814)),('right_side',(.002,.590,.848),(CX+.298,FRONT+.295,.814)),('top',(.598,.590,.020),(CX,FRONT+.295,1.228)),('back',(.598,.002,.848),(CX,FRONT+.589,.814)),('bottom',(.598,.590,.010),(CX,FRONT+.295,.395))]:add(p,c,n,m.box(size,pos))
# Front steel sheet, original circular door recess and service opening.
frontpoly=Polygon([(-.299,.39),(.299,.39),(.299,1.218),(-.299,1.218)])
frontpoly=frontpoly.difference(Point(0,.845).buffer(.235,quad_segs=32)).difference(Point(.194,.490).buffer(.077,quad_segs=24))
for i,g in enumerate(constrained_delaunay_triangles(frontpoly).geoms):
 xy=np.array(g.exterior.coords)[:-1];v=np.vstack([np.c_[xy[:,0]+CX,np.full(len(xy),FRONT+d),xy[:,1]] for d in (0,.002)]);f=np.array([[0,2,1],[3,4,5],[0,1,4],[0,4,3],[1,2,5],[1,5,4],[2,0,3],[2,3,5]]);add(p,c,'panel'+str(i),m.Mesh(v,f))
# Original controls are visual in this first contact-only gate.
add(p,c,'control_panel',m.box((.590,.014,.125),(CX,FRONT-.006,1.175)),'white')
add(p,c,'display',m.box((.175,.003,.075),(CX+.165,FRONT-.014,1.185)),'black')
add(p,c,'dial',m.cylinder(.031,.018,(CX,FRONT-.018,1.185),axis='y'),'dark')
# Source-scaled recessed annulus; colliders preserve central322mm aperture.
for i in range(48):
 a,b=i*2*math.pi/48,(i+1)*2*math.pi/48
 xy=[(.161*math.cos(a),.161*math.sin(a)),(.235*math.cos(a),.235*math.sin(a)),(.235*math.cos(b),.235*math.sin(b)),(.161*math.cos(b),.161*math.sin(b))]
 g=m.prism(xy,0,.018).turned(90,'x').moved((CX,FRONT+.035,.845));add(p,c,'recess'+str(i),g,'dark')
# Actual drum envelope, stationary after power interruption. Finite depth.
for i in range(48):
 a,b=i*2*math.pi/48,(i+1)*2*math.pi/48
 xy=[(.245*math.cos(a),.245*math.sin(a)),(.249*math.cos(a),.249*math.sin(a)),(.249*math.cos(b),.249*math.sin(b)),(.245*math.cos(b),.245*math.sin(b))]
 add(p,c,'drum'+str(i),m.prism(xy,0,.350).turned(-90,'x').moved((CX,FRONT+.045,.845)),'steel')
add(p,c,'drum_back',m.cylinder(.249,.003,(CX,FRONT+.395,.845),axis='y'),'steel')
BODY=m.fixed('washer_cabinet',(0,0,0),p,c,LOOK,friction=(.5,.4))
# Original door hinge, polymer annular rim and inward glass bowl. Door mass and
# hinge resistance are disclosed engineering estimates, not measured OEM values.
p,c=[],[]
# Original upper-right rear handhold reconstructed from genuine frame photos.
# Its estimated 10 mm front lip and 35.5 mm radial recess are disclosed in the
# source gate. This replaces the former solid-ring approximation.
for i in range(72):
 a,b=i*2*math.pi/72,(i+1)*2*math.pi/72
 def sector(r0,r1,z0,z1):
  xy=[(r0*math.cos(a),r0*math.sin(a)),(r1*math.cos(a),r1*math.sin(a)),(r1*math.cos(b),r1*math.sin(b)),(r0*math.cos(b),r0*math.sin(b))]
  return m.prism(xy,z0,z1).turned(90,'x').moved((.180,0,0))
 if 2<=i<14:
  add(p,c,'rim_inner'+str(i),sector(.161,.210,.010,.046),'dark')
  add(p,c,'handhold_lip'+str(i),sector(.210,.2455,.036,.046),'dark')
 else:add(p,c,'rim'+str(i),sector(.161,.2455,.010,.046),'dark')
# Thin convex sections of the original glass bowl, maintaining its surface shape.
for j in range(8):
 t0,t1=j*math.pi/16,(j+1)*math.pi/16
 for i in range(32):
  a,b=i*math.pi/16,(i+1)*math.pi/16;verts=[]
  for thick in (0,.003):
   for t,ang in [(t0,a),(t0,b),(t1,b),(t1,a)]:verts.append([.180+(.161-thick)*math.cos(t)*math.cos(ang),(.090-thick)*math.sin(t),(.161-thick)*math.cos(t)*math.sin(ang)])
  g=m.Mesh(np.array(verts),np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]));add(p,c,f'glass{j}_{i}',g,'glass')
# Hook silhouette from genuine00627057; actual native lock will be added next.
outline=[(0,0),(.028,0),(.030,.003),(.028,.007),(.008,.007),(.008,.015),(0,.015)]
add(p,c,'hook',m.prism(outline,-.002,.002).turned(90,'x').moved((.363,.007,0)),'steel')
link=m.hinge('leaf','Z',(0,0,0),-160,0,p,c,mass=3.2,gravity=True,com=(.180,-.008,0),inertia=(.050,.10,.050),friction=.02,damping=.15,armature=0,spring=0)
DOOR=m.mechanism('washer_door',HINGE,[link],LOOK,friction=(.5,.4),solver_iters=64)
b=DOOR.build;DOOR=replace(DOOR,build=lambda:b().replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysxContactReportAPI"').replace('float physics:mass =','float physxContactReport:threshold = 0\n        float physics:mass ='))
OBJECTS=[BODY,DOOR]
@dataclass
class JointWaypoint(Waypoint):
 q_goal:tuple=()
@dataclass
class Oriented(Waypoint):
 tilt:float=0.
 door_angle:float=0.
def rotation(w):
 if not isinstance(w,Oriented):return motion.grip_rotation(w.yaw)
 return axis_angle_matrix(np.array([0.,0,1.]),-math.radians(w.door_angle))@axis_angle_matrix(np.array([1.,0,0]),math.radians(w.tilt))@motion.grip_rotation(w.yaw)
def target(self,t):
 p,r,f=self.start_pos,self.start_rot,self.finger_start
 for w in self.waypoints:
  goal=np.asarray(w.pos);rot=rotation(w)
  if isinstance(w,JointWaypoint):goal,rot=ik.tcp_pose(np.asarray(w.q_goal),self.base_pos)
  if t<w.duration:
   a=ik.smoothstep(t/w.duration);return p+a*(goal-p),ik.slerp_matrix(r,rot,a),f+a*(w.finger-f),w.label
  t-=w.duration;p,r,f=goal,rot,w.finger
 return p,r,f,'done'
motion.ArmPlan.target=target
_native_action7=motion.ArmPlan.action7
def action7(self,t):
 w=self.waypoints[0]
 if isinstance(w,JointWaypoint) and (t<w.duration or len(self.waypoints)==1):
  a=ik.smoothstep(min(t/w.duration,1.));q0=np.asarray(motion.READY);self.q=q0+a*(np.asarray(w.q_goal)-q0)
  finger=self.finger_start+a*(w.finger-self.finger_start)
  return np.r_[self.q,np.clip(finger/motion.FINGER_OPEN,0,1)],w.label
 return _native_action7(self,t)
motion.ArmPlan.action7=action7
def doorpos(deg):return HINGE+axis_angle_matrix(np.array([0.,0,1.]),-math.radians(deg))@np.array([0.3902634065925028, -0.040000000000000036, 0.09804743672384231])
def demo():
 yaw=math.radians(150);p=doorpos(0)
 # Native joint interpolation avoids the unreachable Cartesian approach branch.
 # 81 joint-space samples clear native table/cabinet convex geometry.
 q=(2.6419522785925618, 1.6243296260738258, 1.2154118308300967, -0.4690361292363282, 0.8635421607252208, -0.30573173758948846)
 left=[JointWaypoint(4,tuple(p+[0,-.07,0]),-.04695,'approach original rim',0,q),Oriented(3,tuple(p),-.04695,'enter original rim grip',yaw,60,0),Oriented(2,tuple(p),0,'close on original rim',yaw,60,0)]
 for a in range(5,21,5):left.append(Oriented(1,tuple(doorpos(a)),0,'open original door',yaw,60,a))
 left.append(Oriented(1,tuple(doorpos(20)),-.04695,'release rim',yaw,60,20))
 return {'left':left,'right':[JointWaypoint(14,(0,0,0),-.04695,'hold native ready',0,tuple(motion.READY))]}
class Observe:
 def reset(self,env=None):
  self.n=0;self.contacts={}
  from pxr import PhysicsSchemaTools
  from omni.physx import get_physx_simulation_interface
  def report(headers,data):
   for h in headers:
    if not h.num_contact_data:continue
    a=str(PhysicsSchemaTools.intToSdfPath(h.collider0));b=str(PhysicsSchemaTools.intToSdfPath(h.collider1))
    if 'washer_door' not in a+b:continue
    key=a+' | '+b;entry=self.contacts.setdefault(key,{'impulse_Ns':0.,'count':0,'last_point':None})
    for ix in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
     d=data[ix];entry['impulse_Ns']+=float(np.linalg.norm(d.impulse));entry['count']+=1;entry['last_point']=[float(x) for x in d.position]
  self.subscription=get_physx_simulation_interface().subscribe_contact_report_events(report)
 def step(self,env):
  self.n+=1
  if self.n==1 or self.n%10==0:
   s=m.state(env,'washer_door');a=env.unwrapped.scene['left_robot'];print('WASHER_DOOR_GATE',self.n,{'door_deg':s.deg('leaf'),'left_q':m._np(a.data.joint_pos)[0].tolist(),'left_effort':m._np(a.data.applied_torque)[0].tolist(),'contacts':self.contacts},flush=True)
   self.contacts={}
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
