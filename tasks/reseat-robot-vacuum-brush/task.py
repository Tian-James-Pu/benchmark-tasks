"""S7 brush assembly contact prototype; scoring intentionally disabled pending validation."""
import pathlib,sys,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from simready import SimReady
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED
TOP=TABLE_TOP['station'];SCENE='station';SUCCESS_HOLD_S=0.
INSTRUCTION='Restore the robot vacuum’s main brush and cover so it is ready to clean.'
C=np.array([.40,0,TOP+.017]); AX=.018; AZ=.0775
COVER_SEATED_Z=.0925  # reference for offline fit audit; no pose snapping
LOOK={k:m.Look(color=m.rgb(*rgb)) for k,rgb in {'body':(.80,.82,.83),'black':(.10,.11,.12),'rubber':(.75,.045,.055),'grey':(.38,.40,.43),'metal':(.65,.67,.69),'lens':(.025,.04,.045)}.items()}
def hull(n,mesh):
 return m.col_hull(n,mesh).replace('contactOffset = 0.004','contactOffset = 0.0005')
def add(parts,cols,name,shape,look='body',collision=True):
 name=name.replace('-','m').replace('.','p')
 parts.append((name,shape,look))
 if collision:cols.append(hull(name+'_c',shape))

# Free inverted robot body. Its brush bay is a real opening, not a drawn recess.
bp=[];bc=[]
for i in range(64):
 a=2*np.pi*i/64;b=2*np.pi*(i+1)/64
 # Radial shell wall leaves the central brush bay unobstructed.
 outline=[(.169*np.cos(a),.169*np.sin(a)),(.176*np.cos(a),.176*np.sin(a)),(.176*np.cos(b),.176*np.sin(b)),(.169*np.cos(b),.169*np.sin(b))]
 add(bp,bc,f'shell{i}',m.prism(outline,.016,.090))
add(bp,bc,'top_lid',m.cylinder(.169,.006,(0,0,.019),segments=64))
# Actual receiver pockets must pass through the underside deck. The previous
# solid deck intersected both the four hooks and the two released latch teeth.
DECK_HOLES=[(AX+.035,AX+.054,y-.007,y+.007) for y in (-.085,-.035,.035,.085)]
DECK_HOLES += [(AX-.043,AX-.024,.109,.114),(AX-.043,AX-.024,-.114,-.109)]
def deck_rect(name,x0,x1,y0,y1):
 xs=sorted({x0,x1,*[v for hole in DECK_HOLES for v in hole[:2] if x0<v<x1]})
 ys=sorted({y0,y1,*[v for hole in DECK_HOLES for v in hole[2:] if y0<v<y1]})
 index=0
 for xa,xb in zip(xs,xs[1:]):
  for ya,yb in zip(ys,ys[1:]):
   if xb-xa<1e-9 or yb-ya<1e-9:continue  # discard floating-point duplicate grid boundaries
   x=(xa+xb)/2;y=(ya+yb)/2
   if any(h[0]<x<h[1] and h[2]<y<h[3] for h in DECK_HOLES):continue
   add(bp,bc,f'{name}_{index}',m.box((xb-xa,yb-ya,.006),(x,y,.086)));index+=1
for side in (-1,1):
 for i in range(16):
  yy=side*(.112+i*.0036);xlim=np.sqrt(max(0,.169**2-yy**2))
  deck_rect(f'underside{side}_{i}',-xlim,xlim,yy-.0018,yy+.0018)
for lo,hi in [(-.165,AX-.043),(AX+.043,.165)]:
 for i in range(18):
  xa=lo+i*(hi-lo)/18;xb=lo+(i+1)*(hi-lo)/18
  ylim=min(.112,np.sqrt(max(0,.169**2-((xa+xb)/2)**2)))
  deck_rect(f'panel{lo}_{i}',xa,xb,-ylim,ylim)
# The real rear LiDAR turret faces downward; the folded service towel receives it.
add(bp,bc,'lidar',m.cylinder(.035,.016,(-.075,0,.008)), 'body')
add(bp,bc,'lidar_lens',m.cylinder(.032,.004,(-.075,0,.003)),'lens')
for side in (-1,1):
 add(bp,bc,f'wheel{side}',m.cylinder(.034,.028,(-.022,side*.135,.0625),axis='y'),'black')
 for j in range(12):
  a=j*2*np.pi/12
  add(bp,bc,f'tread{side}_{j}',m.box((.008,.029,.004)).turned(j*30,'y').moved((-.022+.033*np.sin(a),side*.135,.0625+.033*np.cos(a))),'black',False)
add(bp,bc,'caster',m.cylinder(.018,.018,(-.127,.014,.0785),axis='y'),'black')
for j in range(5):
 a=2*np.pi*j/5
 add(bp,bc,f'sidebrush{j}',m.tube([(-.095,-.105,.092),(-.095+.045*np.cos(a),-.105+.045*np.sin(a),.092)],.003),'black',False)
# Brush trough is moulded into the floating cleaning head. It is held at rest here.
add(bp,bc,'trough_floor',m.box((.076,.214,.004),(AX,0,.0505)),'grey')
for xx in (AX-.040,AX+.040):add(bp,bc,f'trough_{xx}',m.box((.005,.218,.029),(xx,0,.0655)),'grey')
# The flat-sided outer bearing sits in an open pocket at +Y.
# Its inner race rotates with the brush; the outer block is a separate link.
for xx in (-.0068,.0068):add(bp,bc,f'bearing_side{xx}',m.box((.004,.004,.012),(AX+xx,.096,AZ-.002)),'grey')
add(bp,bc,'bearing_floor',m.box((.0136,.004,.004),(AX,.096,AZ-.006)),'grey')
# Motor coupling: recessed female square socket; the brush carries the male peg.
# This topology follows the official roller photograph and the OEM gearbox views.
dp=[];dc=[]
# Circular output cup around the square recess. The earlier30mm square exterior
# intersected the cover; this22mm diameter estimate is consistent with the23mm
# adjacent cap envelope. The8.6mm square contact aperture is unchanged.
for i in range(16):
 a=i*2*np.pi/16;b=(i+1)*2*np.pi/16
 def inner(theta):
  radius=.0043/max(abs(np.cos(theta)),abs(np.sin(theta)))
  return (radius*np.cos(theta),radius*np.sin(theta))
 outline=[inner(a),(.011*np.cos(a),.011*np.sin(a)),(.011*np.cos(b),.011*np.sin(b)),inner(b)]
 shape=m.prism(outline,.094,.104).turned(90,'x').moved((AX,0,AZ))
 add(dp,dc,f'drive_socket_wall{i}',shape,'grey')
add(dp,dc,'drive_back',m.cylinder(.011,.003,(AX,-.1055,AZ),axis='y'),'grey')
# Four rear sockets. Open towards the front; their roofs capture inserted cover hooks.
for i,y in enumerate((-.085,-.035,.035,.085)):
 add(bp,bc,f'hookroof{i}',m.box((.009,.014,.003),(AX+.047,y,.0915)),'grey')
 add(bp,bc,f'hookback{i}',m.box((.003,.014,.011),(AX+.053,y,.0865)),'grey')
# Cover's two outward latch teeth catch under the end lips.
for side in (-1,1):
 add(bp,bc,f'latchlip{side}',m.box((.017,.006,.003),(AX-.033,side*.112,.0915)),'grey')
 add(bp,bc,f'latchwall{side}',m.box((.018,.004,.015),(AX-.033,side*.117,.0845)),'grey')
# Unpowered motor output rotates on its actual brush axis. The square key is
# coupled only by collider contact; no joint or pose snapping attaches the roller.
# Gear drag is omitted in this geometric engagement prototype. Motor mass is a
# partition of the manual's approximate 4.7 kg total, not an OEM measurement.
DRIVE=m.hinge('motor_socket','Y',(AX,-.099,AZ),-math.inf,math.inf,dp,dc,
 mass=.020,spring=0.,damping=0.,friction=0.,armature=0.,gravity=True,
 com=(AX,-.100,AZ),inertia=(2e-6,3e-6,2e-6))
ROBOT=m.mechanism('vacuum',tuple(C),[DRIVE],LOOK,frame_parts=bp,frame_colliders=bc,
 free_mass=4.464,friction=(.55,.45),solver_iters=64,frame_inertia=(.037,.037,.070))

# Roller is free, including its real flanged caps and rubber helical vanes.
rp=[];rc=[]
add(rp,rc,'core',m.cylinder(.013,.170,(0,0,0),axis='y'),'rubber')
for j in range(6):
 for k in range(14):
  y0=-.085+.170*k/14;y1=-.085+.170*(k+1)/14
  a0=j*np.pi/3+.42*np.sin(np.pi*y0/.170);a1=j*np.pi/3+.42*np.sin(np.pi*y1/.170)
  verts=[]
  for y,a in ((y0,a0),(y1,a1)):
   for r,da in ((.012,-.035),(.019,-.035),(.019,.035),(.012,.035)):
    verts.append((r*np.cos(a+da),y,r*np.sin(a+da)))
  mesh=m.Mesh(np.array(verts),np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]))
  add(rp,rc,f'vane{j}_{k}',mesh,'rubber')
for side in (-1,1):
 # Photo-informed reconstruction: neck ends flush with the outer flange
 # at93.5mm, not0.5mm beyond it. Exact OEM neck dimensions are unpublished.
 add(rp,rc,f'cap_neck{side}',m.cylinder(.007,.0085,(0,side*.08925,0),axis='y'),'grey')
 for y,r,h in ((side*.0875,.0115,.003),(side*.092,.011,.003)):
  add(rp,rc,f'cap{side}_{y}',m.cylinder(r,h,(0,y,0),axis='y'),'grey')
# Male square drive peg goes into the housing socket, never a software coupling.
add(rp,rc,'drive_peg',m.box((.008,.005,.008),(0,-.096,0)),'black')
# Bearing inner shaft is part of the rotor. Its outer sleeve is free to rotate
# relative to it and fits the housing by flat-sided contact, not a joint to it.
add(rp,rc,'bearing_axle',m.cylinder(.0025,.005,(0,.096,0),axis='y'),'metal')
bearing_parts=[];bearing_cols=[]
add(bearing_parts,bearing_cols,'bearing_outer',m.box((.009,.005,.008),(0,.096,0)),'black')
BEARING=m.hinge('outer_bearing','Y',(0,.096,0),-math.inf,math.inf,
 bearing_parts,bearing_cols,mass=.005,spring=0.,damping=0.,friction=0.,
 armature=0.,gravity=True,com=(0,.096,0),inertia=(4e-8,6e-8,5e-8))
rollq=(math.sin(math.radians(-5)/2),0,0,math.cos(math.radians(-5)/2))
ROLLER=m.mechanism('roller',tuple(C+np.array([AX,.001,AZ+.010])),[BEARING],LOOK,
 frame_parts=rp,frame_colliders=rc,free_mass=.130,friction=(.75,.55),solver_iters=64,
 start_quat=rollq,frame_inertia=(.00038,.000025,.00038))

# Separate free cover. Four fixed hooks and two sliding spring catches physically retain it.
cp=[];cc=[]
# Photo-proportional22mm front/rear rails leave a36mm short opening.
add(cp,cc,'rear_edge',m.box((.022,.230,.006),(.029,0,0)),'black')
add(cp,cc,'front_edge_middle',m.box((.022,.216,.006),(-.029,0,0)),'black')
# Molded underside clearances admit the existing housing receiver lips.
# Upper surface and230mm outer width are preserved; no extra robot access.
for side in (-1,1):
 add(cp,cc,f'front_corner_roof{side}',m.box((.022,.007,.002),(-.029,side*.1115,.002)),'black')
 add(cp,cc,f'end{side}',m.box((.036,.022,.006),(0,side*.104,0)),'black')
for i,y in enumerate((-.085,-.035,.035,.085)):
 add(cp,cc,f'hook{i}',m.box((.012,.010,.003),(.044,y,-.004)),'black')
 add(cp,cc,f'hookneck{i}',m.box((.004,.010,.008),(.037,y,-.001)),'black')
 add(cp,cc,f'guard{i}',m.box((.009,.007,.003),(.025,y,.003)),'black')
add(cp,cc,'squeegee',m.box((.002,.204,.007),(-.027,0,-.002)),'grey')
links=[]
for side,n in ((-1,'right_catch'),(1,'left_catch')):
 p=[];col=[]
 add(p,col,n+'_bar',m.box((.010,.046,.006),(-.034,side*.086,-.003)),'black')
 add(p,col,n+'_pad',m.box((.014,.009,.006),(-.037,side*.066,0)),'grey')
 # Manual p29 specifies press-to-lock. A lower lead-in cams the spring catch
 # inward, while the upper shoulder retains it. This 45-degree profile and
 # 0.5mm fully-retracted clearance are disclosed reconstruction estimates,
 # not measured OEM dimensions. The old rectangular prism had no lead-in.
 # Coordinates below are (outward Y, Z); extrusion is along X.
 yz=[(.106,-.008),(.1085,-.008),(.1125,-.004),(.106,-.004)]
 vertices=np.array([(x,side*y,z) for x in (-.042,-.026) for y,z in yz])
 faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],
                 [0,1,5],[0,5,4],[1,2,6],[1,6,5],
                 [2,3,7],[2,7,6],[3,0,4],[3,4,7]])
 if side<0: faces=faces[:,::-1]
 add(p,col,n+'_tooth',m.Mesh(vertices,faces),'black')
 links.append(m.slider(n,'Y',-.004 if side>0 else 0,0 if side>0 else .004,p,col,mass=.008,spring=650.,damping=.8,friction=0.,gravity=True))
COVER=m.mechanism('cover',(.37,-.285,TOP+.013),links,LOOK,frame_parts=cp,frame_colliders=cc,free_mass=.065,friction=(.65,.5),solver_iters=64,frame_inertia=(.0004,.00008,.0004))
OBJECTS=[ROBOT,ROLLER,COVER,
 SimReady('towel',pos=(.40,0,TOP),size=(.409,.209,.017),mass=.210,package='consumable_microfiber_towel_folded_large_a01',usd='sm_consumable_microfiber_towel_folded_large_a01_01.usd'),
 SimReady('mug',pos=(.63,.25,TOP),size=(.114,.09,.08),mass=.3,package='dishware_mug_a01',usd='sm_dishware_mug_a01_01.usda'),
 SimReady('tape',pos=(.64,-.25,TOP),size=(.10,.10,.05),mass=.06,package='tape_clear_a01',usd='sm_tape_clear_a01_01.usd')]
class Trial:
 def reset(self,env=None):self.steps=0
 def step(self,env):
  self.steps+=1
  if self.steps == 1 or self.steps%20==0:
   print('VACUUM_STATE',self.steps,{n:dict(pos=m.state(env,n).pos.tolist(),quat=m.state(env,n).quat_xyzw.tolist(),joints=m.state(env,n).joints) for n in ('vacuum','roller','cover')},flush=True)
T=Trial();T.reset();m.on_reset(T.reset);m.on_step(T.step)
def score(objects):return 0.
def success(objects):return False
def demo():return {}
