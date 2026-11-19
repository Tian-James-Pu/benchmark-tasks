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
C=np.array([.40,0,TOP+.017]); AX=.018; AZ=.087
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
# Side sections of the underside retain the actual 220mm wide brush opening.
for side in (-1,1):
 for i in range(16):
  yy=side*(.112+i*.0036);xlim=np.sqrt(max(0,.169**2-yy**2))
  add(bp,bc,f'underside{side}_{i}',m.box((2*xlim,.0036,.006),(0,yy,.086)))
for lo,hi in [(-.165,AX-.043),(AX+.043,.165)]:
 for i in range(18):
  xx=lo+(i+.5)*(hi-lo)/18;ylim=min(.112,np.sqrt(max(0,.169**2-xx**2)))
  add(bp,bc,f'panel{lo}_{i}'.replace('-','m').replace('.','p'),m.box(((hi-lo)/18,2*ylim,.006),(xx,0,.086)))
# The real rear LiDAR turret faces downward; the folded service towel receives it.
add(bp,bc,'lidar',m.cylinder(.035,.016,(-.075,0,.008)), 'body')
add(bp,bc,'lidar_lens',m.cylinder(.032,.004,(-.075,0,.003)),'lens')
for side in (-1,1):
 add(bp,bc,f'wheel{side}',m.cylinder(.034,.028,(-.022,side*.135,.081),axis='y'),'black')
 for j in range(12):
  a=j*2*np.pi/12
  add(bp,bc,f'tread{side}_{j}',m.box((.008,.029,.004)).turned(j*30,'y').moved((-.022+.033*np.sin(a),side*.135,.081+.033*np.cos(a))),'black',False)
add(bp,bc,'caster',m.cylinder(.018,.018,(-.127,.014,.095),axis='y'),'black')
for j in range(5):
 a=2*np.pi*j/5
 add(bp,bc,f'sidebrush{j}',m.tube([(-.095,-.105,.098),(-.095+.045*np.cos(a),-.105+.045*np.sin(a),.098)],.003),'black',False)
# Brush trough is moulded into the floating cleaning head. It is held at rest here.
add(bp,bc,'trough_floor',m.box((.076,.214,.004),(AX,0,.060)),'grey')
for xx in (AX-.040,AX+.040):add(bp,bc,f'trough_{xx}',m.box((.005,.218,.029),(xx,0,.075)),'grey')
# Round bearing sits in an open, shallow U pocket at +Y.
for xx in (-.007,.007):add(bp,bc,f'bearing_side{xx}',m.box((.004,.008,.014),(AX+xx,.107,AZ-.001)),'grey')
add(bp,bc,'bearing_floor',m.box((.014,.008,.004),(AX,.107,AZ-.007)),'grey')
# Motor coupling: recessed female square socket; the brush carries the male peg.
# This topology follows the official roller photograph and the OEM gearbox views.
for x in (-.00965,.00965):add(bp,bc,f'drive_socket_x{x}',m.box((.0107,.010,.030),(AX+x,-.107,AZ)),'grey')
for z in (-.00965,.00965):add(bp,bc,f'drive_socket_z{z}',m.box((.0086,.010,.0107),(AX,-.107,AZ+z)),'grey')
add(bp,bc,'drive_back',m.box((.030,.003,.030),(AX,-.114,AZ)),'grey')
# Four rear sockets. Open towards the front; their roofs capture inserted cover hooks.
for i,y in enumerate((-.085,-.035,.035,.085)):
 add(bp,bc,f'hookroof{i}',m.box((.009,.014,.003),(AX+.047,y,.101)),'grey')
 add(bp,bc,f'hookback{i}',m.box((.003,.014,.011),(AX+.053,y,.096)),'grey')
# Cover's two outward latch teeth catch under the end lips.
for side in (-1,1):
 add(bp,bc,f'latchlip{side}',m.box((.017,.006,.003),(AX-.033,side*.112,.101)),'grey')
 add(bp,bc,f'latchwall{side}',m.box((.018,.004,.015),(AX-.033,side*.117,.094)),'grey')
ROBOT=m.free('vacuum',tuple(C),bp,bc,LOOK,mass=4.55,friction=(.55,.45),solver_iters=64)

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
 for y,r,h in ((side*.089,.0115,.003),(side*.094,.011,.003)):
  add(rp,rc,f'cap{side}_{y}',m.cylinder(r,h,(0,y,0),axis='y'),'grey')
# Male square drive peg goes into the housing socket, never a software coupling.
add(rp,rc,'drive_peg',m.box((.008,.012,.008),(0,-.101,0)),'black')
add(rp,rc,'bearing_axle',m.cylinder(.0045,.012,(0,.103,0),axis='y'),'black')
rollq=(math.sin(math.radians(-5)/2),0,0,math.cos(math.radians(-5)/2))
ROLLER=m.free('roller',tuple(C+np.array([AX,.001,AZ+.010])),rp,rc,LOOK,mass=.135,friction=(.75,.55),solver_iters=64,start_quat=rollq)

# Separate free cover. Four fixed hooks and two sliding spring catches physically retain it.
cp=[];cc=[]
for xx in (-.034,.034):add(cp,cc,f'long_edge{xx}',m.box((.012,.230,.006),(xx,0,0)),'black')
for y in (-.109,.109):add(cp,cc,f'end{y}',m.box((.058,.012,.006),(0,y,0)),'black')
for i,y in enumerate((-.085,-.035,.035,.085)):
 add(cp,cc,f'hook{i}',m.box((.012,.010,.003),(.044,y,-.004)),'black')
 add(cp,cc,f'hookneck{i}',m.box((.004,.010,.008),(.037,y,-.001)),'black')
 add(cp,cc,f'guard{i}',m.box((.009,.007,.003),(.025,y,.003)),'black')
add(cp,cc,'squeegee',m.box((.002,.204,.007),(-.027,0,-.002)),'grey')
links=[]
for side,n in ((-1,'right_catch'),(1,'left_catch')):
 p=[];col=[]
 add(p,col,n+'_bar',m.box((.010,.046,.006),(-.034,side*.086,-.003)),'black')
 add(p,col,n+'_pad',m.box((.014,.009,.010),(-.037,side*.066,.002)),'grey')
 # Sloped tooth nose retracts the spring slider as cover is pressed into the lip.
 outline=[(-.042,side*.106),(-.026,side*.106),(-.026,side*.113),(-.042,side*.113)]
 if side<0:outline=outline[::-1]
 add(p,col,n+'_tooth',m.prism(outline,-.008,-.004),'black')
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
  if self.steps in (1,20,40,60,100,200):
   print('VACUUM_STATE',self.steps,{n:(m.state(env,n).pos.tolist(),m.state(env,n).quat_xyzw.tolist()) for n in ('vacuum','roller','cover')},flush=True)
T=Trial();T.reset();m.on_reset(T.reset);m.on_step(T.step)
def score(objects):return 0.
def success(objects):return False
def demo():
 return {'left':[
  Waypoint(3.,(.40,.10,TOP+.25),FINGER_OPEN,'approach brush',-math.pi/2),
  Waypoint(2.,(.431,.006,TOP+.18),FINGER_OPEN,'above brush',-math.pi/2),
  Waypoint(2.,(.431,.006,.865),FINGER_OPEN,'straddle roller',-math.pi/2),
  Waypoint(1.,(.431,.006,.865),FINGER_CLOSED,'grip roller',-math.pi/2),
  Waypoint(2.,(.431,.006,.915),FINGER_CLOSED,'lift from bay',-math.pi/2),
  Waypoint(2.,(.431,.006,.915),FINGER_CLOSED,'hold inspection',-math.pi/2)]}

