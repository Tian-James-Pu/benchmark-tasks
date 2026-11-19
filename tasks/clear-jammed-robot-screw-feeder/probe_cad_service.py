"""Drawing-corrected native service gate; no complete repair score.
Original side-facingM2.6 threads, fixed slotted bracket and movable threaded
L-plate. Actual bolt/driver contact. Preload estimate is not OEM calibration.
"""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from shapely.geometry import Point,Polygon,box as pbox
from shapely import constrained_delaunay_triangles
from yam_arena import ik,demo as motion
from yam_arena.urdf import axis_angle_matrix
PITCH=.00045;E=200e9;AREA=3.70e-6;LENGTH=.005;BOLT_K=E*AREA/LENGTH
PRELOAD_EST=.10/(.2*.0026);STRETCH=PRELOAD_EST/BOLT_K
N=96;theta=np.arange(N)*2*math.pi/N
def radial(rs,zs,inward=False):
    v=np.concatenate([np.c_[r*np.cos(theta),r*np.sin(theta),np.full(N,z)] for r,z in zip(rs,zs)])
    f=[]
    for i in range(len(zs)-1):
        for j in range(N):
            a=i*N+j;b=i*N+(j+1)%N;f.extend([(a,b,b+N),(a,b+N,a+N)])
    f=np.array(f)
    return v,f[:,::-1] if inward else f

def thread_r(z,internal=False):
    u=np.mod(z/PITCH-theta/(2*math.pi),1)
    if internal:return .001056431+.000243569*np.interp(u,[0,.125,.4375,.5625,.875,1],[0,0,1,1,0,0])+.000010
    return .001023955+.000276045*np.interp(u,[0,1/12,.4375,.5625,11/12,1],[0,0,1,1,0,0])

def closed_shaft():
    zs=np.linspace(0,.005,241);v,f=radial([thread_r(z) for z in zs],zs);f=f.tolist()
    for idx,up in [(0,False),(len(zs)-1,True)]:
        k=len(v);v=np.vstack((v,[0,0,zs[idx]]))
        for j in range(N):
            tri=(k,idx*N+j,idx*N+(j+1)%N);f.append(tri if up else tri[::-1])
    return m.Mesh(v,np.array(f)).turned(90,'y')

def bore():
    zs=np.linspace(.0027,.0039,61);iv,iff=radial([thread_r(z) for z in zs],zs,True);ov,off=radial([np.full(N,.0021)]*2,[zs[0],zs[-1]])
    v=np.vstack((ov,iv));f=np.vstack((off,iff+len(ov))).tolist()
    for oi,ii,up in [(0,len(ov),False),(N,len(ov)+(len(zs)-1)*N,True)]:
        for j in range(N):
            k=(j+1)%N;t=[(oi+j,ii+k,oi+k),(oi+j,ii+j,ii+k)];f.extend([x[::-1] for x in t] if up else t)
    return m.Mesh(v,np.array(f)).turned(90,'y')

def convex_cells(poly):
    cells=list(constrained_delaunay_triangles(poly).geoms)
    while True:
        edges={};pairs=[]
        for i,g in enumerate(cells):
            xy=list(g.exterior.coords)
            for a,b in zip(xy,xy[1:]):
                key=tuple(sorted((a,b)))
                if key in edges:pairs.append((edges[key],i))
                else:edges[key]=i
        used=set();new=[]
        for i,j in pairs:
            if i in used or j in used:continue
            u=cells[i].union(cells[j])
            if u.geom_type=='Polygon' and abs(u.area-u.convex_hull.area)<1e-16:used.update([i,j]);new.append(u)
        if not used:return cells
        cells=[g for i,g in enumerate(cells) if i not in used]+new

# Enable native reports before instantiation; no forces/constraints changed.
def reported(p):
    b=p.build
    return replace(p,build=lambda:b().replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysxContactReportAPI"').replace('float physics:mass =','float physxContactReport:threshold = 0\n        float physics:mass ='))
# Keep case shell only, corrected1.5mm foot/body datum in front step.
cp3=[]
for n,g,l in cp:
    if n=='main_front' or n.startswith(('holding_plate','bracket','boss','attaching_head','rail','escaper')):continue
    if n in ('step','pickup_sensor'):g=g.moved((0,0,.0015))
    cp3.append((n,g,l))
cc3=[col(n+'_c',g) for n,g,_ in cp3]
# Manufacturer front-wall notch, not an added access window. Coordinates follow
# CADx129mm face boundary; it accommodates the original plate and rail.
wall=pbox(-.0585,.0892,.0583,.1345)
notch=Polygon([(-.0076,.080),(-.0076,.0892),(-.0046,.0922),(-.0046,.0973),(.0201,.0973),(.0201,.105),(.0217,.105),(.0217,.0898),(.0209,.0898),(.0209,.080)])
wall=wall.difference(notch)
for i,g in enumerate(convex_cells(wall)):
    raw=m.prism(list(g.exterior.coords)[:-1],.0508,.0524)
    add(cp3,cc3,'main_front_panel'+str(i),m.Mesh(raw.v[:,[2,0,1]],raw.f),'case')

# Fixed original outer slotted bracket.22mm length,1.6mm thickness.
outline=pbox(.0288,.0878,.0508,.1166)
for x in [.0328,.0448]:
    slot=Point(x,.095).buffer(.0015,quad_segs=12).union(Point(x,.107).buffer(.0015,quad_segs=12)).union(pbox(x-.0015,.095,x+.0015,.107))
    outline=outline.difference(slot)
for i,g in enumerate(convex_cells(outline)):
    raw=m.prism(list(g.exterior.coords)[:-1],.0201,.0217)
    add(cp3,cc3,'fixed_slot'+str(i),m.Mesh(raw.v[:,[0,2,1]],raw.f[:,::-1]))
add(cp3,cc3,'fixed_bracket_foot',m.box((.011,.012,.0016),(.0423,.0261,.087)))
# Actual32mm escaper envelope. Powered rotation/feed is a subsequent gate.
add(cp3,cc3,'escaper',m.cylinder(.016,.0016,(.0226,0,.0854065)))
for y in [-.0031,.0031]:add(cp3,cc3,'rail'+str(y),m.box((.029,.003,.0045),(.0381,y,.08695)))
CASE=reported(safe(m.free('feeder',CASE_POS,cp3,cc3,LOOK,mass=2.9913,friction=(.3,.2),solver_iters=64,com=(.098,0,.065))))
# The original L-plate goes on the INNER side and contains the threaded holes.
pp3,pc3=[],[]
platepoly=pbox(.0201,-.0016,.0524,.0189).difference(Point(.0226,0).buffer(.009,quad_segs=32))
for i,g in enumerate(convex_cells(platepoly)):add(pp3,pc3,'holding_plate'+str(i),m.prism(list(g.exterior.coords)[:-1],.0921,.0933))
back=pbox(.0288,.0933,.0498,.105)
for x in [.0328,.0448]:back=back.difference(Point(x,.101).buffer(.0021,quad_segs=24))
for i,g in enumerate(convex_cells(back)):
    raw=m.prism(list(g.exterior.coords)[:-1],.0189,.0201);add(pp3,pc3,'inner_bracket'+str(i),m.Mesh(raw.v[:,[0,2,1]],raw.f[:,::-1]))
for i,x in enumerate([.0328,.0448]):add(pp3,pc3,'thread_bore'+str(i),bore().turned(-90,'z').moved((x,.0228,.101)),'steel',True)
PLATE=reported(safe(m.free('holding_plate',CASE_POS,pp3,pc3,LOOK,mass=.0073,friction=(.2,.15),solver_iters=64,torsional_patch=None)))
BOLTS=[]
for i,x in enumerate([.0328,.0448]):
    sp,sc=[],[];add(sp,sc,'finite_thread',closed_shaft(),'steel',True)
    hp3,hc3=[],[];socket=Polygon(np.c_[.00205/math.sqrt(3)*np.cos(np.arange(6)*math.pi/3),.00205/math.sqrt(3)*np.sin(np.arange(6)*math.pi/3)])
    phase=.000010/math.sqrt(3)+STRETCH
    ann=Point(0,0).buffer(.00225,quad_segs=24).difference(socket)
    for j,g in enumerate(convex_cells(ann)):add(hp3,hc3,'socketwall'+str(j),m.prism(list(g.exterior.coords)[:-1],-.0026-phase,-.0013-phase).turned(90,'y'))
    add(hp3,hc3,'headbase',m.cylinder(.00225,.0013,(-.00065-phase,0,0),axis='x'))
    # Captive original washer stack represented in the head body at this stage.
    # Independent washer rotation/spring compliance is a disclosed limitation.
    for k,(rad,z0,z1) in enumerate([(.0026,0,.0006),(.00375,.0006,.0011)]):
        ann=Point(0,0).buffer(rad,quad_segs=24).difference(Point(0,0).buffer(.00135,quad_segs=24))
        for j,g in enumerate(convex_cells(ann)):add(hp3,hc3,'washer'+str(k)+'_'+str(j),m.prism(list(g.exterior.coords)[:-1],z0-phase,z1-phase).turned(90,'y'))
    h=m.slider('head_stretch','X',-.0005,.0005,hp3,hc3,mass=.0005,spring=BOLT_K,damping=.2,friction=0,effort=1000,armature=0,gravity=True,com=(-.0003-phase,0,0),inertia=(2.e-9,1.7e-9,1.7e-9))
    b=safe(m.mechanism('bolt'+str(i),CASE_POS+[x,.0228-phase,.101],[h],LOOK,frame_parts=sp,frame_colliders=sc,free_mass=.0002,frame_inertia=(1.7e-10,4.5e-10,4.5e-10),friction=(.2,.15),solver_iters=64,yaw_deg=-90))
    BOLTS.append(reported(b))
# Corrected2mm driver model42424: identical100mm handle,50mm blade.
hp4,hc4=[],[]
for n,g,l in hp:
    if n in ('shaft','hex_tip'):continue
    add(hp4,hc4,n,g.moved((.010,0,0)),l,n=='tapered_handle')
add(hp4,hc4,'shaft',m.cylinder(.0013,.045,(-.0275,0,0),axis='x'))
a=np.arange(6)*math.pi/3;rad=.002/math.sqrt(3);add(hp4,hc4,'hex_tip',m.prism(np.c_[rad*np.cos(a),rad*np.sin(a)],-.005,0).turned(90,'y'))
ep4,ec4=[],[]
for n,g,l in ep:add(ep4,ec4,n,g.moved((.010,0,0)),l,True)
caplink=m.hinge('cap','X',(0,0,0),-float('inf'),float('inf'),ep4,ec4,mass=.003,gravity=True,friction=0,damping=0,armature=0,com=(-.140,0,0),inertia=(1.1e-7,1.5e-7,1.5e-7))
DRIVER=reported(safe(m.mechanism('driver',(.30,.120,TOP+.0092),[caplink],LOOK,frame_parts=hp4,frame_colliders=hc4,free_mass=.014,frame_inertia=(2e-7,1.7e-5,1.7e-5),friction=(.6,.45),solver_iters=64,yaw_deg=-90)))
OBJECTS=[CASE,PLATE,*BOLTS,DRIVER]
# Old right-gripper observation is retained; new records include the actual left.
class CADObserve:
    def reset(self,env=None):self.n=0;self.view=None
    def step(self,env):
        self.n+=1
        if self.view is None:
            import isaaclab.sim as su
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[str(p.GetPath()) for p in su.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            sensors=[p for p in bodies if '/bolt' in p and p.endswith('/head_stretch')];filters=[p for p in bodies if p.endswith('/holding_plate') or p.endswith('/feeder')]
            self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=2048)
        if self.n==1 or self.n%10==0:
            row={}
            for name in ['holding_plate','bolt0','bolt1','driver']:
                s=m.state(env,name);row[name]=dict(p=s.pos.tolist(),q=s.quat_xyzw.tolist(),v=s.lin_vel.tolist(),joints=s.joints)
            r=env.unwrapped.scene['left_robot'];row['arm_q']=m._np(r.data.joint_pos)[0].tolist();row['effort']=m._np(r.data.applied_torque)[0].tolist();row['head_plate_case_force_N']=self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist()
            print('CAD_SERVICE_GATE',self.n,row,flush=True)
    def preload(self,env):
        self.reset(env)
        for i in range(2):m.set_target(env,'bolt'+str(i),'head_stretch',STRETCH)
CO=CADObserve();m.on_reset(CO.preload);m.on_step(CO.step)
def demo():
    p=np.array([.300,.220,TOP+.017]);yaw=math.radians(270)
    return {'left':[Waypoint(1.5,tuple(p+[0,0,.080]),-.025,'approach driver',yaw),Waypoint(1.5,tuple(p),-.025,'seat on narrow waist',yaw),Waypoint(1.5,tuple(p),0.,'grasp',yaw),Waypoint(2.,tuple(p+[0,0,.100]),0.,'lift',yaw),Waypoint(.5,tuple(p+[0,0,.100]),0.,'measure grasp',yaw)]}
