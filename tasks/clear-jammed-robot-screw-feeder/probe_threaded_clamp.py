"""Physical finite-thread/driver contact gate; score0, not complete repair.

M3x0.5 and AF2.5 are disclosed reconstruction from the service photograph.
Native head/shaft axial spring represents steel bolt stretch, not a plate lock.
Nominal hand-tight0.10Nm / K0.2 / M3 gives167N as an engineering load estimate,
not OEM calibration. Finite SDF threads permit complete physical unscrewing.
"""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from shapely.geometry import Point,Polygon,box as pbox
from shapely import constrained_delaunay_triangles
from yam_arena import ik,demo as motion
PITCH=.0005;MAJOR=.0015;E=200e9;AREA=5.03e-6;LENGTH=.006;BOLT_K=E*AREA/LENGTH
PRELOAD_EST=.10/(.2*.003);STRETCH=PRELOAD_EST/BOLT_K
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
    if internal:return .001229367+.000270633*np.interp(u,[0,.125,.4375,.5625,.875,1],[0,0,1,1,0,0])+.000010
    return .001193283+.000306717*np.interp(u,[0,1/12,.4375,.5625,11/12,1],[0,0,1,1,0,0])

def closed_shaft():
    zs=np.linspace(0,.006,241);v,f=radial([thread_r(z) for z in zs],zs);f=f.tolist()
    for idx,up in [(0,False),(len(zs)-1,True)]:
        k=len(v);v=np.vstack((v,[0,0,zs[idx]]))
        for j in range(N):
            tri=(k,idx*N+j,idx*N+(j+1)%N);f.append(tri if up else tri[::-1])
    return m.Mesh(v,np.array(f)).turned(90,'y')

def bore():
    zs=np.linspace(.0012,.0072,241);iv,iff=radial([thread_r(z) for z in zs],zs,True);ov,off=radial([np.full(N,.0035)]*2,[zs[0],zs[-1]])
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

# Remove fixed diagnostic bracket/heads/bosses; keep original loose case.
cp2=[x for x in cp if not x[0].startswith(('holding_plate','bracket','boss','attaching_head'))]
cc2=[col(n+'_c',g) for n,g,_ in cp2]
for i,y in enumerate([.008,.028]):
    g=bore().moved((.043,y,.099));add(cp2,cc2,'thread_boss'+str(i),g,'steel',True)
CASE=safe(m.free('feeder',CASE_POS,cp2,cc2,LOOK,mass=2.9805,friction=(.3,.2),solver_iters=64,com=(.098,0,.065)))
pp,pc=[],[]
# 0.5mm nominal head clearance:84.7rail+2.4panhead+0.5clearance.
add(pp,pc,'horizontal_plate',m.box((.031,.032,.0012),(.0275,.018,.0882)))
outline=pbox(.002,.0888,.034,.1188)
for y in [.008,.028]:
    slot=Point(y,.091).buffer(.0017,quad_segs=12).union(Point(y,.115).buffer(.0017,quad_segs=12)).union(pbox(y-.0017,.091,y+.0017,.115))
    outline=outline.difference(slot)
for i,g in enumerate(convex_cells(outline)):
    raw=m.prism(list(g.exterior.coords)[:-1],.043,.0442)
    mesh=m.Mesh(raw.v[:,[2,0,1]],raw.f)
    add(pp,pc,'slotted_bracket'+str(i),mesh)
PLATE=safe(m.free('holding_plate',CASE_POS,pp,pc,LOOK,mass=.018,friction=(.25,.2),solver_iters=64,torsional_patch=None))
BOLTS=[]
for i,y in enumerate([.008,.028]):
    sp,sc=[],[];add(sp,sc,'finite_thread',closed_shaft(),'steel',True)
    hp2,hc2=[],[];socket=Polygon(np.c_[.00255/math.sqrt(3)*np.cos(np.arange(6)*math.pi/3),.00255/math.sqrt(3)*np.sin(np.arange(6)*math.pi/3)])
    # Standard-size head with real female hex socket, not a pre-keyed projection.
    ann=Point(0,0).buffer(.00275,quad_segs=24).difference(socket)
    phase=.000010/math.sqrt(3)+STRETCH
    for j,g in enumerate(convex_cells(ann)):
        mesh=m.prism(list(g.exterior.coords)[:-1],-.003-phase,-.0015-phase).turned(90,'y');add(hp2,hc2,'socketwall'+str(j),mesh)
    add(hp2,hc2,'headbase',m.cylinder(.00275,.0015,(-.00075-phase,0,0),axis='x'))
    head=m.slider('head_stretch','X',-.0005,.0005,hp2,hc2,mass=.00035,spring=BOLT_K,damping=.2,friction=0,effort=1000,armature=0,gravity=True,com=(-.0015-phase,0,0),inertia=(1.3e-9,1.e-9,1.e-9))
    bolt=safe(m.mechanism('bolt'+str(i),CASE_POS+[.043+phase,y,.099],[head],LOOK,frame_parts=sp,frame_colliders=sc,free_mass=.000325,frame_inertia=(3.7e-10,1.16e-9,1.16e-9),friction=(.2,.15),solver_iters=64))
    BOLTS.append(bolt)
OBJECTS=[CASE,PLATE,*BOLTS,DRIVER]
def preload(env):
    for i in range(2):m.set_target(env,'bolt'+str(i),'head_stretch',STRETCH)
m.on_reset(preload)
class ClampObserve:
    def reset(self,env=None):self.n=0;self.view=None
    def step(self,env):
        self.n+=1
        if self.view is None:
            from isaaclab_physx.physics import PhysxManager
            import isaaclab.sim as su
            from pxr import UsdPhysics
            bodies=[str(p.GetPath()) for p in su.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            sensors=[p for p in bodies if '/bolt' in p and p.endswith('/head_stretch')]
            filters=[p for p in bodies if '/holding_plate/' in p or p.endswith('/holding_plate')]
            print('CLAMP_CONTACT_PATHS',sensors,filters,flush=True)
            self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=2048)
        if self.n==1 or self.n%10==0:
            row={}
            for name in ['holding_plate','bolt0','bolt1']:
                s=m.state(env,name);row[name]=dict(p=s.pos.tolist(),q=s.quat_xyzw.tolist(),v=s.lin_vel.tolist(),joints=s.joints)
            r=env.unwrapped.scene['right_robot'];row['arm_q']=m._np(r.data.joint_pos)[0].tolist();row['head_plate_force_N']=self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist()
            print('CLAMP_GATE',self.n,row,flush=True)
C=ClampObserve();m.on_reset(C.reset);m.on_step(C.step)
def demo():
    p=np.array([.300,-.170,TOP+.017])
    return {'right':[Waypoint(1.5,tuple(p+[0,0,.080]),-.025,'approach driver'),Waypoint(1.5,tuple(p),-.025,'seat on narrow waist'),Waypoint(1.5,tuple(p),0.,'grasp'),Waypoint(2.,tuple(p+[0,0,.100]),0.,'lift'),Waypoint(.5,tuple(p+[0,0,.100]),0.,'measure grasp')]}
