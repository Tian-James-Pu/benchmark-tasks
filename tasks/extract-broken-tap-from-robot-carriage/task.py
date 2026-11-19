"""SR052 full-scale grasp/contact development gate. All scores remain zero.

AR4 carriage and Guhring302 M8 dimensions are manufacturer sourced. Walton10313
holder/flute/finger sections are disclosed reconstructions. This first gate uses
two bending planes per finger; fracture and permanent set are not yet modeled.
"""
import pathlib,sys,math
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Clear the damaged mounting hole so this robot carriage can be assembled.'
SUCCESS_HOLD_S=0.
LOOK={n:m.Look(color=m.rgb(*v),metallic=.65,roughness=.4) for n,v in {'steel':(.55,.57,.60),'aluminum':(.72,.74,.76),'tap':(.18,.17,.20),'vise':(.18,.24,.12),'yellow':(.8,.55,.05)}.items()}
Q=(0.,-math.sqrt(.5),0.,math.sqrt(.5))
PITCH=.00125;MAJOR=.004;MINOR=.0033234
TARGET=np.array([.40,-.07,TOP+.020+.1262])
TOOL_START=np.array([.32,-.19,TOP+.0052])

def coll(name,mesh,sdf=False):
    text=m.col_hull(name,mesh).replace('contactOffset = 0.004','contactOffset = 0.00005')
    if sdf:text=text.replace('"PhysicsMeshCollisionAPI"','"PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"').replace('physics:approximation = "convexHull"','physics:approximation = "sdf"\n int physxSDFMeshCollision:sdfResolution = 512')
    return text

def add(p,c,name,mesh,look='steel',sdf=False):
    p.append((name,mesh,look));c.append(coll(name+'_c',mesh,sdf))

def radial(radii,zs,theta,inner=False):
    n=len(theta);v=np.concatenate([np.c_[r*np.cos(theta),r*np.sin(theta),np.full(n,z)] for z,r in zip(zs,radii)])
    f=[]
    for i in range(len(zs)-1):
        for j in range(n):
            a=i*n+j;b=i*n+(j+1)%n;f.extend([(a,b,b+n),(a,b+n,a+n)])
    f=np.array(f);return v,f[:,::-1] if inner else f

def capped(radii,zs,theta):
    v,f=radial(radii,zs,theta);n=len(theta);f=f.tolist()
    for i,up in ((0,False),(len(zs)-1,True)):
        c=len(v);v=np.vstack((v,[0,0,zs[i]]))
        for j in range(n):
            tri=(c,i*n+j,i*n+(j+1)%n);f.append(tri if up else tri[::-1])
    return m.Mesh(v,np.asarray(f))

def thread_radius(theta,z):
    # ISO60deg basic internal profile: H=0.866025P; radial depth5H/8.
    # Crest/root flats and60deg flanks follow exact basic pitch, no coarse pitch.
    u=np.mod(z/PITCH-theta/(2*math.pi),1.)
    return MINOR+(MAJOR-MINOR)*np.interp(u,[0,.125,.4375,.5625,.875,1],[0,0,1,1,0,0])

def flute_radius(theta,outer):
    out=outer.copy()
    for a in (0,2*math.pi/3,4*math.pi/3):
        delta=np.arctan2(np.sin(theta-a),np.cos(theta-a));d=.0032;radius=.0015
        valid=(np.abs(delta)<math.pi/2)&(np.abs(d*np.sin(delta))<radius)
        cut=d*np.cos(delta)-np.sqrt(np.maximum(0,radius**2-(d*np.sin(delta))**2))
        out=np.where(valid,np.minimum(out,cut),out)
    return out

# Real finite thread surfaces. Thin annular zone is part of the original plate,
# not a separate insert or an enlarged bore. Remaining plate exact planar union.
theta=np.linspace(0,2*math.pi,192,endpoint=False)
zs=np.linspace(-.00635,0,154)
rv=[thread_radius(theta,z)+.000025 for z in zs]
iv,iff=radial(rv,zs,theta,True);ov,off=radial([np.full(192,.006)]*2,[-.00635,0],theta)
v=np.vstack((ov,iv));f=(np.vstack((off,iff+len(ov)))).tolist()
for o,i,up in ((0,len(ov),False),(192,len(ov)+(len(zs)-1)*192,True)):
    for j in range(192):
        k=(j+1)%192;tris=[(o+j,i+k,o+k),(o+j,i+j,i+k)]
        f.extend([t[::-1] for t in tris] if up else tris)
ring=m.Mesh(v,np.asarray(f));pp,pc=[],[];add(pp,pc,'threaded_hole',ring,'aluminum',True)
from shapely.geometry import Polygon,Point
from shapely import constrained_delaunay_triangles
outline=Polygon([(-.1262,-.035),(.0262,-.035),(.0262,.2698),(-.1262,.2698)])
holes=[(0,0,.006),(-.100,0,.0034),(0,.100,.0034),(-.100,.100,.0034)]
for x in (-.0003,-.0263,-.0737,-.0997):
    for y in (.011,.039,.1958,.2238):holes.append((x,y,.00275))
for x in (-.0732,-.1192):
    for y in (.0844,.1504):holes.append((x,y,.00165))
for x in (-.035,-.065):holes.append((x,.2398,.00165))
for x,y,r in holes:outline=outline.difference(Point(x,y).buffer(r,quad_segs=12))
cells=list(constrained_delaunay_triangles(outline).geoms)
# Merge only adjacent polygons whose exact union is convex. No hole enlargement.
while True:
    edges={};pairs=[]
    for i,g in enumerate(cells):
        pts=list(g.exterior.coords)
        for a,b in zip(pts,pts[1:]):
            key=tuple(sorted((tuple(a),tuple(b))))
            if key in edges:pairs.append((edges[key],i))
            else:edges[key]=i
    used=set();new=[]
    for i,j in pairs:
        if i in used or j in used:continue
        u=cells[i].union(cells[j])
        if u.geom_type=='Polygon' and abs(u.convex_hull.area-u.area)<1e-14:
            used.update((i,j));new.append(u)
    if not used:break
    cells=[g for i,g in enumerate(cells) if i not in used]+new
for i,t in enumerate(cells):
    add(pp,pc,'plate_'+str(i),m.prism(list(t.exterior.coords)[:-1],-.00635,0),'aluminum')
PLATE=m.fixed('carriage',TARGET,pp,pc,LOOK,friction=(.2,.15));b=PLATE.build
PLATE=replace(PLATE,build=lambda:b().replace('def Xform "carriage"','def Xform "carriage"'))
# fixed parts lack start_quat API: turn all local meshes to the physical upright
# pose before building, retaining the original drawing dimensions and thickness.
pp=[(n,mesh.turned(-90,'y'),look) for n,mesh,look in pp]
pc=[coll(n+'_c',mesh, n=='threaded_hole') for n,mesh,look in pp]
PLATE=m.fixed('carriage',TARGET,pp,pc,LOOK,friction=(.2,.15))
# Cleaned single fragment: upper broken face recessed0.5mm, full thread torear.
tz=np.linspace(-.00635,-.0005,142)
tap=capped([flute_radius(theta,thread_radius(theta,z)) for z in tz],tz,theta)
FRAGMENT=m.free('tap_fragment',TARGET,[('fragment',tap,'tap')],[coll('fragment_c',tap,True)],LOOK,mass=.00155,friction=(.2,.15),torsional_patch=0,solver_iters=64,start_quat=Q)
# Small ordinary machine vise: primary MS4 jaw50x10mm, open6.35mm.
# Fixed setup equivalent to a bench-clamped vise; outer casting height estimated.
vp,vc=[],[]
add(vp,vc,'base',m.box((.090,.055,.010),(.40,.075,TOP+.005)),'vise')
for x in (.395,.41135):add(vp,vc,'jaw'+str(x).replace('.','p'),m.box((.010,.050,.010),(x,.075,TOP+.025)),'steel')
add(vp,vc,'bed',m.box((.020,.050,.010),(.403175,.075,TOP+.015)),'vise')
VISE=m.fixed('machine_vise',(0,0,0),vp,vc,LOOK,friction=(.4,.3))

# Holder has the same three longitudinal support grooves; its OD fits M8root.
holder=capped([flute_radius(theta,np.full(192,.00325))]*2,[0,.055],theta)
fp,fc=[],[];add(fp,fc,'holder',holder,sdf=True)
add(fp,fc,'square',m.box((.0048,.0048,.010),(0,0,.060)))
# Original collar guides longitudinal finger adjustment;0 is six mm projection.
cp,cc=[],[]
# Annular collar represented by32 convex sectors, no filled bore.
def ring_parts(p,c,prefix,ri,ro,z0,z1):
    for k in range(32):
        a,b=2*math.pi*k/32,2*math.pi*(k+1)/32
        mesh=m.prism([(ri*math.cos(a),ri*math.sin(a)),(ro*math.cos(a),ro*math.sin(a)),(ro*math.cos(b),ro*math.sin(b)),(ri*math.cos(b),ri*math.sin(b))],z0,z1)
        add(p,c,prefix+str(k),mesh)
ring_parts(cp,cc,'collar',.00335,.005,.047,.053)
links=[m.hinge('collar_rotation','Z',(0,0,0),-float('inf'),float('inf'),[],[],mass=.0001,gravity=True,com=(0,0,.050),friction=0,damping=0,armature=0,inertia=(1e-9,1e-9,1e-9)),
       m.slider('collar','Z',-.004,.008,cp,cc,parent='collar_rotation',mass=.0029,gravity=True,friction=0,damping=.02,armature=0,inertia=(1e-8,1e-8,4.9e-8))]
# Finger outer section is the convex lens of root-circle and flute-circle,
# with75micrometre edge clearance; reconstructed, not OEM dimensioned CAD.
# EI/L derives from steelE200GPa in both orthogonal bending planes.
E=200e9;length=.014
section=Point(0,0).buffer(.003175,quad_segs=32).intersection(Point(.0032,0).buffer(.001425,quad_segs=32))
xy=np.asarray(section.exterior.coords)[:-1]
# Polygon area moment about radialX: integral(y^2 dA), SI units.
cross=xy[:,0]*np.roll(xy[:,1],-1)-np.roll(xy[:,0],-1)*xy[:,1]
Ixx=abs(float(np.sum((xy[:,1]**2+xy[:,1]*np.roll(xy[:,1],-1)+np.roll(xy[:,1],-1)**2)*cross)/12))
Iyy=abs(float(np.sum((xy[:,0]**2+xy[:,0]*np.roll(xy[:,0],-1)+np.roll(xy[:,0],-1)**2)*cross)/12))-section.area*section.centroid.x**2
EI=E*Ixx
finger_area=section.area
radial_center=section.centroid.x
for k in range(3):
    a=2*math.pi*k/3;parent='collar'
    for j in range(4):
        hi=.050-j*length;lo=hi-length;name=f'finger{k}_{j}'
        mesh=m.prism(list(section.exterior.coords)[:-1],lo,hi).turned(math.degrees(a),'z')
        at=(radial_center*math.cos(a),radial_center*math.sin(a),hi)
        # Two native revolute joints at the same station form an elastic
        # universal joint. Split physical segment mass/inertia between bodies;
        # no extra mass is added and only the distal body carries its collision.
        center=(radial_center*math.cos(a),radial_center*math.sin(a),(hi+lo)/2)
        for suffix,axis,stiffness,parts,cols in [('_t','X',E*Ixx/length,[],[]),('','Y',E*Iyy/length,[(name,mesh,'steel')],[coll(name+'_c',mesh)])]:
            nm=name+suffix
            links.append(m.hinge(nm,axis,at,-60,60,parts,cols,mass=7850*finger_area*length/2,spring=stiffness,damping=.00002,friction=0,effort=10,armature=0,gravity=True,com=center,parent=parent,inertia=(2.55e-9,2.55e-9,1e-10)))
            parent=nm
sp,sc=[],[];ring_parts(sp,sc,'sleeve',.00335,.005,.015,.030)
links.append(m.hinge('sleeve_rotation','Z',(0,0,0),-float('inf'),float('inf'),[],[],mass=.0001,gravity=True,com=(0,0,.0225),friction=0,damping=0,armature=0,inertia=(2e-9,2e-9,1.5e-9)))
links.append(m.slider('sleeve','Z',-.015,.020,sp,sc,parent='sleeve_rotation',mass=.0059,gravity=True,friction=0,damping=.01,armature=0,inertia=(1.98e-7,1.98e-7,9.85e-8)))
EXTRACTOR=m.mechanism('extractor',TOOL_START,links,LOOK,frame_parts=fp,frame_colliders=fc,friction=(.3,.2),solver_iters=64,free_mass=.014,start_quat=Q,frame_inertia=(5e-6,5e-6,8e-8))
_build=EXTRACTOR.build
def tool_build():
    s=_build().replace('enabledSelfCollisions = 0','enabledSelfCollisions = 1')
    for k in range(3):
        a=2*math.pi*k/3;q=f'({math.cos(a/2):.8f}, 0, 0, {math.sin(a/2):.8f})'
        for j in range(4):
            for suffix in ('_t',''):
                marker='def PhysicsRevoluteJoint "finger'+str(k)+'_'+str(j)+suffix+'_joint"';start=s.index(marker);end=s.index('\n    }',start)
                chunk=s[start:end].replace('(1, 0, 0, 0)',q);s=s[:start]+chunk+s[end:]
    return s
EXTRACTOR=replace(EXTRACTOR,build=tool_build)
OBJECTS=[VISE,PLATE,FRAGMENT,EXTRACTOR]
class Observe:
    def __init__(self):self.n=0
    def reset(self,env=None):self.n=0
    def step(self,env):
        self.n+=1
        if self.n==1 or self.n%10==0:
            s=m.state(env,'extractor');f=m.state(env,'tap_fragment')
            print('TAP_GATE',self.n,{'tool_pos':s.pos.tolist(),'tool_quat':s.quat_xyzw.tolist(),'joints':s.joints,'fragment_pos':f.pos.tolist(),'fragment_quat':f.quat_xyzw.tolist()},flush=True)
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    return {'right':[Waypoint(2,(.265,-.19,TOP+.10),-.04695,'approach'),Waypoint(2,(.265,-.19,TOP+.007),-.04695,'descend'),Waypoint(2,(.265,-.19,TOP+.007),0,'pinch original square'),Waypoint(3,(.265,-.19,TOP+.10),0,'lift'),Waypoint(3,(.265,-.19,TOP+.10),0,'hold')], 'left':[Waypoint(12,(.10,.35,1.05),-.04695,'park')]}
