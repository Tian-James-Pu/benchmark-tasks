"""Physical assembly gate: independent sleeve/holder/finger bundle, ordinary wrench.

No axial or rotary joint connects those three components. Contact with actual
bores/flutes provides guidance, so a component can slide off or be dropped.
"""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
import trimesh
from shapely.geometry import Polygon,Point
from shapely import constrained_delaunay_triangles
from shapely.geometry.polygon import orient

START=TOOL_START+np.array([0,0,.0053])
def partition(poly):
    cells=list(constrained_delaunay_triangles(poly).geoms)
    while True:
        edges={};pairs=[]
        for i,g in enumerate(cells):
            v=list(g.exterior.coords)
            for a,b in zip(v,v[1:]):
                key=tuple(sorted((tuple(a),tuple(b))))
                if key in edges:pairs.append((edges[key],i))
                else:edges[key]=i
        used=set();new=[]
        for i,j in pairs:
            if i in used or j in used:continue
            u=cells[i].union(cells[j])
            if u.geom_type=='Polygon' and abs(u.area-u.convex_hull.area)<1e-14:used.update((i,j));new.append(u)
        if not used:break
        cells=[g for i,g in enumerate(cells) if i not in used]+new
    return cells

def props(parts):
    meshes=[trimesh.Trimesh(mesh.v,mesh.f,process=False) for _,mesh,_ in parts]
    joined=trimesh.util.concatenate(meshes);joined.density=7850
    return float(joined.mass),np.asarray(joined.center_mass),np.asarray(joined.moment_inertia)

# Exact convex partition of the SAME cylindrical-flute holder section replaces
# long-shape SDF cooking whose voxel scale was comparable to finger clearance.
hp,hc=[],[]
hr=flute_radius(theta,np.full(192,.00325));poly=Polygon(np.c_[hr*np.cos(theta),hr*np.sin(theta)])
for i,g in enumerate(partition(poly)):add(hp,hc,'holder'+str(i),m.prism(list(orient(g,1).exterior.coords)[:-1],0,.055))
add(hp,hc,'square',m.box((.0048,.0048,.010),(0,0,.060)))
# Starrett91B, maker225mm length and4–7mm square capacity. Other outline scales
# from maker orthographic photo; body8mm thickness is disclosed estimate.
# Standard pre-tightened V-jaw clamp on the original square is a rigid assembled
# tool connection, present from reset, never attached to a robot or tap fragment.
outline=[(-.00535,-.1125),(.00535,-.1125),(.00535,-.030),(.0103,-.017),(.0103,.017),(.00535,.030),(.00535,.061),(-.00535,.061),(-.00535,.030),(-.0103,.017),(-.0103,-.017),(-.00535,-.030)]
wrench=Polygon(outline).difference(Polygon([(-.0024,-.0024),(.0024,-.0024),(.0024,.0024),(-.0024,.0024)]))
for i,g in enumerate(partition(wrench)):add(hp,hc,'wrench'+str(i),m.prism(list(orient(g,1).exterior.coords)[:-1],.056,.064))
add(hp,hc,'knurled_handle',m.cylinder(.0081,.0515,(0,.08675,.060),axis='y',segments=48))
hmass,hcom,hi=props(hp)
EXTRACTOR=m.free('extractor',START,hp,hc,LOOK,mass=hmass,friction=(.3,.2),torsional_patch=0,solver_iters=64,start_quat=Q,com=tuple(hcom))

# The collar is the sole root of its hooked flexible fingers. There is no
# prismatic guide or rotation constraint to the holder, and no invented stops.
beam_links=[]
for l in links:
    if not l.name.startswith('finger'):continue
    parts=[]
    for name,mesh,look in l.parts:
        test=trimesh.Trimesh(mesh.v,mesh.f,process=False)
        if test.volume<0:mesh=m.Mesh(mesh.v,mesh.f[:,::-1])
        parts.append((name,mesh,look))
    cols=[coll(name+'_c',mesh) for name,mesh,look in parts]
    beam_links.append(replace(l,parent=None if l.parent=='collar' else l.parent,parts=tuple(parts),colliders=tuple(cols)))
cmass,ccom,ci=props(cp)
BUNDLE=m.mechanism('finger_bundle',START,beam_links,LOOK,frame_parts=cp,frame_colliders=cc,free_mass=cmass,frame_inertia=tuple(np.diag(ci)),start_quat=Q,friction=(.3,.2),solver_iters=64)
bbuild=BUNDLE.build

def bundle_build():
    s=bbuild().replace('enabledSelfCollisions = 0','enabledSelfCollisions = 1')
    s=s.replace('float3 physics:diagonalInertia =',f'point3f physics:centerOfMass = ({ccom[0]}, {ccom[1]}, {ccom[2]})\n        float3 physics:diagonalInertia =',1)
    for k in range(3):
        a=2*math.pi*k/3;q=f'({math.cos(a/2):.8f}, 0, 0, {math.sin(a/2):.8f})'
        for j in range(4):
            for suffix in ('_t',''):
                marker=f'def PhysicsRevoluteJoint "finger{k}_{j}{suffix}_joint"';start=s.index(marker);end=s.index('\n    }',start)
                chunk=s[start:end].replace('(1, 0, 0, 0)',q);s=s[:start]+chunk+s[end:]
            if j:
                # Consecutive finite beam cells share a hinge interface; contact
                # there would double-count the same material after the2-DOFsplit.
                marker=f'def Xform "finger{k}_{j}"';start=s.index(marker);end=s.index('\n    }',start)
                chunk=s[start:end].replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysicsFilteredPairsAPI"')
                chunk+=f'\n        rel physics:filteredPairs = [</P_finger_bundle/finger{k}_{j-1}>]\n'
                s=s[:start]+chunk+s[end:]
    return s
BUNDLE=replace(BUNDLE,build=bundle_build)
smass,scom,si=props(sp)
SLEEVE=m.free('support_sleeve',START,sp,sc,LOOK,mass=smass,friction=(.3,.2),torsional_patch=0,solver_iters=64,start_quat=Q,com=tuple(scom))

# Single fragment retains the factory FormC3-thread chamfer beyond the farface.
# It is not a second flat break conveniently hiding the rear-access shortcut.
tz=np.linspace(-.01010,-.0005,232)
radii=[]
for z in tz:
    fade=np.clip((z+.01010)/.00375,0,1)
    # Linear lead chamfer envelope; exact OEM relief and coolant bore omitted.
    r=np.minimum(thread_radius(theta,z),MINOR+(MAJOR-MINOR)*fade)
    radii.append(flute_radius(theta,r))
tap_full=capped(radii,tz,theta)
tmesh=trimesh.Trimesh(tap_full.v,tap_full.f,process=False)
FRAGMENT=m.free('tap_fragment',TARGET,[('fragment',tap_full,'tap')],[coll('fragment_c',tap_full,True)],LOOK,mass=float(tmesh.volume)*8150,friction=(.2,.15),torsional_patch=0,solver_iters=64,start_quat=Q)
corrected=[]
for name,mesh,look in pp:
    test=trimesh.Trimesh(mesh.v,mesh.f,process=False)
    if test.volume<0:mesh=m.Mesh(mesh.v,mesh.f[:,::-1])
    corrected.append((name,mesh,look))
PLATE=m.fixed('carriage',TARGET,corrected,[coll(name+'_c',mesh,name=='threaded_hole') for name,mesh,look in corrected],LOOK,friction=(.2,.15))
OBJECTS=[VISE,PLATE,FRAGMENT,EXTRACTOR,BUNDLE,SLEEVE]

class AssemblyObserve:
    def reset(self,env=None):self.n=0
    def step(self,env):
        self.n+=1
        if self.n==1 or self.n%10==0:
            print('TAP_ASSEMBLY',self.n,{n:{'pos':m.state(env,n).pos.tolist(),'quat':m.state(env,n).quat_xyzw.tolist()} for n in ('extractor','finger_bundle','support_sleeve')},flush=True)
A=AssemblyObserve();m.on_reset(A.reset);m.on_step(A.step)
def demo():
    return {'right':[Waypoint(2,(.260,-.235,TOP+.10),-.04695,'approach',yaw=math.pi/2),Waypoint(2,(.260,-.235,TOP+.0123),-.04695,'descend',yaw=math.pi/2),Waypoint(2,(.260,-.235,TOP+.0123),0,'grasp wrench',yaw=math.pi/2),Waypoint(3,(.260,-.235,TOP+.10),0,'lift assembly',yaw=math.pi/2),Waypoint(3,(.260,-.235,TOP+.10),0,'hold',yaw=math.pi/2)]}
