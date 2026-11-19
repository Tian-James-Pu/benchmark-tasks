"""Full-scale caster contact development gate. No task validation or scored repair.

Dimensions follow the i7 manual/ruler reconstruction. Elastic sectors approximate
the original compliant retainers; the hair mat is an uncalibrated aggregate FEM
model whose physical behavior is the subject of this gate, not a validated fault.
"""
import math, pathlib, sys
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
SCENE='station'; TOP=TABLE_TOP[SCENE]; SUCCESS_HOLD_S=0.
INSTRUCTION='Restore free movement of the robot vacuum caster.'
C=np.array([.34,0,TOP]); H=np.array([.22,0,TOP+.066]); W=H+[-.010,0,.015]
LOOK={n:m.Look(color=m.rgb(*v),metallic=metal,roughness=.55) for n,v,metal in [('case',(.12,.13,.14),0),('tire',(.035,.035,.04),0),('hub',(.38,.40,.42),.1),('steel',(.6,.63,.66),.75),('fiber',(.24,.14,.07),0)]}
def col(n,g,sdf=False):
    s=m.col_hull(n,g).replace('contactOffset = 0.004','contactOffset = 0.00005')
    if sdf:s=s.replace('"PhysicsMeshCollisionAPI"','"PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"').replace('physics:approximation = "convexHull"','physics:approximation = "sdf"\n int physxSDFMeshCollision:sdfResolution = 512')
    return s
def add(p,c,n,g,look='case',sdf=False):p.append((n,g,look));c.append(col(n+'_collision',g,sdf))
def ring(ri,ro,z0,z1,n=64):
    # A closed annular manifold. Never fill the actual axle/stem bore with a hull.
    th=np.linspace(0,2*math.pi,n,endpoint=False)
    v=np.vstack([np.c_[r*np.cos(th),r*np.sin(th),np.full(n,z)] for r,z in [(ri,z0),(ro,z0),(ri,z1),(ro,z1)]])
    f=[]
    for i in range(n):
        j=(i+1)%n
        for a,b,c,d in [(i,j,n+j,n+i),(2*n+i,3*n+i,3*n+j,2*n+j),(i,2*n+i,2*n+j,j),(n+i,n+j,3*n+j,3*n+i)]:f.extend([(a,b,c),(a,c,d)])
    return m.Mesh(v,np.array(f))
def safe_part(p):
    build=p.build
    return replace(p,build=lambda:build().replace('contactOffset = 0.004','contactOffset = 0.00005'))

# Loose complete unit envelope; not a fixed jig. A real front stem bore is cut.
bp,bc=[],[]
add(bp,bc,'upper_case',m.cylinder(.170,.047,(0,0,.0235)))
# Top underside shell split into exact circular sectors around the caster bore.
from shapely.geometry import Point
from shapely import constrained_delaunay_triangles
from shapely.geometry import box as planar_box
from shapely.geometry.polygon import orient
def extrude_xz(poly,y0,y1):
    poly=orient(poly,sign=1);xy=list(poly.exterior.coords)[:-1];lookup={tuple(p):i for i,p in enumerate(xy)};n=len(xy)
    v=np.array([(x,y,z) for y in (y0,y1) for x,z in xy]);f=[]
    for g in constrained_delaunay_triangles(poly).geoms:
        ids=[lookup[tuple(p)] for p in list(g.exterior.coords)[:-1]]
        f.extend([tuple(ids),tuple(n+i for i in ids[::-1])])
    for i in range(n):
        j=(i+1)%n;f.extend([(i,n+i,n+j),(i,n+j,j)])
    import trimesh
    tm=trimesh.Trimesh(v,f,process=True);tm.fix_normals();assert tm.is_watertight
    return m.Mesh(np.asarray(tm.vertices),np.asarray(tm.faces))
poly=Point(0,0).buffer(.170,quad_segs=32).difference(Point(-.120,0).buffer(.0026,quad_segs=16))
for i,g in enumerate(constrained_delaunay_triangles(poly).geoms):
    add(bp,bc,'underside_'+str(i),m.prism(list(g.exterior.coords)[:-1],.047,.066))
# Cosmetic drive-wheel and brush envelopes remain part of the loose whole robot.
for y in [-.125,.125]:add(bp,bc,'drive_'+str(y).replace('-','n').replace('.','p'),m.cylinder(.035,.018,(.010,y,.059),axis='y'),'tire')
for x in [.025,.060]:add(bp,bc,'brush_'+str(x).replace('.','p'),m.cylinder(.016,.170,(x,0,.068),axis='y'),'tire')
E_ABS=2.0e9; beam_w=.004; beam_t=.0006; beam_L=.004
K=3*E_ABS*(beam_w*beam_t**3/12)/beam_L**3
body_links=[]
for k in range(3):
    a=2*math.pi*k/3; pp,cc=[],[]
    # Radially compliant sector of the original stem-retaining socket.
    g=m.prism([(.0023,-.001),(.0038,-.001),(.0038,.001),(.0023,.001)],.0528,.0552).turned(math.degrees(a)).moved((-.120,0,0))
    add(pp,cc,'socket_lip',g)
    # A Cartesian radial axis is authored by rotating the joint frame below.
    body_links.append(m.slider('socket'+str(k),'X',0,.001,pp,cc,mass=.00002,spring=K,damping=.01,friction=0,effort=50,armature=0,gravity=True))
BODY=m.mechanism('robot',C,body_links,LOOK,frame_parts=bp,frame_colliders=bc,free_mass=3.37454,friction=(.5,.4),solver_iters=64)
bb=BODY.build
def body_build():
    s=bb()
    for k in range(3):
        a=2*math.pi*k/3;q=f'({math.cos(a/2)}, 0, 0, {math.sin(a/2)})';marker='def PhysicsPrismaticJoint "socket'+str(k)+'_joint"';start=s.index(marker);end=s.index('\n    }',start)
        s=s[:start]+s[start:end].replace('(1, 0, 0, 0)',q)+s[end:]
    return s.replace('contactOffset = 0.004','contactOffset = 0.00005')
BODY=replace(BODY,build=body_build)

yp,yc=[],[]
flange_outline=Point(0,0).buffer(.025,quad_segs=24).difference(__import__('shapely').geometry.box(-.026,-.0095,.0032,.0095))
for i,g in enumerate(constrained_delaunay_triangles(flange_outline).geoms):
    add(yp,yc,'flange'+str(i),m.prism(list(g.exterior.coords)[:-1],0,.004))
add(yp,yc,'dome_back',m.box((.017,.038,.015),(.0135,0,.0115)))
for sign in [-1,1]:
    # Cheeks stop below axle axis; opposed elastic lips form its open snap seats.
    add(yp,yc,'cheek'+str(sign).replace('-','n'),m.box((.029,.0085,.009),(-.0095,sign*.01475,.0085)))
stem=m.lathe([(0,-.018),(.0025,-.018),(.0025,-.014),(.0021,-.0128),(.0021,-.0108),(.0025,-.0096),(.0025,0),(0,0)],48)
add(yp,yc,'original_grooved_stem',stem,'steel',True)
links=[]
for side in [-1,1]:
    for sgn in [-1,1]:
        pp,cc=[],[];yy=side*.012
        # Actual open C-seat:1.55mm circular seat and2.7mm top throat.
        # Its two halves elastically spread during extraction, rather than
        # overlapping the axle with rectangular blocks at reset.
        seat=planar_box(-.014,.010,-.006,.020).difference(Point(-.010,.015).buffer(.00155,quad_segs=24)).difference(planar_box(-.01135,.015,-.00865,.021))
        seat=seat.intersection(planar_box(-.014 if sgn<0 else -.010,.009,-.010 if sgn<0 else -.006,.021))
        g=extrude_xz(seat,yy-.002,yy+.002)
        add(pp,cc,'axle_lip',g,sdf=True)
        links.append(m.slider('lip'+str(side).replace('-','n')+str(sgn).replace('-','n'),'X',-.001,.001,pp,cc,mass=.00002,spring=K,damping=.01,friction=0,effort=50,armature=0,gravity=True))
YOKE=safe_part(m.mechanism('caster',H,links,LOOK,frame_parts=yp,frame_colliders=yc,free_mass=.01549,friction=(.35,.25),solver_iters=64))
ap,ac=[],[];add(ap,ac,'steel_axle',m.cylinder(.0015,.024,axis='y'),'steel')
AXLE=safe_part(m.free('axle',W,ap,ac,LOOK,mass=.00133,friction=(.3,.2),torsional_patch=None,solver_iters=64))
# Rounded tread is a surface of revolution with the actual through-bore.
profile=[(.00155,-.009),(.009,-.009),(.012,-.007),(.013,0),(.012,.007),(.009,.009),(.00155,.009),(.00155,-.009)]
tire=m.lathe(profile,96).turned(90,'x')
import trimesh
_tm=trimesh.Trimesh(tire.v,tire.f,process=False);_tm.merge_vertices()
assert _tm.is_watertight
tire=m.Mesh(np.asarray(_tm.vertices),np.asarray(_tm.faces))
wp,wc=[],[];add(wp,wc,'rounded_wheel',tire,'tire',True)
WHEEL=safe_part(m.free('wheel',W,wp,wc,LOOK,mass=.0085,friction=(.6,.45),torsional_patch=None,solver_iters=64))

# No invisible attachment: deformable annular aggregate is squeezed between the
# wheel side and yoke. Individual fibre entanglement/fracture is not represented.
MAT=m.fixed('fibres',W+(0,.00975,0),[],[],LOOK)
def mat_tetrahedra(nu=32,nv=8):
    # Exact torus-prism tessellation, avoiding auto-voxel degeneracy at mm scale.
    v=[];t=[]
    for i in range(nu):
        a=2*math.pi*i/nu
        for j in range(nv+1):
            b=2*math.pi*(j-1)/nv;r=.003 if j==0 else .003+.001*math.cos(b);y=0 if j==0 else .001*math.sin(b)
            v.append((r*math.cos(a),y,r*math.sin(a)))
    v=np.array(v)
    for i in range(nu):
        for j in range(nv):
            ids=sorted([0,j+1,(j+1)%nv+1]);a,b,c=[i*(nv+1)+k for k in ids];d,e,f=[((i+1)%nu)*(nv+1)+k for k in ids]
            for tet in [(a,b,c,f),(a,b,e,f),(a,d,e,f)]:
                tet=list(tet);vv=v[tet];det=np.linalg.det((vv[1:]-vv[0]).T)
                if det<0:tet[1],tet[2]=tet[2],tet[1]
                assert abs(det)>1e-13
                t.append(tet)
    return v,np.array(t)
def mat_build():
    from pxr import Usd,UsdGeom,UsdPhysics,Gf,PhysxSchema
    from omni.physx.scripts import deformableUtils,physicsUtils
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    root=UsdGeom.Xform.Define(stage,'/mat');stage.SetDefaultPrim(root.GetPrim());mesh=UsdGeom.TetMesh.Define(stage,'/mat/mesh')
    v,t=mat_tetrahedra();mesh.CreatePointsAttr(v.tolist());mesh.CreateTetVertexIndicesAttr(t.tolist());mesh.CreateDisplayColorAttr([(.24,.14,.07)])
    ok=deformableUtils.set_physics_volume_deformable_body(stage,'/mat/mesh')
    if not ok:raise RuntimeError('Native deformable hierarchy creation failed')
    mesh.GetPrim().ApplyAPI('PhysxBaseDeformableBodyAPI');mesh.GetPrim().GetAttribute('physxDeformableBody:selfCollision').Set(True)
    mesh.GetPrim().GetAttribute('physxDeformableBody:solverPositionIterationCount').Set(32)
    ca=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());ca.CreateContactOffsetAttr(.00005);ca.CreateRestOffsetAttr(0.)
    deformableUtils.add_deformable_material(stage,'/mat/material',density=200.,static_friction=.4,dynamic_friction=.3,youngs_modulus=50000.,poissons_ratio=.2)
    physicsUtils.add_physics_material_to_prim(stage,mesh.GetPrim(),'/mat/material')
    return stage.GetRootLayer().ExportToString()
MAT=replace(MAT,build=mat_build)
OBJECTS=[BODY,YOKE,AXLE,WHEEL,MAT]
class Observe:
    def __init__(self):self.n=0
    def reset(self,env=None):self.n=0
    def step(self,env):
        self.n+=1
        if self.n==1 or self.n%10==0:
            row={}
            for name in ['robot','caster','axle','wheel']:
                s=m.state(env,name);row[name]={'p':s.pos.tolist(),'q':s.quat_xyzw.tolist(),'v':s.lin_vel.tolist(),'w':s.ang_vel.tolist(),'joints':s.joints}
            print('CASTER_GATE',self.n,row,flush=True)
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    p=W+[0,0,.022]
    return {'left':[Waypoint(2.,tuple(p+[0,0,.080]),-.025,'approach'),Waypoint(2.,tuple(p),-.025,'original wheel'),Waypoint(1.,tuple(p),0.,'grip'),Waypoint(2.,tuple(p+[0,0,.035]),0.,'lift original wheel'),Waypoint(1.,tuple(p+[0,0,.035]),0.,'observe')]}
