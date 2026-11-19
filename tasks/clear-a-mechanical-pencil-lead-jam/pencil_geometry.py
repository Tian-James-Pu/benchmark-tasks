"""PG1019-scale contact reconstruction, not OEM CAD.

Coordinates: +Y from writing end to rear button; metres, kilograms.
The 0.9 mm lead family and 4 mm sleeve are manufacturer specifications.
All other internal sizes and elastic constants are disclosed estimates.
The three-jaw/ring/retainer topology follows JWIMA's mechanism description.
"""
import math
import re
from dataclasses import replace
import numpy as np
import mechkit as m

LOOK = {n:m.Look(color=m.rgb(*c),metallic=metal,roughness=.4) for n,c,metal in [
    ('steel',(.65,.68,.71),.8),('brass',(.63,.47,.20),.7),
    ('body',(.20,.23,.26),.5),('graphite',(.08,.08,.085),.1),
    ('rubber',(.10,.10,.11),0),('blue',(.025,.08,.42),.1)]}
CONTACT_OFFSET = .00002
LEAD_R = .00045
GUIDE_R = .00050
GEOMETRY = {}

def collar(r0,r1,y0,y1,n=96):
    return m.lathe([(r0,y0),(r1,y0),(r1,y1),(r0,y1),(r0,y0)],n).aimed('+Y')

def sector(inner, outer, y0, y1, angle=100., n=12):
    """Closed annular sector centered on local +X, along Y."""
    a=np.linspace(-math.radians(angle)/2,math.radians(angle)/2,n)
    xy=np.vstack((np.c_[outer*np.cos(a),outer*np.sin(a)],
                  np.c_[inner*np.cos(a[::-1]),inner*np.sin(a[::-1])]))
    # This outline is concave: triangulate only end caps with ear clipping.
    from shapely.geometry import Polygon
    from shapely import constrained_delaunay_triangles
    poly=Polygon(xy); verts=[]; faces=[]; index={}
    def vi(x,z,y):
        key=(float(x),float(y),float(z))
        if key not in index:index[key]=len(verts);verts.append(key)
        return index[key]
    for tri in constrained_delaunay_triangles(poly).geoms:
        pts=list(tri.exterior.coords)[:3]
        # Ensure geometric normals, independent of Shapely ring orientation.
        for y,sign in ((y0,-1),(y1,1)):
            ids=[vi(x,z,y) for x,z in pts]
            vv=np.array([verts[i] for i in ids]);normal=np.cross(vv[1]-vv[0],vv[2]-vv[0])[1]
            faces.append(ids if normal*sign>0 else ids[::-1])
    # Polygon CCW in x,z has an inverted handedness relative to x,y,z.
    coords=list(poly.exterior.coords)
    for (x,z),(xx,zz) in zip(coords,coords[1:]):
        a,b,c,d=vi(x,z,y0),vi(xx,zz,y0),vi(xx,zz,y1),vi(x,z,y1)
        faces.extend(((a,c,b),(a,d,c)))
    return m.Mesh(np.asarray(verts),np.asarray(faces))

def collider(name,mesh,sdf=False):
    s=m.col_hull(name,mesh).replace('contactOffset = 0.004',f'contactOffset = {CONTACT_OFFSET}')
    if sdf:
        s=s.replace('"PhysicsMeshCollisionAPI"','"PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"')
        s=s.replace('physics:approximation = "convexHull"',
                    'physics:approximation = "sdf"\n int physxSDFMeshCollision:sdfResolution = 512')
    return s

def add(parts,cols,name,mesh,look='steel',sdf=False):
    # Closed lathe profiles share their first and last rings geometrically.
    # Weld identical vertices so concave SDF cooking gets a closed manifold.
    vertices,inverse=np.unique(np.round(mesh.v,12),axis=0,return_inverse=True)
    mesh=m.Mesh(vertices,inverse[mesh.f])
    GEOMETRY[name]=mesh
    parts.append((name,mesh,look));cols.append(collider(name+'_collision',mesh,sdf))

def physical(part,radial_joints=()):
    """Enable genuine internal contacts and orient the three radial sliders.

    The original helper's five-decimal mass serialization rounds sub-5 mg
    bodies to zero. Their authored mass/inertia are preserved below, not enlarged.
    """
    build=part.build
    def wrapped():
        text=build().replace('enabledSelfCollisions = 0','enabledSelfCollisions = 1')
        text=text.replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysxContactReportAPI"')
        text=text.replace('float physics:mass =','float physxContactReport:threshold = 0\n        float physics:mass =')
        for name,angle in radial_joints:
            q=(math.cos(math.radians(angle)/2),0,math.sin(math.radians(angle)/2),0)
            value='('+', '.join(f'{x:.12g}' for x in q)+')'
            pattern=r'(def PhysicsPrismaticJoint "'+name+r'_joint".*?\n    })'
            text=re.sub(pattern,lambda mt:mt.group(1).replace('(1, 0, 0, 0)',value),text,flags=re.S)
        return text
    return replace(part,build=wrapped)

def feed_core(pos, fixed=False, yaw=-45.):
    """Physical feed core at extended-writing position.

    Retraction/clip and removable threaded nose are deliberately absent from
    this bounded component gate, so it cannot be a completed task or scored.
    Frame only supports the actual ring stop/lead retainer; fixed=True is a
    laboratory diagnostic fixture, never the proposed final robot setup.
    """
    fp,fc=[],[]
    add(fp,fc,'guide_sleeve',collar(GUIDE_R,.00065,0,.004),'steel',True)
    add(fp,fc,'nib_cone',m.lathe([(GUIDE_R,.004),(.0008,.004),(.0025,.010),
        (.0025,.011),(.0010,.011),(.0010,.009),(GUIDE_R,.004)],96).aimed('+Y'),'steel',True)
    add(fp,fc,'carrier_stop',collar(.00155,.0021,.0209,.0214),'brass',True)
    add(fp,fc,'carrier_rear_stop',collar(.00155,.0021,.0235,.0240),'brass',True)
    # Hollow full-scale barrel and grip: no invisible central solid.
    add(fp,fc,'grip',collar(.0026,.00475,.015,.051),'steel',True)
    add(fp,fc,'barrel',collar(.0026,.0037,.051,.140),'body',True)
    links=[];radial=[]
    pp,pc=[],[]
    add(pp,pc,'lead_tank',collar(.0010,.0013,.026,.140),'body',True)
    add(pp,pc,'rear_button',m.cylinder(.0033,.010,(0,.145,0),axis='y'),'steel')
    # Native axial spring; 0.5 N per mm is an engineering estimate, not OEM test.
    links.append(m.slider('feed','Y',-.0035,0,pp,pc,mass=.003,spring=500.,damping=.1,
        friction=0,armature=0,gravity=True,com=(0,.09,0),inertia=(4e-6,4e-9,4e-6)))
    rp,rc=[],[]
    add(rp,rc,'chuck_ring',collar(.00155,.0019,.022,.0235),'brass',True)
    links.append(m.slider('ring','Y',-.0006,0,rp,rc,mass=.00012,spring=0,damping=.0001,
        friction=0,armature=0,gravity=True,com=(0,.02275,0),inertia=(2.0e-10,3.6e-10,2.0e-10)))
    for i,ang in enumerate((0,120,240)):
        jp,jc=[],[]
        # Actual independent 3-way jaw contact, not a forced lead/feed coupling.
        # Short bands approximate the tapered annular chuck profile.
        profile=[(.016,.00074),(.0208,.00114),(.0218,.00156),(.0232,.00156),(.024,.00114)]
        for k,((y0,r0),(y1,r1)) in enumerate(zip(profile,profile[1:])):
            g0=sector(.00045,r0,y0,y1)
            # Linear radial taper only changes the outer vertices.
            rr=np.hypot(g0.v[:,0],g0.v[:,2]); outer=rr>.00046
            fac=((r0+(r1-r0)*(g0.v[:,1]-y0)/(y1-y0))/np.maximum(rr,1e-12))
            g0.v[outer,0]*=fac[outer];g0.v[outer,2]*=fac[outer]
            # Construct the closed circular sector, then open it radially.
            # Increasing the radius of an open concentric sector instead would
            # leave irreducible corner interference when it closes.
            g=g0.moved((.00011,0,0)).turned(ang,'y');add(jp,jc,f'jaw{i}_band{k}',g,'brass',True)
        links.append(m.slider(f'jaw{i}','X',-.00013,0,jp,jc,parent='feed',mass=.000035,
            spring=150.,damping=.001,friction=0,armature=0,gravity=True,
            com=tuple(np.array([.001,.020,0])@np.array([[math.cos(math.radians(ang)),0,-math.sin(math.radians(ang))],[0,1,0],[math.sin(math.radians(ang)),0,math.cos(math.radians(ang))]])),
            inertia=(2e-10,3e-11,2e-10)))
        radial.append((f'jaw{i}',ang))
    # Rubber retainer: three native radial compliant sectors, not a sticky lead flag.
    # 1 kN/m per segment gives 10 mN at 10 um compression; this is uncalibrated.
    for i,ang in enumerate((0,120,240)):
        pp,pc=[],[]
        add(pp,pc,f'retainer{i}',sector(.00044,.0008,.0091,.0096).turned(ang,'y'),'rubber',True)
        links.append(m.slider(f'retainer{i}','X',0,.00015,pp,pc,mass=2e-7,spring=1000.,
            damping=.0001,friction=0,armature=0,gravity=True,inertia=(5e-14,5e-14,5e-14)))
        radial.append((f'retainer{i}',ang))
    starts=[('feed',-.0014)]+[('jaw'+str(i),-.00011) for i in range(3)]+[('retainer'+str(i),.00002) for i in range(3)]
    part=m.mechanism('pencil',pos,links,LOOK,fp,fc,yaw_deg=yaw,free_mass=None if fixed else .014,
        frame_inertia=(1.8e-5,1.5e-7,1.8e-5),friction=(.4,.3),solver_iters=96,start=starts)
    return physical(part,radial)

def lead(pos,yaw=-45.):
    p,c=[],[];g=m.cylinder(LEAD_R,.060,(0,.0335,0),axis='y',segments=48)
    add(p,c,'lead',g,'graphite')
    mass=1800*math.pi*LEAD_R**2*.060
    return physical(m.free('lead',pos,p,c,LOOK,mass=mass,friction=(.4,.3),
        torsional_patch=0,solver_iters=96,yaw_deg=yaw,com=(0,.0335,0)))
