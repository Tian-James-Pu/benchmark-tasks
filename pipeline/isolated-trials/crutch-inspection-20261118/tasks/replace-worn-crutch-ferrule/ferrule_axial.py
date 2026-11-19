"""Dimensioned rubber-foot equivalent with a contact-driven compliant bore.

38 mm maximum diameter and 45 mm height retain the source tip's envelope.
Twelve continuous full-depth radial sectors approximate an axially coupled
rubber bore. Each sector has one uniform radial displacement; local progressive
strain and axial rubber deformation are omitted. Whole-depth mass, stiffness,
damping, travel and friction match the prior segmented approximation. The frame
and elements never attach to the crutch; contact friction carries all loads.
"""
from dataclasses import replace
import math
import numpy as np
import mechkit as m

LOOK={'rubber':m.Look(color=m.rgb(.16,.17,.18),roughness=.9),
      'worn':m.Look(color=m.rgb(.31,.32,.33),roughness=.95)}

def wedge(r0,r1,z0,z1,angle,width=30.):
    a=np.radians(np.array([angle-width/2,angle,angle+width/2]))
    outline=np.vstack([r1*np.c_[np.cos(a),np.sin(a)],(r0*np.c_[np.cos(a),np.sin(a)])[::-1]])
    return m.prism(outline,z0,z1)

def make(name,pos,quat,*,worn=False,bore_radius=None):
    inner=(.01098 if worn else .0108) if bore_radius is None else bore_radius
    z0=.012 if worn else 0.
    parts=[];cols=[];links=[]
    look='worn' if worn else 'rubber'
    # Rigid outer shell is separated from the compliant bore by 0.6 mm.
    # Segmenting collision shells keeps the real central opening unfilled.
    for i in range(24):
        a=i*15.
        w=wedge(.013,.019,z0,.022,a,15.)
        u=wedge(.013,.015,.022,.045,a,15.)
        parts.extend([(f'base{i}',w,look),(f'upper{i}',u,look)])
        cols.extend([m.col_hull(f'base{i}_c',w),m.col_hull(f'upper{i}_c',u)])
    if not worn:
        stop=m.cylinder(.013,.008,(0,0,.004),segments=32)
        parts.append(('sole',stop,look));cols.append(m.col_cylinder('sole_c',.013,.008,(0,0,.004)))
    low=max(.008,z0)
    boundaries=np.array([low,.045])  # one continuous wall; no independent internal ledges
    sections=[]
    for layer,(zlo,zhi) in enumerate(zip(boundaries[:-1],boundaries[1:])):
        share=(zhi-zlo)/(.045-low)
        for angular in range(12):
            i=layer*12+angular;a=angular*30.;q=math.radians(a)
            if zhi>.040:
                profile=[(inner,zlo),(inner+.0016,zlo),(inner+.0016,zhi),(inner+.0009,zhi),(inner,.040)]
            else:
                profile=[(inner,zlo),(inner+.0016,zlo),(inner+.0016,zhi),(inner,zhi)]
            raw=m.prism(profile,-.00275,.00275)
            liner=m.Mesh(raw.v[:,[0,2,1]],raw.f[:,::-1]).turned(a)
            body_mass=.0015*share
            inertia=tuple(body_mass*(u*u+v*v)/12 for u,v in [(.0055,zhi-zlo),(.0016,zhi-zlo),(.0016,.0055)])
            links.append(m.slider(f'liner{i}','X',-.0005,.0015,[('rubber',liner,look)],[],
                                  mass=body_mass,spring=20000.*share,damping=3.*share,friction=0.,armature=0.,effort=80.*share,gravity=True,
                                  com=((inner+.0008)*math.cos(q),(inner+.0008)*math.sin(q),(zlo+zhi)/2),inertia=inertia))
            sections.append((i,angular,zlo,zhi))
    p=m.mechanism(name,pos,links,LOOK,frame_parts=parts,frame_colliders=cols,free_mass=.032 if not worn else .024,
                  start_quat=quat,friction=(.8,.65),solver_iters=64,frame_inertia=(1.2e-5,1.2e-5,6e-6))
    orig=p.build
    def build():
        from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
        l=Sdf.Layer.CreateAnonymous('.usda');l.ImportFromString(orig());s=Usd.Stage.Open(l);root=s.GetDefaultPrim()
        # Explicit free frame as root ensures reset/pose state is unambiguous.
        root.RemoveAPI(UsdPhysics.ArticulationRootAPI);root.RemoveAppliedSchema('PhysxArticulationAPI')
        frame=s.GetPrimAtPath(root.GetPath().AppendChild('frame'));UsdPhysics.ArticulationRootAPI.Apply(frame)
        frame.AddAppliedSchema('PhysxArticulationAPI')
        frame.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(False)
        frame.CreateAttribute('physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(64)
        frame.CreateAttribute('physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int).Set(8)
        for i,angular,zlo,zhi in sections:
            j=s.GetPrimAtPath(root.GetPath().AppendChild(f'liner{i}_joint'))
            q=Gf.Quatf(math.cos(math.radians(angular*30.)/2),Gf.Vec3f(0,0,math.sin(math.radians(angular*30.)/2)))
            j.GetAttribute('physics:localRot0').Set(q);j.GetAttribute('physics:localRot1').Set(q)
            body=root.GetPath().AppendChild(f'liner{i}')
            s.RemovePrim(body.AppendChild('rubber_c'))
            # One convex subset replaces the overlapping analytic wall/bevel boxes.
            # Inner wall and entry planes are unchanged; the hidden outer bevel
            # protrusion is clipped at the existing outerwall radius. Healthyfoot
            # collider volume decreases12.0676mm^3 per sector; mass/inertia and all
            # spring/material/offset settings are preserved explicitly above.
            tilt=math.atan2(.0009,.005)
            top_outer=.045-(.0016-.0009)*math.tan(tilt)
            profile=[(inner,zlo),(inner+.0016,zlo),(inner+.0016,top_outer),(inner+.0009,.045),(inner,.040)]
            raw=m.prism(profile,-.00275,.00275)
            mesh=m.Mesh(raw.v[:,[0,2,1]],raw.f[:,::-1]).turned(angular*30.)
            collider=UsdGeom.Mesh.Define(s,body.AppendChild('continuous_bore'))
            collider.CreatePointsAttr([Gf.Vec3f(*v) for v in mesh.v])
            collider.CreateFaceVertexCountsAttr([3]*len(mesh.f))
            collider.CreateFaceVertexIndicesAttr(mesh.f.reshape(-1).tolist())
            collider.CreateSubdivisionSchemeAttr('none');collider.CreateVisibilityAttr('invisible')
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(collider.GetPrim()).CreateApproximationAttr('convexHull')
            collider.GetPrim().AddAppliedSchema('MaterialBindingAPI')
            collider.GetPrim().CreateRelationship('material:binding:physics').SetTargets([root.GetPath().AppendChild('PhysicsMaterial')])
        for x in s.Traverse():
            if x.HasAPI(UsdPhysics.CollisionAPI):
                x.AddAppliedSchema('PhysxCollisionAPI')
                x.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.00005)
                x.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
        return l.ExportToString()
    from scaled_convex import scaled_convex_part
    return scaled_convex_part(replace(p,build=build))
