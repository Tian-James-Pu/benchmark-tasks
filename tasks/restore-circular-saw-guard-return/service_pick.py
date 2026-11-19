"""130-mm miniature right-angle pick, dimensioned rebuildable equivalent.

Envelope/material references: Bahco 2633/S4 (130 mm, four pieces/80g,
plastic handles and stainless blades). Handle and tip dimensions below are
explicit reconstruction estimates, not proprietary manufacturer CAD.
"""
import math
from dataclasses import replace
import numpy as np
import mechkit as m

LOOK={'handle':m.Look(color=m.rgb(.80,.28,.08),roughness=.7),
      'steel':m.Look(color=m.rgb(.65,.68,.70),metallic=.8,roughness=.25)}


def make_pick(pos=(.23,-.22,.7475+.0075)):
    # Origin is handle centre; +X is the long shaft direction.
    handle=m.lathe([(0.,-.030),(.006,-.030),(.0075,-.025),(.0075,.020),(.005,.030),(0.,.030)],segments=12).aimed('+X')
    shaft=m.tube(np.array([[.030,0,0],[.098,0,0],[.100,0,-.002],[.100,0,-.006]]),.00075,sides=12)
    part=m.free('service_pick',pos,[('handle',handle,'handle'),('shaft',shaft,'steel')],
                [m.col_hull('handle_c',handle)],LOOK,mass=.020,friction=(.8,.65),
                torsional_patch=.003,solver_iters=64,yaw_deg=90.,com=(.008,0.,-.0001))
    build_original=part.build
    def build():
        from pxr import Usd,Sdf,UsdGeom,UsdPhysics,Gf
        layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(build_original())
        stage=Usd.Stage.Open(layer);root=stage.GetDefaultPrim().GetPath()
        pts=[(.030,0,0),(.098,0,0),(.100,0,-.002),(.100,0,-.006)]
        for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
            a=np.array(a);b=np.array(b);d=b-a
            cap=UsdGeom.Capsule.Define(stage,str(root)+f'/shaft_c{i}')
            cap.CreateRadiusAttr(.00075);cap.CreateHeightAttr(float(np.linalg.norm(d)));cap.CreateAxisAttr('Z')
            cap.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
            cap.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*(d/np.linalg.norm(d)))).GetQuat()))
            cap.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            cap.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
            cap.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0001)
            cap.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
        return layer.ExportToString()
    return replace(part,build=build)
