"""Service-stage saw with buildable spring anchor reconstruction.

Blade and battery are removed before the episode. The small 2-mm anchor pin
with a 3.6-mm retaining head is an explicitly specified buildable equivalent
of the missing source spring anchorage; it is not a robot grip feature.
"""
import math
from dataclasses import replace
import numpy as np
import mechkit as m
from asset import build as source_build
from spring_mechanics import PIVOT,PLANE_X

C=np.array([.20,-.05,.7475+.142])
Q=(-.5,.5,.5,.5)
START_Q=math.radians(35.)
PIN_R=.024
WIRE_R=.000225
HOOK_R=.0018
LOOK={'wire':m.Look(color=m.rgb(.25,.27,.28),metallic=.8,roughness=.3)}


def service_build():
    from pxr import Usd,Sdf,UsdGeom,UsdPhysics,Gf
    layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(source_build())
    stage=Usd.Stage.Open(layer)
    for path in ['/Asset/Joints/blade_spin','/Asset/Joints/battery_mount',
                 '/Asset/Links/blade','/Asset/Links/battery']:
        stage.RemovePrim(path)
    j=stage.GetPrimAtPath('/Asset/Joints/guard_pivot')
    for name in ['physxJointAxis:angular:staticFrictionEffort','physxJointAxis:angular:dynamicFrictionEffort']:
        j.GetAttribute(name).Set(.002)
    # Retaining shaft on the existing inner guard face; no handles or tabs.
    def cylinder(path,r,h,xyz):
        p=UsdGeom.Cylinder.Define(stage,path);p.CreateRadiusAttr(r);p.CreateHeightAttr(h);p.CreateAxisAttr('X')
        p.AddTranslateOp().Set(Gf.Vec3d(*xyz));p.CreateDisplayColorAttr([(0.62,.64,.66)])
        api=UsdPhysics.CollisionAPI.Apply(p.GetPrim());api.CreateCollisionEnabledAttr(True)
        p.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
        p.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.00005)
        p.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
    cylinder('/Asset/Links/lower_guard/spring_pin',.001,.005,(-.0155,0.,-PIN_R))
    cylinder('/Asset/Links/lower_guard/spring_pin_head',.0018,.001,(-.0185,0.,-PIN_R))
    cylinder('/Asset/Links/housing/bearing_boss',.0195,.009,(-.0685,0.,.065))
    # Low bearing resistance is an estimate, disclosed separately from OEM data.
    return layer.ExportToString()


def objects(engaged=True):
    saw=m.Part('saw',pos=tuple(C),size=(.287,.350,.235),role='jointed',build=service_build,start_quat=Q,
               drives=(('guard_pivot',(0.,.0005,4.,.002,0.)),('trigger_travel',(200.,.8,40.,0.,0.))),
               start_joints=(('guard_pivot',START_Q),))
    # J hook measured from the ruler photograph: 3.6-mm bend diameter,
    # ~0.45-mm wire; open tail, not a closed virtual attachment.
    pts=[(-.006,-HOOK_R,0.),(0.,-HOOK_R,0.)]
    pts += [(HOOK_R*math.cos(a),HOOK_R*math.sin(a),0.) for a in np.linspace(-math.pi/2,math.pi/2,17)[1:]]
    pts += [(-.003,HOOK_R,0.)]
    parts=[];cols=[]
    for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
        wire=m.tube(np.array([a,b]),WIRE_R,sides=10)
        parts.append((f'wire{i}',wire,'wire'))
        cols.append(m.col_hull(f'wire{i}_c',wire).replace('contactOffset = 0.004','contactOffset = 0.00005'))
    R=m.quat_matrix(Q);pivot=C+R@PIVOT
    rot=np.array([[math.cos(START_Q),-math.sin(START_Q),0.],[math.sin(START_Q),math.cos(START_Q),0.],[0.,0.,1.]])
    at=pivot+rot@np.array([-.0005,-PIN_R,-(PLANE_X-PIVOT[0])])
    if not engaged:at+=rot@np.array([0.,-.006,0.])
    hook=m.free('spring_hook',at,parts,cols,LOOK,mass=.001,friction=(.25,.18),
                torsional_patch=None,solver_iters=64,yaw_deg=math.degrees(START_Q),com=(-.002,0.,0.))
    return [saw,hook]
