"""Visible coils for the reduced spring model. Rendering never changes physics."""
import math
import numpy as np
import mechkit as m
from spring_mechanics_v6 import GUIDE_RADIUS,FIXED_ANGLE,REST_LENGTH,PIVOT,PLANE_X


def points(saw_pos,saw_rotation,hook_pos,hook_rotation,span=None):
    attach=hook_pos+hook_rotation@np.array([-.008,-.0018,0.])
    a=saw_rotation.T@(attach-saw_pos)-PIVOT
    end=np.array([-a[1],a[2],-(a[0]-(PLANE_X-PIVOT[0]))])
    r=max(np.linalg.norm(end[:2]),GUIDE_RADIUS+1e-7)
    if span is None:
        phi=math.atan2(end[1],end[0]);phi+=2*math.pi if phi<FIXED_ANGLE else 0.
        span=phi-math.acos(GUIDE_RADIUS/r)-FIXED_ANGLE
    if span>0:
        angles=np.linspace(FIXED_ANGLE,FIXED_ANGLE+span,600)
        arc=np.column_stack([GUIDE_RADIUS*np.cos(angles),GUIDE_RADIUS*np.sin(angles),np.zeros(len(angles))])
        centers=np.vstack([arc,np.linspace(arc[-1],end,201)[1:]])
    else:
        anchor=GUIDE_RADIUS*np.array([math.cos(FIXED_ANGLE),math.sin(FIXED_ANGLE),0.])
        centers=np.linspace(anchor,end,800)
    # Relaxed coils can bow outside the boss instead of being axially compressed.
    length=np.linalg.norm(np.diff(centers,axis=0),axis=1).sum()
    if length<REST_LENGTH:
        radial=centers.copy();radial[:,2]=0.;radial/=np.maximum(np.linalg.norm(radial,axis=1)[:,None],1e-9)
        shape=np.sin(np.linspace(0,math.pi,len(centers)))[:,None]*radial
        lo,hi=0.,.020
        for _ in range(12):
            mid=(lo+hi)/2;trial=centers+mid*shape
            if np.linalg.norm(np.diff(trial,axis=0),axis=1).sum()<REST_LENGTH:lo=mid
            else:hi=mid
        centers+=hi*shape
    tangent=np.gradient(centers,axis=0);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-9)
    sideways=np.column_stack([-tangent[:,1],tangent[:,0],np.zeros(len(tangent))])
    phase=np.linspace(0,2*math.pi*125,len(centers))
    coil=centers+.0018*(np.cos(phase)[:,None]*sideways+np.sin(phase)[:,None]*np.array([0,0,1]))
    native=np.column_stack([PLANE_X-PIVOT[0]-coil[:,2],-coil[:,0],coil[:,1]])+PIVOT
    return native@saw_rotation.T+saw_pos


def part(initial_points):
    text=f'''#usda 1.0
(defaultPrim = "SpringVisual")
def Xform "SpringVisual" {{
 def BasisCurves "coil" {{
  uniform token type = "linear"
  uniform token wrap = "nonperiodic"
  int[] curveVertexCounts = [{len(initial_points)}]
  point3f[] points = [{m._fmt(initial_points)}]
  float[] widths = [0.00045] (interpolation = "constant")
  color3f[] primvars:displayColor = [(0.08, 0.09, 0.10)]
 }}
}}
'''
    return m.Part('spring_visual',pos=(0,0,0),size=(.1,.1,.01),role='fixed',build=lambda:text)


class Render:
    def __init__(self,spring):
        self.spring=spring;m.on_step(self.draw);m.on_reset(self.draw)
    def draw(self,env=None):
        if env is None:return
        import isaaclab.sim as su
        from pxr import UsdGeom,Vt
        saw=m.state(env,'saw');hook=m.state(env,'spring_hook')
        p=points(saw.pos,saw.rotation(),hook.pos,hook.rotation(),self.spring.last.get('wrap_span_rad'))
        prim=su.get_current_stage().GetPrimAtPath(m.ENV_PATH+'/spring_visual/coil')
        if not prim:raise RuntimeError('spring render prim not found')
        UsdGeom.BasisCurves(prim).GetPointsAttr().Set(Vt.Vec3fArray(p.tolist()))
