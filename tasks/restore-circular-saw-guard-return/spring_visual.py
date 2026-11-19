"""Render the continuous spring centerline; these curves have no colliders."""
import math
import numpy as np
import mechkit as m
from spring_mechanics import GUIDE_RADIUS,PIVOT,PLANE_X,routed_length_force


def coil_points(endpoint,theta):
    # Equal point count keeps the USD BasisCurves topology constant.
    span=(theta-math.pi/2)%(2*math.pi)
    arc=np.linspace(math.pi/2,math.pi/2+span,700)
    centers=np.column_stack([GUIDE_RADIUS*np.cos(arc),GUIDE_RADIUS*np.sin(arc),np.zeros(len(arc))])
    end=np.asarray(endpoint,float)
    straight=np.linspace(centers[-1],end,101)[1:]
    centers=np.vstack([centers,straight])
    tangent=np.gradient(centers,axis=0)
    tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-9)
    horizontal=np.column_stack([-tangent[:,1],tangent[:,0],np.zeros(len(tangent))])
    phase=np.linspace(0,2*math.pi*125,len(centers))
    points=centers+.0018*(np.cos(phase)[:,None]*horizontal+np.sin(phase)[:,None]*np.array([0.,0.,1.]))
    return points


def visual_part(points):
    points=np.asarray(points)
    body=f'''#usda 1.0
(defaultPrim = "SpringVisual")
def Xform "SpringVisual" {{
 def BasisCurves "coil" {{
  uniform token type = "linear"
  uniform token wrap = "nonperiodic"
  int[] curveVertexCounts = [{len(points)}]
  point3f[] points = [{m._fmt(points)}]
  float[] widths = [0.00045] (interpolation = "constant")
  color3f[] primvars:displayColor = [(0.15, 0.17, 0.18)]
 }}
}}
'''
    return m.Part('spring_visual',pos=(0.,0.,0.),size=(.12,.12,.01),role='fixed',build=lambda:body)


class CoilVisual:
    def __init__(self):
        m.on_step(self.draw);m.on_reset(self.draw)
    def draw(self,env=None):
        if env is None:return
        import isaaclab.sim as sim_utils
        from pxr import UsdGeom,Vt
        saw=m.state(env,'saw');hook=m.state(env,'spring_hook')
        attach=hook.pos+hook.rotation()@np.array([-.006,-.0018,0.])
        a=saw.rotation().T@(attach-saw.pos)-PIVOT
        endpoint=np.array([-a[1],a[2],-(a[0]-(PLANE_X-PIVOT[0]))])
        _,_,theta=routed_length_force(endpoint)
        plane=coil_points(endpoint,theta)
        native=np.column_stack([PLANE_X-PIVOT[0]-plane[:,2],-plane[:,0],plane[:,1]])+PIVOT
        points=native@saw.rotation().T+saw.pos
        prim=sim_utils.get_current_stage().GetPrimAtPath(m.ENV_PATH+'/spring_visual/coil')
        if not prim:raise RuntimeError('spring visual prim absent')
        UsdGeom.BasisCurves(prim).GetPointsAttr().Set(Vt.Vec3fArray(points.tolist()))
