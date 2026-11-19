"""Continuous extension spring with a free rigid wire hook.

The spring follows a frictionless circular bearing boss; its stored energy is
0.5*k*max(path_length-rest_length, 0)**2. Guard torque comes only from collision
between the free hook and the guard's mounting pin. No attachment test, pose
snap, state change, or proximity-dependent force is used.
"""
import math
import numpy as np
import mechkit as m
from physics_hooks import on_physics_step

GUIDE_RADIUS=.022
REST_LENGTH=.065
SPRING_K=60.
SPRING_C=.025
PIVOT=np.array([-.055,0.,.065])
PLANE_X=-.0715


def routed_length_force(point):
    """Length and unit gradient in saw-local world-oriented XY coordinates.

The bearing lies at the origin; fixed spring end is at polar angle +pi/2.
The coil travels counterclockwise around the bearing before reaching its hook.
"""
    x,y,z=point
    r=math.hypot(x,y)
    if r <= GUIDE_RADIUS+1e-8:
        # The rigid bearing should prevent this. An inward endpoint is a model
        # diagnostic, never permission to attach it to the guard.
        r=GUIDE_RADIUS+1e-8
    phi=math.atan2(y,x)
    alpha=math.acos(min(1.,GUIDE_RADIUS/r))
    theta=phi-alpha
    span=(theta-math.pi/2)%(2*math.pi)
    t=np.array([GUIDE_RADIUS*math.cos(theta),GUIDE_RADIUS*math.sin(theta),0.])
    v=np.array([x,y,z])-t
    straight=float(np.linalg.norm(v))
    length=GUIDE_RADIUS*span+straight
    gradient=v/max(straight,1e-12)
    return length,gradient,theta


class ExtensionSpring:
    def __init__(self):
        self.samples=[];self.calls=0;self.last={};self.environment=None
        on_physics_step(self.physics)
        m.on_reset(self.reset)
    def reset(self,env=None):
        self.samples=[];self.calls=0
    def physics(self,env,dt):
        # Physical state, rather than a scripted trajectory, determines force.
        saw=m.state(env,'saw');hook=m.state(env,'spring_hook')
        R=saw.rotation(); H=hook.rotation()
        attach=hook.pos+H@np.array([-.006,-.0018,0.])
        # Saw native frame -> plane coordinates (u=-native y,v=native z).
        a=R.T@(attach-saw.pos)-PIVOT
        plane=np.array([-a[1],a[2],-(a[0]-(PLANE_X-PIVOT[0]))])
        length,g,theta=routed_length_force(plane)
        native_gradient=np.array([-g[2],-g[0],g[1]])
        world_gradient=R@native_gradient
        # Relative endpoint velocity includes the housing's rigid motion.
        vh=hook.lin_vel+np.cross(hook.ang_vel,H@np.array([-.006,-.0018,0.]))
        vs=saw.lin_vel+np.cross(saw.ang_vel,attach-saw.pos)
        speed=float(np.dot(vh-vs,world_gradient))
        tension=max(0.,SPRING_K*max(length-REST_LENGTH,0.)+SPRING_C*speed)
        force=-tension*world_gradient
        m.push(env,'spring_hook',force,at=attach)
        # Equivalent resultant of the fixed-end and bearing contact reactions.
        m.push(env,'saw',-force,at=attach,body='housing')
        self.calls+=1
        self.last={'length_m':length,'tension_n':tension,'hook_attachment':attach.tolist(),
                   'hook_pos':hook.pos.tolist(),'guard_rad':saw.joints['guard_pivot'],
                   'spring_energy_j':.5*SPRING_K*max(length-REST_LENGTH,0.)**2,
                   'endpoint_radius_m':float(np.linalg.norm(plane[:2]))}
        if self.calls%96==0:self.samples.append(dict(self.last))
