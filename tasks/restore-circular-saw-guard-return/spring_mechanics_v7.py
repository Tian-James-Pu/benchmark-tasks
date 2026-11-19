"""GPU-resident conservative spring, evaluated once per physics step.

Planar routed extension plus a weak transverse elastic term. These form an
explicit continuous potential; no per-substep CPU state copies.
"""
import math
import mechkit as m
from physics_hooks import on_physics_step
from spring_mechanics import REST_LENGTH,SPRING_K,SPRING_C,PIVOT,PLANE_X
GUIDE_RADIUS=.0213
FIXED_ANGLE=math.radians(65.)


def rotate(q,v):
    import torch
    xyz=q[...,:3];w=q[...,3:4]
    t=2*torch.linalg.cross(xyz,v,dim=-1)
    return v+w*t+torch.linalg.cross(xyz,t,dim=-1)


class ExtensionSpring:
    def __init__(self):
        self.environment=None;self.calls=0;self.last={};self.last_tensor=None;self.previous_phi=None;self.unwrapped_phi=None
        on_physics_step(self.physics);m.on_reset(self.reset);m.on_step(self.sample)
    def reset(self,env=None):
        self.sample(env)
    def physics(self,env,dt):
        import torch
        base=env.unwrapped
        if self.environment is not base:
            self.environment=base;self.saw=base.scene['saw'];self.hook=base.scene['spring_hook']
            self.housing_idx=self.saw.body_names.index('housing')
            self.guard_idx=self.saw.joint_names.index('guard_pivot')
            self.pivot=torch.tensor(PIVOT,device=self.hook.device,dtype=torch.float32)[None,:]
            self.attachment=torch.tensor([-.008,-.0018,0.],device=self.hook.device)[None,:]
            m._PUSHED.add(('saw',self.housing_idx));m._PUSHED.add(('spring_hook',0))
        sp=self.saw.data.root_link_pose_w.torch;hp=self.hook.data.root_link_pose_w.torch
        sv=self.saw.data.root_link_vel_w.torch;hv=self.hook.data.root_link_vel_w.torch
        offset=rotate(hp[:,3:7],self.attachment)
        at=hp[:,:3]+offset
        invq=sp[:,3:7].clone();invq[:,:3]*=-1
        local=rotate(invq,at-sp[:,:3])-self.pivot
        x,y,z=-local[:,1],local[:,2],-(local[:,0]-(PLANE_X-PIVOT[0]))
        r=torch.sqrt(x*x+y*y).clamp_min(GUIDE_RADIUS+1e-7)
        root=torch.sqrt((r*r-GUIDE_RADIUS**2).clamp_min(0.))
        phi=torch.atan2(y,x)
        if self.previous_phi is None:
            self.unwrapped_phi=phi+torch.where(phi<FIXED_ANGLE,2*math.pi,0.)
        else:
            delta=torch.atan2(torch.sin(phi-self.previous_phi),torch.cos(phi-self.previous_phi))
            self.unwrapped_phi=self.unwrapped_phi+delta
        self.previous_phi=phi.clone()
        theta=self.unwrapped_phi-torch.acos(GUIDE_RADIUS/r)
        span=theta-FIXED_ANGLE
        wrapped=span>0.
        direct_x=x-GUIDE_RADIUS*math.cos(FIXED_ANGLE)
        direct_y=y-GUIDE_RADIUS*math.sin(FIXED_ANGLE)
        direct_length=torch.sqrt(direct_x*direct_x+direct_y*direct_y).clamp_min(1e-8)
        length=torch.where(wrapped,GUIDE_RADIUS*span+root,direct_length)
        # dL/dr=sqrt(r^2-R^2)/r, dL/dphi=R. Finite at the guide.
        dr=root/r
        gx=dr*x/r-GUIDE_RADIUS*y/(r*r)
        gy=dr*y/r+GUIDE_RADIUS*x/(r*r)
        gx=torch.where(wrapped,gx,direct_x/direct_length)
        gy=torch.where(wrapped,gy,direct_y/direct_length)
        gradient=rotate(sp[:,3:7],torch.stack([torch.zeros_like(x),-gx,gy],dim=1))
        normal=rotate(sp[:,3:7],torch.tensor([[-1.,0.,0.]],device=self.hook.device))
        vh=hv[:,:3]+torch.linalg.cross(hv[:,3:6],offset,dim=1)
        vs=sv[:,:3]+torch.linalg.cross(sv[:,3:6],at-sp[:,:3],dim=1)
        speed=((vh-vs)*gradient).sum(1)
        vertical_speed=((vh-vs)*normal).sum(1)
        extension=(length-REST_LENGTH).clamp_min(0.)
        tension=(SPRING_K*extension+SPRING_C*speed).clamp_min(0.)
        # A weak distributed transverse elasticity keeps the coil endpoint
        # coupled to the spring plane; 0.5 N/m is an explicitly estimated term.
        transverse=-.5*z-.005*vertical_speed
        force=(-tension[:,None]*gradient+transverse[:,None]*normal)[:,None,:]
        pos=at[:,None,:]
        damping_torque=(-1e-6*(hv[:,3:6]-sv[:,3:6]))[:,None,:]
        self.hook.permanent_wrench_composer.set_forces_and_torques_index(forces=force,torques=damping_torque,positions=pos,body_ids=[0],is_global=True)
        self.saw.permanent_wrench_composer.set_forces_and_torques_index(forces=-force,torques=-damping_torque,positions=pos,body_ids=[self.housing_idx],is_global=True)
        q=self.saw.data.joint_pos.torch[:,self.guard_idx]
        self.last_tensor=torch.cat([length[:,None],tension[:,None],at,hp[:,:3],q[:,None],(.5*SPRING_K*extension**2+.5*.5*z*z)[:,None],r[:,None],sp[:,:3],span[:,None]],dim=1)
        self.calls+=1
        if self.calls in (1,2,10,100):
            self.sample(env);print('SPRING_INIT',self.calls,self.last,flush=True)
    def sample(self,env=None):
        if self.last_tensor is None:return
        a=self.last_tensor.detach().cpu().numpy()[0]
        self.last={'length_m':float(a[0]),'tension_n':float(a[1]),'hook_attachment':a[2:5].tolist(),
                   'hook_pos':a[5:8].tolist(),'guard_rad':float(a[8]),'spring_energy_j':float(a[9]),
                   'endpoint_radius_m':float(a[10]),'saw_pos':a[11:14].tolist(),'wrap_span_rad':float(a[14])}
