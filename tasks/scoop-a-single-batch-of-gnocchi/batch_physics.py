"""Independent engineering buoyancy/cooking proxy, SI units; no third-party task code.

Equal-volume quadrature over ellipsoids, Archimedes force at submerged centroid,
quadratic water drag and explicit damping. Cooking only advances during immersion.
Density change is an estimated expansion proxy with fixed native collision geometry.
"""
import numpy as np
SIZE=np.array([.024,.018,.014]); HALF=SIZE/2
VOLUME=4*np.pi*np.prod(HALF)/3
RAW_DENSITY=1080.; FLOAT_DENSITY=950.; WATER_DENSITY=1000.; GRAVITY=9.81
MASS=RAW_DENSITY*VOLUME
N=32
# One batch, identical immersion age, modest within-batch size/cooking variation.
INITIAL_AGE=80.
FLOAT_AGES=np.linspace(90.,120.,N)[np.random.default_rng(2048).permutation(N)]
a=np.linspace(-.9,.9,7)
pts=np.array(np.meshgrid(a,a,a,indexing='ij')).reshape(3,-1).T
LOCAL=pts[np.sum(pts*pts,axis=1)<=1]*HALF

def rotations(q):
    q=np.asarray(q);x,y,z,w=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)

def water_forces(poses,velocities,age,surface,pan_center,pan_radius,floor,pan_pose=None,pan_velocity=None):
    rotations_=rotations(poses[:,3:7]);offset=np.einsum('bij,kj->bki',rotations_,LOCAL)
    p=poses[:,:3,None].transpose(0,2,1)+offset
    if pan_pose is None:
        radial=np.linalg.norm(p[:,:,:2]-np.asarray(pan_center),axis=2)
        above_floor=p[:,:,2]>floor
    else:
        local=(p-pan_pose[:3])@rotations(pan_pose[None,3:7])[0]
        radial=np.linalg.norm(local[:,:,:2],axis=2)
        above_floor=local[:,:,2]>.005
    #1mm surface transition prevents discontinuous numerical impulses at quadrature boundaries.
    wet=np.clip((surface-p[:,:,2])/.001+.5,0.,1.)*(radial<pan_radius)*above_floor
    fraction=wet.mean(axis=1)
    progress=np.clip((age-FLOAT_AGES)/2.,0.,1.)
    rho=RAW_DENSITY+(FLOAT_DENSITY-RAW_DENSITY)*progress
    effective_volume=VOLUME*RAW_DENSITY/rho
    lift=WATER_DENSITY*GRAVITY*effective_volume[:,None]/len(LOCAL)*wet
    torque=np.cross(offset,np.stack([np.zeros_like(lift),np.zeros_like(lift),lift],axis=2)).sum(axis=1)
    # Conservative drag approximation; no prescribed attraction towards spoon/plate.
    v=velocities[:,:3].copy()
    angular=velocities[:,3:6].copy()
    if pan_velocity is not None:
        v-=pan_velocity[:3]+np.cross(pan_velocity[3:6],poses[:,:3]-pan_pose[:3])
        angular-=pan_velocity[3:6]
    speed=np.linalg.norm(v,axis=1)
    area=np.pi*HALF[0]*HALF[1]
    force=-fraction[:,None]*(.5*WATER_DENSITY*.9*area*speed[:,None]*v+.020*v)
    force[:,2]+=lift.sum(axis=1)
    torque-=fraction[:,None]*1e-5*angular
    return force,torque,fraction

class BatchClock:
    def __init__(self):
        self.age=np.full(N,INITIAL_AGE);self.post_float=np.zeros(N)
        self.floated=np.zeros(N,dtype=bool);self.first_float=np.full(N,np.nan)
        self.t=0.
    def advance(self,dt,immersion,top,surface,unsupported):
        hot=np.asarray(immersion)>=.75
        self.age+=dt*hot
        newly=(~self.floated)&(self.age>=FLOAT_AGES+2)&hot&(np.abs(top-surface)<.004)&unsupported
        self.first_float[newly]=self.t;self.floated|=newly
        self.post_float+=dt*(self.floated&hot)
        self.t+=dt
    @property
    def ready(self):return self.floated&(self.post_float>=60.)
