"""Read-only 480 Hz latch/tool/grip contact evidence; no mechanics changes."""
import json
import numpy as np
import mechkit as m
from axial_observer_v15 import on_physics_step

VIEWS=[]
SAMPLES=[]
STEP=0
LAST=None
CAPACITY=4096

def initialize(env):
    from pxr import Usd,UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab_physx.physics import PhysxManager
    prims=Usd.PrimRange(sim_utils.get_current_stage().GetPseudoRoot(),Usd.TraverseInstanceProxies())
    paths=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    def unique(fragment,ending=None):
        found=[p for p in paths if fragment in p and (ending is None or p.endswith('/'+ending))]
        if len(found)!=1:raise RuntimeError('Latch contact coverage '+str(found))
        return found[0]
    assembly=[unique('/stapler/',n) for n in ['frame','cover','release']]
    tool=unique('/screwdriver')
    robot=[unique('/RightArm/',n) for n in ['tip_left','tip_right','gripper','link5']]
    groups=[('assembly_tool',assembly,[tool]),('grip_tool',robot,[tool]),('release_case',[assembly[2]],assembly[:2])]
    sim=PhysxManager.get_physics_sim_view()
    for name,sensors,filters in groups:
        view=sim.create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=CAPACITY)
        VIEWS.append((name,view,sensors,filters))
    print('LATCH_CONTACT_COVERAGE '+json.dumps({n:{'sensors':s,'filters':f} for n,_,s,f in VIEWS}),flush=True)

def ranges(counts,starts,size):
    if (counts<0).any() or counts.sum()>=CAPACITY:raise RuntimeError('Latch contact capacity/count')
    active=counts>0;b=starts[active];e=b+counts[active];order=np.argsort(b)
    if (b<0).any() or (e>size).any() or (b[order][1:]<e[order][:-1]).any():raise RuntimeError('Latch contact index/overlap')

def capture(dt):
    pairs={}
    for name,view,sensors,filters in VIEWS:
        normal,points,directions,_,counts,starts=view.get_contact_data(dt=dt)
        # Native getters share scratch buffers: copy before the next getter.
        normal=normal.numpy().copy().reshape(-1);points=points.numpy().copy();directions=directions.numpy().copy()
        counts=counts.numpy().copy();starts=starts.numpy().copy()
        friction,_,fc,fs=view.get_friction_data(dt=dt)
        friction=friction.numpy().copy();fc=fc.numpy().copy();fs=fs.numpy().copy()
        ranges(counts,starts,len(normal));ranges(fc,fs,len(friction))
        if not all(np.isfinite(a).all() for a in [normal,directions,friction]):raise RuntimeError('Nonfinite latch contact buffer')
        for i,sensor in enumerate(sensors):
            for j,target in enumerate(filters):
                n=int(counts[i,j]);start=int(starts[i,j]);fn=int(fc[i,j]);fstart=int(fs[i,j])
                nv=np.sum(normal[start:start+n,None]*directions[start:start+n],axis=0) if n else np.zeros(3)
                fv=np.sum(friction[fstart:fstart+fn],axis=0) if fn else np.zeros(3)
                key=name+':'+sensor.rsplit('/',1)[1]+':'+target.rsplit('/',1)[1]
                pairs[key]={'normal_count':n,'friction_count':fn,'normal_N':nv.tolist(),'friction_N':fv.tolist(),'total_N':(nv+fv).tolist(),'sum_normal_magnitudes_N':float(np.abs(normal[start:start+n]).sum()) if n else 0.,'sum_friction_magnitudes_N':float(np.linalg.norm(friction[fstart:fstart+fn],axis=-1).sum()) if fn else 0.,'points_m':points[start:start+min(n,3)].tolist() if n else []}
    return pairs

@on_physics_step
def physics(env,dt):
    if VIEWS:SAMPLES.append(capture(dt))

@m.on_step
def terminal(env):
    global STEP,LAST
    STEP+=1
    if not VIEWS:initialize(env);SAMPLES.clear();return
    SAMPLES.append(capture(env.unwrapped.physics_dt))
    if len(SAMPLES)!=25:raise RuntimeError('Expected 25 latch samples, got '+str(len(SAMPLES)))
    peaks={key:{'max_normal_count':max(s[key]['normal_count'] for s in SAMPLES),'max_normal_magnitudes_N':max(s[key]['sum_normal_magnitudes_N'] for s in SAMPLES),'max_friction_magnitudes_N':max(s[key]['sum_friction_magnitudes_N'] for s in SAMPLES)} for key in SAMPLES[-1]}
    LAST={'step':STEP,'samples':len(SAMPLES),'terminal':SAMPLES[-1],'peaks':peaks}
    print('LATCH_CONTACT '+json.dumps(LAST),flush=True)
    SAMPLES.clear()
