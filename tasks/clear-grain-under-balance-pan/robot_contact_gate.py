"""Read-only contact audit of every robot rigid body at every physics step.

Robot contact reporters are already enabled by the standard YAM asset. This
module changes no bodies, forces, poses, constraints or simulation settings.
Individual contacts and friction are checked in addition to net force, so
opposing forces cannot cancel into a false release witness.
"""
import json
import numpy as np
import mechkit as m
from physics_observer import on_physics_step

OBJECTS=('balance','pan','verification_mass','spatula','paper_cup','grain')
DIRECT={'balance','pan','verification_mass'}
VIEW=None;BODIES=[];FILTERS=[];FILTER_OBJECTS=[];SAMPLES=[];STEPS=0
LAST={'direct_clear':False,'contact_objects':[],'samples':0}

def initialize(env):
    global VIEW,BODIES,FILTERS,FILTER_OBJECTS
    import isaaclab.sim as sim_utils
    from isaaclab_physx.physics import PhysxManager
    from pxr import Usd,UsdPhysics,PhysxSchema
    stage=sim_utils.get_current_stage()
    rigid=[p for p in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies())
           if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    prefix='/World/envs/env_0/'
    BODIES=sorted(str(p.GetPath()) for p in rigid
        if str(p.GetPath()).startswith(prefix)
        and any('/'+side+'Arm/' in str(p.GetPath()) for side in ('Left','Right')))
    expected=sum(len(env.unwrapped.scene[side+'_robot'].body_names) for side in ('left','right'))
    if len(BODIES)!=expected or not BODIES:
        raise RuntimeError(f'Robot contact coverage incomplete: {len(BODIES)} vs {expected}')
    for side,title in (('left','Left'),('right','Right')):
        paths=[p for p in BODIES if '/'+title+'Arm/' in p]
        names={stage.GetPrimAtPath(p).GetName() for p in paths}
        if names!=set(env.unwrapped.scene[side+'_robot'].body_names):
            raise RuntimeError('Robot contact body names do not match '+side+' articulation')
    for path in BODIES:
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxContactReportAPI):
            raise RuntimeError('Missing robot contact reporter: '+path)
    FILTERS=[];FILTER_OBJECTS=[]
    for name in OBJECTS:
        found=sorted(str(p.GetPath()) for p in rigid
            if str(p.GetPath())==prefix+name or str(p.GetPath()).startswith(prefix+name+'/'))
        if not found:raise RuntimeError('Missing contact target bodies: '+name)
        FILTERS.extend(found);FILTER_OBJECTS.extend([name]*len(found))
    VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        BODIES,filter_patterns=[FILTERS]*len(BODIES),max_contact_data_count=32768)
    print('ROBOT_CONTACT_COVERAGE',json.dumps({'bodies':BODIES,'filters':FILTERS,
        'filter_objects':FILTER_OBJECTS,'expected_robot_bodies':expected}),flush=True)

def active_values(values,counts,starts):
    """Only active slices: unused contact buffer entries may contain old data."""
    out=[];n=len(values)
    for count,start in zip(counts.ravel(),starts.ravel()):
        count=int(count);start=int(start)
        if count==0:continue
        if count<0 or start<0 or start+count>n:raise RuntimeError('Invalid contact buffer range')
        if count:out.append(values[start:start+count])
    return np.concatenate(out,axis=0) if out else np.zeros((0,)+values.shape[1:])

def validate_ranges(counts,starts,capacity):
    """Reject saturation, invalid indexing, and overlapping active slices."""
    if counts.shape!=starts.shape or np.any(counts<0) or counts.sum()>=capacity:
        raise RuntimeError('Invalid or exhausted robot contact buffer')
    active=counts>0;begin=starts[active];end=begin+counts[active]
    if np.any(begin<0) or np.any(end>capacity):
        raise RuntimeError('Robot contact buffer range outside capacity')
    order=np.argsort(begin)
    if np.any(begin[order][1:]<end[order][:-1]):
        raise RuntimeError('Overlapping robot contact buffer ranges')

def capture(env,dt,detailed=False):
    if VIEW is None:return
    shape=(len(BODIES),len(FILTERS))
    matrix=np.asarray(VIEW.get_contact_force_matrix(dt=dt).numpy(),dtype=float).copy()
    normal,_,_,_,counts,starts=VIEW.get_contact_data(dt=dt)
    # Friction refreshes the same count/index buffers; copy normal buffers first.
    counts=np.asarray(counts.numpy(),dtype=int).copy()
    starts=np.asarray(starts.numpy(),dtype=int).copy()
    if detailed:
        normal=np.asarray(normal.numpy(),dtype=float).copy()
    friction,_,fcounts,fstarts=VIEW.get_friction_data(dt=dt)
    fcounts=np.asarray(fcounts.numpy(),dtype=int).copy()
    fstarts=np.asarray(fstarts.numpy(),dtype=int).copy()
    if detailed:
        friction=np.asarray(friction.numpy(),dtype=float).copy()
    if matrix.shape!=shape+(3,) or counts.shape!=shape or fcounts.shape!=shape:
        raise RuntimeError(f'Unexpected contact shapes {matrix.shape}/{counts.shape}/{fcounts.shape}')
    if not np.isfinite(matrix).all() or (detailed and not all(np.isfinite(a).all() for a in (normal,friction))):
        raise RuntimeError('Nonfinite robot contact data')
    validate_ranges(counts,starts,32768)
    validate_ranges(fcounts,fstarts,32768)
    norm=np.linalg.norm(matrix,axis=-1)
    active=(counts>0)|(fcounts>0)|(norm>1e-12)
    objects={FILTER_OBJECTS[j] for j in np.flatnonzero(active.any(axis=0))}
    row={'objects':objects,'net_max_n':float(norm.max()),
        'normal_points':int(counts.sum()),'friction_points':int(fcounts.sum())}
    if detailed:
        active_normal=active_values(normal,counts,starts)
        active_friction=active_values(friction,fcounts,fstarts)
        normals={name:matrix[:,[i for i,v in enumerate(FILTER_OBJECTS) if v==name],:].sum(axis=(0,1))
                 for name in OBJECTS}
        frictions={name:np.zeros(3) for name in OBJECTS}
        for i,j in zip(*np.nonzero(fcounts)):
            count=int(fcounts[i,j]);start=int(fstarts[i,j])
            frictions[FILTER_OBJECTS[j]]+=friction[start:start+count].sum(axis=0)
        row.update(normal_force_terminal_n={n:v.tolist() for n,v in normals.items()},
            friction_force_terminal_n={n:v.tolist() for n,v in frictions.items()},
            total_force_terminal_n={n:(normals[n]+frictions[n]).tolist() for n in OBJECTS},
            normal_max_terminal_n=float(np.max(np.abs(active_normal),initial=0.)),
            friction_max_terminal_n=float(np.max(np.abs(active_friction),initial=0.)))
    SAMPLES.append(row)

@on_physics_step
def physics_sample(env,dt):capture(env,dt)

def reset(env=None):
    global VIEW,STEPS
    VIEW=None;STEPS=0;SAMPLES.clear()
    LAST.clear();LAST.update(direct_clear=False,contact_objects=[],samples=0)
    if env is not None:initialize(env)

def step(env):
    global STEPS
    STEPS+=1
    if VIEW is None:initialize(env)
    capture(env,env.unwrapped.physics_dt,detailed=True)
    ratio=float(env.unwrapped.step_dt/env.unwrapped.physics_dt)
    substeps=round(ratio)
    if substeps<1 or abs(ratio-substeps)>1e-8:
        raise RuntimeError('Nonintegral physics substeps per control interval')
    if len(SAMPLES)!=substeps+1:
        raise RuntimeError(f'Incomplete robot contact interval: {len(SAMPLES)} vs {substeps+1} samples')
    touched=set().union(*(s['objects'] for s in SAMPLES))
    LAST.clear();LAST.update(direct_clear=not bool(touched&DIRECT),
        contact_objects=sorted(touched),samples=len(SAMPLES),expected_samples=substeps+1,
        **{k:SAMPLES[-1][k] for k in ('normal_force_terminal_n','friction_force_terminal_n',
            'total_force_terminal_n','normal_max_terminal_n','friction_max_terminal_n')},
        **{k:max(s[k] for s in SAMPLES) for k in
           ('net_max_n','normal_points','friction_points')})
    if STEPS==1 or STEPS%20==0:print('ROBOT_CONTACT_GATE',STEPS,json.dumps(LAST),flush=True)
    SAMPLES.clear()

m.on_reset(reset);m.on_step(step)
