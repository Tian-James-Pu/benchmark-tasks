"""Unchanged repair-policy prefix with read-only substep grip instrumentation."""
import pathlib,sys,json
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from physics_observer import on_physics_step
from native_mouth_audit import array

ROWS=[];SERIAL=0
def reset_grip(env=None):
    global SERIAL
    SERIAL=0;ROWS.clear()
m.on_reset(reset_grip)

def read_grip(env,dt,boundary=False):
    global SERIAL
    if audit.view is None:return
    view=audit.view
    net=array(view.get_contact_force_matrix(dt=dt))
    nf,points,normals,separations,counts,starts=view.get_contact_data(dt=dt)
    # Every normal buffer is copied before the shared friction API buffers change.
    nv=array(nf).reshape(-1);npnt=array(points).reshape(-1,3);nn=array(normals).reshape(-1,3);sep=array(separations).reshape(-1);nc=array(counts);ns=array(starts)
    audit.pair_sums(np.abs(nv),nc,ns)
    ff,fp,counts,starts=view.get_friction_data(dt=dt)
    fv=array(ff).reshape(-1,3);fp=array(fp).reshape(-1,3);fc=array(counts);fs=array(starts)
    audit.pair_sums(np.linalg.norm(fv,axis=1),fc,fs)
    pairs=[];j=audit.filters.index('/World/envs/env_0/crutch')
    for i,body in enumerate(audit.bodies):
        if not body.endswith(('/tip_left','/tip_right')):continue
        na,nb=int(ns[i,j]),int(nc[i,j]);fa,fb=int(fs[i,j]),int(fc[i,j])
        pairs.append({'body':body,'net_normal_vector_n':net[i,j].tolist(),'normal_force_n':nv[na:na+nb].tolist(),'normal_directions_world':nn[na:na+nb].tolist(),'normal_vectors_world_n':(nv[na:na+nb,None]*nn[na:na+nb]).tolist(),'contact_positions_world_m':npnt[na:na+nb].tolist(),'separations_m':sep[na:na+nb].tolist(),'friction_vectors_world_n':fv[fa:fa+fb].tolist(),'friction_positions_world_m':fp[fa:fa+fb].tolist()})
    arms={}
    for side in ['left','right']:
        robot=env.unwrapped.scene[side+'_robot'];ids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0];root=robot.data._root_view
        entry={'finger_joint_names':['joint7','joint8'],'finger_positions_m':m._np(robot.data.joint_pos)[0,ids].tolist(),'finger_velocities_m_s':m._np(robot.data.joint_vel)[0,ids].tolist(),'finger_applied_drive_estimate_n':m._np(robot.data.applied_torque)[0,ids].tolist(),'finger_effort_limits_n':m._np(robot.data.joint_effort_limits)[0,ids].tolist(),'native_all_dof_positions':array(root.get_dof_positions()).reshape(-1).tolist(),'native_all_dof_velocities':array(root.get_dof_velocities()).reshape(-1).tolist()}
        try:
            entry['native_dof_names']=list(root.shared_metatype.dof_names)
            entry['native_projected_joint_forces']=array(root.get_dof_projected_joint_forces()).reshape(-1).tolist()
        except (AttributeError,RuntimeError) as exc:entry['projected_force_read_error']=str(exc)
        arms[side]=entry
    shaft=env.unwrapped.scene['crutch'].data._root_view
    row={'serial':SERIAL,'completed_control_step':c.COUNT,'boundary':boundary,'dt_s':dt,'pairs':pairs,'arms':arms,'shaft_transform_xyzw':array(shaft.get_transforms()).reshape(-1,7)[0].tolist()}
    SERIAL+=1;ROWS.append(row)

@on_physics_step
def capture_grip(env,dt):
    if 419<=c.COUNT<460:read_grip(env,dt)

def finish_grip_interval(env):
    if not 420<=c.COUNT<=460:return
    read_grip(env,env.unwrapped.physics_dt,True)
    if len(ROWS)!=25:raise RuntimeError(f'Expected25 detailed grip samples, got{len(ROWS)}')
    print('GRIP_SUBSTEP_SAMPLES',c.COUNT,json.dumps(ROWS),flush=True);ROWS.clear()
m.on_step(finish_grip_interval)
