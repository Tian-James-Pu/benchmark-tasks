"""Read-only diagnostic of each native rod joint before integration."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from physics_observer import on_physics_step


def install(paths, center):
    state = {'view': None, 'sample': 0}
    expected = [f'/World/envs/env_0/braid/w{w}_s{i:04d}'
                for w, points in enumerate(paths) for i in range(len(points)-1)]
    lengths = np.concatenate([np.linalg.norm(np.diff(p, axis=0), axis=1) for p in paths])
    starts = np.cumsum([0]+[len(p)-1 for p in paths])
    pairs = np.concatenate([np.arange(a,b-1) for a,b in zip(starts[:-1],starts[1:])])
    authored = np.concatenate([(p[1:]+p[:-1])/2 for p in paths])+center

    @on_physics_step
    def capture(env, dt):
        from isaaclab_physx.physics import PhysxManager
        state['sample'] += 1
        sample = state['sample']
        if state['view'] is None:
            state['view'] = PhysxManager.get_physics_sim_view().create_rigid_body_view(expected)
            view_paths = list(state['view'].prim_paths)
            if len(view_paths) != len(expected) or set(view_paths) != set(expected):
                raise RuntimeError('Braid body-view identity mismatch')
            state['order'] = [view_paths.index(p) for p in expected]
        if sample > 10 and sample % 60:
            return
        poses = np.asarray(state['view'].get_transforms().numpy())[state['order']]
        rot = Rotation.from_quat(poses[:,3:7])
        axis = rot.apply(np.column_stack([lengths/2,np.zeros((len(lengths),2))]))
        head, tail = poses[:,:3]-axis, poses[:,:3]+axis
        gaps = np.linalg.norm(tail[pairs]-head[pairs+1],axis=1)
        clamp_errors = []
        for w,(a,b) in enumerate(zip(starts[:-1],starts[1:])):
            clamp_errors += [np.linalg.norm(head[a]-(center+paths[w][0])),
                             np.linalg.norm(tail[b-1]-(center+paths[w][-1]))]
        row = {'sample': sample, 'dt': dt,
               'max_joint_gap_m': float(gaps.max()),
               'median_joint_gap_m': float(np.median(gaps)),
               'max_clamp_error_m': float(max(clamp_errors)),
               'max_authored_displacement_m': float(np.linalg.norm(poses[:,:3]-authored,axis=1).max()),
               'min_x_m': float(poses[:,0].min()-center[0]),
               'max_x_m': float(poses[:,0].max()-center[0]),
               'worst_joint_parent': expected[pairs[gaps.argmax()]]}
        print('BRAID_CONSTRAINT_AUDIT',json.dumps(row),flush=True)
