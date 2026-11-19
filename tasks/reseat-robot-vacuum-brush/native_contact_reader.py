"""External read-only contact acquisition draft, not a support classifier.

Collects individual normal and friction contacts as well as net normal forces.
No nonzero score or supported/released flag is inferred from these records.
Native force direction, path ordering and reset calibration remain required.
"""
import numpy as np
from native_pose_reader import LINKS, array

CAPACITY = 32768


def indices(value):
    if hasattr(value, 'torch'):
        value = value.torch
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    elif hasattr(value, 'numpy'):
        value = value.numpy()
    result = np.asarray(value).copy()
    if not np.issubdtype(result.dtype, np.integer):
        raise ValueError('native contact indices must be integers')
    return result


def ranges(counts, starts, shape, data_length):
    if counts.shape != shape or starts.shape != shape or np.any(counts < 0):
        raise ValueError('invalid native contact indexing shape/count')
    if int(counts.sum()) >= CAPACITY:
        raise ValueError('native contact buffer may be saturated')
    result = []
    for i, j in np.argwhere(counts > 0):
        lo, count = int(starts[i, j]), int(counts[i, j])
        if lo < 0 or lo+count > data_length:
            raise ValueError('native contact range exceeds data buffer')
        result.append((int(i), int(j), lo, lo+count))
    intervals = sorted((lo, hi) for _, _, lo, hi in result)
    if any(b[0] < a[1] for a, b in zip(intervals, intervals[1:])):
        raise ValueError('overlapping native contact ranges')
    return result


class NativeContactReader:
    def __init__(self):
        self.view = None
        self.bodies = []
        self.filters = []
        self.robot_columns = []

    def initialize(self, env):
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        import isaaclab.sim as sim_utils
        prefix = '/World/envs/env_0/'
        prims = list(sim_utils.get_current_stage().Traverse())
        rigid = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.bodies = [prefix+asset+'/'+link for asset, link in LINKS.values()]
        if len(self.bodies) != 7 or not set(self.bodies).issubset(rigid):
            raise ValueError('missing one of seven original assembly bodies')
        # Include other assembly bodies: they provide physical support paths.
        # Dynamic rigid bodies are filters once, never again as child shapes.
        self.filters = [p for p in rigid if p.startswith(prefix)]
        for prim in prims:
            path = str(prim.GetPath())
            if not path.startswith(prefix) or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            parent, has_rigid_parent = prim, False
            while parent and str(parent.GetPath()).startswith(prefix):
                if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                    has_rigid_parent = True
                    break
                parent = parent.GetParent()
            if not has_rigid_parent:
                self.filters.append(path)
        if len(self.filters) != len(set(self.filters)):
            raise ValueError('duplicate native contact filters')
        self.robot_columns = [i for i, p in enumerate(self.filters)
                              if '/LeftArm/' in p or '/RightArm/' in p]
        if len(self.robot_columns) != 18:
            raise ValueError('all eighteen original robot bodies are required')
        if not any(p == prefix+'towel' or p.startswith(prefix+'towel/') for p in self.filters):
            raise ValueError('original service towel missing from support filters')
        if not any('/table/' in p for p in self.filters):
            raise ValueError('original bench missing from support filters')
        self.shape = (len(self.bodies), len(self.filters))
        self.view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            self.bodies, filter_patterns=[self.filters]*len(self.bodies),
            max_contact_data_count=CAPACITY)

    def read(self, dt):
        if self.view is None or not np.isfinite(dt) or dt <= 0:
            raise ValueError('contact view not initialized or invalid physics dt')
        net = array(self.view.get_contact_force_matrix(dt=dt))
        if net.shape != (*self.shape, 3):
            raise ValueError('unexpected native contact matrix shape')
        forces, points, normals, separations, counts, starts = self.view.get_contact_data(dt=dt)
        # The friction getter can overwrite PhysX buffers; copy all normal data first.
        counts, starts = indices(counts), indices(starts)
        # Unused capacity slots are unspecified; validate only indexed contacts.
        forces, points = array(forces, finite=False).reshape(-1), array(points, finite=False).reshape(-1, 3)
        normals, separations = array(normals, finite=False).reshape(-1, 3), array(separations, finite=False).reshape(-1)
        normal_ranges = ranges(counts, starts, self.shape, len(forces))
        if not (len(forces) == len(points) == len(normals) == len(separations)):
            raise ValueError('inconsistent native normal contact buffers')
        friction, friction_points, friction_counts, friction_starts = self.view.get_friction_data(dt=dt)
        friction_counts, friction_starts = indices(friction_counts), indices(friction_starts)
        friction, friction_points = array(friction, finite=False).reshape(-1, 3), array(friction_points, finite=False).reshape(-1, 3)
        if len(friction) != len(friction_points):
            raise ValueError('inconsistent native friction contact buffers')
        friction_ranges = ranges(friction_counts, friction_starts, self.shape, len(friction))
        pairs = {}
        def pair(i, j):
            return pairs.setdefault((i, j), dict(body=self.bodies[i], other=self.filters[j],
                net_normal_world_N=net[i, j].tolist(), normal_forces_N=[],
                normal_points_world_m=[], normals=[], separations_m=[],
                friction_forces_world_N=[], friction_points_world_m=[]))
        for i, j, lo, hi in normal_ranges:
            if not all(np.isfinite(x[lo:hi]).all() for x in (forces, points, normals, separations)):
                raise ValueError('nonfinite active native normal contact')
            row = pair(i, j)
            row.update(normal_forces_N=forces[lo:hi].tolist(),
                       normal_points_world_m=points[lo:hi].tolist(),
                       normals=normals[lo:hi].tolist(),
                       separations_m=separations[lo:hi].tolist())
        for i, j, lo, hi in friction_ranges:
            if not all(np.isfinite(x[lo:hi]).all() for x in (friction, friction_points)):
                raise ValueError('nonfinite active native friction contact')
            row = pair(i, j)
            row.update(friction_forces_world_N=friction[lo:hi].tolist(),
                       friction_points_world_m=friction_points[lo:hi].tolist())
        # Preserve any net-only result; never interpret missing point data as zero.
        for i, j in np.argwhere(np.linalg.norm(net, axis=-1) > 0):
            pair(int(i), int(j))
        return dict(dt_s=float(dt), pairs=list(pairs.values()),
                    normal_point_count=int(counts.sum()),
                    friction_point_count=int(friction_counts.sum()))
