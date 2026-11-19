"""Conservative all-link mesh-bound clearance, external and not native-validated.

Distance between enclosing AABBs is a lower bound on collision-mesh distance.
An overlap returns zero; it never fabricates clearance through an empty region
of a concave object. All collision meshes must be covered or acquisition fails.
"""
from itertools import product
import numpy as np
from assembly_score import Pose
from native_pose_reader import LINKS, array


def corners(low, high):
    return np.array(list(product(*zip(low, high))), dtype=float)


class NativeClearanceReader:
    def __init__(self):
        self.local_bounds = {}
        self.robot_paths = {}

    def initialize(self, env):
        import isaaclab.sim as sim_utils
        from pxr import UsdPhysics, UsdGeom, Usd, Gf
        stage = sim_utils.get_current_stage()
        prefix = '/World/envs/env_0/'
        expected = {prefix+a+'/'+b for a, b in LINKS.values()}
        for arm, asset_name in [('LeftArm', 'left_robot'), ('RightArm', 'right_robot')]:
            names = list(env.unwrapped.scene[asset_name].body_names)
            rigid = [str(p.GetPath()) for p in stage.Traverse()
                     if p.HasAPI(UsdPhysics.RigidBodyAPI)
                     and str(p.GetPath()).startswith(prefix+arm+'/')]
            if len(rigid) != 9 or len(names) != 9:
                raise ValueError('expected nine original bodies per arm')
            paths = {}
            for name in names:
                matches = [p for p in rigid if p.rsplit('/', 1)[-1] == name]
                if len(matches) != 1:
                    raise ValueError('robot native-name/USD-body mapping is ambiguous')
                paths[name] = matches[0]
            self.robot_paths[asset_name] = paths
            expected.update(rigid)
        transforms = UsdGeom.XformCache(Usd.TimeCode.Default())
        vertices = {p: [] for p in expected}
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            parent = prim
            while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
                parent = parent.GetParent()
            if not parent or str(parent.GetPath()) not in expected:
                continue
            if not prim.IsA(UsdGeom.Mesh):
                raise ValueError('unhandled collision geometry; no clearance assumed')
            points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float)
            if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                raise ValueError('invalid collision mesh points')
            # Transform all eight corners of a local shape enclosure. This
            # remains conservative with arbitrary authored scale/orientation.
            shape_world = transforms.GetLocalToWorldTransform(prim)
            body_inverse = transforms.GetLocalToWorldTransform(parent).GetInverse()
            bound = [body_inverse.Transform(shape_world.Transform(Gf.Vec3d(*map(float, p))))
                     for p in corners(points.min(0), points.max(0))]
            vertices[str(parent.GetPath())].extend(bound)
        for body, values in vertices.items():
            if not values:
                raise ValueError(f'missing collision mesh bound for {body}')
            values = np.asarray(values, float)
            self.local_bounds[body] = corners(values.min(0), values.max(0))

    def read(self, env, snapshot):
        prefix = '/World/envs/env_0/'
        poses = {prefix+asset+'/'+link: getattr(snapshot.assembly, name)
                 for name, (asset, link) in LINKS.items()}
        assembly_paths = set(poses)
        origin = np.asarray(snapshot.origin_world_m)
        for asset_name, paths in self.robot_paths.items():
            asset = env.unwrapped.scene[asset_name]
            if not type(asset.data).__module__.startswith('isaaclab_physx.'):
                raise ValueError('robot poses require verified PhysX xyzw backend')
            raw = array(asset.data.body_link_pose_w)
            if raw.shape != (1, len(paths), 7):
                raise ValueError('unexpected robot body pose shape')
            for i, name in enumerate(asset.body_names):
                poses[paths[name]] = Pose(tuple(raw[0, i, :3]-origin), tuple(raw[0, i, 3:]))
        if set(poses) != set(self.local_bounds) or len(poses) != 25:
            raise ValueError('incomplete all-body clearance coverage')
        bounds = {}
        for path, pose in poses.items():
            verts = pose.world(self.local_bounds[path])
            bounds[path] = (verts.min(0), verts.max(0))
        result = None
        for a in assembly_paths:
            low_a, high_a = bounds[a]
            for b in set(poses)-assembly_paths:
                low_b, high_b = bounds[b]
                gap = np.maximum(np.maximum(low_a-high_b, low_b-high_a), 0.)
                distance = float(np.linalg.norm(gap))
                if result is None or distance < result['lower_bound_m']:
                    result = dict(lower_bound_m=distance, assembly_body=a, robot_body=b)
        return result
