"""Closed chain with reduced-coordinate sections and four ordinary closure pins.

This changes the numerical constraint representation, preserving every body,
collider, mass, inertia, pin response and roller degree of freedom. Each section
has at most64 links; the installed GPU PhysX has a reported >64-link bug. This
is a score-disabled drivetrain diagnostic, not a complete bicycle task.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dataclasses import replace
import probe_chain_closed_loop as base
from probe_chain_closed_loop import *
from physics_hooks import on_physics_step

ROOT_LINKS = (0, 32, 64, 96)
BOUNDARIES = (31, 63, 95, 105)
_original_chain = CHAIN.build


def articulated_build():
    from pxr import Sdf, Usd, UsdPhysics
    layer = Sdf.Layer.CreateAnonymous('.usda')
    layer.ImportFromString(_original_chain())
    stage = Usd.Stage.Open(layer)
    root = stage.GetDefaultPrim().GetPath()
    for i in ROOT_LINKS:
        prim = stage.GetPrimAtPath(root.AppendChild(f'chain_{i:03}'))
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        prim.AddAppliedSchema('PhysxArticulationAPI')
        prim.CreateAttribute('physxArticulation:enabledSelfCollisions', Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute('physxArticulation:solverPositionIterationCount', Sdf.ValueTypeNames.Int).Set(64)
        prim.CreateAttribute('physxArticulation:solverVelocityIterationCount', Sdf.ValueTypeNames.Int).Set(1)
    for i in BOUNDARIES:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(root.AppendPath(f'chain_{i:03}/next_pin')))
        joint.CreateExcludeFromArticulationAttr(True)
    return layer.ExportToString()


CHAIN = replace(CHAIN, build=articulated_build)
OBJECTS = [CHAIN, DRIVE]
# The old observer only supports maximal-coordinate rigid-body tensors.
m._STEP.remove(base.observer.observe)


class ArticulatedTrace:
    def __init__(self):
        self.count = 0
        self.views = None

    def physics(self, env, dt):
        if self.views is None:
            import omni.physics.tensors as tensors
            self.sim = tensors.create_simulation_view('torch')
            self.sim.set_subspace_roots('/')
            self.views = [self.sim.create_articulation_view(f'/World/envs/env_0/chain/chain_{i:03}')
                for i in ROOT_LINKS]
            self.paths = [list(v.link_paths[0]) for v in self.views]
            assert [len(p) for p in self.paths] == [64, 64, 64, 20]
            self.names = [p.rsplit('/', 1)[-1] for paths in self.paths for p in paths]
            assert len(set(self.names)) == 212
            self.indices = {n: i for i, n in enumerate(self.names)}
            print('CHAIN_ARTICULATED_NATIVE', dict(link_counts=[len(p) for p in self.paths],
                masses=[m._np(v.get_masses()).tolist() for v in self.views],
                dof_paths=[v.dof_paths for v in self.views],
                dof_limits=[m._np(v.get_dof_limits()).tolist() for v in self.views],
                dof_stiffness=[m._np(v.get_dof_stiffnesses()).tolist() for v in self.views],
                dof_damping=[m._np(v.get_dof_dampings()).tolist() for v in self.views]), flush=True)
        if self.count in (0, 1, 2, 4, 8, 16, 32, 64, 120, 240, 300):
            poses = np.concatenate([m._np(v.get_link_transforms())[0] for v in self.views])
            velocities = np.concatenate([m._np(v.get_link_velocities())[0] for v in self.views])
            points = {n: poses[i, :3] for n, i in self.indices.items()}
            rotations = {n: m.quat_matrix(poses[i, 3:]) for n, i in self.indices.items()}
            pin_errors = []
            for i in range(106):
                a = f'chain_{i:03}'; b = f'chain_{(i+1)%106:03}'
                p = points[a] + rotations[a] @ np.array([.00635, 0., 0.])
                q = points[b] + rotations[b] @ np.array([-.00635, 0., 0.])
                pin_errors.append(float(np.linalg.norm(p-q)))
            bearing_errors = []
            for n in self.names:
                if '_roller_' not in n:
                    continue
                parent, side = n.split('_roller_')
                q = points[parent] + rotations[parent] @ np.array([-.00635 if side == 'a' else .00635, 0., 0.])
                bearing_errors.append(float(np.linalg.norm(points[n]-q)))
            print('CHAIN_ARTICULATED_TRACE', self.count, dict(time_s=self.count*dt,
                maximum_linear_speed_m_s=float(np.linalg.norm(velocities[:, :3], axis=1).max()),
                maximum_angular_speed_rad_s=float(np.linalg.norm(velocities[:, 3:], axis=1).max()),
                maximum_pin_error_m=max(pin_errors), maximum_bearing_error_m=max(bearing_errors),
                boundary_pin_errors_m=[pin_errors[i] for i in BOUNDARIES],
                bounds_m=[poses[:, :3].min(0).tolist(), poses[:, :3].max(0).tolist()]), flush=True)
        self.count += 1


trace = ArticulatedTrace()
on_physics_step(trace.physics)
