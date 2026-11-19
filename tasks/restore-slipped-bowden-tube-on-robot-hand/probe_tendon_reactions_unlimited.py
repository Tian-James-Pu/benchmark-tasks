"""Internal tendon forces across two free articulated bodies; not a task demo."""

import pathlib
import sys
from dataclasses import replace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from bowden import tendon_load
from physics_hooks import on_physics_step
from point_forces import apply_point_loads
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']
def configure_scene(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1. / 960.
    original_callback = scene.env_cfg_callback
    def callback(control_hz=None):
        original = original_callback(control_hz)
        def configure(cfg):
            cfg = original(cfg)
            cfg.sim.gravity = (0., 0., 0.)
            return cfg
        return configure
    scene.env_cfg_callback = callback
m._Finder.targets['yam_arena.scene'] = configure_scene

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Measure internal tendon reactions in an isolated articulation.'
SUCCESS_HOLD_S = 0.
MASS, LENGTH, WIDTH = .02, .10, .01
IZ = MASS * (LENGTH**2 + WIDTH**2) / 12.
IS = MASS * WIDTH**2 / 6.
INERTIAS = np.array([[IS, IZ, IZ], [IZ, IS, IZ]])
POINTS = np.array([[-.04, 0., 0.], [.05, .09, 0.]])
REST_LENGTH, STIFFNESS, DAMPING = .11, 5., .02
looks = {'base': m.Look(color=m.rgb(.2, .5, .7)),
         'child': m.Look(color=m.rgb(.8, .5, .2))}
child = m.hinge('child', 'Z', (.05, 0., 0.), -float('inf'), float('inf'),
    [('bar', m.box((WIDTH, LENGTH, WIDTH), (.05, .05, 0.)), 'child')],
    [m.col_box('bar_c', (WIDTH, LENGTH, WIDTH), (.05, .05, 0.))],
    mass=MASS, com=(.05, .05, 0.), inertia=tuple(INERTIAS[1]),
    spring=0., damping=0., friction=0., armature=0., gravity=True)
assembly = m.mechanism('assembly', (.30, 0., TOP+.25), [child], looks,
    frame_parts=[('bar', m.box((LENGTH, WIDTH, WIDTH)), 'base')],
    frame_colliders=[m.col_box('bar_c', (LENGTH, WIDTH, WIDTH))],
    free_mass=MASS, frame_inertia=tuple(INERTIAS[0]), solver_iters=64)
original_build = assembly.build
def build():
    return original_build().replace(
        '"PhysicsRigidBodyAPI", "PhysicsMassAPI"]',
        '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]').replace(
        'float physics:mass =',
        'float physxRigidBody:linearDamping = 0\n'
        '        float physxRigidBody:angularDamping = 0\n        float physics:mass =').replace(
        'solverVelocityIterationCount = 1', 'solverVelocityIterationCount = 8')
OBJECTS = [replace(assembly, build=build)]

class Trial:
    active = False
    def snapshot(self, env):
        asset = env.unwrapped.scene['assembly']
        ids = [asset.body_names.index(n) for n in ('frame', 'child')]
        poses = m._np(asset.data.body_link_pose_w)[0, ids].copy()
        centers = m._np(asset.data.body_com_pos_w)[0, ids].copy()
        velocities = m._np(asset.data.body_com_vel_w)[0, ids].copy()
        origin = m._np(env.unwrapped.scene.env_origins[0])
        poses[:, :3] -= origin
        centers -= origin
        rotations = np.array([m.quat_matrix(p[3:]) for p in poses])
        points = poses[:, :3] + np.einsum('nij,nj->ni', rotations, POINTS)
        point_velocities = velocities[:, :3] + np.cross(
            velocities[:, 3:], points-centers)
        load = tendon_load(points, point_velocities, REST_LENGTH, STIFFNESS, DAMPING)
        com = centers.mean(axis=0)
        angular = sum((rotations[i] @ np.diag(INERTIAS[i]) @ rotations[i].T
                       @ velocities[i, 3:] +
                       np.cross(centers[i]-com, MASS*velocities[i, :3])) for i in range(2))
        kinetic = .5*MASS*float(np.sum(velocities[:, :3]**2)) + sum(
            .5*float(velocities[i, 3:] @ rotations[i] @ np.diag(INERTIAS[i])
                     @ rotations[i].T @ velocities[i, 3:]) for i in range(2))
        return points, load, com, MASS*velocities[:, :3].sum(axis=0), angular, kinetic
    def reset(self, env=None):
        self.active = env is not None
        self.steps = self.calls = 0
        if self.active:
            _, load, self.start_com, _, _, kinetic = self.snapshot(env)
            self.start_energy = load.energy + kinetic
    def physics(self, env, dt):
        loads = []
        if self.active:
            points, load, *_ = self.snapshot(env)
            loads = [('assembly', body, force, point) for body, force, point in
                     zip(('frame', 'child'), load.forces, points, strict=True)]
            self.calls += 1
        apply_point_loads(env, ['assembly'], loads)
    def step(self, env):
        self.steps += 1
        _, load, com, momentum, angular, kinetic = self.snapshot(env)
        print('TENDON_REACTIONS', self.steps, {
            'physics_calls': self.calls, 'com_drift_m': (com-self.start_com).tolist(),
            'linear_momentum_kg_m_s': momentum.tolist(),
            'angular_momentum_kg_m2_s': angular.tolist(),
            'energy_j': kinetic+load.energy, 'initial_energy_j': self.start_energy,
            'tension_n': load.tension, 'length_m': load.length,
            'joint_deg': m.state(env, 'assembly').deg('child')}, flush=True)

trial = Trial()
on_physics_step(trial.physics)
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right': [Waypoint(3., (.20, -.25, TOP+.22), -.04695, 'idle', 0.)]}
