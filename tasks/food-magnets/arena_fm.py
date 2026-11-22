"""Bridge between task.py and the pinned YAM/Arena loader (yam-isaac-lab-arena 0059d02a), on the pattern of
build/ring-on-finger/arena_ring.py: ``Box`` subclasses in task.OBJECTS and an import hook wrapping
``yam_arena.scene.task_object`` / ``object_states`` so that

- each magnet (strawberry, cookie, donut, plain) is one dynamic rigid body authored by scene.author_magnet,
- ``card`` is the note's rigid plate (scene.author_card),
- ``lid`` is the hinged lid; its spawn also authors the static board and box body and the hinge (scene.author_lid).

MagnetForces: before every write of the asset buffers to the simulator (Isaac Lab writes external wrenches before each
physics step it runs), each magnet gets the attraction F0 / (1 + g / 1 mm)^2 along +x at its back-face disc, from its
current pose (geometry.pull). This is the card's "Real physics: each magnet is pulled toward the steel by a force that
falls off with its back-face gap". It never writes a pose or velocity, and nothing here reaches a policy.
"""
import importlib.abc
import importlib.util
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

import geometry as G

SPECS = {}
TRIAL = dict(seed=int(os.environ.get('ROBOCURVE_FM_SEED', '0')), tracker=None, audit={}, draw=None, magnets=None)
MAGNETS = ('strawberry', 'cookie', 'donut', 'plain')


@dataclass(frozen=True)
class Magnet(Box):
    @property
    def free(self):
        return False   # no yam jitter: the draw is authored


@dataclass(frozen=True)
class NoteCard(Box):
    @property
    def free(self):
        return False


@dataclass(frozen=True)
class Lid(Box):
    @property
    def free(self):
        return False


@dataclass(frozen=True)
class BodyState(ObjectState):
    rot: np.ndarray = field(default_factory=lambda: np.eye(3))


def _arr(value):
    return getattr(value, 'torch', value).detach().cpu().numpy()


def _base(env):
    return env.unwrapped if hasattr(env, 'unwrapped') else env


def _rigid_object(spec, author, rot_xyzw=(0., 0., 0., 1.)):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        from isaaclab.sim.utils.stage import get_current_stage
        stage = get_current_stage()
        t = tuple(spec.pos) if translation is None else tuple(float(v) for v in translation)
        TRIAL['audit'].setdefault('spawn_translation', {})[spec.name] = list(t)
        return author(stage, str(prim_path), t)

    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object = clone(spawn)

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=SpawnCfg(),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in rot_xyzw)))


def magnet_asset(spec):
    from scene import author_magnet
    return _rigid_object(spec, author_magnet(spec.name))


def card_asset(spec):
    from scene import author_card
    return _rigid_object(spec, author_card)


def lid_asset(spec):
    from scene import author_lid
    w, x, y, z = G.matrix_to_quat_wxyz(G.LID_OPEN_ROT)
    return _rigid_object(spec, author_lid(lambda: TRIAL['draw']['box_y']), rot_xyzw=(x, y, z, w))


# -- state readers (scorer, diagnostics and the magnetic force; never a policy) -----------------------------

def body_pose(env, name):
    base = _base(env)
    obj = base.scene[name]
    origin = _arr(base.scene.env_origins)[0]
    pose = _arr(obj.data.root_link_pose_w)[0].astype(float)
    return pose[:3] - origin, G.quat_xyzw_to_matrix(pose[3:7])


def body_velocity(env, name):
    base = _base(env)
    obj = base.scene[name]
    v = _arr(obj.data.root_link_vel_w)[0].astype(float) if hasattr(obj.data, 'root_link_vel_w') else np.zeros(6)
    return v[:3], v[3:6]


def _rigid_state(env, name):
    p, r = body_pose(env, name)
    v, w = body_velocity(env, name)
    qw, qx, qy, qz = G.matrix_to_quat_wxyz(r)
    return BodyState(pos=p, quat_xyzw=np.array([qx, qy, qz, qw]), lin_vel=v, ang_vel=w, low=p - .01, high=p + .01, rot=r)


_HULLS = {}


def _gripper_link_hulls():
    if not _HULLS:
        from yam_arena.analysis import link_hulls
        _HULLS.update({k: v for k, v in link_hulls().items() if k in ('gripper', 'tip_left', 'tip_right')})
    return _HULLS


def world_hulls(link_poses):
    from yam_arena.analysis import Hull
    out = []
    for name, pose in link_poses.items():
        r, p = pose[:3, :3], pose[:3, 3]
        for h in _gripper_link_hulls().get(name, []):
            n_w = h.planes[:, :3] @ r.T
            out.append((name, Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def robot_hulls(env):
    """The grippers' collision hulls (links gripper, tip_left, tip_right of yam_arena.analysis.link_hulls, read once)
    moved to the simulator's link poses."""
    base = _base(env)
    origin = _arr(base.scene.env_origins)[0]
    out = []
    for arm in ('left_robot', 'right_robot'):
        robot = base.scene[arm]
        poses = _arr(robot.data.body_link_pose_w)[0]
        mats = {}
        for i, name in enumerate(robot.body_names):
            t = np.eye(4)
            t[:3, :3] = G.quat_xyzw_to_matrix(poses[i, 3:7])
            t[:3, 3] = poses[i, :3] - origin
            mats[name] = t
        out += world_hulls(mats)
    return out


class MagnetForces:
    """Per physics write: each magnet's attraction to the steel (instantaneous wrench, world frame, at the disc)."""

    def __init__(self, env):
        import torch
        self.torch = torch
        self.base = _base(env)
        self.calls = 0
        self.last = {}
        self.objs = {n: self.base.scene[n] for n in MAGNETS}
        for name, obj in self.objs.items():
            original = obj.write_data_to_sim

            def write(name=name, obj=obj, original=original):
                self.apply(name, obj)
                return original()

            obj.write_data_to_sim = write

    def apply(self, name, obj):
        torch = self.torch
        origin = _arr(self.base.scene.env_origins)[0]
        pose = _arr(obj.data.root_link_pose_w)[0].astype(float)
        pos, rot = pose[:3] - origin, G.quat_xyzw_to_matrix(pose[3:7])
        f, c, g = G.pull(name, pos, rot)
        self.calls += 1
        self.last[name] = dict(force_N=f, gap_mm=g / G.MM)
        dev = self.base.device
        forces = torch.tensor([[[f, 0.0, 0.0]]], dtype=torch.float32, device=dev)
        torques = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
        positions = torch.tensor([[(c + origin).tolist()]], dtype=torch.float32, device=dev)
        obj.instantaneous_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torques, positions=positions,
                                                                       is_global=True)


def install_magnets(env):
    if TRIAL.get('magnets') is None:
        TRIAL['magnets'] = MagnetForces(env)
    return TRIAL['magnets']


# -- patches ------------------------------------------------------------------------------------------------

def _patch_scene(scene):
    if getattr(scene, '_fm_installed', False):
        return
    original_object, original_states = scene.task_object, scene.object_states
    makers = {Magnet: magnet_asset, NoteCard: card_asset, Lid: lid_asset}

    def task_object(spec):
        for kind, make in makers.items():
            if isinstance(spec, kind):
                spec._check()
                SPECS[spec.name] = spec
                return make(spec)
        return original_object(spec)

    def object_states(env, names):
        ordinary = [n for n in names if n not in SPECS]
        result = original_states(env, ordinary) if ordinary else {}
        for n in names:
            if n in SPECS:
                result[n] = _rigid_state(env, n)
        return {n: result[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._fm_installed = True


def _patch_embodiment(module):
    cls = module.YamArenaEmbodiment
    if getattr(cls, '_fm_installed', False):
        return
    original_reset, original_ensure = cls.reset, cls._ensure_sim

    def _ensure_sim(self, goal):
        original_ensure(self, goal)
        install_magnets(self._env)

    def reset(self, scene, *, seed=None):
        obs = original_reset(self, scene, seed=seed)
        TRIAL['tracker'] = None
        return obs

    cls._ensure_sim = _ensure_sim
    cls.reset = reset
    cls._fm_installed = True


class _Patcher(importlib.abc.MetaPathFinder):
    patches = {'yam_arena.scene': _patch_scene, 'yam_arena.inspect_embodiment': _patch_embodiment}

    def find_spec(self, fullname, path=None, target=None):
        if fullname not in self.patches:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        original, patch = spec.loader.exec_module, self.patches[fullname]

        def execute(module):
            original(module)
            patch(module)

        spec.loader.exec_module = execute
        return spec


def install():
    for name, patch in _Patcher.patches.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Patcher) for f in sys.meta_path):
        sys.meta_path.insert(0, _Patcher())


install()
