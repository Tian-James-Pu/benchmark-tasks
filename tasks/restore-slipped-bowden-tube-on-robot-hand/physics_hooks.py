"""Task-local callbacks for continuous mechanical forces at the physics rate.

These callbacks implement scene mechanics, not the robot's action policy.
They run before force buffers are written for each physics step, including
reset settling. No shared simulator files are modified.
"""

import sys

import mechkit as m

_CALLBACKS = []
_original_patch = m._Finder.targets['yam_arena.inspect_embodiment']


def on_physics_step(function):
    _CALLBACKS.append(function)
    return function


def _patch(module):
    _original_patch(module)
    cls = module.YamArenaEmbodiment
    if getattr(cls, '_bowden_physics_patch_installed', False):
        return
    ensure_sim = cls._ensure_sim

    def ensure_with_forces(self, *args, **kwargs):
        result = ensure_sim(self, *args, **kwargs)
        base = self._env.unwrapped
        if getattr(base, '_bowden_physics_hook_installed', False):
            return result
        if base._physics_handles_decimation:
            raise RuntimeError('Continuous force callbacks require individual physics steps')
        apply_action = base.action_manager.apply_action
        environment = self._env

        def apply_with_forces():
            apply_action()
            for function in _CALLBACKS:
                function(environment, base.physics_dt)

        base.action_manager.apply_action = apply_with_forces
        base._bowden_physics_hook_installed = True
        return result

    cls._ensure_sim = ensure_with_forces
    cls._bowden_physics_patch_installed = True


m._Finder.targets['yam_arena.inspect_embodiment'] = _patch
if 'yam_arena.inspect_embodiment' in sys.modules:
    _patch(sys.modules['yam_arena.inspect_embodiment'])
