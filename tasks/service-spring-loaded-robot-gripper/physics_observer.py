"""Read-only physics-step observation hook for balance validation.

Hooks run after action application and before each 960 Hz integration.
No object poses, forces, constraints, or solver settings are changed.
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
    if getattr(cls, '_mechkit_physics_patch_installed', False):
        return
    ensure_sim = cls._ensure_sim

    def ensure_with_observer(self, *args, **kwargs):
        result = ensure_sim(self, *args, **kwargs)
        base = self._env.unwrapped
        if getattr(base, '_mechkit_physics_hook_installed', False):
            return result
        if base._physics_handles_decimation:
            raise RuntimeError('Physics observers require individual physics steps')
        apply_action = base.action_manager.apply_action
        environment = self._env

        def apply_with_observer():
            apply_action()
            for function in _CALLBACKS:
                function(environment, base.physics_dt)

        base.action_manager.apply_action = apply_with_observer
        base._mechkit_physics_hook_installed = True
        return result

    cls._ensure_sim = ensure_with_observer
    cls._mechkit_physics_patch_installed = True


m._Finder.targets['yam_arena.inspect_embodiment'] = _patch
if 'yam_arena.inspect_embodiment' in sys.modules:
    _patch(sys.modules['yam_arena.inspect_embodiment'])
