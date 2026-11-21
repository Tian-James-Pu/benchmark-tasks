"""closedloop: play a task file's closed-loop ``demo_script(ctx)`` (demokit's API) under the harness's own
``yam_scripted`` policy, so that the ordinary ``inspect-robots run --policy yam_scripted`` command plays it.

demokit (next to this file, unchanged) normally runs a script from its own runner. Here the ``yam_scripted``
policy's ``act`` is wrapped instead: at each control step it advances the script by one step and sends demokit's
arm commands. The script sees what demokit gives it (object poses, its own joints) through the live environment,
which an ``on_reset`` hook keeps. Nothing in the scene is moved or edited by this file.
"""

from __future__ import annotations

import os
import types

import demokit as d
import mechkit as m

_ENV: dict = {}


def _keep(env) -> None:
    _ENV["env"] = env


m.on_reset(_keep)


def install(script, names, hold_s: float = 3.0) -> None:
    """Have ``yam_scripted`` play ``script(ctx)``; ``names`` are the task's object names."""
    import yam_arena.scripted as scripted
    from inspect_robots.types import Action, ActionChunk

    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_closedloop", False):
        return
    reset0 = cls.reset

    def reset(self, scene):
        reset0(self, scene)
        self._cl, self._cl_end = None, None

    def act(self, obs):
        if self._cl is None:
            emb = types.SimpleNamespace(_env=_ENV["env"], task=types.SimpleNamespace(names=tuple(names)))
            ctx = d.Ctx(emb, obs, os.environ.get("TASK_DEMO", "full"))
            self._cl = (ctx, script(ctx))
        ctx, gen = self._cl
        ctx.obs, ctx.t = obs, float(obs.state_time)
        if self._cl_end is None:
            try:
                next(gen)
            except StopIteration:
                self._cl_end = ctx.t
                print(f"[closedloop] script ended at {ctx.t:.2f} s", flush=True)
        stop = self._cl_end is not None and ctx.t > self._cl_end + hold_s
        return ActionChunk(actions=[Action(data=ctx.action().data, meta={"request_stop": True} if stop else {})], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._closedloop = reset, act, True
