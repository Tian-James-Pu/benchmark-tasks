"""Physics rate and refreshed reset images for the live physical-force LCD."""
import mechkit as m
_scene=m._Finder.targets['yam_arena.scene']
def scene(module):
 _scene(module);module.PHYSICS_DT=1/960.
m._Finder.targets['yam_arena.scene']=scene
_embodiment=m._Finder.targets['yam_arena.inspect_embodiment']
def embodiment(module):
 _embodiment(module)
 cls=module.YamArenaEmbodiment;original=cls.reset
 def reset(self,*args,**kwargs):
  original(self,*args,**kwargs)
  # Electronics' on_reset callback has initialized pixels from the settled
  # sensor force. Refresh images only; no extra physics or robot actions.
  from inspect_robots_yam.config import DEFAULT_CAMERAS
  for _ in range(16):
   self._base.sim.render()
   self._base.sim.render_context.reset_scene_state_cadence()
   for name in DEFAULT_CAMERAS:
    # Camera.update(force_recompute=True) still skips a false outdated mask.
    # Reset only the sensor bookkeeping; neither objects nor physics advance.
    self._base.scene[name].reset()
    self._base.scene[name].update(0.,force_recompute=True)
  return self._observation()
 cls.reset=reset
m._Finder.targets['yam_arena.inspect_embodiment']=embodiment

# Task files may load after the embodiment module. Apply the same wrapper then.
import sys
if "yam_arena.inspect_embodiment" in sys.modules:
 embodiment(sys.modules["yam_arena.inspect_embodiment"])


def refresh_lcd_pixels(env):
 """Refresh visual edits after electronics paint; never advance physics."""
 from inspect_robots_yam.config import DEFAULT_CAMERAS
 base=env.unwrapped
 base.sim.render_context.reset_scene_state_cadence()
 base.sim.render()
 for name in DEFAULT_CAMERAS:
  base.scene[name].reset()
  base.scene[name].update(0.,force_recompute=True)
