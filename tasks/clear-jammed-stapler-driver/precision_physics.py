"""Task-local 480Hz integration for millimetre wire contacts;20Hz control."""
import sys
import mechkit as m
_original=m._Finder.targets['yam_arena.scene']
def _patch(module):
    _original(module)
    if getattr(module,'_staple_precision',False):return
    old=module.env_cfg_callback
    def callback(control_hz=None):
        original=old(control_hz)
        def configure(cfg):
            original(cfg)
            cfg.sim.dt=1/480
            cfg.decimation=round(480/(control_hz or 20))
            cfg.sim.render_interval=cfg.decimation
            return cfg
        return configure
    module.env_cfg_callback=callback
    module._staple_precision=True
m._Finder.targets['yam_arena.scene']=_patch
if 'yam_arena.scene' in sys.modules:_patch(sys.modules['yam_arena.scene'])
