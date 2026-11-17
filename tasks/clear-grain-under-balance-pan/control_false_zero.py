"""Physically power-cycle the faulty balance, then check the real 100 g mass.

This control uses the original fault and scorer. It must not earn repair credit
merely because startup zero hides the empty-load error.
"""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from hardware import panel_z
from motion import demo as original_demo
from yam_arena.demo import Waypoint as W

def demo():
    O=-.04695;x=.384;y=-.045
    z=TOP+panel_z(y)+.0011+.01284-.00028
    right=[W(1.5,(x,y,TOP+.10),0.,'approach power key',0.),
        W(1.5,(x,y,z),0.,'press power key',0.),
        W(1.,(x,y,z),0.,'hold key',0.),
        W(1.5,(x,y,TOP+.10),O,'release key',0.),
        W(2.,(.20,-.25,TOP+.22),O,'observe',0.)]
    right+=original_demo()['right'][1:]
    return {'left':[W(41.,(.20,.25,TOP+.22),O,'leave fault untouched',0.)], 'right':right}

def log_power(env):
    if monitor.steps%20==0:
        print('POWER_CONTROL',monitor.steps,{'on':monitor.e.on,
            'startup':monitor.e.startup,'epoch':monitor.e.epoch,
            'key_displacement_m':m.state(env,'balance').joint('power')},flush=True)
m.on_step(log_power)
