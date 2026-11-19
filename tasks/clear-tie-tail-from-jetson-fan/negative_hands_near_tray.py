"""Release normally, then keep the empty fingers within the 20 mm clearance."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import task as base
from task import *
def demo():
    motion=base.demo()['right'][:7]
    motion += [Waypoint(1.,(.32,-.26,TOP+.037),FINGER_OPEN,'let released strip settle',-.385),
               Waypoint(1.,(.32,-.26,TOP+.020),FINGER_CLOSED,'keep empty hand too close',-.385),
               Waypoint(10.,(.32,-.26,TOP+.020),FINGER_CLOSED,'wait without clearing fingers',-.385)]
    return {'right':motion}
def observe(env):
    if base.monitor.steps%20:return
    state=m.state(env,'tail');points=shape.v@state.rotation().T+state.pos-TRAY
    print('CLEARANCE_OBSERVATION',base.monitor.steps,{'speed_m_s':float(np.linalg.norm(state.lin_vel)),
        'tray_low':points.min(0).tolist(),'tray_high':points.max(0).tolist(),
        'contained_without_release_gate':contained(state)},flush=True)
m.on_step(observe)
