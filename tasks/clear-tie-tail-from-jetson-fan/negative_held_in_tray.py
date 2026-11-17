"""Attempt to keep gripping the offcut inside the tray; must not earn placement."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import task as base
from task import *
def demo():
    motion=base.demo()['right'][:6]
    motion += [Waypoint(1.,(.32,-.26,TOP+.004),FINGER_CLOSED,'retain grip in tray',-.385),
               Waypoint(12.,(.32,-.26,TOP+.004),FINGER_CLOSED,'hold without release',-.385)]
    return {'right':motion}
