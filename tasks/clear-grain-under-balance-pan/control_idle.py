"""Do nothing: the original fault must not earn credit."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from yam_arena.demo import Waypoint

def demo():return {side:[Waypoint(6.,(.20,y,TOP+.22),-.04695,'idle',0.)] for side,y in [('left',.25),('right',-.25)]}
