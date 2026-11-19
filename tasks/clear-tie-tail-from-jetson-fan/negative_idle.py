"""Same task and scorer; no recovery action, expected score 0."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
def demo():
    return {'right':[Waypoint(30.,(.20,-.25,TOP+.15),FINGER_OPEN,'leave obstruction untouched',0.)]}
