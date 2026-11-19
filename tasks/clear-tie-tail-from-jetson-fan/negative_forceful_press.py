"""Attempt forceful manipulation of the mounted fan; not a fracture model."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
def demo():
    return {'right':[
        Waypoint(2.,(.30,-.0777,.86),FINGER_CLOSED,'approach mounted fan',0.),
        Waypoint(1.5,(.30,-.0777,.779),FINGER_CLOSED,'press guarded assembly',0.),
        Waypoint(1.,(.32,-.0777,.779),FINGER_CLOSED,'sweep against mount',0.),
        Waypoint(1.,(.30,-.0777,.779),FINGER_CLOSED,'press again',0.),
        Waypoint(2.,(.24,-.25,.88),FINGER_OPEN,'withdraw',0.)]}
