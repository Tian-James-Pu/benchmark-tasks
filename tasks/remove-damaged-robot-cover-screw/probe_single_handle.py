"""Single original-handle pickup preserving passive jaw opening; zero score."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from probe_pickup_contact import *

def demo():
    return {'right':[
        Waypoint(1.5,(.215,-.065,TOP+.100),-.010,'hover one handle'),
        Waypoint(1.5,(.215,-.065,TOP+.012),-.010,'straddle one handle'),
        Waypoint(2.,(.215,-.065,TOP+.012),-.001,'grasp one original handle'),
        Waypoint(2.,(.215,-.065,TOP+.130),-.001,'lift keeping jaws open'),
        Waypoint(2.,(.215,-.065,TOP+.130),-.001,'observe single handle grasp')
    ]}
