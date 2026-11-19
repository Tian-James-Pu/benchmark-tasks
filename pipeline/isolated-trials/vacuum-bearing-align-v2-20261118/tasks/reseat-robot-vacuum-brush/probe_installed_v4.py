"""Score-disabled preassembled fit diagnostic; not a repair demonstration.

Only the initial conditions place the roller and cover in the installed pose.
All subsequent motion is native physics; no state or joint is written.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
from dataclasses import replace
ROLLER=replace(ROLLER,pos=tuple(C+np.array([AX,0,AZ])),start_quat=(0.,0.,0.,1.))
COVER=replace(COVER,pos=tuple(C+np.array([AX,0,COVER_SEATED_Z])),start_quat=(0.,0.,0.,1.))
OBJECTS=[ROLLER if x.name=='roller' else COVER if x.name=='cover' else x for x in OBJECTS]
