"""Preassembled positive scoring control; not a repair demonstration."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_native_support_v2 import *
from support_hold import demo
OBJECTS=[replace(x,pos=tuple(C+np.array([AX,0,AZ])),start_quat=(0.,0.,0.,1.)) if x.name=='roller' else x for x in OBJECTS]
OBJECTS=[replace(x,pos=tuple(C+np.array([AX,0,COVER_SEATED_Z])),start_quat=(0.,0.,0.,1.)) if x.name=='cover' else x for x in OBJECTS]
