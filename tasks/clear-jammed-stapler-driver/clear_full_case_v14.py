"""Matched no-wire control; same frame, rails and fixed mechanical targets."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from full_case_v14 import *
OBJECTS=[x for x in OBJECTS if x.name!="bent_staple"]
