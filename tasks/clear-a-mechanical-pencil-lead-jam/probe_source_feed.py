"""Final bounded source-corrected feed baseline. Still diagnostic score zero."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from probe_healthy_feed import *
from pencil_source_core import source_core
OBJECTS=[source_core(ORIGIN,fixed=True),lead(ORIGIN)]
