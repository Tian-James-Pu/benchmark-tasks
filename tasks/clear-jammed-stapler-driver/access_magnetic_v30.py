"""Matched axial reset with continuous reference-magnet retention approximation."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from access_axial_v30 import *
import magnetic_return_v21
_base_demo=demo_script
def demo_script(ctx):
 yield from _base_demo(ctx)
 yield from ctx.right.hold(3.)
