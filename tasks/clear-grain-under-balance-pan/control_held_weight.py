"""Repair the balance, then keep holding the check weight during measurement."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from repair_motion import demo as repair_demo
from yam_arena.demo import Waypoint as W

def demo():
    plan=repair_demo();right=plan['right'][:7];last=right[-1]
    right.append(W(15.,last.pos,last.finger,'hold weight on pan without release',last.yaw))
    return {'left':plan['left'],'right':right}
