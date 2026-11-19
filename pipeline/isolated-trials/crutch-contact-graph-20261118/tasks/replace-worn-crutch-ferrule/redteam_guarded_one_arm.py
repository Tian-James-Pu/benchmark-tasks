"""Full-scene validation negative; identical hardware and guarded scoring."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
SUCCESS_HOLD_S=0.
def success(objects):return False
def demo():return {'left':[],'right':controller.demo()['right'][:8]}
