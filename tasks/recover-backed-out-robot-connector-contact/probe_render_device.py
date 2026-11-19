"""Score-disabled renderer device diagnostic; stock station and robot."""
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m

def patch_launcher(module):
    original = module.AppLauncher.__init__
    def init(self, *args, **kwargs):
        kwargs["multi_gpu"] = False
        print("RENDER_DEVICE_DIAGNOSTIC", {"device": kwargs.get("device"), "multi_gpu": False}, flush=True)
        return original(self, *args, **kwargs)
    module.AppLauncher.__init__ = init

_module = "isaaclab.app.app_launcher"
if _module in sys.modules:
    patch_launcher(sys.modules[_module])
else:
    m._Finder.targets[_module] = patch_launcher
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE = "station"
TOP = TABLE_TOP[SCENE]
OBJECTS = []
SUCCESS_HOLD_S = 0.
def score(objects): return 0.
def success(objects): return False
def demo():
    return {"right": [Waypoint(1., (.20, -.25, TOP+.22), -.04695, "renderer diagnostic", 0.)]}
