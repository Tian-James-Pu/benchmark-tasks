"""Simulation-only nonfinger support control using the canonical scorer."""
import pathlib,sys,math
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
import robot_contact_gate as contact
from closedloop import placeholder_demo
demo=placeholder_demo
def demo_script(ctx):
    arm=ctx.right;O=-.04695
    hover=(.554,0.,TOP+.14)
    yield from arm.move(hover,2.5,O,pitch=math.pi/2,yaw=0.)
    yield from arm.hold(.5)
    for _ in arm.move((.554,0.,TOP+.06),3.,O,rot=arm.rot.copy()):
        yield
        if (not arm.offline and not contact.LAST['direct_clear']
                and contact.LAST.get('net_max_n',0.)>.02):
            print('PALM_FORCE_STOP',{'tcp':arm.measured_pos.tolist(),
                'finger_clear':monitor.last.get('hands_clear'),
                'contact':contact.LAST},flush=True)
            break
    yield from arm.hold(.5)
    yield from arm.move(hover,2.,O,rot=arm.rot.copy())
    yield from arm.home(2.)
