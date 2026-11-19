"""Engineering fixture only: reciprocal ideal transmission, no task credit.

The arms remain at READY. Bounded external joint efforts calibrate the native
constraint; these forces are instrumentation and are not a robot demonstration.
"""
from pathlib import Path
import sys
import json
import math
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import mechkit as m
import closedloop as cl

SCENE='station'
INSTRUCTION='Observe an isolated bench-vise transmission calibration; the robot remains stationary.'
SUCCESS_HOLD_S=0.
JOINTS=['joint_slidingJaw_01_joint_00','joint_screw_01_joint_00','joint_handle_01_joint_00']
OBJECTS=[m.Part('vise',pos=(.28,-.05,.7475),size=(.325,.233,.181),role='jointed',yaw_deg=-90.,
                build=lambda:(HERE/'vise-passive-component.usda').read_text(),
                drives=tuple((name,(0.,0.,5.,0.,0.)) for name in JOINTS))]


class Probe:
    def reset(self,env=None):
        self.step=0
    def observe(self,env):
        import torch
        self.step+=1
        asset=env.unwrapped.scene['vise']
        q=asset.data.joint_pos[0].detach().cpu().numpy()
        v=asset.data.joint_vel[0].detach().cpu().numpy()
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
            raise RuntimeError('VISE_INVALID: non-finite native joint state; no calibration evidence')
        indices={name:asset.joint_names.index(name) for name in JOINTS}
        x=float(q[indices[JOINTS[0]]]);angle=float(q[indices[JOINTS[1]]])
        torque=.002 if 20<=self.step<25 else 0.
        force=.5 if 45<=self.step<50 else 0.
        print('VISE_PROBE '+json.dumps({'step':self.step,'joint_names':asset.joint_names,
              'q':q.tolist(),'v':v.tolist(),'coupling_error_m':x-.003*angle/(2*math.pi),
              'next_fixture_torque_Nm':torque,'next_fixture_axial_force_N':force}),flush=True)
        effort=torch.zeros_like(asset.data.joint_pos)
        effort[0,indices[JOINTS[0]]]=force
        effort[0,indices[JOINTS[1]]]=torque
        asset.set_joint_effort_target(effort)


PROBE=Probe();PROBE.reset()
m.on_reset(PROBE.reset);m.on_step(PROBE.observe)
demo=cl.placeholder_demo


def demo_script(ctx):
    yield from ctx.right.hold(4.)


def score(objects):return 0.
def success(objects):return False
