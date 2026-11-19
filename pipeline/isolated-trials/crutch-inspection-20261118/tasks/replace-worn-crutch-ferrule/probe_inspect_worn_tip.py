"""Existing-camera observability check; identical fixed objects, no repair action."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
import json
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena import ik,station

GOAL=json.loads((pathlib.Path(__file__).parent/'inspection_goal.json').read_text())
START=ik.tcp_pose(np.array(READY),station.BASES['right'])
TARGET=(np.array(GOAL['tcp']),np.array(GOAL['rotation']))
INSTRUCTION='Inspect the worn foot without touching it.'
SUCCESS_HOLD_S=0.
success=lambda objects:False

def demo():
    return {'left':[], 'right':[
        c.point(2.,FINGER_OPEN,'allow original scene to settle',lambda:START),
        c.point(3.,FINGER_OPEN,'inspect worn underside without contact',lambda:TARGET),
        c.point(1.,FINGER_OPEN,'observe exposed tube end',lambda:TARGET)]}

def inspection_observe(env):
    if c.COUNT%10:return
    q=m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0,:6]
    p,r=ik.tcp_pose(q,station.BASES['right']);camera=station.wrist_camera('right')
    from yam_arena.constants import TCP_OFFSET
    cp=p+r@(np.array(camera.pos)-np.array(TCP_OFFSET))
    old=m.state(env,'old_ferrule');shaft=m.state(env,'crutch')
    witness=result.contacts
    prop=witness.prop_measure(env.unwrapped.physics_dt)
    robot=np.array([x in witness.robot_bodies for x in witness.prop_filters])
    print('WORN_TIP_INSPECTION',c.COUNT,json.dumps({'camera_position':cp.tolist(),'camera_rotation':(r@Rotation.from_quat(camera.rot_xyzw).as_matrix()).tolist(),'crutch_position':shaft.pos.tolist(),'crutch_rotation':shaft.rotation().tolist(),'old_position':old.pos.tolist(),'old_rotation':old.rotation().tolist(),'assembly_witness':witness.last,'all_prop_robot_contact_n':float(prop[:,robot].sum()),'whole_arm':audit.last}),flush=True)
m.on_step(inspection_observe)
