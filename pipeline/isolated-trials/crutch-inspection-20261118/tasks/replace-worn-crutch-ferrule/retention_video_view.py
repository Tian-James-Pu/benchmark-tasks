"""Supplementary fixed recording camera; no collider or policy observation changes.

The standard three policy cameras and their physical housings remain in place.
This adds only a USD imaging sensor beyond the bench, to record the foot below
both grippers during retention. No mechanics or object state is written.
"""
import os,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import mechkit as m

def patch(module):
    old=module.BimanualYamEmbodiment._camera_cfg
    def cameras(self,height,width,overview):
        cfg=old(self,height,width,overview)
        pos=np.array([1.10,-.02,.91]);target=np.array([.12,.18,.84])
        z=pos-target;z/=np.linalg.norm(z);x=np.cross([0.,0.,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
        quat=tuple(Rotation.from_matrix(np.column_stack([x,y,z])).as_quat())
        cam=module.CameraCfg(prim_path='{ENV_REGEX_NS}/RetentionAuditCam',height=height,width=width,data_types=['rgb'],
            spawn=module.sim_utils.PinholeCameraCfg(focal_length=28.,clipping_range=(.03,20.)),
            offset=module.CameraCfg.OffsetCfg(pos=tuple(pos),rot=quat,convention='opengl'),renderer_cfg=module.RENDERER)
        return module.make_configclass('FerruleAuditCameraCfg',[('retention_audit_cam',module.CameraCfg,cam)],bases=(type(cfg),))()
    module.BimanualYamEmbodiment._camera_cfg=cameras
m._Finder.targets['yam_arena.embodiment']=patch
if 'yam_arena.embodiment' in sys.modules:patch(sys.modules['yam_arena.embodiment'])
COUNT=0
OUTPUT=Path(os.environ.get('FERRULE_AUDIT_IMAGES','/tmp/ferrule-retention-audit'))
def reset(env=None):
    global COUNT
    COUNT=0
    OUTPUT.mkdir(parents=True,exist_ok=True)
def record(env):
    global COUNT
    COUNT+=1
    if COUNT>=1:
        from PIL import Image
        frame=m._np(env.unwrapped.scene['retention_audit_cam'].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
        Image.fromarray(frame).save(OUTPUT/f'{COUNT:05d}.png')
m.on_reset(reset);m.on_step(record)
