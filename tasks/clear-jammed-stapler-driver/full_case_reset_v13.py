"""Fixed mechanical targets initialized natively before settling, service gate."""
import sys,json
import mechkit as m

def snapshot(env,label):
 a=env.scene['stapler'];b=env.scene['bent_staple']
 print('FULL_CASE '+json.dumps({'label':label,'joint_names':a.joint_names,
 'q':a.data.joint_pos[0].detach().cpu().numpy().tolist(),
 'bodies':a.data.body_link_pose_w[0].detach().cpu().numpy().tolist(),
 'wire':b.data.root_link_pose_w[0].detach().cpu().numpy().tolist()}),flush=True)

def initialize(env,env_ids):
 import torch
 a=env.scene['stapler'];q=torch.zeros_like(a.data.default_joint_pos.torch[env_ids]);target=q.clone()
 for name,start,rest in [('driver_joint',0.,-.010),('cover_joint',.150,.150),('follower_joint',-.198,-.208)]:
  j=a.joint_names.index(name);q[:,j]=start;target[:,j]=rest
 a.write_joint_position_to_sim_index(position=q,env_ids=env_ids)
 a.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q),env_ids=env_ids)
 a.actuators.target_command.set_position_index(value=target,env_ids=env_ids)
 a.actuators.target_command.set_velocity_index(value=torch.zeros_like(q),env_ids=env_ids)
 snapshot(env,'pre_settle')
_original=m._Finder.targets['yam_arena.scene']
def _patch(module):
 _original(module);old=module.env_cfg_callback
 def callback(control_hz=None):
  original=old(control_hz)
  def configure(cfg):
   from isaaclab.managers import EventTermCfg
   original(cfg);cfg.sim.dt=1/480;cfg.decimation=round(480/(control_hz or 20));cfg.sim.render_interval=cfg.decimation
   cfg.events.staple_reset=EventTermCfg(func=initialize,mode='reset');return cfg
  return configure
 module.env_cfg_callback=callback
m._Finder.targets['yam_arena.scene']=_patch
if 'yam_arena.scene' in sys.modules:_patch(sys.modules['yam_arena.scene'])
