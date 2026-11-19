"""Physics-rate evaluation of the prototype's passive bistable retainer.

The two quadratic wells have equal energy at half travel. Choosing the nearer
well gives U(q)=0.5*k*min(q*q,(q-travel)**2). Stiffness, damping, effort and travel
remain properties of the authored joint. This is an approximate detent, not a
measured manufacturer force curve, and it never changes geometry or poses.
"""

import mechkit as m
from physics_hooks import on_physics_step


class PassiveDetent:
    def __init__(self, part='housing', joint='tpa_joint', travel=.005, stiffness=8000.):
        self.part, self.joint = part, joint
        self.travel, self.stiffness = travel, stiffness
        self.environment = None
        self.calls = 0
        self.baseline_calls = 0
        on_physics_step(self.physics)
        m.on_reset(self.reset)

    def physics(self, env, dt):
        import torch
        base = env.unwrapped
        if self.environment is not base:
            self.environment = base
            self.asset = base.scene[self.part]
            self.index = self.asset.joint_names.index(self.joint)
            self.previous_target = torch.zeros((1, 1), device=self.asset.device)
            self.switches = torch.zeros((), device=self.asset.device, dtype=torch.int64)
            self.max_energy_jump = torch.zeros((), device=self.asset.device)
            self.signed_energy_jump = torch.zeros((), device=self.asset.device)
        q = self.asset.data.joint_pos.torch[:, self.index:self.index+1]
        target = torch.where(q < self.travel/2., 0., self.travel)
        changed = target != self.previous_target
        jump = .5*self.stiffness*((q-target)**2-(q-self.previous_target)**2)
        jump = torch.where(changed, jump, 0.)
        self.switches += changed.sum()
        self.max_energy_jump = torch.maximum(self.max_energy_jump, jump.abs().max())
        self.signed_energy_jump += jump.sum()
        self.asset.set_joint_position_target(target, joint_ids=[self.index])
        self.previous_target = target
        self.calls += 1

    def reset(self, env=None):
        self.baseline_calls = self.calls
        if self.environment is not None:
            self.switches.zero_()
            self.max_energy_jump.zero_()
            self.signed_energy_jump.zero_()

    def diagnostics(self):
        return {'physics_calls': self.calls-self.baseline_calls,
                'switches': int(self.switches.item()),
                'max_discrete_well_switch_energy_jump_j': float(self.max_energy_jump.item()),
                'signed_discrete_well_switch_energy_jump_j': float(self.signed_energy_jump.item())}
