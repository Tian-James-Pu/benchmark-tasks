"""fastening: a push-to-start cordless screwdriver driving a receptacle's two captive 6-32 mounting screws into the
wall box's tapped screw bosses. Written for this task; it builds on mechkit (``mechkit.py`` next to it).

The driver is a free mechanism: its housing, and its spindle (bit holder and #2 Phillips bit) on a sprung slider along
its axis. Pressing the bit against something pushes the spindle back; past ``START`` the motor runs (the tool's
push-and-go mode), and it stops when the spindle springs out again.

A screw turns when the running bit sits in its head (on the screw's axis within ``BIT_R``, against the head within
``BIT_AX``, and in line within ``BIT_DEG``). A turning screw whose tip is inside its boss's hole catches its thread.
From then on the thread sets how far the strap can stand off the boss at that screw: the gap the screw caught at,
less one pitch per turn. Where the strap stands further off than that, the thread pulls it in (a stiff spring, applied
at every physics step, capped at ``F_MAX``); when the gap reaches zero the screw is home and its slip clutch stops it,
and from then on it holds the strap down with ``PRELOAD``. The thread only pulls: nothing in it pushes the strap out.
Screws cannot be backed out (the tool runs forward only).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as m

PITCH = 0.0254 / 32  # 6-32 UNC: 0.794 mm per turn
RPS = 5.0  # turns per second under load (Bosch GO: 0-360 rpm no load)
START = 0.0015  # m of spindle travel that starts the motor
BIT_R, BIT_AX, BIT_DEG = 0.0015, 0.0025, 12.0
HOLE_R = 0.0022  # the tip must be this close to the hole's axis, and past the boss's face, to catch its thread
K_THREAD, C_THREAD, F_MAX, PRELOAD = 1000.0, 3.0, 60.0, 15.0


class Screws:
    """The driver, both screws and their threads. ``screws``: per screw, (strap-back point, tip point, head-front
    point) in the device's authored frame, and the boss hole's axis point (station frame) and the wall face x."""

    def __init__(self, part: str, strap_pts, tip_pts, head_pts, holes, wall_x: float,
                 driver: str, bit_tip_local, spindle: str = "spindle"):
        self.part, self.driver, self.spindle = part, driver, spindle
        self.strap = np.asarray(strap_pts, float)
        self.tip = np.asarray(tip_pts, float)
        self.head = np.asarray(head_pts, float)
        self.holes = np.asarray(holes, float)
        self.wall_x = wall_x
        self.bit_tip = np.asarray(bit_tip_local, float)
        self._patched = False
        self.reset()

    # -- state ---------------------------------------------------------------------------------------------
    def reset(self, env=None):
        n = len(self.strap)
        self.turns = np.zeros(n)        # turns the screw has made
        self.caught = np.zeros(n, bool)  # its thread has caught in the boss
        self.allow = np.full(n, np.inf)  # the largest gap the thread allows (m)
        self.home = np.zeros(n, bool)
        self.running = False
        self.driving = -1
        if env is not None:
            self._patch(env)
            asset = env.unwrapped.scene[self.part]
            self.dev = int(asset.body_names.index("device"))

    def _device(self, env):
        P = __import__("conductors").poses(env, self.part)
        return P["device"]

    def gaps(self, R, p) -> np.ndarray:
        """Each screw's strap-back standoff from the wall face (m): positive is out from the wall."""
        return np.array([self.wall_x - (R @ s + p)[0] for s in self.strap])

    def tips_in_holes(self, R, p) -> np.ndarray:
        out = []
        for t, h in zip(self.tip, self.holes):
            w = R @ t + p
            out.append(bool(w[0] > self.wall_x + 0.002 and math.hypot(w[1] - h[1], w[2] - h[2]) < HOLE_R))
        return np.array(out)

    def bit(self, env):
        """(bit tip, bit axis) in the station frame, and the spindle's push-in (m)."""
        d = m.state(env, self.driver)
        push = -d.joint(self.spindle)
        tip = d.to_world(self.bit_tip + np.array([-push, 0.0, 0.0]))[0]
        return tip, d.rotation()[:, 0], push

    def on_head(self, R, p, tip, axis) -> int:
        """Which screw the bit sits in, or -1."""
        s_axis = R[:, 0]
        if np.degrees(np.arccos(np.clip(float(axis @ s_axis), -1.0, 1.0))) > BIT_DEG:
            return -1
        for i, h in enumerate(self.head):
            hw = R @ h + p
            d = tip - hw
            ax = float(d @ s_axis)
            rad = float(np.linalg.norm(d - ax * s_axis))
            if rad < BIT_R and -0.001 < ax < BIT_AX:
                return i
        return -1

    def step(self, env):
        """Once per control step: the motor, the screw it drives, and each thread's allowed gap."""
        dt = env.unwrapped.step_dt
        R, p = self._device(env)
        tip, axis, push = self.bit(env)
        self.running = push > START
        i = self.on_head(R, p, tip, axis) if self.running else -1
        self.driving = i
        if i >= 0 and not self.home[i]:
            turn = RPS * dt
            self.turns[i] += turn
            g = self.gaps(R, p)[i]
            if not self.caught[i] and self.tips_in_holes(R, p)[i]:
                self.caught[i] = True
                self.allow[i] = max(g, 0.0)
            elif self.caught[i]:
                self.allow[i] = max(self.allow[i] - turn * PITCH, 0.0)
            if self.caught[i] and self.allow[i] <= 0.0:
                self.home[i] = True

    # -- the threads' pull, at every physics step ---------------------------------------------------------
    def _patch(self, env):
        if self._patched:
            return
        asset = env.unwrapped.scene[self.part]
        write = asset.write_data_to_sim
        me = self

        def write_with_threads(*a, **k):
            me._pull(env, asset)
            return write(*a, **k)

        asset.write_data_to_sim = write_with_threads
        self._patched = True

    def _pull(self, env, asset):
        import torch

        if not self.caught.any():
            return
        origin = m._np(env.unwrapped.scene.env_origins[0])
        pq = m._np(asset.data.body_link_pose_w)[0, self.dev]
        vel = m._np(asset.data.body_link_vel_w)[0, self.dev]
        R, p = m.quat_matrix(pq[3:7]), pq[:3] - origin
        forces, points = [], []
        for i, s in enumerate(self.strap):
            w = R @ s + p
            if not self.caught[i]:
                continue
            g = self.wall_x - w[0]
            v_out = -(vel[0] + np.cross(vel[3:6], R @ s)[0])  # the strap point's speed away from the wall
            f = (PRELOAD + K_THREAD * g) if self.home[i] else (K_THREAD * (g - self.allow[i]) + C_THREAD * v_out)
            f = float(np.clip(f, 0.0, F_MAX))
            forces.append([f, 0.0, 0.0])
            points.append(w + origin)
        if not forces:
            return
        # the composer takes one force per body: sum them, with the matching moment about the body's origin
        F = np.sum(forces, axis=0)
        com = pq[:3]
        T = np.sum([np.cross(pt - com, fv) for pt, fv in zip(points, forces)], axis=0)
        dev = asset.device
        asset.permanent_wrench_composer.set_forces_and_torques_index(
            forces=torch.tensor([[F.tolist()]], dtype=torch.float32, device=dev),
            torques=torch.tensor([[T.tolist()]], dtype=torch.float32, device=dev),
            positions=torch.tensor([[com.tolist()]], dtype=torch.float32, device=dev),
            body_ids=[self.dev], is_global=True)
        m._PUSHED.add((self.part, self.dev))
