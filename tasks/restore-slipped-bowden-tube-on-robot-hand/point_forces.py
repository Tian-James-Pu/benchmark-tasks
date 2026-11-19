"""Apply a complete set of mechanical point loads without dropping reactions.

One caller owns the permanent wrench buffers for the listed parts. Call once
per physics step, including with an empty load list when the tendon is slack.
Each load is (part, body name or None, force in station axes, station point).
This changes forces only; it never changes poses, joints, or attachments.
"""

import numpy as np


def apply_point_loads(environment, parts, loads):
    import torch
    import mechkit as m

    base = environment.unwrapped
    assets = {name: base.scene[name] for name in parts}
    origin = m._np(base.scene.env_origins[0])
    prepared = []
    for part, body, force, point in loads:
        asset = assets[part]
        force, point = np.asarray(force, dtype=float), np.asarray(point, dtype=float)
        if force.shape != (3,) or point.shape != (3,) or not (
                np.isfinite(force).all() and np.isfinite(point).all()):
            raise ValueError('Mechanical force and application point must be finite 3-vectors')
        if body is None and 'frame' in getattr(asset, 'body_names', ()):
            body = 'frame'
        index = 0 if body is None else asset.body_names.index(body)
        f = torch.as_tensor(force, dtype=torch.float32, device=asset.device)[None, None, :]
        p = torch.as_tensor(point + origin, dtype=torch.float32, device=asset.device)[None, None, :]
        prepared.append((asset, index, f, p))
    # set_forces_and_torques_index clears the entire environment's wrench
    # buffers, even when selecting one body. Reset once, then ADD every load.
    for asset in assets.values():
        asset.permanent_wrench_composer.reset()
    for asset, index, force, point in prepared:
        asset.permanent_wrench_composer.add_forces_and_torques_index(
            forces=force, positions=point, body_ids=[index], is_global=True)
