"""Continuous tendon mechanics for an unvalidated Bowden-repair prototype.

Each point is a tendon guide or end attachment. The host simulation must apply
the returned force at that physical point, including equal reactions on moving
sheath guides and attachments. This module neither detects seating nor attaches,
teleports, or actuates objects. Stiffness and damping require calibration.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TendonLoad:
    length: float
    extension: float
    extension_rate: float
    tension: float
    energy: float
    forces: np.ndarray


def tendon_load(points, velocities, rest_length, stiffness, damping=0., rest_velocity=0.):
    """Evaluate a tension-only elastic line through physical guide points.

    Units are metres, seconds, newtons. Positive rest_velocity pays line out;
    negative rest_velocity takes it up. A motor changing rest_length must also
    receive the corresponding resisting torque, via its real spool radius.
    No friction/capstan losses are modeled yet. A sleeve cannot be made
    functional by changing this law when a seating condition becomes true.
    """
    p = np.asarray(points, dtype=float)
    v = np.asarray(velocities, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or v.shape != p.shape:
        raise ValueError("Expected matching N-by-3 point and velocity arrays, N >= 2")
    if not np.isfinite(p).all() or not np.isfinite(v).all():
        raise ValueError("Point states must be finite")
    parameters = np.asarray([rest_length, stiffness, damping, rest_velocity], dtype=float)
    if not np.isfinite(parameters).all() or rest_length < 0 or stiffness <= 0 or damping < 0:
        raise ValueError("Invalid tendon material or rest-length parameters")
    segments = np.diff(p, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError("Adjacent guide points must be distinct")
    directions = segments / lengths[:, None]
    length = float(lengths.sum())
    extension = max(0., length - rest_length)
    extension_rate = float(np.sum(directions * np.diff(v, axis=0)) - rest_velocity)
    tension = max(0., stiffness * extension + damping * extension_rate) if extension > 0 else 0.
    forces = np.zeros_like(p)
    segment_forces = tension * directions
    forces[:-1] += segment_forces
    forces[1:] -= segment_forces
    return TendonLoad(length, extension, extension_rate, tension,
                      .5 * stiffness * extension ** 2, forces)
