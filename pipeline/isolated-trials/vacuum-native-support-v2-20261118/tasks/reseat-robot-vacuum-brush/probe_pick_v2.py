"""Score-disabled repeat of roller pickup with repaired contact mechanism."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
from dataclasses import dataclass
import yam_arena.demo as _demo
from yam_arena import ik as _ik
@dataclass
class RotWaypoint(Waypoint):
    """A waypoint with the gripper's full orientation: `approach` is where the fingers point, `closing` the line they
    close along (world vectors)."""

    approach: tuple = (0.0, 0.0, -1.0)
    closing: tuple = (0.0, 1.0, 0.0)


def _rot(approach, closing) -> np.ndarray:
    z = np.asarray(approach, float)
    z /= np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - x.dot(z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _wp_rot(w) -> np.ndarray:
    return _rot(w.approach, w.closing) if isinstance(w, RotWaypoint) else _demo.grip_rotation(w.yaw)


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), _wp_rot(w)
        if t < w.duration:
            s = _ik.smoothstep(t / w.duration)
            return prev_pos + s * (goal - prev_pos), _ik.slerp_matrix(prev_rot, rot, s), prev_finger + s * (w.finger - prev_finger), w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


if getattr(_demo.ArmPlan.target, "__name__", "") != "_target":
    _demo.ArmPlan.target = _target


def demo():
 a=math.radians(35)
 def W(d,p,f,label):return RotWaypoint(d,p,f,label,0.,(math.sin(a),0.,-math.cos(a)),(-math.cos(a),0.,-math.sin(a)))
 return {'left':[
  W(3.,(.36,.08,TOP+.23),-0.027,'approach brush'),
  W(2.,(.427,.006,TOP+.18),-0.027,'above brush'),
  W(2.,(.427,.006,.865),-0.027,'straddle roller'),
  W(1.,(.427,.006,.865),FINGER_CLOSED,'grip roller'),
  W(2.,(.427,.006,.915),FINGER_CLOSED,'lift from bay'),
  W(2.,(.427,.006,.915),FINGER_CLOSED,'hold inspection')]}

