"""Geometric completion checks; the rotation witness is measured by task.py.

Source mesh coordinates refer to the unscaled NVIDIA spatula. All distances
are metres. No demo phase or commanded pose is used to award credit.
"""
import itertools
import hashlib
import pathlib
import numpy as np

SPATULA_LOW=np.array([-.0278980061,-.187166199,.000018096])
SPATULA_HIGH=np.array([.027894333,.113104381,.006158698])
SPATULA_CORNERS=np.array(list(itertools.product(*zip(SPATULA_LOW,SPATULA_HIGH))))

# Derived from NVIDIA's pinned unscaled geometries.usd, CC-BY4.0.
# Mesh vertices preserve support geometry; OBB corners include empty space.
SOURCE_GEOMETRY_SHA256="1f2fb68d57d03251f4cb6fe5a6a68459d54eef39a8e21eb8bbf236cc90ce8303"
VERTEX_DATA_SHA256="c600b2a12373ce2a2effeba7ae5ab29424b890ecaed973bef46cd90f1b2c7cd2"
_vertex_path=pathlib.Path(__file__).with_name('spatula_vertices.npy')
if hashlib.sha256(_vertex_path.read_bytes()).hexdigest()!=VERTEX_DATA_SHA256:
    raise ValueError('Spatula measurement vertices do not match pinned source data')
SPATULA_VERTICES=np.load(_vertex_path,allow_pickle=False)
if SPATULA_VERTICES.shape!=(1116,3) or not np.isfinite(SPATULA_VERTICES).all():
    raise ValueError('Invalid spatula measurement geometry')

def point_box_distance(points,low,high):
    points=np.asarray(points)
    return np.linalg.norm(np.maximum(np.maximum(low-points,points-high),0),axis=-1)

def evaluate(*,top,spatula_pos,spatula_rotation,spatula_velocity,spatula_omega,
             rack_pos,rack_rotation,rack_velocity,rack_omega,fingertips):
    """Current, observable conditions only; no persistent success latch."""
    p=np.asarray(spatula_pos);r=np.asarray(spatula_rotation)
    rp=np.asarray(rack_pos);rr=np.asarray(rack_rotation)
    tips=np.asarray(fingertips)
    vertices=SPATULA_VERTICES@r.T+p
    lo=vertices.min(axis=0);hi=vertices.max(axis=0)
    # The side counter is the ordinary station tabletop outside the case/door.
    on_side=(lo[1]>=.235 or hi[1]<=-.235)
    within_table=lo[0]>=-.14 and hi[0]<=.86 and lo[1]>=-.60 and hi[1]<=.60
    on_counter=on_side and within_table and top-.004<=lo[2]<=top+.006 and hi[2]<=top+.035
    utensil_still=np.linalg.norm(spatula_velocity)<.01 and np.linalg.norm(spatula_omega)<.15
    # A pulled-out rack may remain on the open door for partial credit. A
    # displaced, tipped or dropped rack cannot receive any score.
    rack_upright=rr[2,2]>=np.cos(np.radians(3)) and abs(np.arctan2(rr[1,0],rr[0,0]))<=np.radians(3)
    rack_supported=(.19<=rp[0]<=.531 and abs(rp[1])<=.008 and abs(rp[2]-(top+.084))<=.004 and rack_upright)
    rack_still=np.linalg.norm(rack_velocity)<.01 and np.linalg.norm(rack_omega)<.10
    rack_home=abs(rp[0]-.525)<=.005 and abs(rp[1])<=.005
    local_tips=(tips-p)@r
    utensil_released=bool(np.all(point_box_distance(local_tips,SPATULA_LOW,SPATULA_HIGH)>=.030))
    # Hands outside the entire appliance and its rack before the test starts.
    case_low=np.array([.31,-.235,top+.04]);case_high=np.array([.78,.235,top+.48])
    hands_out=bool(np.all(point_box_distance(tips,case_low,case_high)>=.020))
    rack_tips=(tips-rp)@rr
    rack_released=bool(np.all(point_box_distance(rack_tips,np.array([-.174,-.169,0.]),np.array([.174,.169,.190]))>=.020))
    recovered=bool(rack_released and on_counter and utensil_still and utensil_released and rack_supported and rack_still and hands_out)
    return dict(recovered=recovered,ready_for_rotation=bool(recovered and rack_home),
                utensil_on_counter=bool(on_counter),utensil_still=bool(utensil_still),
                utensil_released=utensil_released,rack_supported=bool(rack_supported),
                rack_still=bool(rack_still),rack_released=rack_released,rack_home=bool(rack_home),hands_out=hands_out,
                utensil_low=lo.tolist(),utensil_high=hi.tolist())

class CompletionTracker:
    """Light, external rotation measurement after released recovery.

    Returning to an invalid configuration immediately cancels the witness.
    The robot has no access to this observer's internal state.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.recovered_s=0.;self.ready_s=0.;self.passed_s=0.
        self.start_angle=None;self.rotation_passed=False
        self.apply_test_torque=False;self.score=0.

    def update(self,conditions,angle_deg,dt=.05):
        self.recovered_s=self.recovered_s+dt if conditions['recovered'] else 0.
        if not conditions['ready_for_rotation'] or not np.isfinite(angle_deg):
            self.ready_s=0.;self.passed_s=0.;self.start_angle=None
            self.rotation_passed=False;self.apply_test_torque=False
        else:
            self.ready_s+=dt
            if self.ready_s>=1. and self.start_angle is None:
                self.start_angle=float(angle_deg)
            if self.start_angle is not None and abs(angle_deg-self.start_angle)>=360.:
                self.rotation_passed=True
            self.apply_test_torque=self.start_angle is not None and not self.rotation_passed
            self.passed_s=self.passed_s+dt if self.rotation_passed else 0.
        self.score=1. if self.passed_s>=3. else .5 if self.recovered_s>=1. else 0.
        return self.score
