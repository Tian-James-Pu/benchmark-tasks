# Recover a utensil obstructing a dishwasher spray arm

A wooden spatula has fallen through a partially withdrawn dishwasher rack. Its
handle intersects the lower spray arm. Extract it onto the counter and return
the rack inside so the arm can turn freely. The appliance is a full-size HAVA
R01 countertop model, not a miniature floor-standing dishwasher.

Two independent scripted recoveries scored 1.0, each within 1,399 actions.
Dream-1 scored 0.0 after its full 1,800-action budget; the utensil remained lodged.

## Reproduce and score

Use the YAM Arena environment with its two parallel-gripper arms. From the
repository root:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/clear-utensil-from-dishwasher-spray-arm/task.py \
  -E object_jitter=0 -E control_hz=20 -E device=cuda:0 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Restore free rotation of the dishwasher's lower spray arm, leaving its rack fully inside and the loose wooden utensil on the counter." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/dishwasher-demo
```

The same scene and scorer are used for the model run; replace `yam_scripted`
with `dream` and configure its API connection. Reset reloads the fixed starting
layout. No material is consumed and no random seed or object jitter is used.
The action limit is 1,800 control steps, 90 seconds at 20 Hz. The first complete
trial took 588 seconds of wall-clock time on a shared GPU, including startup.
The independent repeat took 2022 seconds on a more heavily shared GPU.
Each trial used 1,399 actions and produced 1,400 recorded frames (70 seconds of video).
The Dream-1 trial took 2,350 seconds of wall-clock time on a shared GPU,
including startup and API waits. It used 1,800 actions and produced 1,801
frames at 320 × 176 resolution (90.05 seconds of video), with no trial errors.

The demonstration uses exact simulator object poses and measured robot joint
positions as feedback. It produces robot joint/gripper actions only: it never
attaches, teleports or rewrites an object's pose. This feedback is specific to
the scripted feasibility demonstration; it is not supplied to the model.
A one-arm extraction is permitted. Returning the rack by contact during that
extraction is also a valid solution.

## Recorded checks

All results below use the final scene and scorer. Controls initialized at an
intermediate state are tests of that condition, not full recovery demos.

| Check | Result |
| --- | --- |
| Complete recovery, fresh process | 1.0, 1,399 actions; source revision `8f33138a7549` |
| Independent complete recovery, second fresh process | 1.0, 1,399 actions; unchanged scene, scorer and controller |
| No robot action, 200 actions | 0.0; utensil remains lodged |
| Utensil physically held on counter, all other end-state conditions satisfied | 0.0; release condition rejects the held utensil |
| Recorded rack-home state with utensil still inside, external clearance test | 0.0; arm stops at 35.47 degrees, short of a full turn |
| Dream-1, complete 1,800-action budget | 0.0; utensil remains lodged and rack partially withdrawn |

The complete demo and model recordings were reviewed through sampled frames,
along with the setup image. Separate scorer
checks reject a raised utensil, a dropped rack, objects beyond the counter,
and reaching back inside after a successful turn. Restoring a valid pose does
not restore an invalidated rotation witness; another full turn is required.

## Setup

World coordinates are metres: x points from the robots into the appliance,
y points left, and the table is z = 0.7475. Dimensions below are at scale 1.

| Object | Geometry, material and mass | Starting placement |
| --- | --- | --- |
| Dishwasher case | 430 × 430 × 460 mm; plastic outer shell and steel cavity, corresponding to the 13 kg appliance; fixed in simulation | Front x = 0.330, centred y = 0, bottom on counter |
| Cavity | 340 mm wide, 350 mm deep, 320 mm high | y = ±0.170; rear inside face x = 0.700; floor z = 0.8125 |
| Open door | 310 × 410 × 25 mm rigid plastic/metal panel, 1.3 kg; 260 × 320 mm window | Hinge at (0.330, 0, 0.8165), panel projects toward x = 0.020; free hinge at its open stop |
| Dish rack | 348 × 338 × 191 mm overall envelope; coated steel wires, 760 g frame and six 15 g plastic wheels | Frame at (0.415, 0, 0.8315); gravity settles it on the door and rails |
| Rack details | Ordinary rim 110 mm above frame, tall cup rail 187 mm; main wire diameter 5 mm, floor wire diameter 4 mm; six wheels radius 17 mm and width 9 mm, with free axles | Wheel local x = −0.130, 0, +0.130; y = ±0.16441; z = 0.017 |
| Rails | Two fixed plastic strips 350 × 10 × 19 mm | x = 0.350…0.700, y = ±0.16441, top z = 0.8315 |
| Lower spray arm | Tapered plastic blade, 310 mm span, width 26…50 mm, thickness 10 mm; 28 mm hub; 55 g | Free vertical-axis hinge at (0.525, 0, 0.8355), initially along x |
| Wooden spatula | NVIDIA mesh, 300 × 56 × 6 mm, 60 g | Authored origin (0.350, 0.105, 0.9485), long axis tilted 45° upward toward the robot; gravity produces the photographed lodged pose |
| Tea cup | NVIDIA ceramic cup, 119 × 84 × 102 mm, 200 g | Root (0.600, −0.360, 0.7485), upright |
| Banana | NVIDIA fruit mesh, 39 × 197 × 74 mm, 130 g | Root (0.400, −0.520, 0.7485) |
| Rear counter support | Fixed wood top 420 × 480 × 35 mm with two 45 mm square legs | Top centred at (0.680, 0, 0.7475), aligned with the station counter |

The rack is a free rigid basket with six physically rotating wheels. It has no
axial guide, grasp tab, lift restraint, attachment or recovery snap. The arm has
its ordinary solid tapered form: no utensil-sized hole was invented. The
spatula contacts the rack and arm through its unscaled authored SDF collision.

## Scoring

`completion.py` uses current object geometry, velocities and measured finger
positions. It awards no credit from a commanded pose or a demonstration phase.

- **0:** all outcomes that fail the released-recovery conditions below.
- **0.5:** for at least one second, all of the spatula's actual source-mesh vertices are on the side counter, at least 20 mm outside the case's side plane and
  50 mm inside the tabletop edges. Its lowest point is within −4…+6 mm of the
  counter, its highest point is no more than 35 mm above it, speed is below
  10 mm/s and angular speed below 0.15 rad/s. Every sampled finger point is at
  least 30 mm from its oriented source bounding box and 20 mm outside the expanded case envelope.
  Every finger point must also be at least 20 mm outside the actual oriented
  rack envelope (348 × 338 × 190 mm). The rack remains supported and upright: x = 0.190…0.531, |y| ≤ 8 mm,
  height within 4 mm of 0.8315 m, tilt and yaw within 3°, speed below 10 mm/s,
  angular speed below 0.10 rad/s.
- **1.0:** those conditions continue, the rack is within 5 mm of its home
  centre (x = 0.525, y = 0), and the lower arm physically completes a full
  360° turn in the external clearance check. The valid recovered state must
  then persist for three seconds. Moving the utensil or rack out of its
  qualifying state, or reaching back inside, cancels the rotation witness.

The checker starts only after one second with the rack home and the hands out.
It applies 0.001 N m about the real spray-arm hinge until the arm completes
one full turn, then removes that torque. This is an explicit evaluation
measurement, analogous to checking the arm by hand, not a hidden wash motor,
water-flow model or robot action. Full credit requires the actual hinge angle
to traverse a whole revolution; elapsed time and partial oscillations do not
substitute for rotation. The normal initial fault remains blocked under the
same light test in the separate physical control.

## Sources and fidelity

- [HAVA R01 appliance](https://havalab.com/products/r01-dishwasher): published
  430 × 430 × 460 mm external dimensions, 340 × 350 × 320 mm cavity, 13 kg,
  and 740 mm overall depth with the door open.
- [Replacement rack](https://havalab.com/products/dish-rack-for-r01-dishwasher):
  13.3 × 13.7 × 7.5 inches. The 337.82 mm dimension is assigned across the
  340 mm cavity and the 347.98 mm dimension along its 350 mm depth. Wire
  layout, six-wheel arrangement and cup rail follow the product image.
- [Replacement lower arm](https://havalab.com/products/lower-spray-arm-for-r01-dishwasher):
  tapered blade and hub form. The 310 mm span is inferred against the cavity;
  it is not a published manufacturer measurement.
- [HAVA manual](https://havalab.com/cdn/shop/files/HAVA-R01-Countertop-Dishwasher-User-Manual.pdf?v=1747819440):
  printed pages 3 and 14 show the interior and require checking free arm
  movement after replacement.

The mechanism is reconstructed from manufacturer envelopes and photographs,
not proprietary CAD. Individual wire positions, 17 mm wheel radii, rails,
arm thickness, masses of moving components and friction are explicit
approximations. Coated wires, wheels, arm and case are rigid; no plastic or
wood fracture, compliance, water, detergent, cleaning or electrical cycle is
modeled. The heavy case and rails are fixed. The door and rack remain dynamic.
Contact offsets are 0.3 mm to avoid artificially closing the narrow wire gaps.
Arm hinge friction is 0.0001 N m and damping 0.00015 N m s/rad; these are
unmeasured parameters, not calibrated hydraulic performance.

All three loose props are © NVIDIA, CC-BY 4.0, pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [kitchenware_spatula_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_spatula_b01)
- [dishware_cup_tea_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_tea_a01)
- [food_banana_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_banana_a01)

Assets download on the first run. Scale remains 1; masses are set as listed,
authored inertia is scaled with mass, the spatula retains its authored SDF collision at resolution 512 (its narrow neck must not be filled by a global hull); the other two props use convex hulls,
and the spatula has 0.3 s⁻¹ angular damping. NVIDIA labels the spatula material
as wood; it is removed from the appliance rather than reloaded for washing.
`mechkit.py` derives from this repository's `heat-shrink-lug` helper;
`simready.py` derives from `cutters-off-pegboard`. `closedloop.py` provides
joint-space approach and Cartesian robot feedback control.

The scorer includes `spatula_vertices.npy`: 1,116 unscaled measurement vertices
derived from NVIDIA's pinned `payloads/geometries.usd`, under the same CC-BY4.0
license. Source geometry SHA-256:
`1f2fb68d57d03251f4cb6fe5a6a68459d54eef39a8e21eb8bbf236cc90ce8303`.
`completion.py` verifies the measurement array's SHA-256 before loading it.
Support and counter boundaries use actual mesh extrema; conservative object-box
distance remains the released-finger clearance check. The task-level success
hold is zero because the scorer itself enforces the documented three-second hold.
