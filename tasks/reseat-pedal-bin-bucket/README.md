# Reseat a pedal bin’s inner bucket

The inner bucket was returned a quarter turn from its keyed position. Its rim rests on the internal lift-rod guide, leaving the lid visibly ajar. Restore a seated insert and a closed, released lid. Janitors and building cleaners routinely gather and empty trash; returning reusable bins to working condition is part of that workflow. The [BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/building-and-grounds-cleaning/janitors-and-building-cleaners.htm) reports about 2.4 million jobs in this occupation in 2025.

## Run and score

Requires an Isaac Lab environment with `yam_arena`, `inspect-robots`, and the SimReady assets named below. Run from the repository root:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/reseat-pedal-bin-bucket/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "The lid of this pedal bin won't shut. Fix it so the bin closes properly." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir /tmp/pedal-bin-demo
```

Use `--policy dream` for a Dream-1 trial. The control budget is 2,400 actions at 20 Hz (120 simulated seconds). The fixed scripted plan currently lasts 38.2 simulated seconds. There is no object jitter or randomized seed. Reset reconstructs the quarter-turned insert and lets the rigid bodies settle under gravity. Nothing is consumed.

**Build status: parked; do not submit.** The full recovery demo scores 0; no-action and push-lid adversaries also score 0. Exact correspondence between the modeled notch, guide and bail grip and a commercial pedal bin has not been established. The manufacturer examples inspected use different insert shapes, rim profiles or handles. A matching primary product source and faithful geometry are required before further validation. No successful demonstration or model score is claimed.

## Scene and mechanism

Bench surface z = 0.7475 m; +x points away from the robot bases and +y to their left. The bin base is centred at (0.320, 0.030) m. Dimensions and positions below are metres unless otherwise stated.

| Object | Dimensions, mass, material, and start |
|---|---|
| Bin body | 0.250 diameter, rim 0.275 above bench, 2.20 kg; stainless wall, black polymer base and rubber foot. Free body. Inside bore 0.237 diameter, floor 0.020 above bench. |
| Lift-rod guide | Fixed steel guide inside the back (+x) wall; 0.020 wide, 0.013 thick, z = 0.2515–0.266 relative to the base. Its inner edge lies x = 0.1075 from the bin axis. |
| Lid | Stainless dome 0.256 diameter, 0.031 rise above its rim, 0.40 kg. Gravity hinge at (0.150, 0, 0.283) relative to the bin; stops at 0° and 85° open. Viscous damping 0.45 N·m·s/rad. Settles about 9° ajar on the misplaced insert. |
| Pedal | Polymer plate 0.050 × 0.090 × 0.010, initially centred (−0.150, 0, 0.035) relative to bin; steel arm, assembly 0.12 kg. Spring 1.1 N·m/rad, preload 0.25 rad, 25° stop. About 20° depression opens the lid to about 79°. |
| Inner bucket | Grey polypropylene, 0.250 high; diameter tapers 0.204–0.211, rim band 0.229 diameter; nominal wall thickness 0.003, collider wall 0.005. Mass 0.30 kg. Rim notch approximately 0.040 wide and 0.020 deep. Starts yaw +90°, resting on the guide and leaning about 3°. |
| Bail | Freely hinged steel wire 0.004 diameter with black polymer grip 0.090 long and 0.018 diameter; 0.06 kg. Pivot height 0.242 above bucket base; grip 0.070 above pivots when upright. Starts folded toward the notch. |
| Glove box | SimReady `box_glove_a01/sm_box_glove_a01_01.usd`, cardboard; size (0.178, 0.117, 0.056), 0.35 kg. Bottom centre (0.50, −0.30, 0.7475), yaw +20°. |
| Disinfectant bottle | SimReady `disinfectant_a01/sm_disinfectant_a01_01.usd`, rigid plastic bottle; size (0.045, 0.067, 0.191), 0.50 kg. Bottom centre (0.30, −0.25, 0.7475), yaw −30°. No liquid transfer is modeled. |

The bin and bucket remain independent dynamic bodies. Rigid contact between the guide and the unnotched rim creates the fault. The pedal-to-lid push linkage uses a 6 N·m/rad compliant target that applies opening torque only while the measured lid is below the opening requested by the measured pedal angle. It never pulls the lid down or switches based on grasp, robot position, or demonstration phase. Release permits gravity closure. The bucket and bail move only through gravity, joint friction, and collision contact.

Custom segmented meshes model the articulated bin, hollow insert, and bail; visual and collision surfaces are separate. The two cleaning supplies use SimReady assets. No shell deformation, fracture, or liquid behavior is simulated. Mechanism friction and damping are stated nominal simulation parameters, not a calibrated commercial product.

## Scoring

The scorer uses relative poses, speed, and fingertip clearance. A ruler, angle template, video, and setup photographs suffice to reproduce the measurements physically.

A seated bucket has base height 0.020 ± 0.003 above the bin base, horizontal centre offset at most 0.008, relative tilt at most 2°, and notch yaw within ±15° of the back guide. The lid is shut at an opening angle at most 0.2° from its closed stop (approximately 1 mm front-edge gap).

Both grippers must finish at least 0.030 from the bin, lid, pedal, hinge bracket, and bucket/bail envelope. Bin and bucket speeds must remain at most 0.010 m/s and 2°/s; lid speed must remain at most 2°/s, for 20 consecutive control steps (one second). Clearance uses conservative cylindrical envelopes for curved surfaces, including the space swept by the bail.

Score is 1.0 when seated, shut, clear, and still; 0.5 when seated, clear, and still but the lid remains open; otherwise 0. Any run in which the bin or bucket base falls more than 0.050 below the bench scores zero permanently until reset. Safe temporary slide, tilt, and alternate execution are allowed. Removing the bucket and closing an empty bin earns zero.

The scripted controller is an example solution only. Its waypoints never enter the mechanism or scorer. `mechkit.py` is the mechanism kit from the public `heat-shrink-lug` task; `simready.py` provides the shared SimReady asset loader.

Manufacturer audit references: [IKEA SNÄPP assembly manual, page 8](https://www.ikea.com/nl/en/assembly_instructions/snapp-pedal-bin-grey__AA-1794875-8-100.pdf), [Brabantia NewIcon 5L insert](https://www.brabantia.com/int_en/newicon-plastic-inner-bucket-5-litre-dark-grey/), and [Brabantia 20L insert](https://www.brabantia.com/plastic-inner-bucket-20-litre-black-396922/). These establish real removable inserts but do not establish that the current model’s central fault geometry is representative.
