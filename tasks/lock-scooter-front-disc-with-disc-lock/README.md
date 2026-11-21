# Fit a disc lock through a vent hole in a parked scooter's front brake disc

A 50 cc scooter is parked with its left side to the robot. Its front brake disc (220 mm across, 4 mm thick, nine 9 mm vent holes)
is bolted to the front wheel, and a disc lock lies on the floor beside the wheel. The instruction states only the goal:
"Lock the scooter's front brake disc with the disc lock."

A disc lock is a steel C with a 7 mm jaw. The rider slides the jaw over the disc's edge and pushes the lock's 5.5 mm pin across the
jaw, through one of the disc's vent holes, until it catches. Once it has caught, the wheel cannot turn more than part of a revolution
before the lock hits the fork or the caliper. The pin only goes home through a hole. Two things make that hard for a robot:

- **Holding it square and on a hole.** The jaw has 1.5 mm of play on each side of the disc. The pin has 1.75 mm of play in the hole.
  The lock must be held on the disc's edge with its pin over a hole the hand cannot see behind the lock body. Pressed off-centre, the
  pin stops on the steel, the cap stays 8 mm proud and the lock is not shut: let go, and it drops off.
- **Two hands in two roles.** A person holds the lock with their fingers and presses the plunger with their thumb. A parallel gripper
  cannot do both, so one arm holds the 0.56 kg lock on the disc while the other presses the plunger's cap flush along the pin's axis.
  If no hole is where the lock goes, the wheel can be turned (it takes a firm push: about 1 N m) or the lock moved round the disc to
  the next hole, 40 degrees on.

Turning the wheel with the lock on is the classic way disc locks do damage: the lock is carried round into the caliper or fork. The
scorer records it.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the lock's catch, the wheel's friction, the scorer (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged |
| `demokit.py` | Grip orientations for the scripted demo (`tilted`, `level`); its closed-loop runner is not used, unchanged |
| `simready.py` | Loads the SimReady distractors, unchanged |
| `task.json` | The task's details for the board |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From the repository root:

```bash
# the scripted demonstration (about 22 s of motion, then the 3 s hold)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lock-scooter-front-disc-with-disc-lock/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Lock the scooter's front brake disc with the disc lock." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/disc-lock-demo

# a model run (Dream-1)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/lock-scooter-front-disc-with-disc-lock/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the scooter's front brake disc with the disc lock." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/disc-lock-dream
```

The run prints `task_score`. Runtime per trial: about 1 to 2 minutes on an L40S once the shaders are compiled (several minutes the
first time on a GPU). Seeds: the layout is fixed (`object_jitter=0`), so the seed changes nothing in the scene; the demo is open loop
from that published layout.

## Scoring

The scorer (`Trial` in `task.py`) reads the end state each control step. It cannot create the evidence: it only reads the lock's
pose, its plunger's joint, the wheel's angle and the fingertip points.

- **Locked**: the plunger has been taken by the lock's catch (pressed at least 9.5 mm of its 10 mm travel: the cap within 0.5 mm of
  flush) and is still at least 9 mm in, and the pin's axis crosses the disc within 4.5 mm of a vent hole's centre (`pin_offset`), so
  the pin is through that hole. A pin over the steel cannot reach the catch, because the disc stops it after 2 mm. A lock pressed shut in
  the air and then hung on the disc has no hole round its pin and scores 0.
- **1.0**: locked, both grippers at least 3 cm from the lock (24 points on the fingers against the lock's box and cap, `hands_gap`),
  the lock still (under 1 cm/s and 0.1 rad/s) and the wheel still (under 0.02 rad/s), all for 2 s in a row (40 control steps). The
  harness then waits another second (`SUCCESS_HOLD_S`) before it ends the episode.
- **0.5**: locked at the end, but a gripper is still within 3 cm of the lock, or it has not yet been still for 2 s.
- **0**: anything else: the pin stopped on the steel with the cap standing proud, the lock held but not pressed, the lock off the disc
  or on the floor, or caught with no vent hole round its pin.
- **Damage (0, whatever follows)**: once the lock has caught, if any corner of it comes within 3 mm of the brake caliper or the fork
  leg (`strike_gap`): the wheel was turned with the lock on. The record is cleared only by a reset (mechkit's `on_reset`), never because
  the lock looks right again.

Credit does not stick: the 1.0 needs the hold to be running at the last step (or the harness's success, which ends the episode at
once). Lifting, tilting or regripping the lock, turning the wheel before the lock catches, and moving the lock round to a different
hole are all allowed and cost nothing.

Possible scores: 0, 0.5, 1.0.

## The demo

`demo()` is a fixed plan of waypoints from the published starting layout, played by the `yam_scripted` policy (open loop: it reads
nothing during the run).

1. Left: comes over the lock from above and behind (a 45-degree grip), closes its fingers across the lock's 38 mm back and lifts it.
2. Left: carries it to the disc, turning it 20 degrees on the way, and slides it 3.5 cm onto the disc's edge, so that the pin is
   over the vent hole 20 degrees below the disc's front point. It holds it there.
3. Right: comes round in front of the lock with its fingers closed and level, and presses the plunger's cap 10 mm in, flush. The
   catch takes it.
4. Right backs away; then the left opens and backs away. The lock hangs on its pin in the hole, drooping a few degrees until the
   jaw's slot rests on the disc's edge.

The left arm's waypoints carry a trim (`LEFT_TRIM`) measured in the simulator: under the lock's weight the arm sags 22 mm and falls
9 mm short at that reach, and the plan sends it that much further so the pin arrives over the hole.

### Runs (GPU, `object_jitter=0`, `yam_scripted`)

| Run | What happens | task_score |
|---|---|---|
| Demo, run 1 | the pin crossed the disc 0.1 to 0.2 mm from the hole's centre; pressed, the plunger reached 9.95 mm and caught; after both hands let go the lock drooped on its pin until it rested on the disc's edge, its pin bearing on the hole's side (1.8 mm off centre), the wheel turned 0.35 degrees under its weight and held | 1.0 |
| Demo, run 2 | identical, step for step | 1.0 |
| Demo, run 3 | after the tuning log was taken out of `task.py`: identical | 1.0 |
| Do nothing | both arms stay at their ready pose; the lock stays on the floor | 0 |
| Shut it in the air first | the left lifts the lock and the right presses the cap 3 cm past flush with nothing behind the jaw: the plunger caught (9.8 mm); slid onto the disc as in the demo, the shut pin stopped on the disc's edge, 15.7 mm from the hole, and when let go the lock (still shut) dropped to the floor | 0 |
| Shut it in the air, gently | as above with the demo's press: the lock slid 1 cm in the left's fingers and the plunger reached only 5 mm, then sprang back; put on the disc over the hole unpressed and let go, it dropped | 0 |
| Careless | the lock slid onto the disc's front point, between two vent holes (17 mm from each), and the cap pressed anyway: the plunger stopped at 1.6 mm on the steel and sprang back; let go, the lock dropped to the floor | 0 |
| One arm | the left puts the lock on the disc with its pin over the vent hole (0.1 to 0.8 mm off centre), as the demo does, and lets go; nobody presses the plunger, and the lock drops to the floor | 0 |
| Wheel turned with the lock on (a scorer check, not a robot route) | the demo, then a steady 15 N push up on the tyre's front from 3.5 s after the catch: the wheel turned 54 degrees in the 9 s left, carrying the locked lock round with it; the run ended before the lock reached the fork leg, so it scored as a locked lock that is not still | 0.5 |

The damage rule was checked offline on the same geometry: a lock fitted 20 degrees below the front point and carried round with the
wheel comes within 3 mm of the fork leg after about 110 degrees of turn (4.9 mm at 110 degrees, inside the leg at 120), and from then
on the trial scores 0. It was not seen to fire in a GPU run.

An earlier tuning run, kept for the record: before the left arm's sag was fully trimmed, the pin crossed the disc 6.3 mm from the
hole's centre; pressed, the plunger stopped at 1.9 mm on the steel, and when the hands let go the lock dropped to the floor (score 0).

## Importance and sources

- **Who does it.** Disc locks are the everyday lock for scooters and motorcycles: small enough to carry, fitted and taken off at
  each stop. ACEM, the European motorcycle industry association: "There are more than 39 million motorcycles and scooters on Europe's
  roads (2019 estimate)" ([ACEM statistical release, January-December 2025](https://www.acem.eu/wp-content/uploads/2026/01/ACEM_statistical_release_-_January-December_2025.pdf)).
- **The product.** Kryptonite Keeper 5-S2 disc lock (model 000877), a "stapler style" disc lock: 5.5 mm locking pin, 0.56 kg, discs
  up to 6.0 mm thick and up to 41.5 mm into the jaw ([Kryptonite product page](https://www.kryptonitelock.com/en/products/product-information/current-key/000877.html)).
  RevZilla lists the Kryptonite Keeper 5-S disc lock at $44.95 ([RevZilla](https://www.revzilla.com/motorcycle/kryptonite-keeper-disc-lock)).
  The same class from another maker, the OnGuard Boxer 5.5 mm (product 8051, $47.95), is sold as a "stapler-style Touch Lock system
  that lets you secure your motorcycle or scooter quickly with a single hand" ([RevZilla](https://www.revzilla.com/motorcycle/onguard-boxer-55mm-disc-lock)).
- **The maker's procedure.** Kryptonite's disc-lock instruction sheet: "Place on rotor as seen in the image. Replace crossbar or align
  pin-style Disc Lock depending on style of lock. ... Check the lock before leaving the motorcycle to be sure it is properly secured.
  Warning: Always remove your lock before operating the motorcycle!"
  ([Kryptonite instruction sheet](https://www.kryptonitelock.com/content/dam/kryt-us/en/Inserts/Miscellaneous/INS_Disc_noATPO.pdf)).
- **The wheel.** A 3.50-10 tyre on a 10-inch rim: 254 mm bead seat plus twice the 89 mm section height, about 432 mm across, as the size
  code gives it. The 220 mm, 4 mm disc and its vent holes (nine 9 mm holes on a 194 mm circle) are representative of a 50 cc scooter's
  front disc, not taken from one model's drawing.

## Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| TK-CGMK5X, U-lock a bike wheel to a stand | a U-lock's shackle round a rim and a stand, angled into its foot | a shackle round two bars; no pin to bring through a hole, no plunger |
| TK-PPP2NQ, brake pad change round a disc | service work at a caliper: pads pushed into retainers, the caliper swung down | the disc is only the place; here the job is a lock fitted through the disc itself |
| TK-UKX5MH, linear-guide carriage past a spring plunger | sliding a carriage onto a rail past a passive sprung plunger | here the plunger is driven by a second hand, along a 5.5 mm pin that must be over a 9 mm hole first |
| "Hook a Club across a steering wheel" (not accepted) | a bar hooked round a steering wheel's rim and drawn out | hooks a rim; no pin and hole, no alignment |

No task on the board, and none found in LIBERO, RoboCasa, BEHAVIOR or RoboARC, fits a disc lock or aligns a pin through a hole in a
brake disc.

## Real trials

### The rig

- A 50 cc scooter with a drilled front disc (any with vent holes of 8 to 10 mm), on its centre stand. If its front tyre rests on the
  floor on the stand, put the front on a motorcycle front paddock stand so that the wheel turns by hand.
- A Kryptonite Keeper 5-S2 disc lock (or another 5.5 mm pin stapler-style disc lock), with its key.
- Floor tape, a paint pen, a 0.5 mm feeler gauge, a phone camera on a stand behind the robot, a luggage scale.

### Preparation (each session)

1. Wipe the disc clean and dry; brake dust on the pads of the grippers makes them slip. Make sure the disc is cold.
2. Mark the start: turn the wheel so that two vent holes sit 20 degrees either side of the disc's front point (a 3D-printed or card
   template with the 194 mm hole circle, or a protractor), then put a paint dot on the tyre's sidewall level with a tape mark on the
   fender.
3. Check the wheel's drag: hook the luggage scale on the tyre's tread and pull along the tread. It should start the wheel turning at
   4 to 6 N (about 1 N m). If it turns more easily, the 0.56 kg lock's own weight may turn it once fitted; a light touch of the front
   brake lever held by a strap gives some drag.
4. Unlock the lock, check its plunger stands out fully, and lay it flat at its tape marks (x 27 cm, y 19 cm from the midpoint of the
   arm bases), slot facing the wheel.

### Measurement (at the end, before anyone touches the scene)

1. Photograph the lock from the robot's side, then look at the plunger's cap: flush (a 0.5 mm feeler gauge does not fit under a
   straightedge across the cap and the lock's face) or proud.
2. With the cap flush, look behind the disc: the pin's tip shows in the far jaw through a vent hole. If the cap is proud, or the lock
   is on the floor, it is not locked.
3. From the video: whether both grippers were at least 3 cm clear and the lock and wheel still for the last 2 s, and whether the lock,
   once shut, ever touched the caliper or the fork leg (a frame showing contact scores 0).

### Reset (about 2 minutes)

Unlock the lock with its key and lay it on its tape marks; turn the wheel back to its paint mark; put the distractors on their marks.
Nothing is consumed.

### Safety

The robot works at the front wheel only; the scooter's ignition is off and its key out. The disc and the lock have no sharp edges,
and the disc must be cold (it is hot after a ride). The robot's grippers take nothing heavier than the 0.56 kg lock.

## The model

- **The wheel** (`scooter` mechanism, `wheel` link) turns on an unlimited hinge about the axle. Its friction is an elasto-plastic
  (Coulomb) model on the joint's drive (`Drag`): a 300 N m/rad spring holds the wheel at an anchor angle, and when the wheel is pushed
  more than 1.0 N m past it the anchor follows 0.8 N m behind. So the wheel stays put under less than 1.0 N m (the hanging lock puts
  about 0.6 N m on it) and turns against 0.8 N m under more. These values are estimates for the bearings, the pads' drag and the tyre
  on the floor; they are checked on the real scooter with the luggage scale (Preparation, step 3).
- **The disc** is made of convex collider pieces: around each vent hole, twelve wedges that leave a 12-sided hole whose inscribed
  circle is the 9 mm hole; between the holes, band sectors; inside, the solid disc to the hub. The pin is a 5.5 mm cylinder with a
  0.6 mm chamfer at its tip, so with its axis within 1.75 mm of a hole's centre it passes, and further off it stops on the steel.
- **The lock** (`disc_lock` mechanism) is a free 0.54 kg body (its two jaws and spine, with 1.5 mm chamfers at the jaw's mouth) with
  one link, the plunger (20 g), on a slider along the pin, sprung back to open (400 N/m: 4 N at full press). The catch (`Catch`)
  locks the slider's joint when it reaches 9.5 mm; only a reset frees it, as only the key would. The real Keeper closes with a
  stapler action and is then locked with its key; the model keeps the part a robot does (the pin brought through the hole and pushed
  home) and leaves out the key.
- **The scooter's front end**: the fork legs (34 mm), axle and nut, caliper and fender are fixed colliders. The scooter's body
  past the fork crown is left out.

## Assets

The distractors come from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01) (sunglasses)
- [`food_beverage_coffeeCup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_coffeeCup_a01) (paper coffee cup)
- [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01) (rag)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: the sunglasses' and
the cup's parts are welded into one body each; masses are set to 30 g (sunglasses), 250 g (cup) and 40 g (rag); the invisible
grasp-annotation curves are deactivated. The files are not committed: they download on first use.

The scooter's front end and the disc lock are custom meshes and colliders built in `task.py` with `mechkit.py`; SimReady has no
scooter, wheel of this size or disc lock.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository, `tasks/heat-shrink-lug/`,
which is published under the MIT licence; `simready.py` is adapted from `tasks/cutters-off-pegboard/` (loader unchanged). Kryptonite
and OnGuard are trademarks of their owners, named only to identify the product class.
