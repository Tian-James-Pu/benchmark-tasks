# Free a jammed steering lock: turn the wheel off the bolt and hold it there while the key turns to ON

A car with a keyed ignition was parked and its steering wheel was moved after the key came out. The column lock's bolt dropped into a
notch of the lock collar on the steering shaft, and the wind-up stored in the front tyres now presses the notch's wall onto the bolt.
The key's cam cannot draw a pinched bolt back, so the key turns a few degrees and stops. The fix most drivers learn is two-handed:

- **Find the way the wheel gives.** Clockwise, the wheel is already against the bolt: a dead stop. Anticlockwise it gives, against the
  tyres' spring.
- **Hold it there.** Let go and the spring puts the wall back on the bolt.
- **Turn the key while still holding**, through ACC to ON, and let go of it in its ON detent. Only then let go of the wheel.

Forcing the key alone does not free the bolt, and in a real car can snap the key. The instruction states only the goal: "Turn the key to ON."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the lock's working, the scorer (`score`, `success`) and the scripted `demo_script` |
| `wheel_colliders.json` | Convex collider hulls for the steering wheel, taken from the SimReady wheel's own mesh (40 points each) |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `simready.py` | Loads SimReady props (the distractors) and fetches the wheel's files, unchanged |
| `task.json` | The task's details for the board |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the key to ON." --scorer task_score --max-steps 2400
```

The scene is the same for every seed (nothing is randomised). Runtime: the scripted demo reaches full credit after 19.45 s of simulated
time (389 steps). With cameras, a shared GPU ran 2 to 3 steps a second, so a 2,400-step trial can take up to about 20 minutes of wall time; an
idle card is faster.

## The demo

```bash
python demokit.py task.py OUT                                           # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --no-record --mode none,onearm,wrongway,acc,hold   # the failure checks
python demokit.py task.py OUT_pull --fast --mode pull                   # a check of the lock itself (CPU)
```

The demo is a scripted reference, not a policy. It reads the wheel's angle (in view of the cameras) and its own arms' joints, and takes
the key's and the rim's places from the published starting layout; it never reads the scorer.

1. The left hand takes the rim at 10 o'clock, coming from the driver's side along the column with its fingers either side of the rim's
   section, and turns the wheel 4 degrees anticlockwise, off the notch's wall. It keeps it there against the spring, closed loop on the
   wheel's seen angle, for as long as the right hand works.
2. The right hand comes down over the key's head from above, pinches it across its 11 mm and turns it clockwise about the key's axis,
   through ACC to 67 degrees. It lets go in the ON detent and moves out of the way, outside the rim.
3. The left hand lets the wheel unwind to where the tyres hold it, lets go and moves back off the rim.

The scorer needs the key's head clear of both hands for 2 s, so full credit comes 2 s after the right hand opens, while the left hand is
still on the rim (the wheel is not scored); the trial ends there, before step 3.

Results with 8 N m on the bolt at the start, in GPU physics with cameras, with the final scene and scorer. The two runs without a mode name
ran as their own task files (copies of `task.py` with another `demo_script`); the others are modes of `task.py` (`--mode`).

| run | what the script does | score |
|---|---|---|
| `full` | as above | 1.0, in 19.45 s (389 steps); twice in a row |
| one hand, patient | the right hand only turns the key, keeping the torque through the key at 1.6 to 2.0 N m for 80 s | 0.0: the key stays at about 6 degrees and the bolt does not move (at most 0.1 mm) |
| careless | the left hand turns the wheel 4 degrees off the bolt and lets go; then the right turns the key | 0.0: the wheel springs back onto the bolt; the key does not turn and is forced (2.8 N m) |
| `none` | does nothing | 0.0 |
| `onearm` | turns the key with the right hand only, the wheel left loaded | 0.0: the key does not turn; it is forced (3.5 N m) |
| `wrongway` | the left hand holds the wheel 4 degrees clockwise, onto the bolt, while the right turns the key | 0.0: the key does not turn; it is forced (2.8 N m) |
| `acc` | as `full`, but lets go of the key at ACC | 0.5 |
| `hold` | as `full`, but keeps the key pinched at ON | 0.0 |

`pull` turns off the cam's drive on the bolt and pulls the bolt back with a growing force while the wheel presses it with 8 N m: the
bolt does not move until the pull reaches about 250 N (GPU; about 225 to 250 N on CPU). The key's cam can pull it with at most 120 N (the
key's forcing torque through the cam), so with the wheel loaded the key cannot free the bolt.

Model run: Dream-1, with the command under "Run a trial" (2,400 steps), scored 0.0. It moved both arms about the wheel and pushed the
service book on the ledge, but never took the rim or touched the key.

How much load the YAM can relieve: the same `full` demo, with the bolt's load changed, scored 1.0 at 10 and 12 N m (GPU, one run each;
held 4 degrees off the wall, the wheel then pulls back with 12 and 14 N m, about 80 and 90 N at the rim). At 14 N m the left hand lost
the rim and the trial scored 0.

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold.

- **1.0** once the key has rested in its ON detent (60 to 70 degrees) for 2 s (40 control steps in a row) with every one of both
  grippers' fingertip points (`mechkit.fingertips`: tip and 3 cm up each finger, middle and both edges) at least 3 cm from the key's head.
  It stays 1.0 from then on.
- **0.5** if, at the end, the key rests in ACC (25 to 40 degrees) with both grippers 3 cm clear of its head.
- **0** otherwise: the key short of ACC, between ACC and ON, or at ON but held by a gripper.
- **0 whatever else happens** if the key was forced before full credit: more than 2.45 N m through the key's cam for 3 control steps in a
  row (0.15 s). 2.45 N m is the torque that UNECE Regulation 116 has a locked cylinder resist without its key. The record is cleared only
  at a reset.

A turn to START is allowed: the key springs back to ON. The wheel is not scored: a hand may still hold it at the end, at any angle (with the
key at ACC or beyond, the bolt is fully drawn and the wheel no longer acts on the key).

Real trial: photograph the key's head against the bezel marks at the end (the marks are at 0, 30, 65 and 88 degrees), and check the
grippers' distance on the video. A forced key shows as a bent or snapped key, or in the wrist's torque log.

## The model

- **The column** (`column`): a fixed frame (the shroud and the ignition's housing) and one link, the steering wheel with the steering shaft
  and the lock collar. The wheel turns on the column's axis, positive clockwise as the driver sees it, limited to plus or minus 25 degrees:
  the steering's travel beyond the lock's play is cut there.
- **The tyres' wind-up** is the wheel joint's spring: 0.5 N m per degree, at rest 16 degrees clockwise past the notch's wall, so the wheel
  presses the bolt with 8 N m at the start. Held 4 degrees off the wall it pulls back with 10 N m.
- **The lock** (`lock`, a second articulation so that its bolt collides with the collar): the bolt, a slider into the collar's notch
  (8 x 10 mm, 8 mm of travel), and the key, a hinge in the ignition about an axis parallel to the column's. The notch's walls are planes
  parallel to the bolt's sides; between them the wheel has 8 degrees of play. The bolt is pinched by real contact with the wall (friction
  0.65 static, 0.55 dynamic: dry steel on steel) and by its guide's joint friction, which PhysX scales with the load through the joint.
- **The key's cam** (code, each control step): it draws the bolt back 8 mm as the key turns from 2.5 to 25 degrees (0.36 mm a degree). The
  bolt's drive is sent to the cam's position for the key's angle, but never pulls it with more than 120 N (the key's forcing torque through
  the cam). While the bolt is short of where the cam puts it, the key is held at the angle the bolt's actual position allows, with the
  stiffness of a rigid cam (41.5 N m per radian): the key turns only as far as the bolt moves. The torque this takes from the key is what the
  scorer records as forcing.
- **Key detents**: springs to the nearest of LOCK 0, ACC 30 and ON 65 degrees (0.5 N m per radian); past 80 degrees (START, to 92) a
  1.5 N m per radian spring returns it to ON.

### The load

The 8 N m on the bolt and the 0.5 N m per degree are estimates, not measurements. A design figure for electric column locks is that they
must free a column carrying up to 50 N m of residual torque (Delphi's patent application EP 1 442 948: "unlock columns with residual torque up to 50 newtons
meters"), the worst case of wheels turned hard against a kerb; a wheel moved by hand after parking stores much less. To check the figure
for a particular car: parked on level ground with the wheels straight, take the key out, turn the wheel until the lock catches and lean on
it as a driver getting out would; then hook a luggage scale on the rim at 9 o'clock and pull anticlockwise slowly, noting the reading at
which the key turns. Repeat several times. `TAU_PRELOAD` is the median times the rim radius, and `K_WHEEL` comes from the scale's reading
4 degrees past the release point.

## Real trials: the build

- **Column.** A keyed steering column with its ignition lock and wheel from a scrapyard car, bolted to a bench at 25 degrees from
  horizontal, the wheel's centre 22 cm above the bench and 25 cm in front of the line of the arms' bases. A torsion spring (or a lever
  and an extension spring) on the column's lower end stands in for the tyres: set it so that the wheel presses the bolt with 8 N m (52 N
  on a luggage scale at the rim at 9 o'clock, 15.5 cm out), rising 0.5 N m per degree turned against it. The key's head should be about
  32 x 26 x 11 mm.
- **Dash.** A board face 50 cm wide and 30 cm tall, 50 cm in front of the arms' bases, with a 22 cm deep ledge on top.
- **Distractors.** Sunglasses on the ledge, a takeaway coffee cup with its lid beside the dash, and a service book on the bench, as in
  `setup`.

## Assets

The steering wheel and the distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`steeringSystem_steering_wheel_e01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/steeringSystem_steering_wheel_e01)
- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01)
- [`food_beverage_coffeeCup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_coffeeCup_a01)
- [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:

- the steering wheel's meshes and materials are shown inside the wheel link of the column mechanism: its own rigid bodies, joints
  (its spoke buttons are on joints in the file) and colliders are left out, and it collides through 29 convex hulls computed from its own
  mesh (`wheel_colliders.json`: the rim in 24 sections, the three spokes, the hub and the boss). Its mass on the joint is set to 1.0 kg.
- the sunglasses' and the cup's parts are welded into one body each; masses are set to 30 g (sunglasses), 250 g (cup with coffee) and
  120 g (book).
- the invisible grasp-annotation curves are deactivated.

The column, lock, key and dash are custom primitives (`mechkit.py` meshes). The files are not committed: they download on first use.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source tasks in this repository (`mechkit.py` as in
`tasks/heat-shrink-lug/`, `demokit.py` as in `tasks/heat-shrink-lug/`); `simready.py` is adapted from `tasks/cutters-off-pegboard/`.
