# Drive a toy remote-control car into its shoebox garage with the two-lever remote

A toy RC car stands on the table, out of the arms' reach, its nose to the far edge. Its garage is a children's shoebox base, lid off
and open side up, with a door cut in one end, at the right of the table, its door facing the car's side. The car's remote lies between the arms in
a cardboard cradle: the **left lever is the throttle** (push for forward, pull for reverse) and the **right lever is the steering** (left
or right). The instruction is only the goal: *"Drive the toy car into its garage with the remote control."*

The knack is the one every child learns with a cheap RC car: both levers are on/off switches, so speed and line come only from tapping,
and both thumbs work at once. The car can't turn tighter than a 34 cm radius, so it can't swing round to face the door in the room it
has between the far edge and the robot's own side of the table: it has to go forward on one lock, reverse on the other (which keeps it
turning the same way), and then thread a 13 cm door with 2 cm to spare on each side, stopping inside a 27 cm box without hitting the back
wall hard or leaving the car half out. Driving it straight off the table, or reaching out and carrying it in, scores nothing.

| File | What it is |
|---|---|
| `task.py` | The scene, the toy car's behaviour, the scorer and the closed-loop demo (`demo_script`, with failure modes) |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `simready.py` | Loads the SimReady distractors (see Assets) |
| `task.json` | The task's details for the board |

## Requirements

- The `yam_arena` embodiment for `inspect-robots` (Isaac Lab-Arena), as in the program's walkthrough.
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Drive the toy car into its garage with the remote control." --scorer task_score --max-steps 900
```

The scripted, closed-loop demonstration (it reads only what a camera could: the car's pose and the levers' angles):

```bash
python demokit.py task.py ~/rc-car-demo                         # the recorded run: ~/rc-car-demo/full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/rc-car-checks --no-record --mode none,onearm,grab,edge,careless,half,lined
```

Nothing is randomised, so every seed gives the same scene. Runtime per trial: about 3-5 minutes of wall time on one L40 (Isaac start
plus up to 900 steps at about 3-5 steps/s); the demo reaches full credit after 16.35 s of simulated time (327 steps), identically in
two recorded runs. The run limit, 900 steps (45 s), is about 2.75 times that.

## The demo

`demo_script` in `task.py`:

1. The left hand comes in from behind and pinches the throttle's ball across the lever's pivot axis (fingers closing left-right); the
   right hand comes in from its own side, tilted 35 degrees outward so the two wrists stay clear of each other, and pinches the
   steering's ball across its pivot axis (fingers closing front-back). Each ball turns freely between the pads as its lever tilts.
2. Every control step, both hands put their ball where the drive needs it: 19 degrees over (past the 10-degree switch, short of the
   22-degree stop) or centred, plus a correction learnt from how far the lever is seen to lag (the arms are compliant). The throttle is
   tapped to hold a set speed read from the car's seen motion (0.20 m/s in the shunt, 0.22 then 0.15 m/s towards the door), the way a
   person taps a toy car's throttle.
3. Steering to full right lock first, then throttle forward, until the car has turned 30 degrees (or its front wheels come within 10 cm
   of the far edge); then full left lock and reverse, which keeps it turning clockwise, until it nearly faces the door (heading -80
   degrees: it coasts a few degrees further). If it isn't round yet it shunts again.
4. Forward towards the door, steering by taps onto the garage's centre line (a heading set from the car's sideways offset), until its
   tail is just inside the door; it coasts on a few cm. Then both levers are released and the car stops. Full credit comes 1 s after it
   stops, with the hands still resting on the centred levers, and the run ends there.

Failure modes, run with `--mode`: `none` does nothing (0); `onearm` works the throttle only and drives off the far edge (0); `grab`
reaches for the car to carry it in (0: the car stands out of the arms' reach, about 0.77 m beyond what the hand can get to; a finger
within 1 cm of it would also score 0); `edge` holds the throttle on and goes off the table (0);
`careless` takes the turn on one lock with no shunt (0: off the edge, or not lined up); `half` stops with the car half through the door
(0.5); `lined` stops it lined up about 15 cm short of the door (0.25).

## Scoring

`score()` and `success()` read a tracker (`Trial` in `task.py`) that runs after every control step. Measured at the end of the run:

- **1.0**: the whole car (all four corners of its 18 x 8.8 cm footprint) is inside the garage (behind the door's face at y = -0.36 m,
  between the side walls, in front of the back wall), both levers are released (back within 5 degrees of centre), the car has been still
  (under 1 cm/s, its motor off) for 1 s, and no gripper finger is within 3 cm of the car. The run ends with success when this has held
  for 1 s.
- **0.5**: not 1.0, but the car's centre is inside the garage: at least half the car is through the door.
- **0.25**: neither, but the car is lined up at the door: its centre within 3 cm of the garage's centre line (x = 0.68 m) and no more than
  25 cm out from the door's face, and its length within 15 degrees of the garage's axis (nose or tail first).
- **0** for anything else.
- **0 whatever the end state** if at any moment:
  - a gripper finger came within 1 cm of the car's body (a hand on the car);
  - the car moved (over 5 cm/s, or turned over 0.5 rad/s) for 3 steps running while its own motor was off (pushed or dragged by
    something else);
  - any of its four wheels went past the table's edge. The car is stopped where it went over: the fall itself isn't simulated.

  These are latched in the tracker and cleared only when the scene is reset.

Possible scores: 0, 0.25, 0.5, 1.0. The partial levels are the end state, not the best moment: a car that was half in and then backed
out scores what it ends on.

How the simulator judges each rule: the car's pose is its three joints (x, y, heading) plus its start; its corners and wheel centres
come from its measured footprint and wheelbase. The levers' angles are their hinge joints. "Finger within 1 cm / 3 cm" uses mechkit's 24
fingertip points against the car's 18 x 8.8 x 6.4 cm box. "Still" uses the car's joint velocities and the motor model's own speed.

## The toy car

The car body is one rigid body (190 g) on a planar chain of joints (slide x, slide y, turn about z) fixed to the table frame. Every
control step the remote's two switches drive a small model of a toy car's drive, and the model's velocities become the joints' velocity
targets (`Car.step`):

| Quantity | Value | Why |
|---|---|---|
| Top speed with the throttle on | 0.60 m/s | a 1:24 toy car on its slow setting; real ones are faster on a hard floor, which only makes the task harder |
| Motor spin-up (and reversal) time constant | 0.30 s | a small brushed motor through the toy's plastic gears |
| Coast-down with the throttle off | 0.12 s | the motor's drag brakes the car: a released toy car stops within a few cm |
| Steering lock | 18 degrees | toy cars have little lock: on an 11 cm wheelbase this gives a 34 cm turning radius |
| Steering travel | 0.08 s | a toy car's steering is a spring-centred magnet or servo that snaps to full lock |
| Switch point / lever travel | 10 / 22 degrees | the remote's levers are on/off: past 10 degrees the motor or steering is fully on |
| Push limit of the drive | 2 N, 0.15 N m | what the rubber tyres can push before they slip: a wall stops the car, as it does a real one |

The car can't slip sideways (no lateral velocity is commanded) and its body collides with the garage, the remote, the arms and the props.
These numbers are estimates for this class of toy, not measurements of one car; the real-trial section gives a two-minute check of the
three that matter (top speed, coast-down distance and turning radius) on the car actually used, and the course can be scaled to a
different turning radius.

## Real trials

### The car, the remote and the garage

- **Car and remote**: any 1:24 toy RC car (about 18 x 8-9 cm, 150-250 g) sold with a two-lever remote whose left lever is the throttle
  and right lever the steering, both on/off. Many supermarket and toy-shop cars use exactly this remote; for example Kmart (Anko)
  keycode 43803972, whose manual reads "Push the throttle lever, the vehicle will move forwards. Pull the throttle lever, the vehicle
  will move backwards ... Move the direction lever right, the vehicle will turn right"
  (https://www.kmart.com.au/wcsstore/Kmart/pdfs/43803972_Manual.pdf). Use the slow speed setting if the car has one. Price: about
  US$15-30.
- **Check the car** (2 minutes, tape measure and a phone video): full throttle over 1 m from a standing start, timed on the video (top
  speed); release at full speed and measure how far it rolls (coast-down: about 7 cm here); full lock in a circle, measured across
  (turning radius: 34 cm here). If the turning radius differs by more than 3 cm, scale the car's start and the garage's position with it.
- **Remote**: the simulated remote is 20 x 9.5 x 3.2 cm, 180 g, with its two levers 15 cm apart, each a 28 mm stem with a 20 mm ball
  top (a rubber thumb cap slipped over a flat paddle lever gives the same ball). A remote with levers closer together than about 13 cm
  leaves no room for two grippers side by side.
- **Remote cradle**: a strip of cardboard 21.6 x 11.1 cm with 1.2 cm rims front and back, taped to the table at x 0.27 m, y 0, so the
  remote lies flat and can't slide while the two grippers work its levers (a person holds it in two hands).
- **Garage**: a children's shoebox base about 28 x 17 x 10 cm, lid off, standing open side up (so the car can be seen inside from
  above); cut a door 13 cm wide and 7.5 cm high up from the bottom edge in the middle of one end. Tape its two back corners to the table
  so it can't move. The car rolls over the box's own thin cardboard bottom.

### Layout (from the arm bases: x forward, y left, origin midway between them)

- Car: centre at x 0.58 m, y +0.20 m, nose pointing +x (towards the far edge, 24 cm away). Mark the four tyre contact points with tape.
- Garage: inside 16 cm wide from x 0.60 to 0.76 m, from the back wall at y -0.63 m to the door's face at y -0.36 m, the door facing +y.
- Remote: in its cradle, centre at x 0.27 m, y 0, levers up; left lever (throttle) at y +0.075, right lever (steering) at y -0.075.
- Distractors (not scored): a mug at x 0.84, y +0.36; a puzzle cube at x 0.85, y -0.24. Neither is in the car's way.

### Measurement

At the end of the run, before anyone touches anything:
- Is the car entirely behind the door's face? Look down into the open box, or sight along the door's face with a ruler: no part of the
  car may cross it. If it is, are both levers centred and no gripper finger within 3 cm of the car (a ruler)? Then 1.0.
- Otherwise, is the car's centre (marked on its roof with a dot) behind the door's face? Then 0.5.
- Otherwise, with a tape measure: the roof dot's distance from the garage's centre line (3 cm at most) and from the door's face (25 cm at
  most); with a protractor or a phone level app laid along the car's roof: its angle to the garage's axis (15 degrees at most). All three
  hold: 0.25.
- From the video: did a gripper touch the car, did the car move with its motor off (pushed), or did a wheel go over the table's edge?
  Any of them: 0.

### Reset (about 2 minutes)

Car back on its tape marks, levers centred (they spring back), batteries checked every ten runs (a weak pack slows the car; recheck the
top speed). The garage and cradle stay taped.

### Safety

A toy car of 190 g at under 1 m/s can't hurt anyone. Put a soft-sided box on the floor under the far edge to catch a car that goes over.

## Importance and sources

- **Category: Societal relatability.** Driving a toy RC car, and parking it in a box "garage", is one of the most familiar children's
  games, and most adults have driven a toy remote-control car themselves. Cheap toy RC cars are sold in supermarkets and toy shops, and
  a common control on them is a two-lever on/off remote: a throttle lever and a direction lever, as in the manual of Kmart's keycode
  43803972 (https://www.kmart.com.au/wcsstore/Kmart/pdfs/43803972_Manual.pdf: "Push the throttle lever, the vehicle will move
  forwards ... Move the direction lever right, the vehicle will turn right").
- **Why it's hard for a robot**: two hands, each on its own control, working at the same time; both controls are on/off, so speed and
  line come from timed taps read off the car's motion, with a lag of about 0.3 s from the motor; the car moves only along its heading
  and turns no tighter than 34 cm, so getting it to face a door 90 degrees round in a 40 cm wide strip needs a planned shunt; and the door
  leaves 2 cm each side. Done naively (hold both levers on) the car leaves the table in under a second.
- **Why the end state is fair to measure**: a toy car stopped in its garage stays there, and a car half in or lined up at the door is a
  visibly better attempt than one that is not.

## Distinct from

| Nearest board task | What it tests | How this differs |
|---|---|---|
| #3073 claw machine (accepted) | one spring-centred joystick moving a gantry at a fixed speed in x and y, then a drop button | a car-like vehicle: it moves only along its heading and turns on a radius, so it needs a forward-and-reverse shunt; two levers worked at once by two hands; momentum and coast |
| #2746 pull-back toy car (accepted) | winding a pull-back car and releasing it through a gate | no remote; one release, no closed-loop control after it |
| #2583 table football pass (accepted) | working a table football rod by hand to pass the ball | the rod is held directly; no vehicle, no remote, no non-holonomic manoeuvre |
| #2571 Etch A Sketch (accepted) | two knobs worked together to draw a line | the knobs move a stylus directly in x and y; here two on/off levers drive a vehicle that can't move sideways and keeps rolling |
| #1488 desktop CNC joystick jog (not accepted) | jogging a router over a mark with a joystick | specialist machine; holonomic x/y jog |
| #2849 pushing a toy train round a bend (not accepted) | pushing a train by hand along its track | hands on the toy, no remote |

None of LIBERO, RoboCasa, BEHAVIOR-1K, RoboTwin 2, SimplerEnv, DataCenterBench or RoboARC has a task in which the robot drives a
vehicle through a remote control.

## Assets

The distractors come from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`toys_rubiksCube_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/toys_rubiksCube_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: each prop's parts are
welded into one rigid body (the puzzle cube's 27 cubies included, with convex-hull colliders); masses are set to 300 g (mug) and 100 g (cube).

The car, the remote and its cradle, and the shoebox garage are custom primitives (`mechkit.py` meshes): SimReady has no toy RC car,
remote or shoebox. The SimReady files are not committed: they download on first use.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository, `tasks/heat-shrink-lug/`,
published under the MIT licence; `simready.py` is adapted from `tasks/cutters-off-pegboard/` (loader unchanged).
