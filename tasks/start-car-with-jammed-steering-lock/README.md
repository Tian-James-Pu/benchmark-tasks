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

## Response to review

The review asked for two things. Both are answered below, and the team note is applied.

1. **Scoring: forcing has to be measurable in real trials.** The old rule (more than 2.45 N m through the key for 0.15 s) needed a
   key-axis torque nobody could read on a real rig. It is now replaced, in `task.py` and in `task.json` alike, by a rule made only of two
   angles a camera sees:
   - the key's head at or past its **forcing mark**, and short of ACC's 25 degrees,
   - while the wheel is still **on the bolt** (within 0.5 degrees of its jammed angle),
   - for 0.15 s running (3 control steps; 9 frames of 60 fps video), before full credit. This scores 0.

   The forcing mark is the angle the key's head reaches under 2.45 N m with the wheel jammed. While the bolt is pinched, the key, its
   blade and the cam make a stiff torsion spring, so that angle stands for the torque. On a real rig the mark is calibrated with a luggage
   scale on a 10 cm lever clamped to the key's head (under "Real trials: calibrate the forcing mark"). The simulation runs the same
   calibration (`--mode calibrate`): the key reached 6.78 degrees at 2.45 N m, so the simulated mark is 6.78 degrees.

   The rule was checked against the simulator's own torque log on every failure mode (see the table under "The demo"): the two agreed in
   every run. Two diagnostics in the code, `tau` and `tau_over`, still log the torque, but they no longer count towards the score.
2. **Clarity and reproducibility: a buildable column and preload fixture.** "Real trials: the build" now names the kind of column to buy,
   with two example models, and how to tell whether a column is suitable. It gives a dimensioned lever-and-spring preload fixture with its
   parts, where they go, stops, adjustment steps and target readings, so the wheel presses the bolt with 8 N m and rises 0.5 N m per
   degree. It also lists rig checks that reject an unsuitable column, and a cost list.
3. **Note for the team (shared rig):** the coffee cup is now an empty, dry paper cup with its lid on, ballasted to 250 g with dry sand
   sealed in a bag and taped inside. Its simulated mass (250 g) and its place are unchanged. The scrapyard column must have its airbag
   module removed (see "Safety").

Scene, instruction and layout are unchanged. The scripted demo was run again on the GPU with the new scorer, twice, and scored 1.0 both
times. The failure modes were also run again. The Dream-1 run was not repeated, because nothing it saw has changed.

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

Every seed gives the same scene, because nothing is randomised. Runtime: the scripted demo reaches full credit after 19.45 s of
simulated time (389 steps). With cameras, a shared GPU ran 2 to 5 steps a second, so a 2,400-step trial can take up to about
20 minutes of wall time. An idle card is faster.

## The demo

```bash
python demokit.py task.py OUT                                           # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --no-record --mode none,onearm,careless,wrongway,acc,hold   # the failure checks
python demokit.py task.py OUT_cal --no-record --mode calibrate          # the forcing mark's calibration
python demokit.py task.py OUT_pull --fast --mode pull                   # a check of the lock itself (CPU)
```

The demo is a scripted reference, not a policy. It reads the wheel's angle (which the cameras can see) and its own arms' joints, and
takes the key's and the rim's places from the published starting layout. It never reads the scorer.

1. The left hand takes the rim at 10 o'clock, coming from the driver's side along the column with its fingers either side of the rim's
   section, and turns the wheel 4 degrees anticlockwise, off the notch's wall. It keeps the wheel there against the spring, closed loop
   on the wheel's seen angle, for as long as the right hand works.
2. The right hand comes down over the key's head from above, pinches it across its 11 mm and turns it clockwise about the key's axis,
   through ACC to 67 degrees. It lets go in the ON detent and moves out of the way, outside the rim.
3. The left hand lets the wheel unwind to where the tyres hold it, lets go and moves back off the rim.

The scorer needs the key's head clear of both hands for 2 s, so full credit comes 2 s after the right hand opens, while the left hand is
still on the rim (the wheel is not scored). The trial ends there, before step 3.

All results below use 8 N m on the bolt at the start, GPU physics with cameras, and the final scene and scorer (this revision). "Forced"
is the scored rule; the torque column shows the simulator's own torque log next to it.

| run | what the script does | score | forced (scored rule) | torque log: peak; over 2.45 N m for 0.15 s |
|---|---|---|---|---|
| `full` | as above | 1.0, in 19.45 s (389 steps); twice in a row, recorded | no | 0.00 N m; no |
| `none` | does nothing | 0.0 | no | 0.00 N m; no |
| `onearm` | turns the key with the right hand only, the wheel left loaded | 0.0: the key does not turn | yes | 3.54 N m; yes |
| `careless` | the left hand turns the wheel off the bolt and lets go; then the right turns the key | 0.0: the wheel springs back onto the bolt and the key does not turn | yes | 2.77 N m; yes |
| `wrongway` | the left hand holds the wheel 4 degrees clockwise, onto the bolt, while the right turns the key | 0.0: the key does not turn | yes | 2.66 N m; yes |
| `acc` | as `full`, but lets go of the key at ACC | 0.5 | no | 0.00 N m; no |
| `hold` | as `full`, but keeps the key pinched at ON | 0.0 | no | 0.00 N m; no |

The calibration (`calibrate`) puts no hands on the rig: the wheel stays jammed while a pull 10 cm from the key's axis turns it with
0.6, 1.2, 1.8, 2.45 and 3.0 N m in turn. The key's angles were 4.21, 5.05, 5.89, 6.78 and 7.53 degrees (GPU; the same on CPU), so the mark is 6.78
degrees, and 1.2 N m and 2.45 N m lie 1.73 degrees apart. The forcing record was set at the 2.45 N m step, not before.

`pull` turns off the cam's drive on the bolt and pulls the bolt back with a growing force while the wheel presses it with 8 N m. In the
earlier revision's runs (the lock is unchanged), the bolt did not move until the pull reached about 250 N (GPU; about 225 to 250 N on CPU).
The key's cam can pull it with at most 120 N, the pull of 2.45 N m through the cam, so with the wheel loaded the key cannot free the
bolt. An earlier one-hand run kept the torque through the key at 1.6 to 2.0 N m for 80 s, below the forcing mark. It scored 0: the key
stayed at about 6 degrees and the bolt moved at most 0.1 mm.

Model run: Dream-1, with the command under "Run a trial" (2,400 steps), scored 0.0. It moved both arms about the wheel and pushed the
service book on the ledge, but never took the rim or touched the key. The scene and instruction are unchanged in this revision, so that
run still stands.

How much load the YAM can relieve: the same `full` demo, with the bolt's load changed, scored 1.0 at 10 and 12 N m (GPU, one run each,
earlier revision). Held 4 degrees off the wall, the wheel then pulls back with 12 and 14 N m, about 80 and 90 N at the rim. At 14 N m the
left hand lost the rim and the trial scored 0.

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold.

- **1.0** once the key has rested in its ON detent (60 to 70 degrees) for 2 s (40 control steps in a row) with every one of both
  grippers' fingertip points (`mechkit.fingertips`: the tip and 3 cm up each finger, at the middle and both edges) at least 3 cm from the
  key's head. It stays 1.0 from then on.
- **0.5** if, at the end, the key rests in ACC (25 to 40 degrees) with both grippers 3 cm clear of its head.
- **0** otherwise: the key short of ACC, between ACC and ON, or at ON but held by a gripper.
- **0 whatever else happens** if the key was forced before full credit. Forced means all three of these hold together for 3 control
  steps in a row (0.15 s):
  - the key is at or past the forcing mark (6.78 degrees) and short of 25 degrees;
  - the wheel is within 0.5 degrees of its jammed angle (still on the bolt);
  - full credit has not yet been reached.

  The forcing mark is the key's angle under 2.45 N m with the wheel jammed. 2.45 N m is the torque that UNECE Regulation 116 has a
  locked cylinder resist without its key. The jammed angle is the wheel's angle in the settled starting layout, read at the reset. The
  record is cleared only at a reset (`on_reset`).

Past 25 degrees the cam has drawn the bolt fully, and the wheel no longer acts on the key, so that range cannot be forcing. A wheel held
more than 0.5 degrees off its jammed angle is off the notch's wall, so the bolt is free and the key turns at well under 2.45 N m. A turn
to START is allowed, and the key springs back to ON. Beyond the forcing rule the wheel is not scored: a hand may still hold it at the
end, at any angle.

## Real trials

The scoring above is read in a real trial from three things: the key's angle, the wheel's angle and the grippers' distance from the
key's head. All three come from video plus the marks set up below. The ordinary tools needed are a digital luggage scale (0.01 kg
resolution, up to 25 kg or more), a tape measure, a steel rule, a protractor or a phone inclinometer app, a marker, masking tape, two
binder clips, and two phones on small tripods that film at 60 fps.

### The build

**Column (the main mechanism).** Use a car's upper steering column with its own key-operated column lock, ignition lock cylinder, key
and steering wheel. The lock must be the mechanical kind, a spring-loaded bolt in the lock housing that drops into a collar on the
steering shaft when the key is out. Almost all cars with a turn-key ignition and no push-button start have one. Two common examples are
the 2003 to 2008 Toyota Corolla and the 2001 to 2005 Honda Civic, but any column will do if it passes the checks under "Rig checks".
Self-service scrapyards sell the upper column as one item. Ask for the lock cylinder's key, or buy the column with the cylinder and have
a key cut to it. No battery or wiring is needed: the lock is purely mechanical.

- Remove the airbag module from the wheel's hub (see "Safety"). The hub may be left open.
- If the key's head differs much from the simulated head (32 x 26 x 11 mm, black plastic), note its size with the results.
- If the wheel's size differs from the simulated wheel (35.0 x 32.8 cm, rim section 3 x 4 cm), measure its rim radius R: from the hub's
  centre to the middle of the rim's section, at 3 o'clock. Set every load below as a torque, using the scale readings for that R:
  reading in kg = torque in N m / (9.81 x R in m).

**Stand.** The stand holds the column at its real rake, 25 degrees from horizontal and rising towards the robot. Make it from 18 mm
plywood and screw it to the bench:

- a base board 60 x 25 cm;
- two side cheeks cut as right-angled wedges, 45 cm long with 25 degrees at the foot, with a top board 45 x 20 cm screwed across them;
- the column's own upper mounting bracket (the one that bolted to the car's dash beam), bolted to the top board with M8 bolts, washers
  and nyloc nuts through its own holes;
- the lower end supported in a 50 mm pipe clamp on a wooden block.

Pack the base with plywood spacers until the hub's centre is 22 cm above the bench (tape measure). Set it 25 cm in front of the line
through the two arms' bases, on the midline between them, with the ignition on the robot's right. Check the rake on the shroud's top
with a phone inclinometer or a protractor: 25 degrees, within 1 degree.

**Dash.** A board face 50 cm wide and 30 cm tall, 50 cm in front of the arms' bases, with a 22 cm deep ledge on top, both of 18 mm
plywood painted matt black.

**Preload fixture (the tyres' wind-up).** A lever on the bottom of the steering shaft, pulled by an extension spring through a
turnbuckle. The target is 8 N m pressing the wheel clockwise (as the driver sees it) onto the bolt, rising 0.5 N m for every degree the
wheel is turned anticlockwise.

| part | specification |
|---|---|
| shaft clamp | a two-piece steel clamping shaft collar with tapped holes in its face, bored to fit a round part of the shaft below the column's lower universal joint (usually 18 to 20 mm; take a calliper or the column to the shop) |
| lever | steel flat bar 30 x 6 mm, 160 mm long. One end is bolted to the collar's face with two M5 screws. Drill 6.5 mm holes at 80, 100 and 120 mm from the shaft's centre |
| spring | steel extension spring with hook ends, rate 2.0 to 4.5 N/mm. Initial tension under 60 N. It must stretch a further 0.45 x L past its length at the start (45 mm at the 100 mm hole) and stay within its safe working load of 20.5 N m / L: 260 N at 80 mm, 205 N at 100 mm, 171 N at 120 mm. Run a 3 mm steel safety cable through it |
| turnbuckle | M8 hook-and-eye turnbuckle with a working load of at least 300 N, between the spring and the anchor |
| anchor | M8 eye bolt through the stand's base, placed so that the spring's line meets the lever at 90 degrees (check with a square) with the wheel jammed, and so that the spring and turnbuckle together are at least 250 mm long |
| stops | two 40 x 40 mm hardwood blocks screwed to the stand, where the lever meets them at 25 degrees either side of its jammed position (52 mm along the arc at the 120 mm hole). They stand in for the steering's travel, which the simulation also cuts at 25 degrees |
| angle pointer | a 300 mm steel rule, clamped to the lever with two binder clips, its tip 250 mm from the shaft's centre over a millimetre scale glued to the stand. 0.5 degrees is 2.2 mm at the tip |

Pick the lever hole for your spring's rate k (N/mm). The torsional rate is k x L squared, and you want 28.6 N m per radian (0.5 N m per
degree):

| spring rate k | lever hole L |
|---|---|
| 4.5 N/mm | 80 mm |
| 2.9 N/mm | 100 mm |
| 2.0 N/mm | 120 mm |

At the 100 mm hole, the spring pulls 80 N with the wheel jammed and 5.0 N more per degree.

A real collar may have more than one notch: use the notch the bolt finds nearest the wheel's straight-ahead position, and the same one
in every trial.

### Preparation and adjustment

Do this once per rig, and again whenever anything is moved. The scale readings are for R = 15.5 cm, as in the simulation.

1. **Find the jammed position.** Take the key out (the bolt springs out). Turn the wheel slowly until the bolt drops into the collar's
   notch. The wheel then rocks only within the notch's play. Turn it clockwise to the end of that play: this is the jammed position.
   Mark it as 0 on the pointer's millimetre scale. Put a strip of masking tape on the rim at 3 o'clock.
2. **Fit the stops** at 25 degrees either side of the jammed position (step 1's 0).
3. **Set the preload.** Insert the key and turn it to ON, which draws the bolt. Hook the spring onto the chosen lever hole and the
   turnbuckle onto the anchor. The spring now holds the lever against the clockwise stop.
   - Loop the luggage scale's strap round the rim at 3 o'clock and pull along the rim's tangent, in the wheel's plane, anticlockwise.
     That is upwards along the wheel's face.
   - Read the scale with the pointer at 0. Adjust the turnbuckle until it reads 8 N m: 5.26 kg (51.6 N).
4. **Check the rate.** Keep pulling until the pointer reads 10 degrees anticlockwise (43.6 mm at the pointer's tip). The scale should read
   13 N m: 8.55 kg (83.9 N). If it is more than 10 % off, move the spring to another lever hole and repeat from step 3. Write both readings
   down.
5. **Jam it.** Turn the key back to LOCK and leave it in. Turn the wheel until the bolt drops into the notch, then let go. The spring puts
   the notch's wall onto the bolt, and the pointer reads 0 within 1 mm. Pull at 3 o'clock again: the wheel should start to move off the
   wall at 5.26 kg, within 0.3 kg.

### Rig checks

These check the column before any trial. They need the forcing-mark lever from the next section, and take about 5 minutes.

- **Jammed, the key must not reach ACC under 2.45 N m.** In the forcing-mark calibration below, the key stays short of ACC. If it gets
  there, this column's lock frees itself at a lower preload than the simulation's. Raise the preload in 1 N m steps, up to 12 N m (the
  simulated demo still scored 1.0 at 12 N m), and record the value used. If it still reaches ACC at 12 N m, the column is unsuitable.
- **Relieved, the key must turn freely.** Pull the rim at 3 o'clock with the scale until the pointer reads 4 degrees anticlockwise (17.5
  mm), and hold it there. The key, turned by hand with the lever, must go from LOCK through ACC to ON with under 0.5 kg on the scale at 10
  cm (0.5 N m).
- **The shaft clamp must not slip.** Draw a paint line across the collar and the shaft, and check that it is unbroken after the checks.

### Calibrate the forcing mark

The mark is the key's angle under 2.45 N m with the wheel jammed. While the bolt is pinched, the key, its blade and the cam form a stiff
torsion spring. A nickel-silver blade 8 x 2.2 mm in section and 25 mm long twists about 1.3 degrees per N m on its own (shear modulus 48
GPa), so the angle reads the torque. The simulation runs the same procedure with `--mode calibrate`.

1. Print a protractor scale with 1-degree marks and a 35 mm radius on paper. Stick it to the ignition's face round the key, with 0 on
   the head's centre line at LOCK.
2. Set the angle camera (phone 1) on its tripod in the place it will have in trials. It should look at the key's head from the driver's
   side and above the right arm's base, within 30 degrees of the key's axis and out of the arms' way, filming at 60 fps.
3. With the rig jammed (step 5 above) and the key at LOCK, clip a 150 mm steel rule along the long side of the key's head with two binder
   clips, so that it sticks out past the head. Hook the luggage scale's strap on the rule 100 mm from the key's axis.
4. Pull at 90 degrees to the rule, clockwise as the driver sees it, slowly up to 2.50 kg (24.5 N, so 2.45 N m). Hold for 2 s while
   phone 1 films, and read the head's edge on the printed scale to 0.5 degrees. Let go.
5. Repeat step 4 three times. The readings must agree within 1 degree; if they do not, look for a loose clip or cylinder play and start
   again.
6. Do the same at 1.25 kg (1.2 N m). The 2.45 N m angle must be at least 1 degree past it, or the mark cannot separate the two.
7. Draw the forcing mark on the printed scale at the mean of the 2.45 N m readings. From phone 1's video, save a frame of the head at the
   mark: that frame is the reference for the head's edge, seen from the trial camera. Remove the rule and clips before any trial.

### Measurement during a trial

- **Phone 1** (key) films the key's head and the printed scale at 60 fps. **Phone 2** films the pointer's tip over its millimetre scale
  at 60 fps. Before the robot starts, tap the rim once in view of both so the videos can be lined up to a frame.
- **Forced:** in 9 or more frames running (0.15 s), the head's edge is at or past the forcing mark and short of ACC (25 degrees) in
  phone 1, while phone 2's pointer is within 2.2 mm (0.5 degrees) of 0. Any such run before full credit scores 0.
- **Key position at the end:** read the head against the printed scale and the bezel marks (LOCK 0, ACC 30, ON 65, START 88 degrees).
  ON is 60 to 70 degrees, ACC 25 to 40.
- **Hands clear:** in phone 1's video, no part of either gripper comes within 3 cm of the key's head for the 2 s (120 frames) the key is
  resting at ON. The printed scale's 35 mm radius gives the size reference.
- **Hold time:** count frames: 2 s is 120 frames at 60 fps.

### Reset between trials

1. Turn the key back to LOCK. If the wheel was left away from the notch, the bolt rides on the collar for now.
2. Turn the wheel until the bolt drops into the notch, and let go. The pointer must read 0 within 1 mm.
3. Pull at 3 o'clock: the wheel leaves the wall at 5.26 kg, within 0.3 kg. Adjust the turnbuckle if not.
4. Inspect the key. Laid flat on the bench, the head and blade must not rock, and the blade must show no twist against the steel rule.
   The key must still enter the cylinder fully and turn LOCK to ON with the wheel relieved. If it fails any of these, replace it with a
   duplicate cut from the same blank and calibrate the forcing mark again.
5. Put the distractors back on their tape outlines.

Nothing is consumed between trials.

### Safety

- **Airbag.** Never fit or leave a live airbag module in the wheel. Have the yard remove it, or remove it with the column off any power
  following the car's workshop manual, and store or dispose of it as the yard advises. The rig needs only the wheel, the column and the
  lock.
- **Spring.** At the stops the spring stores up to about 10 J. Keep the safety cable through it, keep hands away from the lever and stops
  while the wheel moves, and fit the lever below the bench top where the robot cannot reach it.
- **Cup.** The paper coffee cup is empty and dry, with its lid on, and ballasted to 250 g with dry sand sealed in a small bag taped inside
  it. A spill can never reach the rig or the robot.
- The robot's forces stay well under what a person uses on a steering wheel. The scored loads are 8 to 20 N m at the wheel, under 3 N m
  at the key.

### Approximate cost (excluding the robot)

| item | approx. US$ |
|---|---|
| used upper column with lock, cylinder, key and wheel (self-service yard) | 40 to 100 |
| key duplicate cut to the cylinder, if needed | 5 to 15 |
| clamping shaft collar, flat bar, M5 and M8 hardware | 25 |
| extension spring, safety cable, turnbuckle, eye bolt | 20 |
| plywood (stand and dash), pipe clamp, hardwood blocks | 30 |
| digital luggage scale, steel rules, binder clips, printed scales | 20 |

## The model

- **The column** (`column`): a fixed frame (the shroud and the ignition's housing) and one link, the steering wheel with the steering shaft
  and the lock collar. The wheel turns on the column's axis, positive clockwise as the driver sees it, limited to plus or minus 25 degrees:
  the steering's travel beyond the lock's play is cut there (the real rig's stops).
- **The tyres' wind-up** is the wheel joint's spring: 0.5 N m per degree, at rest 16 degrees clockwise past the notch's wall, so the wheel
  presses the bolt with 8 N m at the start. Held 4 degrees off the wall it pulls back with 10 N m.
- **The lock** (`lock`, a second articulation so that its bolt collides with the collar): the bolt, a slider into the collar's notch
  (8 x 10 mm, 8 mm of travel), and the key, a hinge in the ignition about an axis parallel to the column's. The notch's walls are planes
  parallel to the bolt's sides; between them the wheel has 8 degrees of play. The bolt is pinched by real contact with the wall (friction
  0.65 static, 0.55 dynamic: dry steel on steel) and by its guide's joint friction, which PhysX scales with the load through the joint.
- **The key's cam** (code, each control step): it draws the bolt back 8 mm as the key turns from 2.5 to 25 degrees (0.36 mm a degree). The
  bolt's drive is sent to the cam's position for the key's angle, but never pulls it with more than 120 N (the pull of 2.45 N m through
  the cam). While the bolt is short of where the cam puts it, the key is held at the angle the bolt's actual position allows, with the
  stiffness of a rigid cam (41.5 N m per radian): the key turns only as far as the bolt moves. That stiffness is what makes the key's angle
  a reading of its torque while the bolt is pinched (the forcing mark).
- **Key detents**: springs to the nearest of LOCK 0, ACC 30 and ON 65 degrees (0.5 N m per radian); past 80 degrees (START, to 92) a
  1.5 N m per radian spring returns it to ON.

### The load

The 8 N m on the bolt and the 0.5 N m per degree are estimates, not measurements. A design figure for electric column locks is that they
must free a column carrying up to 50 N m of residual torque (Delphi's patent application EP 1 442 948: "unlock columns with residual
torque up to 50 newtons meters"), the worst case of wheels turned hard against a kerb; a wheel moved by hand after parking stores much
less. To check the figure for a particular car: parked on level ground with the wheels straight, take the key out, turn the wheel until
the lock catches and lean on it as a driver getting out would; then hook a luggage scale on the rim at 3 o'clock and pull anticlockwise
slowly, noting the reading at which the key turns. Repeat several times. `TAU_PRELOAD` is the median times the rim radius, and `K_WHEEL`
comes from the scale's reading 4 degrees past the release point.

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
- the sunglasses' and the cup's parts are welded into one body each; masses are set to 30 g (sunglasses), 250 g (the cup, empty and
  ballasted with sand) and 120 g (book).
- the invisible grasp-annotation curves are deactivated.

The column, lock, key and dash are custom primitives (`mechkit.py` meshes). The files are not committed: they download on first use.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository, `tasks/heat-shrink-lug/`,
which is published under the MIT licence; `simready.py` is adapted from `tasks/cutters-off-pegboard/` (loader unchanged). Toyota and Honda are trademarks of their owners, named only to identify compatible
parts.
