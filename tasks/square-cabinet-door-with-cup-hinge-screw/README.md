# Square a crooked cabinet door with its cup hinge's side screw

A wall cabinet's door hangs crooked on its two concealed cup hinges: shut, the gap to the next door is 1.6 mm at the
bottom and 2.8 mm at the top. Every cup hinge has a side-adjustment screw on its arm, inside the cabinet, that moves the
door sideways at that hinge. Turning the lower hinge's screw clockwise draws the bottom of the door towards its hinge side
and opens the bottom of the gap. The robot has to:

- **choose the screw and the way to turn it.** The gap is narrow at the bottom, so it is the lower hinge, turned
  clockwise. Turning the upper screw, or the lower one the other way, makes the gap worse or moves it out of range.
- **seat a PH2 screwdriver square in a recess 6 mm across**, inside a cabinet whose door stands open, at the screw's
  height 11 cm above the bench, with the screwdriver level and pointing at the side panel.
- **push and turn at the same time.** The screw turns only while the screwdriver is pushed in with at least 10 N and
  held within 6 degrees of the screw's axis. Pushed too lightly, tilted or off the recess, the screwdriver turns without
  the screw (cam-out). The screw resists with 0.2 N m, which the grip has to carry.
- **turn it by the right amount,** between about half a turn and one turn (1 mm of movement per turn at the hinge), which
  takes more than one twist of a wrist. That means easing the push off, winding the wrist back with the tip still in the
  recess, then pushing and turning again.
- **shut the door and let go.** The gap can only be read with the door shut.

The instruction gives only the goal: "The right-hand cabinet door hangs crooked, so the gap between the two doors is
uneven. Make the gap even, 2 to 3 mm wide from top to bottom, and leave the door shut."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the hinges' working, the screw rule, the scorer (`score`, `success`) and the scripted `demo` |
| `task.json` | The task's details for the board |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged |
| `simready.py` | Loads the SimReady props (the screwdriver and the distractors), unchanged |

## An independent implementation of an earlier idea

This is an independent implementation of an idea first submitted in PR #3101. That submission was not accepted, and its
48-hour hold lapsed without a revision. Nothing here is copied from it: the scene, the mechanism, the screw rule, the
scorer, the demo and the text are our own. Its review asked for three things, and they are handled as follows.

1. **The screw turns only with the screwdriver realistically aligned and seated in the recess.** The recess is real
   geometry: four sloping flanks into a 6 mm mouth, 3 mm deep, on the screw's head. The screwdriver is the SimReady PH2
   screwdriver colliding with its own mesh (signed distance field), so its real tip has to go into the recess. The screw
   counts as engaged only while the tip is within 0.8 mm of the screw's axis and the screwdriver's axis is within 6 degrees
   of the screw's. Pushed up to the head but off the recess, the tip rests on the hinge arm and nothing turns.
2. **Representative axial pressure and turning effort, force-gated.** The recess sits on a 20 N/mm spring along the
   screw's axis, so its give measures the push. The screw turns only with at least 10 N behind it (the recess down
   0.5 mm or more). While it turns, the screw resists with 0.2 N m (3 N m against the end of its travel). That torque is
   applied to the screwdriver's body, so the gripper must hold it hard enough to turn it. The screw follows the
   screwdriver's own rotation about its axis, not the wrist's, so a screwdriver slipping in the grip turns nothing. Our
   red-team runs show that turning without pushing scores 0, and that turning with the gripper swung 20 degrees off
   the axis scores 0 (below).
3. **Feasibility under that rule, within the run limit.** The scripted demo meets the rule as it stands and scores 1.0 in
   about 45 s, inside the 2,400-step (2 minute) run limit. It pushes and turns through a full 220-degree twist of the
   wrist, eases off and winds back with the tip in the recess, and turns 90 degrees more. Twice in a row it scored 1.0.

Two more differences from the earlier submission. The door here starts open, because the side screws are reached only
with it open, and it has to be shut at the end. And the screw's end of travel is a stop (Blum's screw has one), not a
stripping rule.

## Importance and sources

- **The occupations.** Carpenters held [about 889,700 US jobs in 2025](https://www.bls.gov/ooh/construction-and-extraction/carpenters.htm),
  and the same Bureau of Labor Statistics page says some carpenters "install drywall or kitchen cabinets in homes".
  O*NET's [construction carpenter](https://www.onetonline.org/link/summary/47-2031.01) task list includes "Install
  structures or fixtures, such as windows, frames, floorings, trim, or hardware, using carpenters' hand or power tools"
  and "Build or repair cabinets, doors, frameworks, floors, or other wooden fixtures used in buildings".
  [General maintenance and repair workers](https://www.onetonline.org/link/summary/49-9071.00) (1,621,800 employed in
  2025, per O*NET) have the task "Adjust functional parts of devices or control instruments, using hand tools, levels,
  plumb bobs, or straightedges."
- **The real operation.** Blum's adjustment sheet for its hinges says: "Rotate front screw to increase or decrease door
  overlay (±2 mm)" ([Blum, Inc., 2021, retailer-hosted copy](https://cdn.shopify.com/s/files/1/0709/8495/8225/files/Blum_Straight_Arm_Hinge_Adjustments.pdf)).
  Blum's [CLIP top page](https://www.blum.com/tw/en/products/hingesystems/clip-top/assembly/) advertises "Easy side,
  height and infinitely variable depth adjustment with spiral screw."
  *Inference (ours):* squaring a door's gaps with these screws is the last step of hanging any door on cup hinges, and is
  repeated when a door drops out of line. Hanging a door is regular core work for cabinet-installing carpenters. That it
  is regular, core work is our reading of the sources above, not a statistic about how often it is done.
- **The product.** The Blum 71T3550 [product data sheet](https://www.wurthmachinery.com/skin/common_files/files/specs/B71T355_Datasheet.pdf)
  (a distributor's copy) gives: CLIP top standard hinge, 110 degrees, with spring; 35 mm boss drilling, 13 mm deep, 3 to
  7 mm drilling distance; "type of side adjustment: With screw"; "Side adjustment +/- 2 mm"; depth adjustment "convenient
  with spiral screw", "plus 3/minus 2 mm". The model uses the ±2 mm side travel and the 35 mm cup, 5 mm in from the
  edge.
- **What is not published.** We found no published figure for how far the side screw moves a door per turn. *Assumption:*
  1 mm per turn, so the full ±2 mm travel is two turns each way. The real-trial section below calibrates it. The turning
  torque (0.2 N m), the push needed (10 N), the 6 degree alignment limit and the hinges' self-closing angle (the last 25
  degrees) are our estimates, labelled as such wherever they appear. *Inference (ours):* the 10 N push follows from the
  torque. A Phillips driver's wings bear on faces sloped a few degrees off its axis, so a turning force of T/r (about
  110 N at 0.2 N m and an effective radius of about 1.8 mm) pushes it out with roughly a tenth of that.

## Run a trial

From the repository root, with `yam-isaac-lab-arena` and Isaac Lab-Arena installed (`source ~/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/square-cabinet-door-with-cup-hinge-screw/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "The right-hand cabinet door hangs crooked, so the gap between the two doors is uneven. Make the gap even, 2 to 3 mm wide from top to bottom, and leave the door shut." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/square-cabinet-door

`yam_scripted` plays `demo()`. For a model, use its policy in place of `yam_scripted` (for Dream-1, `--policy dream`
with 320 x 176 frames). The score is `task_score`. `SQUARE_TRACE=1` prints a line every 0.5 s with what the rules saw:
the door's angle, both gaps, both screws' turns, the lower recess's give, how far the tip is off the axis, the
screwdriver's angle, whether the screw is engaged, and the grippers' distance from the doors and cabinets.

The scene is fixed (no randomisation; `object_jitter=0`), so the seed does not matter. A demo run takes about 905 control
steps (45 s simulated) and 6 to 8 minutes on one GPU, including start-up. Network access is needed the first time, when
`simready.py` downloads the SimReady packages into `~/.cache/simready`.

## How the simulation works

**The cabinets.** Two frameless wall cabinets of 18 mm melamine-faced particleboard hang on a wall section, 30 mm above
the bench: ours, 450 wide, and its neighbour, 300 wide. Both are 300 deep and 400 tall. Both doors are 19 mm thick and
396 tall, with 2 mm reveals top and bottom. The neighbour's door is shut and fixed: it is the reference line the gap is
read against. Its own hinges are not modelled.

**Our door and its hinges.** The door (446 x 396 x 19 mm, 2.2 kg) hangs on a chain of three joints:
- a sideways slide;
- a tilt about the door's normal, through the lower hinge;
- the swing hinge.

The two side screws drive the first two. Each screw's turns `phi` (from where it was found, clockwise positive) move the
door at that hinge by 1 mm per turn towards the hinge side, so `delta = delta0 + 1 mm x phi` at each hinge. The slide
carries the lower hinge's movement, and the tilt carries the difference between the hinges over their 236 mm spacing.
As found, the lower hinge sits 0.70 mm too far towards the free edge and the upper one 0.10 mm towards the hinge side.
That gives gaps of 1.60 mm and 2.80 mm at the two reading heights (20 mm from the bottom and from the top). The swing
hinge stops at shut and at 110 degrees. Below 25 degrees open, a 0.6 N m/rad spring aiming 10 degrees past shut pulls
the door closed (as the CLIP top's spring does). Above that, the door stays where it is put. It starts open 50 degrees.

**The gap, as a feeler gauge reads it.** Each step the scorer takes the door's simulated pose, finds where its free edge
is at the two heights, and measures the distance to the neighbour's edge. Shut, with the screws as found, that gives
1.60 and 2.80 mm. Turning the lower screw clockwise by `t` turns makes them about `1.6 + 1.25 t` (bottom) and
`2.8 - 0.25 t` (top). The 1.0 window (both 2.0 to 3.0 mm, within 0.5 mm of each other) is therefore met for lower-screw
turns of about 0.47 to 1.13. Turning the upper screw instead cannot meet it, and turning the lower one anticlockwise
makes the bottom gap smaller.

**The screw rule.** Every control step:
1. The tip's position and the screwdriver's axis come from its simulated pose: the tip is at the end of the PH2 mesh.
2. For each side screw, *engaged* means three things together:
   - the recess's spring is pressed in 0.5 mm or more (10 N);
   - the tip is within 0.8 mm of the screw's axis;
   - the screwdriver's axis is within 6 degrees of the screw's.
3. While a screw is engaged (this step and the last), its turns change by the screwdriver's own rotation about the screw's
   axis since the last step, one for one, within ±2 turns.
4. While engaged, a torque of 0.2 N m against the screwdriver's spin about the screw's axis is applied to the screwdriver
   (`0.2 tanh(w / 0.15 rad/s)`). Against the stop it is 3 N m.
5. The slide and tilt targets are set from both screws' turns, and the screw heads turn visibly, with a paint mark.

The turns are the only state the rule keeps, and they are cleared only in mechkit's `on_reset` hook.

## Scoring

All of it is read at the end of the run.

- **Gaps:** between the two doors' facing edges, 20 mm below the top and 20 mm above the bottom (feeler gauges in 0.1 mm
  steps).
- **Shut:** our door within 0.5 degrees of its closed stop (its free edge within about 4 mm of the neighbour's face).
- **Clear:** every fingertip point of both grippers (mechkit's `fingertips`: tips and 3 cm up each finger, middle and
  both edges) at least 3 cm from both doors (knobs included) and from both cabinets. The cabinets' boxes include their
  insides and the hinges.
- **Still:** the door's free edge moving under 9 mm/s.

| Score | Condition |
|---|---|
| 1.0 | Shut, both gaps 2.0 to 3.0 mm and within 0.5 mm of each other, clear and still, all held for 1 s (the episode ends then) |
| 0.5 | At the end: shut with gaps in that window, without the 1.0 conditions held for 1 s (not clear, not still, or not yet for 1 s) |
| 0.25 | Otherwise: at the end, the lower hinge's side screw has turned at least a quarter turn clockwise from where it was found (its paint mark) |
| 0 | Anything else |

Possible scores: 0, 0.25, 0.5, 1.0. A quarter turn the right way is a real intermediate: the bottom gap is opening, but
the door is not yet square, or it is still open. Nothing in the scene can be damaged, so no condition zeroes the score.
Overturning the screw counts only as turned.

## The demo

`demo()` is an open-loop plan from the published starting layout, played by `yam_scripted`.

1. **The left arm takes the screwdriver.** It grips the handle from above, with its fingers along it, and lifts it out of
   its stand. Behind the open door, it tips it level, pointing at the right-hand side panel.
2. **It seats the tip.** It brings the screwdriver into the cabinet at the lower hinge's height, lines the tip up with the
   side screw, and moves it into the recess.
3. **It pushes and turns clockwise through 220 degrees.** It pushes in by aiming about 37 mm past the recess's floor;
   the soft arm then holds about 10 to 14 N.
4. **It eases off and winds the wrist back,** with the tip still in the recess and a light push (the screw does not turn).
5. **It pushes and turns again,** through 88 degrees. The screw has then turned about 0.81 of a turn.
6. **It backs out of the cabinet** and holds the screwdriver upright, clear of the door's swing.
7. **The right arm shuts the door.** It pushes the door's outer face from 50 degrees to 12 degrees and lets go. The hinge
   springs close it.

The plan has corrections measured on the settled scene:
- `TRIM`: how far the arm droops holding the screwdriver level;
- `TIP_OFF`: where the tip hangs relative to the gripper;
- `LEAN`: where to aim the gripper at each roll of the wrist, so that the soft wrist, bent by the push, ends up in line
  with the screw.

It reads nothing from the simulation while it runs.

Results on the final `task.py` (`run_trial` with `yam_scripted`, 640 x 360):

| Run | task_score | Steps | Lower screw | Gaps at the end (top, bottom) |
|---|---|---|---|---|
| Demo 1 | 1.0 | 905 | +0.81 turn | 2.59 mm, 2.62 mm |
| Demo 2 | 1.0 | 905 | +0.81 turn | 2.59 mm, 2.62 mm |

In its one run (2,400 steps), Dream-1 pushed the open door shut with the gap still uneven and never picked up the screwdriver, scoring 0.0.

## Red team

Each variant is `task.py` with another `MODE` (the same scene and scorer), run with `yam_scripted` like the demo:

| Variant | What it does | task_score |
|---|---|---|
| none | Both arms wait | 0 |
| nopush | Seats the tip in the recess and turns the full 220 degrees with only a light push (about 1 to 2 N), then shuts the door: the screw does not turn | 0 |
| tilted | Pushes and turns as the demo does, but with the gripper swung 20 degrees off the screw's axis about the tip (the screwdriver sits 7 to 17 degrees off in the grip): 0.05 turn | 0 |
| ccw | Pushes and turns the right screw the wrong way (anticlockwise), then shuts the door | 0 |
| over | Three full strokes, 1.75 turns: the gaps end at 2.36 and 3.80 mm | 0.25 |
| onearm | Turns the screw as the demo does (0.81 turn), but nothing shuts the door | 0.25 |
| hold | The demo, but the right hand pushes the door shut and stays on it | 0.5 |

A 10-degree swing of the gripper is not enough to break the rule: the screwdriver pivots in the grip and its tip stays in
the recess, so it ends about 4.5 degrees off the axis. That is within the 6-degree limit, and it turned the screw 0.28
of a turn before slipping (0.25). The rule judges the screwdriver's actual axis, not the gripper's.

## Real trials: build, prepare, measure, reset

**Parts.**
- Two frameless wall cabinets, cut from 18 mm melamine-faced particleboard: ours 450 wide, its neighbour 300 wide, both
  300 deep and 400 tall, with 8 mm hardboard backs, screwed to a stud wall or a 12.5 mm plasterboard-faced test wall
  30 mm above the bench, and screwed to each other.
- Two full-overlay doors, 446 x 396 x 19 mm and 297 x 396 x 19 mm, each with a 32 mm knob 35 mm in from its free edge
  and 60 mm up.
- Two Blum CLIP top 110-degree hinges with spring (71T3550) on Blum 0 mm CLIP mounting plates for ours. The neighbour
  hangs on the same kind of hinges, already adjusted. Cups are bored 35 mm, 13 mm deep, 5 mm from the door's edge, with
  centres 80 mm from its ends.
- A PH2 screwdriver with a 100 mm blade and a 34 mm handle.
- A 70 x 70 x 45 mm pine block screwed to the bench as its stand: drilled 6 mm to 28 mm deep, then counterbored 10.5 mm
  to 17 mm deep.
- A mug, a closed utility knife and a marker pen as distractors.
- A feeler-gauge set (0.05 to 1.00 mm), a pencil, a paint pen and a protractor or angle finder.

**Preparing the fault (every trial).**
1. With the door shut, adjust both side screws until both gaps read 2.5 mm.
2. Turn the lower side screw anticlockwise by about 0.7 turn. Check with the gauges that the gaps read 1.6 mm (bottom)
   and 2.8 mm (top), ±0.1 mm. Trim the lower screw if not.
3. Mark the lower screw's head and the hinge arm beside it with one paint-pen line.
4. Open the door to 50 degrees, against a tape mark on the bench under its free edge.
5. Stand the screwdriver in its stand.

**Calibrating the model (once).**
- Turn the lower screw one full turn and measure how much the bottom gap changes. The model assumes about 1.25 mm (1 mm
  at the hinge).
- Check the screw's turning torque with a small torque screwdriver or a luggage scale on a lever. The model assumes about
  0.2 N m.
- Stand the cabinet's side on a kitchen scale and note the push at which a PH2 driver stops camming out when turned. The
  model assumes 10 N.

If the hinge differs, scale `PITCH`, `T_TURN` or `SEAT_MIN` in `task.py`.

**Measuring.** After the run, without touching anything:
1. Check the door is shut: its free edge within 4 mm of the neighbour's face, by ruler.
2. Read both gaps at the pencil marks 20 mm from the top and bottom with the feeler gauges (the thickest blade that
   slides in).
3. Read the screw's paint line against the arm's line for the quarter-turn credit.
4. Check from the video that both grippers were 3 cm or more from the doors and cabinets and the door was still for the
   last second.

**Reset, about 2 minutes.** Turn the lower screw back to its paint line, check the gaps, put the screwdriver back in its
stand and open the door to its tape mark.

**Safety.** The forces are small: a 10 to 20 N push and 0.2 N m. The screwdriver tip is blunt. The only pinch point
is the closing door, which its spring shuts with a fraction of a newton-metre.

## Distinct from the nearest tasks

| Task | What it is | How this differs |
|---|---|---|
| PR #1865 (another team, not accepted, hold lapsed) | Even out a lift-up flap door's gaps with its hinge's side screw, then close it | A side-hung door on a named, purchasable cup hinge (its review asked for an identified product). Ours turns the screw with a hand screwdriver that must be seated and pushed, and the right screw and direction must be chosen |
| TK-W8FDKM (accepted) | Fit a lever set to a bored door; drive two screws while holding the lever level | Driving fasteners home, not adjusting an installed mechanism to a measured result. Its bit drives while its tip is near the axis; ours also needs push and alignment, against a turning torque |
| PR #2407 (closed) | Reattach a concealed cabinet hinge | Clipping a hinge on, not adjusting one |
| TK-Q8FKPK (accepted) | Shim a door's hinge jamb with opposed tapered shims | Packing a door frame, not adjusting a hinge |

We found no comparable task in LIBERO, RoboCasa, BEHAVIOR or RoboARC. RoboARC's screw tasks assemble furniture by
driving screws home, rather than adjusting an installed hinge until a gap is right.

## Fidelity

- Cabinets, doors, knobs, mounting plates, hinge arms and the screw stand are rigid primitives.
- The hinge's working (the slide and tilt that the side screws drive, and the self-closing spring) is modelled, not its
  linkage.
- The depth screws are fixed. The neighbour's door is fixed shut.
- The screw rule is a rule: the turns follow the screwdriver's rotation while the engagement conditions hold. The PH2
  recess is a four-flank funnel on a spring, not a cross. Its turning torque is applied to the screwdriver.
- Per-turn movement, torque, push threshold, alignment limit and self-closing angle are estimates (see Importance and
  sources).
- Door gravity is off: the door turns about a vertical axis, and its weight's sag on the hinges is not modelled.
- Friction is uniform per part.
- The SimReady screwdriver collides with its own mesh (signed distance field). Its mass is set to 0.10 kg.

## Assets and licences

- `mechkit.py`: the mechanism kit from another team's open-source task in this repository (`tasks/heat-shrink-lug/`),
  unchanged.
- `simready.py`: the SimReady loader from another team's open-source task in this repository
  (`tasks/cutters-off-pegboard/`), unchanged.
- NVIDIA SimReady props, © NVIDIA, CC-BY 4.0, from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets)
  at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`. They are downloaded at first run; nothing large is committed.
  - [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01):
    the screwdriver. Its mass is set to 0.10 kg, and it starts standing tip-down.
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
    a distractor, its mass set to 0.30 kg.
  - [`handTool_knife_retractableBlade_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_knife_retractableBlade_a01):
    a distractor, its mass set to 0.15 kg. Its joints are welded into one body by the loader.
  - [`stationery_marker_thin_blue_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_marker_thin_blue_a01):
    a distractor, its mass set to 0.01 kg.
