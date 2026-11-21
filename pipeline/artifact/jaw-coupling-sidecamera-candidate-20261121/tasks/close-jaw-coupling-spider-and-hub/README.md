# Close a Lovejoy L100 jaw coupling: seat the new spider, turn the loose hub into mesh and set the gap

A task file for the `yam_arena` embodiment (Isaac Lab-Arena, station scene): [task.py](task.py).

A drive is bolted to the bench: a NEMA 56 brake motor and, in line with it, a 5/8 in driven shaft that turns freely
in two UCP202-10 pillow blocks. They are joined by a Lovejoy L100 jaw coupling. The motor's hub is set-screwed to the
motor shaft, one jaw straight up. The second hub sits on the driven shaft's key. It has been slid back 60 mm from its
working position and turned 30 degrees out of mesh, as a mechanic leaves it after taking out a worn spider. A new
L099/100 spider stands in a wooden cradle on the robot's left. The instruction gives only the goal: close the coupling
with the new spider, with the hub faces 18 mm apart (the catalogue gap G for an L100).

To do that, the robot has to:

1. Put the six-legged spider into the motor hub along the axis, with its legs lined up between the three jaws. The
   clearance is 0.5 mm a side.
2. Bring the loose hub up to the spider. Pushed straight in, its three jaw tips stop on three of the spider's legs.
3. Turn the loose hub on its free shaft while pressing it forward, until its jaws drop into the spider's three free
   gaps. This is the phase, found by contact.
4. Slide the hub to the gap and stop there. The hub faces must end 17.0 to 19.0 mm apart. Driving the hub hard
   against the spider closes the gap to 15.5 mm, and the guide warns against this.
5. Let go.

In the demonstration the left hand handles the spider and the right hand handles the hub. Which hand does what is not
scored.


## Run and score

With the simulator installed, run from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/close-jaw-coupling-spider-and-hub/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Close the jaw coupling between the motor and the driven shaft with the new spider, with the hub faces 18 mm apart." \
  --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval/jaw-coupling-demo
```

That command plays the scripted demonstration (`demo()` in `task.py`). The model run uses the same command with
`--policy dream -E cam_width=320 -E cam_height=176 --max-steps 2400`. The harness prints `task_score` at the end. The
episode ends as soon as `success()` is true.

- **Runtime:** the demonstration reaches success after 1,027 controls (51.35 s simulated). The video includes the initial
  image, so it contains 1,028 frames. The trial takes
  about 6 to 10 minutes of wall time on an L40S, including start-up. A model run is at most 2,400 steps (2 minutes
  simulated).
- **Seeds:** `object_jitter=0` keeps the layout fixed, so the seed does not matter. The demonstration is open loop:
  fixed waypoints computed from the published layout. It is deterministic.
- **Red-team plans:** `TASK_DEMO=<mode>` selects another plan in `demo_plan()`. The modes are `none`, `no_spider`,
  `drop_spider`, `no_turn`, `hard_push`, `hold` and `one_arm` (see [Red team](#red-team)).
- **Debug log:** setting `JAW_LOG=/path/file.txt` makes the task append one line per step: the hub's slide and turn,
  the gap, the spider's pose, the scorer's conditions and the fingertips. It does not change the scoring.
- **Review video:** the task-specific station top camera sits at (0.30, -0.65, 0.98) m and points to (0.30, 0, 0.85) m, looking across the coupling axis. The submitted video uses this 640 x 360 view full frame, so the spider, both hubs and contact sequence stay visible at normal player size. To prepare it from the captured top-camera video, run the command under [Demonstration](#demonstration).

## Scoring

`score()` and `success()` in `task.py` implement this exactly (class `Trial`). The thresholds are the module's
constants. The score is read from the state at each step, and the state at the end of the trial counts. The only
thing carried across steps is the 3 s stillness window, and it is cleared only at a reset (mechkit `on_reset`).

| Score | End state |
|---|---|
| 0.25 | **Spider in the motor hub.** Its far face is within 5 mm of the motor hub's face, so its legs are at least 11.5 mm into the 16.5 mm jaws. Its centre is within 1.5 mm of the axis, its axis is within 3 degrees of the shafts', and its legs are within 5 degrees of their slots between the jaws. Every fingertip is at least 3 cm from it, so the spider is not being held in place; once the loose hub is in mesh the spider is trapped between the hubs and this hands-off check is dropped until 1.0. |
| 0.5 | That, and the loose hub is **in mesh**: its jaw tips are at least 5 mm past the spider's near face. They can only be there inside the spider's free gaps. |
| 0.75 | That, and the **gap** between the hub faces is 17.0 to 19.0 mm. |
| 1.0 | That, and **hands off and still**: every fingertip is at least 3 cm from both hubs and the spider. For 3 s the loose hub moves less than 0.3 mm along the shaft and less than 0.5 degrees about it, and the spider moves slower than 5 mm/s. |
| 0 | Anything else. That includes doing nothing, hubs closed jaw-between-jaw with the spider missing or fallen out, and the spider still held in the hand. |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

The target is the current catalogue's G for an L100, 0.71 in (18.03 mm), and the window is 17.0 to 19.0 mm, about
1 mm either side. Lovejoy's 2012 installation guide gives a slightly larger G, 0.750 in (19.05 mm). That figure
lies 0.05 mm outside the window, so a mechanic who sets the guide's gap exactly would score 0.5; the instruction
names 18 mm, so the catalogue value is the one asked for. The window's lower edge, 17.0 mm, sits 1.5 mm above the
15.5 mm a hub reaches when it is driven onto the spider's legs.

Full credit means the coupling is assembled and set to its gap, ready for the loose hub's set screw. It does not
mean the drive is ready to run: the set screw is not modelled or tightened, so the loose hub is still free to slide
on its shaft (see [Fidelity](#fidelity-and-approximations)).

**In the real world** each check uses ordinary tools at the end state, before anyone touches the coupling:

- The spider's depth in the motor hub: use a steel rule or caliper along the shaft axis. Measure the clearance between
  the motor-hub face and the spider face closest to the motor. The clearance must be at most 5 mm, which gives at
  least 11.5 mm of engagement in the 16.5 mm jaws.
- Mesh: whether the loose hub's jaws are inside the spider, from a photo looking across the coupling.
- The gap: feeler gauges or a caliper between the hub faces at the OD, at three places 120 degrees apart. Use the
  mean.
- Hands clear and still: the trial video.

Both machines are bolted down, so moving them is not possible and is not scored.

## The mechanism (`task.py`)

- **`motor`** is a fixed part: the brake motor, its shaft and its hub. The hub's body is one convex collider (OD
  64.5 mm, 35.05 mm long through the bore). Each of its three jaws is a convex hull. The jaw's flanks are parallel to
  the spider legs either side of it, with a 0.5 mm clearance and a 1 mm chamfer at the tip. Holding the motor's
  shaft fixed is the brake's job: a brake motor's spring-set brake holds the shaft whenever the motor is not powered.
- **`drive`** is one articulation. Its frame is the two pillow blocks and their steel riser. Its link `shaft` is the
  driven shaft, on an unlimited hinge about the axis (joint friction 0.02). Its link `hub` is the loose hub. It is
  a slider along the axis, a child of the shaft, so it turns with the shaft as the key makes it, and it slides along
  the shaft (joint friction 0.3, damping 4). The slider's stop at a 15.0 mm gap is never reached: the spider's legs or
  the motor hub's jaw tips come first. Self-collision is off inside an articulation, so the bore's fit on the shaft
  belongs to the joints, and the hub collides with the motor's hub and the spider by contact.
- **`spider`** is a free rigid body, 27 g. It has six parallel-sided legs, 10.92 mm thick and 15.49 mm wide along
  the axis, with 0.8 mm edge breaks at both faces. They stand on a ring from the 13.1 mm open-centre radius out to
  18 mm. That makes 18 convex colliders.
- Nothing is detected, triggered or applied by code: the spider's seating, the blocked jaws, the drop into mesh and
  the gap all come from contact and the two joints.

## Physical setup

Millimetres from the midpoint between the arm bases (x forward, y to the robot's left). Heights are above the bench
top. The shafts' axis runs along x at y 0, 88.9 above the bench.

| Object | What it is | Where |
|---|---|---|
| Brake motor | NEMA 56 frame (shaft height D 3-1/2 in, 5/8 in shaft), painted cast frame 168 across, end shield face at x 352, red brake housing at the rear to x 687; rigid base; fixed | shaft along x, its end at x 300 |
| Motor hub | Lovejoy L100 x 5/8 in, 3/16 x 3/32 in keyway (UPC 685144 11510), sintered iron, OD 64.5, length through bore 35.05, three jaws 16.5 long, 700 g; set-screwed | face (jaw roots) at x 300, flush with the shaft end, jaws toward -x, one jaw straight up |
| Driven shaft | 5/8 in (15.9) keyed steel shaft, 192.5 long, turns freely, cannot slide | from x 262.5 (37.5 from the motor hub's face) to x 70 |
| Pillow blocks | 2 x UCP202-10 (5/8 in bore, centre height 1-3/16 in = 30.2, base 5 in = 127 long), cast iron | centred at x 150 and x 100 |
| Riser | steel block 110 (x) x 150 (y) x 58.7 high, under both pillow blocks; fixed | x 70 to 180 |
| Loose hub | Lovejoy L100 x 5/8 in, as above, on the driven shaft's key | face at x 222 (78 from the motor hub's face), turned 30 degrees: its jaws face three of the spider-leg positions |
| Spider | Lovejoy L099/100 open-centre SOX (NBR) spider (UPC 685144 11492), black, 64.5 across, 15.5 wide, six legs 10.9 thick, 26.2 centre hole, 27 g | standing in the cradle, faces square to x, two legs level, centre at (215, 175, 34.7) |
| Cradle | wooden: floor 17.1 (x) x 100 (y) x 4 thick, two walls 12 x 100 x 18 high either side; fixed | centred at (215, 175) |
| Distractor: shop rag | SimReady `cleaning_cloth_a01`, cotton, 260 x 233, 50 g | (480, 280), turned 23 degrees |
| Distractor: combination wrench | SimReady `handTool_wrench_combination_a01`, steel, 260 long, 250 g | (200, -200), along x |
| Distractor: oil can | SimReady `consumable_lubricant_oil_multi_purpose_a01`, 52 x 104 x 272, 450 g | (600, -240) |

The scorer ignores the distractors. None of them is reached by the demonstration's paths, and none offers a
shortcut: none can stand in for a spider, push a hub, or set a gap the scorer would accept.

### Building it for a real trial

- Buy two Lovejoy L100 x 5/8 in hubs with keyways (UPC 685144 11510) and one L099/100 open-centre spider (UPC 685144
  11492). Get two UCP202-10 pillow blocks, a 5/8 in keyed shaft cut to 192.5 mm with a 3/16 in key, and a steel
  riser 58.7 mm high. For the driver, use a 56-frame brake motor with a 5/8 in shaft (for example Baldor-Reliance
  VBM3538; listed at $1,485.72). It is never powered.
  - *Lower-cost alternative (our suggestion):* any 5/8 in keyed shaft held so it cannot turn. A third pillow block
    with its set screws on the shaft, plus a clamp on the shaft, gives the same fixed motor-side hub. The motor only
    supplies the installed context.
- Bolt both to the bench with the shafts in line to within 0.015 in (0.38 mm), which is the guide's parallel limit
  for an L100 with an NBR spider. The driven shaft's end goes 37.5 mm from the motor hub's face.
- Motor hub: flush with the motor shaft's end, one jaw straight up, set screw tightened (5/16-18, 150-160 in-lb in
  the guide's Table 2). Loose hub: set screw backed off. It should slide by hand.
- Reset in about 2 minutes:
  1. Pull the loose hub back until its face is 78 mm from the motor hub's face, at a pencil mark on the shaft.
  2. Turn the shaft until a paint mark on the hub is 30 degrees from the mesh mark, which is also painted.
  3. Lift the spider out and stand it in its cradle with two legs level.
- Nothing is consumed. The arms touch only iron and rubber, with no sharp edges, heat or liquids. Renew the spider
  if a leg is nicked.

## Demonstration

`demo()` is a fixed plan of waypoints for both arms, computed from the layout above. It reads no simulator state.
Two corrections were measured in earlier runs of this same plan and are named constants: `ROLL_FIX`, the hub's own
3-degree turn toward mesh when it is pressed onto the legs, and `GAP_FIX`, how far short of the commanded position
the hub ends up once let go. Every step is slow and pauses where something happens, so each one can be seen in the
video:

1. **Left hand, the spider** (about 22 s). The pads open to 82 mm, come down either side of the spider and pinch it
   across its two level legs' ends, 64.5 mm apart. The fingertips are 4.5 mm below its centre, which keeps them above
   the motor hub's lower jaws. The hand lifts the spider out of the cradle, carries it over the gap between the hubs
   and lowers it onto the shafts' line, with its far face 4 mm short of the jaw tips. It pauses there, then takes 3.5
   s to slide the spider 16 mm along the axis into the motor hub's jaws. It lets go there, with the far face 4 mm off
   the hub's face, and moves away. The fingers stay outside the hubs' 64.5 mm OD all the way in.
2. **Right hand, the loose hub** (about 32 s). The pads open to 84 mm and pinch the hub across its OD. The hand slides
   the hub 41 mm along its shaft in 3 s and pauses 3 mm short of the spider. It brings the jaw tips onto the spider's
   legs, which pushes the spider fully into the motor hub, and keeps pressing 2.5 mm past contact. Then it holds:
   **blocked, out of phase**.
3. It **rolls the hub slowly**, about 3.4 degrees a second for 8 s, about the axis while pressing. The hub turns with
   its shaft. At about 27 degrees the jaws reach the spider's free gaps and drop in. The hand holds for 2 s while they
   go in.
4. The hand **slides the hub to the gap** in 3 s, holds, opens, backs off along its fingers and moves away. Success
   is declared once the hub has stayed still for 3 s with both hands clear, so the video ends with the right hand
   back at its parking position and the closed coupling in plain view.

Both final-code demo runs scored 1.0. The submitted screenshot and video use the second run, which reached full score after 1,027 controls (51.35 s); its video has 1,028 frames including the initial frame. The reviewer video is the task-specific top-camera view at 640 x 360:

```bash
ffmpeg -i top_cam.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart demo.mp4
```

## Red team

Each red-team variant is the same scene and scorer with a different plan (`TASK_DEMO=<mode>` selects it in
`demo_plan()`). Each was run once in the simulator on the final code, and these are the scores the code returned:

| Mode | What the plan does | Score | Why |
|---|---|---|---|
| `full` | The demonstration | 1.0 in both runs of the final code | Both runs meet all conditions after 1,027 controls. |
| `none` | Neither arm moves | 0.0 | The spider stays in its cradle. |
| `no_spider` | The spider is left in its cradle; the hub is pushed, turned and set to the gap as in the demo | 0.0 | The hubs close jaw between jaw with nothing between them. |
| `drop_spider` | The careless way: the spider is let go in the gap between the hubs instead of being put into the jaws | 0.0 | The spider falls out of the coupling. |
| `no_turn` | The hub is pushed straight in and never turned | 0.25 | Its jaw tips stay on the spider's legs, about 32 mm from the motor hub's face, as in the demo before it turns. The spider is in the motor hub. |
| `hard_push` | After mesh, the hub is driven hard against the spider | 0.5 | Gap 15.5 mm, below the 17.0 mm limit. |
| `hold` | The hand never lets go of the hub | 0.75 | Everything is in place, but the hand is still on the hub. |
| `one_arm` | The left hand seats the spider, then works the hub; the right arm never moves | 1.0 (success at step 1032) | The same work done by one hand in turn. A valid route, not a shortcut. |

The "one arm" route is not a shortcut here. One hand can do every step in turn, because nothing has to be held while
something else is done. That is a valid route and earns full credit, as the `one_arm` row shows: it is the same
work, done by one hand.

Moving the cradle or other objects onto the coupling is not a route to credit: the scorer reads only the spider's
pose, the hub's joints and the fingertips. A spider wedged between the hubs at an angle fails the 3-degree squareness
and 1.5 mm centring checks.

## Fidelity and approximations

- **Rigid elastomer.** The NBR spider is a rigid body. A real spider is a close fit and gives a little. The 0.5 mm
  clearance a side between leg and jaw is our estimate. Lovejoy does not publish it. Measure it on the bought parts.
- **Estimated dimensions.** These come from the catalogue: OD, length through bore, G, the spider's A, H and W, and
  the B dimension, which we read as the leg thickness. These are our estimates: the jaws' length (16.5 mm, shorter
  than G so the tips never reach the other hub's face), the radius where the jaws start (20 mm), the spider ring's
  outer radius (18 mm), the jaw-tip chamfer (1 mm) and the leg edge break (0.8 mm). Hubs are drawn as plain cylinders
  with a dark keyway and tapped hole. The set screws are not modelled.
- **Bore, key and shaft are joints.** The hub's slide and turn on the shaft are joints, not contact. The slide's
  joint friction (0.3) and the shaft's (0.02) are set so the hub slides by hand and the shaft turns freely. They are
  not measured forces. The motor's brake is modelled by fixing the motor-side shaft.
- **Stops here.** The real job continues: the gap is checked at three places, alignment is checked, and the loose
  hub's set screw is torqued. Those steps are not part of this task.
- **Primitives.** The motor (revolved shells and boxes), pillow blocks, riser and cradle are built from primitives.
  SimReady has no motor, pillow block or jaw coupling. The distractors are SimReady props.
- **Uniform friction.** Friction is uniform: 0.9/0.8 for the spider, and 0.8/0.7 for the hubs and frame.

## Importance and sources

- **Occupation and size.** Industrial machinery mechanics, machinery maintenance workers and millwrights held about
  547,300 US jobs in 2025: 446,900 industrial machinery mechanics, 59,700 machinery maintenance workers and 40,700
  millwrights. Source: BLS Occupational Outlook Handbook,
  [Industrial Machinery Mechanics, Machinery Maintenance Workers, and Millwrights](https://www.bls.gov/ooh/installation-maintenance-and-repair/industrial-machinery-mechanics-and-maintenance-workers-and-millwrights.htm).
  The same page says that millwrights "perform repairs that include replacing worn or defective parts of machines".
- **That this is their regular work.**
  - O*NET's task list for industrial machinery mechanics includes "Repair or replace broken or malfunctioning
    components of machinery or equipment" and "Reassemble equipment after completion of inspections, testing, or
    repairs" ([O*NET 49-9041.00](https://www.onetonline.org/link/details/49-9041.00)).
  - For millwrights it includes "Replace defective parts of machine, or adjust clearances and alignment of moving
    parts" ([O*NET 49-9044.00](https://www.onetonline.org/link/details/49-9044.00)).
  - *Inference:* neither list names couplings. That renewing a coupling's spider is part of this regular work is our
    reading: the spider is the coupling's wearing part, and Lovejoy's own advice covers its wear. Lovejoy's
    [engineering tips](https://lovejoy-inc.com/resources/technical-articles/engineering-tips-tricks/) say: "When the
    spider begins to wear, just pull it away from the jaws and advance it one 'leg'", "Because just three of the legs
    transmit torque."
  - We have no source giving the share of these workers who do this operation, or how often.
- **The real operation.** Lovejoy's
  [L-Line jaw coupling installation guide (2012)](https://www.lovejoy-inc.com/wp-content/uploads/2017/11/JawCouplingLLineInstallGuide2012.pdf):
  - Step 6.5: "Slide the appropriate hub on each of the shafts over the keys". 
  - Step 6.6: "If using a spider, insert the spider into one of the hubs at this time".
  - Step 6.7: bring the hubs to "the 'G' gap dimension between hub faces as specified in Table-4", noting that "the
    hubs could be moved back on the shafts or overhung slightly to compensate for discrepancies in shaft
    separation. Ideally, the amount of hub engagement on the shaft should be equal to the diameter of the shaft", and
    that "If the hubs are pressed too together tightly against the spider, the coupling could lose some of its
    capability to accommodate misalignment."
  - Table 4 gives G = 0.750 in for the L100 and a 0.015 in parallel misalignment limit. Table 2 gives the 5/16-18
    set screw torque. Table 6 gives the L099/100 spider: 2.54 in across, 0.61 in wide, 1.03 in centre hole.
  - Our loose hub sits on 15.5 mm of its 15.9 mm shaft at the gap, which follows the guide's overhang advice.
- **The product.** Lovejoy
  [jaw coupling catalogue (2025)](https://www.lovejoy-inc.com/wp-content/uploads/2025/12/jaw-section-2025-jw21kept.pdf):
  - L-type dimensional data, page 25: L100 OAL 3.48 in, G 0.71 in, LTB 1.38 in, OD 2.54 in, 5/16-18 set screw, 2.40
    to 3.20 lb a coupling.
  - Spider dimensional data, page 24: L099/L100 A 2.54, H 1.03, W 0.61 and B 0.43 in. *Inference:* we read B as the
    leg thickness.
  - L-type hub UPC table, page 12: L100, 5/8 in bore, 3/16 x 3/32 in keyway = 11510.
  - Spider UPC table, page 11: L099/100 SOX (NBR) open centre = 11492.
  - The same L100 figures (OD 2.54 in, LTB 1.38 in, G 0.71 in, 1.3 lb a hub) are on the maker's
    [CAD page](https://cad.timken.com/item/standard-jaw-couplings-l-type/jaw-type-couplings-spling-hubs/68514437905?plpver=10).
  - *Inference:* the 700 g hub mass is ours, from the catalogue's coupling weights.
- **Pillow blocks.** AMI UCP202-10: 5/8 in bore, center line to base 1-3/16 in, base length 5 in. Source:
  [Applied Industrial listing](https://www.applied.com/c-brands/c-ami-bearings-inc-/ucp202-10/5-8-WIDE-SET-SCREW-PILLOW-BLOCK-SINGLE-ROW-BALL-BEARING/p/101699951).
- **Motor.** NEMA 56 shaft height D 3-1/2 in and shaft U 5/8 in: the
  [ABB NEMA quick-reference poster](https://library.e.abb.com/public/cf3a80c5c7fd40aab5af7a89fbabbc33/NEMA%20Wall%20Poster_24x36.pdf).
  Baldor-Reliance VBM3538 brake motor: 56C frame, 5/8 in shaft, 3 ft-lb, $1,485.72
  ([distributor listing](https://www.standardelectricsupply.com/Baldor-Electric-VBM3538-Brake-Motor-Ac)).
  - *Inference:* that the brake holds the unpowered shaft is how a spring-set motor brake works; we give no source
    for it here. The motor's frame and brake housing lengths are approximate. Only the shaft and its height matter
    to the task.

## Distinct from

| Nearest task | What it does | The physical difference here |
|---|---|---|
| Accepted: mesh a three-gear train, finding each gear's tooth phase by contact (TK-YNPRML) | A loose gear on its own shaft meshes with a gear on a parallel shaft. The teeth engage side by side, at the pitch circle, and the gear can turn freely on its shaft to find the phase. | See below |
| Accepted: tighten a 6 mm bit in a keyed drill chuck, keeping the key's pinion meshed | A bevel pinion engages a ring gear while it is turned under rising torque. | The difficulty there is keeping a mesh under load. Here it is making a coaxial three-body engagement and stopping at a dimension. |
| Not accepted: seat a keyed coupling and prove drive transfer; key a pulley onto a shaft | A single part goes onto a shaft over a key. | Here there is no key to fit. Two hubs and a loose elastomer insert must interleave. |

**How the jaw-mesh step differs from gear phase tasks:**

- **The engagement is axial and coaxial.** The hub's jaws enter the spider's gaps along the shared axis. The tips
  bear on the end faces of the legs, not on tooth flanks side by side. Pressing is along the same axis the phase is
  turned about, so the hand has to push and turn about one axis at once. In a gear train the push (down the shaft)
  and the mesh (sideways, at the pitch circle) are on different axes.
- **The part that finds the phase cannot turn on its own.** The loose hub is keyed to a shaft that turns in its
  bearings. It is turned by its OD, under pressing load, through about 27 degrees with a capture window of a few
  degrees. A loose gear just spins on its pin.
- **The thing it meshes with is itself a loose, phased insert.** The spider has to be put into the motor hub first,
  with six legs between three jaws at 0.5 mm clearance, and it is pushed home by the loose hub. Without the spider the
  hubs still close, jaw between jaw. That is the classic "spider left out" error, and the scorer gives it 0.
- **It ends at a dimension, not a stop.** After mesh the hub must stop with the faces 17 to 19 mm apart. Pushing on
  to the stop gives 15.5 mm and only 0.5.

## Demonstration visibility


- **Clearer view.** The task-specific top camera looks across the shaft axis. At 640 x 360 the coupling stays large enough to see the spider seating, the jaws stopping on the legs, the hub roll into mesh and the final gap without shrinking three views into a narrow strip.
- **Slower, visible steps.** Each step is its own slow move with a pause where something happens:
  - lining up in front of the jaws (1 s pause), then 3.5 s into them;
  - pressing on the legs while blocked (1 s);
  - the roll at about 3.4 degrees a second for 8 s;
  - 2 s while the jaws drop in;
  - 3 s to the gap and 1 s hold;
  - the let-go.

  The demonstration reaches success after 51.35 simulated seconds, with the hands back at their parking positions.
- **What happens is measurable in the video and the log.** `JAW_LOG` records each step's hub slide, turn, gap and
  spider pose, so every phase can be checked against the frames.

We also applied today's review bar, beyond the original review:

- real products at real scale, with sourced dimensions (Lovejoy catalogue and guide, UCP202-10, NEMA 56);
- a goal-only instruction;
- numeric end-state scoring with partial credit;
- no violation record that a later state could clear;
- fingertip checks for every hands-off condition;
- SimReady distractors;
- the importance case and its limits stated, with sources.

## Assets and credits

- `mechkit.py` and `demokit.py`: the shared mechanism and demonstration kits, copied unchanged from
  `tasks/heat-shrink-lug/` in this repository (open-source task code of another team). See their docstrings.
- `simready.py`: the SimReady loader, from `tasks/cutters-off-pegboard/` in this repository. See its docstring.
- SimReady props, © NVIDIA, CC-BY 4.0, from
  [nvidia/simready-assets](https://huggingface.co/datasets/nvidia/simready-assets) at the revision pinned in
  `simready.py`. They are downloaded at first run, and nothing large is committed. At load time each prop's mass is
  set (rag 50 g, wrench 250 g, oil can 450 g), its joints are welded into one rigid body, and its grasp guide curves
  are deactivated:
  - `cleaning_cloth_a01` (`sm_cleaning_cloth_a01_01.usd`);
  - `handTool_wrench_combination_a01` (`sm_handTool_wrench_combination_a01_01.usd`);
  - `consumable_lubricant_oil_multi_purpose_a01` (`sm_consumable_lubricant_oil_multi_purpose_a01_01.usd`).
- Everything else (the motor, hubs, spider, shafts, pillow blocks, riser and cradle) is built in `task.py` from the
  dimensions above.
