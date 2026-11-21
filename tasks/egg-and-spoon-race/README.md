# Egg-and-spoon race: round the cup, tip the egg into the bowl

**Instruction:** "Run the egg-and-spoon race: holding the spoon only by the end of its handle, carry the egg round the upturned
cup and tip it into the bowl, without touching the egg."

A wooden practice egg lies loose in the bowl of a 22.5 cm wooden race spoon on the table. The robot must pick the spoon up by
the last 7 cm of its handle, carry the egg out and round an upturned paper cup (the turning marker), bring it back to the finish
bowl and tip it in, without any finger touching the egg. It is the school sports-day race, with the course scaled to the
tabletop and the real spoon and egg.

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score`, `success`) and the closed-loop scripted demo (`demo_script`, with failure-check modes) |
| `task.json` | The task's details |
| `demokit.py` | The closed-loop demo runner, unchanged from `tasks/heat-shrink-lug/` (another team's open-source task) |
| `mechkit.py` | The standard mechanism kit, unchanged from `tasks/heat-shrink-lug/`; used here for its step and reset hooks and `fingertips` |
| `simready.py` | The SimReady prop loader, unchanged from `tasks/cutters-off-pegboard/` (another team's open-source task) |

## Importance and sources

- **Relatability.** "At many primary schools an egg-and-spoon race is staged as part of the annual Sports Day"; competitors
  "must balance an egg or similarly shaped item upon a spoon", and the usual cheats are "sticking the egg to the spoon, or
  holding onto the egg with one finger" ([Wikipedia: Egg-and-spoon race](https://en.wikipedia.org/wiki/Egg-and-spoon_race)).
  Most adults have run one, and a 10 s clip of a spoon carrying an egg round a marker is recognisable at once.
- **Wooden eggs and spoons are what the race is run with.** Dedicated wooden racing sets go back at least to the 1920s (the
  Victoria and Albert Museum holds a turned wooden "Egg and spoon race set", 1920-1929, with egg-cups, eggs and spoons:
  [V&A O10749](https://collections.vam.ac.uk/item/O10749/egg-and-spoon/)), and they are sold today as children's games, for
  example Janod's set of 4 wooden eggs and 4 wooden spoons, painted with water-based paint, $17.99
  ([Rainbow Resource 031922](https://www.rainbowresource.com/031922.html)). Wikipedia notes that some schools ban raw eggs.
- **What the robot must do (numbers measured in this simulation, `--mode cal_pitch,cal_roll` and the failure checks below).**
  The egg (45 g, 5.8 x 4.6 cm) rides on a 3.9 cm wide, 5 mm deep bowl, its centre 14.4 cm out from where the spoon may be held.
  Its lever, about 0.08 N m with the spoon's own weight, makes the spoon sag 3 to 6 degrees in the pads, so the spoon has to be
  levelled from what is seen of it. The egg starts to roll in the bowl at 14 to 17 degrees of tilt along the spoon and leaves
  it sideways at 24 to 27 degrees (two calibration runs), and an abrupt start or stop at 0.6 m/s throws it out (0.3 m/s does
  not). At the finish the spoon must be tipped well past that, over the bowl: tilted along the spoon, the egg first rolls
  2 cm forward and lodges at the spoon's tip, and drops off only past about 30 degrees.

## Run and score

From this folder, after `source ~/IsaacLab-Arena/yam.env`:

```bash
# model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Run the egg-and-spoon race: holding the spoon only by the end of its handle, carry the egg round the upturned cup and tip it into the bowl, without touching the egg." \
  --scorer task_score --max-steps 1800

# scripted demonstration (closed loop, right arm), with video: OUT/full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py OUT

# the failure checks and the calibration (CPU, no cameras)
python demokit.py task.py OUT_checks --fast --mode none,careless,reckless,jerky0.3,jerky0.6,direct,slide,midgrip,touch,highgrip
python demokit.py task.py OUT_cal --fast --mode cal_pitch,cal_roll
```

Nothing is randomised (`object_jitter=0`), so every seed gives the same scene. The scripted demo reaches full credit after
about 38 s of simulated time (about 770 steps, including the 1 s hold); the run limit is 1,800 steps (90 s). On a shared GPU
with cameras a trial ran at about 1 step a second of wall time, so allow up to 30 minutes for a full 1,800-step model trial;
the CPU checks run at 30 to 50 steps a second.

## The scene

World frame: x forward from the arm bases, y to the left, z up; positions in cm, heights above the table.

| Object | Model | Size, material, mass | Place |
|---|---|---|---|
| Spoon | SimReady `kitchenware_spoon_c01` | wooden race spoon, 22.5 cm long, bowl 3.9 x 6 cm and 5 mm deep, round handle 1.1 cm thick; 25 g | bowl-up along y, handle's end at (20.0, -29.4), tip at (20.0, -6.9) |
| Egg | SimReady `food_egg_chicken_a01`, scaled 0.72 | wooden practice egg painted white, 5.8 x 4.6 cm, 45 g | lengthwise in the spoon's bowl, centre at (20.0, -11.5), 2.9 cm up |
| Turning marker | SimReady `dishware_cup_paper_a01` | white paper cup, 8 cm rim, 11 cm tall, upside down, taped down | centred at (35.0, -10.0) |
| Finish bowl | SimReady `kitchenware_bowl_b01` | beige ceramic bowl, 17.3 cm across, 6 cm tall; 350 g, loose | centred at (24.0, 5.0) |
| Distractor | SimReady `dishware_plastic_cup_a01` | blue plastic tumbler, 6.4 cm across, 10 cm tall, 30 g, loose | (42, 22) |
| Distractor | SimReady `misc_tape_refill_a01` | roll of tape, 5.5 cm across, 1.9 cm thick, 60 g, loose, lying flat | (31, 25) |
| Distractor | SimReady `stationery_pencil_watercolor_red_a01` | red pencil, 19.9 cm long, 0.8 cm thick, 6 g, loose | (17, 21), turned 25 degrees from x |

The distractors are at least 8 cm from every task object and outside the demo's path; the scorer ignores them, and none of
them can serve as a support or a ramp for the egg.

Model details (all in `task.py`):

- The spoon collides as the signed distance field of its SimReady mesh (watertight), so its bowl is a real shallow dish and its
  handle a real round rod. The egg collides as the convex hull of its mesh, a smooth ellipsoid, and rolls freely.
- The egg's SimReady rigid body sits at its centre, turned 90 degrees in its file; the reset places that body, so `task.py`
  gives the egg's centre and orientation directly.
- The spoon's friction is raised from SimReady's 0.5 to 1.0 (PhysX averages the two surfaces' values, so 0.75 against the
  gripper's pads). With the authored 0.5 the pads held the 1 cm handle too weakly and the spoon swung down 27 degrees in them.

## Scoring

The scorer runs after every control step (`Race.step`, a mechkit step hook) and reads the poses of the spoon, the egg and the
bowl, and mechkit's fingertip points (tip and 3 cm up each finger, middle and both edges).

- **On the spoon:** the egg's centre, in the spoon's frame, within 3 cm of where it settled at the start. The first step it is
  not, the egg has left the spoon, for good; where it left (inside the finish zone or not) and whether the cup had been rounded
  by then are recorded.
- **Rounding the cup:** seen from above, the egg's centre sweeps 270 degrees round the cup's axis. A step counts only while the
  egg is on the spoon, its centre at least 8 cm above the table (the spoon off the table, not dragged), and 6 to 20 cm from the
  cup's axis (round the cup, not over its top: the rim is 4 cm from the axis). In practice: past the cup on one side, behind it
  and back along the other.
- **Finish zone:** the egg's centre within 10 cm of the bowl's centre, horizontally.
- **In the bowl:** the egg off the spoon, its centre within 6.5 cm of the bowl's centre and below the bowl's rim, still (under
  1 cm/s and 0.5 rad/s), every fingertip point at least 3 cm from the egg's surface and the spoon (six points on its bowl, tip
  and neck) at least 1.5 cm from it.
- **Fouls**, recorded across steps and cleared only by mechkit's reset hook: a fingertip point within 1 cm of the egg's surface
  (touching the egg); a fingertip point within 1 cm of the spoon anywhere but the last 7 cm of its handle (holding it other
  than by the end of the handle); the bowl moved more than 3 cm.

| Score | Condition |
|---|---|
| 1.0 | the egg rounded the cup, then left the spoon inside the finish zone, and has been in the bowl for 1 s (20 steps); 1.0 from then on unless a foul follows |
| 0.5 | it rounded the cup and is on the spoon inside the finish zone now, or left the spoon inside the finish zone but is not in the bowl |
| 0.25 | it rounded the cup but left the spoon outside the finish zone (dropped on the way back), or is still on the spoon outside the zone |
| 0 | any foul; the egg left the spoon before the cup was rounded; or the cup was never rounded |

Possible scores: 0, 0.25, 0.5, 1.0. Doing nothing scores 0.

## The demo

`demo_script` (right arm only, closed loop; it reads only object poses, as a policy could from the cameras):

1. Comes down over the handle 4 cm from its end, fingers closing across the handle, to 9 mm above the table so the pads close
   round the handle's middle, and grips.
2. Lifts slowly (5.5 s to 15 cm), then levels the spoon: reads its pitch and roll and turns the grip against them, twice.
3. Carries the egg at 17 cm (the handle clears the 11 cm cup) along six eased moves: out along the cup's right, behind it 8 cm
   from its axis, back along its left, and to the bowl (about 15 s); the egg's sweep is about 300 degrees.
4. Over the bowl, levels again, then tips the spoon's bowl down 38 degrees in 3 s; the egg rolls off the tip into the bowl.
5. Tips back, lifts the spoon away and holds it clear.

Results with this version of `task.py`: on the GPU with cameras (the video runs), 1.0 at step 769 each time; on the CPU
(`--fast`), 1.0 at step 770. The video run's score is in `OUT/full/result.json`.

Model run: Dream-1, run once on this version of `task.py` for the full 1,800 steps, scored 0; its right gripper hovered
over the spoon's bowl end for most of the run, and the spoon stayed on the table.

### Failure checks (`--mode`)

| Mode | What it does | Score |
|---|---|---|
| `none` | nothing | 0 |
| `direct` | carries the egg straight to the bowl and tips it in, without rounding the cup | 0 (left the spoon before rounding) |
| `slide` | drags the spoon on the table round the cup, then lifts it at the finish and tips | 0 (the dragged spoon spills the egg; dragging never counts as rounding) |
| `midgrip` | holds the spoon at its neck, next to the bowl | 0 (grip foul; the fingers are also within 1 cm of the egg) |
| `touch` | the other gripper rests on the egg while the spoon is lifted | 0 (touch foul) |
| `jerky0.6` | each carry move starts and stops abruptly at 0.6 m/s | 0 (egg thrown out on the way out, after a 97-degree sweep) |
| `jerky0.3` | the same at 0.3 m/s | 1.0 |
| `careless`, `reckless` | the demo's eased moves 4 and 8 times faster | 1.0 |
| `highgrip` | the pads 6 mm higher on the handle | 1.0 |

The last three are recorded honestly: speed with eased starts and stops does not dislodge the egg here, so the task rewards a
smooth, level carry and a controlled tip, not slowness.

## Real trial: preparation, reset and measurement

- **Props:** a wooden egg-and-spoon race spoon and egg matching the table above (any wooden spoon 22 to 23 cm long with a
  4 cm wide shallow bowl and a 1 cm round handle, and a wooden egg about 58 x 46 mm and 45 g), a paper cup, a ceramic cereal
  bowl, a roll of tape, a pencil and a plastic tumbler. Cost under $30 besides the robot.
- **Preparation:** mark the table with tape: the spoon's outline (handle along y, end at (20.0, -29.4)), the cup's place
  with a 12 cm circle drawn round it, and the bowl's outline. Tape the cup down. Wrap a band of coloured tape round the
  spoon's handle 7 cm from its end: the gripper may touch the spoon only between the band and the end. Lay the egg
  lengthwise in the spoon's bowl.
- **Measurement:** an overhead camera records the trial. From the video: whether the egg left the spoon, where (inside the
  10 cm finish circle round the bowl or not), whether it went round the cup outside the 12 cm circle, past one side, behind
  and back along the other, with the spoon off the table, and whether a gripper touched the egg or the spoon past the band.
  At the end, before anyone touches the scene, a photo shows whether the egg lies in the bowl clear of the spoon; a ruler
  checks the bowl is within 3 cm of its outline.
- **Reset (about 1 minute):** spoon back on its outline with the egg in it, bowl on its outline, cup still taped. Nothing is
  consumed. The egg is wooden and the heaviest thing moved is the 70 g spoon and egg, so nothing in the trial can hurt the
  rig or a person.

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| TK-DF9CF5, "Carry a full cup of water past an open laptop" (accepted) | carrying a held, spillable cup steadily past an obstacle | here the load is a free rolling egg on a utensil held at the far end of a 22.5 cm lever, which sags in the grip and must be levelled; the course goes round a marker (a 270-degree sweep), and the egg must then be tipped out into a target without contact |
| TK-WJENGF, "carry-ball-tray" (not accepted: a variant of a loaded-tray task) | carrying a tray of loose balls without spilling | a two-hand tray carry, versus a one-hand cantilevered utensil carry and a controlled tip-out |
| TK-UWTXEC, "paddle-bounce-ping-pong" (not accepted: embodied-olympics-fun's Paddle Bounce) | rhythmic bouncing of a ball on a paddle | no bouncing or timing; the egg must never leave the spoon until the finish |

Among external sets, ManiGuard's "dusty transfer" ends by tipping a tray so a potato rolls into a box without being touched;
it has no carry round a course and no utensil held at its end. No egg-and-spoon task was found on the board, in LIBERO,
RoboCasa, SimplerEnv, BEHAVIOR or the published task lists of Embodied First Exam (whose Embodied Olympics - Fun suite does not
list its 79 task families in full, so it could not be checked completely).

## Simulation notes and honest limits

- The egg is rigid and cannot break; real dropped raw eggs would, which is one reason wooden eggs are used.
- PhysX has no rolling resistance, so the egg rolls a little more freely than a real wooden egg.
- The spoon's friction (1.0, averaged to 0.75 with the pads) also makes the egg-to-spoon friction 0.73, higher than real wood
  on wood (about 0.5). The egg rolls rather than slides in every check, so its tilt limits come from the bowl's shape.
- The cup is fixed, as if taped down.
- Touching the egg and gripping the spoon past the 7 cm zone are judged by fingertip proximity (1 cm), not by contact force.
- The course is tabletop-sized (the egg travels about 60 cm); the spoon and egg are full size.

## Assets and helpers

SimReady props from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) at pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. Files download at first run; nothing large is committed.

- [`kitchenware_spoon_c01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_spoon_c01): the spoon; mass set to 25 g, friction raised to 1.0 static / 0.9 dynamic.
- [`food_egg_chicken_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_egg_chicken_a01): the egg; scaled 0.72, mass set to 45 g, colliding as its convex hull.
- [`dishware_cup_paper_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_paper_a01): the turning marker; upside down, fixed.
- [`kitchenware_bowl_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_bowl_b01): the finish bowl; mass set to 350 g.
- [`dishware_plastic_cup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_cup_a01), [`misc_tape_refill_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_tape_refill_a01), [`stationery_pencil_watercolor_red_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_pencil_watercolor_red_a01): distractors; masses set to 30, 60 and 6 g.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository,
`tasks/heat-shrink-lug/`; `simready.py` from `tasks/cutters-off-pegboard/`. Credit to their authors.
