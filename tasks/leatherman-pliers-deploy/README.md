# Open a Leatherman multitool's pliers

A closed Leatherman Wave+ lies flat on a desk. The robot opens its pliers, ready to use: each of the two handles swings
half a turn about its own pivot, the two in opposite directions, until they meet back to back on the other side of the
plier head and click home. The tool is loose, so a handle pushed round on its own drags the whole tool with it; one
hand has to hold the tool still while the other swings a handle, and then they swap.

Instruction: "Open the pliers out of the multitool, ready to use, and leave it on the desk."

## Importance and sources

- **Everyday operation.** A folding multitool is opened into pliers every time its pliers are used: on the Wave the
  plier head folds inside the two handles, and the handles swing out round the head to bring the jaws out. This is the
  first thing anyone does with such a tool, in a kitchen drawer, a car, a bike bag or a toolbox.
- **How common the tool is.** Leatherman had sold more than 20 million tools by the end of 2001 ([Wikipedia,
  "Leatherman"](https://en.wikipedia.org/wiki/Leatherman)), having gone from 200 tools in 1983 to 1 million in 1993
  ([GearJunkie, "Origin of Leatherman"](https://gearjunkie.com/proj/leatherman/origin-of-leatherman)). These are one
  brand's sales; other makers (Gerber, SOG, Victorinox's plier tools) sell the same fold-out design. Limit: we found no
  survey of how many adults have opened one, and do not claim that most have.
- **The product.** The Wave+ was measured by [OutdoorGearLab](https://www.outdoorgearlab.com/reviews/camping-and-hiking/multi-tool/leatherman-wave-plus):
  4.0 x 1.5 x 0.7 in closed, 8.5 oz, 6.3 in long with the pliers open, $130 list. The model uses exactly those
  numbers (102 x 38 x 18 mm, 241 g, 160 mm open).
- **Why it is a manipulation problem and not a lookup.** Each handle turns about its own pivot, not about the tool's
  middle, so the push has to follow two different circles in opposite senses; a handle pushed towards the other one is
  against its stop and does nothing. A detent holds each handle shut, and the tool weighs 241 g on a smooth desk, so the
  force that starts a handle moving also slides and turns the whole tool unless it is held. Done naively with one hand,
  the tool skates across the desk and the handles stay shut (the one-arm check below scores 0).

## Run and score

The scene, scorer and demo are in `task.py` (one file, read by the simulator's task loader). From the repository root,
with the simulator's environment active:

    # a model run (Dream-1 or any policy), scored by the task's own scorer
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/leatherman-pliers-deploy/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Open the pliers out of the multitool, ready to use, and leave it on the desk." \
      --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/leatherman-pliers-deploy/dream

    # the scripted demonstration (closed loop), from the task folder: video, setup image and result.json with task_score
    cd tasks/leatherman-pliers-deploy
    python demokit.py task.py ~/yam-eval/leatherman-pliers-deploy/demo --max-steps 1200

    # the failure checks: do nothing, one arm, the second handle pushed the wrong way, letting go early, never letting go
    python demokit.py task.py ~/yam-eval/leatherman-pliers-deploy/checks --no-record --mode none,onearm,sameway,early,hold --max-steps 1200

- Run limit: 1,200 steps (60 s at 20 Hz). The demo reaches full score at about step 450 (22.5 s).
- Runtime per trial: about 1 minute on an L40S once shaders are compiled; `--fast` (CPU physics, no cameras) runs a
  trial in about 30 s for debugging.
- Seeds: none. The layout is fixed (`object_jitter=0`) and nothing in the task is random.

## The tool and its mechanism

| Part | Model | Source |
|---|---|---|
| Closed tool | 101.6 x 38.1 x 17.8 mm, 241 g, lying on its large face | OutdoorGearLab: 4.0 x 1.5 x 0.7 in, 8.5 oz |
| Handles (2) | 19 x 17.8 mm section, 92.1 mm from pivot to free end, rounded about the pivot (radius 9.5 mm), 85 g each | half the closed width each; the pivots sit at the centre of each handle's rounded end |
| Plier head | jaws shut, 68 mm from the pivot line to the needle-nose tip, 25 mm wide at the pivots, 9 mm thick, 71 g | open length 6.3 in = 92 mm handle + 68 mm jaws |
| Pivots | one per handle, 19 mm apart, about the axis through the tool's thickness | the Wave's handles swing in the plane of its large faces |
| Detents | shut: holds the handle within its first 12 degrees (spring 0.10 N m/rad preloaded 6 degrees: 0.01 N m to start it, 0.03 N m at 12 degrees); open: pulls it onto its stop through its last 40 degrees | estimates; real Leatherman handles are held by ramps that "click into place" |
| Pivot friction | 0.008 N m | estimate |

In code (`task.py`): a free mechkit articulation. Its frame is the plier head; the two handles are hinges about z
through their pivots, with joint limits at shut (0 degrees) and at the open stop (180 degrees), so a handle pushed the
wrong way meets its stop. The detents are the hinges' springs, re-aimed every step by the task (`Trial._detents`):
towards 6 degrees past shut inside the first 12 degrees, towards 6 degrees past the open stop inside the last 40, and
resting where the handle is in between (free but for friction). The two handles do not collide with each other or the
head (as in the real tool, which is built so they pass).

**Friction.** PhysX averages the two materials in a contact, and in this simulator the desk and the gripper pads share
one material (0.5). To get the real pairs, the tool's underside and head are given 0.2/0.15 (steel on a desk:
0.35/0.33) and the handles' top and side faces 1.4/1.3 (a rubber pad on steel: 0.95/0.9). This is stated in
`fidelity`.

## Scoring

Read from the state at each step; nothing is remembered except a fall off the desk, which is recorded in the trial
tracker and cleared only when the scene is reset (mechkit's `on_reset` hook). The episode ends as soon as full credit
has held for 1 s.

| Outcome | Score |
|---|---|
| Both handles at their open stops (each more than 170 degrees from shut and their free ends no more than 2 mm apart), the tool flat on the desk (underside within 3 mm, tilt 5 degrees or less), every fingertip at least 30 mm from the tool, all still for 1 s (20 steps) | 1.0 |
| Both handles swung at least 90 degrees from shut (the jaws clear of both handles), without the full conditions | 0.5 |
| One handle swung at least 90 degrees from shut | 0.25 |
| Anything else, including the start | 0 |
| The tool fell off the desk at any time in the trial | 0 |

How the simulator judges each rule (`Trial.step`):
- Handle angles are the hinges' joint angles. The free-end gap is computed from them: the corner of each handle's back
  at its free end, in the head's frame; 1 degree short of the stop leaves a 1.6 mm gap.
- Flat: the frame's origin is on the tool's underside; its height above the desk and the frame's tilt.
- Hands clear: mechkit's `fingertips()` (24 points on both grippers' fingers) against boxes round the head and each
  handle where it is now; every point must be 30 mm or more away.
- Still: tool speed under 1 cm/s, turn rate under 0.1 rad/s, and neither handle moving more than 0.5 degrees in a step.
- Fell: the tool's underside more than 5 cm below the desk top.

## Results in simulation

Isaac Lab-Arena, `object_jitter=0`, `control_hz=20`, run limit 1,200 steps.

| Run | What it does | Score |
|---|---|---|
| Demo, GPU, final render (1) | see below | 1.0 (full credit held at step 451, 22.6 s) |
| Demo, GPU, final render (2) | the same, a second fresh run | 1.0 (step 451) |
| Demo, CPU physics (`--fast`), four press depths 11-15 mm | the same | 1.0 at 11, 13 and 15 mm (450-530 steps); 0.5 with a smaller lead (9 degrees) |
| `none` | the arms do nothing | 0.0 |
| `onearm` | the right arm alone pushes each handle round in turn, nothing holds the tool | 0.0 (the tool skates; the handles stay shut) |
| `sameway` | the first handle is opened properly; the second is pushed towards the first, against its stop | 0.25 |
| `early` | lets go of each handle at 120 degrees, before the open detent can take it | 0.5 |
| `hold` | opens both but never lets go of the second | 0.5 |
| Dream-1 | the task's instruction, full run limit | pending |

The failure checks were run with CPU physics (`--fast`); the demo's score is from the GPU runs.

## The demonstration

Closed loop (`demo_script`, run by `demokit.py`): it reads only what a camera policy could see, the tool's pose and its
handles' angles, and never the scorer's state. The tool lies with its pivots towards the robot.

1. The left gripper points straight down with its fingers open 40 mm and in line with the left-hand handle, and presses
   both fingertips on that handle's top near its pivot: two contacts 49 mm apart, so the tool can neither slide nor
   turn. The right gripper, closed and tilted 60 degrees out to the right (so that the two wrists stay apart over a
   38 mm-wide tool), presses on the right-hand handle's top near its free end, on the half nearer the seam.
2. The right gripper drags that handle round its pivot, flat on the desk: out to the right and back towards the robot.
   Its goal runs ahead along the handle's arc but never more than 12 degrees ahead of the handle as seen; if the handle
   stops following for 0.6 s, it lifts, puts the fingertip back on the handle and presses again.
3. Past 145 degrees the right lifts off and the open detent pulls the handle onto its stop; the left lets go.
4. They swap: the right pins the open handle the same way, and the left, tilted out to the left, drags the left-hand
   handle round the other way. Past 145 degrees the right lets go first, the left lifts off, and the second handle
   clicks home back to back with the first. Both arms move clear and wait.

## Real trial: preparation, reset and measurement

- **Rig.** A Leatherman Wave+ (any finish) on a desk or table with a smooth wooden or laminate top. Mark the closed
  tool's outline on the desk with masking tape, pivot end towards the robot, the pivots' midpoint 30 cm in front of
  the arms' base line on the midline. Place the mug, glasses and tape dispenser on their tape marks. Fold the knife,
  saw and every other blade in, and check that each blade's lock is closed, so nothing sharp is exposed; the pliers
  themselves have no edge but the wire cutter, which sits between the jaws.
- **Preparation check.** Before each trial, open and close each handle once by hand to check that both detents click,
  then close both fully (no gap between the handles at the free end) and lay the tool in its outline.
- **Reset.** Fold each handle back over the head until its detent holds it, put the tool back in its outline, open both
  grippers in the ready pose. About a minute. Nothing is consumed.
- **Measurement at the end, before anyone touches the scene.** From the top camera's last frame: both handles on the
  robot's side of the head, past square to the head (90 degrees, judged with a protractor on the frame) for the
  partial credit. For full credit: slide a 2 mm feeler gauge between the handles' free ends (it must not fit), check
  the tool lies flat (no gap under it at its ends with the same gauge plus 1 mm), and check in the video that both
  grippers were at least 3 cm from the tool for the last second.

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| TK-P4D4Z4, fold a lever nail clipper shut (accepted) | one small lever, swung half a turn about a pin and flipped over, with one hand, the clipper boxed in a tray slot | two handles on two separate pivots, swung in opposite senses; the tool is loose, so one hand must hold it while the other swings, then they swap |
| TK-WBHW69, widen tongue-and-groove pliers to the last groove (accepted) | opening, sliding and closing an adjustable plier joint | no adjustment: deploying folded handles round the plier head until they click home |
| TK-Y8KTZZ, stand a tablet by unfolding its back support (accepted) | lifting and unfolding one hinged flap into a stand | two half-turn swings flat on the desk with detents at both ends |
| TK-HKR8WV, fold an open button-lock pocket knife shut (not accepted) | closing one blade safely | opening a plier tool; no blade is moved |

No accepted or pending task on the board mentions a multitool or Leatherman (searched the board's titles for
"multitool", "multi-tool", "Leatherman", "pliers", "unfold", "knife", 21 November 2026).

## Simplifications

- Rigid parts from the published outer dimensions: the handles are solid (their channels, blades and inside tools are
  not modelled), and the plier head is one body with its jaws shut. Only the two handle pivots move.
- Detent springs and pivot friction are estimates, not measurements of a Wave+.
- Per-face friction values reproduce the real contact pairs (see "Friction").
- No force sensing; nothing is latched or moved by code except the detents' spring targets.

## Assets and helpers

The multitool is an authored mechanism (mechkit meshes and colliders); SimReady has no multitool. The distractors are
NVIDIA SimReady props, downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) at pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
  distractor, loose, mass set to 0.30 kg.
- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01):
  distractor, loose, mass set to 0.03 kg (its joints are welded into one body by the loader).
- [`misc_tape_dispenser_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_tape_dispenser_a01):
  distractor, loose, mass set to 0.35 kg.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository,
`tasks/heat-shrink-lug/`. `simready.py` is adapted from `tasks/cutters-off-pegboard/simready.py` (loader unchanged).
