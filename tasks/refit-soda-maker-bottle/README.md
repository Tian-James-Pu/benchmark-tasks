# Click a full 1 L bottle into a sparkling water maker's tilted Snap-Lock

A sparkling water maker of the SodaStream Terra kind stands on the counter. Its carbonating bottle, filled with 1 L of water,
stands beside it. The robot has to fit the bottle back so that it locks. On these machines the bottle hangs from a
**Snap-Lock** holder under the head: a cup on a hinge that rests tilted forward. The manufacturer's steps are to put the
bottle's neck into the Snap-Lock at that angle, push the bottle up and back, and let the Snap-Lock hold it vertical,
hanging just above the base.

What makes it hard for a robot:

- **Angled insertion of a heavy part.** The neck has to go into a tilted mouth along the holder's axis, not straight up:
  pushed straight up, the bottle meets the cup's front wall and tips the machine.
- **Past a spring, then a swing.** Pushed up the holder's axis, the neck's collar cams two sprung jaws apart and drops
  onto them. Only then can the bottle's bottom be swung back about the holder's hinge to vertical, where the Snap-Lock
  clicks. Swung early, the neck is not in the jaws and nothing holds it.
- **1 kg in a soft hand.** The full bottle sags and turns in the gripper, and the machine is free-standing: shoving it
  slides or tips it.

The instruction states only the goal: "Put the bottle back on the sparkling water maker so that it locks in place."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the Snap-Lock's working, the scorer (`score`, `success`) and the scripted `demo_script` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged, from `tasks/heat-shrink-lug` (another team's open-source kit) |
| `demokit.py` | The closed-loop demo runner, unchanged, from the same source |
| `simready.py` | Loads the SimReady distractor props, adapted from `tasks/cutters-off-pegboard` |
| `task.json` | The task's details for the board |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.
- `scipy` (the holder's collision hulls are built with `scipy.spatial.ConvexHull`). If Isaac Sim crashes at start-up in
  `blas_thread_shutdown`, set `OPENBLAS_NUM_THREADS=1`.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the bottle back on the sparkling water maker so that it locks in place." \
  --scorer task_score --max-steps 2400
```

Nothing is randomised, so every seed gives the same scene. Runtime: the scripted demo reaches full credit after about
33.5 s of simulated time (670 steps; the trial ends there). With cameras, a shared GPU runs a few steps a second, so a 2,400-step trial can
take 10 to 20 minutes of wall time.

## The demo

```bash
python demokit.py task.py OUT                                          # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --fast --mode none,naive,early,hold   # the failure checks (CPU, no cameras)
```

The demo is a scripted reference, not a policy. It reads only what a camera could see: the bottle's pose, the machine's
pose and the holder's angle, plus its own arm's joints. It never reads the scorer.

1. The right hand comes at the bottle from the front, fingers 30 degrees below level and either side of the body, 13 cm
   up, grips it and lifts it 6 cm.
2. From then on the hand is steered closed loop on the bottle's seen neck tip and axis (each step the command moves by the
   path's step plus a share of the seen error, turning about the neck tip), which takes out the full bottle's droop in a
   soft hand. It leans the bottle 30 degrees, brings the neck under the holder's lead-in, into it, leans the bottle back
   to the holder's 15 degrees while pushing up the holder's axis until the collar is past the jaws, and lets the collar
   settle on them.
3. It swings the bottle's bottom back about the holder's hinge, aiming 6 degrees past vertical (the neck has a little
   play in the holder). As soon as the holder's angle reads upright it stops pushing, lets go and backs off.

The left arm is not used.

| run | what the script does | score |
|---|---|---|
| `full` | as above | 1.0 (CPU and GPU; see below) |
| `none` | does nothing | 0.0 |
| `naive` | lifts the bottle upright under the holder and pushes it straight up, then lets go | 0.0: the bottle meets the tilted cup and the machine tips past 5 degrees |
| `early` | puts the neck in and the collar on the jaws, then lets go before swinging it back | 0.25: the collar hangs in the jaws, the holder stays tilted |
| `hold` | the full route, but keeps hold of the bottle | 0.5: latched upright, but the hand is on the bottle |

Dream-1 run (2,400 steps): 0. It brought its right gripper in to the side of the bottle and stayed there without grasping or lifting it, while the left arm hovered on its own side of the table; the bottle never left its mark and the run ended at the step limit.

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold.

- **1.0** once all of these have held for 3 s running (60 control steps), and from then on unless a zeroing event below
  happens later:
  - the holder has latched: it reached within 0.6 degrees of upright with the collar in the jaws, and is then locked
    there, as the real Snap-Lock's catch holds it;
  - the collar is in the jaws: its underside at most 1.5 mm below the jaws' tops, its centre within 5 mm of the holder's
    axis, the bottle's axis within 8 degrees of the holder's, and both jaws closed (back within 1 mm);
  - the bottle is within 3 degrees of vertical, its bottom 2 to 15 mm above the base (the manual's "gap");
  - every fingertip point of both grippers (`mechkit.fingertips`: tip and 3 cm up each finger, middle and both edges) is
    at least 2 cm from the bottle and from the machine's outline.
- Otherwise, the last step's state: **0.5** if the collar is in the jaws with the holder within 5 degrees of upright,
  **0.25** if the collar is in the jaws with the holder tilted more, **0** otherwise.
- **0 for the whole trial** if at any time: the bottle tilts past 45 degrees (falls over), or falls faster than 1.4 m/s
  (dropped); or the machine slides more than 2 cm, tilts more than 5 degrees or rises more than 1.5 cm. The record is
  cleared only at a reset (`on_reset`).

For a real trial, use a SodaStream Terra with a CO2 cylinder fitted and its 1 L bottle, uncapped, filled with water to
the fill line; tape the machine's feet, the bottle's starting mark and the distractors' marks on the table. The click is
the Snap-Lock's own catch: the holder stays upright and the bottle hangs when the hand is gone (in simulation, the
holder reaching within 0.6 degrees of upright with the collar on the jaws). "Collar on the jaws" is the bottle hanging
from the holder; the sub-millimetre collar thresholds above are how the simulation detects it. A drop is the bottle
falling free about 10 cm or more (1.4 m/s).
In a real trial these are read from a side video with the bottle and holder against a protractor card, a steel rule
under the bottle's bottom (or a 2 mm and a 15 mm feeler), tape marks round the machine's feet, and the fingers' distance
from the bottle and machine in the same video.

## Model

- **Machine**: a free-standing articulation, 3.0 kg (an estimate with a CO2 cylinder fitted). Base 20 x 13 x 2.5 cm, a
  tower at the back, 44 cm tall, a head overhanging the bottle bay. Collision is boxes.
- **Snap-Lock holder**: a hinge about y, 30 cm up and 3.5 cm behind the bottle's axis. Joint zero is its rest, 15 degrees
  forward; its stop is at upright. A spring of 1.5 N m/rad, preloaded to 0.6 N m at rest, holds it forward. Its cup has a
  conical lead-in (6.8 cm across), a chamber above the jaws, and a bore round the neck 3.16 cm across with a 4 mm chamfer,
  closed 3 mm above the neck's tip; all are convex-hull colliders.
- **Jaws**: two sliders on the holder, 800 N/m each, 4 mm of travel, ramped below and flat on top, in POM.
- **Latch**: code (`Trial.step`): when the holder reaches within 0.6 degrees of upright with the collar in the jaws, its
  joint is locked (`mechkit.lock`). It is not released in this task.
- **Bottle**: one rigid body of 1.05 kg (PET bottle and 1 L of water as rigid mass, centre of mass 9.5 cm up), 8.5 cm
  across and 28 cm tall, neck 2.8 cm, collar 3.4 cm, colliding as convex hulls.

## Assets

The distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_glass_tumbler_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_tumbler_a01)
- [`consumable_paperTowel_roll_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_paperTowel_roll_a01)
- [`cleaning_dishSoap_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_dishSoap_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses
are set (glass 300 g, paper towel roll 200 g, dish soap 500 g), and the dish soap's jointed cap is welded into one rigid
body. SimReady has no sparkling water maker or carbonating bottle, so the machine and bottle are built from primitives and
hulls in `task.py`. Nothing large is committed: the files download at first run.

## Importance and sources

Fitting the bottle is the one manipulation every use of these machines starts with, and the manufacturer's own steps are
the angled-then-vertical motion modelled here:

- SodaStream TERRA user manual (US), "Snap-Lock system bottle insert": "make sure that the Plastic Carbonating Bottle
  rest is angled forward", "Insert bottle neck into the snap-lock. Push it up and then back."
  https://cdn.shopify.com/s/files/1/2798/7364/files/US_User_Manual_-_TERRA.pdf
