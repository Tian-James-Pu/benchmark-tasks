# Take a spirit level out from under a loose riser shelf

A task file for the `yam_arena` embodiment (yam-isaac-lab-arena, Isaac Lab-Arena 0.3.1) on the station scene:
[task.py](task.py).

A spirit level lies on the table with 9 cm of its length under a low shelf: a loose board on two loose blocks,
with a drinking glass on it and 1.5 cm of air above the level. Lifted where it lies, the level catches under
the board's edge and levers the board and the glass up. It has to be kept low and drawn clear first.

## Run a trial

Dream-1:

```bash
source ~/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/level-from-under-loose-shelf/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the spirit level out from under the shelf and lay it on the green mat." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/level-from-under-loose-shelf/dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. Both arms grip the
exposed part of the level, raise it 6 mm, carry it 13 cm to the right until it is clear of the board, then lift
it and set it on the mat. The demo plays a fixed plan from the published layout.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the board was ever tilted past
5 degrees or the glass ever tipped past 30 degrees or left the board. Otherwise it needs, at the end: the level
flat, at rest and lying on the table or the mat (its underside within 3 mm of it) with no part of it under the
board; the board flat and lying on its blocks (centre within 3 mm of its starting height); the glass upright and
standing on the board (base within 4 mm of its top); both within 3 cm of where they started; and no point on
either gripper's fingers within 2 cm of the level, the board or the glass. With all of that it returns 1.0 when
the level is wholly on the mat and the board and glass are within 1 cm of their starting places, else 0.5;
anything else is 0. `success()` is a score of 1.0, and the scorer requires it to hold for 3 s.

The harm record is module-level state kept for the whole trial. It is cleared only in `mechkit`'s `on_reset`
hook, which runs when the scene is reset, never because objects return to where they started. The fingertip
positions come from `mechkit`'s `fingertips()`, read after every control step.

## Runtime and seeds

- Demo: about 340 steps (17 s simulated), about 1 minute on one GPU once shaders are compiled.
- Dream-1: 2,400 steps (2 minutes simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and
  deterministic.

## Other strategies tried

All run with the current task file (18 November), with the same command and a different `demo()`:

| Variant | Score |
|---|---|
| Scripted demo, two fresh runs | 1.0, 1.0 |
| Do nothing (both arms hover) | 0 |
| Both arms grip the level as in the demo and lift it straight up, then carry it to the mat (the board is levered up well past 5 degrees) | 0 |
| Right arm alone grips the level 12 cm from the board's edge, lifts it straight up and carries it to the mat (the board's edge is levered up to 5.1 degrees and drops back; the glass stays standing) | 0 |
| The demo, but the grippers keep hold of the level once it is on the mat | 0 |

Two arms are not physically required: a one-armed draw that keeps the level low until it is clear also earns 1.0.

## Importance and sources

Loose riser shelves are sold to add storage inside cupboards and on benches. IKEA's VARIERA shelf
insert, for example, is a steel shelf that stands loose on an existing shelf "to get more storage space for
glasses, bowls and spice jars", with room underneath
([IKEA, VARIERA shelf insert 32x13x16 cm](https://www.ikea.com/ch/en/p/variera-shelf-insert-white-80136622/)).
Glasses on top and things slid underneath is the situation this task models. The physical difficulty is
real: a long, flat item lifted where it lies catches under the riser's edge and levers the riser up. It has to
be drawn out low before it is raised.

Limits: the source shows that such risers exist and what they are sold for. It is not evidence of how often a
long item gets pushed under one, and we found no statistic for that. The riser here is a plain board on two
loose blocks, 4 cm high, much lower than the 16 cm insert above, and a store-bought insert is one rigid piece,
so it would tip as a whole rather than lift off its supports. A tipped glass is the scored harm; the simulated
glass cannot break.

## Changes since the first build

The scorer used to clear its harm record when the objects jumped back to the starting layout in one step. It
now clears it only in the `on_reset` hook. "Lying on" checks are now within 3 to 4 mm of the supporting
surface, and full or partial credit requires both grippers to be clear of the level, board and glass. The task
is now in the Societal relatability category, and the README has a sources section.

## Assets

The drinking glass and the two distractors are props from NVIDIA's SimReady library
([`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) on Hugging Face), pinned to
revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_glass_8oz_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_8oz_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
- masses are set by hand: glass 0.18 kg, tape 0.06 kg, screwdriver 0.10 kg;
- the glass and the screwdriver collide as convex hulls;
- the invisible grasp-annotation curves are deactivated.

The blocks, board, level and mat are `yam_arena` boxes. `mechkit.py` is copied unchanged from
`tasks/heat-shrink-lug/mechkit.py` in this repository (another team's open-source mechanism kit); this task uses
only its reset and step hooks and its `fingertips()` reader. The prop files are not committed: `simready.py`
(adapted from `tasks/cutters-off-pegboard/simready.py` in this repository) downloads them into
`~/.cache/simready` the first time the scene is built, which needs network access.
